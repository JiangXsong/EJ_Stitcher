#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kallsyms.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/usb.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define MIXER_NUM_SOURCES        2
//#define MIXER_SRC_NUM_BUFS       4       /* 每路 UVC source 的 buffer 數 */
#define MIXER_OUT_NUM_BUFS       4       /* output queue buffer 數 */

#define MIXER_DEFAULT_VID        0x1E4E
#define MIXER_DEFAULT_PID_P      0x7301
#define MIXER_DEFAULT_PID_S      0x7302

#define MIXER_CALL_OP(src, op, ...) \
({ \
        int __ret = -ENOTTY; \
        struct video_device *__vd = (src)->vdev; \
        if (__vd && __vd->ioctl_ops && __vd->ioctl_ops->op) { \
                if (__vd->lock) mutex_lock(__vd->lock); \
                __ret = __vd->ioctl_ops->op(__VA_ARGS__); \
                if (__vd->lock) mutex_unlock(__vd->lock); \
        } \
        __ret; \
})

/* ------------------------------------------------------------------ */
struct mixer_slot {
        struct file *filp;
        struct video_device *vdev;
        struct v4l2_fh *fh;
        struct vb2_queue *vbq;
        struct v4l2_format fmt;

        /* Current buffer, dequeued from vb2 queue */
        struct vb2_buffer *cur_vb;
        ktime_t cur_ts;
        bool buf_ready;

        /* Related USB device*/
        struct usb_device *udev;
};

struct mixer_disc_work {
        struct work_struct work;
        struct proxy_mixer *mixer;
        struct video_device *vdev;
        bool is_add;
};

enum buffer_state {
        BUF_STATE_IDLE = 0,
        BUF_STATE_QUEUED = 1,
        BUF_STATE_ACTIVE = 2,
        BUF_STATE_READY = 3,
        BUF_STATE_DONE = 4,
        BUF_STATE_ERROR = 5,
};

struct mixer_buffer {
        struct vb2_v4l2_buffer buf;
        struct list_head queue;

        enum buffer_state state;
};

struct mixer_video_queue {
        struct vb2_queue vbq;
        struct mutex mutex; /* Protects queue */

        spinlock_t irqlock; /* Protrcts irqqueue */
        struct list_head irqqueue;
        wait_queue_head_t       buf_wq;
};

struct proxy_mixer {
        struct v4l2_device v4l2_dev;
        struct video_device *vdev;
        struct mutex lock;

        struct class_interface class_intf;
        int src_ready_count; /* Protected by src_disc_lock */
        struct mutex src_disc_lock;

        struct mixer_slot src[2];

        /* Output format */
        u32 out_width;
        u32 out_height;
        /* per-source format (src_height = out_height / 2) */
        u32 src_width;
        u32 src_height;

        struct mixer_video_queue out_q;


        struct mutex uvc_ctrl_lock; /* Protects UVC control operations */

        struct task_struct *mixer_task;
        atomic_t streaming;
};

/* ------------------------------------------------------------------ */
/**
 * Moudule parameters
 */
static unsigned int param_out_width  = 1280;
static unsigned int param_out_height = 1440;
module_param(param_out_width,  uint, 0444);
module_param(param_out_height, uint, 0444);
MODULE_PARM_DESC(param_out_width,  "Combined output width  (default 1280)");
MODULE_PARM_DESC(param_out_height, "Combined output height (default 1440)");

static struct proxy_mixer *g_mixer;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Syu-Song Chiang");
MODULE_DESCRIPTION("V4L2 Proxy-Mixer: combine two UVC sources in kernel space");

/* ------------------------------------------------------------------ */
/**
 * Workqueue function for handling device discovery/removal asynchronously,
 * to avoid blocking the caller (e.g., uvcvideo) and to ensure proper locking.
 */
static void mixer_disc_work_fn(struct work_struct *w)
{
        struct mixer_disc_work *dw = container_of(w, struct mixer_disc_work, work);
        struct proxy_mixer *mixer = dw->mixer;
        struct video_device *vdev  = dw->vdev;
        int slot = -1;
        int i;

        if (dw->is_add) {
                /* ADD */
                mutex_lock(&mixer->src_disc_lock);

                /* Ignore if all slots are filled */
                if (mixer->src_ready_count >= MIXER_NUM_SOURCES) {
                        mutex_unlock(&mixer->src_disc_lock);
                        goto out;
                }

                /* find empty slot */
                for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                        if (!mixer->src[i].filp) {
                                slot = i;
                                break;
                        }
                }
                mutex_unlock(&mixer->src_disc_lock);

                if (slot < 0)
                goto out;

                /* filp_open */
                char devpath[32];
                snprintf(devpath, sizeof(devpath), "/dev/video%d", vdev->num);

                struct file *filp = filp_open(devpath, O_RDWR | O_NONBLOCK, 0);
                if (IS_ERR(filp)) {
                        pr_err("proxy_mixer: filp_open(%s): %ld\n",
                                devpath, PTR_ERR(filp));
                        goto out;
                }

                mutex_lock(&mixer->src_disc_lock);
                mixer->src[slot].filp = filp;
                mixer->src[slot].vdev = video_devdata(filp);
                mixer->src[slot].fh = filp->private_data;
                mixer->src[slot].vbq = mixer->src[slot].vdev->queue;
                mixer->src[slot].udev = interface_to_usbdev(to_usb_interface(vdev->v4l2_dev->dev));
                mixer->src_ready_count++;
                mutex_unlock(&mixer->src_disc_lock);

                pr_info("proxy_mixer: slot%d filled (%s), ready=%d/%d\n",
                        slot, devpath,
                        mixer->src_ready_count, MIXER_NUM_SOURCES);

                /*
                * 不在此處啟動 pipeline。
                * pipeline 啟動由 VIDIOC_STREAMON 觸發。
                */
        } else {
                /* REMOVE */

                /* If streaming, stop */
                if (atomic_read(&mixer->streaming))
                        mixer_uvc_stop(mixer);

                mutex_lock(&mixer->src_disc_lock);
                for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                        if (mixer->src[i].vdev == vdev) {
                                filp_close(mixer->src[i].filp, NULL);
                                memset(&mixer->src[i], 0, sizeof(mixer->src[i]));
                                mixer->src_ready_count--;
                                pr_info("proxy_mixer: slot%d removed, ready=%d/%d\n",
                                        i, mixer->src_ready_count, MIXER_NUM_SOURCES);
                                break;
                        }
                }
                mutex_unlock(&mixer->src_disc_lock);
        }

        out:
        kfree(dw);
}

/**
 * Discover UVC devices matching the specified VID/PID,
 * and add them as sources to the mixer.
 */
static bool mixer_match_device(struct video_device *vdev)
{
        struct device *intf_dev;
        struct usb_interface *intf;
        struct usb_device *udev;

        if (!(vdev->device_caps & V4L2_CAP_VIDEO_CAPTURE))
                return false;

        if (!vdev->v4l2_dev || !vdev->v4l2_dev->dev)
                return false;

        intf_dev = vdev->v4l2_dev->dev;

        if (!intf_dev->bus || strcmp(intf_dev->bus->name, "usb") != 0)
                return false;
        if (!intf_dev->driver || strcmp(intf_dev->driver->name, "uvcvideo") != 0)
                return false;

        /* USB VID/PID */
        intf  = to_usb_interface(intf_dev);
        udev  = interface_to_usbdev(intf);

        if (le16_to_cpu(udev->descriptor.idVendor)  != MIXER_DEFAULT_VID &&
                (le16_to_cpu(udev->descriptor.idProduct) != MIXER_DEFAULT_PID_P ||
                le16_to_cpu(udev->descriptor.idProduct) != MIXER_DEFAULT_PID_S))
                return false;

        return true;
}

static int mixer_class_add_dev(struct device *dev,
                                struct class_interface *ci)
{
        struct proxy_mixer *mixer = container_of(ci, struct proxy_mixer, class_intf);
        struct video_device *vdev = to_video_device(dev);
        struct mixer_disc_work *dw;

        if (!vdev)
                return 0;

        if (!mixer_match_device(vdev))
                return 0;

        dw = kzalloc(sizeof(*dw), GFP_ATOMIC);
        if (!dw)
                return -ENOMEM;

        INIT_WORK(&dw->work, mixer_disc_work_fn);
        dw->mixer  = mixer;
        dw->vdev   = vdev;
        dw->is_add = true;

        schedule_work(&dw->work);
        return 0;
}

static void mixer_class_remove_dev(struct device *dev,
                                    struct class_interface *ci)
{
        struct proxy_mixer *mixer = container_of(ci, struct proxy_mixer, class_intf);
        struct video_device *vdev = to_video_device(dev);
        struct mixer_disc_work *dw;
        bool is_ours = false;
        int i;

        if (!vdev)
                return;

        mutex_lock(&mixer->src_disc_lock);
        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                if (mixer->src[i].vdev == vdev) {
                        is_ours = true;
                        break;
                }
        }
        mutex_unlock(&mixer->src_disc_lock);

        if (!is_ours)
                return;

        dw = kzalloc(sizeof(*dw), GFP_ATOMIC);
        if (!dw)
                return;

        INIT_WORK(&dw->work, mixer_disc_work_fn);
        dw->mixer  = mixer;
        dw->vdev   = vdev;
        dw->is_add = false;

        schedule_work(&dw->work);
}

int mixer_register_class_intf(struct proxy_mixer *mixer)
{
        /**
         * In drivers/media/v4l2-core/v4l2-dev.c
         * defined struc class video_class as static.
         * Here clould'n extern it directly, 
         * but we can extern the symbol video_class via kallsyms_lookup_name("video_class").
         * --
         * If couldn't find video_class, perhaps try to add patch in v4l2-dev.c to export it:
         * EXPORT_SYMBOL(video_class);
         * --
         */
        struct class *vcls;

        vcls = (struct class *)kallsyms_lookup_name("video_class");
        if (!vcls) {
                pr_err("proxy_mixer: cannot find video_class\n");
                return -ENODEV;
        }

        mixer->class_intf.class      = vcls;
        mixer->class_intf.add_dev    = mixer_class_add_dev;
        mixer->class_intf.remove_dev = mixer_class_remove_dev;

        /**
         * class_interface_register - Registers a class 
         * 
         * The add_dev callback will be called for each existing device in the class, 
         * and then for each new device that gets added to the class.
         */
        return class_interface_register(&mixer->class_intf);
}

void mixer_unregister_class_intf(struct proxy_mixer *mixer)
{
    class_interface_unregister(&mixer->class_intf);
}
/* ------------------------------------------------------------------ */
/**
 * UVC streaming control
 */
static int mixer_uvc_negotiate_src_fmt(struct proxy_mixer *mixer, int slot) {
        struct mixer_slot *src = &mixer->src[slot];
        int ret;

        src->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        src->fmt.fmt.pix.width = mixer->src_width;
        src->fmt.fmt.pix.height = mixer->src_height;
        src->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        src->fmt.fmt.pix.field = V4L2_FIELD_NONE;

        ret = MIXER_CALL_OP(src, vidioc_s_fmt_vid_cap, src->filp, src->fh, &src->fmt);
        if (ret) {
                pr_err("proxy_mixer: vidioc_s_fmt_vid_cap failed for slot%d: %d\n", slot, ret);
                return ret;
        }

        ret = MIXER_CALL_OP(src, vidioc_g_fmt_vid_cap, src->filp, src->fh, &src->fmt);
        if (ret) {
                pr_err("proxy_mixer: vidioc_g_fmt_vid_cap failed for slot%d: %d\n", slot, ret);
                return ret;
        }

        pr_info("proxy_mixer: src%d format negotiated: %ux%u, fmt=0x%X\n",
                slot, src->fmt.fmt.pix.width, src->fmt.fmt.pix.height, src->fmt.fmt.pix.pixelformat);
        
        return 0;
}

static int mixer_uvc_reqbufs(struct proxy_mixer *mixer, int idx) {
        struct mixer_slot *src = &mixer->src[idx];
        struct v4l2_requestbuffers reqbufs = {
                .count = MIXER_OUT_NUM_BUFS,
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
        };
        unsigned int i;
        int ret;

        ret = MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &reqbufs);
        if (ret) {
                pr_err("proxy_mixer: vidioc_reqbufs failed for slot%d: %d\n", idx, ret);
                return ret;
        }
        if (reqbufs.count < MIXER_OUT_NUM_BUFS) {
                pr_err("proxy_mixer: slot%d requested buffer count %d less than required %d\n",
                        idx, reqbufs.count, MIXER_OUT_NUM_BUFS);
                return -ENOMEM;
        }

        for (i = 0; i < reqbufs.count; i++) {
                struct v4l2_buffer buf = {
                        .index  = i,
                        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                        .memory = V4L2_MEMORY_MMAP,
                };
                ret = MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
                if (ret) {
                        pr_err("proxy_mixer: src%d QBUF[%u] failed: %d\n",
                                idx, i, ret);
                        return ret;
                }
        }

        return 0;
}

static int mixer_uvc_streamon(struct proxy_mixer *mixer, int idx)
{
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        int ret = MIXER_CALL_OP(&mixer->src[idx], vidioc_streamon,
                                mixer->src[idx].filp,
                                mixer->src[idx].fh, type);
        if (ret)
                pr_err("proxy_mixer: src%d STREAMON failed: %d\n", idx, ret);
        return ret;
}

static int mixer_uvc_streamoff(struct proxy_mixer *mixer, int idx) {
        struct mixer_slot *src = &mixer->src[idx];
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        struct v4l2_requestbuffers reqbufs = {
                .count = 0,
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
        };
        int ret;

        if (IS_ERR_OR_NULL(src->filp) || !src->vdev)
                return -ENODEV;

        ret = MIXER_CALL_OP(src, vidioc_streamoff, src->filp, src->fh, type);
        if (ret)
                return ret;

        ret = MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &reqbufs);
        if (ret)
                return ret;
        
        return ret;
}

static int mixer_uvc_start(struct proxy_mixer *mixer) {
        int i, ret;

        /* Check all source slots*/
        mutex_lock(&mixer->src_disc_lock);
        if (mixer->src_ready_count < MIXER_NUM_SOURCES) {
                mutex_unlock(&mixer->src_disc_lock);

                return -ENODEV;
        }
        mutex_unlock(&mixer->src_disc_lock);

        /* Negotiate format and request buffers for each source */
        mutex_lock(&mixer->uvc_ctrl_lock);
        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                ret = mixer_uvc_negotiate_src_fmt(mixer, i);
                if (ret)
                        goto err_out;

                ret = mixer_uvc_reqbufs(mixer, i);
                if (ret)
                        goto err_out;
        }

        /* Stream on each source */
        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                ret = mixer_uvc_streamon(mixer, i);
                if (ret) {
                        /* Stream off any previously started sources */
                        goto err_streamoff;
                }
        }

        /* Start the mixer thread */
        atomic_set(&mixer->streaming, 1);
        mixer->mixer_task = kthread_run(mixer_thread_fn, mixer, "proxy_mixer");
        if (IS_ERR(mixer->mixer_task)) {
                ret = PTR_ERR(mixer->mixer_task);
                mixer->mixer_task = NULL;
                atomic_set(&mixer->streaming, 0);
                goto err_streamoff;
        }

        mutex_unlock(&mixer->uvc_ctrl_lock);
        return 0;
err_streamoff:
        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                mixer_uvc_streamoff(mixer, i);
        }
err_out:
        mutex_unlock(&mixer->uvc_ctrl_lock);
        return ret;       
}

static void mixer_uvc_stop(struct proxy_mixer *mixer) {
        int i;

        if (!atomic_cmpxchg(&mixer->streaming, 1, 0)) {
                /* Not streaming */
                return;
        }

        wake_up_all(&mixer->out_q.buf_wq);
        if (mixer->mixer_task) {
                kthread_stop(mixer->mixer_task);
                mixer->mixer_task = NULL;
        }

        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                struct mixer_slot *src = &mixer->src[i];
                if (src->buf_ready) {
                        /* Return the active buffer to the source queue */
                        struct v4l2_buffer buf = {
                                .index  = src->cur_vb->index,
                                .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                .memory = V4L2_MEMORY_MMAP,
                        };
                        MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
                        src->cur_vb = NULL;
                        src->buf_ready = false;
                }
        }

        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
                mixer_uvc_streamoff(mixer, i);
        }
}

const struct v4l2_ioctl_ops mixer_ioctl_ops = {
        // clang-format off
        .vidioc_querycap         = mixer_vidioc_querycap,
        .vidioc_enum_fmt_vid_cap = mixer_vidioc_enum_fmt,
        .vidioc_g_fmt_vid_cap    = mixer_vidioc_g_fmt,
        .vidioc_s_fmt_vid_cap    = mixer_vidioc_s_fmt,
        .vidioc_try_fmt_vid_cap  = mixer_vidioc_g_fmt,

        /* buffer 管理：vb2 helper（僅 reqbufs/querybuf/qbuf/dqbuf/expbuf）*/
        .vidioc_reqbufs          = vb2_ioctl_reqbufs,
        .vidioc_querybuf         = vb2_ioctl_querybuf,
        .vidioc_qbuf             = vb2_ioctl_qbuf,
        .vidioc_dqbuf            = vb2_ioctl_dqbuf,
        .vidioc_expbuf           = vb2_ioctl_expbuf,

        /* stream 控制：自訂包裝 */
        .vidioc_streamon         = mixer_vidioc_streamon,
        .vidioc_streamoff        = mixer_vidioc_streamoff,
        // clang-format on
};

/* ------------------------------------------------------------------ */
/**
 * 
 */
static void mixer_queue_release(struct mixer_video_queue *q) {
        mutex_lock(&q->mutex);
        vb2_queue_release(&q->vbq);
        mutex_unlock(&q->mutex);
}

static int init_queue(struct mixer_video_queue *q) {
        int ret;

        mutex_init(&q->mutex);
        spin_lock_init(&q->irqlock);
        init_waitqueue_head(&q->buf_wq);
        INIT_LIST_HEAD(&q->irqqueue);

        q->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        q->vbq.io_modes = VB2_MMAP | VB2_DMABUF;
        q->vbq.drv_priv = q;
        q->vbq.ops = &mixer_vb2_ops;
        q->vbq.mem_ops = &vb2_vmalloc_memops;
        q->vbq.buf_struct_size = sizeof(struct mixer_buffer);
        q->vbq.lock = &q->mutex;
        q->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

        ret = vb2_queue_init(&q->vbq);
        if (ret) {
                return ret;
        }

        return 0;
}

/* ------------------------------------------------------------------ */
/**/
static void init_vdev(struct video_device *vdev) {
        vdev->fops = &mixer_fops;
        vdev->ioctl_ops = &mixer_ioctl_ops;
        vdev->release = mixer_vdev_release;
        vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
        strscpy(vdev->name, "Dul Video Mixer", sizeof(vdev->name));
        vdev->vfl_type = VFL_TYPE_VIDEO;
        vdev->vfl_dir = VFL_DIR_RX;
}

static void mixer_unregister(struct proxy_mixer *mixer) {
        if (!mixer)
                return;

        if (mixer->vdev) {
                video_set_drvdata(mixer->vdev, NULL);
                video_unregister_device(mixer->vdev);
        }

        if (mixer->out_q.vbq.buf_struct_size) {
                mixer_queue_release(&mixer->out_q);
        }
        
        if (mixer->v4l2_dev.dev) {
                v4l2_device_unregister(&mixer->v4l2_dev);
        }
        kfree(mixer);
}

/**/
static int mixer_add(void) {
        struct proxy_mixer *mixer;
        int ret;

        mixer = kzalloc(sizeof(*mixer), GFP_KERNEL);
        if (!mixer)
                return -ENOMEM;

        mutex_init(&mixer->lock);
        mutex_init(&mixer->src_disc_lock);
        mutex_init(&mixer->uvc_ctrl_lock);
        atomic_set(&mixer->streaming, 0);

        mixer->out_width  = param_out_width;
        mixer->out_height = param_out_height;
        mixer->src_width  = param_out_width;
        mixer->src_height = param_out_height / 2;
        
        ret = v4l2_device_register(NULL, &mixer->v4l2_dev);
        if (ret) {
                goto err_out;
        }

        init_queue(&mixer->out_q);

        mixer->vdev = video_device_alloc();
        if (mixer->vdev == NULL) {
                goto err_out;
        }

        init_vdev(mixer->vdev);
        mixer->vdev->queue = &mixer->out_q.vbq;
        mixer->vdev->v4l2_dev = &mixer->v4l2_dev;
        video_set_drvdata(mixer->vdev, mixer);

        ret = video_register_device(mixer->vdev, VFL_TYPE_VIDEO, -1);
        if (ret) {
                goto err_out;
        }

        ret = mixer_register_class_intf(mixer);
        if (ret) {
                goto err_out;
        }

        g_mixer = mixer;

        return 0;

err_out:
        mixer_unregister(mixer);
        return ret;
}

static void mixer_remove(struct proxy_mixer *mixer) {
        int i;

        if (!mixer)
                return;

        /* unregister the class interface */
        mixer_unregister_class_intf(mixer);

        /* If streaming, stop */

        /* close all source filp */

        /* unregister video node */
        mixer_unregister(mixer);
        g_mixer = NULL;
}

/**/
static int __init proxy_mixer_init(void) {

}