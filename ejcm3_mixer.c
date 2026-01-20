#include <linux/module.h>
#include <linux/kthread.h>
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

        struct task_struct *mixer_task;
        atomic_t streaming;
};

/**
 * Moudule parameters
 */
static unsigned int param_out_width  = 1280;
static unsigned int param_out_height = 1440;
module_param(param_out_width,  uint, 0444);
module_param(param_out_height, uint, 0444);
MODULE_PARM_DESC(param_out_width,  "Combined output width  (default 1280)");
MODULE_PARM_DESC(param_out_height, "Combined output height (default 1440)");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Syu-Song Chiang");
MODULE_DESCRIPTION("V4L2 Proxy-Mixer: combine two UVC sources in kernel space");

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
        struct proxy_mixer   *mixer = container_of(ci, struct proxy_mixer, class_intf);
        struct video_device  *vdev  = to_video_device(dev);
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
        /*
        * video_class 在 drivers/media/v4l2-core/videodev.c 定義。
        * 若未 export，在此宣告 extern：
        *   extern struct class video_class;
        * 或使用 kallsyms_lookup_name("video_class")。
        */
        extern struct class video_class;

        mixer->class_intf.class      = &video_class;
        mixer->class_intf.add_dev    = mixer_class_add_dev;
        mixer->class_intf.remove_dev = mixer_class_remove_dev;

        /*
        * class_interface_register 會對 video4linux class 中
        * 所有已存在的 device 立即觸發 add_dev，
        * 解決 module 晚於 uvcvideo 載入的競態。
        */
        return class_interface_register(&mixer->class_intf);
}

void mixer_unregister_class_intf(struct proxy_mixer *mixer)
{
    class_interface_unregister(&mixer->class_intf);
}

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

/**/
static int mixer_add() {
        struct proxy_mixer *mixer;
        int ret;

        mixer = kzalloc(sizeof(*mixer), GFP_KERNEL);
        if (!mixer)
                return -ENOMEM;

        mutex_init(&mixer->lock);
        mutex_init(&mixer->src_disc_lock);
        atomic_set(&mixer->streaming, 0);

        mixer->out_width  = param_out_width;
        mixer->out_height = param_out_height;
        mixer->src_width  = param_out_width;
        mixer->src_height = param_out_height / 2;
        
        ret = v4l2_device_register(NULL, &mixer->v4l2_dev);
        if (ret) {
                goto err_free;
        }

        init_queue(&mixer->out_q);

        mixer->vdev = video_device_alloc();
        if (mixer->vdev == NULL) {
                goto err_unregister;
        }

        init_vdev(mixer->vdev);
        mixer->vdev->queue = &mixer->out_q.vbq;
        mixer->vdev->v4l2_dev = &mixer->v4l2_dev;
        video_set_drvdata(mixer->vdev, mixer);

        ret = video_register_device(mixer->vdev, VFL_TYPE_VIDEO, -1);
        if (ret) {
                goto err_vdev_release;
        }

err_vdev_release:
        if (mixer->vdev) {
                video_set_drvdata(mixer->vdev, NULL);
                video_unregister_device(mixer->vdev);
        }
err_unregister:
        mixer_queue_release(&mixer->out_q);
        v4l2_device_unregister(&mixer->v4l2_dev);
err_free:
        kfree(mixer);
        return ret;
}

/**/
static int __init proxy_mixer_init(void) {

}