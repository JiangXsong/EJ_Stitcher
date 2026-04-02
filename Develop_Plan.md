# Dul_Video_Input_Kernel_Driver# Kernel V4L2 Proxy-Mixer — 開發方案 
**目標環境：Linux 6.14 · 不修改 uvcvideo · 純 kernel-space**

---

## 架構說明

### 整體控制流程

```
modprobe mixer target_vid=0x1234 target_pid=0x5678
  │
  ├─ v4l2_device_register()
  ├─ video_register_device()      → /dev/video2 立即存在
  └─ class_interface_register()   → 開始監聽 UVC 裝置

UVC 裝置插入（VID/PID 符合）
  └─ add_dev workqueue
       └─ filp_open() + video_devdata()
            └─ 填入 src[0] / src[1] slot（僅此，不啟動 pipeline）

ffmpeg / OBS:
  open("/dev/video2")    → 成功
  VIDIOC_S_FMT           → 設定輸出格式（out_width/height，不需 UVC 就緒）
  VIDIOC_REQBUFS         → 分配輸出 buffer（使用已設定的 out_width/height）
  VIDIOC_QBUF × N        → 將 output buffer 放入 queue
  VIDIOC_STREAMON
    ├─ 檢查 src[0], src[1] 是否就緒  → -ENODEV（若未就緒，終止）
    ├─ UVC Format 協商（S_FMT / G_FMT）
    ├─ UVC REQBUFS + 初始 QBUF
    ├─ UVC STREAMON × 2
    └─ kthread_run(mixer_kthread_fn)
```

### 資料流（kthread 主迴圈）

```
buf_list（userspace 已 QBUF 的 output buffer）
  │
  ▼
取出一個 output vb2 buffer（dst）← 等待，若 list 空則 sleep
  │
  ├─ DQBUF from UVC src[0]  → src0_vb（含 frame data）
  ├─ DQBUF from UVC src[1]  → src1_vb（含 frame data）
  │
  ├─ Frame 同步（timestamp 比對）
  │
  ├─ vb2_plane_vaddr(dst)    → dst ptr
  ├─ vb2_plane_vaddr(src0)   → src0 ptr
  ├─ vb2_plane_vaddr(src1)   → src1 ptr
  │
  ├─ mixer_combine_yuyv(dst, src0, src1)   ← 唯一一次 memcpy
  │
  ├─ vb2_buffer_done(dst, DONE)  → 通知 ffmpeg/OBS 可 DQBUF
  │
  ├─ QBUF src0_vb → UVC src[0] queue
  └─ QBUF src1_vb → UVC src[1] queue
```

---

## 檔案結構

```
proxy_mixer/
├── proxy_mixer.h           ← 共用資料結構與函式宣告
├── proxy_mixer_main.c      ← module_init / module_exit / video node 註冊
├── proxy_mixer_disc.c      ← class_interface 裝置發現
├── proxy_mixer_ctrl.c      ← vidioc_streamon/off / UVC 控制路徑
├── proxy_mixer_data.c      ← kthread / frame sync / combine
├── proxy_mixer_output.c    ← vb2_ops / ioctl_ops
└── Makefile
```

---

## proxy_mixer.h — 共用資料結構

```c
#ifndef _PROXY_MIXER_H
#define _PROXY_MIXER_H

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

/* ------------------------------------------------------------------ */
/* 常數 */

#define MIXER_NUM_SOURCES         2
#define MIXER_SRC_NUM_BUFS        4
#define MIXER_OUT_NUM_BUFS        4
#define MIXER_SYNC_TOLERANCE_US   16667ULL   /* 半幀 @ 30fps */
#define MIXER_SYNC_MAX_DROP       8

/* ------------------------------------------------------------------ */
/* 每路 UVC source 的狀態 */

struct mixer_src {
    /* filp_open 後有效 */
    struct file          *filp;
    struct video_device  *vdev;
    struct v4l2_fh       *fh;
    struct vb2_queue     *vbq;

    /* UVC format 協商後有效 */
    struct v4l2_format    fmt;

    /* kthread 使用：目前持有的 dequeued buffer（DQ 後到 QBuf 前有效）*/
    struct vb2_buffer    *cur_vb;
    ktime_t               cur_ts;
    bool                  buf_ready;

    /* hotplug remove 比對用 */
    struct usb_device    *udev;
};

/* output vb2 buffer wrapper */
struct mixer_buf {
    struct vb2_v4l2_buffer  vbuf;   /* 必須是第一個欄位 */
    struct list_head         list;
};

/* ------------------------------------------------------------------ */
/* 主要裝置結構 */

struct proxy_mixer {
    /* V4L2 framework */
    struct v4l2_device      v4l2_dev;
    struct video_device     vdev_out;

    /* 裝置發現 */
    struct class_interface  class_intf;
    struct mutex            src_disc_lock;  /* 保護 src[] 和 src_ready_count */
    int                     src_ready_count;

    /* UVC sources */
    struct mixer_src        src[MIXER_NUM_SOURCES];

    /* 輸出格式（S_FMT 時設定，不依賴 UVC 就緒）*/
    u32                     out_width;
    u32                     out_height;
    /* per-source 的寬度（= out_width / NUM_SOURCES for SBS）*/
    u32                     src_width;
    u32                     src_height;

    /* output vb2 queue */
    struct vb2_queue        out_queue;
    struct mutex            out_lock;
    spinlock_t              buf_lock;
    struct list_head        buf_list;
    wait_queue_head_t       buf_wq;     /* kthread 等待 output buffer 用 */

    /* kthread */
    struct task_struct     *mixer_task;
    atomic_t                streaming;

    /* UVC 控制 lock */
    struct mutex            ctrl_lock;
};

/* ------------------------------------------------------------------ */
/* 跨檔案函式宣告 */

/* proxy_mixer_disc.c */
int  mixer_register_class_intf(struct proxy_mixer *mixer);
void mixer_unregister_class_intf(struct proxy_mixer *mixer);

/* proxy_mixer_ctrl.c */
int  mixer_uvc_negotiate_and_start(struct proxy_mixer *mixer);
void mixer_uvc_stop(struct proxy_mixer *mixer);

/* proxy_mixer_data.c */
int  mixer_kthread_fn(void *data);

/* proxy_mixer_output.c */
int  mixer_output_register(struct proxy_mixer *mixer);
void mixer_output_unregister(struct proxy_mixer *mixer);

extern const struct v4l2_ioctl_ops   mixer_ioctl_ops;
extern const struct v4l2_file_operations mixer_fops;
extern const struct vb2_ops          mixer_out_vb2_ops;

/* ------------------------------------------------------------------ */
/* MIXER_CALL_OP：直接呼叫 UVC driver 的 ioctl_ops，繞過 copy_from_user */

#define MIXER_CALL_OP(src, op, ...)                               \
({                                                                 \
    int __ret = -ENOTTY;                                          \
    struct video_device *__vd = (src)->vdev;                     \
    if (__vd && __vd->ioctl_ops && __vd->ioctl_ops->op) {        \
        if (__vd->lock) mutex_lock(__vd->lock);                  \
        __ret = __vd->ioctl_ops->op(__VA_ARGS__);                \
        if (__vd->lock) mutex_unlock(__vd->lock);                \
    }                                                             \
    __ret;                                                        \
})

#endif /* _PROXY_MIXER_H */
```

---

## proxy_mixer_main.c — module_init / module_exit

```c
#include "proxy_mixer.h"

/* ------------------------------------------------------------------ */
/* Module 參數 */

static unsigned short target_vid;
static unsigned short target_pid;
module_param(target_vid, ushort, 0444);
module_param(target_pid, ushort, 0444);
MODULE_PARM_DESC(target_vid, "USB Vendor ID  e.g. 0x1234");
MODULE_PARM_DESC(target_pid, "USB Product ID e.g. 0x5678");

/*
 * 輸出格式預設值：side-by-side，兩路各 1280x720
 * 可透過 module_param 覆蓋
 */
static unsigned int param_out_width  = 2560;
static unsigned int param_out_height = 720;
module_param(param_out_width,  uint, 0444);
module_param(param_out_height, uint, 0444);
MODULE_PARM_DESC(param_out_width,  "Combined output width  (default 2560)");
MODULE_PARM_DESC(param_out_height, "Combined output height (default 720)");

static struct proxy_mixer *g_mixer;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("V4L2 Proxy-Mixer: combine two UVC sources in kernel space");

/* ------------------------------------------------------------------ */

static int __init proxy_mixer_init(void)
{
    struct proxy_mixer *mixer;
    int ret;

    if (!target_vid && !target_pid) {
        pr_err("proxy_mixer: target_vid and target_pid required\n");
        pr_err("proxy_mixer: insmod proxy_mixer.ko "
               "target_vid=0x1234 target_pid=0x5678\n");
        return -EINVAL;
    }

    mixer = kzalloc(sizeof(*mixer), GFP_KERNEL);
    if (!mixer)
        return -ENOMEM;

    /* 初始化同步原語 */
    mutex_init(&mixer->ctrl_lock);
    mutex_init(&mixer->out_lock);
    mutex_init(&mixer->src_disc_lock);
    spin_lock_init(&mixer->buf_lock);
    INIT_LIST_HEAD(&mixer->buf_list);
    init_waitqueue_head(&mixer->buf_wq);
    atomic_set(&mixer->streaming, 0);

    /*
     * 輸出格式在 module_init 時就設定完成。
     * 不依賴 UVC 裝置就緒，out_width/height 始終有效。
     * userspace 可在 UVC 插入之前就完成 S_FMT + REQBUFS。
     */
    mixer->out_width  = param_out_width;
    mixer->out_height = param_out_height;
    mixer->src_width  = param_out_width  / MIXER_NUM_SOURCES;
    mixer->src_height = param_out_height;

    /* 1. 註冊 v4l2_device */
    ret = v4l2_device_register(NULL, &mixer->v4l2_dev);
    if (ret) {
        pr_err("proxy_mixer: v4l2_device_register: %d\n", ret);
        goto err_free;
    }

    /*
     * 2. 立即建立 /dev/video* output node。
     *    此時 UVC sources 尚未就緒，
     *    但 out_width/height 已知，queue_setup 可正常運作。
     *    STREAMON 前的所有 ioctl（S_FMT / REQBUFS / QBUF）皆可成功。
     *    STREAMON 本身會檢查 UVC slots 是否就緒。
     */
    ret = mixer_output_register(mixer);
    if (ret) {
        pr_err("proxy_mixer: output register: %d\n", ret);
        goto err_v4l2;
    }

    /*
     * 3. 註冊 class_interface，監聽 video4linux class 事件。
     *    - 若 uvcvideo 已載入：對現有符合裝置立即觸發 add_dev
     *    - 若 uvcvideo 未載入：等待未來的 add_dev 事件
     *    add_dev 只負責 filp_open → 填入 slot，不啟動 pipeline。
     */
    ret = mixer_register_class_intf(mixer);
    if (ret) {
        pr_err("proxy_mixer: class_intf register: %d\n", ret);
        goto err_output;
    }

    g_mixer = mixer;
    pr_info("proxy_mixer: loaded — /dev/video%d ready "
            "(VID=0x%04x PID=0x%04x, output %ux%u)\n",
            mixer->vdev_out.num,
            target_vid, target_pid,
            mixer->out_width, mixer->out_height);
    return 0;

err_output:
    mixer_output_unregister(mixer);
err_v4l2:
    v4l2_device_unregister(&mixer->v4l2_dev);
err_free:
    kfree(mixer);
    return ret;
}

static void __exit proxy_mixer_exit(void)
{
    struct proxy_mixer *mixer = g_mixer;
    int i;

    if (!mixer)
        return;

    /* 1. 停止接收新事件（class_interface 登出）*/
    mixer_unregister_class_intf(mixer);

    /* 2. 等待所有 workqueue 任務完成 */
    flush_scheduled_work();

    /* 3. 停止 UVC streaming 與 kthread */
    mixer_uvc_stop(mixer);

    /* 4. 關閉所有 source filp */
    for (i = 0; i < MIXER_NUM_SOURCES; i++) {
        if (!IS_ERR_OR_NULL(mixer->src[i].filp)) {
            filp_close(mixer->src[i].filp, NULL);
            mixer->src[i].filp = NULL;
        }
    }

    /* 5. 登出 output video node（先於 v4l2_device）*/
    mixer_output_unregister(mixer);

    /* 6. 登出 v4l2_device */
    v4l2_device_unregister(&mixer->v4l2_dev);

    kfree(mixer);
    g_mixer = NULL;
    pr_info("proxy_mixer: unloaded\n");
}

module_init(proxy_mixer_init);
module_exit(proxy_mixer_exit);
```

---

## proxy_mixer_disc.c — class_interface 裝置發現

```c
#include "proxy_mixer.h"

/* module 參數（extern，在 main.c 定義）*/
extern unsigned short target_vid;
extern unsigned short target_pid;

/* ------------------------------------------------------------------ */
/* VID/PID + CAPTURE 過濾 */

static bool mixer_match_device(struct video_device *vdev,
                                u16 vid, u16 pid)
{
    struct device        *intf_dev;
    struct usb_interface *intf;
    struct usb_device    *udev;

    /* 條件 1：必須支援 VIDEO_CAPTURE */
    if (!(vdev->device_caps & V4L2_CAP_VIDEO_CAPTURE))
        return false;

    /* 條件 2：parent 必須是 USB bus + uvcvideo driver */
    if (!vdev->v4l2_dev || !vdev->v4l2_dev->dev)
        return false;

    intf_dev = vdev->v4l2_dev->dev;

    if (!intf_dev->bus || strcmp(intf_dev->bus->name, "usb") != 0)
        return false;
    if (!intf_dev->driver || strcmp(intf_dev->driver->name, "uvcvideo") != 0)
        return false;

    /* 條件 3：USB VID/PID 符合 */
    intf  = to_usb_interface(intf_dev);
    udev  = interface_to_usbdev(intf);

    if (le16_to_cpu(udev->descriptor.idVendor)  != vid ||
        le16_to_cpu(udev->descriptor.idProduct) != pid)
        return false;

    return true;
}

/* ------------------------------------------------------------------ */
/* workqueue：在非 class mutex context 執行實際 open */

struct mixer_disc_work {
    struct work_struct    work;
    struct proxy_mixer   *mixer;
    struct video_device  *vdev;
    bool                  is_add;
};

static void mixer_disc_work_fn(struct work_struct *w)
{
    struct mixer_disc_work *dw =
        container_of(w, struct mixer_disc_work, work);
    struct proxy_mixer  *mixer = dw->mixer;
    struct video_device *vdev  = dw->vdev;
    int slot = -1, i;

    if (dw->is_add) {
        /* ── ADD 路徑 ── */
        mutex_lock(&mixer->src_disc_lock);

        /* 已有兩路，忽略多餘裝置 */
        if (mixer->src_ready_count >= MIXER_NUM_SOURCES) {
            mutex_unlock(&mixer->src_disc_lock);
            goto out;
        }

        /* 找空 slot */
        for (i = 0; i < MIXER_NUM_SOURCES; i++) {
            if (!mixer->src[i].filp) {
                slot = i;
                break;
            }
        }
        mutex_unlock(&mixer->src_disc_lock);

        if (slot < 0)
            goto out;

        /* filp_open：使用 vdev->num 確保路徑正確 */
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
        mixer->src[slot].fh   = filp->private_data;
        mixer->src[slot].vbq  = mixer->src[slot].vdev->queue;
        mixer->src[slot].udev =
            interface_to_usbdev(
                to_usb_interface(vdev->v4l2_dev->dev));
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
        /* ── REMOVE 路徑 ── */

        /* 若正在 streaming，立即停止 */
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

/* ------------------------------------------------------------------ */
/* class_interface callbacks（在 class mutex 下呼叫，最小化工作）*/

static int mixer_class_add_dev(struct device *dev,
                                struct class_interface *ci)
{
    struct proxy_mixer   *mixer =
        container_of(ci, struct proxy_mixer, class_intf);
    struct video_device  *vdev  = to_video_device(dev);
    struct mixer_disc_work *dw;

    if (!vdev)
        return 0;

    if (!mixer_match_device(vdev, target_vid, target_pid))
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
    struct proxy_mixer   *mixer =
        container_of(ci, struct proxy_mixer, class_intf);
    struct video_device  *vdev  = to_video_device(dev);
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

/* ------------------------------------------------------------------ */

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
```

---

## proxy_mixer_ctrl.c — UVC 控制路徑與 STREAMON / STREAMOFF

```c
#include "proxy_mixer.h"

/* ------------------------------------------------------------------ */
/* UVC Format 協商（在 STREAMON 時呼叫）*/

static int mixer_uvc_negotiate_format(struct proxy_mixer *mixer, int idx)
{
    struct mixer_src *src = &mixer->src[idx];
    int ret;

    src->fmt.type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    src->fmt.fmt.pix.width        = mixer->src_width;
    src->fmt.fmt.pix.height       = mixer->src_height;
    src->fmt.fmt.pix.pixelformat  = V4L2_PIX_FMT_YUYV;
    src->fmt.fmt.pix.field        = V4L2_FIELD_NONE;

    ret = MIXER_CALL_OP(src, vidioc_s_fmt_vid_cap,
                        src->filp, src->fh, &src->fmt);
    if (ret) {
        pr_err("proxy_mixer: src%d S_FMT failed: %d\n", idx, ret);
        return ret;
    }

    /* 讀回實際協商結果 */
    ret = MIXER_CALL_OP(src, vidioc_g_fmt_vid_cap,
                        src->filp, src->fh, &src->fmt);
    if (ret)
        return ret;

    pr_info("proxy_mixer: src%d format: %ux%u pixfmt=0x%08x\n",
            idx,
            src->fmt.fmt.pix.width,
            src->fmt.fmt.pix.height,
            src->fmt.fmt.pix.pixelformat);

    /* 確認 UVC 接受的格式與我們要求的一致 */
    if (src->fmt.fmt.pix.width  != mixer->src_width  ||
        src->fmt.fmt.pix.height != mixer->src_height ||
        src->fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        pr_warn("proxy_mixer: src%d format mismatch "
                "(got %ux%u 0x%08x)\n",
                idx,
                src->fmt.fmt.pix.width,
                src->fmt.fmt.pix.height,
                src->fmt.fmt.pix.pixelformat);
        /*
         * 繼續執行（UVC 可能給出略為不同的尺寸）。
         * 若嚴格要求一致，此處改為 return -EINVAL。
         */
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* UVC REQBUFS + 初始 QBUF */

static int mixer_uvc_reqbufs(struct proxy_mixer *mixer, int idx)
{
    struct mixer_src *src = &mixer->src[idx];
    struct v4l2_requestbuffers req = {
        .count  = MIXER_SRC_NUM_BUFS,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    unsigned int i;
    int ret;

    ret = MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
    if (ret) {
        pr_err("proxy_mixer: src%d REQBUFS failed: %d\n", idx, ret);
        return ret;
    }
    if (req.count < 2) {
        pr_err("proxy_mixer: src%d: only %u buffers\n", idx, req.count);
        return -ENOMEM;
    }

    for (i = 0; i < req.count; i++) {
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

    pr_info("proxy_mixer: src%d: %u buffers ready\n", idx, req.count);
    return 0;
}

/* ------------------------------------------------------------------ */
/* UVC STREAMON */

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

/* ------------------------------------------------------------------ */
/* UVC STREAMOFF + REQBUFS(0) */

static void mixer_uvc_streamoff_one(struct proxy_mixer *mixer, int idx)
{
    struct mixer_src *src = &mixer->src[idx];
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_requestbuffers req = {
        .count  = 0,   /* 釋放所有 buffer */
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (IS_ERR_OR_NULL(src->filp) || !src->vdev)
        return;

    MIXER_CALL_OP(src, vidioc_streamoff, src->filp, src->fh, type);
    MIXER_CALL_OP(src, vidioc_reqbufs,   src->filp, src->fh, &req);
}

/* ------------------------------------------------------------------ */
/* mixer_uvc_negotiate_and_start：STREAMON 時的完整 UVC 啟動序列 */

int mixer_uvc_negotiate_and_start(struct proxy_mixer *mixer)
{
    int i, ret;

    /*
     * ── 步驟 1：確認兩個 slots 都有 UVC 裝置 ──
     *
     * 這是 STREAMON 的第一道關卡。
     * 若任一 slot 未就緒，立即回傳 -ENODEV，
     * kthread 不會啟動，UVC 不會被觸碰。
     */
    mutex_lock(&mixer->src_disc_lock);
    if (mixer->src_ready_count < MIXER_NUM_SOURCES) {
        int ready = mixer->src_ready_count;
        mutex_unlock(&mixer->src_disc_lock);
        pr_err("proxy_mixer: STREAMON rejected: only %d/%d UVC sources ready\n",
               ready, MIXER_NUM_SOURCES);
        return -ENODEV;
    }
    mutex_unlock(&mixer->src_disc_lock);

    mutex_lock(&mixer->ctrl_lock);

    /* ── 步驟 2：Format 協商 ── */
    for (i = 0; i < MIXER_NUM_SOURCES; i++) {
        ret = mixer_uvc_negotiate_format(mixer, i);
        if (ret) goto err_unlock;
    }

    /* ── 步驟 3：UVC REQBUFS + 初始 QBUF ── */
    for (i = 0; i < MIXER_NUM_SOURCES; i++) {
        ret = mixer_uvc_reqbufs(mixer, i);
        if (ret) goto err_unlock;
    }

    /* ── 步驟 4：UVC STREAMON × 2 ── */
    for (i = 0; i < MIXER_NUM_SOURCES; i++) {
        ret = mixer_uvc_streamon(mixer, i);
        if (ret) goto err_streamoff;
    }

    /* ── 步驟 5：啟動 kthread ── */
    atomic_set(&mixer->streaming, 1);
    mixer->mixer_task = kthread_run(mixer_kthread_fn, mixer,
                                    "proxy_mixer/combine");
    if (IS_ERR(mixer->mixer_task)) {
        ret = PTR_ERR(mixer->mixer_task);
        mixer->mixer_task = NULL;
        atomic_set(&mixer->streaming, 0);
        goto err_streamoff;
    }

    mutex_unlock(&mixer->ctrl_lock);
    pr_info("proxy_mixer: pipeline started\n");
    return 0;

err_streamoff:
    for (i = 0; i < MIXER_NUM_SOURCES; i++)
        mixer_uvc_streamoff_one(mixer, i);
err_unlock:
    mutex_unlock(&mixer->ctrl_lock);
    return ret;
}

/* ------------------------------------------------------------------ */
/* mixer_uvc_stop：STREAMOFF 或 hotplug remove 時呼叫 */

void mixer_uvc_stop(struct proxy_mixer *mixer)
{
    int i;

    /* 冪等：若已停止則直接返回 */
    if (!atomic_cmpxchg(&mixer->streaming, 1, 0))
        return;

    pr_info("proxy_mixer: stopping pipeline\n");

    /* 1. 喚醒 kthread（若正在等待 output buffer），讓它能離開 wait */
    wake_up_all(&mixer->buf_wq);

    /* 2. 等待 kthread 退出 */
    if (mixer->mixer_task) {
        kthread_stop(mixer->mixer_task);
        mixer->mixer_task = NULL;
    }

    /* 3. 還回 kthread 可能持有的 source buffer */
    for (i = 0; i < MIXER_NUM_SOURCES; i++) {
        struct mixer_src *src = &mixer->src[i];
        if (src->buf_ready) {
            struct v4l2_buffer buf = {
                .index  = src->cur_vb->index,
                .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                .memory = V4L2_MEMORY_MMAP,
            };
            MIXER_CALL_OP(src, vidioc_qbuf,
                          src->filp, src->fh, &buf);
            src->cur_vb    = NULL;
            src->buf_ready = false;
        }
    }

    /* 4. UVC STREAMOFF + 釋放 UVC buffer */
    for (i = 0; i < MIXER_NUM_SOURCES; i++)
        mixer_uvc_streamoff_one(mixer, i);

    pr_info("proxy_mixer: pipeline stopped\n");
}
```

---

## proxy_mixer_data.c — kthread 與資料流

```c
#include "proxy_mixer.h"

/* ------------------------------------------------------------------ */
/* UVC DQBUF / QBUF 包裝 */

static int mixer_dqbuf_src(struct proxy_mixer *mixer, int idx)
{
    struct mixer_src *src = &mixer->src[idx];
    struct v4l2_buffer buf = {
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    int ret;

    ret = MIXER_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
    if (ret)
        return ret;

    if (buf.index >= src->vbq->num_buffers)
        return -EINVAL;

    src->cur_vb  = src->vbq->bufs[buf.index];
    src->cur_ts  = ns_to_ktime(
                       (u64)buf.timestamp.tv_sec  * NSEC_PER_SEC +
                       (u64)buf.timestamp.tv_usec * NSEC_PER_USEC);
    src->buf_ready = true;
    return 0;
}

static void mixer_qbuf_src(struct proxy_mixer *mixer, int idx)
{
    struct mixer_src *src = &mixer->src[idx];
    struct v4l2_buffer buf = {
        .index  = src->cur_vb->index,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
    src->cur_vb    = NULL;
    src->buf_ready = false;
}

/* ------------------------------------------------------------------ */
/* Frame 同步 */

static void mixer_sync_frames(struct proxy_mixer *mixer)
{
    int drop = 0;

    while (drop < MIXER_SYNC_MAX_DROP) {
        s64 delta = ktime_to_us(
            ktime_sub(mixer->src[0].cur_ts,
                      mixer->src[1].cur_ts));

        if (abs(delta) <= (s64)MIXER_SYNC_TOLERANCE_US)
            return;

        /* 丟棄較舊的那一路，重新 DQBUF */
        if (delta > 0) {
            mixer_qbuf_src(mixer, 1);
            if (mixer_dqbuf_src(mixer, 1) != 0) return;
        } else {
            mixer_qbuf_src(mixer, 0);
            if (mixer_dqbuf_src(mixer, 0) != 0) return;
        }
        drop++;
    }
    /* 超過 MAX_DROP，強制繼續（接受時間差）*/
}

/* ------------------------------------------------------------------ */
/* Frame 合併：YUYV side-by-side */

static void mixer_combine_yuyv(
    u8 *dst,
    const u8 *src0, u32 w0,
    const u8 *src1, u32 w1,
    u32 h)
{
    const u32 s0       = w0 * 2;
    const u32 s1       = w1 * 2;
    const u32 dst_s    = (w0 + w1) * 2;

    for (u32 y = 0; y < h; y++) {
        u8       *d  = dst  + (u64)y * dst_s;
        const u8 *a  = src0 + (u64)y * s0;
        const u8 *b  = src1 + (u64)y * s1;
        memcpy(d,      a, s0);
        memcpy(d + s0, b, s1);
    }
}

/* ------------------------------------------------------------------ */
/* kthread 主迴圈 */

int mixer_kthread_fn(void *data)
{
    struct proxy_mixer *mixer = data;
    int ret;

    /* FIFO RT priority：避免被 UI process 搶占造成 frame drop */
    sched_set_fifo(current);

    pr_info("proxy_mixer: kthread started\n");

    while (!kthread_should_stop()) {

        /* ── 步驟 1：等待一個 output buffer 被 userspace QBUF ── */
        struct mixer_buf  *mbuf;
        struct vb2_buffer *out_vb;
        unsigned long flags;

        /*
         * 等待 buf_list 非空或 streaming 停止。
         * 使用 wait_event_interruptible 而不是 spin-poll，
         * 讓出 CPU 給其他 process。
         */
        ret = wait_event_interruptible(
            mixer->buf_wq,
            !list_empty(&mixer->buf_list) ||
            kthread_should_stop() ||
            !atomic_read(&mixer->streaming));

        if (kthread_should_stop() || !atomic_read(&mixer->streaming))
            break;
        if (ret == -ERESTARTSYS)
            continue;

        spin_lock_irqsave(&mixer->buf_lock, flags);
        if (list_empty(&mixer->buf_list)) {
            spin_unlock_irqrestore(&mixer->buf_lock, flags);
            continue;
        }
        mbuf = list_first_entry(&mixer->buf_list,
                                struct mixer_buf, list);
        list_del(&mbuf->list);
        spin_unlock_irqrestore(&mixer->buf_lock, flags);

        out_vb = &mbuf->vbuf.vb2_buf;

        /* ── 步驟 2：DQBUF from UVC src[0] ── */
        ret = mixer_dqbuf_src(mixer, 0);
        if (ret) {
            if (ret == -ENODEV || ret == -EIO) {
                pr_err("proxy_mixer: src0 fatal: %d\n", ret);
                vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
                break;
            }
            /* 暫時失敗：把 output buffer 還回 list 重試 */
            spin_lock_irqsave(&mixer->buf_lock, flags);
            list_add(&mbuf->list, &mixer->buf_list);
            spin_unlock_irqrestore(&mixer->buf_lock, flags);
            usleep_range(1000, 2000);
            continue;
        }

        /* ── 步驟 3：DQBUF from UVC src[1] ── */
        ret = mixer_dqbuf_src(mixer, 1);
        if (ret) {
            /* src0 已 dequeued，必須先還回 */
            mixer_qbuf_src(mixer, 0);
            if (ret == -ENODEV || ret == -EIO) {
                pr_err("proxy_mixer: src1 fatal: %d\n", ret);
                vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
                break;
            }
            spin_lock_irqsave(&mixer->buf_lock, flags);
            list_add(&mbuf->list, &mixer->buf_list);
            spin_unlock_irqrestore(&mixer->buf_lock, flags);
            usleep_range(1000, 2000);
            continue;
        }

        /* ── 步驟 4：Frame 同步 ── */
        mixer_sync_frames(mixer);

        /* ── 步驟 5：取得三塊 buffer 的 kernel virtual address ──
         *
         * vb2_plane_vaddr() 在 DQ 後、QBuf 前保證有效。
         * uvcvideo 使用 videobuf2-vmalloc，回傳 vmalloc VA，
         * 不需要額外 vmap()。
         */
        void *dst  = vb2_plane_vaddr(out_vb,                 0);
        void *src0 = vb2_plane_vaddr(mixer->src[0].cur_vb, 0);
        void *src1 = vb2_plane_vaddr(mixer->src[1].cur_vb, 0);

        if (!dst || !src0 || !src1) {
            pr_err("proxy_mixer: vb2_plane_vaddr NULL\n");
            vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
            mixer_qbuf_src(mixer, 0);
            mixer_qbuf_src(mixer, 1);
            continue;
        }

        /* ── 步驟 6：合併（唯一一次 memcpy）── */
        mixer_combine_yuyv(
            dst,
            src0, mixer->src[0].fmt.fmt.pix.width,
            src1, mixer->src[1].fmt.fmt.pix.width,
            mixer->out_height);

        /* ── 步驟 7：設定 payload 與 timestamp，通知 consumer ── */
        vb2_set_plane_payload(out_vb, 0,
            (size_t)mixer->out_width * mixer->out_height * 2);
        out_vb->timestamp = ktime_get_ns();
        vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);

        /* ── 步驟 8：歸還 source buffer（必須在 combine 之後）── */
        mixer_qbuf_src(mixer, 0);
        mixer_qbuf_src(mixer, 1);
    }

    pr_info("proxy_mixer: kthread stopped\n");
    return 0;
}
```

---

## proxy_mixer_output.c — vb2_ops / ioctl_ops / video_device 註冊

```c
#include "proxy_mixer.h"

/* ------------------------------------------------------------------ */
/* vb2_ops */

static int mixer_out_queue_setup(struct vb2_queue *q,
    unsigned int *nbuffers, unsigned int *nplanes,
    unsigned int sizes[], struct device *alloc_devs[])
{
    struct proxy_mixer *mixer = vb2_get_drv_priv(q);

    /*
     * out_width/height 在 module_init 時由 module_param 設定，
     * 此處始終有效，不依賴 UVC sources 是否就緒。
     */
    unsigned int size = mixer->out_width * mixer->out_height * 2; /* YUYV */

    if (*nplanes)
        return sizes[0] < size ? -EINVAL : 0;

    *nplanes  = 1;
    sizes[0]  = size;
    *nbuffers = max(*nbuffers, (unsigned int)MIXER_OUT_NUM_BUFS);
    return 0;
}

static void mixer_out_buf_queue(struct vb2_buffer *vb)
{
    struct proxy_mixer    *mixer = vb2_get_drv_priv(vb->vb2_queue);
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct mixer_buf      *mbuf  =
        container_of(vbuf, struct mixer_buf, vbuf);
    unsigned long flags;

    spin_lock_irqsave(&mixer->buf_lock, flags);
    list_add_tail(&mbuf->list, &mixer->buf_list);
    spin_unlock_irqrestore(&mixer->buf_lock, flags);

    /* 通知 kthread 有新的 output buffer 可用 */
    wake_up(&mixer->buf_wq);
}

static int mixer_out_start_streaming(struct vb2_queue *q,
                                      unsigned int count)
{
    /*
     * vb2_ioctl_streamon 會呼叫此 callback。
     * 實際的 UVC 啟動和 kthread 在 vidioc_streamon 中
     * 已先完成，此處不需要額外動作。
     */
    return 0;
}

static void mixer_out_stop_streaming(struct vb2_queue *q)
{
    struct proxy_mixer *mixer = vb2_get_drv_priv(q);
    struct mixer_buf *mbuf, *tmp;
    unsigned long flags;

    /* 將所有尚未填充的 buffer 以 ERROR 回傳給 userspace */
    spin_lock_irqsave(&mixer->buf_lock, flags);
    list_for_each_entry_safe(mbuf, tmp, &mixer->buf_list, list) {
        list_del(&mbuf->list);
        vb2_buffer_done(&mbuf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&mixer->buf_lock, flags);
}

const struct vb2_ops mixer_out_vb2_ops = {
    .queue_setup     = mixer_out_queue_setup,
    .buf_queue       = mixer_out_buf_queue,
    .start_streaming = mixer_out_start_streaming,
    .stop_streaming  = mixer_out_stop_streaming,
    /* wait_prepare / wait_finish：kernel ≥ 6.8 optional */
};

/* ------------------------------------------------------------------ */
/* ioctl_ops */

static int mixer_vidioc_querycap(struct file *file, void *fh,
                                  struct v4l2_capability *cap)
{
    strscpy(cap->driver,   "proxy_mixer",         sizeof(cap->driver));
    strscpy(cap->card,     "V4L2 Proxy-Mixer",    sizeof(cap->card));
    strscpy(cap->bus_info, "platform:proxy_mixer", sizeof(cap->bus_info));
    cap->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
    return 0;
}

static int mixer_vidioc_enum_fmt(struct file *file, void *fh,
                                  struct v4l2_fmtdesc *f)
{
    if (f->index > 0)
        return -EINVAL;
    f->pixelformat = V4L2_PIX_FMT_YUYV;
    strscpy(f->description, "YUYV 4:2:2", sizeof(f->description));
    return 0;
}

static void mixer_fill_fmt(struct proxy_mixer *mixer,
                            struct v4l2_format *f)
{
    f->type                 = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f->fmt.pix.width        = mixer->out_width;
    f->fmt.pix.height       = mixer->out_height;
    f->fmt.pix.pixelformat  = V4L2_PIX_FMT_YUYV;
    f->fmt.pix.field        = V4L2_FIELD_NONE;
    f->fmt.pix.bytesperline = mixer->out_width * 2;
    f->fmt.pix.sizeimage    = mixer->out_width * mixer->out_height * 2;
    f->fmt.pix.colorspace   = V4L2_COLORSPACE_SRGB;
}

static int mixer_vidioc_g_fmt(struct file *file, void *fh,
                               struct v4l2_format *f)
{
    mixer_fill_fmt(video_drvdata(file), f);
    return 0;
}

static int mixer_vidioc_s_fmt(struct file *file, void *fh,
                               struct v4l2_format *f)
{
    /*
     * 輸出格式由 module_param 固定。
     * 忽略 userspace 的修改，回傳實際格式。
     * ffmpeg / OBS 在 S_FMT 後會做 G_FMT 確認，
     * 兩者一致即可正常運作。
     */
    mixer_fill_fmt(video_drvdata(file), f);
    return 0;
}

/*
 * vidioc_reqbufs：使用 vb2 helper。
 * queue_setup 中 out_width/height 已知，可正常分配 buffer。
 * UVC 的 REQBUFS 在 STREAMON 時才執行。
 */

/*
 * vidioc_streamon：自訂包裝。
 * 先執行 UVC 完整啟動序列，成功後才啟動 output vb2 queue。
 */
static int mixer_vidioc_streamon(struct file *file, void *fh,
                                  enum v4l2_buf_type type)
{
    struct proxy_mixer *mixer = video_drvdata(file);
    int ret;

    if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    /* UVC 檢查 + format 協商 + UVC REQBUFS/QBUF + UVC STREAMON + kthread */
    ret = mixer_uvc_negotiate_and_start(mixer);
    if (ret)
        return ret;

    /* output vb2 queue 啟動（呼叫 start_streaming callback）*/
    ret = vb2_ioctl_streamon(file, fh, type);
    if (ret) {
        mixer_uvc_stop(mixer);
        return ret;
    }

    return 0;
}

/*
 * vidioc_streamoff：自訂包裝。
 * 先停止 kthread 與 UVC，再停止 output vb2 queue。
 */
static int mixer_vidioc_streamoff(struct file *file, void *fh,
                                   enum v4l2_buf_type type)
{
    struct proxy_mixer *mixer = video_drvdata(file);

    if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    /* 停止 kthread + UVC */
    mixer_uvc_stop(mixer);

    /* 停止 output vb2 queue（呼叫 stop_streaming callback）*/
    return vb2_ioctl_streamoff(file, fh, type);
}

const struct v4l2_ioctl_ops mixer_ioctl_ops = {
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
};

const struct v4l2_file_operations mixer_fops = {
    .owner          = THIS_MODULE,
    .open           = v4l2_fh_open,
    .release        = vb2_fop_release,
    .read           = vb2_fop_read,
    .poll           = vb2_fop_poll,
    .mmap           = vb2_fop_mmap,
    .unlocked_ioctl = video_ioctl2,
};

/* ------------------------------------------------------------------ */
/* video_device 初始化與登出 */

int mixer_output_register(struct proxy_mixer *mixer)
{
    struct vb2_queue    *q    = &mixer->out_queue;
    struct video_device *vdev = &mixer->vdev_out;
    int ret;

    memset(q, 0, sizeof(*q));
    q->type               = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes           = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
    q->drv_priv           = mixer;
    q->buf_struct_size    = sizeof(struct mixer_buf);
    q->ops                = &mixer_out_vb2_ops;
    q->mem_ops            = &vb2_vmalloc_memops;
    q->timestamp_flags    = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->lock               = &mixer->out_lock;
    q->min_queued_buffers = 2;   /* kernel ≥ 6.8；舊版用 min_buffers_needed */

    ret = vb2_queue_init(q);
    if (ret)
        return ret;

    memset(vdev, 0, sizeof(*vdev));
    strscpy(vdev->name, "proxy_mixer_out", sizeof(vdev->name));
    vdev->fops        = &mixer_fops;
    vdev->ioctl_ops   = &mixer_ioctl_ops;
    vdev->release     = video_device_release_empty;
    vdev->v4l2_dev    = &mixer->v4l2_dev;
    vdev->queue       = q;
    vdev->lock        = &mixer->out_lock;
    vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
    video_set_drvdata(vdev, mixer);

    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        vb2_queue_release(q);
        return ret;
    }

    pr_info("proxy_mixer: output node: /dev/video%d\n", vdev->num);
    return 0;
}

void mixer_output_unregister(struct proxy_mixer *mixer)
{
    if (mixer->vdev_out.v4l2_dev) {
        video_unregister_device(&mixer->vdev_out);
        vb2_queue_release(&mixer->out_queue);
        mixer->vdev_out.v4l2_dev = NULL;
    }
}
```

---

## Makefile

```makefile
obj-m := proxy_mixer.o
proxy_mixer-objs := proxy_mixer_main.o  \
                    proxy_mixer_disc.o  \
                    proxy_mixer_ctrl.o  \
                    proxy_mixer_data.o  \
                    proxy_mixer_output.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
```

---

## 完整狀態機與關鍵事件

```
State: IDLE
  ├─ modprobe              → video_register_device() → /dev/video* 建立
  │                           State: IDLE（等待 UVC）
  │
  ├─ UVC add_dev（slot++） → src_ready_count = 0→1→2
  │                           State: IDLE（slots 填滿，但未 streaming）
  │
  ├─ ffmpeg STREAMON
  │    └─ src_ready_count < 2  → -ENODEV  State: IDLE
  │    └─ src_ready_count == 2
  │         → UVC S_FMT / G_FMT
  │         → UVC REQBUFS + QBUF
  │         → UVC STREAMON × 2
  │         → kthread_run()
  │         → vb2_ioctl_streamon()
  │              State: STREAMING
  │
  ├─ ffmpeg STREAMOFF      → mixer_uvc_stop()
  │                           → vb2_ioctl_streamoff()
  │                              State: IDLE（slots 保留）
  │
  ├─ UVC remove_dev        → mixer_uvc_stop()（若 STREAMING）
  │                           → filp_close()，slot 清除
  │                              State: IDLE（slots 不足）
  │
  └─ rmmod                 → mixer_uvc_stop()
                              → video_unregister_device()
                                 State: 不存在
```
