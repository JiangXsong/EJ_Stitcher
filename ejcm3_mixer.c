#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/kallsyms.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/scatterlist.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define MIXER_NUM_SOURCES   2
#define MIXER_SRC_NUM_BUFS  4
#define MIXER_OUT_NUM_BUFS  4
#define MIXER_MAX_OUT_BUFS	8
#define MIXER_SYNC_TOLERANCE_US 16667ULL /* half frame @ 30fps */
#define MIXER_SYNC_MAX_DROP 8

#define MIXER_DEFAULT_VID   0x1E4E
#define MIXER_DEFAULT_PID_P 0x7301
#define MIXER_DEFAULT_PID_S 0x7302

#define MIXER_DEFAULT_OUT_WIDTH  3840
#define MIXER_DEFAULT_OUT_HEIGHT 2160
#define MIXER_DEFAULT_FPS_NUM    1
#define MIXER_DEFAULT_FPS_DEN    60

/* ================================================================== */
/* Module parameters                                                  */
/* ================================================================== */
static bool zerocopy = true;
module_param(zerocopy, bool, 0444);
MODULE_PARM_DESC(zerocopy, "Enable DMABUF feed-through zero-copy (default: true)");

/*
 * MIXER_CALL_OP - call a UVC ioctl op directly, holding the vdev lock.
 *
 * NOTE: This bypasses V4L2 core's capability checks and v4l2_fh state
 * validation. It is intentional for this kernel-space proxy design, but
 * callers must ensure the source filp/vdev remain valid (protected by
 * src_disc_lock or uvc_ctrl_lock as appropriate).
 */
#define MIXER_CALL_OP(src, op, ...)                           \
	({                                                        \
		int __ret = -ENOTTY;                                  \
		struct video_device *__vd = (src)->vdev;              \
		if (__vd && __vd->ioctl_ops && __vd->ioctl_ops->op) { \
            if (__vd->lock) mutex_lock(__vd->lock);           \
			__ret = __vd->ioctl_ops->op(__VA_ARGS__);         \
            if (__vd->lock) mutex_unlock(__vd->lock);         \
		}                                                     \
		__ret;                                                \
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

	/* Related USB device */
	struct usb_device *udev;
};

/* ── dma_buf exporter private data ── */
struct mixer_dmabuf_priv {
	struct page **pages;
	unsigned int num_pages;
	unsigned int offset;/* byte offset within first page */
	size_t size;/* total byte length */
};

/*
 * Per-output-buffer DMABUF tracking.
 * Each output buffer has one dma_buf per source (covering its half).
 */
struct mixer_dmabuf_slot {
	struct dma_buf *dbuf[MIXER_NUM_SOURCES];
	struct page **pages;/* vmalloc page array for this buffer */
	unsigned int num_pages;
	bool created;
};

struct mixer_buffer {
	struct vb2_v4l2_buffer buf;
	struct list_head queue;
};

struct mixer_disc_work {
	struct work_struct work;
	struct proxy_mixer *mixer;
	struct video_device *vdev;
	bool is_add;
};

struct mixer_video_queue {
	struct vb2_queue vbq;
	struct mutex mutex;

	spinlock_t irqlock;
	struct list_head irqqueue;
	wait_queue_head_t buf_wq;
};

struct proxy_mixer {
	struct v4l2_device v4l2_dev;
	struct video_device *vdev;
	struct mutex lock;

	struct class_interface class_intf;
	int src_ready_count;
	struct mutex src_disc_lock;

	struct workqueue_struct *disc_wq;
	struct mixer_slot src[MIXER_NUM_SOURCES];

	u32 out_width;
	u32 out_height;
	u32 out_pixfmt;
	u32 src_width;
	u32 src_height;
	u32 fps_num;
	u32 fps_den;

	struct mixer_video_queue out_q;
	bool out_q_initialized;

	struct mutex uvc_ctrl_lock; /* Protects UVC control operations */

	struct task_struct *mixer_task;
	atomic_t streaming;

	/*
	 * DMABUF feed-through state.
	 * use_dmabuf: resolved at start_streaming (zerocopy && VB2_DMABUF ok).
	 * dma_slots: per-output-buffer dma_buf tracking.
	 * out_vb_map: UVC buffer index → output vb2_buffer pointer.
	 * num_dmabuf_slots: number of active dma_slots.
	 */
	bool use_dmabuf;
	unsigned int num_dmabuf_slots;
	struct mixer_dmabuf_slot dma_slots[MIXER_MAX_OUT_BUFS];
	struct vb2_buffer *out_vb_map[MIXER_MAX_OUT_BUFS];
};

/* ------------------------------------------------------------------ */
/* Forward declarations */
static int mixer_uvc_start(struct proxy_mixer *mixer);
static void mixer_uvc_stop(struct proxy_mixer *mixer);
static int mixer_register_class_intf(struct proxy_mixer *mixer,
				     const struct class *vcls);
static void mixer_unregister_class_intf(struct proxy_mixer *mixer);
static void mixer_unregister(struct proxy_mixer *mixer);
static int init_queue(struct mixer_video_queue *q);
static void mixer_queue_release(struct mixer_video_queue *q);

static struct proxy_mixer *g_mixer;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Syu-Song Chiang");
MODULE_DESCRIPTION("V4L2 Proxy-Mixer: combine two UVC sources in kernel space");
MODULE_IMPORT_NS("DMA_BUF");

/* ------------------------------------------------------------------ */
/* Pixel-format helpers */

/*
 * mixer_image_size - return total bytes for one frame at given w/h/pixfmt.
 *   YUYV : w * h * 2
 *   NV12 : w * h * 3 / 2
 */
static inline unsigned int mixer_image_size(u32 w, u32 h, u32 pixfmt)
{
	if (pixfmt == V4L2_PIX_FMT_NV12)
		return w * h * 3 / 2;
	return w * h * 2; /* YUYV default */
}

/*
 * mixer_bytesperline - return bytes-per-line for the given format.
 *   YUYV : w * 2   (packed, single plane)
 *   NV12 : w       (Y stride; UV stride is the same for interleaved UV)
 */
static inline unsigned int mixer_bytesperline(u32 w, u32 pixfmt)
{
	if (pixfmt == V4L2_PIX_FMT_NV12)
		return w;
	return w * 2; /* YUYV */
}

static inline bool mixer_pixfmt_valid(u32 pixfmt)
{
	return pixfmt == V4L2_PIX_FMT_YUYV || pixfmt == V4L2_PIX_FMT_NV12;
}

/* ================================================================== */
/* dma_buf exporter ops                                                */
/* ================================================================== */

static struct sg_table *mixer_dmabuf_map(struct dma_buf_attachment *attach,
					 enum dma_data_direction dir)
{
	struct mixer_dmabuf_priv *priv = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	/*
	 * sg_alloc_table_from_pages handles non-page-aligned ranges:
	 * offset = byte offset within first page, size = total bytes.
	 * The SG entries describe the exact byte range for DMA.
	 */
	ret = sg_alloc_table_from_pages(sgt, priv->pages, priv->num_pages,
					priv->offset, priv->size, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}

	/* Map for the importing device (USB host controller) */
	ret = dma_map_sgtable(attach->dev, sgt, dir, 0);
	if (ret) {
		sg_free_table(sgt);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	return sgt;
}

static void mixer_dmabuf_unmap(struct dma_buf_attachment *attach,
			       struct sg_table *sgt,
			       enum dma_data_direction dir)
{
	dma_unmap_sgtable(attach->dev, sgt, dir, 0);
	sg_free_table(sgt);
	kfree(sgt);
}

static void mixer_dmabuf_release(struct dma_buf *dbuf)
{
	kfree(dbuf->priv);
}

static int mixer_dmabuf_vmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	struct mixer_dmabuf_priv *priv = dbuf->priv;
	void *va;

	va = vmap(priv->pages, priv->num_pages, VM_MAP, PAGE_KERNEL);
	if (!va)
		return -ENOMEM;

	iosys_map_set_vaddr(map, va + priv->offset);
	return 0;
}

static void mixer_dmabuf_vunmap(struct dma_buf *dbuf, struct iosys_map *map)
{
	void *va = map->vaddr;

	if (va) {
		struct mixer_dmabuf_priv *priv = dbuf->priv;

		vunmap(va - priv->offset);
	}
}

static const struct dma_buf_ops mixer_dmabuf_ops = {
	.map_dma_buf = mixer_dmabuf_map,
	.unmap_dma_buf = mixer_dmabuf_unmap,
	.release = mixer_dmabuf_release,
	.vmap = mixer_dmabuf_vmap,
	.vunmap = mixer_dmabuf_vunmap,
};

/* ================================================================== */
/* DMABUF management helpers                                           */
/* ================================================================== */

/*
 * mixer_create_dmabuf - export a byte range of a vmalloc page array as dma_buf.
 *
 * @pages:     array of struct page pointers (from vmalloc_to_page)
 * @num_pages: number of pages in array
 * @offset:    byte offset within first page
 * @size:      total byte size of the range
 *
 * The dma_buf holds a reference to priv, not to the pages themselves.
 * Pages must outlive the dma_buf (guaranteed: output buffer freed after dma_buf).
 */
static struct dma_buf *mixer_create_dmabuf(struct page **pages,
					   unsigned int num_pages,
					   unsigned int offset, size_t size)
{
	struct mixer_dmabuf_priv *priv;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dbuf;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	priv->pages = pages;
	priv->num_pages = num_pages;
	priv->offset = offset;
	priv->size = size;

	exp_info.ops = &mixer_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.priv = priv;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf))
		kfree(priv);

	return dbuf;
}

/*
 * mixer_dmabuf_qbuf_src - QBUF a dma_buf to UVC source via temporary fd.
 *
 * Installs a temporary fd in current->files, calls vidioc_qbuf (which
 * internally does dma_buf_get(fd) → fget → gets our dma_buf), then
 * immediately closes the fd.  vb2 holds its own ref via dma_buf_attach.
 *
 * Safe in both ioctl context (userspace files) and kthread (init_files).
 * The fd exists for microseconds — no practical side effects.
 */
static int mixer_dmabuf_qbuf_src(struct proxy_mixer *mixer, int slot,
				 unsigned int buf_index, struct dma_buf *dbuf)
{
	struct mixer_slot *src = &mixer->src[slot];
	struct v4l2_buffer v4l2_buf = {
		.index = buf_index,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int fd, ret;

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev)
		return -ENODEV;

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0)
		return fd;

	get_file(dbuf->file);
	fd_install(fd, dbuf->file);

	v4l2_buf.m.fd = fd;
	ret = MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &v4l2_buf);

	close_fd(fd);

	if (ret)
		pr_err("proxy_mixer: dmabuf QBUF src%d idx%u failed: %d\n",
		       slot, buf_index, ret);

	return ret;
}

/*
 * mixer_dmabuf_setup_slot - create dma_bufs for one output buffer.
 *
 * Splits the output buffer's pages into two halves (one per source).
 * sg_alloc_table_from_pages with offset handles non-page-aligned splits.
 */
static int mixer_dmabuf_setup_slot(struct proxy_mixer *mixer,
				   struct mixer_dmabuf_slot *ds,
				   struct vb2_buffer *out_vb,
				   unsigned int half_size)
{
	void *va;
	unsigned int frame_size = half_size * 2;
	unsigned int total_pages = PAGE_ALIGN(frame_size) >> PAGE_SHIFT;
	unsigned int a_num_pages, b_first_page, b_offset, b_num_pages;
	unsigned int i;
	int ret = 0;

	va = vb2_plane_vaddr(out_vb, 0);
	if (!va) {
		pr_err("proxy_mixer: vb2_plane_vaddr NULL for out buf %d\n",
		       out_vb->index);
		return -EFAULT;
	}

	/* Collect vmalloc pages */
	ds->pages = kcalloc(total_pages, sizeof(struct page *), GFP_KERNEL);
	if (!ds->pages)
		return -ENOMEM;

	ds->num_pages = total_pages;

	for (i = 0; i < total_pages; i++) {
		ds->pages[i] = vmalloc_to_page((u8 *)va + (u64)i * PAGE_SIZE);
		if (!ds->pages[i]) {
			pr_err("proxy_mixer: vmalloc_to_page NULL at page %u\n", i);
			ret = -EFAULT;
			goto err_pages;
		}
	}

	/*
	 * Half A: bytes [0 .. half_size-1] → feed to src0
	 * Half B: bytes [half_size .. frame_size-1] → feed to src1
	 *
	 * When half_size is NOT page-aligned, page at the split point is
	 * shared between A and B.  sg_alloc_table_from_pages handles this
	 * via the offset parameter — DMA hardware operates at byte level.
	 */
	a_num_pages = (half_size + PAGE_SIZE - 1) >> PAGE_SHIFT;
	b_first_page = half_size >> PAGE_SHIFT;
	b_offset = half_size & (PAGE_SIZE - 1);
	b_num_pages = total_pages - b_first_page;

	/* Verify cache-line alignment of split point within shared page */
	if (b_offset && (b_offset % L1_CACHE_BYTES))
		pr_warn("proxy_mixer: split offset %u not cache-line aligned (line=%u)\n",
			b_offset, L1_CACHE_BYTES);

	/* dma_buf for src0 (first half) */
	ds->dbuf[0] = mixer_create_dmabuf(ds->pages, a_num_pages, 0, half_size);
	if (IS_ERR(ds->dbuf[0])) {
		ret = PTR_ERR(ds->dbuf[0]);
		ds->dbuf[0] = NULL;
		goto err_pages;
	}

	/* dma_buf for src1 (second half) */
	ds->dbuf[1] = mixer_create_dmabuf(&ds->pages[b_first_page], b_num_pages,
					   b_offset, frame_size - half_size);
	if (IS_ERR(ds->dbuf[1])) {
		ret = PTR_ERR(ds->dbuf[1]);
		ds->dbuf[1] = NULL;
		goto err_dbuf0;
	}

	ds->created = true;

	pr_debug("proxy_mixer: dmabuf slot buf%d: A=%u pages off=0 len=%u, "
		 "B=%u pages off=%u len=%u\n",
		 out_vb->index, a_num_pages, half_size,
		 b_num_pages, b_offset, frame_size - half_size);

	return 0;

err_dbuf0:
	dma_buf_put(ds->dbuf[0]);
	ds->dbuf[0] = NULL;
err_pages:
	kfree(ds->pages);
	ds->pages = NULL;
	return ret;
}

static void mixer_dmabuf_teardown_slot(struct mixer_dmabuf_slot *ds)
{
	int i;

	if (!ds->created)
		return;

	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		if (ds->dbuf[i]) {
			dma_buf_put(ds->dbuf[i]);
			ds->dbuf[i] = NULL;
		}
	}

	kfree(ds->pages);
	ds->pages = NULL;
	ds->created = false;
}

static void mixer_dmabuf_teardown_all(struct proxy_mixer *mixer)
{
	unsigned int i;

	for (i = 0; i < mixer->num_dmabuf_slots; i++)
		mixer_dmabuf_teardown_slot(&mixer->dma_slots[i]);
	mixer->num_dmabuf_slots = 0;
}

/*
 * mixer_dmabuf_check_support - check if both UVC sources support DMABUF import.
 * Must be called after sources are discovered (vbq populated).
 */
static bool mixer_dmabuf_check_support(struct proxy_mixer *mixer)
{
	int i;

	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		struct vb2_queue *vbq = mixer->src[i].vbq;

		if (!vbq) {
			pr_info("proxy_mixer: src%d vbq NULL, DMABUF not available\n", i);
			return false;
		}
		if (!(vbq->io_modes & VB2_DMABUF)) {
			pr_info("proxy_mixer: src%d lacks VB2_DMABUF, falling back to memcpy\n", i);
			return false;
		}
	}
	return true;
}


/* ------------------------------------------------------------------ */
/*
 * UVC vb2_queue accessor — see previous commit for full rationale.
 * Scans the opaque uvc_streaming allocation for a vb2_queue whose
 * drv_priv == &self (invariant set by uvc_queue_init).
 */
#define UVC_STREAM_SCAN_BYTES 8192UL

static struct vb2_queue *mixer_scan_for_uvc_vbq(void *stream_base)
{
	char *p = (char *)stream_base;
	char *end = p + UVC_STREAM_SCAN_BYTES - sizeof(struct vb2_queue);
	struct vb2_queue *candidate;

	for (; p <= end; p += sizeof(void *)) {
		candidate = (struct vb2_queue *)p;
		if (candidate->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
			continue;
		if (candidate->drv_priv != (void *)candidate)
			continue;
		if (!candidate->ops || !candidate->mem_ops)
			continue;
		pr_info("proxy_mixer: found uvc vb2_queue at offset +%td\n",
			p - (char *)stream_base);
		return candidate;
	}
	return NULL;
}

static struct vb2_queue *mixer_get_uvc_vbq(struct mixer_slot *src)
{
	void *stream;
	struct vb2_queue *vbq;

	if (!src->vdev)
		return NULL;

	/* Fast path: vdev->queue populated by newer UVC / standard V4L2 */
	vbq = src->vdev->queue;
	if (vbq)
		return vbq;

	/* Slow path: scan uvc_streaming via video_drvdata */
	stream = video_get_drvdata(src->vdev);
	if (!stream) {
		pr_err("proxy_mixer: video_get_drvdata NULL\n");
		return NULL;
	}
	vbq = mixer_scan_for_uvc_vbq(stream);
	if (!vbq)
		pr_err("proxy_mixer: vb2_queue not found in uvc_streaming\n");
	return vbq;
}

/* ------------------------------------------------------------------ */
/**
 * mixer_disc_work_fn - workqueue function for device discovery/removal.
 * 
 * Workqueue function for handling device discovery/removal asynchronously,
 * to avoid blocking the caller (e.g., uvcvideo) and to ensure proper locking.
 */
static void mixer_disc_work_fn(struct work_struct *w)
{
	struct mixer_disc_work *dw =
		container_of(w, struct mixer_disc_work, work);
	struct proxy_mixer *mixer = dw->mixer;
	struct video_device *vdev = dw->vdev;
    struct device *intf_dev;
	struct usb_interface *intf;
	struct usb_device *udev;
	u16 pid;
	char devpath[32];
	struct file *filp;
	int slot = -1;
	int i;

	if (dw->is_add) {
        intf_dev = vdev->v4l2_dev->dev;
        intf = to_usb_interface(intf_dev);
	    udev = interface_to_usbdev(intf);
        pid = le16_to_cpu(udev->descriptor.idProduct);

		mutex_lock(&mixer->src_disc_lock);
		if (mixer->src_ready_count >= MIXER_NUM_SOURCES) {
			mutex_unlock(&mixer->src_disc_lock);
			goto out;
		}

        if (pid == MIXER_DEFAULT_PID_P) {
            if (!mixer->src[0].filp)
				slot = 0;
        } else if (pid == MIXER_DEFAULT_PID_S) {
            if (!mixer->src[1].filp)
				slot = 1;
        }

		if (slot < 0) {
			mutex_unlock(&mixer->src_disc_lock);
			goto out;
		}

		/* Placeholder to mark slot as taken */
		mixer->src[slot].filp = ERR_PTR(-EBUSY);
		mixer->src_ready_count++;
		mutex_unlock(&mixer->src_disc_lock);

		/* filp_open - Open without O_NONBLOCK. */
		snprintf(devpath, sizeof(devpath), "/dev/video%d", vdev->num);
		filp = filp_open(devpath, O_RDWR, 0);
		if (IS_ERR(filp)) {
			pr_err("proxy_mixer: filp_open(%s): %ld\n",
                   devpath, PTR_ERR(filp));
			mutex_lock(&mixer->src_disc_lock);
			mixer->src[slot].filp = NULL;
			mixer->src_ready_count--;
			mutex_unlock(&mixer->src_disc_lock);
			goto out;
		}

		mutex_lock(&mixer->src_disc_lock);
		mixer->src[slot].filp = filp;
		mixer->src[slot].vdev = video_devdata(filp);
		mixer->src[slot].fh = filp->private_data;
		mixer->src[slot].vbq = mixer_get_uvc_vbq(&mixer->src[slot]);
		if (!mixer->src[slot].vbq) {
			pr_err("proxy_mixer: slot%d: cannot locate vb2_queue\n",
			       slot);
			filp_close(filp, NULL);
			mixer->src[slot].filp = NULL;
			mixer->src[slot].vdev = NULL;
			mixer->src[slot].fh = NULL;
			mixer->src_ready_count--;
			mutex_unlock(&mixer->src_disc_lock);
			goto out;
		}
		mixer->src[slot].udev = interface_to_usbdev(
			to_usb_interface(vdev->v4l2_dev->dev));

		pr_info("proxy_mixer: slot%d filled (%s), ready=%d/%d\n",
                slot, devpath, mixer->src_ready_count, MIXER_NUM_SOURCES);
		mutex_unlock(&mixer->src_disc_lock);
	} else {
		if (atomic_read(&mixer->streaming))
			mixer_uvc_stop(mixer);

		mutex_lock(&mixer->src_disc_lock);
		for (i = 0; i < MIXER_NUM_SOURCES; i++) {
			if (mixer->src[i].vdev == vdev) {
				filp_close(mixer->src[i].filp, NULL);
				memset(&mixer->src[i], 0,
				       sizeof(mixer->src[i]));
				mixer->src_ready_count--;
				pr_info("proxy_mixer: slot%d removed, ready=%d/%d\n",
					i, mixer->src_ready_count,
					MIXER_NUM_SOURCES);
				break;
			}
		}
		mutex_unlock(&mixer->src_disc_lock);
	}

out:
	put_device(&vdev->dev);
	kfree(dw);
}

/**
 * mixer_match_device -  return true if vdev is one of target devices.
 * 
 * Discover UVC devices matching the specified VID/PID,
 * and add them as sources to the mixer.
 */
static bool mixer_match_device(struct video_device *vdev)
{
	struct device *intf_dev;
	struct usb_interface *intf;
	struct usb_device *udev;
	u16 pid;

	if (!(vdev->device_caps & V4L2_CAP_VIDEO_CAPTURE))
		return false;

	if (!vdev->v4l2_dev || !vdev->v4l2_dev->dev)
		return false;

	intf_dev = vdev->v4l2_dev->dev;

	if (!intf_dev->bus || strcmp(intf_dev->bus->name, "usb") != 0)
		return false;
	if (!intf_dev->driver ||
	    strcmp(intf_dev->driver->name, "uvcvideo") != 0)
		return false;

	intf = to_usb_interface(intf_dev);
	udev = interface_to_usbdev(intf);

	if (le16_to_cpu(udev->descriptor.idVendor) != MIXER_DEFAULT_VID) {
		pr_info("proxy_mixer: ignoring USB device with VID=0x%04X\n",
			    le16_to_cpu(udev->descriptor.idVendor));
		return false;
	}

	pid = le16_to_cpu(udev->descriptor.idProduct);
	if (pid != MIXER_DEFAULT_PID_P && pid != MIXER_DEFAULT_PID_S) {
		pr_info("proxy_mixer: ignoring USB device with PID=0x%04X\n",
			    pid);
		return false;
	}

	return true;
}

static int mixer_class_add_dev(struct device *dev)
{
	struct proxy_mixer *mixer = g_mixer;
	struct video_device *vdev = to_video_device(dev);
	struct mixer_disc_work *dw;

	if (!vdev || vdev == mixer->vdev)
		return 0;

	if (!mixer_match_device(vdev))
		return 0;

	dw = kzalloc(sizeof(*dw), GFP_ATOMIC);
	if (!dw)
		return -ENOMEM;

	/* Hold reference until workqueue processes this device */
	get_device(&vdev->dev);
	INIT_WORK(&dw->work, mixer_disc_work_fn);
	dw->mixer = mixer;
	dw->vdev = vdev;
	dw->is_add = true;

	queue_work(mixer->disc_wq, &dw->work);
	return 0;
}

static void mixer_class_remove_dev(struct device *dev)
{
	struct proxy_mixer *mixer = g_mixer;
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

	/* Hold reference until workqueue processes this device */
	get_device(&vdev->dev);
	INIT_WORK(&dw->work, mixer_disc_work_fn);
	dw->mixer = mixer;
	dw->vdev = vdev;
	dw->is_add = false;

	queue_work(mixer->disc_wq, &dw->work);
}

static int mixer_register_class_intf(struct proxy_mixer *mixer,
				     const struct class *vcls)
{
	mixer->class_intf.class = vcls;
	mixer->class_intf.add_dev = mixer_class_add_dev;
	mixer->class_intf.remove_dev = mixer_class_remove_dev;

	return class_interface_register(&mixer->class_intf);
}

static void mixer_unregister_class_intf(struct proxy_mixer *mixer)
{
	class_interface_unregister(&mixer->class_intf);
}

/* ------------------------------------------------------------------ */
/**
 * SRC UVC streaming control
 */
static int mixer_uvc_check_slots(struct proxy_mixer *mixer)
{
	pr_info("proxy_mixer: checking source slots...\n");
	mutex_lock(&mixer->src_disc_lock);
	if (mixer->src_ready_count < MIXER_NUM_SOURCES) {
		pr_err("proxy_mixer: not all sources ready: %d/%d\n",
		       mixer->src_ready_count, MIXER_NUM_SOURCES);
		mutex_unlock(&mixer->src_disc_lock);
		return -ENODEV;
	}
	mutex_unlock(&mixer->src_disc_lock);
	pr_info("proxy_mixer: all source slots are filled, ready to start streaming\n");
	return 0;
}

static int mixer_uvc_negotiate_src_fmt(struct proxy_mixer *mixer, int slot)
{
	struct mixer_slot *src = &mixer->src[slot];
	int ret;

	src->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	src->fmt.fmt.pix.width = mixer->src_width;
	src->fmt.fmt.pix.height = mixer->src_height;
	src->fmt.fmt.pix.pixelformat = mixer->out_pixfmt;
	src->fmt.fmt.pix.field = V4L2_FIELD_NONE;

	ret = MIXER_CALL_OP(src, vidioc_s_fmt_vid_cap, src->filp, src->fh,
			    &src->fmt);
	if (ret) {
		pr_err("proxy_mixer: vidioc_s_fmt_vid_cap failed for slot%d: %d\n",
		       slot, ret);
		return ret;
	}

	ret = MIXER_CALL_OP(src, vidioc_g_fmt_vid_cap, src->filp, src->fh,
			    &src->fmt);
	if (ret) {
		pr_err("proxy_mixer: vidioc_g_fmt_vid_cap failed for slot%d: %d\n",
		       slot, ret);
		return ret;
	}

	pr_info("proxy_mixer: src%d format negotiated: %ux%u, fmt=0x%X\n", slot,
		src->fmt.fmt.pix.width, src->fmt.fmt.pix.height,
		src->fmt.fmt.pix.pixelformat);

	return 0;
}

/**
 * mixer_uvc_negotiate_src_fps - set the frame interval on a UVC source.
 *
 * Called after s_fmt so the UVC driver knows the active resolution.
 * Uses mixer->fps_num / fps_den chosen by userspace via s_parm.
 */
static int mixer_uvc_negotiate_src_fps(struct proxy_mixer *mixer, int slot)
{
	struct mixer_slot *src = &mixer->src[slot];
	struct v4l2_streamparm parm = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	};
	int ret;

	parm.parm.capture.timeperframe.numerator = mixer->fps_num;
	parm.parm.capture.timeperframe.denominator = mixer->fps_den;

	ret = MIXER_CALL_OP(src, vidioc_s_parm, src->filp, src->fh, &parm);
	if (ret) {
		/* Not fatal — some UVC devices don't support s_parm */
		pr_info("proxy_mixer: src%d s_parm failed: %d (non-fatal)\n",
			slot, ret);
		return 0;
	}

	pr_info("proxy_mixer: src%d fps negotiated: %u/%u\n", slot,
		parm.parm.capture.timeperframe.denominator,
		parm.parm.capture.timeperframe.numerator);
	return 0;
}

static int mixer_uvc_reqbufs_mmap(struct proxy_mixer *mixer, int idx, unsigned int count)
{
	struct mixer_slot *src = &mixer->src[idx];
	struct v4l2_requestbuffers req = {
		.count = count,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	unsigned int i;
	int ret;

	ret = MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
	if (ret) {
		pr_err("proxy_mixer: vidioc_reqbufs failed for slot%d: %d\n",
		       idx, ret);
		return ret;
	}
	if (req.count < 2) {
		pr_err("proxy_mixer: slot%d only has %d buffers\n", idx,
		       req.count);
		return -ENOMEM;
	}

	for (i = 0; i < req.count; i++) {
		struct v4l2_buffer buf = {
			.index = i,
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory = V4L2_MEMORY_MMAP,
		};
		ret = MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
		if (ret) {
			pr_err("proxy_mixer: src%d QBUF[%u] failed: %d\n", idx,
			       i, ret);
			return ret;
		}
	}
	return 0;
}

/* REQBUFS for DMABUF mode (buffer slots only, no physical memory) */
static int mixer_uvc_reqbufs_dmabuf(struct proxy_mixer *mixer, int idx,
				    unsigned int count)
{
	struct mixer_slot *src = &mixer->src[idx];
	struct v4l2_requestbuffers req = {
		.count = count,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	ret = MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
	if (ret)
		return ret;
	if (req.count < count) {
		pr_warn("proxy_mixer: src%d REQBUFS(DMABUF) got %u/%u\n",
			idx, req.count, count);
		if (req.count < 2)
			return -ENOMEM;
	}
	return 0;
}

static int mixer_uvc_streamon(struct proxy_mixer *mixer, int idx)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int ret = MIXER_CALL_OP(&mixer->src[idx], vidioc_streamon,
				mixer->src[idx].filp, mixer->src[idx].fh, type);
	if (ret)
		pr_err("proxy_mixer: src%d STREAMON failed: %d\n", idx, ret);
	return ret;
}

/**
 * mixer_uvc_streamoff_only - STREAMOFF without freeing buffers.
 * Used in the stop path to unblock kthread's DQBUF before kthread_stop.
 */
static void mixer_uvc_streamoff_only(struct proxy_mixer *mixer, int idx)
{
	struct mixer_slot *src = &mixer->src[idx];
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev)
		return;
	MIXER_CALL_OP(src, vidioc_streamoff, src->filp, src->fh, type);
}

/**
 * mixer_uvc_free_bufs - REQBUFS(0) to free UVC buffers.
 * Must be called AFTER all held page references are released.
 */
static void mixer_uvc_free_bufs(struct proxy_mixer *mixer, int idx)
{
	struct mixer_slot *src = &mixer->src[idx];
	u32 mem = mixer->use_dmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
	struct v4l2_requestbuffers req = {
		.count = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = mem,
	};

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev)
		return;
	MIXER_CALL_OP(src, vidioc_reqbufs, src->filp, src->fh, &req);
}

/* Forward declaration */
static int mixer_thread_fn(void *data);

static int mixer_uvc_start(struct proxy_mixer *mixer)
{
	unsigned int out_count;
	unsigned int half_size;
	unsigned int i;
	int j, ret;

	/* Negotiate format and request buffers for each source */
	mutex_lock(&mixer->uvc_ctrl_lock);

	/* Verify all sources are ready */
	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		if (IS_ERR_OR_NULL(mixer->src[i].filp) || !mixer->src[i].vdev) {
			pr_err("proxy_mixer: src%d is not ready (filp=%p, vdev=%p)\n",
			       i, mixer->src[i].filp, mixer->src[i].vdev);
			mutex_unlock(&mixer->uvc_ctrl_lock);
			return -ENODEV;
		}
	}

    /* Negotiate format and framerate */
	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		ret = mixer_uvc_negotiate_src_fmt(mixer, i);
		if (ret) {
			pr_err("proxy_mixer: failed to negotiate format for src%d\n", i);
			goto err_out;
		}
		mixer_uvc_negotiate_src_fps(mixer, i);
	}

    /* Determine DMABUF mode at runtime */
	mixer->use_dmabuf = zerocopy && mixer_dmabuf_check_support(mixer);

	/* Count queued output buffers */
	out_count = vb2_get_num_buffers(&mixer->out_q.vbq);
	if (out_count > MIXER_MAX_OUT_BUFS)
		out_count = MIXER_MAX_OUT_BUFS;

	half_size = mixer_image_size(mixer->out_width, mixer->out_height,
				     mixer->out_pixfmt) / 2;

	if (mixer->use_dmabuf) {
		pr_info("proxy_mixer: using DMABUF feed-through (out_bufs=%u half=%u)\n",
			out_count, half_size);

		/* REQBUFS(DMABUF) for each UVC source */
		for (i = 0; i < MIXER_NUM_SOURCES; i++) {
			ret = mixer_uvc_reqbufs_dmabuf(mixer, i, out_count);
			if (ret)
				goto err_out;
		}

		/* Create dma_bufs for all output buffers */
		mixer->num_dmabuf_slots = out_count;
		for (i = 0; i < out_count; i++) {
			struct vb2_buffer *vb =
				vb2_get_buffer(&mixer->out_q.vbq, i);
			if (!vb) {
				ret = -EINVAL;
				goto err_dmabuf;
			}
			mixer->out_vb_map[i] = vb;

			ret = mixer_dmabuf_setup_slot(mixer,
						      &mixer->dma_slots[i],
						      vb, half_size);
			if (ret)
				goto err_dmabuf;
		}

        /*
		 * Pre-queue all output buffers to UVC sources.
		 * Take them from irqqueue (queued by userspace) and submit.
		 */
		{
			unsigned long flags;
			struct mixer_buffer *mbuf;

			spin_lock_irqsave(&mixer->out_q.irqlock, flags);
			while (!list_empty(&mixer->out_q.irqqueue)) {
				mbuf = list_first_entry(&mixer->out_q.irqqueue,
							struct mixer_buffer,
							queue);
				list_del(&mbuf->queue);
				spin_unlock_irqrestore(&mixer->out_q.irqlock,
						       flags);

				i = mbuf->buf.vb2_buf.index;
				for (j = 0; j < MIXER_NUM_SOURCES; j++) {
					ret = mixer_dmabuf_qbuf_src(
						mixer, j, i,
						mixer->dma_slots[i].dbuf[j]);
					if (ret)
						goto err_dmabuf;
				}

				spin_lock_irqsave(&mixer->out_q.irqlock,
						  flags);
			}
			spin_unlock_irqrestore(&mixer->out_q.irqlock, flags);
		}
	} else {
		pr_info("proxy_mixer: using memcpy fallback\n");

		for (i = 0; i < MIXER_NUM_SOURCES; i++) {
			ret = mixer_uvc_reqbufs_mmap(mixer, i,
						     MIXER_OUT_NUM_BUFS);
			if (ret)
				goto err_out;
		}
	}

	/* STREAMON each source */
	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		ret = mixer_uvc_streamon(mixer, i);
		if (ret) {
			pr_err("proxy_mixer: failed to stream on src%d\n", i);
			goto err_streamoff;
		}
	}

	/* Start kthread */
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
	for (i = 0; i < MIXER_NUM_SOURCES; i++)
		mixer_uvc_streamoff_only(mixer, i);
err_dmabuf:
	if (mixer->use_dmabuf)
		mixer_dmabuf_teardown_all(mixer);
	for (i = 0; i < MIXER_NUM_SOURCES; i++)
		mixer_uvc_free_bufs(mixer, i);
err_out:
	mutex_unlock(&mixer->uvc_ctrl_lock);
	return ret;
}

static void mixer_uvc_stop(struct proxy_mixer *mixer)
{
	int i;

	if (!atomic_cmpxchg(&mixer->streaming, 1, 0))
		return;

	pr_info("proxy_mixer: stopping pipeline\n");

	/* Phase 1: STREAMOFF — unblocks kthread's blocking DQBUF */
	for (i = 0; i < MIXER_NUM_SOURCES; i++)
		mixer_uvc_streamoff_only(mixer, i);

	/* Phase 2: wake and stop kthread */
	wake_up_all(&mixer->out_q.buf_wq);
	if (mixer->mixer_task) {
		kthread_stop(mixer->mixer_task);
		mixer->mixer_task = NULL;
	}

	/*
	 * Phase 3: REQBUFS(0) — releases UVC buffer slots.
	 * For DMABUF mode: vb2 detaches our dma_bufs (dma_buf_put).
	 */
	for (i = 0; i < MIXER_NUM_SOURCES; i++)
		mixer_uvc_free_bufs(mixer, i);

	/*
	 * Phase 4: destroy our dma_bufs.
	 * Safe now — UVC dropped its references in Phase 3.
	 */
	if (mixer->use_dmabuf)
		mixer_dmabuf_teardown_all(mixer);

	pr_info("proxy_mixer: pipeline stopped\n");
}

/* DQBUF for MMAP mode — full: saves cur_vb, timestamp */
static int mixer_dqbuf_src_mmap(struct proxy_mixer *mixer, int slot)
{
	struct mixer_slot *src = &mixer->src[slot];
	struct v4l2_buffer buf = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	int ret;

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev || !src->vbq) {
		pr_err("proxy_mixer: src%d invalid state (filp=%p, vdev=%p, vbq=%p)\n",
		       slot, src->filp, src->vdev, src->vbq);
		return -ENODEV;
	}

	ret = MIXER_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
	if (ret == -EINVAL && !atomic_read(&mixer->streaming)) {
		return ret;
	} else if (ret) {
		pr_err("proxy_mixer: src%d DQBUF failed: %d\n", slot, ret);
		return ret;
	}

	if (buf.index >= src->vbq->max_num_buffers) {
		pr_err("proxy_mixer: src%d DQBUF got invalid buffer index %d\n",
		       slot, buf.index);
		return -EFAULT;
	}

	src->cur_vb = vb2_get_buffer(src->vbq, buf.index);
	if (!src->cur_vb) {
		pr_err("proxy_mixer: src%d DQBUF got invalid buffer index %d\n",
		       slot, buf.index);
		return -EFAULT;
	}
	src->cur_ts = ns_to_ktime((u64)buf.timestamp.tv_sec * NSEC_PER_SEC +
				  (u64)buf.timestamp.tv_usec * NSEC_PER_USEC);
	src->buf_ready = true;

	return 0;
}

static int mixer_dqbuf_src_dmabuf(struct proxy_mixer *mixer, int slot,
                                  unsigned int *out_index)
{
	struct mixer_slot *src = &mixer->src[slot];
	struct v4l2_buffer buf = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	int ret;

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev) {
		pr_info("proxy_mixer: src%d dqbuf skipped (invalid state)\n",
			slot);
		return -ENODEV;
	}

	ret = MIXER_CALL_OP(src, vidioc_dqbuf, src->filp, src->fh, &buf);
	if (ret) {
        pr_err("proxy_mixer: src%d DQBUF failed: %d\n", slot, ret);
        return ret;
    }

    *out_index = buf.index;
	return 0;
}

static void mixer_qbuf_src_mmap(struct proxy_mixer *mixer, int slot)
{
	struct mixer_slot *src = &mixer->src[slot];
	struct v4l2_buffer buf = {
		.index = src->cur_vb->index,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};

	if (IS_ERR_OR_NULL(src->filp) || !src->vdev || !src->cur_vb)
		return;

	MIXER_CALL_OP(src, vidioc_qbuf, src->filp, src->fh, &buf);
	src->cur_vb = NULL;
	src->buf_ready = false;
}

/* Memcpy */
static void mixer_stitch_memcpy(u8 *dst, const u8 *src0, const u8 *src1,
				  u64 raw_size)
{
	memcpy(dst, src0, raw_size);
	memcpy(dst + raw_size, src1, raw_size);
}

static int mixer_thread_fn(void *data)
{
	struct proxy_mixer *mixer = data;
    const bool dmabuf_mode = mixer->use_dmabuf;
	struct mixer_buffer *mbuf;
	struct vb2_buffer *out_vb;
	unsigned long flags;
	u64 half_size;
	u32 pixfmt;
    u32 out_size;
	int ret;

	sched_set_fifo(current);

	mutex_lock(&mixer->lock);
	pixfmt = mixer->out_pixfmt;
    out_size = mixer_image_size(mixer->out_width, mixer->out_height, pixfmt);
	mutex_unlock(&mixer->lock);
    half_size = (u64)out_size / 2;

	if (dmabuf_mode) {
		/*
		 * ═══════════════════════════════════════════════
		 * DMABUF feed-through path
		 *
		 * Output buffer pages are exported as dma_buf and
		 * pre-queued to UVC sources.  xHCI DMA writes
		 * directly to output pages.  Kthread just waits
		 * for DMA completion, signals done, re-queues.
		 * ═══════════════════════════════════════════════
		 */
		while (!kthread_should_stop()) {
			unsigned int idx0, idx1;
			int i;

			/*
			 * Phase 1: re-submit any output buffers that
			 * userspace has re-QBUF'd (added to irqqueue
			 * by buf_queue).
			 */
			spin_lock_irqsave(&mixer->out_q.irqlock, flags);
			while (!list_empty(&mixer->out_q.irqqueue)) {
				mbuf = list_first_entry(
					&mixer->out_q.irqqueue,
					struct mixer_buffer, queue);
				list_del(&mbuf->queue);
				spin_unlock_irqrestore(
					&mixer->out_q.irqlock, flags);

				i = mbuf->buf.vb2_buf.index;
				for (int s = 0; s < MIXER_NUM_SOURCES; s++) {
					mixer_dmabuf_qbuf_src(
						mixer, s, i,
						mixer->dma_slots[i].dbuf[s]);
				}

				spin_lock_irqsave(&mixer->out_q.irqlock,
						  flags);
			}
			spin_unlock_irqrestore(&mixer->out_q.irqlock, flags);

			if (!atomic_read(&mixer->streaming))
				break;

			/*
			 * Phase 2: DQBUF from both sources (blocking).
			 * Both complete the same output buffer (same HC,
			 * same frame clock, FIFO order).
			 */
			ret = mixer_dqbuf_src_dmabuf(mixer, 0, &idx0);
			if (ret) {
				if (!atomic_read(&mixer->streaming))
					break;
				if (ret == -ENODEV || ret == -EIO)
					break;
				continue;
			}

			ret = mixer_dqbuf_src_dmabuf(mixer, 1, &idx1);
			if (ret) {
				/* Re-QBUF src0 */
				mixer_dmabuf_qbuf_src(
					mixer, 0, idx0,
					mixer->dma_slots[idx0].dbuf[0]);
				if (!atomic_read(&mixer->streaming))
					break;
				if (ret == -ENODEV || ret == -EIO)
					break;
				continue;
			}

			if (idx0 != idx1) {
				pr_warn("proxy_mixer: DQBUF index mismatch %u vs %u\n",
					idx0, idx1);
				/* Re-QBUF both and retry */
				mixer_dmabuf_qbuf_src(
					mixer, 0, idx0,
					mixer->dma_slots[idx0].dbuf[0]);
				mixer_dmabuf_qbuf_src(
					mixer, 1, idx1,
					mixer->dma_slots[idx1].dbuf[1]);
				continue;
			}

			/*
			 * Phase 3: output buffer idx0 is filled by DMA.
			 * Set payload + timestamp and signal done.
			 */
			if (idx0 >= mixer->num_dmabuf_slots) {
				pr_err("proxy_mixer: DQBUF index %u out of range\n",
				       idx0);
				break;
			}

			out_vb = mixer->out_vb_map[idx0];
			if (!out_vb) {
				pr_err("proxy_mixer: out_vb_map[%u] NULL\n",
				       idx0);
				break;
			}

			vb2_set_plane_payload(out_vb, 0, out_size);
			out_vb->timestamp = ktime_get_ns();
			vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);

			/*
			 * Don't re-QBUF to UVC here.  Wait for
			 * userspace to re-QBUF the output buffer
			 * (buf_queue → Phase 1 in next iteration).
			 */
		}
	} else {
		/*
		 * ═══════════════════════════════════════════════
		 * Memcpy fallback path (original behavior)
		 * ═══════════════════════════════════════════════
		 */
		while (!kthread_should_stop()) {
			void *src0_va, *src1_va, *dst;

			ret = wait_event_interruptible(
				mixer->out_q.buf_wq,
				!list_empty(&mixer->out_q.irqqueue) ||
				kthread_should_stop() ||
				!atomic_read(&mixer->streaming));

			if (kthread_should_stop() ||
			    !atomic_read(&mixer->streaming))
				break;
			if (ret == -ERESTARTSYS)
				continue;

			spin_lock_irqsave(&mixer->out_q.irqlock, flags);
			if (list_empty(&mixer->out_q.irqqueue)) {
				spin_unlock_irqrestore(
					&mixer->out_q.irqlock, flags);
				continue;
			}
			mbuf = list_first_entry(&mixer->out_q.irqqueue,
						struct mixer_buffer, queue);
			list_del(&mbuf->queue);
			spin_unlock_irqrestore(&mixer->out_q.irqlock, flags);

			out_vb = &mbuf->buf.vb2_buf;

			if (!atomic_read(&mixer->streaming))
				goto return_buf;

			/* DQBUF from UVC sources */
			ret = mixer_dqbuf_src_mmap(mixer, 0);
			if (ret) {
				if (!atomic_read(&mixer->streaming))
					goto return_buf;
				if (ret == -ENODEV || ret == -EIO) {
					vb2_buffer_done(out_vb,
							VB2_BUF_STATE_ERROR);
					break;
				}
				spin_lock_irqsave(&mixer->out_q.irqlock,
						  flags);
				list_add_tail(&mbuf->queue,
					      &mixer->out_q.irqqueue);
				spin_unlock_irqrestore(
					&mixer->out_q.irqlock, flags);
				continue;
			}

			if (!atomic_read(&mixer->streaming)) {
				mixer_qbuf_src_mmap(mixer, 0);
				goto return_buf;
			}

			ret = mixer_dqbuf_src_mmap(mixer, 1);
			if (ret) {
				mixer_qbuf_src_mmap(mixer, 0);
				if (!atomic_read(&mixer->streaming))
					goto return_buf;
				if (ret == -ENODEV || ret == -EIO) {
					vb2_buffer_done(out_vb,
							VB2_BUF_STATE_ERROR);
					break;
				}
				spin_lock_irqsave(&mixer->out_q.irqlock,
						  flags);
				list_add_tail(&mbuf->queue,
					      &mixer->out_q.irqqueue);
				spin_unlock_irqrestore(
					&mixer->out_q.irqlock, flags);
				continue;
			}

			src0_va = vb2_plane_vaddr(mixer->src[0].cur_vb, 0);
			src1_va = vb2_plane_vaddr(mixer->src[1].cur_vb, 0);
			dst = vb2_plane_vaddr(out_vb, 0);

			if (!dst || !src0_va || !src1_va) {
				mixer_qbuf_src_mmap(mixer, 0);
				mixer_qbuf_src_mmap(mixer, 1);
				vb2_buffer_done(out_vb, VB2_BUF_STATE_ERROR);
				continue;
			}

			mixer_stitch_memcpy(dst, src0_va, src1_va, half_size);

			mixer_qbuf_src_mmap(mixer, 0);
			mixer_qbuf_src_mmap(mixer, 1);

			vb2_set_plane_payload(out_vb, 0, out_size);
			out_vb->timestamp = ktime_get_ns();
			vb2_buffer_done(out_vb, VB2_BUF_STATE_DONE);
		}
	}

	pr_info("proxy_mixer: kthread exiting\n");
	return 0;

return_buf:
	/*
     * Shutdown path: return the output buffer to irqqueue so that
     * stop_streaming → mixer_video_queue_return_buffers can finalize it.
     * Do NOT call vb2_buffer_done here — stop_streaming handles that.
     */
	spin_lock_irqsave(&mixer->out_q.irqlock, flags);
	list_add(&mbuf->queue, &mixer->out_q.irqqueue);
	spin_unlock_irqrestore(&mixer->out_q.irqlock, flags);
	pr_info("proxy_mixer: kthread shutdown — returned output buffer to queue\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/**
 * vb2 queue operations
 */
static void mixer_video_queue_return_buffers(struct mixer_video_queue *q,
					     enum vb2_buffer_state state)
{
	while (!list_empty(&q->irqqueue)) {
		struct mixer_buffer *mbuf = list_first_entry(
			&q->irqqueue, struct mixer_buffer, queue);
		list_del(&mbuf->queue);
		vb2_buffer_done(&mbuf->buf.vb2_buf, state);
	}
}

static int mixer_queue_setup(struct vb2_queue *vbq, unsigned int *num_buffers,
			     unsigned int *num_planes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	struct mixer_video_queue *q = vb2_get_drv_priv(vbq);
	struct proxy_mixer *mixer = container_of(q, struct proxy_mixer, out_q);
	unsigned int size;

	mutex_lock(&mixer->lock);
	size = mixer_image_size(mixer->out_width, mixer->out_height,
				mixer->out_pixfmt);
	mutex_unlock(&mixer->lock);

	pr_debug("proxy_mixer: queue_setup (nplanes=%d, nbuffers=%d)\n",
		 *num_planes, *num_buffers);
	if (*num_planes)
		return sizes[0] < size ? -EINVAL : 0;

	*num_planes = 1;
	sizes[0] = size;
	*num_buffers = clamp_t(unsigned int, *num_buffers, 2, MIXER_MAX_OUT_BUFS);

	pr_info("proxy_mixer: queue_setup size=%u buffers=%u\n",
		size, *num_buffers);
	return 0;
}

static int mixer_buf_prepare(struct vb2_buffer *vb)
{
	struct mixer_video_queue *q = vb2_get_drv_priv(vb->vb2_queue);
	struct proxy_mixer *mixer = container_of(q, struct proxy_mixer, out_q);
	unsigned int size;

	mutex_lock(&mixer->lock);
	size = mixer_image_size(mixer->out_width, mixer->out_height,
				mixer->out_pixfmt);
	mutex_unlock(&mixer->lock);

	// pr_debug("proxy_mixer: buf_prepare idx=%d, size requirement=%u\n",
	// 	 vb->index, size);
	// if (vb->index >= q->vbq.max_num_buffers) {
	// 	pr_err("proxy_mixer: buffer index %d out of range\n",
	// 	       vb->index);
	// 	return -EINVAL;
	// }
	if (vb2_plane_size(vb, 0) < size) {
		pr_err("proxy_mixer: buffer plane size < required %d\n", size);
		return -EINVAL;
	}
    
	return 0;
}

static void mixer_buf_queue(struct vb2_buffer *vb)
{
	struct mixer_video_queue *q = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct mixer_buffer *mbuf =
		container_of(vbuf, struct mixer_buffer, buf);
	unsigned long flags;

	pr_debug("proxy_mixer: buf_queue idx=%d\n", vb->index);
	// mbuf->state = BUF_STATE_QUEUED;

	spin_lock_irqsave(&q->irqlock, flags);
	list_add_tail(&mbuf->queue, &q->irqqueue);
	spin_unlock_irqrestore(&q->irqlock, flags);

	wake_up_interruptible(&q->buf_wq);
}

static int mixer_start_streaming(struct vb2_queue *vbq, unsigned int count)
{
	struct mixer_video_queue *q = vb2_get_drv_priv(vbq);
	struct proxy_mixer *mixer = container_of(q, struct proxy_mixer, out_q);
	unsigned long flags;
	int ret;

	pr_info("proxy_mixer: start_streaming called with %u buffers\n", count);
	lockdep_assert_irqs_enabled();

	ret = mixer_uvc_start(mixer);
	if (ret) {		
		pr_err("proxy_mixer: uvc_start failed: %d, returning buffers\n", ret);
        spin_lock_irqsave(&q->irqlock, flags);
        mixer_video_queue_return_buffers(q, VB2_BUF_STATE_QUEUED);
        spin_unlock_irqrestore(&q->irqlock, flags);
        return ret;
	}

	pr_info("proxy_mixer: streaming started successfully\n");
	return 0;
}

static void mixer_stop_streaming(struct vb2_queue *vbq)
{
	struct mixer_video_queue *q = vb2_get_drv_priv(vbq);
	struct proxy_mixer *mixer = container_of(q, struct proxy_mixer, out_q);
	unsigned long flags;
	int i;

	pr_info("proxy_mixer: stop_streaming called\n");
	lockdep_assert_irqs_enabled();

	mixer_uvc_stop(mixer);

	spin_lock_irqsave(&q->irqlock, flags);
	mixer_video_queue_return_buffers(q, VB2_BUF_STATE_ERROR);
	spin_unlock_irqrestore(&q->irqlock, flags);

	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		mixer->src[i].cur_vb = NULL;
		mixer->src[i].buf_ready = false;
	}
	pr_info("proxy_mixer: stop_streaming completed\n");
}

const struct vb2_ops mixer_vb2_ops = {
	// clang-format off
    .queue_setup     = mixer_queue_setup,
	.buf_prepare     = mixer_buf_prepare,
	.buf_queue       = mixer_buf_queue,
	.start_streaming = mixer_start_streaming,
	.stop_streaming  = mixer_stop_streaming,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	// clang-format on
};

static void mixer_queue_release(struct mixer_video_queue *q)
{
	vb2_queue_release(&q->vbq);
}

static int init_queue(struct mixer_video_queue *q)
{
	int ret;

	mutex_init(&q->mutex);
	spin_lock_init(&q->irqlock);
	init_waitqueue_head(&q->buf_wq);
	INIT_LIST_HEAD(&q->irqqueue);

	q->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->vbq.io_modes = VB2_MMAP | VB2_READ;
	q->vbq.drv_priv = q;
	q->vbq.ops = &mixer_vb2_ops;
	q->vbq.mem_ops = &vb2_vmalloc_memops;
	q->vbq.buf_struct_size = sizeof(struct mixer_buffer);
	q->vbq.lock = &q->mutex;
	q->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(&q->vbq);
	if (ret)
		return ret;

	return 0;
}

/* ------------------------------------------------------------------ */
/**
 * vidioc_streamon
 * Check if sources are ready
 */
static int mixer_vidioc_streamon(struct file *file, void *fh,
				 enum v4l2_buf_type type)
{
	struct proxy_mixer *mixer = video_drvdata(file);
	int ret;

	pr_info("proxy_mixer: vidioc_streamon called (type=%d)\n", type);
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("proxy_mixer: invalid buffer type %d\n", type);
		return -EINVAL;
	}

	ret = mixer_uvc_check_slots(mixer);
	if (ret) {
		pr_err("proxy_mixer: slot check failed: %d\n", ret);
		return ret;
	}

	return vb2_ioctl_streamon(file, fh, type);
}

/**
 * vidioc_streamoff
 * Stop kthread and source streaming
 * then call vb2_ioctl_streamoff to complete the stream off process
 */
static int mixer_vidioc_streamoff(struct file *file, void *fh,
				  enum v4l2_buf_type type)
{
	pr_info("proxy_mixer: vidioc_streamoff called (type=%d)\n", type);
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("proxy_mixer: invalid buffer type %d\n", type);
		return -EINVAL;
	}

	return vb2_ioctl_streamoff(file, fh, type);
}

static int mixer_vidioc_querycap(struct file *file, void *fh,
				 struct v4l2_capability *cap)
{
	strscpy(cap->driver, "proxy_mixer", sizeof(cap->driver));
	strscpy(cap->card, "Proxy Video Mixer", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:proxy_mixer", sizeof(cap->bus_info));
	//cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static const u32 mixer_formats[] = {
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_NV12,
};
#define MIXER_NUM_FORMATS ARRAY_SIZE(mixer_formats)

static int mixer_vidioc_enum_fmt(struct file *file, void *fh,
				 struct v4l2_fmtdesc *fmtdesc)
{
	if (fmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (fmtdesc->index >= MIXER_NUM_FORMATS)
		return -EINVAL;

	fmtdesc->pixelformat = mixer_formats[fmtdesc->index];
	return 0;
}

static void fill_fmt(struct proxy_mixer *mixer, struct v4l2_format *f)
{
	mutex_lock(&mixer->lock);
	f->fmt.pix.width = mixer->out_width;
	f->fmt.pix.height = mixer->out_height;
	f->fmt.pix.pixelformat = mixer->out_pixfmt;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline =
		mixer_bytesperline(mixer->out_width, mixer->out_pixfmt);
	f->fmt.pix.sizeimage = 
        mixer_image_size(mixer->out_width, mixer->out_height, mixer->out_pixfmt);
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	mutex_unlock(&mixer->lock);
}

static int mixer_vidioc_g_fmt(struct file *file, void *fh,
			      struct v4l2_format *fmt)
{
	struct proxy_mixer *mixer = video_drvdata(file);

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fill_fmt(mixer, fmt);
	return 0;
}

static int mixer_vidioc_s_fmt(struct file *file, void *fh,
			      struct v4l2_format *fmt)
{
	struct proxy_mixer *mixer = video_drvdata(file);

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/*
	 * Accept YUYV or NV12.  Anything else falls back to YUYV.
	 */
	mutex_lock(&mixer->lock);
	mixer->out_pixfmt = mixer_pixfmt_valid(fmt->fmt.pix.pixelformat) ?
				    fmt->fmt.pix.pixelformat :
				    V4L2_PIX_FMT_YUYV;

	if (fmt->fmt.pix.width > 0 && fmt->fmt.pix.height >= 2) {
		mixer->out_width = fmt->fmt.pix.width;
		mixer->out_height = fmt->fmt.pix.height & ~1u; /* round down to even */
		mixer->src_width = mixer->out_width;
		mixer->src_height = mixer->out_height / 2;
		pr_info("proxy_mixer: s_fmt accepted %ux%u fmt=0x%X (src %ux%u)\n",
			mixer->out_width, mixer->out_height, mixer->out_pixfmt,
			mixer->src_width, mixer->src_height);
	}
	mutex_unlock(&mixer->lock);

	/* Return the (possibly adjusted) format back to userspace */
	fill_fmt(mixer, fmt);
	return 0;
}

static int mixer_vidioc_try_fmt(struct file *file, void *fh,
				struct v4l2_format *fmt)
{
	struct proxy_mixer *mixer = video_drvdata(file);
	u32 pixfmt;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pixfmt = mixer_pixfmt_valid(fmt->fmt.pix.pixelformat) ?
			 fmt->fmt.pix.pixelformat :
			 V4L2_PIX_FMT_YUYV;

	fmt->fmt.pix.pixelformat = pixfmt;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	/* Accept proposed size if valid, otherwise return current */
	if (fmt->fmt.pix.width == 0 || fmt->fmt.pix.height < 2) {
		mutex_lock(&mixer->lock);
		fmt->fmt.pix.width = mixer->out_width;
		fmt->fmt.pix.height = mixer->out_height;
		mutex_unlock(&mixer->lock);
	} else {
		fmt->fmt.pix.height &= ~1u; /* round down to even */
	}

	fmt->fmt.pix.bytesperline =
		mixer_bytesperline(fmt->fmt.pix.width, pixfmt);
	fmt->fmt.pix.sizeimage = 
        mixer_image_size(fmt->fmt.pix.width,
						 fmt->fmt.pix.height, pixfmt);
	return 0;
}

static int mixer_vidioc_s_input(struct file *file, void *fh, unsigned int input)
{
	return input ? -EINVAL : 0;
}

static int mixer_vidioc_g_input(struct file *file, void *fh,
				unsigned int *input)
{
	*input = 0;
	return 0;
}

static int mixer_vidioc_enum_input(struct file *file, void *fh,
				   struct v4l2_input *input)
{
	if (input->index != 0)
		return -EINVAL;

	strscpy(input->name, "Mixed Input", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;
	return 0;
}

static int mixer_vidioc_g_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	struct proxy_mixer *mixer = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(&a->parm.capture, 0, sizeof(a->parm.capture));
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;

	mutex_lock(&mixer->lock);
	a->parm.capture.timeperframe.numerator = mixer->fps_num;
	a->parm.capture.timeperframe.denominator = mixer->fps_den;
	mutex_unlock(&mixer->lock);

	return 0;
}

static int mixer_vidioc_s_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	struct proxy_mixer *mixer = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&mixer->lock);
	if (a->parm.capture.timeperframe.numerator &&
	    a->parm.capture.timeperframe.denominator) {
		mixer->fps_num = a->parm.capture.timeperframe.numerator;
		mixer->fps_den = a->parm.capture.timeperframe.denominator;
		pr_info("proxy_mixer: s_parm fps=%u/%u\n", mixer->fps_den,
			mixer->fps_num);
	}
	mutex_unlock(&mixer->lock);

	/* Read back what we stored */
	return mixer_vidioc_g_parm(file, fh, a);
}

static int mixer_vidioc_enum_framesizes(struct file *file, void *fh,
					struct v4l2_frmsizeenum *fsize)
{
	struct proxy_mixer *mixer = video_drvdata(file);
	struct mixer_slot *src;
	int ret;

	if (!mixer_pixfmt_valid(fsize->pixel_format))
		return -EINVAL;

	mutex_lock(&mixer->src_disc_lock);
	if (mixer->src_ready_count < 1 || IS_ERR_OR_NULL(mixer->src[0].filp) ||
	    !mixer->src[0].vdev) {
		mutex_unlock(&mixer->src_disc_lock);
		return -ENODEV;
	}
	src = &mixer->src[0];

	ret = MIXER_CALL_OP(src, vidioc_enum_framesizes, src->filp, src->fh,
			    fsize);
	mutex_unlock(&mixer->src_disc_lock);

	if (ret)
		return ret;

	/*
     * The UVC driver returns DISCRETE entries.  Double the height
     * to represent the stitched output.
     */
	if (fsize->type == V4L2_FRMSIZE_TYPE_DISCRETE) {
		fsize->discrete.height *= 2;
	} else if (fsize->type == V4L2_FRMSIZE_TYPE_STEPWISE ||
		   fsize->type == V4L2_FRMSIZE_TYPE_CONTINUOUS) {
		fsize->stepwise.min_height *= 2;
		fsize->stepwise.max_height *= 2;
		fsize->stepwise.step_height *= 2;
	}

	return 0;
}

static int mixer_vidioc_enum_frameintervals(struct file *file, void *fh,
					    struct v4l2_frmivalenum *fival)
{
	struct proxy_mixer *mixer = video_drvdata(file);
	struct mixer_slot *src;
	int ret;

	if (!mixer_pixfmt_valid(fival->pixel_format))
		return -EINVAL;

	mutex_lock(&mixer->src_disc_lock);
	if (mixer->src_ready_count < 1 || IS_ERR_OR_NULL(mixer->src[0].filp) ||
	    !mixer->src[0].vdev) {
		mutex_unlock(&mixer->src_disc_lock);
		return -ENODEV;
	}
	src = &mixer->src[0];

	/*
     * OBS passes the mixer's output height (doubled).
     * Halve it to match the source camera's native resolution.
     */
	fival->height /= 2;

	ret = MIXER_CALL_OP(src, vidioc_enum_frameintervals, src->filp, src->fh,
			    fival);

	/* Restore height to mixer output value for userspace */
	fival->height *= 2;

	mutex_unlock(&mixer->src_disc_lock);
	return ret;
}

// static int mixer_fop_open(struct file *file)
// {
//     struct proxy_mixer *mixer = video_drvdata(file);
//     int ret = v4l2_fh_open(file);
//     if (ret)
//         return ret;

//     /* Capture inode address_space for unmap_mapping_range */
//     if (!mixer->mapping)
//         mixer->mapping = file_inode(file)->i_mapping;
//     return 0;
// }

const struct v4l2_ioctl_ops mixer_ioctl_ops = {
	// clang-format off
    .vidioc_querycap             = mixer_vidioc_querycap,
    .vidioc_enum_fmt_vid_cap     = mixer_vidioc_enum_fmt,
    .vidioc_g_fmt_vid_cap        = mixer_vidioc_g_fmt,
    .vidioc_s_fmt_vid_cap        = mixer_vidioc_s_fmt,
    .vidioc_try_fmt_vid_cap      = mixer_vidioc_try_fmt,

    .vidioc_s_input              = mixer_vidioc_s_input,
    .vidioc_g_input              = mixer_vidioc_g_input,
    .vidioc_enum_input           = mixer_vidioc_enum_input,

    .vidioc_g_parm               = mixer_vidioc_g_parm,
    .vidioc_s_parm               = mixer_vidioc_s_parm,

    .vidioc_enum_framesizes      = mixer_vidioc_enum_framesizes,
    .vidioc_enum_frameintervals  = mixer_vidioc_enum_frameintervals,

    .vidioc_reqbufs              = vb2_ioctl_reqbufs,
    .vidioc_querybuf             = vb2_ioctl_querybuf,
    .vidioc_qbuf                 = vb2_ioctl_qbuf,
    .vidioc_dqbuf                = vb2_ioctl_dqbuf,
    .vidioc_expbuf               = vb2_ioctl_expbuf,

    .vidioc_streamon             = mixer_vidioc_streamon,
    .vidioc_streamoff            = mixer_vidioc_streamoff,
	// clang-format on
};

const struct v4l2_file_operations mixer_fops = {
	// clang-format off
    .owner          = THIS_MODULE,
    .open           = v4l2_fh_open,
    .release        = vb2_fop_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap           = vb2_fop_mmap,
    .read           = vb2_fop_read,
    .poll           = vb2_fop_poll,
	// clang-format on
};

/**/
static void init_vdev(struct proxy_mixer *mixer)
{
	struct video_device *vdev = mixer->vdev;

	vdev->fops = &mixer_fops;
	vdev->ioctl_ops = &mixer_ioctl_ops;
	vdev->release = video_device_release;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	strscpy(vdev->name, "Dual Video Mixer", sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_VIDEO;
	vdev->vfl_dir = VFL_DIR_RX;

	/*
     * Use the output queue mutex as the vdev lock.  This serializes
     * all ioctls and — critically — allows STREAMOFF to preempt a
     * blocking DQBUF: vb2's wait_prepare releases this lock while
     * sleeping, so STREAMOFF can acquire it and cancel the queue.
     *
     * Without this, ffplay's SDL event loop cannot interrupt a
     * blocking DQBUF to process the 'q' key, because close() →
     * vb2_fop_release needs vdev->lock to proceed.
     */
	vdev->lock = &mixer->out_q.mutex;

	vdev->queue = &mixer->out_q.vbq;
	vdev->v4l2_dev = &mixer->v4l2_dev;
	video_set_drvdata(vdev, mixer);
}

/* ------------------------------------------------------------------ */
static void mixer_unregister(struct proxy_mixer *mixer)
{
	if (!mixer)
		return;

	if (mixer->vdev) {
		video_set_drvdata(mixer->vdev, NULL);
		video_unregister_device(mixer->vdev);
		mixer->vdev = NULL;
	}

	if (mixer->out_q_initialized) {
		mixer_queue_release(&mixer->out_q);
		mixer->out_q_initialized = false;
	}

	if (mixer->v4l2_dev.dev) {
		v4l2_device_unregister(&mixer->v4l2_dev);
	}
}

/**/
static int mixer_add(void)
{
	struct proxy_mixer *mixer;
	const struct class *vcls;
	int ret;

	pr_info("proxy_mixer: initializing mixer...\n");
	mixer = kzalloc(sizeof(*mixer), GFP_KERNEL);
	if (!mixer) {
		pr_err("proxy_mixer: failed to allocate memory for mixer\n");
		return -ENOMEM;
	}

	mutex_init(&mixer->lock);
	mutex_init(&mixer->src_disc_lock);
	mutex_init(&mixer->uvc_ctrl_lock);
	atomic_set(&mixer->streaming, 0);

	mixer->out_width = MIXER_DEFAULT_OUT_WIDTH;
	mixer->out_height = MIXER_DEFAULT_OUT_HEIGHT;
	mixer->out_pixfmt = V4L2_PIX_FMT_YUYV;
	mixer->src_width = MIXER_DEFAULT_OUT_WIDTH;
	mixer->src_height = MIXER_DEFAULT_OUT_HEIGHT / 2;
	mixer->fps_num = MIXER_DEFAULT_FPS_NUM;
	mixer->fps_den = MIXER_DEFAULT_FPS_DEN;

	mixer->disc_wq = alloc_ordered_workqueue("proxy_mixer_disc", 0);
	if (!mixer->disc_wq) {
		pr_err("proxy_mixer: failed to allocate workqueue\n");
		ret = -ENOMEM;
		goto err_free;
	}
	pr_info("proxy_mixer: workqueue created\n");

	strscpy(mixer->v4l2_dev.name, "proxy_mixer",
		sizeof(mixer->v4l2_dev.name));
	ret = v4l2_device_register(NULL, &mixer->v4l2_dev);
	if (ret) {
		pr_err("proxy_mixer: v4l2_device_register failed: %d\n", ret);
		goto err_wq;
	}

	ret = init_queue(&mixer->out_q);
	if (ret) {
		pr_err("proxy_mixer: init_queue failed: %d\n", ret);
		goto err_v4l2;
	}
	mixer->out_q_initialized = true;
	pr_info("proxy_mixer: output queue initialized\n");

	mixer->vdev = video_device_alloc();
	if (!mixer->vdev) {
		pr_err("proxy_mixer: failed to allocate video device\n");
		goto err_queue;
	}

	init_vdev(mixer);
	pr_info("proxy_mixer: video device initialized\n");

	ret = video_register_device(mixer->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		pr_err("proxy_mixer: video_register_device failed: %d\n", ret);
		goto err_vdev;
	}
	pr_info("proxy_mixer: video device registered as /dev/video%d\n",
		mixer->vdev->num);
	g_mixer = mixer;

	vcls = mixer->vdev->dev.class;
	if (!vcls) {
		pr_err("proxy_mixer: cannot get video_class from vdev\n");
		ret = -ENODEV;
		goto err_intf;
	}
	ret = mixer_register_class_intf(mixer, vcls);
	if (ret) {
		pr_err("proxy_mixer: failed to register class interface: %d\n",
		       ret);
		goto err_intf;
	}
	pr_info("proxy_mixer: class interface registered, monitoring devices...\n");

	pr_info("proxy_mixer: initialization complete\n");
	return 0;

err_intf:
	video_set_drvdata(mixer->vdev, NULL);
	video_unregister_device(mixer->vdev);
err_vdev:
	video_device_release(mixer->vdev);
	mixer->vdev = NULL;
err_queue:
	mixer_queue_release(&mixer->out_q);
	mixer->out_q_initialized = false;
err_v4l2:
	v4l2_device_unregister(&mixer->v4l2_dev);
err_wq:
	destroy_workqueue(mixer->disc_wq);
err_free:
	kfree(mixer);
	return ret;
}

static void mixer_remove(struct proxy_mixer *mixer)
{
	int i;

	pr_info("proxy_mixer: removing mixer...\n");
	if (!mixer) {
		pr_err("proxy_mixer: mixer is NULL\n");
		return;
	}

	/* unregister the class interface */
	pr_info("proxy_mixer: unregistering class interface...\n");
	mixer_unregister_class_intf(mixer);

	pr_info("proxy_mixer: draining and destroying workqueue...\n");
	drain_workqueue(mixer->disc_wq);
	destroy_workqueue(mixer->disc_wq);
	mixer->disc_wq = NULL;

	/* If streaming, stop */
	if (atomic_read(&mixer->streaming)) {
		pr_info("proxy_mixer: stopping active streaming...\n");
		mixer_uvc_stop(mixer);
	}

	/* close all source filp */
	for (i = 0; i < MIXER_NUM_SOURCES; i++) {
		if (!IS_ERR_OR_NULL(mixer->src[i].filp)) {
			pr_info("proxy_mixer: closing src%d...\n", i);
			filp_close(mixer->src[i].filp, NULL);
			mixer->src[i].filp = NULL;
		}
	}

	/* unregister video node */
	pr_info("proxy_mixer: unregistering video device...\n");
	mixer_unregister(mixer);
	kfree(mixer);
	g_mixer = NULL;
	pr_info("proxy_mixer: removed\n");
}

/**/
static int __init proxy_mixer_init(void)
{
	pr_info("proxy_mixer: module loading...\n");
	return mixer_add();
}

static void __exit proxy_mixer_exit(void)
{
	pr_info("proxy_mixer: module unloading...\n");
	mixer_remove(g_mixer);
	pr_info("proxy_mixer: module unloaded\n");
}

module_init(proxy_mixer_init);
module_exit(proxy_mixer_exit);
