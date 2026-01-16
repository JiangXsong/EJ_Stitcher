#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/device.h>
#include <linux/unaligned/packed_struct.h>
#include <linux/usb.h>
#include <linux/usb/uvc.h>
#include <linux/usb/video.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#include <media/videobuf2-core.h>

#define DRIVER_AUTHOR "Xu-Song Chiang"

/* -------------------------------------------------------------------
 * EJCM3 device
 * ------------------------------------------------------------------- */
#define VENDOR_ID      0x1e4e
#define PRODUCT_ID_P   0x7301
#define PRODUCT_ID_S   0x7302

#define BULK_IN_EP      0x89
#define MAX_PKT_SIZE    0x0400
#define MAX_BURST       0x0F
#define BULK_BUF_SIZE   (MAX_PKT_SIZE * MAX_BURST)
#define UVC_MAX_PACKETS 32

#define WIDTH                1920
#define HEIGHT               1080
#define MAX_VIDEO_FRAME_SIZE (WIDTH * HEIGHT * 2)
#define FORMAT               V4L2_PIX_FMT_NV12

#define UVC_URBS           5

#define SHARED_POOL_BUFS   8
#define SHARED_BUF_SIZE   (1920 * 1080 * 2)

#define UVC_CTRL_CONTROL_TIMEOUT	5000
#define UVC_CTRL_STREAMING_TIMEOUT	5000

#define UVC_FMT_FLAG_COMPRESSED		0x00000001
/* -------------------------------------------------------------------
 * DBG printing and logging
 * ------------------------------------------------------------------- */
#define EJCM3_DBG_PROBE         (1 << 0)
#define EJCM3_DBG_DESCR         (1 << 1)
#define EJCM3_DBG_CONTROL       (1 << 2)
#define EJCM3_DBG_FORMAT        (1 << 3)
#define EJCM3_DBG_CAPTURE       (1 << 4)
#define EJCM3_DBG_CALLS         (1 << 5)
#define EJCM3_DBG_FRAME         (1 << 7)
#define EJCM3_DBG_SUSPEND       (1 << 8)
#define EJCM3_DBG_STATUS        (1 << 9)
#define EJCM3_DBG_VIDEO         (1 << 10)
#define EJCM3_DBG_STATS         (1 << 11)
#define EJCM3_DBG_CLOCK         (1 << 12)

#define UVC_WARN_MINMAX         0
#define UVC_WARN_PROBE_DEF      1
#define UVC_WARN_XU_GET_RES     2

unsigned int ejcm3_dbg_param;

#define ejcm3_uvc_dbg(_dev, flag, fmt, ...)				\
do {									\
	if (ejcm3_dbg_param & EJCM3_DBG_##flag)				\
		dev_printk(KERN_DEBUG, &(_dev)->udev->dev, fmt,		\
			   ##__VA_ARGS__);				\
} while (0)

#define ejcm3_uvc_dbg_cont(flag, fmt, ...)				\
do {									\
	if (ejcm3_dbg_param & EJCM3_DBG_##flag)				\
		pr_cont(fmt, ##__VA_ARGS__);				\
} while (0)

#define ejcm3_uvc_warn_once(_dev, warn, fmt, ...)			\
do {									\
	if (!test_and_set_bit(warn, &(_dev)->warnings))			\
		dev_info(&(_dev)->udev->dev, fmt, ##__VA_ARGS__);	\
} while (0)


/* -------------------------------------------------------------------
 * Data structures (buffer & queue)
 * ------------------------------------------------------------------- */
#if 0
struct shared_video_mem {
        void *vaddr;
        dma_addr_t dma;
        unsigned int size;

        struct kref ref;
        struct list_head list;
};

struct shared_video_pool {
        struct mutex lock;

        unsigned int num_mems;
        struct list_head free_list;
        struct list_head used_list;
};
#endif

enum buffer_state {
        UVC_BUF_STATE_IDLE   = 0,
        UVC_BUF_STATE_QUEUED = 1,
        UVC_BUF_STATE_ACTIVE = 2,
        UVC_BUF_STATE_READY  = 3,
        UVC_BUF_STATE_DONE   = 4,
        UVC_BUF_STATE_ERROR  = 5,
};

/* buffer for per video frame */
struct video_buffer {
        struct vb2_v4l2_buffer buf;
        struct list_head queue;

        enum buffer_state state;
        unsigned int error;

        /* points to shared mem pool */
        // struct shared_video_mem *mem;
		void *mem;
        unsigned int length;
        unsigned int byteused;

        struct kref kref;
};

#define UVC_QUEUE_DISCONNECTED (1 << 0)

/* v4l2 queue */
struct video_queue {
        struct vb2_queue queue;
        struct mutex mutex;

        unsigned int flags;
        unsigned int buf_used;

        spinlock_t irqlock;
        struct list_head irqqueue;
};

/* -------------------------------------------------------------------
 * struct urb - URB context management structure
 * ------------------------------------------------------------------- */
struct uvc_copy_op {
        struct video_buffer *buf;
        void *dst;
        const __u8 *src;
        size_t len;
};

struct uvc_urb {
        struct urb *urb;
        struct ejcm3_uvc_streaming *stream;
        char *buffer;
        dma_addr_t dma;
        struct sg_table *sgt;

        unsigned int async_operations;
        struct uvc_copy_op copy_operations[UVC_MAX_PACKETS];
        struct work_struct work;
};

/* -------------------------------------------------------------------
 * Individual stream node
 * ------------------------------------------------------------------- */
struct ejcm3_streaming_header {
	u8 bNumFormats;
	u8 bEndpointAddress;
	u8 bmInfo;
	u8 bTerminalLink;
	u8 bStillCaptureMethod;
	u8 bTriggerSupport;
	u8 bTriggerUsage;
	u8 bControlSize;
	u8 *bmaControls;
};

struct ejcm3_uvc_frame {
	u8 bFrameIndex;
	u8 bmCapabilities;
	u16 wWidth;
	u16 wHeight;
	u32 dwMinBitRate;
	u32 dwMaxBitRate;
	u32 dwMaxVideoFrameBufferSize;
	u32 dwDefaultFrameInterval;
	u8 bFrameIntervalType;
	const u32 *dwFrameInterval;
};

struct ejcm3_uvc_format {
	u8 type;
	u8 index;
	u8 bpp;
	enum v4l2_colorspace colorspace;
	u32 fcc;
	u32 flags;

	unsigned int nframes;
	const struct ejcm3_uvc_frame *frames;
};

struct ejcm3_uvc_streaming {
        struct list_head list;
        struct video_device vdev;
	atomic_t active;

        struct ejcm3_uvc *ejcm3_dev;
        struct usb_interface *intf;
        int intfnum;

	struct ejcm3_streaming_header header;
	enum v4l2_buf_type type;
	struct v4l2_prio_state prio;

	unsigned int nformats;
	const struct ejcm3_uvc_format *formats;

        struct uvc_streaming_control ctrl;
	const struct ejcm3_uvc_format *def_format;
	const struct ejcm3_uvc_format *cur_format;
	const struct ejcm3_uvc_frame *cur_frame;

        struct mutex mutex;

        struct video_queue queue;
	struct workqueue_struct *async_wq;
	void (*decode)(struct uvc_urb *uvc_urb, struct video_buffer);

	struct {
		u8 header[256];
		unsigned int header_size;
		int skip_payload;
		u32 payload_size;
		u32 max_payload_size;
	} bulk;
        
	struct uvc_urb uvc_urb[5];
        unsigned int urb_size;

	u32 sequence;
	u8 last_fid;

        // struct shared_video_pool *pool; //
};

#define for_each_uvc_urb(uvc_urb, ejcm3_uvc_streaming) \
	for ((uvc_urb) = &(ejcm3_uvc_streaming)->uvc_urb[0]; \
	     (uvc_urb) < &(ejcm3_uvc_streaming)->uvc_urb[UVC_URBS]; \
	     ++(uvc_urb))


/* The physical device structure */
struct ejcm3_uvc {
        struct usb_device *udev;
        struct usb_interface *intf;
	unsigned long warnings;
        int intfnum;
        char name[32];

	u8 output_TerminalID;
	u16 output_TerminalType;

	/* Protect users */
	struct mutex lock;
	unsigned int users;
	atomic_t nmappings;

        struct v4l2_device v4l2_dev;
        u16 uvc_version;
        u32 clock_frequency;

	/* Video streaming interface */
        struct list_head streams;
        struct kref kref;

        /* Device manager */
        // struct list_head list;
};

/* -----------------------------------------------------------
 * The stitch node
 * ----------------------------------------------------------- */
struct stitch_stream_node {
        struct video_device vdev;
        struct v4l2_device v4l2_dev; // virtual V4L2 parent
        struct video_queue queue;

        struct ejcm3_uvc_streaming *slot[2];
        struct mutex lock;
};
/* -----------------------------------------------------------
 * Global Manager
 * ----------------------------------------------------------- */
struct driver_dev_manager {
    struct mutex lock;
    struct list_head dev_list;
    int dev_count;
    
    struct stitch_stream_node *stitch_node;
};

static struct driver_dev_manager global_mgr;

/* -----------------------------------------------------------
 * Release source
 * ----------------------------------------------------------- */
static void stream_delete(struct ejcm3_uvc_streaming *stream)
{
	if (stream->async_wq)
		destroy_workqueue(stream->async_wq);

	mutex_destroy(&stream->mutex);

	usb_put_intf(stream->intf);

	kfree(stream->formats);
	kfree(stream->header.bmaControls); //
	kfree(stream);		
}

static struct ejcm3_uvc_streaming *stream_new(struct ejcm3_uvc *dev, struct usb_interface *intf)
{
        struct ejcm3_uvc_streaming *stream;

        stream = kzalloc(sizeof(*stream), GFP_KERNEL);
        if (stream == NULL)
                return NULL;

        mutex_init(&stream->mutex);

        stream->ejcm3_dev = dev;
        stream->intf = usb_get_intf(intf);
        stream->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
	v4l2_prio_init(&stream->prio);
        // stream->pool = dev->pool;
        stream->async_wq =
                alloc_workqueue("ejcm3_uvc_wq",
                                 WQ_UNBOUND | WQ_HIGHPRI, 0);
        if (!stream->async_wq) {
                stream_delete(stream);
                return NULL;
        }

        return stream;
}

static void ejcm3_uvc_delete(struct kref *kref)
{
        struct ejcm3_uvc *dev =
                container_of(kref, struct ejcm3_uvc, kref);
	struct list_head *p, *n;

	usb_put_intf(dev->intf);
	usb_put_dev(dev->udev);

	list_for_each_safe(p, n, &dev->streams) {
		struct ejcm3_uvc_streaming *stream =
			list_entry(p, struct ejcm3_uvc_streaming, list);

		usb_driver_release_interface(&ejcm3_uvc_driver, stream->intf);
		stream_delete(stream);
	}

        kfree(dev);
}

/** ------------------------------------------------------------------
 * Referenced from uvc_queue.c
 * 
 * Return all queued buffers to videobuf2 in the requested state.
 *
 * This function must be called with the queue spinlock held.
 */
static void uvc_queue_return_buffers(struct video_queue *queue,
			       enum buffer_state state)
{
	enum vb2_buffer_state vb2_state = state == UVC_BUF_STATE_ERROR
					? VB2_BUF_STATE_ERROR
					: VB2_BUF_STATE_QUEUED;

	while (!list_empty(&queue->irqqueue)) {
		struct video_buffer *buf = list_first_entry(&queue->irqqueue,
							  struct video_buffer,
							  queue);
		list_del(&buf->queue);
		buf->state = state;
		vb2_buffer_done(&buf->buf.vb2_buf, vb2_state);
	}
}

/* -------------------------------------------------------------------
 * videobuf2 queue operations
 */
static int queue_setup(struct vb2_queue *vq,
                        unsigned int *nbuffers, unsigned int *nplanes,
                        unsigned int sizes[], struct device *alloc_devs[])
{
        if (*nplanes)
		return sizes[0] < MAX_VIDEO_FRAME_SIZE ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = MAX_VIDEO_FRAME_SIZE;
	return 0;
}

static int buf_prepare(struct vb2_buffer *vb)
{
        struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct video_buffer *buf = uvc_vbuf_to_buffer(vbuf);

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		// uvc_dbg(uvc_queue_to_stream(queue)->dev, CAPTURE,
		// 	"[E] Bytes used out of bounds\n");
		return -EINVAL;
	}

	if (unlikely(queue->flags & UVC_QUEUE_DISCONNECTED))
		return -ENODEV;

	buf->state = UVC_BUF_STATE_QUEUED;
	buf->error = 0;
	buf->mem = vb2_plane_vaddr(vb, 0);
	buf->length = vb2_plane_size(vb, 0);
	if (vb->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		buf->byteused = 0;
	else
		buf->byteused = vb2_get_plane_payload(vb, 0);

	return 0;
}

static void buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct video_buffer *buf = uvc_vbuf_to_buffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&queue->irqlock, flags);
	if (likely(!(queue->flags & UVC_QUEUE_DISCONNECTED))) {
		kref_init(&buf->kref);
		list_add_tail(&buf->queue, &queue->irqqueue);
	} else {
		/*
		 * If the device is disconnected return the buffer to userspace
		 * directly. The next QBUF call will fail with -ENODEV.
		 */
		buf->state = UVC_BUF_STATE_ERROR;
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&queue->irqlock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct video_queue *queue = vb2_get_drv_priv(vq);
	struct uvc_streaming *stream = uvc_queue_to_stream(queue);
	int ret;

	lockdep_assert_irqs_enabled();

	queue->buf_used = 0;

	ret = uvc_video_start_streaming(stream); //
	if (ret == 0)
		return 0;

	spin_lock_irq(&queue->irqlock);
	uvc_queue_return_buffers(queue, UVC_BUF_STATE_QUEUED);
	spin_unlock_irq(&queue->irqlock);

	return ret;
}

static void stop_streaming(struct vb2_queue *vq)
{
	struct video_queue *queue = vb2_get_drv_priv(vq);

	lockdep_assert_irqs_enabled();

	if (vq->type != V4L2_BUF_TYPE_META_CAPTURE)
		uvc_video_stop_streaming(uvc_queue_to_stream(queue)); //

	spin_lock_irq(&queue->irqlock);
	uvc_queue_return_buffers(queue, UVC_BUF_STATE_ERROR);
	spin_unlock_irq(&queue->irqlock);
}

static const struct vb2_ops queue_ops = {
        .queue_setup = queue_setup,
        .buf_prepare = buf_prepare,
        .buf_queue = buf_queue,
        .start_streaming = start_streaming,
        .stop_streaming = stop_streaming,
};

int queue_init(struct video_queue *queue) {
        int ret;

        queue->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        queue->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
        queue->queue.drv_priv = queue;
        queue->queue.buf_struct_size = sizeof(struct video_buffer);
        queue->queue.mem_ops = &vb2_vmalloc_memops;
        queue->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
        queue->queue.lock = &queue->mutex;

        queue->queue.io_modes |= VB2_DMABUF;
        queue->queue.ops = &queue_ops;

        ret = vb2_queue_init(&queue->queue);
        if(ret) return ret;

        mutex_init(&queue->mutex);
        spin_lock_init(&queue->irqlock);
        INIT_LIST_HEAD(&queue->irqqueue);

        return 0;
}

void queue_release(struct video_queue *queue) {
        mutex_lock(&queue->mutex);
        vb2_queue_release(&queue->queue);
        mutex_unlock(&queue->mutex);
}

/* -------------------------------------------------------------------
 * Shared pool & mem
 */
#if 0
void shared_video_pool_destroy(struct shared_video_pool *pool)
{
        struct shared_video_mem *mem, *tmp;

        if (!pool)
                return;

        mutex_lock(&pool->lock);

        /*
         * At this point, no one should hold references.
         * free_list + used_list should contain all buffers.
         */

        list_for_each_entry_safe(mem, tmp, &pool->free_list, list) {
                list_del(&mem->list);
                kfree(mem->vaddr);
                kfree(mem);
        }

        list_for_each_entry_safe(mem, tmp, &pool->used_list, list) {
                list_del(&mem->list);
                kfree(mem->vaddr);
                kfree(mem);
        }

        mutex_unlock(&pool->lock);

        mutex_destroy(&pool->lock);
        kfree(pool);
}

struct shared_video_pool *shared_video_pool_create(void)
{
        struct shared_video_pool *pool;
        struct shared_video_mem *mem;
        int i;

        pool = kzalloc(sizeof(*pool), GFP_KERNEL);
        if (!pool)
                return NULL;

        mutex_init(&pool->lock);
        INIT_LIST_HEAD(&pool->free_list);
        INIT_LIST_HEAD(&pool->used_list);

        pool->num_mems = SHARED_POOL_BUFS;

        for (i = 0; i < SHARED_POOL_BUFS; i++) {
                mem = kzalloc(sizeof(*mem), GFP_KERNEL);
                if (!mem)
                        goto err;

                mem->size = SHARED_BUF_SIZE;

                /*
                 * For now we use kmalloc.
                 * Can be replaced with dma_alloc_coherent() later.
                 */
                mem->vaddr = kmalloc(mem->size, GFP_KERNEL);
                if (!mem->vaddr) {
                        kfree(mem);
                        goto err;
                }

                mem->dma = 0; /* not used yet */
                kref_init(&mem->ref);
                INIT_LIST_HEAD(&mem->list);

                list_add_tail(&mem->list, &pool->free_list);
        }

        return pool;

err:
        shared_video_pool_destroy(pool);
        return NULL;
}

/**/
static void shared_video_mem_release(struct kref *kref)
{
        struct shared_video_mem *mem =
                container_of(kref, struct shared_video_mem, ref);
        struct shared_video_pool *pool = mem->pool;

        /*
         * This is the LAST reference.
         * Safe to move back to free_list.
         */
        mutex_lock(&pool->lock);

        list_del_init(&mem->list);
        list_add_tail(&mem->list, &pool->free_list);

        mutex_unlock(&pool->lock);
}

struct shared_video_mem *pool_get_mem(struct shared_video_pool *pool)
{
        struct shared_video_mem *mem = NULL;

        if (!pool)
                return NULL;

        mutex_lock(&pool->lock);

        if (list_empty(&pool->free_list)) {
                mutex_unlock(&pool->lock);
                return NULL; /* pool exhausted */
        }

        mem = list_first_entry(&pool->free_list,
                               struct shared_video_mem,
                               list);

        list_del_init(&mem->list);
        list_add_tail(&mem->list, &pool->used_list);

        /*
         * First owner.
         * refcount was 0 when in free_list.
         */
        kref_init(&mem->ref);

        mutex_unlock(&pool->lock);

        return mem;
}

void pool_put_mem(struct shared_video_mem *mem)
{
        if (!mem)
                return;

        /*
         * Drop one reference.
         * If this is the last one, mem_release() is called.
         */
        kref_put(&mem->ref, shared_video_mem_release);
}
#endif
/* ------------------------------------------------------------------
 * V4L2 IOCTLs (Capability negotiation for OBS/FFmpeg)
 * Referenced from uvc_v4l2.c
 * ------------------------------------------------------------------ */
//TODO

static const struct v4l2_ioctl_ops ejcm3_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs		= vb2_ioctl_reqbufs,
	.vidioc_querybuf	= vb2_ioctl_querybuf,
	.vidioc_qbuf		= vb2_ioctl_qbuf,
	.vidioc_dqbuf		= vb2_ioctl_dqbuf,
	.vidioc_streamon	= vb2_ioctl_streamon,
	.vidioc_streamoff	= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations ejcm3_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

/* ------------------------------------------------------------------------
 * Video codecs
 */
static int uvc_video_decode_start(struct ejcm3_uvc_streaming *stream,
		struct video_buffer *buf, const u8 *data, int len)
{
	u8 header_len;
	u8 fid;

	/*
	 * Sanity checks:
	 * - packet must be at least 2 bytes long
	 * - bHeaderLength value must be at least 2 bytes (see above)
	 * - bHeaderLength value can't be larger than the packet size.
	 */
	if (len < 2 || data[0] < 2 || data[0] > len) {
		// stream->stats.frame.nb_invalid++;
		return -EINVAL;
	}

	header_len = data[0];
	fid = data[1] & UVC_STREAM_FID;

	/*
	 * Increase the sequence number regardless of any buffer states, so
	 * that discontinuous sequence numbers always indicate lost frames.
	 */
	if (stream->last_fid != fid) {
		stream->sequence++;
		if (stream->sequence)
			uvc_video_stats_update(stream);
	}

	uvc_video_clock_decode(stream, buf, data, len);
	uvc_video_stats_decode(stream, data, len);

	/*
	 * Store the payload FID bit and return immediately when the buffer is
	 * NULL.
	 */
	if (buf == NULL) {
		stream->last_fid = fid;
		return -ENODATA;
	}

	/* Mark the buffer as bad if the error bit is set. */
	if (data[1] & UVC_STREAM_ERR) {
		ejcm3_uvc_dbg(stream->ejcm3_dev, FRAME,
			"Marking buffer as bad (error bit set)\n");
		buf->error = 1;
	}

	/*
	 * Synchronize to the input stream by waiting for the FID bit to be
	 * toggled when the buffer state is not UVC_BUF_STATE_ACTIVE.
	 * stream->last_fid is initialized to -1, so the first isochronous
	 * frame will always be in sync.
	 *
	 * If the device doesn't toggle the FID bit, invert stream->last_fid
	 * when the EOF bit is set to force synchronisation on the next packet.
	 */
	if (buf->state != UVC_BUF_STATE_ACTIVE) {
		if (fid == stream->last_fid) {
			ejcm3_uvc_dbg(stream->ejcm3_dev, FRAME,
				"Dropping payload (out of sync)\n");

			return -ENODATA;
		}

		buf->buf.field = V4L2_FIELD_NONE;
		buf->buf.sequence = stream->sequence;
		buf->buf.vb2_buf.timestamp = ktime_to_ns(uvc_video_get_time());

		/* TODO: Handle PTS and SCR. */
		buf->state = UVC_BUF_STATE_ACTIVE;
	}

	/*
	 * Mark the buffer as done if we're at the beginning of a new frame.
	 * End of frame detection is better implemented by checking the EOF
	 * bit (FID bit toggling is delayed by one frame compared to the EOF
	 * bit), but some devices don't set the bit at end of frame (and the
	 * last payload can be lost anyway). We thus must check if the FID has
	 * been toggled.
	 *
	 * stream->last_fid is initialized to -1, so the first isochronous
	 * frame will never trigger an end of frame detection.
	 *
	 * Empty buffers (bytesused == 0) don't trigger end of frame detection
	 * as it doesn't make sense to return an empty buffer. This also
	 * avoids detecting end of frame conditions at FID toggling if the
	 * previous payload had the EOF bit set.
	 */
	if (fid != stream->last_fid && buf->byteused != 0) {
		ejcm3_uvc_dbg(stream->ejcm3_dev, FRAME,
			"Frame complete (FID bit toggled)\n");
		buf->state = UVC_BUF_STATE_READY;
		return -EAGAIN;
	}

	stream->last_fid = fid;

	return header_len;
}

static void uvc_video_decode_data(struct uvc_urb *uvc_urb,
		struct video_buffer *buf, const u8 *data, int len)
{
	unsigned int active_op = uvc_urb->async_operations;
	struct uvc_copy_op *op = &uvc_urb->copy_operations[active_op];
	unsigned int maxlen;

	if (len <= 0)
		return;

	maxlen = buf->length - buf->byteused;

	/* Take a buffer reference for async work. */
	kref_get(&buf->kref);

	op->buf = buf;
	op->src = data;
	op->dst = buf->mem + buf->byteused;
	op->len = min_t(unsigned int, len, maxlen);

	buf->byteused += op->len;

	/* Complete the current frame if the buffer size was exceeded. */
	if (len > maxlen) {
		ejcm3_uvc_dbg(uvc_urb->stream->ejcm3_dev, FRAME,
			"Frame complete (overflow)\n");
		buf->error = 1;
		buf->state = UVC_BUF_STATE_READY;
	}

	uvc_urb->async_operations++;
}

static void uvc_video_decode_end(struct ejcm3_uvc_streaming *stream,
		struct video_buffer *buf, const u8 *data, int len)
{
	/* Mark the buffer as done if the EOF marker is set. */
	if (data[1] & UVC_STREAM_EOF && buf->byteused != 0) {
		ejcm3_uvc_dbg(stream->ejcm3_dev, FRAME, "Frame complete (EOF found)\n");
		if (data[0] == len)
			ejcm3_uvc_dbg(stream->ejcm3_dev, FRAME, "EOF in empty payload\n");
		buf->state = UVC_BUF_STATE_READY;
	}
}

/* ------------------------------------------------------------------------
 * URB handling
 * ------------------------------------------------------------------------ */
/*
 * Set error flag for incomplete buffer.
 */
static void uvc_video_validate_buffer(const struct ejcm3_uvc_streaming *stream,
				      struct video_buffer *buf)
{
	if (stream->ctrl.dwMaxVideoFrameSize != buf->byteused &&
	    !(stream->cur_format->flags & UVC_FMT_FLAG_COMPRESSED))
		buf->error = 1;
}

/*
 * Completion handler for video URBs.
 */
static void uvc_video_next_buffers(struct ejcm3_uvc_streaming *stream,
				   struct video_buffer **video_buf)
{
	uvc_video_validate_buffer(stream, *video_buf);
	*video_buf = uvc_queue_next_buffer(&stream->queue, *video_buf);
}

static void uvc_video_decode_bulk(struct uvc_urb *uvc_urb,
			struct video_buffer *buf)
{
	struct urb *urb = uvc_urb->urb;
	struct ejcm3_uvc_streaming *stream = uvc_urb->stream;
	u8 *mem;
	int len, ret;

	/*
	 * Ignore ZLPs if they're not part of a frame, otherwise process them
	 * to trigger the end of payload detection.
	 */
	if (urb->actual_length == 0 && stream->bulk.header_size == 0)
		return;

	mem = urb->transfer_buffer;
	len = urb->actual_length;
	stream->bulk.payload_size += len;

	/*
	 * If the URB is the first of its payload, decode and save the
	 * header.
	 */
	if (stream->bulk.header_size == 0 && !stream->bulk.skip_payload) {	
		ret = uvc_video_decode_start(stream, buf, mem, len);


		/* If an error occurred skip the rest of the payload. */
		if (ret < 0 || buf == NULL) {
			stream->bulk.skip_payload = 1;
		} else {
			memcpy(stream->bulk.header, mem, ret);
			stream->bulk.header_size = ret;

			mem += ret;
			len -= ret;
		}
	}

	/*
	 * The buffer queue might have been cancelled while a bulk transfer
	 * was in progress, so we can reach here with buf equal to NULL. Make
	 * sure buf is never dereferenced if NULL.
	 */

	/* Prepare video data for processing. */
	if (!stream->bulk.skip_payload && buf != NULL)
		uvc_video_decode_data(uvc_urb, buf, mem, len);

	/*
	 * Detect the payload end by a URB smaller than the maximum size (or
	 * a payload size equal to the maximum) and process the header again.
	 */
	if (urb->actual_length < urb->transfer_buffer_length ||
	    stream->bulk.payload_size >= stream->bulk.max_payload_size) {
		if (!stream->bulk.skip_payload && buf != NULL) {
			uvc_video_decode_end(stream, buf, stream->bulk.header,
				stream->bulk.payload_size);
			if (buf->state == UVC_BUF_STATE_READY)
				uvc_video_next_buffers(stream, &buf);
		}

		stream->bulk.header_size = 0;
		stream->bulk.skip_payload = 0;
		stream->bulk.payload_size = 0;
	}
}

/* ------------------------------------------------------------------------
 * UVC Controls
 * ------------------------------------------------------------------------*/
static int __uvc_query_ctrl(struct ejcm3_uvc *dev, u8 query, u8 unit,
			u8 intfnum, u8 cs, void *data, u16 size,
			int timeout)
{
	u8 type = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;

	pipe = (query & 0x80) ? usb_rcvctrlpipe(dev->udev, 0)
			      : usb_sndctrlpipe(dev->udev, 0);
	type |= (query & 0x80) ? USB_DIR_IN : USB_DIR_OUT;

	return usb_control_msg(dev->udev, pipe, query, type, cs << 8,
			unit << 8 | intfnum, data, size, timeout);
}

static int uvc_get_video_ctrl(struct ejcm3_uvc_streaming *stream,
	struct uvc_streaming_control *ctrl, int probe, u8 query)
{
	u16 size = 34; // 0x0110 <= uvc version < 0x0150
	u8 *data;
	int ret;

        data = kmalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	ret = __uvc_query_ctrl(stream->ejcm3_dev, query, 0, stream->intfnum,
		probe ? UVC_VS_PROBE_CONTROL : UVC_VS_COMMIT_CONTROL, data,
		size, UVC_CTRL_CONTROL_TIMEOUT);

                	if ((query == UVC_GET_MIN || query == UVC_GET_MAX) && ret == 2) {
		/*
		 * Some cameras, mostly based on Bison Electronics chipsets,
		 * answer a GET_MIN or GET_MAX request with the wCompQuality
		 * field only.
		 */
		dev_warn(stream->ejcm3_dev, "UVC non "
			"compliance - GET_MIN/MAX(PROBE) incorrectly "
			"supported. Enabling workaround.\n");
		memset(ctrl, 0, sizeof(*ctrl));
		ctrl->wCompQuality = le16_to_cpup((__le16 *)data);
		ret = 0;
		goto out;
	} else if (query == UVC_GET_DEF && probe == 1 && ret != size) {
		/*
		 * Many cameras don't support the GET_DEF request on their
		 * video probe control. Warn once and return, the caller will
		 * fall back to GET_CUR.
		 */
		dev_warn(stream->ejcm3_dev, "UVC non "
			"compliance - GET_DEF(PROBE) not supported. "
			"Enabling workaround.\n");
		ret = -EIO;
		goto out;
	} else if (ret != size) {
		dev_err(&stream->intf->dev,
			"Failed to query (%s) UVC %s control : %d (exp. %u).\n",
			uvc_query_name(query), probe ? "probe" : "commit",
			ret, size);
		ret = (ret == -EPROTO) ? -EPROTO : -EIO;
		goto out;
	}

	ctrl->bmHint = le16_to_cpup((__le16 *)&data[0]);
	ctrl->bFormatIndex = data[2];
	ctrl->bFrameIndex = data[3];
	ctrl->dwFrameInterval = le32_to_cpup((__le32 *)&data[4]);
	ctrl->wKeyFrameRate = le16_to_cpup((__le16 *)&data[8]);
	ctrl->wPFrameRate = le16_to_cpup((__le16 *)&data[10]);
	ctrl->wCompQuality = le16_to_cpup((__le16 *)&data[12]);
	ctrl->wCompWindowSize = le16_to_cpup((__le16 *)&data[14]);
	ctrl->wDelay = le16_to_cpup((__le16 *)&data[16]);
	ctrl->dwMaxVideoFrameSize = get_unaligned_le32(&data[18]);
	ctrl->dwMaxPayloadTransferSize = get_unaligned_le32(&data[22]);

	if (size >= 34) {
		ctrl->dwClockFrequency = get_unaligned_le32(&data[26]);
		ctrl->bmFramingInfo = data[30];
		ctrl->bPreferedVersion = data[31];
		ctrl->bMinVersion = data[32];
		ctrl->bMaxVersion = data[33];
	} else {
		ctrl->dwClockFrequency = stream->ejcm3_dev->clock_frequency;
		ctrl->bmFramingInfo = 0;
		ctrl->bPreferedVersion = 0;
		ctrl->bMinVersion = 0;
		ctrl->bMaxVersion = 0;
	}
        ret = 0;

out:
	kfree(data);
	return ret;
}

static int uvc_set_video_ctrl(struct ejcm3_uvc_streaming *stream,
	struct uvc_streaming_control *ctrl, int probe)
{
	u16 size = uvc_video_ctrl_size(stream);
	u8 *data;
	int ret;

	data = kzalloc(size, GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	*(__le16 *)&data[0] = cpu_to_le16(ctrl->bmHint);
	data[2] = ctrl->bFormatIndex;
	data[3] = ctrl->bFrameIndex;
	*(__le32 *)&data[4] = cpu_to_le32(ctrl->dwFrameInterval);
	*(__le16 *)&data[8] = cpu_to_le16(ctrl->wKeyFrameRate);
	*(__le16 *)&data[10] = cpu_to_le16(ctrl->wPFrameRate);
	*(__le16 *)&data[12] = cpu_to_le16(ctrl->wCompQuality);
	*(__le16 *)&data[14] = cpu_to_le16(ctrl->wCompWindowSize);
	*(__le16 *)&data[16] = cpu_to_le16(ctrl->wDelay);
	put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
	put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

	if (size >= 34) {
		put_unaligned_le32(ctrl->dwClockFrequency, &data[26]);
		data[30] = ctrl->bmFramingInfo;
		data[31] = ctrl->bPreferedVersion;
		data[32] = ctrl->bMinVersion;
		data[33] = ctrl->bMaxVersion;
	}

	ret = __uvc_query_ctrl(stream->ejcm3_dev, UVC_SET_CUR, 0, stream->intfnum,
		probe ? UVC_VS_PROBE_CONTROL : UVC_VS_COMMIT_CONTROL, data,
		size, UVC_CTRL_CONTROL_TIMEOUT);
	if (ret != size) {
		dev_err(&stream->intf->dev,
			"Failed to set UVC %s control : %d (exp. %u).\n",
			probe ? "probe" : "commit", ret, size);
		ret = -EIO;
	}

	kfree(data);
	return ret;
}

/* -------------------------------------------------------------------
 * Parse control, streaming interface, format and frame descriptors
 * ------------------------------------------------------------------- */
static int uvc_parse_format(struct ejcm3_uvc *dev, struct ejcm3_uvc_streaming *streaming,
			    struct ejcm3_uvc_format *format,
			    struct ejcm3_uvc_frame *frames,
			    u32 **intervals, const unsigned char *buffer, int buflen)
{
	struct usb_interface *intf = streaming->intf;
	struct usb_host_interface *alts = intf->cur_altsetting;
	struct ucv_format_desc *fmtdesc;
	struct ejcm3_uvc_frame *frame;
	const unsigned char *start = buffer;
	unsigned int interval;
	unsigned int i, n;
	u8 ftype;

	format->type = buffer[2];
	format->index = buffer[3];
	format->frames = frames;

	switch(buffer[2]) {
		case UVC_VS_FORMAT_UNCOMPRESSED:
		case UVC_VS_FORMAT_FRAME_BASED:
		        n = buffer[2] == UVC_VS_FORMAT_UNCOMPRESSED ? 27 : 28;
			if (buflen < n) {
				ejcm3_uvc_dbg(dev, DESCR,
					"device %d videostreaming interface %d FORMAT %u descriptor too short\n",
					dev->udev->devnum,
					alts->desc.bInterfaceNumber,
					buffer[2]);
				return -EINVAL;
			}

			fmtdesc = uvc_format_by_guid(&buffer[5]);
			if (!fmtdesc) {
				dev_info(&streaming->intf->dev,
				         "Unknown video format %pU1\n", &buffer[5]);
				return 0;
			}

			format->fcc = fmtdesc->fcc; //
			format->bpp = buffer[21];

			if (buffer[2] == UVC_VS_FORMAT_UNCOMPRESSED) {
				ftype = UVC_VS_FRAME_UNCOMPRESSED;
			} else {
				ftype = UVC_VS_FRAME_FRAME_BASED;
				if (buffer[27])
					format->flags = UVC_FMT_FLAG_COMPRESSED;
			}
			break;
		
		case UVC_VS_FORMAT_MJPEG:
			if (buflen < 11) {
				ejcm3_uvc_dbg(dev, DESCR,
					"device %d videostreaming interface %d FORMAT MJPEG descriptor too short\n",
					dev->udev->devnum,
					alts->desc.bInterfaceNumber);
				return -EINVAL;	
			}
			format->fcc = V4L2_PIX_FMT_MJPEG;
			format->flags = UVC_FMT_FLAG_COMPRESSED;
			format->bpp = 0;
			ftype = UVC_VS_FRAME_MJPEG;
			break;

		default:
			ejcm3_uvc_dbg(dev, DESCR,
				"device %d videostreaming interface %d FORMAT %u is not supported\n",
				dev->udev->devnum,
				alts->desc.bInterfaceNumber, buffer[2]);
			return -EINVAL;
	}

	ejcm3_uvc_dbg(dev, DESCR,
	              "Found format %p4cc", &format->fcc);

	buflen -= buffer[0];
	buffer += buffer[0];

	/*
	 * Parse the frame descriptors.
	 * uncompressed, MJPEG and frame-based formats.
	 */
	while (ftype && buflen > 2 && buffer[1] == USB_DT_CS_INTERFACE &&
	       buffer[2] == ftype) {
		unsigned int maxIntervalIndex;

		frame = &frames[format->nframes];
		if (ftype != UVC_VS_FRAME_FRAME_BASED)
			n = buflen > 25 ? buffer[25] : 0;
		else
			n = buflen > 21 ? buffer[21] : 0;

		n = n ? n : 3;

		if (buflen < 26 + 4*n) {
			ejcm3_uvc_dbg(dev, DESCR,
				"device %d videostreaming interface %d FRAME error\n",
				dev->udev->devnum,
				alts->desc.bInterfaceNumber);
			return -EINVAL;
		}

		frame->bFrameIndex = buffer[3];
		frame->bmCapabilities = buffer[4];
		frame->wWidth = get_unaligned_le16(&buffer[5]); // * width_multiplier;
		frame->wHeight = get_unaligned_le16(&buffer[7]);
		frame->dwMinBitRate = get_unaligned_le32(&buffer[9]);
		frame->dwMaxBitRate = get_unaligned_le32(&buffer[13]);
		if (ftype != UVC_VS_FRAME_FRAME_BASED) {
			frame->dwMaxVideoFrameBufferSize =
				get_unaligned_le32(&buffer[17]);
			frame->dwDefaultFrameInterval =
				get_unaligned_le32(&buffer[21]);
			frame->bFrameIntervalType = buffer[25];
		} else {
			frame->dwMaxVideoFrameBufferSize = 0;
			frame->dwDefaultFrameInterval =
				get_unaligned_le32(&buffer[17]);
			frame->bFrameIntervalType = buffer[21];
		}

		/*
		 * Copy the frame intervals.
		 *
		 * Some bogus devices report dwMinFrameInterval equal to
		 * dwMaxFrameInterval and have dwFrameIntervalStep set to
		 * zero. Setting all null intervals to 1 fixes the problem and
		 * some other divisions by zero that could happen.
		 */
		frame->dwFrameInterval = *intervals;

		for (i = 0; i < n; ++i) {
			interval = get_unaligned_le32(&buffer[26+4*i]);
			(*intervals)[i] = interval ? interval : 1;
		}

		/*
		 * Clamp the default frame interval to the boundaries. A zero
		 * bFrameIntervalType value indicates a continuous frame
		 * interval range, with dwFrameInterval[0] storing the minimum
		 * value and dwFrameInterval[1] storing the maximum value.
		 */
		maxIntervalIndex = frame->bFrameIntervalType ? n - 1 : 1;
		frame->dwDefaultFrameInterval =
			clamp(frame->dwDefaultFrameInterval,
			      frame->dwFrameInterval[0],
			      frame->dwFrameInterval[maxIntervalIndex]);

		ejcm3_uvc_dbg(dev, DESCR, "- %ux%u (%u.%u fps)\n",
			frame->wWidth, frame->wHeight,
			10000000 / frame->dwDefaultFrameInterval,
			(100000000 / frame->dwDefaultFrameInterval) % 10);

		format->nframes++;
		*intervals += n;

		buflen -= buffer[0];
		buffer += buffer[0];
	}
	/* No UVC_VS_STILL_IMAGE_FRAME and UVC_VS_COLORFORMAT */
	format->colorspace = V4L2_COLORSPACE_SRGB;

	return buffer - start;
}

static int uvc_parse_streaming(struct ejcm3_uvc *dev, struct usb_interface *intf)
{
	struct ejcm3_uvc_streaming *streaming = NULL;
	struct ejcm3_uvc_format *format;
	struct ejcm3_uvc_frame *frame;
	struct usb_host_interface *alts = &intf->altsetting[0];
	const unsigned char *_buffer, *buffer = alts->extra;
	int _buflen, buflen = alts->extralen;
	unsigned int nformats = 0, nframes = 0, nintervals = 0;
	unsigned int size, i, n, p;
	u32 *interval;
	int ret = -EINVAL;

	if (intf->cur_altsetting->desc.bInterfaceSubClass != UVC_SC_VIDEOSTREAMING) {
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d interface %d isn't a video streaming interface\n",
			dev->udev->devnum,
			intf->altsetting[0].desc.bInterfaceNumber);
		return -EINVAL;
	}

	if (usb_driver_claim_interface(&ejcm3_uvc_driver, intf, dev)) {
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d interface %d is already claimed\n",
			dev->udev->devnum,
			intf->altsetting[0].desc.bInterfaceNumber);
		return -EINVAL;
	}

	streaming = stream_new(dev, intf);
	if (streaming == NULL) {
		usb_driver_release_interface(&ejcm3_uvc_driver, intf);
		return -ENOMEM;
	}

	/* Skip the standard interface descriptors. */
	while (buflen > 2 && buffer[1] != USB_DT_CS_INTERFACE) {
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (buflen <= 2) {
		ejcm3_uvc_dbg(dev, DESCR,
			"no class-specific streaming interface descriptors found\n");
		goto error;
	}

	/* Parse the header descriptor. */
	switch (buffer[2]) {
	case UVC_VS_OUTPUT_HEADER:
		streaming->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		size = 9;
		break;

	case UVC_VS_INPUT_HEADER:
		streaming->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		size = 13;
		break;

	default:
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d videostreaming interface %d HEADER descriptor not found\n",
			dev->udev->devnum, alts->desc.bInterfaceNumber);
		goto error;
	}

	p = buflen >= 4 ? buffer[3] : 0;
	n = buflen >= size ? buffer[size-1] : 0;

	if (buflen < size + p*n) {
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d videostreaming interface %d HEADER descriptor is invalid\n",
			dev->udev->devnum, alts->desc.bInterfaceNumber);
		goto error;
	}

	streaming->header.bNumFormats = p;
	streaming->header.bEndpointAddress = buffer[6];
	if (buffer[2] == UVC_VS_INPUT_HEADER) {
		streaming->header.bmInfo = buffer[7];
		streaming->header.bTerminalLink = buffer[8];
		streaming->header.bStillCaptureMethod = buffer[9];
		streaming->header.bTriggerSupport = buffer[10];
		streaming->header.bTriggerUsage = buffer[11];
	} else {
		streaming->header.bTerminalLink = buffer[7];
	}
	streaming->header.bControlSize = n;

	streaming->header.bmaControls = kmemdup(&buffer[size], p * n,
						GFP_KERNEL);
	if (streaming->header.bmaControls == NULL) {
		ret = -ENOMEM;
		goto error;
	}

	buflen -= buffer[0];
	buffer += buffer[0];

	_buffer = buffer;
	_buflen = buflen;

	/* Count the format and frame descriptors. */
	while (_buflen > 2 && _buffer[1] == USB_DT_CS_INTERFACE) {
		switch (_buffer[2]) {
		case UVC_VS_FORMAT_UNCOMPRESSED:
		case UVC_VS_FORMAT_MJPEG:
		case UVC_VS_FORMAT_FRAME_BASED:
			nformats++;
			break;

		case UVC_VS_FORMAT_DV:
			/*
			 * DV format has no frame descriptor. We will create a
			 * dummy frame descriptor with a dummy frame interval.
			 */
			nformats++;
			nframes++;
			nintervals++;
			break;

		case UVC_VS_FORMAT_MPEG2TS:
		case UVC_VS_FORMAT_STREAM_BASED:
			ejcm3_uvc_dbg(dev, DESCR,
				"device %d videostreaming interface %d FORMAT %u is not supported\n",
				dev->udev->devnum,
				alts->desc.bInterfaceNumber, _buffer[2]);
			break;

		case UVC_VS_FRAME_UNCOMPRESSED:
		case UVC_VS_FRAME_MJPEG:
			nframes++;
			if (_buflen > 25)
				nintervals += _buffer[25] ? _buffer[25] : 3;
			break;

		case UVC_VS_FRAME_FRAME_BASED:
			nframes++;
			if (_buflen > 21)
				nintervals += _buffer[21] ? _buffer[21] : 3;
			break;
		}

		_buflen -= _buffer[0];
		_buffer += _buffer[0];
	}

	if (nformats == 0) {
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d videostreaming interface %d has no supported formats defined\n",
			dev->udev->devnum, alts->desc.bInterfaceNumber);
		goto error;
	}

	/*
	 * Allocate memory for the formats, the frames and the intervals,
	 * plus any required padding to guarantee that everything has the
	 * correct alignment.
	 */
	size = nformats * sizeof(*format);
	size = ALIGN(size, __alignof__(*frame)) + nframes * sizeof(*frame);
	size = ALIGN(size, __alignof__(*interval))
	     + nintervals * sizeof(*interval);

	format = kzalloc(size, GFP_KERNEL);
	if (!format) {
		ret = -ENOMEM;
		goto error;
	}

	frame = (void *)format + nformats * sizeof(*format);
	frame = PTR_ALIGN(frame, __alignof__(*frame));
	interval = (void *)frame + nframes * sizeof(*frame);
	interval = PTR_ALIGN(interval, __alignof__(*interval));

	streaming->formats = format;
	streaming->nformats = 0;

	/* Parse the format descriptors. */
	while (buflen > 2 && buffer[1] == USB_DT_CS_INTERFACE) {
		switch (buffer[2]) {
		case UVC_VS_FORMAT_UNCOMPRESSED:
		case UVC_VS_FORMAT_MJPEG:
		case UVC_VS_FORMAT_DV:
		case UVC_VS_FORMAT_FRAME_BASED:
			ret = uvc_parse_format(dev, streaming, format, frame,
				&interval, buffer, buflen);//
			if (ret < 0)
				goto error;
			if (!ret)
				break;

			streaming->nformats++;
			frame += format->nframes;
			format++;

			buflen -= ret;
			buffer += ret;
			continue;

		default:
			break;
		}

		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (buflen)
		ejcm3_uvc_dbg(dev, DESCR,
			"device %d videostreaming interface %d has %u bytes of trailing descriptor garbage\n",
			dev->udev->devnum, alts->desc.bInterfaceNumber, buflen);

	/* 
	 * Parse the alternate settings to find the maximum bandwidth.
	 * Omit - bulk IN ep 0x89, wMaxPacketSize 0x0400(1024 bytes)
	 */

	list_add_tail(&streaming->list, &dev->streams);
	return 0;

error:
	usb_driver_release_interface(&ejcm3_uvc_driver, intf);
	stream_delete(streaming);
	return ret;
}

static int uvc_parse_standard_control(struct ejcm3_uvc *dev,
        const unsigned char *buffer, int buflen)
{
        struct usb_device *udev = dev->udev;
        struct usb_interface *intf;
        struct usb_host_interface *alts = dev->intf->cur_altsetting;
        unsigned int i, n;

        switch (buffer[2]) {
        case UVC_VC_HEADER:
                n = buflen >= 12 ? buffer[11] : 0;

                if (buflen < 12 + n) {
                        ejcm3_uvc_dbg(dev, DESCR,
                                "device %d videocontrol interface %d HEADER error\n",
                                udev->devnum, alts->desc.bInterfaceNumber);
                        return -EINVAL;
                }

                dev->uvc_version = get_unaligned_le16(&buffer[3]);
                dev->clock_frequency = get_unaligned_le32(&buffer[7]);

                /* Parse all USB Video Streaming interfaces. */
                for (i = 0; i < n; ++i) {
                        intf = usb_ifnum_to_if(udev, buffer[12+i]);
                        if (intf == NULL) {
                                ejcm3_uvc_dbg(dev, DESCR,
                                        "device %d interface %d doesn't exists\n",
                                        udev->devnum, i);
                                continue;
                        }

                        uvc_parse_streaming(dev, intf);
                }
                break;

	case UVC_VC_OUTPUT_TERMINAL:
		dev->output_TerminalID = buffer[3];
		dev->output_TerminalType = get_unaligned_le16(&buffer[4]);
		break;

        case UVC_VC_INPUT_TERMINAL:       
        case UVC_VC_SELECTOR_UNIT:
        case UVC_VC_PROCESSING_UNIT:
        case UVC_VC_EXTENSION_UNIT:
                /* omit */
                break;

        default:
                ejcm3_uvc_dbg(dev, DESCR,
                        "Found an unknown CS_INTERFACE descriptor (%u)\n",
                        buffer[2]);
                break;
        }

        return 0;
}

static int uvc_parse_control(struct ejcm3_uvc *dev)
{
	struct usb_host_interface *alts = dev->intf->cur_altsetting;
	const unsigned char *buffer = alts->extra;
	int buflen = alts->extralen;
	int ret;

	/*
	 * Parse the default alternate setting only, as the UVC specification
	 * defines a single alternate setting, the default alternate setting
	 * zero.
	 */
	while (buflen > 2) {
		if (buffer[1] != USB_DT_CS_INTERFACE)
			goto next_descriptor;

		ret = uvc_parse_standard_control(dev, buffer, buflen);
		if (ret < 0)
			return ret;

next_descriptor:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

        return 0;
}

/* -------------------------------------------------------------------
 * register video
 * ------------------------------------------------------------------- */
static void uvc_release(struct video_device *vdev)
{
	struct ejcm3_uvc_streaming *stream = video_get_drvdata(vdev);
	struct ejcm3_uvc *dev = stream->ejcm3_dev;

	kref_put(&dev->kref, ejcm3_uvc_delete);
}

int video_init(struct ejcm3_uvc_streaming *stream) {
        struct uvc_streaming_control *probe = &stream->ctrl;
        const struct ejcm3_uvc_format *format = NULL;
        const struct ejcm3_uvc_frame *frame = NULL;
        struct uvc_urb *uvc_urb;
	unsigned int i;
        int ret;

	if (stream->nformats == 0) {
		dev_info(&stream->intf->dev,
			 "No supported video formats found.\n");
		return -EINVAL;
	}

	atomic_set(&stream->active, 0);

        // GET_DEF, SET_CUR, GET_CUR
        usb_set_interface(stream->ejcm3_dev->udev, stream->intfnum, 0);

        if (uvc_get_video_ctrl(stream, probe, 1, UVC_GET_DEF) == 0)
		uvc_set_video_ctrl(stream, probe, 1);

        ret = uvc_get_video_ctrl(stream, probe, 1, UVC_GET_CUR);
        if (ret < 0)
		return ret;

	for (i = stream->nformats; i > 0; --i) {
		format = &stream->formats[i-1];
		if (format->index == probe->bFormatIndex)
			break;
	}

	if (format->nframes == 0) {
		dev_info(&stream->intf->dev,
			 "No frame descriptor found for the default format.\n");
		return -EINVAL;
	}

	for (i = format->nframes; i > 0; --i) {
		frame = &format->frames[i-1];
		if (frame->bFrameIndex == probe->bFrameIndex)
			break;
	}

	probe->bFormatIndex = format->index;
	probe->bFrameIndex = frame->bFrameIndex;

	stream->def_format = format;
	stream->cur_format = format;
	stream->cur_frame = frame;

	// Default only V4L2_BUF_TYPE_VIDEO_CAPTURE
	stream->decode = uvc_video_decode_bulk;

	/* Prepare asynchronous work items. */
	for_each_uvc_urb(uvc_urb, stream)
		INIT_WORK(&uvc_urb->work, uvc_video_copy_data_work);

	return 0;
}

/* regist /dev/videoX */
int register_video_node(struct ejcm3_uvc *dev,
                        struct ejcm3_uvc_streaming *stream,
			struct video_device *vdev,
			struct video_queue *queue,
                        const struct v4l2_file_operations *fops, 
                        const struct v4l2_ioctl_ops *ioctl_ops)
{
        int ret;

        /* queue init */
        ret = queue_init(queue);
        if(ret) return ret;

        /* TODO: register dev v4l2*/
        vdev->v4l2_dev = &dev->v4l2_dev;
        vdev->fops = fops;
        vdev->ioctl_ops = ioctl_ops;
	/* /dev/videoX 被 unregister 且所有 App 都關閉後，V4L2 會呼叫此函式。 */
        vdev->release = uvc_release;
        vdev->prio = &stream->prio;
	if (stream->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		vdev->vfl_dir = VFL_DIR_TX;
	else
		vdev->vfl_dir = VFL_DIR_RX;

	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

        if(global_mgr.dev_count > 1) {
                snprintf(vdev->name, sizeof(vdev->name), "EJCM3 Dual Camera");
        } else {
                strscpy(vdev->name, dev->name, sizeof(vdev->name));
        }
        
	/*
	 * Set the driver data before calling video_register_device, otherwise
	 * the file open() handler might race us.
	 */
        video_set_drvdata(&stream->vdev, stream);

        ret = video_register_device(&stream->vdev, VFL_TYPE_VIDEO, -1);
        if(ret < 0) {
                return ret;
        }

        kref_get(&dev->kref);
        
        return 0;
}

static struct ejcm3_uvc_streaming *find_stream_by_id(struct ejcm3_uvc *dev, u8 stream_id)
{
	struct ejcm3_uvc_streaming *stream;

	list_for_each_entry(stream, &dev->streams, list) {
		if (stream->header.bTerminalLink == stream_id) {
			return stream;
		}
	}

	return NULL;
}

/* Initialize sigle ejcm3 uvc device(port) */
static int regist_video(struct ejcm3_uvc *dev)
{
	struct ejcm3_uvc_streaming *stream;
        int ret;

	stream = find_stream_by_id(dev, dev->output_TerminalID);
	if (stream == NULL) {
		dev_info(&dev->udev->dev,
			 "No video streaming interface found for output terminal ID %u.\n",
			 dev->output_TerminalID);
		return -EINVAL;
	}

        /* 
         * Initialize the UVC video device by switching to alternate setting 0 and
         * retrieve the default format.
         */
        ret = video_init(stream);
        if(ret < 0) {
		dev_err(&stream->intf->dev,
			"Failed to initialize the device (%d).\n", ret);
                return ret;
        }

        ret = register_video_node(&dev, stream,
				  &stream->vdev,
				  &stream->queue,
				  &ejcm3_fops,
				  &ejcm3_ioctl_ops);

        return ret;
}

/* -------------------------------------------------------------------
 * EJCM3 USB probe, disconnect
 */
static int ejcm3_uvc_probe(struct usb_interface *intf, const struct usb_device_id *id) {
        struct usb_device *udev = interface_to_usbdev(intf);
        struct ejcm3_uvc *dev;
        struct ejcm3_uvc_streaming *stream;
        int ret;

        dev = kzalloc(sizeof(*dev), GFP_KERNEL);
        if (dev == NULL)
                return -ENOMEM;

	INIT_LIST_HEAD(&dev->streams);
        kref_init(&dev->kref);
	atomic_set(&dev->nmappings, 0);
	mutex_init(&dev->lock);

        dev->udev = usb_get_dev(udev);
        dev->intf = usb_get_intf(intf);
        dev->intfnum = intf->cur_altsetting->desc.bInterfaceNumber;
        // dev->clock_frequency = 0x01C9C380;

	snprintf(dev->name, sizeof(dev->name),
		 "EJCM3 (%04x:%04x)",
		 VENDOR_ID,
		 id->idProduct);

        /* Parse the Video Class control descriptor. */
	if (uvc_parse_control(dev) < 0) {
		dev_err(dev, "Unable to parse UVC descriptors\n");
		goto error;
	}

	dev_info(&dev->udev->dev, "Found UVC %u.%02x device %s (%04x:%04x)\n",
		 dev->uvc_version >> 8, dev->uvc_version & 0xff,
		 udev->product ? udev->product : "<unnamed>",
		 le16_to_cpu(udev->descriptor.idVendor),
		 le16_to_cpu(udev->descriptor.idProduct));
	
	/* Register the V4L2 device */
        ret = v4l2_device_register(&intf->dev, &dev->v4l2_dev);
        if (ret < 0)
                goto error;

        ret = regist_video(stream);
        if (ret)
                goto error;

        usb_set_intfdata(intf, dev);

	/*
	mutex_lock(&global_mgr.lock);

	list_add_tail(&dev->list, &global_mgr.dev_list);
        global_mgr.dev_count++;
        
        if (global_mgr.dev_count == 2 && !global_mgr.stitch_node) {
                ret = create_stitch_node(); //TODO
                if (ret) pr_err("EJCM3: Failed to create stitch node\n");
        }

        mutex_unlock(&global_mgr.lock);
	*/
        
        return 0;

error:
	uvc_unregister_video(dev);
	kref_put(&dev->kref, ejcm3_uvc_delete);
	return -ENODEV;
}

static void ejcm3_uvc_disconnect(struct usb_interface *intf)
{
        struct ejcm3_uvc *dev = usb_get_intfdata(intf);

        if (!dev)
                return;

        usb_set_intfdata(intf, NULL);

        /* -------------------------------------------------- */
        /* remove from global manager                         */
        /* -------------------------------------------------- */
        mutex_lock(&global_mgr.lock);

        if (!list_empty(&dev->list)) {
                list_del_init(&dev->list);
                global_mgr.dev_count--;
        }

        /* teardown stitch node if needed */
        if (global_mgr.dev_count < 2 && global_mgr.stitch_node) {
                destroy_stitch_node(global_mgr.stitch_node);
                global_mgr.stitch_node = NULL;
        }

        mutex_unlock(&global_mgr.lock);

        /* -------------------------------------------------- */
        /* unregister V4L2 devices                            */
        /* -------------------------------------------------- */
        /*
         * video_unregister_device():
         *  - stops new opens
         *  - waits until last fd is closed
         *  - then calls vdev->release()
         */
        /* streaming node will be released in video_release() */

        v4l2_device_unregister(&dev->v4l2_dev);

        /* -------------------------------------------------- */
        /* drop usb references                                */
        /* -------------------------------------------------- */
        usb_put_intf(dev->intf);
        usb_put_dev(dev->udev);

        /* -------------------------------------------------- */
        /* final device put                                   */
        /* -------------------------------------------------- */
        kref_put(&dev->kref, ejcm3_uvc_release);
}

/* -------------------------------------------------------------------
 * Driver initialize
 */

static const struct usb_device_id ejcm3_uvc_ids[] = {

};

MODULE_DEVICE_TABLE(usb, ejcm3_uvc_ids);

struct usb_driver ejcm3_uvc_driver = {
        .name = "uvc_dual_drive",
        .probe = ejcm3_uvc_probe,
        .disconnect = ejcm3_uvc_disconnect,
        .id_table = ejcm3_uvc_ids,
};

static int __init dual_drive_init(void) {
        int ret;
        
        INIT_LIST_HEAD(&global_mgr.dev_list);
        mutex_init(&global_mgr.lock);
        global_mgr.dev_count = 0;
        global_mgr.stitch_node = NULL;



        return 0;
}

static void __exit dual_drive_cleanup(void) {

}

module_init(dual_drive_init);
module_exit(dual_drive_cleanup);

MODULE_AUTHOR(DRIVER_AUTHOR);
