# EJ_Stitcher — a kernel module to discover two specific UVC devices, and stitches their frames top-to-bottom

A Linux kernel module that discovers two UVC cameras by USB VID/PID via
`class_interface`, opens them in kernel space with `filp_open`, and stitches
their NV12 frames top-to-bottom into a single `/dev/videoN` V4L2 capture
device — without modifying `uvcvideo`.

The output node is available immediately after `modprobe`. UVC sources are
filled asynchronously on hotplug; `VIDIOC_STREAMON` is rejected with
`-ENODEV` until both slots are ready. A dedicated kthread dequeues one frame
from each source per iteration, stitches them, and signals the combined buffer
to the consumer.

# Hardware Requirements

| Parameter     | Value                              |
|---------------|------------------------------------|
| Pixel Format  | NV12                               |
| Stitch Mode   | Top-Bottom                         |

Both cameras must be the same model and produce the same resolution.

# Build

To build the kernel module, run:

```bash
$ make
```

This should generate a kernel module file named "proxy_mixer.ko".

## Build again

You must clean the build before re-compiling the module.

```bash
$ make clean
```

Afterwards re-run make to do the actual build.

# install

To install the module, run "make install" (you might have to be 'root' to have all necessary permissions to install the module).

```bash
$ make && sudo make install
```

# RUN

Load the module as root:

```bash
$ sudo modprobe proxy_mixer
```



# Format source files (requires clang-format)
```bash
$ make clang-format
```

---

# Usage

Verify the output device was created, and check info:

```bash
$ v4l2-ctl --list-devices

$ v4l2-ctl -d /dev/videoN --info
```

Query supported formats:

```bash
$ v4l2-ctl -d /dev/videoN --list-formats-ext
```

Capture test (raw NV12)

```bash
ffmpeg -f v4l2 -input_format nv12 -video_size 1280x1440 \
       -i /dev/videoN output.mkv
```

Preview with ffplay:

```bash
ffplay -f v4l2 -video_size 1280x1440 -i /dev/videoN
```
Or use OBS.

# Default Parameters

| Parameter      | Default      | Description                        |
|----------------|--------------|------------------------------------|
| `out_width`    | 1280         | Output frame width (pixels)        |
| `out_height`   | 1440         | Output frame height = src_h × 2   |
| `src_width`    | 1280         | Per-source width                   |
| `src_height`   | 720          | Per-source height                  |
| `fps_num`      | 1            | Frame interval numerator           |
| `fps_den`      | 30           | Frame interval denominator (30fps) |

Output resolution and framerate can be changed via `VIDIOC_S_FMT` and `VIDIOC_S_PARM` before `STREAMON`.

---

# State Machine

```
IDLE  ──modprobe──►  video node created
  │
  │  UVC plug-in (×2, VID/PID match)
  │  src_ready_count: 0 → 1 → 2
  │
  ▼
IDLE (slots filled)
  │
  │  VIDIOC_STREAMON
  │    src_ready_count < 2  ──► return -ENODEV
  │    src_ready_count == 2
  │      UVC S_FMT / REQBUFS / QBUF / STREAMON
  │      kthread_run()
  ▼
STREAMING
  │
  ├── VIDIOC_STREAMOFF ──► mixer_uvc_stop() → IDLE (slots kept)
  ├── UVC unplug       ──► mixer_uvc_stop() → IDLE (slot cleared)
  └── rmmod            ──► mixer_uvc_stop() → unregister → gone
```

# Known Limitations

- VID/PID is hard-coded at compile time.
- Only NV12 pixel format is supported.
- Frame synchronization (`mixer_sync_frames`) is currently disabled.
- `MIXER_CALL_OP` bypasses V4L2 core capability checks; source `filp`/`vdev` lifetime must be guarded by `src_disc_lock` / `uvc_ctrl_lock`.


# License

GPL-2.0-only