# Camera Supported Resolutions & FPS in Ubuntu (v4l2-ctl)

This guide explains how to check all supported resolutions and maximum frame rates (FPS) of camera devices connected to Ubuntu using the `v4l2-ctl` tool.

---

## 1. Install v4l2-ctl

Install the `v4l-utils` package, which includes `v4l2-ctl`:
`sudo apt update
sudo apt install v4l-utils`

---

## 2. List Connected Cameras

Show all available camera devices:

`v4l2-ctl --list-devices`

Typical output shows `/dev/video0`, `/dev/video1`, etc.

---

## 3. Check Supported Formats, Resolutions, and FPS

To display all supported pixel formats, resolutions, and frame intervals (FPS) for a camera device (example: `/dev/video_cam2`):
`v4l2-ctl -d /dev/video_cam2 --list-formats-ext
`
*Replace `/dev/video_cam2` with your actual device (e.g., `/dev/video0`).*

---

## 4. Example Output (Formatted)

Given output:
`ioctl: VIDIOC_ENUM_FMT
Type: Video Capture
: 'NV12' (Y/CbCr 4:2:0)
	Size: Discrete 1920x1080
		Interval: Discrete 0.017s (59.940 fps)
: 'YUYV' (YUYV 4:2:2)[12]
	Size: Discrete 1920x1080
		Interval: Discrete 0.017s (59.940 fps)`

---

## 5. Markdown Table Example

| Format | Description   | Resolution   | Max FPS     |
|--------|--------------|--------------|-------------|
| NV12   | Y/CbCr 4:2:0 | 1920x1080    | 59.940 fps  |
| YUYV   | YUYV 4:2:2   | 1920x1080    | 59.940 fps  |

---

## References

- [Ubuntu v4l2-ctl documentation][web:13][web:31][web:40]
- [Community usage examples][web:9][web:10]

