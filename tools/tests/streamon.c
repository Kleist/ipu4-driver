/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * streamon-attempt — minimal v4l2 client used by tools/tests/streamon-smoke.sh.
 *
 * Walks the V4L2 capture API one step at a time and prints a marker
 * after each ioctl: "STREAM:querycap", "STREAM:s_fmt", "STREAM:reqbufs",
 * "STREAM:qbuf", "STREAM:streamon", "STREAM:dqbuf" on success, or
 * "STREAM:fail step=NAME errno=N" on the first failure.
 *
 * Built statically (musl/-static) so it runs in the busybox initramfs
 * without any libc loader.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <unistd.h>

#include <linux/videodev2.h>

#define WIDTH		800
#define HEIGHT		800
#define PIXFMT		v4l2_fourcc('R', 'G', 'B', '3')	/* RGB24 */
#define NUM_BUFS	2

static int xioctl(int fd, unsigned long req, void *arg, const char *step)
{
	int r = ioctl(fd, req, arg);
	if (r < 0) {
		printf("STREAM:fail step=%s errno=%d (%s)\n",
		       step, errno, strerror(errno));
		return -1;
	}
	printf("STREAM:%s\n", step);
	return 0;
}

int main(int argc, char **argv)
{
	const char *dev = argc > 1 ? argv[1] : "/dev/video0";
	int fd, i;

	fd = open(dev, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		printf("STREAM:fail step=open errno=%d (%s)\n",
		       errno, strerror(errno));
		return 1;
	}
	printf("STREAM:open dev=%s\n", dev);

	struct v4l2_capability cap = {0};
	if (xioctl(fd, VIDIOC_QUERYCAP, &cap, "querycap") < 0)
		return 1;

	struct v4l2_format fmt = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width = WIDTH,
			.height = HEIGHT,
			.pixelformat = PIXFMT,
			.field = V4L2_FIELD_NONE,
		},
	};
	if (xioctl(fd, VIDIOC_S_FMT, &fmt, "s_fmt") < 0)
		return 1;

	struct v4l2_requestbuffers reqbufs = {
		.count = NUM_BUFS,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	if (xioctl(fd, VIDIOC_REQBUFS, &reqbufs, "reqbufs") < 0)
		return 1;

	for (i = 0; i < NUM_BUFS; i++) {
		struct v4l2_buffer buf = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
			.memory = V4L2_MEMORY_MMAP,
			.index = i,
		};
		if (xioctl(fd, VIDIOC_QUERYBUF, &buf, "querybuf") < 0)
			return 1;
		void *p = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
			       MAP_SHARED, fd, buf.m.offset);
		if (p == MAP_FAILED) {
			printf("STREAM:fail step=mmap errno=%d (%s)\n",
			       errno, strerror(errno));
			return 1;
		}
		if (xioctl(fd, VIDIOC_QBUF, &buf, "qbuf") < 0)
			return 1;
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(fd, VIDIOC_STREAMON, &type, "streamon") < 0)
		return 1;

	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	int pr = poll(&pfd, 1, 2000);
	if (pr <= 0) {
		printf("STREAM:fail step=dqbuf_poll errno=%d (timeout=%d)\n",
		       errno, pr == 0);
		return 1;
	}

	struct v4l2_buffer dq = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.memory = V4L2_MEMORY_MMAP,
	};
	if (xioctl(fd, VIDIOC_DQBUF, &dq, "dqbuf") < 0)
		return 1;

	printf("STREAM:done bytes=%u\n", dq.bytesused);
	return 0;
}
