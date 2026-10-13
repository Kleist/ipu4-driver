/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * v4l2-query — call the ISYS capture node's and CSI-2 subdev's query and
 * format ioctls that do not touch the hardware (format and frame size
 * enumeration, try/set format, CREATE_BUFS sizing, mbus code enumeration,
 * crop selection, event subscription). Used by the ipu6 ladder's
 * v4l2-query test (tools/tests/ipu6/tests.d/090-v4l2-query.sh).
 *
 * Prints "QUERY:<step> ok [detail]" or "QUERY:<step> fail errno=N" per
 * step and always exits 0; the host decides which steps must pass.
 *
 * Usage: v4l2-query VIDEO SUBDEV
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

/* The CSI-2 receiver's sink pad and its first source pad. */
#define CSI2_PAD_SINK	0
#define CSI2_PAD_SRC	1

static int step(int fd, unsigned long req, void *arg, const char *name)
{
	if (ioctl(fd, req, arg) < 0) {
		printf("QUERY:%s fail errno=%d\n", name, errno);
		return -1;
	}
	return 0;
}

static void query_video(int fd)
{
	struct v4l2_capability cap = { 0 };
	struct v4l2_fmtdesc fd_desc;
	struct v4l2_frmsizeenum fs;
	struct v4l2_format fmt;
	struct v4l2_create_buffers cb;
	unsigned int n, sizes = 0;
	__u32 type;

	if (!step(fd, VIDIOC_QUERYCAP, &cap, "querycap"))
		printf("QUERY:querycap ok %s\n", cap.card);

	for (type = V4L2_BUF_TYPE_VIDEO_CAPTURE;;
	     type = V4L2_BUF_TYPE_META_CAPTURE) {
		const char *t = type == V4L2_BUF_TYPE_VIDEO_CAPTURE ?
				"vid" : "meta";
		char name[32];

		for (n = 0;; n++) {
			memset(&fd_desc, 0, sizeof(fd_desc));
			fd_desc.index = n;
			fd_desc.type = type;
			if (ioctl(fd, VIDIOC_ENUM_FMT, &fd_desc) < 0)
				break;
			memset(&fs, 0, sizeof(fs));
			fs.pixel_format = fd_desc.pixelformat;
			if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fs) == 0)
				sizes++;
		}
		printf("QUERY:enum_fmt_%s %s n=%u\n", t, n ? "ok" : "fail", n);

		memset(&fmt, 0, sizeof(fmt));
		fmt.type = type;
		snprintf(name, sizeof(name), "g_fmt_%s", t);
		if (step(fd, VIDIOC_G_FMT, &fmt, name))
			goto next;
		printf("QUERY:%s ok\n", name);
		snprintf(name, sizeof(name), "try_fmt_%s", t);
		if (!step(fd, VIDIOC_TRY_FMT, &fmt, name))
			printf("QUERY:%s ok\n", name);
		snprintf(name, sizeof(name), "s_fmt_%s", t);
		if (!step(fd, VIDIOC_S_FMT, &fmt, name))
			printf("QUERY:%s ok\n", name);
		if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			/* count 0 only asks the driver to size the buffers. */
			memset(&cb, 0, sizeof(cb));
			cb.memory = V4L2_MEMORY_MMAP;
			cb.format = fmt;
			if (!step(fd, VIDIOC_CREATE_BUFS, &cb, "create_bufs"))
				printf("QUERY:create_bufs ok\n");
		}
next:
		if (type == V4L2_BUF_TYPE_META_CAPTURE)
			break;
	}
	printf("QUERY:enum_framesizes %s n=%u\n", sizes ? "ok" : "fail", sizes);
}

static void query_subdev(int fd)
{
	struct v4l2_subdev_mbus_code_enum code;
	struct v4l2_event_subscription sub;
	struct v4l2_subdev_selection sel;
	static const struct {
		__u32 target;
		const char *name;
	} targets[] = {
		{ V4L2_SEL_TGT_CROP, "g_sel_crop" },
		{ V4L2_SEL_TGT_CROP_DEFAULT, "g_sel_crop_default" },
		{ V4L2_SEL_TGT_CROP_BOUNDS, "g_sel_crop_bounds" },
	};
	unsigned int i, n;

	for (n = 0;; n++) {
		memset(&code, 0, sizeof(code));
		code.pad = CSI2_PAD_SINK;
		code.index = n;
		code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		if (ioctl(fd, VIDIOC_SUBDEV_ENUM_MBUS_CODE, &code) < 0)
			break;
	}
	printf("QUERY:enum_mbus_code %s n=%u\n", n ? "ok" : "fail", n);

	memset(&sub, 0, sizeof(sub));
	sub.type = V4L2_EVENT_FRAME_SYNC;
	if (!step(fd, VIDIOC_SUBSCRIBE_EVENT, &sub, "subscribe_event") &&
	    !step(fd, VIDIOC_UNSUBSCRIBE_EVENT, &sub, "subscribe_event"))
		printf("QUERY:subscribe_event ok\n");

	for (i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
		memset(&sel, 0, sizeof(sel));
		sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		sel.pad = CSI2_PAD_SRC;
		sel.target = targets[i].target;
		if (!step(fd, VIDIOC_SUBDEV_G_SELECTION, &sel, targets[i].name))
			printf("QUERY:%s ok %ux%u\n", targets[i].name,
			       sel.r.width, sel.r.height);
	}

	/* Crop to the full frame, which leaves the source format as is. */
	memset(&sel, 0, sizeof(sel));
	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = CSI2_PAD_SRC;
	sel.target = V4L2_SEL_TGT_CROP_BOUNDS;
	if (ioctl(fd, VIDIOC_SUBDEV_G_SELECTION, &sel) == 0) {
		sel.target = V4L2_SEL_TGT_CROP;
		if (!step(fd, VIDIOC_SUBDEV_S_SELECTION, &sel, "s_sel_crop"))
			printf("QUERY:s_sel_crop ok %ux%u\n",
			       sel.r.width, sel.r.height);
	}
}

int main(int argc, char **argv)
{
	int fd;

	if (argc != 3) {
		fprintf(stderr, "usage: %s VIDEO SUBDEV\n", argv[0]);
		return 2;
	}
	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("QUERY:open_video fail errno=%d\n", errno);
	} else {
		query_video(fd);
		close(fd);
	}
	fd = open(argv[2], O_RDWR);
	if (fd < 0) {
		printf("QUERY:open_subdev fail errno=%d\n", errno);
	} else {
		query_subdev(fd);
		close(fd);
	}
	return 0;
}
