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
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

#include <linux/media.h>
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

/* Locate the media entity that wraps /dev/videoN by matching the v4l2
 * minor number. Returns the entity id, or 0 on failure.
 */
static __u32 find_video_entity(int mfd, unsigned int want_minor)
{
	__u32 fallback_by_name = 0;
	for (__u32 id = 1; ; id++) {
		struct media_entity_desc ent = { .id = id | MEDIA_ENT_ID_FLAG_NEXT };
		if (ioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
			break;
		id = ent.id;
		if (ent.dev.major == 81 && ent.dev.minor == want_minor)
			return ent.id;
		if (!fallback_by_name &&
		    strcmp(ent.name, "Intel IPU4 ISYS Capture 1") == 0)
			fallback_by_name = ent.id;
	}
	/* Older kernel releases leave ent.dev unset for v4l-io entities.
	 * Fall back to the name the IPU driver gives /dev/video0. */
	return fallback_by_name;
}

/* Enable the media links that connect the virt-sensor's SOURCE pad,
 * through the IPU CSI2 RX subdev, to the video entity behind
 * /dev/video0. The virt-sensor → CSI2 link is already enabled +
 * immutable. What's missing is the CSI2-pad-N → Capture-1 link, which
 * is dynamic and default-disabled. Enabling every link in the graph
 * causes ENOTUNIQ (IPU6's isys publishes dozens of virtual sinks and
 * a sink pad can't take more than one active link); enabling just the
 * single one we need keeps the graph unambiguous.
 */
static int enable_media_links(const char *media_dev, const char *video_dev)
{
	int mfd = open(media_dev, O_RDWR);
	if (mfd < 0) {
		printf("MEDIA:fail step=open errno=%d (%s) dev=%s\n",
		       errno, strerror(errno), media_dev);
		return -1;
	}
	printf("MEDIA:open dev=%s\n", media_dev);

	struct stat vst;
	if (stat(video_dev, &vst) < 0) {
		printf("MEDIA:fail step=stat_video errno=%d dev=%s\n",
		       errno, video_dev);
		close(mfd);
		return -1;
	}
	__u32 video_entity = find_video_entity(mfd, minor(vst.st_rdev));
	if (!video_entity) {
		printf("MEDIA:fail step=find_video_entity minor=%u\n",
		       minor(vst.st_rdev));
		close(mfd);
		return -1;
	}
	printf("MEDIA:video %s = entity id=%u\n", video_dev, video_entity);

	/* Walk every entity that has outbound links to this video entity
	 * and enable one of them (prefer source.index == 1, then 2, 3, 4).
	 * The first enabled link wins — subsequent attempts on the same
	 * sink would ENOTUNIQ.
	 */
	int enabled = 0;
	for (__u32 id = 1; ; id++) {
		struct media_entity_desc ent = { .id = id | MEDIA_ENT_ID_FLAG_NEXT };
		if (ioctl(mfd, MEDIA_IOC_ENUM_ENTITIES, &ent) < 0)
			break;
		id = ent.id;

		if (ent.links == 0)
			continue;

		struct media_pad_desc *pads = calloc(ent.pads, sizeof(*pads));
		struct media_link_desc *links = calloc(ent.links, sizeof(*links));
		struct media_links_enum le = {
			.entity = ent.id, .pads = pads, .links = links,
		};
		if (ioctl(mfd, MEDIA_IOC_ENUM_LINKS, &le) < 0) {
			free(pads);
			free(links);
			continue;
		}

		/* Try pad 1 first, then 2, 3, 4. Pick the first
		 * enableable candidate that targets our video_entity. */
		for (__u32 prefer = 1; prefer <= 4 && !enabled; prefer++) {
			for (__u32 i = 0; i < ent.links; i++) {
				struct media_link_desc *l = &links[i];
				if (l->source.entity != ent.id)
					continue;
				if (l->sink.entity != video_entity)
					continue;
				if (l->source.index != prefer)
					continue;
				if (l->flags & MEDIA_LNK_FL_ENABLED) {
					printf("MEDIA:already_enabled %s:%u -> %u:%u\n",
					       ent.name, l->source.index,
					       l->sink.entity, l->sink.index);
					enabled = 1;
					break;
				}
				if (l->flags & MEDIA_LNK_FL_IMMUTABLE)
					continue;
				l->flags |= MEDIA_LNK_FL_ENABLED;
				if (ioctl(mfd, MEDIA_IOC_SETUP_LINK, l) < 0) {
					printf("MEDIA:link_fail %s:%u -> %u:%u errno=%d (%s)\n",
					       ent.name, l->source.index,
					       l->sink.entity, l->sink.index,
					       errno, strerror(errno));
					continue;
				}
				printf("MEDIA:link_enabled %s:%u -> sink=%u:%u (video entity)\n",
				       ent.name, l->source.index,
				       l->sink.entity, l->sink.index);
				enabled = 1;
				break;
			}
		}
		free(pads);
		free(links);
		if (enabled)
			break;
	}

	if (!enabled)
		printf("MEDIA:no_link_enabled (nothing points at video entity %u)\n",
		       video_entity);
	close(mfd);
	return 0;
}

int main(int argc, char **argv)
{
	const char *dev = argc > 1 ? argv[1] : "/dev/video0";
	const char *media_dev = argc > 2 ? argv[2] : "/dev/media0";
	int fd, i;

	/* Best-effort media-link enable. STREAMON fails with ENOLINK
	 * if the virt-sensor -> CSI2 -> ISA pipeline isn't linked. */
	enable_media_links(media_dev, dev);

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
