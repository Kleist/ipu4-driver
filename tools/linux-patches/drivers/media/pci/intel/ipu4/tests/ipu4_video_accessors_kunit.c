// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the ipu6_isys_get_*() accessors in ipu6-isys-video.c.
 *
 * Each accessor branches on av->aq.vbq.type and reads either pix_fmt
 * (V4L2_BUF_TYPE_VIDEO_CAPTURE) or meta_fmt (V4L2_BUF_TYPE_META_CAPTURE)
 * with a 0-fallback for any other type. The width/height/bytesperline
 * trio also gates the meta-side read on KERNEL_VERSION(6, 10, 0); this
 * suite exercises whichever branch the kernel-under-test compiled.
 */

#include <kunit/test.h>
#include <linux/version.h>
#include <linux/videodev2.h>

#include "ipu6-isys-video.h"

#include "ipu4-compat.h"

static struct ipu6_isys_video *make_video(struct kunit *test, u32 type)
{
	struct ipu6_isys_video *av = kunit_kzalloc(test, sizeof(*av), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, av);
	av->aq.vbq.type = type;
	av->pix_fmt.pixelformat = V4L2_PIX_FMT_SGRBG10;
	av->pix_fmt.sizeimage = 0x12345;
	av->pix_fmt.bytesperline = 1280 * 2;
	av->pix_fmt.width = 1280;
	av->pix_fmt.height = 720;
	av->meta_fmt.dataformat = V4L2_META_FMT_GENERIC_8;
	av->meta_fmt.buffersize = 0x4000;
#if KERNEL_VERSION(6, 10, 0) <= LINUX_VERSION_CODE
	av->meta_fmt.bytesperline = 64;
	av->meta_fmt.width = 32;
	av->meta_fmt.height = 16;
#endif
	return av;
}

static void test_video_capture_reads_pix_fmt(struct kunit *test)
{
	struct ipu6_isys_video *av = make_video(test, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	KUNIT_EXPECT_EQ(test, ipu6_isys_get_format(av), (u32)V4L2_PIX_FMT_SGRBG10);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_data_size(av), 0x12345u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_bytes_per_line(av), 1280u * 2u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_width(av), 1280u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_height(av), 720u);
}

static void test_meta_capture_reads_meta_fmt(struct kunit *test)
{
	struct ipu6_isys_video *av = make_video(test, V4L2_BUF_TYPE_META_CAPTURE);

	KUNIT_EXPECT_EQ(test, ipu6_isys_get_format(av), (u32)V4L2_META_FMT_GENERIC_8);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_data_size(av), 0x4000u);
}

static void test_meta_capture_width_height_bpl_versioned(struct kunit *test)
{
	struct ipu6_isys_video *av = make_video(test, V4L2_BUF_TYPE_META_CAPTURE);

#if KERNEL_VERSION(6, 10, 0) <= LINUX_VERSION_CODE
	/* 6.10+: v4l2_meta_format gained width/height/bytesperline.
	 * The accessor must read them through.
	 */
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_bytes_per_line(av), 64u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_width(av), 32u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_height(av), 16u);
#else
	/* Pre-6.10: meta_fmt has none of those fields, so the accessor
	 * falls through the META branch and returns 0.
	 */
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_bytes_per_line(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_width(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_height(av), 0u);
#endif
}

static void test_unrecognized_type_returns_zero(struct kunit *test)
{
	/* V4L2_BUF_TYPE_VIDEO_OUTPUT is never used by ipu4 — the
	 * accessor must return 0 from every helper rather than reading
	 * a stale field.
	 */
	struct ipu6_isys_video *av = make_video(test, V4L2_BUF_TYPE_VIDEO_OUTPUT);

	KUNIT_EXPECT_EQ(test, ipu6_isys_get_format(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_data_size(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_bytes_per_line(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_width(av), 0u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_get_frame_height(av), 0u);
}

static struct kunit_case ipu4_video_accessors_cases[] = {
	KUNIT_CASE(test_video_capture_reads_pix_fmt),
	KUNIT_CASE(test_meta_capture_reads_meta_fmt),
	KUNIT_CASE(test_meta_capture_width_height_bpl_versioned),
	KUNIT_CASE(test_unrecognized_type_returns_zero),
	{}
};

static struct kunit_suite ipu4_video_accessors_suite = {
	.name = "ipu4_video_accessors",
	.test_cases = ipu4_video_accessors_cases,
};
kunit_test_suite(ipu4_video_accessors_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 video pix/meta accessors");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
