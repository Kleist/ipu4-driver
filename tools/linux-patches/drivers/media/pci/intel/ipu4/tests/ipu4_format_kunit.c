// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for ipu6_isys_get_isys_format().
 *
 * The lookup walks ipu6_isys_pfmts[] and returns a matching entry for
 * (pixelformat, V4L2_BUF_TYPE_*). These tests pin the behavior for a
 * representative set of entries so a rebase that reorders the table or
 * flips the is_meta bits fails loudly.
 */

#include <kunit/test.h>
#include <linux/videodev2.h>
#include <media/v4l2-mediabus.h>

#include "ipu6-isys-video.h"

extern const struct ipu6_isys_pixelformat ipu6_isys_pfmts[];

static void test_sgrbg10_found_for_video_capture(struct kunit *test)
{
	const struct ipu6_isys_pixelformat *p;

	p = ipu6_isys_get_isys_format(V4L2_PIX_FMT_SGRBG10,
				      V4L2_BUF_TYPE_VIDEO_CAPTURE);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->pixelformat, (u32)V4L2_PIX_FMT_SGRBG10);
	KUNIT_EXPECT_FALSE(test, p->is_meta);
}

static void test_meta_format_for_meta_capture(struct kunit *test)
{
	const struct ipu6_isys_pixelformat *p;

	p = ipu6_isys_get_isys_format(V4L2_META_FMT_GENERIC_8,
				      V4L2_BUF_TYPE_META_CAPTURE);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_TRUE(test, p->is_meta);
}

static void test_wrong_buftype_returns_default(struct kunit *test)
{
	const struct ipu6_isys_pixelformat *p;

	/* A video pixelformat queried against META_CAPTURE should fall
	 * back to the first meta entry, not to the matching video entry.
	 */
	p = ipu6_isys_get_isys_format(V4L2_PIX_FMT_SGRBG10,
				      V4L2_BUF_TYPE_META_CAPTURE);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_TRUE(test, p->is_meta);
}

static void test_unknown_pixfmt_returns_default(struct kunit *test)
{
	const struct ipu6_isys_pixelformat *p;

	p = ipu6_isys_get_isys_format(0xdeadbeef,
				      V4L2_BUF_TYPE_VIDEO_CAPTURE);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_FALSE(test, p->is_meta);
}

static void test_type_zero_means_any(struct kunit *test)
{
	const struct ipu6_isys_pixelformat *p;

	/* type == 0 skips the is_meta filter: any exact pixfmt match wins. */
	p = ipu6_isys_get_isys_format(V4L2_PIX_FMT_SGRBG10, 0);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_EXPECT_EQ(test, p->pixelformat, (u32)V4L2_PIX_FMT_SGRBG10);
}

static struct kunit_case ipu4_format_cases[] = {
	KUNIT_CASE(test_sgrbg10_found_for_video_capture),
	KUNIT_CASE(test_meta_format_for_meta_capture),
	KUNIT_CASE(test_wrong_buftype_returns_default),
	KUNIT_CASE(test_unknown_pixfmt_returns_default),
	KUNIT_CASE(test_type_zero_means_any),
	{}
};

static struct kunit_suite ipu4_format_suite = {
	.name = "ipu4_format",
	.test_cases = ipu4_format_cases,
};
kunit_test_suite(ipu4_format_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 pixel format lookup");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
