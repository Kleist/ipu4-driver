// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for ipu6_isys_is_bayer_format() and
 * ipu6_isys_convert_bayer_order().
 *
 * convert_bayer_order() rotates the Bayer pattern by (x, y) mod 2 via
 * a code_map[] XOR trick. These tests pin the full 4-way rotation for
 * every supported bit depth.
 */

#include <kunit/test.h>
#include <media/v4l2-mediabus.h>

#include "ipu6-isys-subdev.h"

struct bayer_rotation_case {
	u32 in;
	u32 out_x1y0;
	u32 out_x0y1;
	u32 out_x1y1;
};

static const struct bayer_rotation_case rggb_rotations[] = {
	{
		.in       = MEDIA_BUS_FMT_SRGGB8_1X8,
		.out_x1y0 = MEDIA_BUS_FMT_SGRBG8_1X8,
		.out_x0y1 = MEDIA_BUS_FMT_SGBRG8_1X8,
		.out_x1y1 = MEDIA_BUS_FMT_SBGGR8_1X8,
	},
	{
		.in       = MEDIA_BUS_FMT_SRGGB10_1X10,
		.out_x1y0 = MEDIA_BUS_FMT_SGRBG10_1X10,
		.out_x0y1 = MEDIA_BUS_FMT_SGBRG10_1X10,
		.out_x1y1 = MEDIA_BUS_FMT_SBGGR10_1X10,
	},
	{
		.in       = MEDIA_BUS_FMT_SRGGB12_1X12,
		.out_x1y0 = MEDIA_BUS_FMT_SGRBG12_1X12,
		.out_x0y1 = MEDIA_BUS_FMT_SGBRG12_1X12,
		.out_x1y1 = MEDIA_BUS_FMT_SBGGR12_1X12,
	},
	{
		.in       = MEDIA_BUS_FMT_SRGGB16_1X16,
		.out_x1y0 = MEDIA_BUS_FMT_SGRBG16_1X16,
		.out_x0y1 = MEDIA_BUS_FMT_SGBRG16_1X16,
		.out_x1y1 = MEDIA_BUS_FMT_SBGGR16_1X16,
	},
};

static void test_rotate_all_rggb_variants(struct kunit *test)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(rggb_rotations); i++) {
		const struct bayer_rotation_case *c = &rggb_rotations[i];

		KUNIT_EXPECT_EQ(test, ipu6_isys_convert_bayer_order(c->in, 0, 0), c->in);
		KUNIT_EXPECT_EQ(test, ipu6_isys_convert_bayer_order(c->in, 1, 0), c->out_x1y0);
		KUNIT_EXPECT_EQ(test, ipu6_isys_convert_bayer_order(c->in, 0, 1), c->out_x0y1);
		KUNIT_EXPECT_EQ(test, ipu6_isys_convert_bayer_order(c->in, 1, 1), c->out_x1y1);
	}
}

static void test_rotate_is_involutive(struct kunit *test)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(rggb_rotations); i++) {
		u32 in = rggb_rotations[i].in;
		u32 mid = ipu6_isys_convert_bayer_order(in, 1, 1);

		KUNIT_EXPECT_EQ(test, ipu6_isys_convert_bayer_order(mid, 1, 1), in);
	}
}

static void test_is_bayer_for_raw_types(struct kunit *test)
{
	KUNIT_EXPECT_TRUE(test,  ipu6_isys_is_bayer_format(MEDIA_BUS_FMT_SRGGB10_1X10));
	KUNIT_EXPECT_TRUE(test,  ipu6_isys_is_bayer_format(MEDIA_BUS_FMT_SGRBG8_1X8));
	KUNIT_EXPECT_TRUE(test,  ipu6_isys_is_bayer_format(MEDIA_BUS_FMT_SBGGR12_1X12));
	KUNIT_EXPECT_FALSE(test, ipu6_isys_is_bayer_format(MEDIA_BUS_FMT_UYVY8_1X16));
	KUNIT_EXPECT_FALSE(test, ipu6_isys_is_bayer_format(MEDIA_BUS_FMT_RGB888_1X24));
}

static struct kunit_case ipu4_bayer_cases[] = {
	KUNIT_CASE(test_rotate_all_rggb_variants),
	KUNIT_CASE(test_rotate_is_involutive),
	KUNIT_CASE(test_is_bayer_for_raw_types),
	{}
};

static struct kunit_suite ipu4_bayer_suite = {
	.name = "ipu4_bayer",
	.test_cases = ipu4_bayer_cases,
};
kunit_test_suite(ipu4_bayer_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 Bayer-order helpers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
