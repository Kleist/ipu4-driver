// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for ipu6_isys_mbus_code_to_bpp() and
 * ipu6_isys_mbus_code_to_mipi().
 *
 * Both helpers are big switch statements over MEDIA_BUS_FMT_* codes
 * with a WARN_ON-default. Pin one case per output bucket plus the
 * default fallback so a future format-table edit can't silently
 * misroute a code to the wrong bpp / DT class.
 */

#include <kunit/test.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-mediabus.h>

#include "ipu6-isys-subdev.h"

static void test_bpp_24(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_RGB888_1X24), 24u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_META_24), 24u);
}

static void test_bpp_16(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_RGB565_1X16), 16u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_UYVY8_1X16), 16u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_Y16_1X16), 16u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_META_16), 16u);
}

static void test_bpp_12(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SBGGR12_1X12), 12u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SRGGB12_1X12), 12u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_Y12_1X12), 12u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_META_12), 12u);
}

static void test_bpp_10(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SBGGR10_1X10), 10u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SGRBG10_1X10), 10u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_Y10_1X10), 10u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_META_10), 10u);
}

static void test_bpp_8(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SBGGR8_1X8), 8u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_SRGGB8_1X8), 8u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_Y8_1X8), 8u);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(MEDIA_BUS_FMT_META_8), 8u);
}

static void test_bpp_unknown_warns_and_returns_8(struct kunit *test)
{
	/* WARN_ON fires; KUnit captures the WARN and the return value
	 * is the safe-default 8. The kunit harness treats WARN as
	 * non-fatal here because we're explicitly exercising the
	 * default arm.
	 */
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_bpp(0xdead0000), 8u);
}

static void test_mipi_rgb565(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_RGB565_1X16),
			(unsigned int)MIPI_CSI2_DT_RGB565);
}

static void test_mipi_rgb888(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_RGB888_1X24),
			(unsigned int)MIPI_CSI2_DT_RGB888);
}

static void test_mipi_yuv422_8b(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_UYVY8_1X16),
			(unsigned int)MIPI_CSI2_DT_YUV422_8B);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_YUYV8_1X16),
			(unsigned int)MIPI_CSI2_DT_YUV422_8B);
}

static void test_mipi_raw16(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_SBGGR16_1X16),
			(unsigned int)MIPI_CSI2_DT_RAW16);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_Y16_1X16),
			(unsigned int)MIPI_CSI2_DT_RAW16);
}

static void test_mipi_raw_bayer_chain(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_SRGGB12_1X12),
			(unsigned int)MIPI_CSI2_DT_RAW12);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_SGRBG10_1X10),
			(unsigned int)MIPI_CSI2_DT_RAW10);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_SBGGR8_1X8),
			(unsigned int)MIPI_CSI2_DT_RAW8);
}

static void test_mipi_meta_embedded(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_META_8),
			(unsigned int)MIPI_CSI2_DT_EMBEDDED_8B);
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(MEDIA_BUS_FMT_META_24),
			(unsigned int)MIPI_CSI2_DT_EMBEDDED_8B);
}

static void test_mipi_unknown_warns_and_returns_3f(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_isys_mbus_code_to_mipi(0xdead0000), 0x3fu);
}

static struct kunit_case ipu4_format_helpers_cases[] = {
	KUNIT_CASE(test_bpp_24),
	KUNIT_CASE(test_bpp_16),
	KUNIT_CASE(test_bpp_12),
	KUNIT_CASE(test_bpp_10),
	KUNIT_CASE(test_bpp_8),
	KUNIT_CASE(test_bpp_unknown_warns_and_returns_8),
	KUNIT_CASE(test_mipi_rgb565),
	KUNIT_CASE(test_mipi_rgb888),
	KUNIT_CASE(test_mipi_yuv422_8b),
	KUNIT_CASE(test_mipi_raw16),
	KUNIT_CASE(test_mipi_raw_bayer_chain),
	KUNIT_CASE(test_mipi_meta_embedded),
	KUNIT_CASE(test_mipi_unknown_warns_and_returns_3f),
	{}
};

static struct kunit_suite ipu4_format_helpers_suite = {
	.name = "ipu4_format_helpers",
	.test_cases = ipu4_format_helpers_cases,
};
kunit_test_suite(ipu4_format_helpers_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 mbus-code conversion helpers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
