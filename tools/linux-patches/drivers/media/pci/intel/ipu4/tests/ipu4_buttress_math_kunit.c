// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for ipu6_buttress_tsc_ticks_to_ns().
 *
 * The function multiplies by 10000 and divides by 192 (the IPU4 TSC
 * runs at 19.2 MHz; the comment in ipu6-buttress.c spells out the
 * derivation). It ignores its `isp` argument so we pass NULL. Pin the
 * arithmetic at one-tick, one-period, a near-overflow value, and
 * truncation.
 */

#include <kunit/test.h>
#include <linux/types.h>

#include "ipu6.h"
#include "ipu6-buttress.h"

static void test_zero_ticks_is_zero_ns(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu6_buttress_tsc_ticks_to_ns(0, NULL), 0ull);
}

static void test_one_tick_is_52_ns_truncated(struct kunit *test)
{
	/* 1 tick * 10000 / 192 = 52.083... -> 52 ns (div_u64 truncates). */
	KUNIT_EXPECT_EQ(test, ipu6_buttress_tsc_ticks_to_ns(1, NULL), 52ull);
}

static void test_one_full_period_is_10000_ns(struct kunit *test)
{
	/* 192 ticks at 19.2 MHz = exactly 10 microseconds = 10000 ns. */
	KUNIT_EXPECT_EQ(test, ipu6_buttress_tsc_ticks_to_ns(192, NULL), 10000ull);
}

static void test_truncation_below_one_period(struct kunit *test)
{
	/* 191 ticks * 10000 / 192 = 9947.91... -> 9947 ns. */
	KUNIT_EXPECT_EQ(test, ipu6_buttress_tsc_ticks_to_ns(191, NULL), 9947ull);
}

static void test_one_million_ticks(struct kunit *test)
{
	/* 1_000_000 * 10000 / 192 = 52083333 ns ≈ 52.08 ms. */
	KUNIT_EXPECT_EQ(test, ipu6_buttress_tsc_ticks_to_ns(1000000, NULL),
			52083333ull);
}

static void test_no_overflow_at_192e9_ticks(struct kunit *test)
{
	/* 192 * 10^9 ticks * 10000 / 192 = 10^13 ns = 10 000 seconds.
	 * The intermediate product is 1.92e15, well above u32 range and
	 * the u64 div_u64 truncates only on the divisor side; this
	 * pins both that the multiplication is in u64 and that the
	 * exact-multiple case has no fencepost loss.
	 */
	KUNIT_EXPECT_EQ(test,
			ipu6_buttress_tsc_ticks_to_ns(192000000000ull, NULL),
			10000000000000ull);
}

static struct kunit_case ipu4_buttress_math_cases[] = {
	KUNIT_CASE(test_zero_ticks_is_zero_ns),
	KUNIT_CASE(test_one_tick_is_52_ns_truncated),
	KUNIT_CASE(test_one_full_period_is_10000_ns),
	KUNIT_CASE(test_truncation_below_one_period),
	KUNIT_CASE(test_one_million_ticks),
	KUNIT_CASE(test_no_overflow_at_192e9_ticks),
	{}
};

static struct kunit_suite ipu4_buttress_math_suite = {
	.name = "ipu4_buttress_math",
	.test_cases = ipu4_buttress_math_cases,
};
kunit_test_suite(ipu4_buttress_math_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 buttress TSC arithmetic");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
