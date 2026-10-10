// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for IPU4 buttress arithmetic and the IPU4-specific
 * buttress control descriptors.
 *
 * Two scopes here:
 *  - ipu6_buttress_tsc_ticks_to_ns(): multiplies by 10000 and divides
 *    by 192 (IPU4 TSC runs at 19.2 MHz; derivation in ipu6-buttress.c).
 *    Pinned at one-tick, one-period, near-overflow, and truncation.
 *  - ipu4_isys_buttress_ctrl / ipu4_psys_buttress_ctrl: every field
 *    pinned against the IPU4 platform constants. The struct uses
 *    designated initializers, so what these cases catch is value
 *    drift — a constant changing in the header, a dropped
 *    initializer leaving a field zero-initialised, or an IS/PS
 *    copy-paste collision. Struct-layout reorders are compiler
 *    no-ops and aren't gated here. The IPU4/IPU6 unification refactor
 *    will revive #ifdef IPU6 dead branches and rename `ipu4_*`
 *    symbols; any binding change has to fail here, not slip into
 *    vm-smoke.
 */

#include <kunit/test.h>
#include <linux/types.h>

#include "ipu4-platform-buttress-regs.h"
#include "ipu6.h"
#include "ipu6-buttress.h"
#include "ipu6-platform-buttress-regs.h"

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

static void test_ipu4_isys_buttress_ctrl_fields(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.ratio,
			(unsigned int)IPU4_IS_FREQ_CTL_DIVISOR);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.qos_floor, 0u);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.freq_ctl,
			(u32)IPU6_BUTTRESS_REG_IS_FREQ_CTL);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.pwr_sts_shift,
			(u32)IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_SHIFT);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.pwr_sts_mask,
			(u32)IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_MASK);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.pwr_sts_on,
			(u32)IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IS_RDY);
	KUNIT_EXPECT_EQ(test, ipu4_isys_buttress_ctrl.pwr_sts_off,
			(u32)IPU4_BUTTRESS_PWR_STATE_IS_PWR_FSM_IDLE);
}

static void test_ipu4_psys_buttress_ctrl_fields(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.ratio,
			(unsigned int)IPU4_PS_FREQ_CTL_DEFAULT_RATIO);
	/* qos_floor and ratio share the same default on PS — that's
	 * deliberate (silicon ships with no QoS headroom). Pin it so a
	 * "looks redundant, fix it" cleanup during the refactor fails
	 * here instead of silently changing thermal behaviour.
	 */
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.qos_floor,
			(unsigned int)IPU4_PS_FREQ_CTL_DEFAULT_RATIO);
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.freq_ctl,
			(u32)IPU6_BUTTRESS_REG_PS_FREQ_CTL);
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.pwr_sts_shift,
			(u32)IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_SHIFT);
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.pwr_sts_mask,
			(u32)IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_MASK);
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.pwr_sts_on,
			(u32)IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_PS_PWR_UP);
	KUNIT_EXPECT_EQ(test, ipu4_psys_buttress_ctrl.pwr_sts_off,
			(u32)IPU4_BUTTRESS_PWR_STATE_PS_PWR_FSM_IDLE);
}

static void test_ipu4_isys_vs_psys_distinct(struct kunit *test)
{
	/* The IS and PS descriptors are deliberately distinct: different
	 * frequency-control registers and non-overlapping PWR_STATE FSM
	 * bit ranges. A copy-paste during ifdef removal that fuses them
	 * would silently merge two independent power domains.
	 */
	KUNIT_EXPECT_NE(test, ipu4_isys_buttress_ctrl.freq_ctl,
			ipu4_psys_buttress_ctrl.freq_ctl);
	KUNIT_EXPECT_NE(test, ipu4_isys_buttress_ctrl.pwr_sts_shift,
			ipu4_psys_buttress_ctrl.pwr_sts_shift);
	KUNIT_EXPECT_EQ(test,
			ipu4_isys_buttress_ctrl.pwr_sts_mask &
			ipu4_psys_buttress_ctrl.pwr_sts_mask, 0u);
}

static struct kunit_case ipu4_buttress_math_cases[] = {
	KUNIT_CASE(test_zero_ticks_is_zero_ns),
	KUNIT_CASE(test_one_tick_is_52_ns_truncated),
	KUNIT_CASE(test_one_full_period_is_10000_ns),
	KUNIT_CASE(test_truncation_below_one_period),
	KUNIT_CASE(test_one_million_ticks),
	KUNIT_CASE(test_no_overflow_at_192e9_ticks),
	KUNIT_CASE(test_ipu4_isys_buttress_ctrl_fields),
	KUNIT_CASE(test_ipu4_psys_buttress_ctrl_fields),
	KUNIT_CASE(test_ipu4_isys_vs_psys_distinct),
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
