// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for ipu6_mmu_pgsize().
 *
 * The helper picks the largest power-of-two page size that fits in 'size'
 * and is compatible with 'addr_merge' alignment, constrained by the page
 * size bitmap. It is currently static in ipu6-mmu.c; landing these tests
 * requires making it non-static (or exposing via a private header). That
 * one-line change is tracked in tools/notes/registers.md under "KUnit
 * exposure".
 */

#include <kunit/test.h>
#include <linux/sizes.h>
#include <linux/types.h>

size_t ipu6_mmu_pgsize(unsigned long pgsize_bitmap,
		       unsigned long addr_merge, size_t size);

#define BITMAP_4K_2M	(SZ_4K | SZ_2M)
#define BITMAP_4K	(SZ_4K)

static void test_picks_4k_for_4k_size(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
		ipu6_mmu_pgsize(BITMAP_4K_2M, SZ_4K, SZ_4K),
		(size_t)SZ_4K);
}

static void test_picks_2m_when_aligned(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test,
		ipu6_mmu_pgsize(BITMAP_4K_2M, SZ_2M, SZ_2M),
		(size_t)SZ_2M);
}

static void test_falls_back_to_4k_when_unaligned(struct kunit *test)
{
	/* 2 MB of size, but addr_merge is only 4 KB aligned. */
	KUNIT_EXPECT_EQ(test,
		ipu6_mmu_pgsize(BITMAP_4K_2M, SZ_4K, SZ_2M),
		(size_t)SZ_4K);
}

static void test_bitmap_narrows_choice(struct kunit *test)
{
	/* 2 MB aligned and sized, but the bitmap only has 4 KB. */
	KUNIT_EXPECT_EQ(test,
		ipu6_mmu_pgsize(BITMAP_4K, SZ_2M, SZ_2M),
		(size_t)SZ_4K);
}

static struct kunit_case ipu4_mmu_cases[] = {
	KUNIT_CASE(test_picks_4k_for_4k_size),
	KUNIT_CASE(test_picks_2m_when_aligned),
	KUNIT_CASE(test_falls_back_to_4k_when_unaligned),
	KUNIT_CASE(test_bitmap_narrows_choice),
	{}
};

static struct kunit_suite ipu4_mmu_suite = {
	.name = "ipu4_mmu",
	.test_cases = ipu4_mmu_cases,
};
kunit_test_suite(ipu4_mmu_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 MMU page-size selection");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
