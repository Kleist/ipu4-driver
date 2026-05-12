// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the syscom ring-buffer mechanics in
 * kernel/ipu4/ipu6-fw-com.c.
 *
 * Pins the four driver-side syscom data-path functions
 * (ipu6_send_get_token, ipu6_send_put_token, ipu6_recv_get_token,
 * ipu6_recv_put_token) at empty / full / wrap-around edges. The
 * IPU4/IPU6 unification refactor's fwcom step (refactor order step 3)
 * is going to touch these; a wr/rd indexing slip should fail this
 * suite, not surface as ENOMEM or a frame hash mismatch under vm-smoke.
 *
 * The tests don't go through ipu6_fw_com_prepare() — that path
 * requires a working DMA backend on a real ipu6_bus_device. Instead
 * they build a synthetic context: a kernel-memory buffer cast to
 * __iomem stands in for DMEM, with wr/rd index slots prefilled before
 * each call. The internal struct layouts come from
 * ipu6-fw-com-priv.h.
 */

#include <kunit/test.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/types.h>

#include "ipu6-fw-com.h"
#include "ipu6-fw-com-priv.h"

#define Q_SIZE_INTERNAL	8u	/* matches q->size; user queue_size = 7 */
#define Q_TOKEN_SIZE	32u
#define DMEM_SLOTS	16u	/* u32s; far more than the test queue needs */

struct fwcom_test_ctx {
	u32 dmem[DMEM_SLOTS];
	u8 queue_buf[Q_SIZE_INTERNAL * Q_TOKEN_SIZE];
	struct ipu6_fw_sys_queue q;
	struct ipu6_fw_com_context ctx;
};

static void fwcom_setup(struct fwcom_test_ctx *t, bool output, u32 wr, u32 rd)
{
	memset(t, 0, sizeof(*t));
	t->q.host_address = (uintptr_t)t->queue_buf;
	t->q.size = Q_SIZE_INTERNAL;
	t->q.token_size = Q_TOKEN_SIZE;
	t->q.wr_reg = 0;
	t->q.rd_reg = 1;
	/* wr at dmem[0] (byte 0 of __iomem), rd at dmem[1] (byte 4) —
	 * matches q->wr_reg=0 with FW_COM_RD_REG=4.
	 */
	t->dmem[0] = wr;
	t->dmem[1] = rd;
	t->ctx.dmem_addr = (__force void __iomem *)t->dmem;
	if (output)
		t->ctx.output_queue = &t->q;
	else
		t->ctx.input_queue = &t->q;
}

static void test_send_get_empty_returns_null(struct kunit *test)
{
	struct fwcom_test_ctx t;

	fwcom_setup(&t, /*output=*/false, /*wr=*/0, /*rd=*/0);

	KUNIT_EXPECT_PTR_EQ(test, ipu6_send_get_token(&t.ctx, 0), NULL);
}

static void test_send_get_some_available_returns_wr_slot(struct kunit *test)
{
	/* wr=2, rd=5, size=8: wr<rd so packets = rd-wr-1 = 2 free
	 * slots; the producer is handed slot wr=2.
	 */
	struct fwcom_test_ctx t;
	void *got;

	fwcom_setup(&t, false, 2, 5);
	got = ipu6_send_get_token(&t.ctx, 0);

	KUNIT_EXPECT_PTR_EQ(test, got, t.queue_buf + 2 * Q_TOKEN_SIZE);
}

static void test_send_get_full_returns_null(struct kunit *test)
{
	/* wr=7, rd=0, size=8: advancing wr would make wr == rd, which
	 * the ring discipline forbids (one slot kept empty), so 0
	 * available — full.
	 */
	struct fwcom_test_ctx t;

	fwcom_setup(&t, false, 7, 0);

	KUNIT_EXPECT_PTR_EQ(test, ipu6_send_get_token(&t.ctx, 0), NULL);
}

static void test_send_put_advances_wr_in_dmem(struct kunit *test)
{
	struct fwcom_test_ctx t;

	fwcom_setup(&t, false, 2, 5);
	ipu6_send_put_token(&t.ctx, 0);

	KUNIT_EXPECT_EQ(test, t.dmem[0], 3u);
	/* rd untouched on the send side. */
	KUNIT_EXPECT_EQ(test, t.dmem[1], 5u);
}

static void test_send_put_wraps_wr_at_size(struct kunit *test)
{
	/* wr at the last internal slot wraps back to 0, not q->size. */
	struct fwcom_test_ctx t;

	fwcom_setup(&t, false, 7, 0);
	ipu6_send_put_token(&t.ctx, 0);

	KUNIT_EXPECT_EQ(test, t.dmem[0], 0u);
}

static void test_recv_get_empty_returns_null(struct kunit *test)
{
	struct fwcom_test_ctx t;

	fwcom_setup(&t, /*output=*/true, 0, 0);

	KUNIT_EXPECT_PTR_EQ(test, ipu6_recv_get_token(&t.ctx, 0), NULL);
}

static void test_recv_get_after_normal_writes(struct kunit *test)
{
	/* wr=3, rd=0: 3 unread packets; consumer reads from slot rd=0. */
	struct fwcom_test_ctx t;
	void *got;

	fwcom_setup(&t, true, 3, 0);
	got = ipu6_recv_get_token(&t.ctx, 0);

	KUNIT_EXPECT_PTR_EQ(test, got, t.queue_buf + 0 * Q_TOKEN_SIZE);
}

static void test_recv_get_after_wrap_writes(struct kunit *test)
{
	/* wr=2, rd=5, size=8: wr<rd means the producer wrapped. The
	 * adjusted wr is 10, so packets = 10 - 5 = 5 unread; consumer
	 * reads from slot rd=5 (the last pre-wrap write the consumer
	 * hasn't seen yet).
	 */
	struct fwcom_test_ctx t;
	void *got;

	fwcom_setup(&t, true, 2, 5);
	got = ipu6_recv_get_token(&t.ctx, 0);

	KUNIT_EXPECT_PTR_EQ(test, got, t.queue_buf + 5 * Q_TOKEN_SIZE);
}

static void test_recv_put_advances_rd_in_dmem(struct kunit *test)
{
	struct fwcom_test_ctx t;

	fwcom_setup(&t, true, 3, 2);
	ipu6_recv_put_token(&t.ctx, 0);

	KUNIT_EXPECT_EQ(test, t.dmem[1], 3u);
	/* wr untouched on the recv side. */
	KUNIT_EXPECT_EQ(test, t.dmem[0], 3u);
}

static void test_recv_put_wraps_rd_at_size(struct kunit *test)
{
	/* rd at the last internal slot wraps back to 0, mirroring the
	 * send_put discipline.
	 */
	struct fwcom_test_ctx t;

	fwcom_setup(&t, true, 0, 7);
	ipu6_recv_put_token(&t.ctx, 0);

	KUNIT_EXPECT_EQ(test, t.dmem[1], 0u);
}

static struct kunit_case ipu4_fw_com_cases[] = {
	KUNIT_CASE(test_send_get_empty_returns_null),
	KUNIT_CASE(test_send_get_some_available_returns_wr_slot),
	KUNIT_CASE(test_send_get_full_returns_null),
	KUNIT_CASE(test_send_put_advances_wr_in_dmem),
	KUNIT_CASE(test_send_put_wraps_wr_at_size),
	KUNIT_CASE(test_recv_get_empty_returns_null),
	KUNIT_CASE(test_recv_get_after_normal_writes),
	KUNIT_CASE(test_recv_get_after_wrap_writes),
	KUNIT_CASE(test_recv_put_advances_rd_in_dmem),
	KUNIT_CASE(test_recv_put_wraps_rd_at_size),
	{}
};

static struct kunit_suite ipu4_fw_com_suite = {
	.name = "ipu4_fw_com",
	.test_cases = ipu4_fw_com_cases,
};
kunit_test_suite(ipu4_fw_com_suite);

MODULE_DESCRIPTION("KUnit tests for IPU4 fw_com syscom ring buffers");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(INTEL_IPU6);
