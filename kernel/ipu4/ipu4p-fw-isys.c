// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Intel Corporation
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/bug.h>
#include <linux/cacheflush.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>

#include "ipu6-bus.h"
#include "ipu6-fw-com.h"
#include "ipu6-isys.h"
#include "ipu6-isys-queue.h"
#include "ipu6-isys-subdev.h"
#include "ipu4p-fw-isys.h"
#include "ipu6-platform-regs.h"

static const char send_msg_types[N_IPU4P_FW_ISYS_SEND_TYPE][32] = {
	"STREAM_OPEN",
	"STREAM_START",
	"STREAM_START_AND_CAPTURE",
	"STREAM_CAPTURE",
	"STREAM_STOP",
	"STREAM_FLUSH",
	"STREAM_CLOSE",
};

static int ipu4p_fw_isys_send_cmd(struct ipu6_isys *isys,
				  const unsigned int stream_handle,
				  void *cpu_mapped_buf,
				  dma_addr_t dma_mapped_buf,
				  size_t size, u16 send_type)
{
	struct ipu4p_fw_isys_send_token *token;
	struct device *dev = &isys->adev->auxdev.dev;

	if (send_type >= N_IPU4P_FW_ISYS_SEND_TYPE ||
	    stream_handle >= IPU4P_ISYS_MAX_STREAMS)
		return -EINVAL;

	if (upper_32_bits(dma_mapped_buf))
		return -ERANGE;

	dev_dbg(dev, "send_token: %s\n", send_msg_types[send_type]);

	if (cpu_mapped_buf)
		clflush_cache_range(cpu_mapped_buf, size);

	token = ipu6_send_get_token(isys->fwctx,
				    stream_handle + IPU4P_BASE_MSG_SEND_QUEUES);
	if (!token)
		return -EBUSY;

	token->payload = lower_32_bits(dma_mapped_buf);
	token->buf_handle = (unsigned long)cpu_mapped_buf;
	token->send_type = send_type;
	token->stream_id = stream_handle;

	ipu6_send_put_token(isys->fwctx,
			    stream_handle + IPU4P_BASE_MSG_SEND_QUEUES);

	return 0;
}

static int ipu4p_fw_isys_close(struct ipu6_isys *isys)
{
	struct device *dev = &isys->adev->auxdev.dev;
	int retry = IPU4P_ISYS_CLOSE_RETRY;
	void *fwctx;
	int ret;

	if (!isys->fwctx)
		return 0;

	ret = ipu6_fw_com_close(isys->fwctx);
	fwctx = isys->fwctx;
	isys->fwctx = NULL;
	if (ret)
		dev_err(dev, "Device close failure: %d\n", ret);

	do {
		usleep_range(400, 500);
		ret = ipu6_fw_com_release(fwctx, 0);
		retry--;
	} while (ret && retry);

	if (ret) {
		dev_err(dev, "Device release timed out: %d\n", ret);
		isys->fwctx = fwctx;
	}

	return ret;
}

static void ipu4p_fw_isys_cleanup(struct ipu6_isys *isys)
{
	int ret;

	if (!isys->fwctx)
		return;

	ret = ipu6_fw_com_release(isys->fwctx, 1);
	if (ret < 0)
		dev_warn(&isys->adev->auxdev.dev,
			 "Device busy, firmware communication release failed\n");

	isys->fwctx = NULL;
}

static void ipu4p_fw_isys_start_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = isys->pdata->base +
		isys->pdata->ipdata->hw_variant.spc_offset;
	u32 val;

	val = IPU6_ISYS_SPC_STATUS_START | IPU6_ISYS_SPC_STATUS_RUN |
	      IPU6_ISYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;
	if (isys->icache_prefetch)
		val |= IPU6_ISYS_SPC_STATUS_ICACHE_PREFETCH;

	writel(val, spc_regs_base + IPU6_ISYS_REG_SPC_STATUS_CTRL);
}

static int ipu4p_fw_isys_query_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = isys->pdata->base +
		isys->pdata->ipdata->hw_variant.spc_offset;
	u32 val;

	val = readl(spc_regs_base + IPU6_ISYS_REG_SPC_STATUS_CTRL);
	val &= IPU6_ISYS_SPC_STATUS_READY | IPU6_ISYS_SPC_STATUS_START;

	return val == IPU6_ISYS_SPC_STATUS_READY;
}

static int
ipu4p_fw_isys_fwcom_cfg_init(struct ipu6_isys *isys,
			     struct ipu6_fw_com_cfg *fwcom_cfg,
			     unsigned int num_streams)
{
	unsigned int max_send_queues, max_sram_blocks, max_devq_size;
	struct ipu6_fw_syscom_queue_config *input_queue_cfg;
	struct ipu6_fw_syscom_queue_config *output_queue_cfg;
	struct ipu4p_fw_isys_fw_config *isys_fw_cfg;
	unsigned int num_message_queues;
	unsigned int max_streams;
	unsigned int i;

	max_streams = min_t(unsigned int, isys->pdata->ipdata->max_streams,
			    IPU4P_ISYS_MAX_STREAMS);
	max_send_queues = isys->pdata->ipdata->max_send_queues;
	max_sram_blocks = min_t(unsigned int,
				isys->pdata->ipdata->max_sram_blocks,
				IPU4P_NOF_SRAM_BLOCKS_MAX);
	max_devq_size = isys->pdata->ipdata->max_devq_size;
	if (!max_streams || !max_sram_blocks)
		return -EINVAL;

	max_streams = min(max_streams, max_sram_blocks);
	if (max_send_queues < IPU4P_BASE_MSG_SEND_QUEUES + max_streams)
		return -EINVAL;

	num_message_queues = clamp(num_streams, 1U, max_streams);

	isys_fw_cfg = kzalloc_obj(*isys_fw_cfg);
	if (!isys_fw_cfg)
		return -ENOMEM;

	input_queue_cfg = kzalloc_objs(*input_queue_cfg, max_send_queues);
	if (!input_queue_cfg)
		goto err_free_fw_cfg;

	output_queue_cfg = kzalloc_objs(*output_queue_cfg,
					IPU4P_N_MAX_RECV_QUEUES);
	if (!output_queue_cfg)
		goto err_free_input_queue_cfg;

	isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_PROXY] =
		IPU4P_N_MAX_PROXY_SEND_QUEUES;
	isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_DEV] =
		IPU4P_N_MAX_DEV_SEND_QUEUES;
	isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_MSG] =
		num_message_queues;
	isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_PROXY] =
		IPU4P_N_MAX_PROXY_RECV_QUEUES;
	isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_DEV] = 0;
	isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_MSG] =
		IPU4P_N_MAX_RECV_QUEUES - IPU4P_N_MAX_PROXY_RECV_QUEUES;

	/*
	 * The GDA has 128 physical pages and a virtualization factor of 32.
	 * Partition the virtual pages evenly across the message queues.
	 */
	for (i = 0; i < max_sram_blocks; i++) {
		if (i < num_message_queues)
			isys_fw_cfg->buffer_partition.num_gda_pages[i] =
				(IPU4P_DEVICE_GDA_NR_PAGES *
				 IPU4P_DEVICE_GDA_VIRT_FACTOR) /
				num_message_queues;
		else
			isys_fw_cfg->buffer_partition.num_gda_pages[i] = 0;
	}

	input_queue_cfg[IPU4P_BASE_PROXY_SEND_QUEUES].token_size =
		sizeof(struct ipu4p_fw_isys_proxy_send_token);
	input_queue_cfg[IPU4P_BASE_PROXY_SEND_QUEUES].queue_size =
		IPU4P_ISYS_SIZE_PROXY_SEND_QUEUE;

	input_queue_cfg[IPU4P_BASE_DEV_SEND_QUEUES].token_size =
		sizeof(struct ipu4p_fw_isys_send_token);
	input_queue_cfg[IPU4P_BASE_DEV_SEND_QUEUES].queue_size = max_devq_size;

	for (i = 0; i < num_message_queues; i++) {
		unsigned int queue = IPU4P_BASE_MSG_SEND_QUEUES + i;

		input_queue_cfg[queue].token_size =
			sizeof(struct ipu4p_fw_isys_send_token);
		input_queue_cfg[queue].queue_size = IPU4P_ISYS_SIZE_SEND_QUEUE;
	}

	output_queue_cfg[IPU4P_BASE_PROXY_RECV_QUEUES].token_size =
		sizeof(struct ipu4p_fw_isys_proxy_recv_token);
	output_queue_cfg[IPU4P_BASE_PROXY_RECV_QUEUES].queue_size =
		IPU4P_ISYS_SIZE_PROXY_RECV_QUEUE;

	output_queue_cfg[IPU4P_BASE_MSG_RECV_QUEUES].token_size =
		sizeof(struct ipu4p_fw_isys_recv_token);
	output_queue_cfg[IPU4P_BASE_MSG_RECV_QUEUES].queue_size =
		IPU4P_ISYS_SIZE_RECV_QUEUE;

	fwcom_cfg->num_input_queues =
		isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_PROXY] +
		isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_DEV] +
		isys_fw_cfg->num_send_queues[IPU4P_FW_ISYS_QUEUE_TYPE_MSG];
	fwcom_cfg->num_output_queues =
		isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_PROXY] +
		isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_DEV] +
		isys_fw_cfg->num_recv_queues[IPU4P_FW_ISYS_QUEUE_TYPE_MSG];
	fwcom_cfg->input = input_queue_cfg;
	fwcom_cfg->output = output_queue_cfg;
	fwcom_cfg->dmem_addr = isys->pdata->ipdata->hw_variant.dmem_offset;
	fwcom_cfg->specific_addr = isys_fw_cfg;
	fwcom_cfg->specific_size = sizeof(*isys_fw_cfg);
	fwcom_cfg->dmem_syscom = true;

	return 0;

err_free_input_queue_cfg:
	kfree(input_queue_cfg);
err_free_fw_cfg:
	kfree(isys_fw_cfg);

	return -ENOMEM;
}

static void ipu4p_fw_isys_fwcom_cfg_free(struct ipu6_fw_com_cfg *fwcom_cfg)
{
	kfree(fwcom_cfg->specific_addr);
	kfree(fwcom_cfg->output);
	kfree(fwcom_cfg->input);
}

static int ipu4p_fw_isys_init(struct ipu6_isys *isys,
			      unsigned int num_streams)
{
	struct device *dev = &isys->adev->auxdev.dev;
	struct ipu6_fw_com_cfg fwcom_cfg = {
		.cell_start = ipu4p_fw_isys_start_sp,
		.cell_ready = ipu4p_fw_isys_query_sp,
	};
	int retry = IPU4P_ISYS_OPEN_RETRY;
	int ret;

	ret = ipu4p_fw_isys_fwcom_cfg_init(isys, &fwcom_cfg, num_streams);
	if (ret)
		return ret;

	isys->fwctx = ipu6_fw_com_prepare(&fwcom_cfg, isys->adev,
					  isys->pdata->base);
	ipu4p_fw_isys_fwcom_cfg_free(&fwcom_cfg);
	if (!isys->fwctx) {
		dev_err(dev, "ISYS firmware communication prepare failed\n");
		return -EIO;
	}

	ret = ipu6_fw_com_open(isys->fwctx);
	if (ret) {
		dev_err(dev, "ISYS firmware communication open failed: %d\n",
			ret);
		ipu4p_fw_isys_cleanup(isys);
		return ret;
	}

	do {
		usleep_range(400, 500);
		if (ipu6_fw_com_ready(isys->fwctx))
			return 0;
		retry--;
	} while (retry);

	dev_err(dev, "ISYS firmware failed to become ready\n");
	ipu4p_fw_isys_close(isys);

	return -EIO;
}

static void
ipu4p_fw_isys_dump_stream_cfg(struct device *dev, struct isys_fw_msgs *msg)
{
	const struct ipu4p_fw_isys_stream_cfg *cfg = &msg->ipu4p.stream;
	unsigned int i;

	dev_dbg(dev, "IPU4P stream config: source %u, VC %u, input pins %u, output pins %u\n",
		cfg->src, cfg->vc, cfg->nof_input_pins, cfg->nof_output_pins);

	for (i = 0; i < cfg->nof_input_pins; i++) {
		const struct ipu4p_fw_isys_input_pin_info *pin =
			&cfg->input_pins[i];

		dev_dbg(dev, "input pin %u: %ux%u, DT 0x%x, bpp %u, mapped DT 0x%x\n",
			i, pin->input_res.width, pin->input_res.height,
			pin->dt, pin->bits_per_pix, pin->mapped_dt);
	}

	for (i = 0; i < cfg->nof_output_pins; i++) {
		const struct ipu4p_fw_isys_output_pin_info *pin =
			&cfg->output_pins[i];

		dev_dbg(dev, "output pin %u: %ux%u, stride %u, format %u, type %u\n",
			i, pin->output_res.width, pin->output_res.height,
			pin->stride, pin->ft, pin->pt);
	}
}

static void
ipu4p_fw_isys_dump_frame_buf_set(struct device *dev,
				 struct isys_fw_msgs *msg,
				 unsigned int outputs)
{
	const struct ipu4p_fw_isys_frame_buff_set *set = &msg->ipu4p.frame;
	unsigned int i;

	for (i = 0; i < outputs; i++) {
		const struct ipu4p_fw_isys_output_pin_payload *pin =
			&set->output_pins[i];

		dev_dbg(dev, "output pin %u: buffer ID 0x%llx, address 0x%x\n",
			i, (unsigned long long)pin->out_buf_id, pin->addr);
	}
}

static int ipu4p_fw_isys_resp_type_to_ipu6(unsigned int type)
{
	switch (type) {
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_OPEN_DONE:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_OPEN_DONE;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_START_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_START_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_STOP_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_STOP_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_FLUSH_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_FLUSH_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_CLOSE_ACK:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_CLOSE_ACK;
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_READY:
		return IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_READY;
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_SOF:
		return IPU6_FW_ISYS_RESP_TYPE_FRAME_SOF;
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_EOF:
		return IPU6_FW_ISYS_RESP_TYPE_FRAME_EOF;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE;
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE:
		return IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE;
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_SKIPPED:
		return IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_SKIPPED;
	case IPU4P_FW_ISYS_RESP_TYPE_STATS_DATA_READY:
		return IPU6_FW_ISYS_RESP_TYPE_STATS_DATA_READY;
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_WATERMARK:
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_SKIPPED:
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_SOF_DISCARDED:
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_EOF_DISCARDED:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
}

/*
 * Resolve each pin response by stream, frame counter and echoed handle when
 * present. Clear each output-pin bit exactly once so duplicate responses
 * cannot release a frame message before all buffers have completed. Never
 * dereference a firmware-provided value as a pointer.
 */
static bool
ipu4p_fw_isys_complete_pin_response(struct ipu6_isys *isys,
				    const struct ipu4p_fw_isys_resp_info *resp)
{
	struct isys_fw_msgs *msg;
	unsigned long flags;
	u64 handle = resp->pin.out_buf_id;
	u8 pin_mask;
	bool found = false;

	if (resp->pin_id >= IPU4P_MAX_OPINS)
		return false;

	pin_mask = BIT(resp->pin_id);
	spin_lock_irqsave(&isys->listlock, flags);
	list_for_each_entry(msg, &isys->framebuflist_fw, head) {
		if (!msg->ipu4p_is_frame ||
		    msg->ipu4p_stream_handle != resp->stream_handle ||
		    msg->ipu4p_frame_counter != resp->frame_counter ||
		    (handle && handle != (u64)(uintptr_t)&msg->ipu4p.frame))
			continue;
		if (!(msg->ipu4p_pending_pin_mask & pin_mask))
			break;

		msg->ipu4p_pending_pin_mask &= ~pin_mask;
		if (!msg->ipu4p_pending_pin_mask)
			list_move(&msg->head, &isys->framebuflist);
		found = true;
		break;
	}
	spin_unlock_irqrestore(&isys->listlock, flags);

	return found;
}

/*
 * A skipped capture has no PIN_DATA_READY response. Copy the output addresses
 * before releasing the frame set, then complete those buffers as skipped.
 */
static bool
ipu4p_fw_isys_complete_skipped_frame(struct ipu6_isys *isys,
				     unsigned int stream_handle,
				     unsigned int frame_counter,
				     u32 addrs[IPU4P_MAX_OPINS],
				     u8 *pending_pin_mask)
{
	struct isys_fw_msgs *msg;
	unsigned long flags;
	bool found = false;
	unsigned int i;

	*pending_pin_mask = 0;
	spin_lock_irqsave(&isys->listlock, flags);
	list_for_each_entry(msg, &isys->framebuflist_fw, head) {
		if (!msg->ipu4p_is_frame ||
		    msg->ipu4p_stream_handle != stream_handle ||
		    msg->ipu4p_frame_counter != frame_counter)
			continue;

		*pending_pin_mask = msg->ipu4p_pending_pin_mask;
		for (i = 0; i < IPU4P_MAX_OPINS; i++) {
			if (*pending_pin_mask & BIT(i))
				addrs[i] = msg->ipu4p.frame.output_pins[i].addr;
		}

		msg->ipu4p_pending_pin_mask = 0;
		list_move(&msg->head, &isys->framebuflist);
		found = true;
		break;
	}
	spin_unlock_irqrestore(&isys->listlock, flags);

	return found;
}

static int ipu4p_fw_isys_send_proxy_token(struct ipu6_isys *isys,
					  unsigned int req_id,
					  unsigned int region_index,
					  u32 value)
{
	struct ipu6_fw_com_context *ctx = isys->fwctx;
	struct ipu4p_fw_isys_proxy_send_token *send_token;
	unsigned int timeout = 1000;

	send_token = ipu6_send_get_token(ctx, IPU4P_BASE_PROXY_SEND_QUEUES);
	if (!send_token)
		return -EBUSY;

	send_token->request_id = req_id;
	send_token->region_index = region_index;
	send_token->offset = 0;
	send_token->value = value;
	ipu6_send_put_token(ctx, IPU4P_BASE_PROXY_SEND_QUEUES);

	do {
		struct ipu4p_fw_isys_proxy_recv_token *recv_token;
		int ret;

		usleep_range(100, 110);
		recv_token = ipu6_recv_get_token(ctx,
						 IPU4P_BASE_PROXY_RECV_QUEUES);
		if (!recv_token)
			continue;

		if (recv_token->proxy_resp_info.request_id != req_id)
			ret = -EIO;
		else if (recv_token->proxy_resp_info.error_info.error)
			ret = -EIO;
		else
			ret = 0;

		ipu6_recv_put_token(ctx, IPU4P_BASE_PROXY_RECV_QUEUES);
		return ret;
	} while (--timeout);

	return -ETIMEDOUT;
}

static int
ipu4p_fw_isys_set_iwake_register(struct ipu6_isys *isys,
				 enum ipu6_isys_iwake_register reg, u32 value)
{
	switch (reg) {
	case IPU6_ISYS_IWAKE_GDA_THRESHOLD:
		return ipu4p_fw_isys_send_proxy_token(isys, 0, 0, value);
	case IPU6_ISYS_IWAKE_GDA_ENABLE:
		return ipu4p_fw_isys_send_proxy_token(isys, 1, 1, value);
	case IPU6_ISYS_IWAKE_GDA_IRQ_CRITICAL_THRESHOLD:
	case IPU6_ISYS_IWAKE_GDA_MEMOPEN_THRESHOLD:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
}

static int ipu4p_fw_isys_isr_one(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	struct device *dev = &adev->auxdev.dev;
	struct ipu4p_fw_isys_recv_token *token;
	struct ipu4p_fw_isys_resp_info *fw_resp;
	struct ipu6_fw_isys_resp_info_abi resp = {};
	int type;

	if (!isys->fwctx)
		return 1;

	token = ipu6_recv_get_token(isys->fwctx, IPU4P_BASE_MSG_RECV_QUEUES);
	if (!token)
		return 1;

	fw_resp = &token->resp_info;
	if (fw_resp->type >= N_IPU4P_FW_ISYS_RESP_TYPE) {
		dev_err(dev, "Unknown IPU4P response type %u stream %u\n",
			fw_resp->type, fw_resp->stream_handle);
		goto out;
	}

	switch (fw_resp->type) {
	case IPU4P_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_SKIPPED: {
		u32 addrs[IPU4P_MAX_OPINS] = {};
		u8 pending_pin_mask;
		unsigned int i;

		if (!ipu4p_fw_isys_complete_skipped_frame(isys,
							  fw_resp->stream_handle,
							  fw_resp->frame_counter,
							  addrs, &pending_pin_mask)) {
			dev_dbg(dev, "No in-flight frame set for skipped stream %u frame %u\n",
				fw_resp->stream_handle, fw_resp->frame_counter);
			goto out;
		}

		for (i = 0; i < IPU4P_MAX_OPINS; i++) {
			struct ipu6_fw_isys_resp_info_abi skipped = {};

			if (!(pending_pin_mask & BIT(i)))
				continue;

			skipped.timestamp[0] = fw_resp->timestamp[0];
			skipped.timestamp[1] = fw_resp->timestamp[1];
			skipped.stream_handle = fw_resp->stream_handle;
			skipped.type = IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_SKIPPED;
			skipped.pin_id = i;
			skipped.pin.addr = addrs[i];
			ipu6_isys_handle_response(adev, &skipped, false);
		}
		goto out;
	}
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_SKIPPED:
		if (!ipu4p_fw_isys_complete_pin_response(isys, fw_resp)) {
			dev_dbg(dev, "No in-flight pin set for stream %u frame %u\n",
				fw_resp->stream_handle, fw_resp->frame_counter);
			goto out;
		}
		break;
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_SOF_DISCARDED:
	case IPU4P_FW_ISYS_RESP_TYPE_FRAME_EOF_DISCARDED:
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_WATERMARK:
		dev_dbg(dev, "Ignoring IPU4P response type %u for stream %u\n",
			fw_resp->type, fw_resp->stream_handle);
		goto out;
	case IPU4P_FW_ISYS_RESP_TYPE_PIN_DATA_READY:
		if (!ipu4p_fw_isys_complete_pin_response(isys, fw_resp)) {
			dev_warn(dev,
				 "Unmatched IPU4P pin response for stream %u frame %u\n",
				 fw_resp->stream_handle, fw_resp->frame_counter);
			goto out;
		}
		break;
	default:
		break;
	}

	type = ipu4p_fw_isys_resp_type_to_ipu6(fw_resp->type);
	if (type < 0) {
		if (type == -EINVAL)
			dev_err(dev, "Unknown IPU4P response type %u\n",
				fw_resp->type);
		else
			dev_dbg(dev, "Ignoring unsupported IPU4P response type %u\n",
				fw_resp->type);
		goto out;
	}

	resp.buf_id = fw_resp->buf_id;
	resp.pin.out_buf_id = fw_resp->pin.out_buf_id;
	resp.pin.addr = fw_resp->pin.addr;
	resp.pin.compress = fw_resp->pin.compress;
	resp.error_info.error = fw_resp->error_info.error;
	resp.error_info.error_details = fw_resp->error_info.error_details;
	resp.timestamp[0] = fw_resp->timestamp[0];
	resp.timestamp[1] = fw_resp->timestamp[1];
	resp.stream_handle = fw_resp->stream_handle;
	resp.type = type;
	resp.pin_id = fw_resp->pin_id;

	/* IPU4P manages message-buffer ownership separately. */
	ipu6_isys_handle_response(adev, &resp, false);

out:
	ipu6_recv_put_token(isys->fwctx, IPU4P_BASE_MSG_RECV_QUEUES);

	return 0;
}

static int ipu4p_fw_isys_pin_cfg(struct ipu6_isys_video *av,
				 struct ipu6_isys_stream *stream,
				 struct media_pad *src_pad,
				 struct v4l2_mbus_frame_desc_entry *entry,
				 void *__cfg)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(src_pad->entity);
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(sd);
	struct ipu4p_fw_isys_stream_cfg *cfg = __cfg;
	struct ipu4p_fw_isys_input_pin_info *input_pin;
	struct ipu4p_fw_isys_output_pin_info *output_pin;
	struct ipu6_isys_queue *aq = &av->aq;
	const struct ipu6_isys_pixelformat *pfmt;
	struct v4l2_mbus_framefmt *fmt;
	unsigned int input_pins, output_pins;
	u32 src_stream;

	if (cfg->nof_input_pins >= IPU4P_MAX_IPINS ||
	    cfg->nof_output_pins >= IPU4P_MAX_OPINS)
		return -EINVAL;

	pfmt = ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	if (!pfmt)
		return -EINVAL;

	src_stream = __ipu6_isys_get_src_stream_by_src_pad(state, src_pad->index);
	fmt = v4l2_subdev_state_get_format(state, src_pad->index, src_stream);
	if (!fmt)
		return -EINVAL;

	input_pins = cfg->nof_input_pins++;
	input_pin = &cfg->input_pins[input_pins];
	input_pin->input_res.width = fmt->width;
	input_pin->input_res.height = fmt->height;
	input_pin->dt = entry->bus.csi2.dt;
	input_pin->bits_per_pix = pfmt->bpp_packed;
	input_pin->mapped_dt = N_IPU4P_FW_ISYS_MIPI_DATA_TYPE;
	input_pin->mipi_store_mode = pfmt->bpp == pfmt->bpp_packed ?
		IPU4P_FW_ISYS_MIPI_STORE_MODE_DISCARD_LONG_HEADER :
		IPU4P_FW_ISYS_MIPI_STORE_MODE_NORMAL;

	output_pins = cfg->nof_output_pins++;
	aq->fw_output = output_pins;
	stream->output_pins_queue[output_pins] = aq;

	output_pin = &cfg->output_pins[output_pins];
	output_pin->input_pin_id = input_pins;
	output_pin->output_res.width = ipu6_isys_get_frame_width(av);
	output_pin->output_res.height = ipu6_isys_get_frame_height(av);
	output_pin->stride = ipu6_isys_get_bytes_per_line(av);
	output_pin->payload_buf_size = 0;
	output_pin->pt = pfmt->bpp == pfmt->bpp_packed ?
		IPU4P_FW_ISYS_PIN_TYPE_MIPI : IPU4P_FW_ISYS_PIN_TYPE_RAW_SOC;
	output_pin->ft = pfmt->css_pixelformat;
	output_pin->send_irq = 1;
	output_pin->link_id = IPU4P_FW_ISYS_LINK_OFFLINE;

	return 0;
}

static int
ipu4p_fw_isys_prepare_stream_cfg(struct ipu6_isys_stream *stream,
				 struct v4l2_mbus_frame_desc *desc,
				 struct isys_fw_msgs *msg)
{
	struct ipu4p_fw_isys_stream_cfg *cfg = &msg->ipu4p.stream;
	struct device *dev = &stream->isys->adev->auxdev.dev;
	int ret;

	memset(cfg, 0, sizeof(*cfg));
	msg->ipu4p_stream_handle = stream->stream_handle;
	msg->ipu4p_frame_counter = 0;
	msg->ipu4p_pending_pin_mask = 0;
	msg->ipu4p_is_frame = false;
	msg->ipu4p_addr_error = false;
	cfg->src = stream->asd->source;
	cfg->vc = stream->vc;
	cfg->isl_use = IPU4P_FW_ISYS_USE_NO_ISL_NO_ISA;

	ret = ipu6_isys_fw_pins_prepare(stream, desc, ipu4p_fw_isys_pin_cfg,
					cfg);
	if (ret)
		return ret;

	stream->nr_output_pins = cfg->nof_output_pins;
	ipu4p_fw_isys_dump_stream_cfg(dev, msg);

	return 0;
}

static int ipu4p_fw_isys_stream_open(struct ipu6_isys *isys,
				     const unsigned int stream_handle,
				     struct isys_fw_msgs *msg)
{
	return ipu4p_fw_isys_send_cmd(isys, stream_handle,
				      &msg->ipu4p.stream, msg->dma_addr,
				      sizeof(msg->ipu4p.stream),
				      IPU4P_FW_ISYS_SEND_TYPE_STREAM_OPEN);
}

static int ipu4p_fw_isys_stream_close(struct ipu6_isys *isys,
				      const unsigned int stream_handle)
{
	return ipu4p_fw_isys_send_cmd(isys, stream_handle, NULL, 0, 0,
				      IPU4P_FW_ISYS_SEND_TYPE_STREAM_CLOSE);
}

static int ipu4p_fw_isys_stream_flush(struct ipu6_isys *isys,
				      const unsigned int stream_handle)
{
	return ipu4p_fw_isys_send_cmd(isys, stream_handle, NULL, 0, 0,
				      IPU4P_FW_ISYS_SEND_TYPE_STREAM_FLUSH);
}

static int
ipu4p_fw_isys_send_frame_buff_set(struct ipu6_isys *isys,
				  const unsigned int stream_handle,
				  struct isys_fw_msgs *msg, u16 send_type)
{
	if (msg->ipu4p_addr_error)
		return -ERANGE;

	return ipu4p_fw_isys_send_cmd(isys, stream_handle,
				      &msg->ipu4p.frame, msg->dma_addr,
				      sizeof(msg->ipu4p.frame), send_type);
}

static int ipu4p_fw_isys_stream_start(struct ipu6_isys *isys,
				      const unsigned int stream_handle,
				      struct isys_fw_msgs *msg, bool capture)
{
	if (!capture)
		return ipu4p_fw_isys_send_cmd(isys, stream_handle, NULL, 0, 0,
					      IPU4P_FW_ISYS_SEND_TYPE_STREAM_START);

	return ipu4p_fw_isys_send_frame_buff_set(isys, stream_handle, msg,
						 IPU4P_FW_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE);
}

static int ipu4p_fw_isys_stream_capture(struct ipu6_isys *isys,
					const unsigned int stream_handle,
					struct isys_fw_msgs *msg)
{
	return ipu4p_fw_isys_send_frame_buff_set(isys, stream_handle, msg,
						 IPU4P_FW_ISYS_SEND_TYPE_STREAM_CAPTURE);
}

static bool
ipu4p_fw_isys_buf_to_frame_buf_pin(struct vb2_buffer *vb,
				   struct ipu4p_fw_isys_frame_buff_set *set)
{
	struct ipu6_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu6_isys_video_buffer *ivb =
		vb2_buffer_to_ipu6_isys_video_buffer(vvb);
	struct ipu4p_fw_isys_output_pin_payload *pin;

	if (aq->fw_output >= IPU4P_MAX_OPINS ||
	    upper_32_bits(ivb->dma_addr))
		return false;

	pin = &set->output_pins[aq->fw_output];
	pin->addr = lower_32_bits(ivb->dma_addr);
	/* Firmware echoes this opaque handle in PIN_DATA_READY. */
	pin->out_buf_id = (u64)(uintptr_t)set;

	return true;
}

static void
ipu4p_fw_isys_prepare_buf_set(struct isys_fw_msgs *msg,
			      struct ipu6_isys_stream *stream,
			      struct ipu6_isys_buffer_list *bl)
{
	struct ipu4p_fw_isys_frame_buff_set *set = &msg->ipu4p.frame;
	struct ipu6_isys_buffer *ib;

	WARN_ON(!bl->nbufs);

	memset(set, 0, sizeof(*set));
	set->frame_counter = atomic_fetch_inc(&stream->buf_id) % 256;
	msg->ipu4p_stream_handle = stream->stream_handle;
	msg->ipu4p_frame_counter = set->frame_counter;
	msg->ipu4p_pending_pin_mask = 0;
	msg->ipu4p_is_frame = true;
	msg->ipu4p_addr_error = false;
	set->send_irq_sof = 1;
	set->send_irq_eof = 1;
	set->send_irq_capture_ack = 1;
	set->send_irq_capture_done = 1;
	set->send_resp_sof = 1;
	set->send_resp_eof = 1;

	list_for_each_entry(ib, &bl->head, head) {
		struct vb2_buffer *vb = ipu6_isys_buffer_to_vb2_buffer(ib);
		struct ipu6_isys_queue *aq =
			vb2_queue_to_isys_queue(vb->vb2_queue);

		if (ipu4p_fw_isys_buf_to_frame_buf_pin(vb, set))
			msg->ipu4p_pending_pin_mask |= BIT(aq->fw_output);
		else
			msg->ipu4p_addr_error = true;
	}
}

const struct ipu6_fw_isys_ops ipu4p_fw_isys_ops = {
	.init = ipu4p_fw_isys_init,
	.close = ipu4p_fw_isys_close,
	.isr_one = ipu4p_fw_isys_isr_one,
	.set_iwake_register = ipu4p_fw_isys_set_iwake_register,
	.send_cmd = ipu4p_fw_isys_send_cmd,
	.cleanup = ipu4p_fw_isys_cleanup,
	.prepare_stream_cfg = ipu4p_fw_isys_prepare_stream_cfg,
	.prepare_buf_set = ipu4p_fw_isys_prepare_buf_set,
	.stream_open = ipu4p_fw_isys_stream_open,
	.stream_start = ipu4p_fw_isys_stream_start,
	.stream_capture = ipu4p_fw_isys_stream_capture,
	.stream_flush = ipu4p_fw_isys_stream_flush,
	.stream_close = ipu4p_fw_isys_stream_close,
	.dump_stream_cfg = ipu4p_fw_isys_dump_stream_cfg,
	.dump_frame_buf_set = ipu4p_fw_isys_dump_frame_buf_set,
};
