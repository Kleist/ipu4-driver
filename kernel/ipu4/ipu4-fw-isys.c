// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013--2024 Intel Corporation
 *
 * IPU4 ISYS firmware interface: queue layout, stream configuration,
 * buffer sets and responses, as ipu7-fw-isys.c does for IPU7.
 */

#include <linux/cacheflush.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "ipu4-fw-isys.h"
#include "ipu6-bus.h"
#include "ipu6-cpd.h"
#include "ipu6-fw-com.h"
#include "ipu6-isys.h"
#include "ipu6-platform-isys-csi2-reg.h"
#include "ipu6-platform-regs.h"
#include "ipu7-fw-isys.h"

static const char send_msg_types[N_IPU6_FW_ISYS_SEND_TYPE][32] = {
	"STREAM_OPEN",
	"STREAM_START",
	"STREAM_START_AND_CAPTURE",
	"STREAM_CAPTURE",
	"STREAM_STOP",
	"STREAM_FLUSH",
	"STREAM_CLOSE"
};

static int ipu4_fw_isys_complex_cmd(struct ipu6_isys *isys,
				    const unsigned int stream_handle,
				    void *cpu_mapped_buf,
				    dma_addr_t dma_mapped_buf,
				    size_t size, u16 send_type)
{
	struct ipu6_fw_com_context *ctx = isys->fwctx;
	struct device *dev = &isys->adev->auxdev.dev;
	struct ipu6_fw_send_queue_token *token;

	if (send_type >= N_IPU6_FW_ISYS_SEND_TYPE)
		return -EINVAL;

	dev_dbg(dev, "send_token: %s\n", send_msg_types[send_type]);

	/*
	 * Time to flush cache in case we have some payload. Not all messages
	 * have that
	 */
	if (cpu_mapped_buf)
		clflush_cache_range(cpu_mapped_buf, size);

	token = ipu6_send_get_token(ctx,
				    stream_handle + IPU4_BASE_MSG_SEND_QUEUES);
	if (!token)
		return -EBUSY;

	token->payload = dma_mapped_buf;
	token->buf_handle = (unsigned long)cpu_mapped_buf;
	token->send_type = send_type;

	ipu6_send_put_token(ctx, stream_handle + IPU4_BASE_MSG_SEND_QUEUES);

	return 0;
}

static int ipu4_fw_isys_close(struct ipu6_isys *isys)
{
	struct device *dev = &isys->adev->auxdev.dev;
	int retry = IPU6_ISYS_CLOSE_RETRY;
	void *fwctx;
	int ret;

	/*
	 * Stop the isys fw. Actual close takes
	 * some time as the FW must stop its actions including code fetch
	 * to SP icache.
	 * spinlock to wait the interrupt handler to be finished
	 */
	ret = ipu6_fw_com_close(isys->fwctx);
	fwctx = isys->fwctx;
	isys->fwctx = NULL;
	if (ret)
		dev_err(dev, "Device close failure: %d\n", ret);

	/* release probably fails if the close failed. Let's try still */
	do {
		usleep_range(400, 500);
		ret = ipu6_fw_com_release(fwctx, 0);
		retry--;
	} while (ret && retry);

	if (ret) {
		dev_err(dev, "Device release time out %d\n", ret);
		isys->fwctx = fwctx;
	}

	return ret;
}

static void ipu4_fw_isys_cleanup(struct ipu6_isys *isys)
{
	int ret;

	ret = ipu6_fw_com_release(isys->fwctx, 1);
	if (ret < 0)
		dev_warn(&isys->adev->auxdev.dev,
			 "Device busy, fw_com release failed.");
	isys->fwctx = NULL;
}

static void start_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = isys->pdata->base +
		isys->pdata->ipdata->hw_variant.spc_offset;
	u32 val = IPU6_ISYS_SPC_STATUS_START |
		IPU6_ISYS_SPC_STATUS_RUN |
		IPU6_ISYS_SPC_STATUS_CTRL_ICACHE_INVALIDATE;

	val |= isys->icache_prefetch ? IPU6_ISYS_SPC_STATUS_ICACHE_PREFETCH : 0;

	writel(val, spc_regs_base + IPU6_ISYS_REG_SPC_STATUS_CTRL);
}

static int query_sp(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	void __iomem *spc_regs_base = isys->pdata->base +
		isys->pdata->ipdata->hw_variant.spc_offset;
	u32 val;

	val = readl(spc_regs_base + IPU6_ISYS_REG_SPC_STATUS_CTRL);
	/* return true when READY == 1, START == 0 */
	val &= IPU6_ISYS_SPC_STATUS_READY | IPU6_ISYS_SPC_STATUS_START;

	return val == IPU6_ISYS_SPC_STATUS_READY;
}

static int ipu4_isys_fwcom_cfg_init(struct ipu6_isys *isys,
				    struct ipu6_fw_com_cfg *fwcom_cfg,
				    unsigned int num_streams)
{
	unsigned int max_send_queues, max_sram_blocks, max_devq_size;
	struct ipu6_fw_syscom_queue_config *input_queue_cfg;
	struct ipu6_fw_syscom_queue_config *output_queue_cfg;
	int type_proxy = IPU6_FW_ISYS_QUEUE_TYPE_PROXY;
	int type_dev = IPU6_FW_ISYS_QUEUE_TYPE_DEV;
	int type_msg = IPU6_FW_ISYS_QUEUE_TYPE_MSG;
	int base_dev_send = IPU4_BASE_DEV_SEND_QUEUES;
	int base_msg_send = IPU4_BASE_MSG_SEND_QUEUES;
	int base_msg_recv = IPU4_BASE_MSG_RECV_QUEUES;
	struct ipu4_fw_isys_fw_config *isys_fw_cfg;
	u32 num_in_message_queues;
	unsigned int max_streams;
	unsigned int i;

	max_streams = isys->pdata->ipdata->max_streams;
	max_send_queues = isys->pdata->ipdata->max_send_queues;
	max_sram_blocks = isys->pdata->ipdata->max_sram_blocks;
	max_devq_size = isys->pdata->ipdata->max_devq_size;
	num_in_message_queues = clamp(num_streams, 1U, max_streams);
	isys_fw_cfg = kzalloc_obj(*isys_fw_cfg);
	if (!isys_fw_cfg)
		return -ENOMEM;

	isys_fw_cfg->num_send_queues[type_proxy] = IPU4_N_MAX_PROXY_SEND_QUEUES;
	isys_fw_cfg->num_send_queues[type_dev] = IPU4_N_MAX_DEV_SEND_QUEUES;
	isys_fw_cfg->num_send_queues[type_msg] = num_in_message_queues;
	isys_fw_cfg->num_recv_queues[type_proxy] = IPU4_N_MAX_PROXY_RECV_QUEUES;
	/* Common msg/dev return queue */
	isys_fw_cfg->num_recv_queues[type_dev] = 0;
	isys_fw_cfg->num_recv_queues[type_msg] = 1;

	input_queue_cfg = kzalloc_objs(*input_queue_cfg, max_send_queues);
	if (!input_queue_cfg)
		goto err_free_fw_cfg;

	output_queue_cfg = kzalloc_objs(*output_queue_cfg,
					IPU4_N_MAX_RECV_QUEUES);
	if (!output_queue_cfg)
		goto err_free_input_queue_cfg;

	fwcom_cfg->input = input_queue_cfg;
	fwcom_cfg->output = output_queue_cfg;

	fwcom_cfg->num_input_queues =
		isys_fw_cfg->num_send_queues[type_proxy] +
		isys_fw_cfg->num_send_queues[type_dev] +
		isys_fw_cfg->num_send_queues[type_msg];

	fwcom_cfg->num_output_queues =
		isys_fw_cfg->num_recv_queues[type_proxy] +
		isys_fw_cfg->num_recv_queues[type_dev] +
		isys_fw_cfg->num_recv_queues[type_msg];

	/* SRAM partitioning. Equal partitioning is set. */
	for (i = 0; i < max_sram_blocks; i++) {
		if (i < num_in_message_queues)
			isys_fw_cfg->buffer_partition.num_gda_pages[i] =
				(IPU6_DEVICE_GDA_NR_PAGES *
				 IPU6_DEVICE_GDA_VIRT_FACTOR) /
				num_in_message_queues;
		else
			isys_fw_cfg->buffer_partition.num_gda_pages[i] = 0;
	}

	/* FW assumes proxy interface at fwcom_cfg queue 0 */
	for (i = 0; i < isys_fw_cfg->num_send_queues[type_proxy]; i++) {
		input_queue_cfg[i].token_size =
			sizeof(struct ipu6_fw_proxy_send_queue_token);
		input_queue_cfg[i].queue_size = IPU6_ISYS_SIZE_PROXY_SEND_QUEUE;
	}

	for (i = 0; i < isys_fw_cfg->num_send_queues[type_dev]; i++) {
		input_queue_cfg[base_dev_send + i].token_size =
			sizeof(struct ipu6_fw_send_queue_token);
		input_queue_cfg[base_dev_send + i].queue_size = max_devq_size;
	}

	for (i = 0; i < isys_fw_cfg->num_send_queues[type_msg]; i++) {
		input_queue_cfg[base_msg_send + i].token_size =
			sizeof(struct ipu6_fw_send_queue_token);
		input_queue_cfg[base_msg_send + i].queue_size =
			IPU6_ISYS_SIZE_SEND_QUEUE;
	}

	for (i = 0; i < isys_fw_cfg->num_recv_queues[type_proxy]; i++) {
		output_queue_cfg[i].token_size =
			sizeof(struct ipu6_fw_proxy_resp_queue_token);
		output_queue_cfg[i].queue_size =
			IPU6_ISYS_SIZE_PROXY_RECV_QUEUE;
	}
	/* There is no recv DEV queue */
	for (i = 0; i < isys_fw_cfg->num_recv_queues[type_msg]; i++) {
		output_queue_cfg[base_msg_recv + i].token_size =
			sizeof(struct ipu4_fw_resp_queue_token);
		output_queue_cfg[base_msg_recv + i].queue_size =
			IPU6_ISYS_SIZE_RECV_QUEUE;
	}

	fwcom_cfg->dmem_addr = isys->pdata->ipdata->hw_variant.dmem_offset;
	fwcom_cfg->dmem_syscom = true;
	fwcom_cfg->specific_addr = isys_fw_cfg;
	fwcom_cfg->specific_size = sizeof(*isys_fw_cfg);

	return 0;

err_free_input_queue_cfg:
	kfree(input_queue_cfg);
err_free_fw_cfg:
	kfree(isys_fw_cfg);

	return -ENOMEM;
}

static void ipu4_isys_fwcom_cfg_free(struct ipu6_fw_com_cfg *fwcom_cfg)
{
	kfree(fwcom_cfg->specific_addr);
	kfree(fwcom_cfg->output);
	kfree(fwcom_cfg->input);
}

static int ipu4_fw_isys_init(struct ipu6_isys *isys, unsigned int num_streams)
{
	struct device *dev = &isys->adev->auxdev.dev;
	int retry = IPU6_ISYS_OPEN_RETRY;
	struct ipu6_fw_com_cfg fwcom_cfg = {
		.cell_start = start_sp,
		.cell_ready = query_sp,
		.buttress_boot_offset = SYSCOM_BUTTRESS_FW_PARAMS_ISYS_OFFSET,
	};
	int ret;

	ret = ipu4_isys_fwcom_cfg_init(isys, &fwcom_cfg, num_streams);
	if (ret)
		return ret;

	isys->fwctx = ipu6_fw_com_prepare(&fwcom_cfg, isys->adev,
					  isys->pdata->base);
	ipu4_isys_fwcom_cfg_free(&fwcom_cfg);
	if (!isys->fwctx) {
		dev_err(dev, "isys fw com prepare failed\n");
		return -EIO;
	}

	ret = ipu6_fw_com_open(isys->fwctx);
	if (ret) {
		dev_err(dev, "isys fw com open failed %d\n", ret);
		return ret;
	}

	do {
		usleep_range(400, 500);
		if (ipu6_fw_com_ready(isys->fwctx))
			break;
		retry--;
	} while (retry > 0);

	if (!retry) {
		dev_err(dev, "isys port open ready failed %d\n", ret);
		ipu4_fw_isys_close(isys);
		ret = -EIO;
	}

	return ret;
}

static struct ipu4_fw_isys_resp_info_abi *
ipu4_fw_isys_get_resp(struct ipu6_isys *isys)
{
	return ipu6_recv_get_token(isys->fwctx, IPU4_BASE_MSG_RECV_QUEUES);
}

static void ipu4_fw_isys_put_resp(struct ipu6_isys *isys)
{
	ipu6_recv_put_token(isys->fwctx, IPU4_BASE_MSG_RECV_QUEUES);
}

static void ipu4_fw_isys_dump_stream_cfg(struct device *dev,
				  struct ipu4_fw_isys_stream_cfg_data_abi *cfg)
{
	unsigned int i;

	dev_dbg(dev, "---------------------------\n");
	dev_dbg(dev, "IPU_FW_ISYS_STREAM_CFG_DATA\n");
	dev_dbg(dev, "---------------------------\n");

	dev_dbg(dev, "crop[0] offset: %lx",
		(uintptr_t)&cfg->crop[0] - (uintptr_t)cfg);
	dev_dbg(dev, "input_pins[0] offset: %lx",
		(uintptr_t)&cfg->input_pins[0] - (uintptr_t)cfg);
	dev_dbg(dev, "output_pins[0] offset: %lx",
		(uintptr_t)&cfg->output_pins[0] - (uintptr_t)cfg);
	dev_dbg(dev, "compfmt offset: %lx",
		(uintptr_t)&cfg->compfmt - (uintptr_t)cfg);

	dev_dbg(dev, "Source %d\n", cfg->src);
	dev_dbg(dev, "VC %d\n", cfg->vc);
	dev_dbg(dev, "Nof input pins %d\n", cfg->nof_input_pins);
	dev_dbg(dev, "Nof output pins %d\n", cfg->nof_output_pins);

	for (i = 0; i < cfg->nof_input_pins; i++) {
		dev_dbg(dev, "Input pin %d\n", i);
		dev_dbg(dev, "Mipi data type 0x%0x\n",
			cfg->input_pins[i].dt);
		dev_dbg(dev, "Mipi store mode %d\n",
			cfg->input_pins[i].mipi_store_mode);
		dev_dbg(dev, "Bits per pixel %d\n",
			cfg->input_pins[i].bits_per_pix);
		dev_dbg(dev, "Mapped data type 0x%0x\n",
			cfg->input_pins[i].mapped_dt);
		dev_dbg(dev, "Input res width %d\n",
			cfg->input_pins[i].input_res.width);
		dev_dbg(dev, "Input res height %d\n",
			cfg->input_pins[i].input_res.height);
	}

	for (i = 0; i < N_IPU_FW_ISYS_CROPPING_LOCATION; i++) {
		dev_dbg(dev, "Crop info %d\n", i);
		dev_dbg(dev, "Crop.top_offset %d\n",
			cfg->crop[i].top_offset);
		dev_dbg(dev, "Crop.left_offset %d\n",
			cfg->crop[i].left_offset);
		dev_dbg(dev, "Crop.bottom_offset %d\n",
			cfg->crop[i].bottom_offset);
		dev_dbg(dev, "Crop.right_offset %d\n",
			cfg->crop[i].right_offset);
		dev_dbg(dev, "----------------\n");
	}

	for (i = 0; i < cfg->nof_output_pins; i++) {
		dev_dbg(dev, "Output pin %d\n", i);
		dev_dbg(dev, "Output input pin id %d\n",
			cfg->output_pins[i].input_pin_id);
		dev_dbg(dev, "Output res width %d\n",
			cfg->output_pins[i].output_res.width);
		dev_dbg(dev, "Output res height %d\n",
			cfg->output_pins[i].output_res.height);
		dev_dbg(dev, "Payload buf size: %d\n",
			cfg->output_pins[i].payload_buf_size);
		dev_dbg(dev, "Stride %d\n", cfg->output_pins[i].stride);
		dev_dbg(dev, "Pin type %d\n", cfg->output_pins[i].pt);
		dev_dbg(dev, "Ft %d\n", cfg->output_pins[i].ft);
		dev_dbg(dev, "Watermar in lines %d\n",
			cfg->output_pins[i].watermark_in_lines);
		dev_dbg(dev, "Send irq %d\n",
			cfg->output_pins[i].send_irq);
		dev_dbg(dev, "Reserve compression %d\n",
			cfg->output_pins[i].reserve_compression);
		dev_dbg(dev, "----------------\n");
	}
}

static void
ipu4_fw_isys_dump_frame_buff_set(struct device *dev,
				 struct ipu4_fw_isys_frame_buff_set_abi *buf,
				 unsigned int outputs)
{
	unsigned int i;

	dev_dbg(dev, "-----------------------------------------------------\n");
	dev_dbg(dev, "FRAME_BUFF_SET with %d outputs\n", outputs);
	for (i = 0; i < outputs; i++) {
		dev_dbg(dev, "output_pin[%d]:\n", i);
		dev_dbg(dev, "\t.out_buf_id = %llu\n",
			buf->output_pins[i].out_buf_id);
		dev_dbg(dev, "\t.addr = 0x%x\n", buf->output_pins[i].addr);
		dev_dbg(dev, "\t.compress = %d\n",
			buf->output_pins[i].compress);
	}

	dev_dbg(dev, "send_irq_sof = 0x%x\n", buf->send_irq_sof);
	dev_dbg(dev, "send_irq_eof = 0x%x\n", buf->send_irq_eof);
	dev_dbg(dev, "send_resp_sof = 0x%x\n", buf->send_resp_sof);
	dev_dbg(dev, "send_resp_eof = 0x%x\n", buf->send_resp_eof);
	dev_dbg(dev, "send_irq_capture_ack = 0x%x\n",
		buf->send_irq_capture_ack);
	dev_dbg(dev, "send_irq_capture_done = 0x%x\n",
		buf->send_irq_capture_done);

	dev_dbg(dev, "-----------------------------------------------------\n");
}

static void ipu4_fw_isys_dump_cfg(struct device *dev,
					 struct isys_fw_msgs *msg)
{
	ipu4_fw_isys_dump_stream_cfg(dev, &msg->ipu4.stream);
}

static void
ipu4_fw_isys_dump_buf_set(struct device *dev, struct isys_fw_msgs *msg,
				unsigned int outputs)
{
	ipu4_fw_isys_dump_frame_buff_set(dev, &msg->ipu4.frame, outputs);
}

struct fwmsg {
	int type;
	char *msg;
	bool valid_ts;
};

static const struct fwmsg fw_msg[] = {
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_OPEN_DONE, "STREAM_OPEN_DONE", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_CLOSE_ACK, "STREAM_CLOSE_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_START_ACK, "STREAM_START_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK,
	 "STREAM_START_AND_CAPTURE_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_STOP_ACK, "STREAM_STOP_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_FLUSH_ACK, "STREAM_FLUSH_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_READY, "PIN_DATA_READY", 1},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK, "STREAM_CAPTURE_ACK", 0},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE,
	 "STREAM_START_AND_CAPTURE_DONE", 1},
	{IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE, "STREAM_CAPTURE_DONE", 1},
	{IPU6_FW_ISYS_RESP_TYPE_FRAME_SOF, "FRAME_SOF", 1},
	{IPU6_FW_ISYS_RESP_TYPE_FRAME_EOF, "FRAME_EOF", 1},
	{IPU6_FW_ISYS_RESP_TYPE_STATS_DATA_READY, "STATS_READY", 1},
	{-1, "UNKNOWN MESSAGE", 0}
};

static u32 resp_type_to_index(int type)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(fw_msg); i++)
		if (fw_msg[i].type == type)
			return i;

	return  ARRAY_SIZE(fw_msg) - 1;
}

static struct isys_fw_msgs *ipu4_fw_msg_from_handle(u64 handle)
{
	if (!handle)
		return NULL;

	return container_of((void *)(uintptr_t)handle, struct isys_fw_msgs,
			    dummy);
}

static int ipu4_isys_isr_one(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	struct ipu4_fw_isys_resp_info_abi *resp;
	struct ipu6_isys_stream *stream;
	struct ipu6_isys_csi2 *csi2 = NULL;
	u32 index;
	u64 ts;

	resp = ipu4_fw_isys_get_resp(isys);
	if (!resp)
		return 1;

	ts = (u64)resp->timestamp[1] << 32 | resp->timestamp[0];

	index = resp_type_to_index(resp->type);
	dev_dbg(&adev->auxdev.dev,
		"FW resp %02d %s, stream %u, ts 0x%16.16llx, pin %d\n",
		resp->type, fw_msg[index].msg, resp->stream_handle,
		fw_msg[index].valid_ts ? ts : 0, resp->pin_id);

	if (resp->error_info.error == IPU6_FW_ISYS_ERROR_STREAM_IN_SUSPENSION)
		/* Suspension is kind of special case: not enough buffers */
		dev_dbg(&adev->auxdev.dev,
			"FW error resp SUSPENSION, details %d\n",
			resp->error_info.error_details);
	else if (resp->error_info.error)
		dev_dbg(&adev->auxdev.dev,
			"FW error resp error %d, details %d\n",
			resp->error_info.error, resp->error_info.error_details);

	guard(spinlock_irqsave)(&isys->streams_lock);

	if (resp->stream_handle >= IPU4_ISYS_MAX_STREAMS) {
		dev_err(&adev->auxdev.dev, "bad stream handle %u\n",
			resp->stream_handle);
		goto leave;
	}

	stream = resp->stream_handle < IPU6_ISYS_MAX_STREAMS ?
		isys->streams_by_handle[resp->stream_handle] : NULL;
	if (!stream) {
		dev_err(&adev->auxdev.dev, "stream of stream_handle %u is unused\n",
			resp->stream_handle);
		goto leave;
	}
	stream->error = resp->error_info.error;

	csi2 = ipu6_isys_subdev_to_csi2(stream->asd);

	switch (resp->type) {
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_OPEN_DONE:
		complete(&stream->stream_open_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_CLOSE_ACK:
		complete(&stream->stream_close_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_START_ACK:
		complete(&stream->stream_start_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK:
		ipu6_put_fw_msg_buf(ipu6_bus_get_drvdata(adev),
				    ipu4_fw_msg_from_handle(resp->buf_handle));
		complete(&stream->stream_start_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_STOP_ACK:
		complete(&stream->stream_stop_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_FLUSH_ACK:
		complete(&stream->stream_stop_completion);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_READY:
		if (resp->pin_id < IPU6_ISYS_OUTPUT_PINS &&
		    stream->output_pins_queue[resp->pin_id])
			ipu6_stream_buf_ready(stream, resp->pin_id, resp->pin.addr,
					      ts, resp->error_info.error ==
					      IPU6_FW_ISYS_ERROR_HW_REPORTED_STR2MMIO);
		else
			dev_warn(&adev->auxdev.dev,
				 "%d:No queue for pin id %d\n",
				 resp->stream_handle, resp->pin_id);
		if (csi2)
			ipu4_isys_csi2_error(csi2);

		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK:
		ipu6_put_fw_msg_buf(ipu6_bus_get_drvdata(adev),
				    ipu4_fw_msg_from_handle(resp->buf_handle));
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE:
	case IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE:
		break;
	case IPU6_FW_ISYS_RESP_TYPE_FRAME_SOF:

		ipu6_isys_csi2_sof_event_by_stream(stream);
		stream->seq[stream->seq_index].sequence =
			atomic_read(&stream->sequence) - 1;
		stream->seq[stream->seq_index].timestamp = ts;
		dev_dbg(&adev->auxdev.dev,
			"sof: handle %d: (index %u), timestamp 0x%16.16llx\n",
			resp->stream_handle,
			stream->seq[stream->seq_index].sequence, ts);
		stream->seq_index = (stream->seq_index + 1)
			% IPU6_ISYS_MAX_PARALLEL_SOF;
		break;
	case IPU6_FW_ISYS_RESP_TYPE_FRAME_EOF:
		dev_dbg(&adev->auxdev.dev,
			"eof: handle %d: (index %u), timestamp 0x%16.16llx\n",
			resp->stream_handle,
			stream->seq[stream->seq_index].sequence, ts);
		break;
	case IPU6_FW_ISYS_RESP_TYPE_STATS_DATA_READY:
		break;
	default:
		dev_err(&adev->auxdev.dev, "%d:unknown response type %u\n",
			resp->stream_handle, resp->type);
		break;
	}

leave:
	ipu4_fw_isys_put_resp(isys);
	return 0;
}

static void ipu4_isys_csi2_isr(struct ipu6_isys_csi2 *csi2)
{
	struct ipu6_isys_stream *stream;
	unsigned int i;
	u32 status;

	status = readl(csi2->base + CSI2_REG_CSI2PART_IRQ_STATUS);
	writel(status, csi2->base + CSI2_REG_CSI2PART_IRQ_CLEAR);

	if (status & CSI2_CSI2PART_IRQ_CSIRX)
		ipu4_isys_register_errors(csi2);
	/* SOF and EOF come from the firmware responses on IPU4. */
	return;

	scoped_guard(spinlock, &csi2->isys->streams_lock) {
		for (i = 0; i < NR_OF_CSI2_VC; i++) {
			if (status & IPU_CSI_RX_IRQ_FS_VC(i)) {
				stream = csi2->streams_by_vc[i];
				if (!stream)
					continue;

				ipu6_isys_csi2_sof_event_by_stream(stream);
			}

			if (status & IPU_CSI_RX_IRQ_FE_VC(i)) {
				stream = csi2->streams_by_vc[i];
				if (!stream)
					continue;

				ipu6_isys_csi2_eof_event_by_stream(stream);
			}
		}
	}
}

irqreturn_t ipu4_isys_isr(struct ipu6_bus_device *adev)
{
	struct ipu6_isys *isys = ipu6_bus_get_drvdata(adev);
	void __iomem *base = isys->pdata->base;
	int pm_status;
	u32 status;

	pm_status = pm_runtime_get_if_active(&adev->auxdev.dev);
	if (!pm_status)
		return IRQ_NONE;

	status = readl(base + IPU4_REG_ISYS_UNISPART_IRQ_STATUS);
	do {
		writel(status, base + IPU4_REG_ISYS_UNISPART_IRQ_CLEAR);

		if (isys->isr_csi2_bits & status) {
			unsigned int i;

			for (i = 0; i < isys->pdata->ipdata->csi2.nports; i++) {
				if (IPU4_ISYS_UNISPART_IRQ_CSI2(i) & status)
					ipu4_isys_csi2_isr(&isys->csi2[i]);
			}
		}

		writel(0, base + IPU4_REG_ISYS_UNISPART_SW_IRQ_REG);

		/*
		 * Handle a single FW event per checking the CSI-2
		 * receiver SOF status. This is done in order to avoid
		 * the case where events arrive to the event queue and
		 * one of them is a SOF event which then could be
		 * handled before the SOF interrupt. This would pose
		 * issues in sequence numbering which is based on SOF
		 * interrupts, always assumed to arrive before FW SOF
		 * events.
		 */
		if (status & IPU4_ISYS_UNISPART_IRQ_SW && !ipu4_isys_isr_one(adev))
			status = IPU4_ISYS_UNISPART_IRQ_SW;
		else
			status = 0;

		status |= readl(isys->pdata->base +
				    IPU4_REG_ISYS_UNISPART_IRQ_STATUS);
	} while (status & (isys->isr_csi2_bits | IPU4_ISYS_UNISPART_IRQ_SW));

	if (pm_status > 0)
		pm_runtime_put(&adev->auxdev.dev);

	return IRQ_HANDLED;
}

static int ipu4_isys_fw_pin_cfg(struct ipu6_isys_video *av,
				struct ipu6_isys_stream *stream,
				struct media_pad *src_pad,
				struct v4l2_mbus_frame_desc_entry *entry,
				void *__cfg)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(src_pad->entity);
	struct v4l2_subdev_state *state = v4l2_subdev_get_locked_active_state(sd);
	struct ipu4_fw_isys_stream_cfg_data_abi *cfg = __cfg;
	struct ipu4_fw_isys_input_pin_info_abi *input_pin;
	struct ipu4_fw_isys_output_pin_info_abi *output_pin;
	struct ipu6_isys_queue *aq = &av->aq;
	struct v4l2_mbus_framefmt *fmt;
	const struct ipu6_isys_pixelformat *pfmt =
		ipu6_isys_get_isys_format(ipu6_isys_get_format(av), 0);
	struct v4l2_rect v4l2_crop;
	int input_pins = cfg->nof_input_pins++;
	int output_pins;
	u32 src_stream;

	src_stream = __ipu6_isys_get_src_stream_by_src_pad(state, src_pad->index);
	fmt = v4l2_subdev_state_get_format(state, src_pad->index, src_stream);
	v4l2_crop = *v4l2_subdev_state_get_crop(state, src_pad->index, src_stream);

	input_pin = &cfg->input_pins[input_pins];
	input_pin->input_res.width = fmt->width;
	input_pin->input_res.height = fmt->height;
	input_pin->dt = entry->bus.csi2.dt;
	input_pin->bits_per_pix = pfmt->bpp_packed;
	input_pin->mapped_dt = 0x40; /* invalid mipi data type */
	input_pin->mipi_store_mode = pfmt->bpp == pfmt->bpp_packed ?
		IPU6_FW_ISYS_MIPI_STORE_MODE_DISCARD_LONG_HEADER :
		IPU6_FW_ISYS_MIPI_STORE_MODE_NORMAL;
	output_pins = cfg->nof_output_pins++;
	aq->fw_output = output_pins;
	stream->output_pins_queue[output_pins] = aq;

	output_pin = &cfg->output_pins[output_pins];
	output_pin->input_pin_id = input_pins;
	output_pin->output_res.width = ipu6_isys_get_frame_width(av);
	output_pin->output_res.height = ipu6_isys_get_frame_height(av);

	output_pin->stride = ipu6_isys_get_bytes_per_line(av);
	if (pfmt->bpp != pfmt->bpp_packed)
		output_pin->pt = IPU6_FW_ISYS_PIN_TYPE_RAW_SOC;
	else
		output_pin->pt = IPU6_FW_ISYS_PIN_TYPE_MIPI;
	output_pin->ft = pfmt->css_pixelformat;
	output_pin->send_irq = 1;
	/* Though payload_buf_size was added for compression, set sane value for
	 * payload_buf_size, just in case...
	 */
	output_pin->payload_buf_size = output_pin->stride *
				       output_pin->output_res.height;

	return 0;
}

static int ipu4_fw_isys_prepare_stream_cfg(struct ipu6_isys_stream *stream,
					   struct v4l2_mbus_frame_desc *desc,
					   struct isys_fw_msgs *msg)
{
	struct ipu4_fw_isys_stream_cfg_data_abi *stream_cfg;
	struct device *dev = &stream->isys->adev->auxdev.dev;
	int ret;

	stream_cfg = &msg->ipu4.stream;
	stream_cfg->src = stream->asd->source;
	stream_cfg->vc = stream->vc;
	stream_cfg->isl_use = 0;

	ret = ipu6_isys_fw_pins_prepare(stream, desc, ipu4_isys_fw_pin_cfg,
					stream_cfg);
	if (ret)
		return ret;

	ipu4_fw_isys_dump_cfg(dev, msg);

	stream->nr_output_pins = stream_cfg->nof_output_pins;

	return 0;
}

static int ipu4_fw_isys_stream_open(struct ipu6_isys *isys,
				    const unsigned int stream_handle,
				    struct isys_fw_msgs *msg)
{
	return ipu4_fw_isys_complex_cmd(isys, stream_handle,
				       &msg->ipu4.stream, msg->dma_addr,
				       sizeof(msg->ipu4.stream),
				       IPU6_FW_ISYS_SEND_TYPE_STREAM_OPEN);
}

static int ipu4_fw_isys_stream_close(struct ipu6_isys *isys,
				     const unsigned int stream_handle)
{
	return ipu4_fw_isys_complex_cmd(isys, stream_handle, NULL, 0, 0,
					IPU6_FW_ISYS_SEND_TYPE_STREAM_CLOSE);
}

static int ipu4_fw_isys_stream_flush(struct ipu6_isys *isys,
				     const unsigned int stream_handle)
{
	return ipu4_fw_isys_complex_cmd(isys, stream_handle, NULL, 0, 0,
					IPU6_FW_ISYS_SEND_TYPE_STREAM_FLUSH);
}

static int ipu4_fw_isys_stream_start(struct ipu6_isys *isys,
				     const unsigned int stream_handle,
				     struct isys_fw_msgs *msg, bool capture)
{
	u16 cmd_type;

	if (capture)
		cmd_type = IPU6_FW_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE;
	else
		cmd_type = IPU6_FW_ISYS_SEND_TYPE_STREAM_START;

	return ipu4_fw_isys_complex_cmd(isys, stream_handle,
					&msg->ipu4.stream, msg->dma_addr,
					sizeof(msg->ipu4.stream), cmd_type);
}

static int ipu4_fw_isys_stream_capture(struct ipu6_isys *isys,
				       const unsigned int stream_handle,
				       struct isys_fw_msgs *msg)
{
	return ipu4_fw_isys_complex_cmd(isys, stream_handle,
					&msg->ipu4.stream, msg->dma_addr,
					sizeof(msg->ipu4.stream),
					IPU6_FW_ISYS_SEND_TYPE_STREAM_CAPTURE);
}

static void
ipu4_isys_buf_to_fw_frame_buf_pin(struct vb2_buffer *vb,
				  struct ipu4_fw_isys_frame_buff_set_abi *set)
{
	struct ipu6_isys_queue *aq = vb2_queue_to_isys_queue(vb->vb2_queue);
	struct vb2_v4l2_buffer *vvb = to_vb2_v4l2_buffer(vb);
	struct ipu6_isys_video_buffer *ivb =
		vb2_buffer_to_ipu6_isys_video_buffer(vvb);

	set->output_pins[aq->fw_output].addr = ivb->dma_addr;
	set->output_pins[aq->fw_output].out_buf_id = vb->index + 1;
}

/*
 * Convert a buffer list to a isys fw ABI framebuffer set. The
 * buffer list is not modified.
 */
#define IPU6_ISYS_FRAME_NUM_THRESHOLD  (30)
static void ipu4_fw_isys_prepare_buf_set(struct isys_fw_msgs *msg,
					 struct ipu6_isys_stream *stream,
					 struct ipu6_isys_buffer_list *bl)
{
	struct ipu4_fw_isys_frame_buff_set_abi *set = &msg->ipu4.frame;
	struct ipu6_isys_buffer *ib;

	WARN_ON(!bl->nbufs);

	set->send_irq_sof = 1;
	set->send_resp_sof = 1;

	set->send_irq_capture_ack = 1;
	set->send_irq_capture_done = 1;
	set->send_irq_eof = 1;
	set->send_resp_eof = 1;

	list_for_each_entry(ib, &bl->head, head) {
		struct vb2_buffer *vb = ipu6_isys_buffer_to_vb2_buffer(ib);

		ipu4_isys_buf_to_fw_frame_buf_pin(vb, set);
	}
}

const struct ipu6_fw_isys_ops ipu4_fw_isys_ops = {
	.init = ipu4_fw_isys_init,
	.close = ipu4_fw_isys_close,
	.cleanup = ipu4_fw_isys_cleanup,
	.prepare_stream_cfg = ipu4_fw_isys_prepare_stream_cfg,
	.prepare_buf_set = ipu4_fw_isys_prepare_buf_set,
	.stream_open = ipu4_fw_isys_stream_open,
	.stream_start = ipu4_fw_isys_stream_start,
	.stream_capture = ipu4_fw_isys_stream_capture,
	.stream_flush = ipu4_fw_isys_stream_flush,
	.stream_close = ipu4_fw_isys_stream_close,
	.dump_stream_cfg = ipu4_fw_isys_dump_cfg,
	.dump_frame_buf_set = ipu4_fw_isys_dump_buf_set,
};
