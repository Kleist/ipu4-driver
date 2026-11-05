/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013--2024 Intel Corporation */
/* IPU4 ISYS firmware ABI; the shared parts are in ipu6-fw-isys.h. */

#ifndef IPU4_FW_ISYS_H
#define IPU4_FW_ISYS_H

#include <linux/irqreturn.h>
#include <linux/types.h>

struct ipu6_bus_device;

#include "ipu6-fw-isys.h"

#define IPU4_MAX_IPINS 4

/* worst case is ISA use where a single input pin produces:
 * Mipi output, NS Pixel Output, and Scaled Pixel Output.
 * This is how the 2 is calculated
 */
#define IPU4_MAX_OPINS ((IPU4_MAX_IPINS) + 2)

/* Max number of supported virtual streams */
#define IPU4_STREAM_ID_MAX (8)

/* Aligned with the approach of having one dedicated per stream */
#define IPU4_N_MAX_MSG_SEND_QUEUES (IPU4_STREAM_ID_MAX)
/* Single return queue for all streams/commands type */
#define IPU4_N_MAX_MSG_RECV_QUEUES (1)
/* Single device queue for high priority commands (bypass in-order queue) */
#define IPU4_N_MAX_DEV_SEND_QUEUES (1)
/* Single dedicated send queue for proxy interface */
#define IPU4_N_MAX_PROXY_SEND_QUEUES (1)
/* Single dedicated recv queue for proxy interface */
#define IPU4_N_MAX_PROXY_RECV_QUEUES (1)
/* Send queues layout */
#define IPU4_BASE_PROXY_SEND_QUEUES (0)
#define IPU4_BASE_DEV_SEND_QUEUES \
	(IPU4_BASE_PROXY_SEND_QUEUES + IPU4_N_MAX_PROXY_SEND_QUEUES)
#define IPU4_BASE_MSG_SEND_QUEUES \
	(IPU4_BASE_DEV_SEND_QUEUES + IPU4_N_MAX_DEV_SEND_QUEUES)
#define IPU4_N_MAX_SEND_QUEUES \
	(IPU4_BASE_MSG_SEND_QUEUES + IPU4_N_MAX_MSG_SEND_QUEUES)
/* Recv queues layout */
#define IPU4_BASE_PROXY_RECV_QUEUES (0)
#define IPU4_BASE_MSG_RECV_QUEUES \
	(IPU4_BASE_PROXY_RECV_QUEUES + IPU4_N_MAX_PROXY_RECV_QUEUES)
#define IPU4_N_MAX_RECV_QUEUES \
	(IPU4_BASE_MSG_RECV_QUEUES + IPU4_N_MAX_MSG_RECV_QUEUES)

#define IPU4_MAX_QUEUE_SIZE (256)
#define IPU4_MIN_QUEUE_SIZE (1)

/**
 * enum ipu_fw_isys_cropping_location. Enumerates the cropping locations in ISYS
 */
enum ipu4_fw_isys_cropping_location {
	/* Cropping executed in ISAPF (mainly),
	 * ISAPF preproc (odd column) and MIPI STR2MMIO (odd row)
	 */
	IPU_FW_ISYS_CROPPING_LOCATION_PRE_ISA = 0,
	/* Reserved for legacy mode which will never be implemented */
	IPU_FW_ISYS_CROPPING_LOCATION_RESERVED_1,
	/* Cropping executed in StreamPifConv in the ISA output for
	 * RAW_NS pin
	 */
	IPU_FW_ISYS_CROPPING_LOCATION_POST_ISA_NONSCALED,
	/* Cropping executed in StreamScaledPifConv
	 * in the ISA output for RAW_S pin
	 */
	IPU_FW_ISYS_CROPPING_LOCATION_POST_ISA_SCALED,
	N_IPU_FW_ISYS_CROPPING_LOCATION
};

/**
 * enum ipu_fw_isys_resolution_info. Describes the resolution,
 * required to setup the various ISA GP registers.
 */
enum ipu4_fw_isys_resolution_info {
	/* Scaled ISA output resolution before
	 * the StreamScaledPifConv cropping
	 */
	IPU_FW_ISYS_RESOLUTION_INFO_POST_ISA_NONSCALED = 0,
	/* Non-Scaled ISA output resolution before the StreamPifConv cropping */
	IPU_FW_ISYS_RESOLUTION_INFO_POST_ISA_SCALED,
	N_IPU_FW_ISYS_RESOLUTION_INFO
};

struct ipu4_fw_isys_buffer_partition_abi {
	u32 num_gda_pages[IPU4_STREAM_ID_MAX];
};

struct ipu4_fw_isys_fw_config {
	struct ipu4_fw_isys_buffer_partition_abi buffer_partition;
	u32 num_send_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
	u32 num_recv_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
};

/**
 * struct ipu4_fw_isys_output_pin_info_abi
 * @output_res: output pin resolution
 * @stride: output stride in Bytes (not valid for statistics)
 * @watermark_in_lines: pin watermark level in lines
 * @payload_buf_size: minimum size in Bytes of all buffers that will be
 *			supplied for capture on this pin
 * @send_irq: assert if pin event should trigger irq
 * @pt: pin type -real format "enum ipu_fw_isys_pin_type"
 * @ft: frame format type -real format "enum ipu_fw_isys_frame_format_type"
 * @input_pin_id: related input pin id
 * @reserve_compression: reserve compression resources for pin
 */
struct ipu4_fw_isys_output_pin_info_abi {
	// AKA ia_css_isys_output_pin_info_comm in old driver
	struct ipu6_fw_isys_resolution_abi output_res;	/* 0x00 */
	u32 stride;					/* 0x08 */
	u32 watermark_in_lines;				/* 0x0b */
	u32 payload_buf_size;				/* 0x10 */
	u8 send_irq;					/* 0x14 */
	u8 input_pin_id;				/* 0x15 */
	u8 pt;						/* 0x16 */
	u8 ft;						/* 0x17 */
	u8 link_id;					/* 0x18 */
	u8 reserve_compression;				/* 0x19 */
	/* 0x1a..0x1b?  auto-padding*/
};

/**
 * struct ipu4_fw_isys_input_pin_info_abi
 * @input_res: input resolution
 * @dt: mipi data type ((enum ipu_fw_isys_mipi_data_type)
 * @mipi_store_mode: defines if legacy long packet header will be stored or
 *			discarded if discarded, output pin type for this
 *			input pin can only be MIPI
 *			(enum ipu_fw_isys_mipi_store_mode)
 * @bits_per_pix: native bits per pixel
 * @mapped_dt: actual data type from sensor
 */
struct ipu4_fw_isys_input_pin_info_abi {
	struct ipu6_fw_isys_resolution_abi input_res;
	u8 dt;
	u8 mipi_store_mode;
	u8 bits_per_pix;
	u8 mapped_dt;
};

/**
 * struct ipu_fw_isys_isa_cfg_abi. Describes the ISA cfg
 */
struct ipu4_fw_isys_isa_cfg_abi {
	struct ipu6_fw_isys_resolution_abi
	 isa_res[N_IPU_FW_ISYS_RESOLUTION_INFO];
	union {
		u32 cfg_fields;
		struct {
			unsigned int blc:1;
			unsigned int lsc:1;
			unsigned int dpc:1;
			unsigned int downscaler:1;
			unsigned int awb:1;
			unsigned int af:1;
			unsigned int ae:1;
			unsigned int paf:8;
			unsigned int send_irq_stats_ready:1;
			unsigned int send_resp_stats_ready:1;
		} cfg;
	};
};

/**
 * struct ipu4_fw_isys_stream_cfg_data_abi
 * ISYS stream configuration data structure
 * @isa_cfg: details about what ACCs are active if ISA is used
 * @crop: defines cropping resolution for the
 * maximum number of input pins which can be cropped,
 * it is directly mapped to the HW devices
 * @input_pins: input pin descriptors
 * @output_pins: output pin descriptors
 * @compfmt: de-compression setting for User Defined Data
 * @nof_input_pins: number of input pins
 * @nof_output_pins: number of output pins
 * @send_irq_sof_discarded: send irq on discarded frame sof response
 *		- if '1' it will override the send_resp_sof_discarded
 *		  and send the response
 *		- if '0' the send_resp_sof_discarded will determine
 *		  whether to send the response
 * @send_irq_eof_discarded: send irq on discarded frame eof response
 *		- if '1' it will override the send_resp_eof_discarded
 *		  and send the response
 *		- if '0' the send_resp_eof_discarded will determine
 *		  whether to send the response
 * @send_resp_sof_discarded: send response for discarded frame sof detected,
 *			     used only when send_irq_sof_discarded is '0'
 * @send_resp_eof_discarded: send response for discarded frame eof detected,
 *			     used only when send_irq_eof_discarded is '0'
 * @src: Stream source index e.g. MIPI_generator_0, CSI2-rx_1
 * @vc: MIPI Virtual Channel (up to 4 virtual per physical channel)
 * @isl_use: indicates whether stream requires ISL and how
 */
struct ipu4_fw_isys_stream_cfg_data_abi {
	// AKA ia_css_isys_stream_cfg_data_comm in old
	struct ipu4_fw_isys_isa_cfg_abi isa_cfg;
	struct ipu6_fw_isys_cropping_abi crop[N_IPU_FW_ISYS_CROPPING_LOCATION];
	struct ipu4_fw_isys_input_pin_info_abi input_pins[IPU4_MAX_IPINS];
	struct ipu4_fw_isys_output_pin_info_abi output_pins[IPU4_MAX_OPINS];
	u32 compfmt;
	u8 nof_input_pins;
	u8 nof_output_pins;
	u8 send_irq_sof_discarded;
	u8 send_irq_eof_discarded;
	u8 send_resp_sof_discarded;
	u8 send_resp_eof_discarded;
	u8 src;
	u8 vc;
	u8 isl_use;
};

/**
 * struct ipu_fw_isys_param_pin_abi
 * @param_buf_id: Points to param port buffer - buffer identifier
 * @addr: Points to param pin buffer - CSS Virtual Address
 */
struct ipu_fw_isys_param_pin_abi { // AKA ia_css_isys_param_pin_comm
	u64 param_buf_id;
	u32 addr;
};

/**
 * struct ipu4_fw_isys_frame_buff_set_abi - frame buffer set
 * @output_pins: output pin addresses
 * @send_irq_sof: send irq on frame sof response
 *		- if '1' it will override the send_resp_sof and
 *		  send the response
 *		- if '0' the send_resp_sof will determine whether to
 *		  send the response
 * @send_irq_eof: send irq on frame eof response
 *		- if '1' it will override the send_resp_eof and
 *		  send the response
 *		- if '0' the send_resp_eof will determine whether to
 *		  send the response
 * @send_resp_sof: send response for frame sof detected,
 *		   used only when send_irq_sof is '0'
 * @send_resp_eof: send response for frame eof detected,
 *		   used only when send_irq_eof is '0'
 */
struct ipu4_fw_isys_frame_buff_set_abi { // AKA ia_css_isys_frame_buff_set_comm
	struct ipu6_fw_isys_output_pin_payload_abi output_pins[IPU4_MAX_OPINS];
	struct ipu_fw_isys_param_pin_abi process_group_light;
	u8 send_irq_sof;
	u8 send_irq_eof;
	u8 send_irq_capture_ack;
	u8 send_irq_capture_done;
	u8 send_resp_sof;
	u8 send_resp_eof;
	u8 reserved; // Was unused 'frame_counter'
};

/**
 * struct ipu4_fw_isys_resp_info_abi
 * @pin: this var is only valid for pin event related responses,
 *     contains pin addresses
 * @error_info: error information from the FW
 * @timestamp: Time information for event if available
 * @stream_handle: stream id the response corresponds to
 * @type: response type (enum ipu6_fw_isys_resp_type)
 * @pin_id: pin id that the pin payload corresponds to
 */
struct ipu4_fw_isys_resp_info_abi { // AKA ia_css_isys_resp_info_comm
	u64 buf_handle; // struct isys_fw_msgs*
	struct ipu6_fw_isys_output_pin_payload_abi pin;
	struct ipu_fw_isys_param_pin_abi process_group_light;
	struct ipu6_fw_isys_error_info_abi error_info;
	u32 timestamp[2];
	u8 stream_handle;
	u8 type;
	u8 pin_id;
	u8 acc_id;
	u8 frame_counter;
	u8 written_direct;
};

/* The response queue token: the IPU4 response, not IPU6's. */
struct ipu4_fw_resp_queue_token {
	struct ipu4_fw_isys_resp_info_abi resp_info;
};

extern const struct ipu6_fw_isys_ops ipu4_fw_isys_ops;
irqreturn_t ipu4_isys_isr(struct ipu6_bus_device *adev);

#endif /* IPU4_FW_ISYS_H */
