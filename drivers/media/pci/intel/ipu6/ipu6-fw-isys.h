/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 2013 - 2023 Intel Corporation */

#ifndef IPU6_FW_ISYS_H
#define IPU6_FW_ISYS_H

struct ipu6_isys;

/* Max number of Input/Output Pins */
#define IPU6_MAX_IPINS 4 // IPU4 OK
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

/* Max number of supported input pins routed in ISL */
#define IPU6_MAX_IPINS_IN_ISL 2

/* Max number of planes for frame formats supported by the FW */
#define IPU6_PIN_PLANES_MAX 4

/*
 * Device close takes some time from last ack message to actual stopping
 * of the SP processor. As long as the SP processor runs we can't proceed with
 * clean up of resources.
 */
#define IPU6_ISYS_OPEN_RETRY			2000
#define IPU6_ISYS_CLOSE_RETRY			2000
#define IPU6_FW_CALL_TIMEOUT_JIFFIES		msecs_to_jiffies(2000)

enum ipu6_fw_isys_resp_type {
	IPU6_FW_ISYS_RESP_TYPE_STREAM_OPEN_DONE = 0,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_START_ACK,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_STOP_ACK,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_FLUSH_ACK,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_CLOSE_ACK,
	IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_READY,
	IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_WATERMARK,
	IPU6_FW_ISYS_RESP_TYPE_FRAME_SOF,
	IPU6_FW_ISYS_RESP_TYPE_FRAME_EOF,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE,
	IPU6_FW_ISYS_RESP_TYPE_PIN_DATA_SKIPPED,
	IPU6_FW_ISYS_RESP_TYPE_STREAM_CAPTURE_SKIPPED,
	IPU6_FW_ISYS_RESP_TYPE_FRAME_SOF_DISCARDED,
	IPU6_FW_ISYS_RESP_TYPE_FRAME_EOF_DISCARDED,
	IPU6_FW_ISYS_RESP_TYPE_STATS_DATA_READY,
	N_IPU6_FW_ISYS_RESP_TYPE
};

enum ipu6_fw_isys_send_type {
	IPU6_FW_ISYS_SEND_TYPE_STREAM_OPEN = 0,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_START,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_CAPTURE,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_STOP,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_FLUSH,
	IPU6_FW_ISYS_SEND_TYPE_STREAM_CLOSE,
	N_IPU6_FW_ISYS_SEND_TYPE
};

enum ipu6_fw_isys_queue_type {
	IPU6_FW_ISYS_QUEUE_TYPE_PROXY = 0,
	IPU6_FW_ISYS_QUEUE_TYPE_DEV,
	IPU6_FW_ISYS_QUEUE_TYPE_MSG,
	N_IPU6_FW_ISYS_QUEUE_TYPE
};

enum ipu6_fw_isys_stream_source {
	IPU6_FW_ISYS_STREAM_SRC_PORT_0 = 0,
	IPU6_FW_ISYS_STREAM_SRC_PORT_1,
	IPU6_FW_ISYS_STREAM_SRC_PORT_2,
	IPU6_FW_ISYS_STREAM_SRC_PORT_3,
	IPU6_FW_ISYS_STREAM_SRC_PORT_4,
	IPU6_FW_ISYS_STREAM_SRC_PORT_5,
	IPU6_FW_ISYS_STREAM_SRC_PORT_6,
	IPU6_FW_ISYS_STREAM_SRC_PORT_7,
	IPU6_FW_ISYS_STREAM_SRC_PORT_8,
	IPU6_FW_ISYS_STREAM_SRC_PORT_9,
	IPU6_FW_ISYS_STREAM_SRC_PORT_10,
	IPU6_FW_ISYS_STREAM_SRC_PORT_11,
	IPU6_FW_ISYS_STREAM_SRC_PORT_12,
	IPU6_FW_ISYS_STREAM_SRC_PORT_13,
	IPU6_FW_ISYS_STREAM_SRC_PORT_14,
	IPU6_FW_ISYS_STREAM_SRC_PORT_15,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_0,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_1,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_2,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_3,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_4,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_5,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_6,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_7,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_8,
	IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_9,
	N_IPU6_FW_ISYS_STREAM_SRC
};

#define IPU6_FW_ISYS_STREAM_SRC_CSI2_PORT0 IPU6_FW_ISYS_STREAM_SRC_PORT_0
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_PORT1 IPU6_FW_ISYS_STREAM_SRC_PORT_1
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_PORT2 IPU6_FW_ISYS_STREAM_SRC_PORT_2
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_PORT3 IPU6_FW_ISYS_STREAM_SRC_PORT_3

#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_PORTA IPU6_FW_ISYS_STREAM_SRC_PORT_4
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_PORTB IPU6_FW_ISYS_STREAM_SRC_PORT_5
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_CPHY_PORT0 \
	IPU6_FW_ISYS_STREAM_SRC_PORT_6
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_CPHY_PORT1 \
	IPU6_FW_ISYS_STREAM_SRC_PORT_7
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_CPHY_PORT2 \
	IPU6_FW_ISYS_STREAM_SRC_PORT_8
#define IPU6_FW_ISYS_STREAM_SRC_CSI2_3PH_CPHY_PORT3 \
	IPU6_FW_ISYS_STREAM_SRC_PORT_9

#define IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_PORT0 IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_0
#define IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_PORT1 IPU6_FW_ISYS_STREAM_SRC_MIPIGEN_1

/**
 * enum ipu6_fw_isys_mipi_vc: MIPI csi2 spec
 * supports up to 4 virtual per physical channel
 */
enum ipu6_fw_isys_mipi_vc {
	IPU6_FW_ISYS_MIPI_VC_0 = 0,
	IPU6_FW_ISYS_MIPI_VC_1,
	IPU6_FW_ISYS_MIPI_VC_2,
	IPU6_FW_ISYS_MIPI_VC_3,
	N_IPU6_FW_ISYS_MIPI_VC
};

enum ipu6_fw_isys_frame_format_type {
	IPU6_FW_ISYS_FRAME_FORMAT_NV11 = 0, /* 12 bit YUV 411, Y, UV plane */
	IPU6_FW_ISYS_FRAME_FORMAT_NV12,	/* 12 bit YUV 420, Y, UV plane */
	IPU6_FW_ISYS_FRAME_FORMAT_NV12_16, /* 16 bit YUV 420, Y, UV plane */
	/* 12 bit YUV 420, Intel proprietary tiled format */
	IPU6_FW_ISYS_FRAME_FORMAT_NV12_TILEY,

	IPU6_FW_ISYS_FRAME_FORMAT_NV16,	/* 16 bit YUV 422, Y, UV plane */
	IPU6_FW_ISYS_FRAME_FORMAT_NV21,	/* 12 bit YUV 420, Y, VU plane */
	IPU6_FW_ISYS_FRAME_FORMAT_NV61,	/* 16 bit YUV 422, Y, VU plane */
	IPU6_FW_ISYS_FRAME_FORMAT_YV12,	/* 12 bit YUV 420, Y, V, U plane */
	IPU6_FW_ISYS_FRAME_FORMAT_YV16,	/* 16 bit YUV 422, Y, V, U plane */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV420, /* 12 bit YUV 420, Y, U, V plane */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV420_10, /* yuv420, 10 bits per subpixel */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV420_12, /* yuv420, 12 bits per subpixel */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV420_14, /* yuv420, 14 bits per subpixel */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV420_16, /* yuv420, 16 bits per subpixel */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV422, /* 16 bit YUV 422, Y, U, V plane */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV422_16, /* yuv422, 16 bits per subpixel */
	IPU6_FW_ISYS_FRAME_FORMAT_UYVY,	/* 16 bit YUV 422, UYVY interleaved */
	IPU6_FW_ISYS_FRAME_FORMAT_YUYV,	/* 16 bit YUV 422, YUYV interleaved */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV444, /* 24 bit YUV 444, Y, U, V plane */
	/* Internal format, 2 y lines followed by a uvinterleaved line */
	IPU6_FW_ISYS_FRAME_FORMAT_YUV_LINE,
	IPU6_FW_ISYS_FRAME_FORMAT_RAW8,	/* RAW8, 1 plane */
	IPU6_FW_ISYS_FRAME_FORMAT_RAW10, /* RAW10, 1 plane */
	IPU6_FW_ISYS_FRAME_FORMAT_RAW12, /* RAW12, 1 plane */
	IPU6_FW_ISYS_FRAME_FORMAT_RAW14, /* RAW14, 1 plane */
	IPU6_FW_ISYS_FRAME_FORMAT_RAW16, /* RAW16, 1 plane */
	/**
	 * 16 bit RGB, 1 plane. Each 3 sub pixels are packed into one 16 bit
	 * value, 5 bits for R, 6 bits for G and 5 bits for B.
	 */
	IPU6_FW_ISYS_FRAME_FORMAT_RGB565,
	IPU6_FW_ISYS_FRAME_FORMAT_PLANAR_RGB888, /* 24 bit RGB, 3 planes */
	IPU6_FW_ISYS_FRAME_FORMAT_RGBA888, /* 32 bit RGBA, 1 plane, A=Alpha */
	IPU6_FW_ISYS_FRAME_FORMAT_QPLANE6, /* Internal, for advanced ISP */
	IPU6_FW_ISYS_FRAME_FORMAT_BINARY_8, /* byte stream, used for jpeg. */
	N_IPU6_FW_ISYS_FRAME_FORMAT
};

#define IPU6_FW_ISYS_FRAME_FORMAT_RAW	(IPU6_FW_ISYS_FRAME_FORMAT_RAW16)

enum ipu6_fw_isys_pin_type {
	/* captured as MIPI packets */
	IPU6_FW_ISYS_PIN_TYPE_MIPI = 0,
	/* captured through the SoC path */
	IPU6_FW_ISYS_PIN_TYPE_RAW_SOC = 3,
};

/**
 * enum ipu6_fw_isys_mipi_store_mode. Describes if long MIPI packets reach
 * MIPI SRAM with the long packet header or
 * if not, then only option is to capture it with pin type MIPI.
 */
enum ipu6_fw_isys_mipi_store_mode {
	IPU6_FW_ISYS_MIPI_STORE_MODE_NORMAL = 0,
	IPU6_FW_ISYS_MIPI_STORE_MODE_DISCARD_LONG_HEADER,
	N_IPU6_FW_ISYS_MIPI_STORE_MODE
};

enum ipu6_fw_isys_capture_mode {
	IPU6_FW_ISYS_CAPTURE_MODE_REGULAR = 0,
	IPU6_FW_ISYS_CAPTURE_MODE_BURST,
	N_IPU6_FW_ISYS_CAPTURE_MODE,
};

enum ipu6_fw_isys_sensor_mode {
	IPU6_FW_ISYS_SENSOR_MODE_NORMAL = 0,
	IPU6_FW_ISYS_SENSOR_MODE_TOBII,
	N_IPU6_FW_ISYS_SENSOR_MODE,
};

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

enum ipu6_fw_isys_error {
	IPU6_FW_ISYS_ERROR_NONE = 0,
	IPU6_FW_ISYS_ERROR_FW_INTERNAL_CONSISTENCY,
	IPU6_FW_ISYS_ERROR_HW_CONSISTENCY,
	IPU6_FW_ISYS_ERROR_DRIVER_INVALID_COMMAND_SEQUENCE,
	IPU6_FW_ISYS_ERROR_DRIVER_INVALID_DEVICE_CONFIGURATION,
	IPU6_FW_ISYS_ERROR_DRIVER_INVALID_STREAM_CONFIGURATION,
	IPU6_FW_ISYS_ERROR_DRIVER_INVALID_FRAME_CONFIGURATION,
	IPU6_FW_ISYS_ERROR_INSUFFICIENT_RESOURCES,
	IPU6_FW_ISYS_ERROR_HW_REPORTED_STR2MMIO,
	IPU6_FW_ISYS_ERROR_HW_REPORTED_SIG2CIO,
	IPU6_FW_ISYS_ERROR_SENSOR_FW_SYNC,
	IPU6_FW_ISYS_ERROR_STREAM_IN_SUSPENSION,
	IPU6_FW_ISYS_ERROR_RESPONSE_QUEUE_FULL,
	N_IPU6_FW_ISYS_ERROR
};

enum ipu6_fw_proxy_error {
	IPU6_FW_PROXY_ERROR_NONE = 0,
	IPU6_FW_PROXY_ERROR_INVALID_WRITE_REGION,
	IPU6_FW_PROXY_ERROR_INVALID_WRITE_OFFSET,
	N_IPU6_FW_PROXY_ERROR
};

/* firmware ABI structure below are aligned in firmware, no need pack */
struct ipu6_fw_isys_buffer_partition_abi {
	u32 num_gda_pages[IPU4_STREAM_ID_MAX];
};

struct ipu6_fw_isys_fw_config {
	struct ipu6_fw_isys_buffer_partition_abi buffer_partition;
	u32 num_send_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
	u32 num_recv_queues[N_IPU6_FW_ISYS_QUEUE_TYPE];
};

/**
 * struct ipu6_fw_isys_resolution_abi: Generic resolution structure.
 */
struct ipu6_fw_isys_resolution_abi { // Same for IPU4
	u32 width;
	u32 height;
};

/**
 * struct ipu6_fw_isys_output_pin_payload_abi
 * @out_buf_id: Points to output pin buffer - buffer identifier
 * @addr: Points to output pin buffer - CSS Virtual Address
 * @compress: Request frame compression (1), or  not (0)
 */
struct ipu6_fw_isys_output_pin_payload_abi { // AKA ia_css_isys_output_pin_payload_comm
	u64 out_buf_id;
	u32 addr;
	u32 compress;
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
struct ipu4_fw_isys_output_pin_info_abi { // AKA ia_css_isys_output_pin_info_comm in old driver
	/* 0x00 */ struct ipu6_fw_isys_resolution_abi output_res;
	/* 0x08 */ u32 stride;
	/* 0x0b */ u32 watermark_in_lines;
	/* 0x10 */ u32 payload_buf_size;
	/* 0x14 */ u8 send_irq;
	/* 0x15 */ u8 input_pin_id;
	/* 0x16 */ u8 pt;
	/* 0x17 */ u8 ft;
	/* 0x18 */ u8 link_id;
	/* 0x19 */ u8 reserve_compression;
	/* 0x1a..0x1b?  auto-padding*/
};

/**
 * struct ipu4_fw_isys_input_pin_info_abi
 * @input_res: input resolution
 * @dt: mipi data type ((enum ipu_fw_isys_mipi_data_type)
 * @mipi_store_mode: defines if legacy long packet header will be stored or
 *		     discarded if discarded, output pin pin type for this
 *		     input pin can only be MIPI
 *		     (enum ipu_fw_isys_mipi_store_mode)
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
 * struct ipu6_fw_isys_cropping_abi - cropping coordinates
 */
struct ipu6_fw_isys_cropping_abi {
	s32 top_offset;
	s32 left_offset;
	s32 bottom_offset;
	s32 right_offset;
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
struct ipu4_fw_isys_stream_cfg_data_abi { // AKA ia_css_isys_stream_cfg_data_comm in old
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
 * struct ipu6_fw_isys_frame_buff_set - frame buffer set
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
 * struct ipu6_fw_isys_error_info_abi
 * @error: error code if something went wrong
 * @error_details: depending on error code, it may contain additional error info
 */
struct ipu6_fw_isys_error_info_abi { // AKA ia_css_isys_error_info_comm
	u32 error;
	u32 error_details;
};

/**
 * struct ipu6_fw_isys_resp_info_comm
 * @pin: this var is only valid for pin event related responses,
 *     contains pin addresses
 * @error_info: error information from the FW
 * @timestamp: Time information for event if available
 * @stream_handle: stream id the response corresponds to
 * @type: response type (enum ipu6_fw_isys_resp_type)
 * @pin_id: pin id that the pin payload corresponds to
 */
struct ipu6_fw_isys_resp_info_abi { // AKA ia_css_isys_resp_info_comm
	u64 buf_id;
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

/**
 * struct ipu6_fw_isys_proxy_error_info_comm
 * @proxy_error: error code if something went wrong
 * @proxy_error_details: depending on error code, it may contain additional
 *			error info
 */
struct ipu6_fw_isys_proxy_error_info_abi {
	u32 error;
	u32 error_details;
};

struct ipu6_fw_isys_proxy_resp_info_abi {
	u32 request_id;
	struct ipu6_fw_isys_proxy_error_info_abi error_info;
};

/**
 * struct ipu6_fw_proxy_write_queue_token
 * @request_id: update id for the specific proxy write request
 * @region_index: Region id for the proxy write request
 * @offset: Offset of the write request according to the base address
 *	    of the region
 * @value: Value that is requested to be written with the proxy write request
 */
struct ipu6_fw_proxy_write_queue_token {
	u32 request_id;
	u32 region_index;
	u32 offset;
	u32 value;
};

/**
 * struct ipu6_fw_resp_queue_token
 */
struct ipu6_fw_resp_queue_token {
	struct ipu6_fw_isys_resp_info_abi resp_info;
};

/**
 * struct ipu6_fw_send_queue_token
 */
struct ipu6_fw_send_queue_token {
	u64 buf_handle;
	u32 payload;
	u16 send_type;
	u16 stream_id;
};

/**
 * struct ipu6_fw_proxy_resp_queue_token
 */
struct ipu6_fw_proxy_resp_queue_token {
	struct ipu6_fw_isys_proxy_resp_info_abi proxy_resp_info;
};

/**
 * struct ipu6_fw_proxy_send_queue_token
 */
struct ipu6_fw_proxy_send_queue_token {
	u32 request_id;
	u32 region_index;
	u32 offset;
	u32 value;
};

void ipu6_fw_isys_dump_stream_cfg(struct device *dev,
				  struct ipu4_fw_isys_stream_cfg_data_abi
				  *stream_cfg);
void
ipu6_fw_isys_dump_frame_buff_set(struct device *dev,
				 struct ipu4_fw_isys_frame_buff_set_abi *buf,
				 unsigned int outputs);
int ipu6_fw_isys_init(struct ipu6_isys *isys, unsigned int num_streams);
int ipu6_fw_isys_close(struct ipu6_isys *isys);
int ipu6_fw_isys_simple_cmd(struct ipu6_isys *isys,
			    const unsigned int stream_handle, u16 send_type);
int ipu6_fw_isys_complex_cmd(struct ipu6_isys *isys,
			     const unsigned int stream_handle,
			     void *cpu_mapped_buf, dma_addr_t dma_mapped_buf,
			     size_t size, u16 send_type);
int ipu6_fw_isys_send_proxy_token(struct ipu6_isys *isys,
				  unsigned int req_id,
				  unsigned int index,
				  unsigned int offset, u32 value);
void ipu6_fw_isys_cleanup(struct ipu6_isys *isys);
struct ipu6_fw_isys_resp_info_abi *
ipu6_fw_isys_get_resp(void *context, unsigned int queue);
void ipu6_fw_isys_put_resp(void *context, unsigned int queue);
#endif
