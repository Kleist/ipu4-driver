// SPDX-License-Identifier: GPL-2.0-only
/*
 * Ambu Toshiba MIPI Bridge driver for TC358748
 * heavily inspired by tc358746.c from upstream kernel
 *
 * Copyright 2023 Andreas Helbech Kleist <akle@ambu.com>
 * Copyright 2022 Marco Felsch <kernel@pengutronix.de>
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include "ambu-gen2-tc358748-regs.h"

#define AMBU_TOSHIBA_REFCLK 19200000 // 19.2 MHz from aBox2/aView2 schematic

/* 16-bit registers */
#define CHIPID_REG			0x0000
#define		CHIPID			GENMASK(15, 8)

#define SYSCTL_REG			0x0002
#define		SRESET			BIT(0)


/* 32-bit registers */
#define CLW_DPHYCONTTX_REG		0x0100

#define CSI_START_REG			0x0518

static const struct v4l2_mbus_framefmt tc358748_def_fmt = {
	.width		= 800,
	.height		= 800,
	.code		= MEDIA_BUS_FMT_RGB888_1X24,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_DEFAULT,
	.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

struct tc358748 {
	struct v4l2_subdev		sd;
	struct media_pad		pad;
	struct v4l2_fwnode_endpoint	endpoint;
	struct v4l2_ctrl_handler	ctrl_hdl;

	struct regmap			*regmap;
	struct i2c_client		*client;
};

static bool tc358748_valid_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case CHIPID_REG ... CSI_START_REG:
		return true;
	default:
		return false;
	}
}

static inline struct tc358748 *to_tc358748(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358748, sd);
}

static const struct regmap_config tc358748_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = CSI_START_REG,
	.writeable_reg = tc358748_valid_reg,
	.readable_reg = tc358748_valid_reg,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
};

static int tc358748_write(struct tc358748 *tc358748, u32 reg, u32 val)
{
	size_t count;
	int err;
	u16 val_array[2] = {0,0};

	/* 32-bit registers starting from CLW_DPHYCONTTX */
	if (reg >= CLW_DPHYCONTTX_REG) {
		count = 2;
		val_array[0] = val >> 16;
		val_array[1] = val & 0xFFFF;
	} else {
		count = 1;
		val_array[0] = val & 0xFFFF;
	}

	err = regmap_bulk_write(tc358748->regmap, reg, &val_array[0], count);
	if (err)
		dev_err(&tc358748->client->dev,
			"Failed to write reg:0x%04x err:%d\n", reg, err);

	return err;
}

static int tc358748_write_crl_reg(struct tc358748 *tc358748,
	const struct crl_register_write_rep *reg)
{
	if (reg->len == CRL_REG_LEN_DELAY) {
		msleep(reg->val);
		return 0;
	} else if (reg->len == CRL_REG_LEN_16BIT
	     || reg->len == CRL_REG_LEN_32BIT) {
		return tc358748_write(tc358748, reg->address, reg->val);
	}
	dev_err(&tc358748->client->dev, "Unhandled write op %d\n", reg->len);
		return -EINVAL;
}

static int tc358748_write_crl_regs(struct tc358748 *tc358748,
	const struct crl_register_write_rep *regs,
	u32 len)
{
	u32 i;
	int ret = 0;

	for (i = 0; i < len; ++i) {
		ret = tc358748_write_crl_reg(tc358748, &regs[i]);
		if (ret)
			break;
	}
	return ret;
}

static int tc358748_read(struct tc358748 *tc358748, u32 reg, u32 *val)
{
	size_t count;
	int err;

	/* 32-bit registers starting from CLW_DPHYCONTTX */
	count = reg < CLW_DPHYCONTTX_REG ? 1 : 2;
	*val = 0;

	err = regmap_bulk_read(tc358748->regmap, reg, val, count);
	if (err)
		dev_err(&tc358748->client->dev,
			"Failed to read reg:0x%04x err:%d\n", reg, err);

	return err;
}

static int
tc358748_update_bits(struct tc358748 *tc358748, u32 reg, u32 mask, u32 val)
{
	u32 tmp, orig;
	int err;

	err = tc358748_read(tc358748, reg, &orig);
	if (err)
		return err;

	tmp = orig & ~mask;
	tmp |= val & mask;

	return tc358748_write(tc358748, reg, tmp);
}

static int tc358748_set_bits(struct tc358748 *tc358748, u32 reg, u32 bits)
{
	return tc358748_update_bits(tc358748, reg, bits, bits);
}

static int tc358748_clear_bits(struct tc358748 *tc358748, u32 reg, u32 bits)
{
	return tc358748_update_bits(tc358748, reg, bits, 0);
}

static int tc358748_sw_reset(struct tc358748 *tc358748)
{
	int err;

	err = tc358748_set_bits(tc358748, SYSCTL_REG, SRESET);
	if (err)
		return err;

	fsleep(10);

	return tc358748_clear_bits(tc358748, SYSCTL_REG, SRESET);
}

static int tc358748_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358748 *tc358748 = to_tc358748(sd);
	int ret = 0;

	if (enable) {
		// TODO - detect if it is 400 or 800
		(void)ieib475_source_RGB_400_400;

		ret = tc358748_write_crl_regs(tc358748,
			ieib475_source_RGB_800_800,
			ARRAY_SIZE(ieib475_source_RGB_800_800));
	} else {
		ret = tc358748_write_crl_regs(tc358748,
			ieib475_streamoff_regs,
			ARRAY_SIZE(ieib475_streamoff_regs));
	}
	return ret;
}

static int tc358748_init_cfg(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = v4l2_subdev_get_pad_format(sd, state, 0);
	*fmt = tc358748_def_fmt;

	return 0;
}

static int tc358748_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	WARN(true, "Not implemented");
	return -EINVAL;
}

static int tc358748_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *src_fmt;

	src_fmt = v4l2_subdev_get_pad_format(sd, sd_state, 0);

	// From 4.19 crl_ieib475_configuration.h: struct crl_csi_data_fmt ieib475_crl_csi_data_fmt
	src_fmt->code = MEDIA_BUS_FMT_RGB888_1X24;
	return 0;
}

static int tc358748_init_hw(struct tc358748 *tc358748)
{
	struct device *dev = &tc358748->client->dev;
	unsigned int chipid;
	u32 val;
	int err;

	 /* Ensure that CSI interface is put into LP-11 state */
	err = tc358748_sw_reset(tc358748);
	if (err) {
		dev_err(dev, "Failed to reset the device\n");
		return err;
	}

	err = tc358748_read(tc358748, CHIPID_REG, &val);
	if (err)
		return -ENODEV;

	chipid = FIELD_GET(CHIPID, val);
	if (chipid != 0x44) {
		dev_err(dev, "Invalid chipid 0x%02x\n", chipid);
		return -ENODEV;
	}

	return 0;
}

static int
tc358748_link_validate(struct v4l2_subdev *sd, struct media_link *link,
		       struct v4l2_subdev_format *source_fmt,
		       struct v4l2_subdev_format *sink_fmt)
{
	WARN(true, "Not implemented");
	return -EINVAL;
}

static int tc358748_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				    struct v4l2_mbus_config *config)
{
	WARN(true, "Not implemented");
	return -EINVAL;
}

static const struct v4l2_subdev_core_ops tc358748_core_ops = {
};

static const struct v4l2_subdev_video_ops tc358748_video_ops = {
	.s_stream = tc358748_s_stream,
};

static const struct v4l2_subdev_pad_ops tc358748_pad_ops = {
	.init_cfg = tc358748_init_cfg,
	.enum_mbus_code = tc358748_enum_mbus_code,
	.set_fmt = tc358748_set_fmt,
	.get_fmt = v4l2_subdev_get_fmt,
	.link_validate = tc358748_link_validate,
	.get_mbus_config = tc358748_get_mbus_config,
};

static const struct v4l2_subdev_ops tc358748_ops = {
	.core = &tc358748_core_ops,
	.video = &tc358748_video_ops,
	.pad = &tc358748_pad_ops,
};

static const struct media_entity_operations tc358748_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_validate = v4l2_subdev_link_validate,
};

static int tc358748_subdev_init(struct tc358748 *tc358748, struct i2c_client *client)
{
	struct v4l2_subdev *sd = &tc358748->sd;
	int err;

	v4l2_i2c_subdev_init(sd, tc358748->client, &tc358748_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &tc358748_entity_ops;

	tc358748->pad.flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_pads_init(&sd->entity, 1, &tc358748->pad);
	if (err)
		return err;

	err = v4l2_subdev_init_finalize(sd);
	if (err)
		media_entity_cleanup(&sd->entity);

	return err;
}

static void tc358748_subdev_cleanup(struct tc358748 *tc358748)
{
	media_entity_cleanup(&tc358748->sd.entity);
	v4l2_subdev_cleanup(&tc358748->sd);
}

static int
tc358748_output_port_init(struct tc358748 *tc358748, unsigned long refclk)
{
	struct device *dev = tc358748->sd.dev;
	struct v4l2_fwnode_endpoint *vep;
	struct fwnode_handle *ep;
	unsigned char csi_lanes;
	int err;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0, 0);
	if (!ep) {
		dev_err(dev, "Missing endpoint node\n");
		return -EINVAL;
	}

	/* Currently we only support 'parallel in' -> 'csi out' */
	vep = &tc358748->endpoint;
	vep->bus_type = V4L2_MBUS_CSI2_DPHY;
	err = v4l2_fwnode_endpoint_alloc_parse(ep, vep);
	fwnode_handle_put(ep);
	if (err) {
		dev_err(dev, "Failed to parse endpoint: %d\n", err);
		return err;
	}

	csi_lanes = vep->bus.mipi_csi2.num_data_lanes;
	if (csi_lanes == 0 || csi_lanes > 4 ||
	    vep->nr_of_link_frequencies == 0) {
		dev_err(dev, "error: Invalid CSI-2 settings: lanes: %d freqs: %d\n",
			csi_lanes, vep->nr_of_link_frequencies);
		err = -EINVAL;
		goto err;
	}
	return 0;

err:
	v4l2_fwnode_endpoint_free(vep);
	return err;
}

static void tc358748_output_port_cleanup(struct tc358748 *tc358748)
{
	v4l2_fwnode_endpoint_free(&tc358748->endpoint);
}

static int tc358748_ctrl_init(struct tc358748 *tc358748)
{
	u64 *link_frequencies = tc358748->endpoint.link_frequencies;
	struct v4l2_ctrl *ctrl;
	int err;

	err = v4l2_ctrl_handler_init(&tc358748->ctrl_hdl, 1);
	if (err)
		return err;

	/*
	 * The driver currently supports only one link-frequency, regardless of
	 * the input from the firmware, see: tc358748_init_output_port(). So
	 * report only the first frequency from the array of possible given
	 * frequencies.
	 */
	ctrl = v4l2_ctrl_new_int_menu(&tc358748->ctrl_hdl, NULL,
				      V4L2_CID_LINK_FREQ, 0, 0,
				      link_frequencies);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	err = tc358748->ctrl_hdl.error;
	if (err) {
		v4l2_ctrl_handler_free(&tc358748->ctrl_hdl);
		return err;
	}

	tc358748->sd.ctrl_handler = &tc358748->ctrl_hdl;

	return 0;
}

static void tc358748_ctrl_cleanup(struct tc358748 *tc358748)
{
	v4l2_ctrl_handler_free(&tc358748->ctrl_hdl);
}

static int tc358748_subdev_register(struct v4l2_subdev *sd)
{
	int err;

	err = v4l2_async_register_subdev(sd);
	if (err)
		dev_err(sd->dev, "v4l2_async_register_subdev error: %d\n", err);
	return err;
}

static void tc358748_subdev_unregister(struct v4l2_subdev *sd)
{
	v4l2_async_unregister_subdev(sd);
	fwnode_handle_put(sd->fwnode);
}

static int tc358748_probe(struct i2c_client *client)
{
	struct tc358748 *tc358748;
	int err;

	tc358748 = devm_kzalloc(&client->dev, sizeof(*tc358748), GFP_KERNEL);
	if (!tc358748)
		return -ENOMEM;
	tc358748->client = client;

	tc358748->regmap = devm_regmap_init_i2c(client, &tc358748_regmap_config);
	if (IS_ERR(tc358748->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(tc358748->regmap),
				     "Failed to init regmap\n");

	err = tc358748_subdev_init(tc358748, client);
	if (err)
		return dev_err_probe(&client->dev, err, "Failed to init subdev\n");

	err = tc358748_output_port_init(tc358748, AMBU_TOSHIBA_REFCLK);
	if (err) {
		dev_err(&client->dev, "tc358748_output_port_init failed: %d", err);
		goto err_subdev_cleanup;
	}

	err = tc358748_ctrl_init(tc358748);
	if (err) {
		dev_err(&client->dev, "tc358748_ctrl_init failed: %d", err);
		goto err_output_port_cleanup;
	}

	dev_set_drvdata(&client->dev, tc358748);

	err = tc358748_init_hw(tc358748);
	if (err) {
		dev_err(&client->dev, "tc358748_init_hw failed: %d", err);
		goto err_ctrl_cleanup;
	}

	err = tc358748_subdev_register(&tc358748->sd);
	if (err) {
		dev_err(&client->dev, "tc358748_subdev_register failed: %d", err);
		goto err_ctrl_cleanup;
	}
	return 0;

err_ctrl_cleanup:
	tc358748_ctrl_cleanup(tc358748);
err_output_port_cleanup:
	tc358748_output_port_cleanup(tc358748);
err_subdev_cleanup:
	tc358748_subdev_cleanup(tc358748);
	return err;
}

static void tc358748_remove(struct i2c_client *client)
{
	struct tc358748 *tc358748 = dev_get_drvdata(&client->dev);

	tc358748_subdev_unregister(&tc358748->sd);
	tc358748_ctrl_cleanup(tc358748);
	tc358748_output_port_cleanup(tc358748);
	tc358748_subdev_cleanup(tc358748);
}

static const struct i2c_device_id tc358748_id[] = {
	{"ambu,tc358748", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358748_id);

static struct i2c_driver tc358748_driver = {
	.driver = {
		.name = "tc358748",
	},
	.probe = tc358748_probe,
	.remove = tc358748_remove,
	.id_table = tc358748_id,
};

module_i2c_driver(tc358748_driver);

MODULE_DESCRIPTION("Ambu Toshiba Bridge driver");
MODULE_AUTHOR("Andreas Helbech Kleist <akle@ambu.com>");
MODULE_LICENSE("GPL");
