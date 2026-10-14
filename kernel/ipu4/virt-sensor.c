// SPDX-License-Identifier: GPL-2.0-only
/*
 * Synthetic CSI2 sensor subdev for the IPU4 QEMU harness.
 *
 * Under CONFIG_VIDEO_IPU4_VIRT_SENSOR, `ipu4_virt_sensor_install()`
 * replaces the ambu_ipu_bridge_init() call in the IPU4 probe path. It
 * does exactly two things that matter to ipu6-isys:
 *
 *   1. Builds a software_node graph on `pdev->dev.fwnode->secondary`
 *      shaped like the ambu bridge's: an IPU HID node with port@0 /
 *      endpoint@0, linked to a sensor node with its own port@0 /
 *      endpoint@0. This is what `fwnode_graph_get_endpoint_by_id()`
 *      in isys_notifier_init() (ipu6-isys.c:824) walks.
 *
 *   2. Registers a v4l2_subdev with a SOURCE pad advertising 800x800
 *      RGB888_1X24 (or a Bayer, greyscale or YUV format once set), with
 *      its fwnode set to the sensor HID node's fwnode so the async
 *      matcher can bind it.
 *
 * `.s_stream()` is a no-op — frame generation is M5's QEMU work. What
 * this file delivers is: probe completes, `/dev/video*` nodes appear.
 */

#include <linux/module.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <media/media-entity.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "ipu4-compat.h"
#include "virt-sensor.h"

#define VIRT_SENSOR_WIDTH		800
#define VIRT_SENSOR_HEIGHT		800
#define VIRT_SENSOR_LANES		4
#define VIRT_SENSOR_LINK_FREQ		450000000ULL

/* Match ambu-bridge's IPU HID; ipu6-isys does not care about the string,
 * but using the same ACPI HID keeps the register/cleanup symmetric with
 * the downstream bridge code the harness is replacing. */
#define IPU_HID				"INT343E"
#define SENSOR_NAME			"ipu4-virt-sensor"

struct virt_sensor {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	const struct virt_sensor_fmt *fmt;	/* active format */

	/* Nodes are in the order expected by software_node_register_node_group. */
	struct software_node ipu_node;
	struct software_node ipu_port;
	struct software_node ipu_endpoint;
	struct software_node sensor_node;
	struct software_node sensor_port;
	struct software_node sensor_endpoint;
	const struct software_node *group[7];		/* 6 nodes + NULL */

	u32 data_lanes[VIRT_SENSOR_LANES];
	u64 link_freqs[1];

	struct property_entry ipu_ep_props[3];		/* data-lanes, remote-ep, NULL */
	struct property_entry sensor_ep_props[5];	/* bus-type, data-lanes, remote-ep, link-freqs, NULL */

	/* PROPERTY_ENTRY_REF_ARRAY uses ARRAY_SIZE internally, so these
	 * must be actual arrays (size 1) rather than plain structs. */
	struct software_node_ref_args ipu_remote_ref[1];
	struct software_node_ref_args sensor_remote_ref[1];

	bool fwnode_secondary_set;
};

/* Only one virt-sensor per system; matches the one-IPU-device reality. */
static struct virt_sensor *g_virt_sensor;

/* The formats the sensor offers, the default first, with their CSI-2
 * data types.
 */
static const struct virt_sensor_fmt {
	u32 code;
	u8 dt;
} virt_sensor_fmts[] = {
	{ MEDIA_BUS_FMT_RGB888_1X24, MIPI_CSI2_DT_RGB888 },
	{ MEDIA_BUS_FMT_RGB565_1X16, MIPI_CSI2_DT_RGB565 },
	{ MEDIA_BUS_FMT_UYVY8_1X16, MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, MIPI_CSI2_DT_RAW12 },
	{ MEDIA_BUS_FMT_Y10_1X10, MIPI_CSI2_DT_RAW10 },
};

static const struct virt_sensor_fmt *virt_sensor_find_fmt(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(virt_sensor_fmts); i++)
		if (virt_sensor_fmts[i].code == code)
			return &virt_sensor_fmts[i];
	return NULL;
}

static const struct v4l2_mbus_framefmt virt_sensor_default_fmt = {
	.width		= VIRT_SENSOR_WIDTH,
	.height		= VIRT_SENSOR_HEIGHT,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_SRGB,
};

static int virt_sensor_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(virt_sensor_fmts))
		return -EINVAL;
	code->code = virt_sensor_fmts[code->index].code;
	return 0;
}

static int virt_sensor_enum_frame_size(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index != 0 || !virt_sensor_find_fmt(fse->code))
		return -EINVAL;
	fse->min_width = fse->max_width = VIRT_SENSOR_WIDTH;
	fse->min_height = fse->max_height = VIRT_SENSOR_HEIGHT;
	return 0;
}

static int virt_sensor_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	struct virt_sensor *vs = container_of(sd, struct virt_sensor, sd);

	fmt->format = virt_sensor_default_fmt;
	fmt->format.code = vs->fmt->code;
	return 0;
}

/* One fixed size; an unknown code gets the default format. */
static int virt_sensor_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	struct virt_sensor *vs = container_of(sd, struct virt_sensor, sd);
	const struct virt_sensor_fmt *f = virt_sensor_find_fmt(fmt->format.code);

	if (!f)
		f = &virt_sensor_fmts[0];
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		vs->fmt = f;
	fmt->format = virt_sensor_default_fmt;
	fmt->format.code = f->code;
	return 0;
}

static int virt_sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	dev_info(sd->dev, "virt-sensor s_stream %s (no-op, M5 delivers frames)\n",
		 enable ? "on" : "off");
	return 0;
}

static bool frame_desc;
module_param_named(virt_sensor_frame_desc, frame_desc, bool, 0644);
MODULE_PARM_DESC(virt_sensor_frame_desc, "Report a CSI-2 frame descriptor");

/*
 * With frame_desc, describe the one CSI-2 stream; without, the IPU
 * derives VC and data type from the format.
 */
static int virt_sensor_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				      struct v4l2_mbus_frame_desc *fd)
{
	struct virt_sensor *vs = container_of(sd, struct virt_sensor, sd);

	if (!frame_desc)
		return -ENOIOCTLCMD;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;
	fd->entry[0].stream = 0;
	fd->entry[0].pixelcode = vs->fmt->code;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = vs->fmt->dt;
	return 0;
}

static const struct v4l2_subdev_pad_ops virt_sensor_pad_ops = {
	.enum_mbus_code		= virt_sensor_enum_mbus_code,
	.enum_frame_size	= virt_sensor_enum_frame_size,
	.get_fmt		= virt_sensor_get_fmt,
	.set_fmt		= virt_sensor_set_fmt,
	.get_frame_desc		= virt_sensor_get_frame_desc,
};

static const struct v4l2_subdev_video_ops virt_sensor_video_ops = {
	.s_stream = virt_sensor_s_stream,
};

static const struct v4l2_subdev_ops virt_sensor_subdev_ops = {
	.pad	= &virt_sensor_pad_ops,
	.video	= &virt_sensor_video_ops,
};

static void virt_sensor_build_swnodes(struct virt_sensor *vs)
{
	unsigned int i;

	for (i = 0; i < VIRT_SENSOR_LANES; i++)
		vs->data_lanes[i] = i + 1;
	vs->link_freqs[0] = VIRT_SENSOR_LINK_FREQ;

	vs->ipu_node.name		= IPU_HID;
	vs->ipu_port.name		= "port@0";
	vs->ipu_port.parent		= &vs->ipu_node;
	vs->ipu_endpoint.name		= "endpoint@0";
	vs->ipu_endpoint.parent		= &vs->ipu_port;

	vs->sensor_node.name		= SENSOR_NAME;
	vs->sensor_port.name		= "port@0";
	vs->sensor_port.parent		= &vs->sensor_node;
	vs->sensor_endpoint.name	= "endpoint@0";
	vs->sensor_endpoint.parent	= &vs->sensor_port;

	vs->ipu_remote_ref[0] = SOFTWARE_NODE_REFERENCE(&vs->sensor_endpoint);
	vs->sensor_remote_ref[0] = SOFTWARE_NODE_REFERENCE(&vs->ipu_endpoint);

	vs->ipu_ep_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
							   vs->data_lanes,
							   VIRT_SENSOR_LANES);
	vs->ipu_ep_props[1] = PROPERTY_ENTRY_REF_ARRAY("remote-endpoint",
						       vs->ipu_remote_ref);
	/* ipu_ep_props[2] is the zero-init terminator. */

	vs->sensor_ep_props[0] = PROPERTY_ENTRY_U32("bus-type",
						    V4L2_FWNODE_BUS_TYPE_CSI2_DPHY);
	vs->sensor_ep_props[1] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
							      vs->data_lanes,
							      VIRT_SENSOR_LANES);
	vs->sensor_ep_props[2] = PROPERTY_ENTRY_REF_ARRAY("remote-endpoint",
							  vs->sensor_remote_ref);
	vs->sensor_ep_props[3] = PROPERTY_ENTRY_U64_ARRAY_LEN("link-frequencies",
							      vs->link_freqs, 1);
	/* sensor_ep_props[4] is the terminator. */

	vs->ipu_endpoint.properties	= vs->ipu_ep_props;
	vs->sensor_endpoint.properties	= vs->sensor_ep_props;

	vs->group[0] = &vs->ipu_node;
	vs->group[1] = &vs->ipu_port;
	vs->group[2] = &vs->ipu_endpoint;
	vs->group[3] = &vs->sensor_node;
	vs->group[4] = &vs->sensor_port;
	vs->group[5] = &vs->sensor_endpoint;
	vs->group[6] = NULL;
}

int ipu4_virt_sensor_install(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct virt_sensor *vs;
	struct fwnode_handle *ipu_fwnode;
	int ret;

	if (g_virt_sensor)
		return -EEXIST;
	if (!dev->fwnode) {
		dev_err(dev, "virt-sensor: IPU device has no primary fwnode\n");
		return -ENODEV;
	}
	if (dev->fwnode->secondary && !IS_ERR(dev->fwnode->secondary)) {
		dev_err(dev, "virt-sensor: fwnode->secondary already set\n");
		return -EBUSY;
	}

	vs = kzalloc(sizeof(*vs), GFP_KERNEL);
	if (!vs)
		return -ENOMEM;
	vs->fmt = &virt_sensor_fmts[0];

	virt_sensor_build_swnodes(vs);

	ret = software_node_register_node_group(vs->group);
	if (ret) {
		dev_err(dev, "virt-sensor: sw-node register failed %d\n", ret);
		goto err_free;
	}

	ipu_fwnode = software_node_fwnode(&vs->ipu_node);
	if (!ipu_fwnode) {
		dev_err(dev, "virt-sensor: no fwnode for IPU swnode\n");
		ret = -ENODEV;
		goto err_nodes;
	}

	/* Mirror ambu_ipu_bridge_init: there is no public inverse of
	 * set_secondary_fwnode(), so set it directly. */
	dev->fwnode->secondary = ipu_fwnode;
	vs->fwnode_secondary_set = true;

	v4l2_subdev_init(&vs->sd, &virt_sensor_subdev_ops);
	vs->sd.dev	= dev;
	vs->sd.fwnode	= software_node_fwnode(&vs->sensor_node);
	vs->sd.owner	= THIS_MODULE;
	vs->sd.flags	= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(vs->sd.name, SENSOR_NAME, sizeof(vs->sd.name));

	/*
	 * Expose V4L2_CID_LINK_FREQ so ipu6_isys_csi2_get_link_freq()
	 * (kernel/ipu4/ipu6-isys-csi2.c:108) can read a sane value from
	 * `v4l2_get_link_freq(ext_sd->ctrl_handler, 0, 0)`. Without the
	 * ctrl, that helper returns -ENOENT, ipu6_isys_csi2_calc_timing()
	 * propagates the error back, and CSI2 enable_streams aborts
	 * before set_stream() ever writes the per-port RX_ENABLE /
	 * DLY_CNT_* / IRQ_* registers — exactly the silicon writes the
	 * Step-2 mmio-trace coverage report wants to drop from
	 * `unimplemented`.
	 */
	ret = v4l2_ctrl_handler_init(&vs->ctrl_handler, 1);
	if (ret) {
		dev_err(dev, "virt-sensor: ctrl handler init failed %d\n", ret);
		goto err_fwnode;
	}
	v4l2_ctrl_new_int_menu(&vs->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, vs->link_freqs);
	if (vs->ctrl_handler.error) {
		ret = vs->ctrl_handler.error;
		dev_err(dev, "virt-sensor: failed to add LINK_FREQ ctrl %d\n",
			ret);
		v4l2_ctrl_handler_free(&vs->ctrl_handler);
		goto err_fwnode;
	}
	vs->sd.ctrl_handler = &vs->ctrl_handler;

	vs->pad.flags		 = MEDIA_PAD_FL_SOURCE;
	vs->sd.entity.function	 = MEDIA_ENT_F_CAM_SENSOR;
	vs->sd.entity.name	 = vs->sd.name;

	ret = media_entity_pads_init(&vs->sd.entity, 1, &vs->pad);
	if (ret) {
		dev_err(dev, "virt-sensor: entity pads init failed %d\n", ret);
		goto err_fwnode;
	}

	ret = v4l2_async_register_subdev(&vs->sd);
	if (ret) {
		dev_err(dev, "virt-sensor: async register failed %d\n", ret);
		goto err_entity;
	}

	g_virt_sensor = vs;
	dev_info(dev, "virt-sensor installed: %ux%u RGB888, %d lanes\n",
		 VIRT_SENSOR_WIDTH, VIRT_SENSOR_HEIGHT, VIRT_SENSOR_LANES);
	return 0;

err_entity:
	media_entity_cleanup(&vs->sd.entity);
	v4l2_ctrl_handler_free(&vs->ctrl_handler);
err_fwnode:
	dev->fwnode->secondary = NULL;
	vs->fwnode_secondary_set = false;
err_nodes:
	software_node_unregister_node_group(vs->group);
err_free:
	kfree(vs);
	return ret;
}
EXPORT_SYMBOL_NS_GPL(ipu4_virt_sensor_install, INTEL_IPU6);

void ipu4_virt_sensor_remove(struct pci_dev *pdev)
{
	struct virt_sensor *vs = g_virt_sensor;

	if (!vs)
		return;
	v4l2_async_unregister_subdev(&vs->sd);
	media_entity_cleanup(&vs->sd.entity);
	v4l2_ctrl_handler_free(&vs->ctrl_handler);
	if (vs->fwnode_secondary_set)
		pdev->dev.fwnode->secondary = NULL;
	software_node_unregister_node_group(vs->group);
	kfree(vs);
	g_virt_sensor = NULL;
}
EXPORT_SYMBOL_NS_GPL(ipu4_virt_sensor_remove, INTEL_IPU6);
