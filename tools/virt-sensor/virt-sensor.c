// SPDX-License-Identifier: GPL-2.0-only
/*
 * Virtual CSI-2 sensor for the IPU4 QEMU test harness.
 *
 * The QEMU IPU4 model has no ACPI sensor description, so on load this
 * module does what ipu_bridge_init() does for a real laptop, without any
 * hook in the IPU driver:
 *
 *  - It builds a software-node graph shaped like the one ipu-bridge
 *    creates (IPU node with port@0/endpoint@0, sensor node with
 *    port@0/endpoint@0, data-lanes, link-frequencies, remote-endpoint
 *    references) and makes the IPU node the secondary fwnode of the IPU
 *    PCI device. isys_notifier_init() walks that graph, and
 *    ipu_bridge_init() returns early because a graph already exists.
 *
 *  - It registers a single-pad v4l2 subdev (800x800 RGB888_1X24) with
 *    the sensor node as its fwnode, so the ISYS async notifier binds it.
 *
 * Load it before intel-ipu6: the graph has to be in place when ISYS
 * probes. Streaming is a no-op; frames come from the QEMU model.
 *
 * This is test-harness code. It is not part of the kernel patch series.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define VIRT_SENSOR_NAME	"ipu4-virt-sensor"
#define VIRT_SENSOR_WIDTH	800
#define VIRT_SENSOR_HEIGHT	800
#define VIRT_SENSOR_CODE	MEDIA_BUS_FMT_RGB888_1X24
#define VIRT_SENSOR_LANES	4
#define VIRT_SENSOR_LINK_FREQ	450000000ULL

/* Same node name ipu-bridge gives the IPU node (its ACPI HID). */
#define VIRT_SENSOR_IPU_NODE	"INT343E"

static ushort vendor = PCI_VENDOR_ID_INTEL;
module_param(vendor, ushort, 0444);
MODULE_PARM_DESC(vendor, "PCI vendor ID of the IPU (default 0x8086)");

static ushort device = 0x5a88;
module_param(device, ushort, 0444);
MODULE_PARM_DESC(device, "PCI device ID of the IPU (default 0x5a88, IPU4)");

enum virt_sensor_swnodes {
	SWNODE_IPU,
	SWNODE_IPU_PORT,
	SWNODE_IPU_ENDPOINT,
	SWNODE_SENSOR,
	SWNODE_SENSOR_PORT,
	SWNODE_SENSOR_ENDPOINT,
	SWNODE_COUNT
};

struct virt_sensor {
	struct pci_dev *ipu;
	struct platform_device *pdev;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;

	struct software_node nodes[SWNODE_COUNT];
	const struct software_node *group[SWNODE_COUNT + 1];
	struct software_node_ref_args ipu_remote[1];
	struct software_node_ref_args sensor_remote[1];
	struct property_entry ipu_ep_props[3];
	struct property_entry sensor_ep_props[5];
};

static const u32 virt_sensor_data_lanes[VIRT_SENSOR_LANES] = { 1, 2, 3, 4 };
static const u64 virt_sensor_link_freqs[] = { VIRT_SENSOR_LINK_FREQ };
static const s64 virt_sensor_link_freq_menu[] = { VIRT_SENSOR_LINK_FREQ };

static struct virt_sensor *virt_sensor;

static void virt_sensor_default_fmt(struct v4l2_mbus_framefmt *fmt)
{
	*fmt = (struct v4l2_mbus_framefmt) {
		.width = VIRT_SENSOR_WIDTH,
		.height = VIRT_SENSOR_HEIGHT,
		.code = VIRT_SENSOR_CODE,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
}

static int virt_sensor_init_state(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state)
{
	virt_sensor_default_fmt(v4l2_subdev_state_get_format(state, 0));
	return 0;
}

static int virt_sensor_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = VIRT_SENSOR_CODE;
	return 0;
}

static int virt_sensor_enum_frame_size(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != VIRT_SENSOR_CODE)
		return -EINVAL;

	fse->min_width = VIRT_SENSOR_WIDTH;
	fse->max_width = VIRT_SENSOR_WIDTH;
	fse->min_height = VIRT_SENSOR_HEIGHT;
	fse->max_height = VIRT_SENSOR_HEIGHT;
	return 0;
}

/* The sensor has a single fixed mode; any request gets it back. */
static int virt_sensor_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_format *fmt)
{
	virt_sensor_default_fmt(&fmt->format);
	*v4l2_subdev_state_get_format(state, fmt->pad) = fmt->format;
	return 0;
}

static int virt_sensor_enable_streams(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state, u32 pad,
				      u64 streams_mask)
{
	dev_dbg(sd->dev, "stream on\n");
	return 0;
}

static int virt_sensor_disable_streams(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state, u32 pad,
				       u64 streams_mask)
{
	dev_dbg(sd->dev, "stream off\n");
	return 0;
}

static const struct v4l2_subdev_pad_ops virt_sensor_pad_ops = {
	.enum_mbus_code = virt_sensor_enum_mbus_code,
	.enum_frame_size = virt_sensor_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = virt_sensor_set_fmt,
	.enable_streams = virt_sensor_enable_streams,
	.disable_streams = virt_sensor_disable_streams,
};

static const struct v4l2_subdev_video_ops virt_sensor_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops virt_sensor_subdev_ops = {
	.video = &virt_sensor_video_ops,
	.pad = &virt_sensor_pad_ops,
};

static const struct v4l2_subdev_internal_ops virt_sensor_internal_ops = {
	.init_state = virt_sensor_init_state,
};

static void virt_sensor_init_swnodes(struct virt_sensor *vs)
{
	struct software_node *n = vs->nodes;
	unsigned int i;

	vs->ipu_remote[0] = SOFTWARE_NODE_REFERENCE(&n[SWNODE_SENSOR_ENDPOINT]);
	vs->sensor_remote[0] = SOFTWARE_NODE_REFERENCE(&n[SWNODE_IPU_ENDPOINT]);

	vs->ipu_ep_props[0] =
		PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", virt_sensor_data_lanes,
					     VIRT_SENSOR_LANES);
	vs->ipu_ep_props[1] =
		PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", vs->ipu_remote);

	vs->sensor_ep_props[0] =
		PROPERTY_ENTRY_U32("bus-type", V4L2_FWNODE_BUS_TYPE_CSI2_DPHY);
	vs->sensor_ep_props[1] =
		PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", virt_sensor_data_lanes,
					     VIRT_SENSOR_LANES);
	vs->sensor_ep_props[2] =
		PROPERTY_ENTRY_REF_ARRAY("remote-endpoint", vs->sensor_remote);
	vs->sensor_ep_props[3] =
		PROPERTY_ENTRY_U64_ARRAY("link-frequencies",
					 virt_sensor_link_freqs);

	n[SWNODE_IPU] = SOFTWARE_NODE(VIRT_SENSOR_IPU_NODE, NULL, NULL);
	n[SWNODE_IPU_PORT] = SOFTWARE_NODE("port@0", NULL, &n[SWNODE_IPU]);
	n[SWNODE_IPU_ENDPOINT] = SOFTWARE_NODE("endpoint@0", vs->ipu_ep_props,
					       &n[SWNODE_IPU_PORT]);
	n[SWNODE_SENSOR] = SOFTWARE_NODE(VIRT_SENSOR_NAME, NULL, NULL);
	n[SWNODE_SENSOR_PORT] = SOFTWARE_NODE("port@0", NULL, &n[SWNODE_SENSOR]);
	n[SWNODE_SENSOR_ENDPOINT] = SOFTWARE_NODE("endpoint@0",
						  vs->sensor_ep_props,
						  &n[SWNODE_SENSOR_PORT]);

	for (i = 0; i < SWNODE_COUNT; i++)
		vs->group[i] = &n[i];
}

static int virt_sensor_init_subdev(struct virt_sensor *vs)
{
	struct v4l2_subdev *sd = &vs->sd;
	int ret;

	v4l2_subdev_init(sd, &virt_sensor_subdev_ops);
	sd->internal_ops = &virt_sensor_internal_ops;
	sd->owner = THIS_MODULE;
	sd->dev = &vs->pdev->dev;
	sd->fwnode = software_node_fwnode(&vs->nodes[SWNODE_SENSOR]);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(sd->name, VIRT_SENSOR_NAME, sizeof(sd->name));

	/* ipu6_isys_csi2_get_link_freq() reads V4L2_CID_LINK_FREQ. */
	v4l2_ctrl_handler_init(&vs->ctrls, 1);
	v4l2_ctrl_new_int_menu(&vs->ctrls, NULL, V4L2_CID_LINK_FREQ, 0, 0,
			       virt_sensor_link_freq_menu);
	ret = vs->ctrls.error;
	if (ret)
		goto err_ctrls;
	sd->ctrl_handler = &vs->ctrls;

	vs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &vs->pad);
	if (ret)
		goto err_ctrls;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_entity;

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_state;

	return 0;

err_state:
	v4l2_subdev_cleanup(sd);
err_entity:
	media_entity_cleanup(&sd->entity);
err_ctrls:
	v4l2_ctrl_handler_free(&vs->ctrls);
	return ret;
}

static void virt_sensor_cleanup_subdev(struct virt_sensor *vs)
{
	v4l2_async_unregister_subdev(&vs->sd);
	v4l2_subdev_cleanup(&vs->sd);
	media_entity_cleanup(&vs->sd.entity);
	v4l2_ctrl_handler_free(&vs->ctrls);
}

static int __init virt_sensor_init(void)
{
	struct platform_device_info pdevinfo = {
		.name = VIRT_SENSOR_NAME,
		.id = PLATFORM_DEVID_NONE,
	};
	struct fwnode_handle *ipu_fwnode, *ep;
	struct virt_sensor *vs;
	struct device *dev;
	int ret;

	vs = kzalloc_obj(*vs);
	if (!vs)
		return -ENOMEM;

	vs->ipu = pci_get_device(vendor, device, NULL);
	if (!vs->ipu) {
		pr_err(VIRT_SENSOR_NAME ": no PCI device %04x:%04x\n",
		       vendor, device);
		ret = -ENODEV;
		goto err_free;
	}
	dev = &vs->ipu->dev;

	/*
	 * The ISYS notifier reads the graph once, at probe time, so a
	 * driver that is already bound would never see this sensor.
	 */
	if (dev->driver) {
		dev_err(dev, "already bound to %s; load %s first\n",
			dev->driver->name, VIRT_SENSOR_NAME);
		ret = -EBUSY;
		goto err_put;
	}
	ep = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (ep) {
		fwnode_handle_put(ep);
		dev_err(dev, "already has a fwnode graph\n");
		ret = -EBUSY;
		goto err_put;
	}

	virt_sensor_init_swnodes(vs);
	ret = software_node_register_node_group(vs->group);
	if (ret) {
		dev_err(dev, "failed to register software nodes: %d\n", ret);
		goto err_put;
	}

	pdevinfo.fwnode = software_node_fwnode(&vs->nodes[SWNODE_SENSOR]);
	vs->pdev = platform_device_register_full(&pdevinfo);
	if (IS_ERR(vs->pdev)) {
		ret = PTR_ERR(vs->pdev);
		dev_err(dev, "failed to register sensor device: %d\n", ret);
		goto err_nodes;
	}

	ipu_fwnode = software_node_fwnode(&vs->nodes[SWNODE_IPU]);
	set_secondary_fwnode(dev, ipu_fwnode);
	dev_info(dev, "%s: fwnode graph attached (port 0, %u lanes, %llu Hz)\n",
		 VIRT_SENSOR_NAME, VIRT_SENSOR_LANES, VIRT_SENSOR_LINK_FREQ);

	ret = virt_sensor_init_subdev(vs);
	if (ret) {
		dev_err(dev, "failed to register subdev: %d\n", ret);
		goto err_fwnode;
	}
	dev_info(&vs->pdev->dev, "subdev %s registered: %ux%u RGB888_1X24\n",
		 vs->sd.name, VIRT_SENSOR_WIDTH, VIRT_SENSOR_HEIGHT);

	virt_sensor = vs;
	return 0;

err_fwnode:
	set_secondary_fwnode(dev, NULL);
	platform_device_unregister(vs->pdev);
err_nodes:
	software_node_unregister_node_group(vs->group);
err_put:
	pci_dev_put(vs->ipu);
err_free:
	kfree(vs);
	return ret;
}
module_init(virt_sensor_init);

static void __exit virt_sensor_exit(void)
{
	struct virt_sensor *vs = virt_sensor;
	struct device *dev = &vs->ipu->dev;

	virt_sensor_cleanup_subdev(vs);
	/*
	 * Unload is refused while ISYS holds the subdev (module reference
	 * taken on bind), but the IPU driver may still be bound with the
	 * sensor never matched. Detaching the graph under a bound driver
	 * is harmless: ISYS only walks it in probe.
	 */
	set_secondary_fwnode(dev, NULL);
	platform_device_unregister(vs->pdev);
	software_node_unregister_node_group(vs->group);
	dev_info(dev, "%s: removed\n", VIRT_SENSOR_NAME);
	pci_dev_put(vs->ipu);
	kfree(vs);
}
module_exit(virt_sensor_exit);

MODULE_DESCRIPTION("Virtual CSI-2 sensor for the IPU4 QEMU test harness");
MODULE_LICENSE("GPL");
