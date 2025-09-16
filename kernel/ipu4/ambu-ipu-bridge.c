// SPDX-License-Identifier: GPL-2.0
/* Author: Andreas Helbech Kleist <andreaskleist@gmail.com> */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mei_cl_bus.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <media/v4l2-fwnode.h>

#include "ambu-ipu-bridge.h"

static struct ipu_bridge *static_bridge;

static const struct mipi_bridge_config mipi_bridge_configs[] = {
	{
		.compatible = "ambu,tc358748",
		.i2c_addr = 0x0e,
		.i2c_adapter = 0,
		.link_freq = 317250000, // ieib475_pll_configurations op_sys_clk from 4.19 driver
		.mcsi_port = 0, 
		.csi2_device = 0,
	},
	{
		.compatible = "ambu,tc358748",
		.i2c_addr = 0x0e,
		.i2c_adapter = 3,
		.link_freq = 317250000, // ieib475_pll_configurations op_sys_clk from 4.19 driver
		.mcsi_port = 1, 
		.csi2_device = 4,
	},

};

static const struct ipu_property_names prop_names = {
	.clock_frequency = "clock-frequency",
	.bus_type = "bus-type",
	.data_lanes = "data-lanes",
	.remote_endpoint = "remote-endpoint",
	.link_frequencies = "link-frequencies",
};

static void ipu_bridge_create_fwnode_properties(
	struct ipu_sensor *sensor,
	struct ipu_bridge *bridge,
	const struct mipi_bridge_config *cfg)
{
	sensor->prop_names = prop_names;

	sensor->local_ref[0] = SOFTWARE_NODE_REFERENCE(&sensor->swnodes[SWNODE_IPU_ENDPOINT]);
	sensor->remote_ref[0] = SOFTWARE_NODE_REFERENCE(&sensor->swnodes[SWNODE_SENSOR_ENDPOINT]);

	sensor->dev_properties[0] = PROPERTY_ENTRY_U32(
					sensor->prop_names.clock_frequency,
					cfg->link_freq);
	sensor->ep_properties[0] = PROPERTY_ENTRY_U32(
					sensor->prop_names.bus_type,
					V4L2_FWNODE_BUS_TYPE_CSI2_DPHY);
	sensor->ep_properties[1] = PROPERTY_ENTRY_U32_ARRAY_LEN(
					sensor->prop_names.data_lanes,
					bridge->data_lanes, sensor->lanes);
	sensor->ep_properties[2] = PROPERTY_ENTRY_REF_ARRAY(
					sensor->prop_names.remote_endpoint,
					sensor->local_ref);

	sensor->ep_properties[3] = PROPERTY_ENTRY_U64_ARRAY_LEN(
		sensor->prop_names.link_frequencies,
		&cfg->link_freq,
		1); // TODO - does this need to be an array when we always have 1?

	sensor->ipu_properties[0] = PROPERTY_ENTRY_U32_ARRAY_LEN(
					sensor->prop_names.data_lanes,
					bridge->data_lanes, sensor->lanes);
	sensor->ipu_properties[1] = PROPERTY_ENTRY_REF_ARRAY(
					sensor->prop_names.remote_endpoint,
					sensor->remote_ref);
}

static void ipu_bridge_init_swnode_names(struct ipu_sensor *sensor)
{
	snprintf(sensor->node_names.remote_port,
		 sizeof(sensor->node_names.remote_port),
		 SWNODE_GRAPH_PORT_NAME_FMT, sensor->link);
	snprintf(sensor->node_names.port,
		 sizeof(sensor->node_names.port),
		 SWNODE_GRAPH_PORT_NAME_FMT, 0); /* Always port 0 */
	snprintf(sensor->node_names.endpoint,
		 sizeof(sensor->node_names.endpoint),
		 SWNODE_GRAPH_ENDPOINT_NAME_FMT, 0); /* And endpoint 0 */
}

static void ipu_bridge_init_swnode_group(struct ipu_sensor *sensor)
{
	struct software_node *nodes = sensor->swnodes;

	sensor->group[SWNODE_SENSOR_HID] = &nodes[SWNODE_SENSOR_HID];
	sensor->group[SWNODE_SENSOR_PORT] = &nodes[SWNODE_SENSOR_PORT];
	sensor->group[SWNODE_SENSOR_ENDPOINT] = &nodes[SWNODE_SENSOR_ENDPOINT];
	sensor->group[SWNODE_IPU_PORT] = &nodes[SWNODE_IPU_PORT];
	sensor->group[SWNODE_IPU_ENDPOINT] = &nodes[SWNODE_IPU_ENDPOINT];
}

static void ipu_bridge_create_connection_swnodes(struct ipu_bridge *bridge,
						 struct ipu_sensor *sensor)
{
	struct software_node *nodes = sensor->swnodes;

	ipu_bridge_init_swnode_names(sensor);

	nodes[SWNODE_SENSOR_HID] = NODE_SENSOR(sensor->name,
					       sensor->dev_properties);
	nodes[SWNODE_SENSOR_PORT] = NODE_PORT(sensor->node_names.port,
					      &nodes[SWNODE_SENSOR_HID]);
	nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT(
						sensor->node_names.endpoint,
						&nodes[SWNODE_SENSOR_PORT],
						sensor->ep_properties);
	nodes[SWNODE_IPU_PORT] = NODE_PORT(sensor->node_names.remote_port,
					   &bridge->ipu_hid_node);
	nodes[SWNODE_IPU_ENDPOINT] = NODE_ENDPOINT(
						sensor->node_names.endpoint,
						&nodes[SWNODE_IPU_PORT],
						sensor->ipu_properties);

	ipu_bridge_init_swnode_group(sensor);
}

static void ipu_bridge_unregister_sensors(struct ipu_bridge *bridge)
{
	struct ipu_sensor *sensor;
	unsigned int i;

	for (i = 0; i < bridge->n_sensors; i++) {
		sensor = &bridge->sensors[i];
		i2c_unregister_device(sensor->i2c_dev);
		software_node_unregister_node_group(sensor->group);
	}
}

static struct i2c_client *ipu_bridge_init_i2c_dev(
	struct device *dev,
	const struct mipi_bridge_config *cfg,
	const struct software_node *swnode)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = { I2C_BOARD_INFO("ambu,tc358748", cfg->i2c_addr) };

	// TODO - when swnode is passed to i2c_new_device, should it also be registered elsewhere?
	info.swnode = swnode;

	adapter = i2c_get_adapter(cfg->i2c_adapter);
	if (!adapter) {
		dev_err(dev, "Invalid I2C adapter %d for port %d",
			cfg->i2c_adapter, cfg->mcsi_port);
		return ERR_PTR(-EINVAL);
	}

	client = i2c_new_client_device(adapter, &info);
	i2c_put_adapter(adapter);
	return client;
}

static int ipu_bridge_setup_mipi_bridge(const struct mipi_bridge_config *cfg,
				     struct ipu_bridge *bridge,
				     struct device *dev)
{
	struct ipu_sensor *sensor;
	int ret;

	if (bridge->n_sensors >= IPU_MAX_PORTS) {
		dev_err(dev, "Exceeded available IPU ports\n");
		return -EINVAL;
	}

	sensor = &bridge->sensors[bridge->n_sensors];

	sensor->lanes = 1; // from 4.19.217 dev_dbg(&csi2->isys->adev->dev, "lane nr %d.\n", nlanes) => "lane nr 1."
	sensor->link = cfg->csi2_device;

	snprintf(sensor->name, sizeof(sensor->name), "%s-%u",
			cfg->compatible, cfg->mcsi_port);

	if (sensor->lanes > IPU_MAX_LANES) {
		dev_err(&sensor->i2c_dev->dev, "Number of lanes is invalid\n");
		return -EINVAL;

	}

	ipu_bridge_create_fwnode_properties(sensor, bridge, cfg);
	ipu_bridge_create_connection_swnodes(bridge, sensor);

	ret = software_node_register_node_group(sensor->group);
	if (ret)
		return ret;

	// HACK - This is probably not the right place to register this device

	sensor->i2c_dev = ipu_bridge_init_i2c_dev(dev, cfg, &sensor->swnodes[
							SWNODE_SENSOR_HID]);
	if (IS_ERR(sensor->i2c_dev)) {
		ret = PTR_ERR(sensor->i2c_dev);
		sensor->i2c_dev = NULL;
		goto err_free_swnodes;
	}

	dev_info(dev, "Found supported sensor %s\n",
			dev_name(&sensor->i2c_dev->dev));

	bridge->n_sensors++;

	return 0;

err_free_swnodes:
	software_node_unregister_node_group(sensor->group);
	return ret;
}

static int ipu_bridge_setup_mipi_bridges(struct ipu_bridge *bridge,
				      struct device *dev)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mipi_bridge_configs); i++) {
		ret = ipu_bridge_setup_mipi_bridge(&mipi_bridge_configs[i], bridge, dev);
		if (ret)
			goto err_unregister_sensors;
	}

	return 0;

err_unregister_sensors:
	ipu_bridge_unregister_sensors(bridge);
	return ret;
}

static int ipu_bridge_sensors_are_ready(void)
{
	//TODO - is there anything we should do here to find out if toshiba bridge is ready?
	return true;
}

int ambu_ipu_bridge_init(struct device *dev)
{
	struct fwnode_handle *fwnode;
	struct ipu_bridge *bridge;
	unsigned int i;
	int ret;

	if (!ipu_bridge_sensors_are_ready())
		return -EPROBE_DEFER;

	static_bridge = kzalloc(sizeof(*bridge), GFP_KERNEL);
	if (!static_bridge)
		return -ENOMEM;
	bridge = static_bridge; // Use alias to avoid changing every line in this file

	strscpy(bridge->ipu_node_name, IPU_HID,
		sizeof(bridge->ipu_node_name));
	bridge->ipu_hid_node.name = bridge->ipu_node_name;

	ret = software_node_register(&bridge->ipu_hid_node);
	if (ret < 0) {
		dev_err(dev, "Failed to register the IPU HID node\n");
		goto err_free_bridge;
	}

	/*
	 * Map the lane arrangement, which is fixed for the IPU3 (meaning we
	 * only need one, rather than one per sensor). We include it as a
	 * member of the struct ipu_bridge rather than a global variable so
	 * that it survives if the module is unloaded along with the rest of
	 * the struct.
	 */
	for (i = 0; i < IPU_MAX_LANES; i++)
		bridge->data_lanes[i] = i + 1;

	ret = ipu_bridge_setup_mipi_bridges(bridge, dev);
	if (ret || bridge->n_sensors == 0)
		goto err_unregister_ipu;

	dev_info(dev, "Connected %d cameras\n", bridge->n_sensors);

	fwnode = software_node_fwnode(&bridge->ipu_hid_node);
	if (!fwnode) {
		dev_err(dev, "Error getting fwnode from ipu software_node\n");
		ret = -ENODEV;
		goto err_unregister_sensors;
	}

	// HACK - there appears to be no inverse of set_secondary_fwnode, so we do this manually
	if (fwnode->secondary) {
		dev_err(dev, "Expected no fwnode->secondary, cannot proceed");
		ret = -ENOTUNIQ;
		goto err_unregister_sensors;
	}
	dev->fwnode->secondary = fwnode;
	return 0;

err_unregister_sensors:
	ipu_bridge_unregister_sensors(bridge);
err_unregister_ipu:
	software_node_unregister(&bridge->ipu_hid_node);
err_free_bridge:
	kfree(bridge);
	static_bridge = NULL;
	return ret;
}
EXPORT_SYMBOL_NS_GPL(ambu_ipu_bridge_init, INTEL_IPU_BRIDGE);


void ambu_ipu_bridge_uninit(struct device *dev)
{
	dev->fwnode->secondary = NULL;
	ipu_bridge_unregister_sensors(static_bridge);
	software_node_unregister(&static_bridge->ipu_hid_node);
	kfree(static_bridge);
	static_bridge = NULL;
}
EXPORT_SYMBOL_NS_GPL(ambu_ipu_bridge_uninit, INTEL_IPU_BRIDGE);
MODULE_LICENSE("GPL v2");
