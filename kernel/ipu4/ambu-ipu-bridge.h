/* SPDX-License-Identifier: GPL-2.0 */
/* Author: Andreas Helbech Kleist <andreaskleist@gmail.com> */
#ifndef __AMBU_IPU_BRIDGE_H
#define __AMBU_IPU_BRIDGE_H

#include <linux/property.h>
#include <linux/types.h>
#include <media/v4l2-fwnode.h>

#define IPU_HID				"INT343E"
#define IPU_MAX_LANES				4
#define IPU_MAX_PORTS				4
#define MAX_NUM_LINK_FREQS			3

/* Values are educated guesses as we don't have a spec */
#define IPU_SENSOR_ROTATION_NORMAL		0
#define IPU_SENSOR_ROTATION_INVERTED		1

#define NODE_SENSOR(_HID, _PROPS)		\
	((const struct software_node) {		\
		.name = _HID,			\
		.properties = _PROPS,		\
	})

#define NODE_PORT(_PORT, _SENSOR_NODE)		\
	((const struct software_node) {		\
		.name = _PORT,			\
		.parent = _SENSOR_NODE,		\
	})

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)	\
	((const struct software_node) {		\
		.name = _EP,			\
		.parent = _PORT,		\
		.properties = _PROPS,		\
	})

enum ipu_sensor_swnodes {
	SWNODE_SENSOR_HID,
	SWNODE_SENSOR_PORT,
	SWNODE_SENSOR_ENDPOINT,
	SWNODE_IPU_PORT,
	SWNODE_IPU_ENDPOINT,
	SWNODE_COUNT
};

struct ipu_property_names {
	char clock_frequency[16];
	char rotation[9];
	char orientation[12];
	char bus_type[9];
	char data_lanes[11];
	char remote_endpoint[16];
	char link_frequencies[17];
};

struct ipu_node_names {
	char port[7];
	char endpoint[11];
	char remote_port[7];
};

struct mipi_bridge_config {
	u8 mcsi_port;
	// "IPU4 CSI2 %d" media-ctl device is this mipi bridge connected to?
	u8 csi2_device;
	u8 i2c_addr;
	u8 i2c_adapter;
	const char *compatible;
	u64 link_freq;
};

struct ipu_sensor {
	/* append ssdb.link(u8) in "-%u" format as suffix of HID */
	char name[ACPI_ID_LEN + 4];

	u8 link; /* Hard-coded "SSDB" info*/
	u8 lanes; /* Hard-coded "SSDB" info*/

	/* SWNODE_COUNT + 1 for terminating NULL */
	const struct software_node *group[SWNODE_COUNT + 1];
	struct software_node swnodes[SWNODE_COUNT];
	struct ipu_node_names node_names;

	struct ipu_property_names prop_names;
	struct property_entry ep_properties[5];
	struct property_entry dev_properties[5];
	struct property_entry ipu_properties[3];
	struct software_node_ref_args local_ref[1];
	struct software_node_ref_args remote_ref[1];
	struct i2c_client *i2c_dev;
};

struct ipu_bridge {
	char ipu_node_name[ACPI_ID_LEN];
	struct software_node ipu_hid_node;
	u32 data_lanes[4];
	unsigned int n_sensors;
	struct ipu_sensor sensors[IPU_MAX_PORTS];
};

int ambu_ipu_bridge_init(struct device *dev);
void ambu_ipu_bridge_uninit(struct device *dev);

#endif
