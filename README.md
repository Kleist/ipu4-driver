# Intel IPU4 Kernel Driver
This repository contains an out of tree Linux kernel driver for IPU4. It is based on the upstream IPU6 driver which was upstreamed in 6.10. To make future synchronization and upstreaming possible, the code is kept as close as possible to the latest synchronized upstream IPU6 driver.

It is the intention over time to follow upstream kernel development, and submit this driver upstream as well, but there is no timeline for when this will happen.

## Caveats
The driver currently does not work with any publicly available hardware. As a bare minimum, you need to change `ambu_ipu_bridge_*` calls to something else.

## How the IPU4 support is implemented
Whenever there is a diffference between IPU4 and IPU6, it is either handled with `#ifdef IPU6` or with new functions that are named ipu4 instead of ipu6. The `IPU6` define It is not intended to be ever set, it is only there to minimize conflicts when backporting upstream changes. If you need a driver for IPU6, use the one in the kernel.

## Supported Devices
| IPU Version | PCI Device ID | Description |
|-------------|---------------|-------------|
| IPU4        | 0x5a88        | 4th Generation IPU |

The driver currently only supports IPU4, not IPU4P.

## Tested kernel versions
* 6.6.111
* 6.12.47

## History
This driver has it's origin in two different drivers:
* The IPU6 driver which was upstreamed by Intel, see https://lore.kernel.org/all/?q=s:%22media:+ipu6:%22 and https://github.com/torvalds/linux/tree/master/drivers/media/pci/intel/ipu6 .
* The IPU4 driver which was part of clear linux https://github.com/clearlinux-pkgs/linux-iot-lts2018

IPU4 support was hacked onto the IPU6 driver to work with 6.6 by @Kleist. See https://lore.kernel.org/all/e136389011517dbc65b30f6bf0b1a9c49ab4e599.camel@gmail.com/ for more information about this work. This work was shared in https://github.com/Kleist/linux/tree/kleist-v6.6-ipu4-hacks-1 .

It was recently updated to work as an out-of-tree module on 6.6 and 6.12 (and possibly working on some intermediate versions).

## Scripts
The scripts added will not work out of the box, but should be seen as a source of inspiration for how one could work with porting this to other devices, or e.g. add IPU4P support.

* trace_ipu4.sh: Setup a stream with media-ctl and capture some frames using gstreamer or yavta
* trace_functions.sh: Helper functions for trace_ipu4.sh
* split_trace.sh: Split a trace based on the markers inserted by trace_ipu4.sh
* postprocess_trace.py: Convert register addresses to register names

## Module Structure

- **intel-ipu4**: Core driver module handling device discovery, initialization, and hardware management
- **intel-ipu4-isys**: Input System module for camera interface and V4L2 video device support
- **ambu-ipu-bridge**: Loads a proprietary driver for a tc358748 Toshiba MIPI Bridge driver.

## License

This project is licensed under the GNU General Public License v2.0 (GPL-2.0-only). See the [LICENSE](LICENSE) file for details.
