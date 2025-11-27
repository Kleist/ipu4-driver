#ifndef IPU4_COMPAT_H
#define IPU4_COMPAT_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
// https://github.com/torvalds/linux/commit/dbbe7eaf0e4795bf003ac06872aaf52b6b6b1310

/* Simple helper for dev_err_probe() when ERR_CAST() is to be returned. */
#define dev_err_cast_probe(dev, ___err_ptr, fmt, ...) \
	ERR_PTR(dev_err_probe(dev, PTR_ERR(___err_ptr), fmt, ##__VA_ARGS__))

#endif // 6.11


#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)

// From include/uapi/linux/media-bus-format.h
/* Generic line based metadata formats for serial buses. Next is 0x8008. */
#define MEDIA_BUS_FMT_META_8			0x8001
#define MEDIA_BUS_FMT_META_10			0x8002
#define MEDIA_BUS_FMT_META_12			0x8003
#define MEDIA_BUS_FMT_META_14			0x8004
#define MEDIA_BUS_FMT_META_16			0x8005
#define MEDIA_BUS_FMT_META_20			0x8006
#define MEDIA_BUS_FMT_META_24			0x8007

// commit d0fde6aae2bacdc024fff43461ba0f325375fa97
// Author: Sakari Ailus <sakari.ailus@linux.intel.com>
// Date:   Fri Oct 13 10:16:06 2023 +0200
//
//     media: v4l: subdev: Rename sub-device state information access functions

#define v4l2_subdev_state_get_format(state, pad, stream) \
    v4l2_subdev_state_get_stream_format(state, pad, stream)

#define v4l2_subdev_state_get_crop(state, pad, stream) \
    v4l2_subdev_state_get_stream_crop(state, pad, stream)

#else // >= 6.10
// Converting old call to new, because old takes 3 args
#define v4l2_subdev_get_pad_format(e1, e2, e3) \
    v4l2_subdev_state_get_format(e2, e3)

#endif // 6.10


#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 103)
// Before 6.9, pm_runtime_get_if_active took an extra argument.
// See https://github.com/torvalds/linux/commit/c0ef3df8dbaef51ee4cfd58a471adf2eaee6f6b3
// It was backported in 6.6.103.
// See https://github.com/gregkh/linux/commit/ac2e62cab0977960420a0f4b3197932591a104f8
#define pm_runtime_get_if_active(DEV) pm_runtime_get_if_active(DEV, true)
#endif // 6.9

#endif // IPU4_COMPAT_H
