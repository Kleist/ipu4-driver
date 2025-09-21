#ifndef IPU4_COMPAT_H
#define IPU4_COMPAT_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)

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

#endif // IPU4_COMPAT_H
