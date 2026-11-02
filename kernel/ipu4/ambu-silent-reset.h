/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Ambu silent reset: when the ISYS needs a power cycle, opening a video node
 * restarts the running stream instead of failing.
 */
#ifndef AMBU_SILENT_RESET_H
#define AMBU_SILENT_RESET_H

#include <linux/types.h>

struct ipu6_isys_video;

int ambu_silent_reset_open(struct ipu6_isys_video *av);
bool ambu_silent_reset_buf_queue(struct ipu6_isys_video *av);
bool ambu_silent_reset_stop(struct ipu6_isys_video *av);

#endif /* AMBU_SILENT_RESET_H */
