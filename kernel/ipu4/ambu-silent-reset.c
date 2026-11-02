// SPDX-License-Identifier: GPL-2.0
/*
 * Ambu silent reset: when the ISYS needs a power cycle (lost buffers at
 * stream stop set need_reset), opening a video node restarts the running
 * stream: stream off, a runtime PM power cycle of the ISYS, which also
 * closes and reopens the firmware, and stream on, deferred to the next
 * queued buffer when none is queued.
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include "ambu-silent-reset.h"
#include "ipu6-bus.h"
#include "ipu6-isys.h"
#include "ipu6-isys-csi2.h"
#include "ipu6-isys-queue.h"
#include "ipu6-isys-video.h"

static bool force_need_reset;
module_param(force_need_reset, bool, 0644);
MODULE_PARM_DESC(force_need_reset, "emulate 'isys power cycle required' on next video_open");

static bool silent_reset_enable = true;
module_param(silent_reset_enable, bool, 0644);
MODULE_PARM_DESC(silent_reset_enable, "perform a silent reset of existing streams when 'isys power cycle required' would otherwise be logged");

/* Wait until the firmware has returned the buffers of the node. */
static bool wait_for_no_active_buffers(struct ipu6_isys_video *av)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(1000);
	struct ipu6_isys_queue *aq = &av->aq;
	unsigned long flags;
	bool empty;

	do {
		spin_lock_irqsave(&aq->lock, flags);
		empty = list_empty(&aq->active);
		spin_unlock_irqrestore(&aq->lock, flags);
		if (empty)
			return true;
		msleep_interruptible(10);
	} while (time_before(jiffies, timeout));

	return false;
}

static int restart_stream(struct ipu6_isys_video *av)
{
	struct ipu6_isys *isys = av->isys;
	struct device *dev = &isys->adev->auxdev.dev;
	bool queued;
	int ret;

	dev_info(dev, "Restarting %s\n", av->vdev.name);

	/* Stream off must not let runtime suspend run in the middle. */
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	mutex_lock(&isys->stream_mutex);

	if (!wait_for_no_active_buffers(av)) {
		dev_err(dev, "%s: buffers still with the firmware\n",
			av->vdev.name);
		ret = -EIO;
		goto out;
	}

	ret = ipu6_isys_video_set_streaming(av, 0);
	if (ret)
		goto out;

	/* The runtime PM callbacks close and reopen the firmware. */
	ret = ipu6_bus_reset_device(isys->adev);
	if (ret)
		goto out;

	/* Stream on needs a buffer; without one, the next buf_queue does it. */
	spin_lock_irq(&av->aq.lock);
	queued = !list_empty(&av->aq.incoming);
	spin_unlock_irq(&av->aq.lock);
	if (queued)
		ret = ipu6_isys_video_set_streaming(av, 1);
	else
		av->restart_pending = true;

out:
	mutex_unlock(&isys->stream_mutex);
	pm_runtime_put(dev);

	if (ret)
		dev_err(dev, "Restarting %s failed: %d\n", av->vdev.name, ret);
	else
		dev_info(dev, "Restarting %s done\n", av->vdev.name);

	return ret;
}

static int restart_streams(struct ipu6_isys_video *av)
{
	struct ipu6_isys *isys = av->isys;
	unsigned int nports = isys->pdata->ipdata->csi2.nports;
	unsigned int i, j;

	for (i = 0; i < nports; i++) {
		for (j = 0; j < NR_OF_CSI2_SRC_PADS; j++) {
			struct ipu6_isys_video *other = &isys->csi2[i].av[j];

			if (other == av || !other->streaming)
				continue;

			/* Only one stream at a time is supported. */
			return restart_stream(other);
		}
	}

	return 0;
}

/*
 * Called by video_open before upstream's need_reset check. A failed restart
 * leaves need_reset set, so the check reports "isys power cycle required".
 */
int ambu_silent_reset_open(struct ipu6_isys_video *av)
{
	struct ipu6_isys *isys = av->isys;
	struct device *dev = &isys->adev->auxdev.dev;
	int ret;

	mutex_lock(&isys->mutex);
	while (isys->resetting) {
		mutex_unlock(&isys->mutex);
		msleep_interruptible(100);
		mutex_lock(&isys->mutex);
	}

	if (!silent_reset_enable || !(isys->need_reset || force_need_reset)) {
		mutex_unlock(&isys->mutex);
		return 0;
	}

	dev_warn(dev, "restarting other streams (need_reset=%d, force_need_reset=%d)\n",
		 isys->need_reset, force_need_reset);
	isys->resetting = true;
	force_need_reset = false;
	mutex_unlock(&isys->mutex);

	ret = restart_streams(av);

	mutex_lock(&isys->mutex);
	isys->resetting = false;
	isys->need_reset = !!ret;
	mutex_unlock(&isys->mutex);

	dev_info(dev, "restarting other streams done: %d\n", ret);

	return 0;
}

/* Called by buf_queue once streaming has started: a pending restart. */
bool ambu_silent_reset_buf_queue(struct ipu6_isys_video *av)
{
	struct device *dev = &av->isys->adev->auxdev.dev;

	guard(mutex)(&av->isys->stream_mutex);

	if (!av->restart_pending)
		return false;

	av->restart_pending = false;
	if (ipu6_isys_video_set_streaming(av, 1))
		dev_err(dev, "restarting %s failed\n", av->vdev.name);

	return true;
}

/* Called by stop_streaming under stream_mutex: nothing to stop if pending. */
bool ambu_silent_reset_stop(struct ipu6_isys_video *av)
{
	lockdep_assert_held(&av->isys->stream_mutex);

	if (!av->restart_pending)
		return false;

	av->restart_pending = false;

	return true;
}
