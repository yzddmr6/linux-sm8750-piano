/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PHY_QCOM_QMP_COMBO_H
#define __PHY_QCOM_QMP_COMBO_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

struct phy;

/**
 * struct qmp_combo_usb_mode_ops - coordinate shared lane reconfiguration
 * @prepare: stop the consumer before its PIPE clock is removed
 * @complete: restart using the actual USB3 availability, including on failure
 *
 * The provider serializes pairs, including registration and removal. Callbacks
 * run in the same sleepable task without the provider's PHY lock held, so the
 * consumer may resume and use its USB PHY. A successful prepare is always
 * followed by complete. A failed prepare must leave USB stopped: it cannot
 * veto the independent DP configuration. Return -ENODEV if no consumer exists.
 * During system sleep, prepare may return -EAGAIN to defer the entire change;
 * the consumer must call qmp_combo_sync_usb_mode() after the PM transition.
 * A failed complete keeps the request pending for the next Type-C event.
 */
struct qmp_combo_usb_mode_ops {
	int (*prepare)(void *data);
	int (*complete)(void *data, bool usb3_available);
};

#if IS_REACHABLE(CONFIG_PHY_QCOM_QMP_COMBO)
int qmp_combo_set_usb_mode_ops(struct phy *phy,
			       const struct qmp_combo_usb_mode_ops *ops,
			       void *data, bool *usb3_available);
int qmp_combo_sync_usb_mode(struct phy *phy);
#else
static inline int qmp_combo_sync_usb_mode(struct phy *phy)
{
	return -EOPNOTSUPP;
}

static inline int qmp_combo_set_usb_mode_ops(struct phy *phy,
					     const struct qmp_combo_usb_mode_ops *ops,
					     void *data, bool *usb3_available)
{
	return -EOPNOTSUPP;
}
#endif

#endif
