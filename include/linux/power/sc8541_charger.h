/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_POWER_SC8541_CHARGER_H
#define _LINUX_POWER_SC8541_CHARGER_H

#include <linux/errno.h>
#include <linux/kconfig.h>

struct power_supply;

enum sc8541_otg_action {
	SC8541_OTG_PRESENT,
	SC8541_OTG_REVERSE_ON,
	SC8541_OTG_REVERSE_OFF,
	SC8541_OTG_DETACH,
};

#if IS_REACHABLE(CONFIG_CHARGER_SC8541)
int sc8541_otg_configure(struct power_supply *psy, enum sc8541_otg_action action);
#else
static inline int sc8541_otg_configure(struct power_supply *psy,
				     enum sc8541_otg_action action)
{
	return -ENODEV;
}
#endif
#endif
