/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _SC8547A_UFCS_H
#define _SC8547A_UFCS_H

#include "include/ufcs_class.h"

struct regmap;

/*
 * Master only. Keep the context alive until ufcs_device_unregister().
 * The map uses 8-bit addresses and values, uncached flags and bulk I/O.
 * prepare() resets the idle chip, waits 10 ms and restores board-specific
 * protection settings. Callers own D+/D-, both pumps and watchdog service.
 */
struct sc8547a_ufcs_phy {
	struct regmap *map;
	int (*prepare)(void *ctx);
	void *ctx;
};

struct ufcs_dev *sc8547a_ufcs_register(struct device *dev,
				       struct sc8547a_ufcs_phy *phy);
/* Threaded IRQ, only while D+/D- belongs to UFCS. Quiesce before removal. */
int sc8547a_ufcs_irq(struct ufcs_dev *ufcs);

#endif
