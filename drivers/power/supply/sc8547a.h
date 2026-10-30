/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __SC8547A_H
#define __SC8547A_H

#include <linux/kconfig.h>
#include <linux/errno.h>
#include <linux/types.h>

struct device_node;
struct sc8547a;

#define SC8547A_PUMPS 2

/* Owned by the component master; its charge mutex protects these pointers. */
struct sc8547a_group {
	struct device_node *nodes[SC8547A_PUMPS];
	struct sc8547a *pumps[SC8547A_PUMPS];
	void (*fault)(void *data);
	void *data;
};

struct sc8547a_sample {
	int vbus_mv;
	int vbat_mv;
	int ibus_ma;
	bool switching;
};

#if IS_REACHABLE(CONFIG_CHARGER_SC8547A)
int sc8547a_prepare(struct sc8547a *cp);
int sc8547a_start(struct sc8547a *cp);
int sc8547a_sample(struct sc8547a *cp, struct sc8547a_sample *sample);
int sc8547a_stop(struct sc8547a *cp);
#else
static inline int sc8547a_prepare(struct sc8547a *cp) { return -ENODEV; }
static inline int sc8547a_start(struct sc8547a *cp) { return -ENODEV; }
static inline int sc8547a_sample(struct sc8547a *cp, struct sc8547a_sample *sample)
{
	return -ENODEV;
}
static inline int sc8547a_stop(struct sc8547a *cp) { return -ENODEV; }
#endif

#endif
