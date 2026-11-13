/* SPDX-License-Identifier: GPL-2.0-only */
// Portions ported to Linux from the OnePlus Android kernel driver
//   vendor/oplus/kernel/charger/wireless_pen/oplus_wireless_pen_glink.c
// and the Android qti_battery_charger.c battery/USB property mapping,
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which have license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved. /
//    Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc.)
//
/* The battery manager is the sole battery_charger GLINK client. */
#ifndef _ERHAI_CHARGING_H_
#define _ERHAI_CHARGING_H_

#include <linux/types.h>

struct device;
struct pmic_glink_client;

/* Direct-charge route status on this attachment. */
#define ERHAI_SERIAL		1U
#define ERHAI_UFCS		2U
#define ERHAI_PPS		4U
struct erhai_probe_status {
	unsigned int ready;
	unsigned int responded;
	unsigned int protected;
	bool delivering;
};

/* All callbacks run under battmgr's charge_lock, including suspend/teardown. */
struct erhai_pps_ops {
	ssize_t (*describe)(void *data, char *buf, ssize_t off);
	bool (*enabled)(void);
	void (*probe_status)(void *data, struct erhai_probe_status *status);
	int (*serial_session)(bool enable);
	bool (*ufcs_active)(void *data);
	unsigned int (*ufcs_update)(void *data);
	bool (*available)(void *data);
	void (*recharge)(void *data);
	bool (*active)(void *data);
	unsigned int (*update)(void *data, bool caps_changed);
	int (*stop)(void *data, bool upgrading);
	void (*detached)(void *data);
};
int erhai_pps_register(struct device *parent, const struct erhai_pps_ops *ops,
		       void *data);
void erhai_pps_unregister(struct device *parent);
int erhai_pps_apdo_current(u32 min_mv, u32 max_mv, u32 *selected);
int erhai_otg_adsp_usb_prop(u32 property);
int erhai_otg_adsp_prop_get(u32 opcode, u32 property);
int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value);
void erhai_charge_tail_complete(void);
bool erhai_otg_charge_inhibited(void);
struct pmic_glink_client *
erhai_otg_glink_client_register_sibling(struct device *consumer, u32 id,
		void (*cb)(const void *, size_t, void *),
		void (*pdr)(void *, int), void *priv);
#endif
