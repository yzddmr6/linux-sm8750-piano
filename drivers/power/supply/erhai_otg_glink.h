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
/*
 * erhai_otg_glink.c shared interface: pmic-glink battery_charger driver,
 * exports ADSP property GET/SET helpers so the SC8547 fast-charge driver and
 * the PPS host engine (erhai_pps) can consult and drive the DSP-side charger
 * classification/negotiation (vendor usb_property_id / battery_property_id,
 * e.g. USB_VOOC_FAST_CHG_TYPE = 26, USB_SET_PPS_VOLT = 34) — the Android
 * engine's is_pd_svooc_adapter()/adapter_sid_power() retry policy.
 */
#ifndef _ERHAI_OTG_GLINK_H_
#define _ERHAI_OTG_GLINK_H_

#include <linux/errno.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_ERHAI_OTG_GLINK)
int erhai_otg_adsp_usb_prop(u32 property);
/*
 * Read/write one ADSP battery_charger property over pmic_glink.  opcode is
 * the battery_charger target:
 *	BC_USB_PROPERTY_GET(0x32) / BC_USB_PROPERTY_SET(0x33) - USB psy
 *	BC_BATTERY_PROPERTY_GET(0x30) / BC_BATTERY_PROPERTY_SET(0x31) - battery
 * GET returns the property value (>= 0) or a negative errno; SET returns 0 or
 * a negative errno (-ENODEV glink session down, -EINVAL unknown opcode,
 * -EINVAL property rejected by the ADSP in its ret_code).
 */
int erhai_otg_adsp_prop_get(u32 opcode, u32 property);
int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value);
int erhai_otg_glink_send_raw(const void *data, size_t len);
bool erhai_otg_charge_inhibited(void);

/*
 * Sibling RX hook: inbound pmic_glink messages owned by the wireless-pen
 * hboost owner (0x8011) are routed to the registered callback.  The
 * CPS8601 pen-charger driver piggybacks on this client for its rail
 * requests and needs the ADSP ACK that this client's own message loop
 * would otherwise drop.  Runs in atomic glink-rx context.
 */
#define ERHAI_OTG_WLS_PEN_OWNER		0x8011
typedef void (*erhai_otg_glink_rx_fn)(const void *data, size_t len);
void erhai_otg_glink_set_rx_hook(erhai_otg_glink_rx_fn fn);

struct pmic_glink_client;
/*
 * Register a pmic_glink client for a non-child device (see the comment
 * in the .c).  Memory is owned by erhai_otg (eternal); the caller MUST
 * call pmic_glink_client_neuter() before unloading.
 */
struct pmic_glink_client *
erhai_otg_glink_client_register_sibling(u32 id,
					void (*cb)(const void *, size_t, void *),
					void (*pdr)(void *, int), void *priv);
#else
static inline int erhai_otg_adsp_usb_prop(u32 property)
{
	return -ENODEV;
}
static inline int erhai_otg_adsp_prop_get(u32 opcode, u32 property)
{
	return -ENODEV;
}
static inline bool erhai_otg_charge_inhibited(void)
{
	return false;
}
static inline int erhai_otg_adsp_prop_set(u32 opcode, u32 property, u32 value)
{
	return -ENODEV;
}
#endif

#endif /* _ERHAI_OTG_GLINK_H_ */
