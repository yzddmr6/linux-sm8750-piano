/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WCD939x USBSS DPDM crossbar control for fast charge.
 *
 * On the OnePlus Pad 3 (erhai) the connector D+/D- lines are routed to the
 * SC8547A charge-pump / VOOC PHY through the WCD939x USBSS analog switch, not
 * directly to the SoC USB PHY. Before the VOOC serial handshake can reach the
 * adapter, the USBSS must be flipped from the normal USB path (DN/DP) to the
 * charge-pump crossbar (DN2/DR2). This helper lets the charge-pump driver
 * request that route without owning the USBSS itself.
 */
#ifndef __LINUX_USB_WCD939X_USBSS_H
#define __LINUX_USB_WCD939X_USBSS_H

#include <linux/types.h>

#if IS_REACHABLE(CONFIG_TYPEC_MUX_WCD939X_USBSS)
/*
 * Route connector D+/D- to the charge-pump crossbar (enable=true) or back to
 * the normal USB path (enable=false). Returns 0 on success, -ENODEV if the
 * USBSS has not probed yet, or a regmap error.
 */
int wcd939x_usbss_dpdm_switch_fast_charge(bool enable);
#else
static inline int wcd939x_usbss_dpdm_switch_fast_charge(bool enable)
{
	return -ENODEV;
}
#endif

#endif /* __LINUX_USB_WCD939X_USBSS_H */
