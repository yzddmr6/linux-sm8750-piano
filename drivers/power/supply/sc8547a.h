/* SPDX-License-Identifier: GPL-2.0-only */
// Ported to Linux from the OnePlus Android kernel driver header
//   vendor/oplus/kernel/charger/v2/voocphy/phy/oplus_sc8547a.h
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which has license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved.)
//
/*
 * sc8547a.h — Southchip SC8547A drive/state interface usable by other
 * charger drivers (erhai_pps).  Only state queries, never register access.
 */
#ifndef _SC8547A_H_
#define _SC8547A_H_

#include <linux/types.h>

/*
 * Is the serial VOOC ladder currently DELIVERING (CP enabled, real SuperVOOC
 * current flowing)?  The PPS host engine stands down while this is true; the
 * genuine serial-SVOOC path keeps ownership of the plug.
 */
bool sc8547a_fastchg_delivering(void);
int sc8547a_ufcs_start(unsigned int min_mv, unsigned int max_mv);
int sc8547a_ufcs_request(unsigned int *mv, unsigned int *ma);
int sc8547a_ufcs_stop(void);
int sc8547a_ufcs_monitor(void);
struct erhai_probe_status;
void sc8547a_probe_status(struct erhai_probe_status *status);
void sc8547a_detached(void);
int sc8547a_serial_session(bool enable);
ssize_t sc8547a_serial_describe(char *buf, ssize_t off, bool *active);

/*
 * Is the serial VOOC ladder in the identify or deliver phase (see sc8547a.c)?
 * The PPS host engine stands down while this is true, not just while
 * sc8547a_fastchg_delivering() reports real current — the identify window must
 * get its full 5 V chance before PPS negotiates.
 */
bool sc8547a_fastchg_identifying(void);

/*
 * PPS host-engine charge-pump API (b5).  Lets erhai_pps run the SC8547A 2:1
 * charge pump from a PD/PPS contract: it servos the PPS voltage so the chip
 * input sits at ~2*vbat + margin, then engages the CP with the same hardware
 * sequence the serial-VOOC deliver path uses, and feeds the 1000 ms REG_09
 * watchdog every poll.  All gated on the master instance; the serial ladder
 * (sc8547a_fastchg_delivering) trumps this path.
 */
int sc8547a_pps_snapshot(int *vbus_mv, int *vbat_mv, int *ibus_ma);
int sc8547a_pps_faults(void);
bool sc8547a_pps_cp_engaged(void);
int sc8547a_pps_cp_engage(int *need_mv);
int sc8547a_pps_cp_feed(int *vbus_mv, int *vbat_mv);
int sc8547a_pps_cp_disengage(void);
int sc8547a_pps_share(int master_ma, int margin_mv, unsigned int demand_ma, int *slave_ma);

/* Keep the master ADC on for a whole PPS session (on) / release it (off). */
int sc8547a_pps_adc_hold(bool on);

/* Raw master REG_06 / REG_07 for the erhai_pps trace (read-only). */
int sc8547a_pps_regs(u8 *r06, u8 *r07);

#endif /* _SC8547A_H_ */
