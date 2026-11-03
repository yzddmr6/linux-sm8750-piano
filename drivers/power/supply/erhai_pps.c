// SPDX-License-Identifier: GPL-2.0-only
// PPS negotiation/limit logic ported to Linux from the Android charger_pd
// PPS handling in vendor/oplus/kernel/charger/,
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which has license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved.)
//
/*
 * erhai_pps.c — ADSP PPS fast-charge host engine (PD/PPS-only bricks).
 *
 * The SC8547A driver owns the proprietary serial (S)VOOC ladder (genuine
 * OnePlus bricks).  PD/PPS-only chargers (3rd-party bricks Android labels
 * "SuperVOOC", e.g. a 100 W Anker) never speak serial VOOC; Android fast-
 * charges them by renegotiating a PPS APDO through the ADSP charger_pd
 * firmware that is also alive on our boot, raising VBUS to ~8.4 V so the
 * PMIC buck converts (8.4 V x 2 A ≈ 17-19 W -> 4.1 V x ~4.4 A pack), then
 * granting the pack demand through the battery-psy profile props.
 *
 * That negotiation is just battery_charger property writes on the pmic_glink
 * edge this file already shares with erhai_otg_glink (v2 usb_property_id /
 * battery_property_id from oplus_hal_adsp.h):
 *
 *	USB (opcode 0x33):  49 USB_PPS_VOOCPHY_ENABLE, 34 USB_SET_PPS_VOLT mV,
 *		35 USB_SET_PPS_CURR mA, 5 USB_INPUT_CURR_LIMIT uA,
 *		39 USB_PPS_MOS_CTRL, 40 USB_PPS_CP_MODE_INIT
 *	Battery (opcode 0x31): 46 BATT_AP_FASTCHG_ALLOW, 10 BATT_CHG_CTRL_LIM uA
 *		(FCC / pack demand), 24 BATT_CHG_EN
 *
 * Live-proven subset (2026-09-23, 100 W Anker): 49/34/35/5 -> VBUS 4.8->8.4 V,
 * pack 1.0->3.6 A when the ADSP is fresh; the battery-psy props are the
 * profile gate Android's engine also drives (the total handshake).  This
 * module automates the whole sequence behind a guarded state machine so a
 * failure always falls back to the ordinary 5 V contract.
 *
 * Design / safety:
 *  - engages ONLY for PD_PPS adapters (ADSP USB_REAL_TYPE == 8) with PPS
 *    support (USB_GET_PPS_TYPE == 1) and NO serial VOOC active
 *    (USB_VOOC_FAST_CHG_TYPE below pps_vooc_min AND sc8547a not delivering) —
 *    the genuine OnePlus brick keeps its existing serial-SVOOC path,
 *  - starts PPS only after a debounce long enough for the serial ladder to
 *    decline (pps_identify_ms), so a slow-starting genuine brick wins,
 *  - steps VBUS up in pps_volt_step_mv increments (vendor order), servos the
 *    PPS current, tapers at vbat/pSoC limits and ALWAYS reverts to the 5 V
 *    contract on exit/detach/abnormal,
 *  - module params mirror the sc8547a driver style; every state crossing and
 *    exit reason is logged; counters/substates behind debugfs.
 *
 * No DT change.  Built as a separate module so shipping/removing it does not
 * touch the OTG/SC8547 drivers.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>			/* simple_write_to_buffer() */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include "erhai_otg_glink.h"
#include "sc8547a.h"

#define CREATE_TRACE_POINTS
#include "erhai_pps_trace.h"

/* sc8547a PPS-CP API (see drivers/power/supply/sc8547a.c) — the 2:1 charge-pump
 * entry points this engine drives when the chip window is reachable.  Resolved
 * lazily via symbol_get() exactly like pps_sc_delivering(): this module must
 * load (and degrade to the buck path) even when the sc8547a module is absent. */
struct erhai_pps_sc_api {
	int	(*snapshot)(int *vbus_mv, int *vbat_mv, int *ibus_ma);
	bool	(*cp_engaged)(void);
	int	(*cp_engage)(int *need_mv);
	int	(*cp_feed)(int *vbus_mv, int *vbat_mv);
	void	(*cp_disengage)(void);
};

static const struct erhai_pps_sc_api *pps_sc_cp_get(void)
{
	static struct erhai_pps_sc_api api;
	static bool resolved;

	/* Latch only on success: the charge-pump module may still be loading
	 * when this engine first looks for it, and a permanent NULL here
	 * would silently degrade every later session to buck charging.
	 * Only symbols that actually resolved may be put back — symbol_put()
	 * on a name that is absent from the kernel BUG_ON()s in
	 * __symbol_put() (kernel/module/main.c), and "module not loaded yet"
	 * is exactly the case this retry exists for. */
	if (!resolved) {
		api.snapshot = (void *)symbol_get(sc8547a_pps_snapshot);
		api.cp_engaged = (void *)symbol_get(sc8547a_pps_cp_engaged);
		api.cp_engage = (void *)symbol_get(sc8547a_pps_cp_engage);
		api.cp_feed = (void *)symbol_get(sc8547a_pps_cp_feed);
		api.cp_disengage = (void *)symbol_get(sc8547a_pps_cp_disengage);
		if (!api.snapshot || !api.cp_engaged || !api.cp_engage ||
		    !api.cp_feed || !api.cp_disengage) {
			if (api.snapshot)
				symbol_put(sc8547a_pps_snapshot);
			if (api.cp_engaged)
				symbol_put(sc8547a_pps_cp_engaged);
			if (api.cp_engage)
				symbol_put(sc8547a_pps_cp_engage);
			if (api.cp_feed)
				symbol_put(sc8547a_pps_cp_feed);
			if (api.cp_disengage)
				symbol_put(sc8547a_pps_cp_disengage);
			memset(&api, 0, sizeof(api));
			return NULL;
		}
		resolved = true;
	}
	return resolved ? &api : NULL;
}

/* returns true while the CP is engaged AND the API is reachable */
static bool pps_cp_is_engaged(void)
{
	const struct erhai_pps_sc_api *a = pps_sc_cp_get();

	return a && a->cp_engaged();
}

static void pps_cp_drop(void)
{
	const struct erhai_pps_sc_api *a = pps_sc_cp_get();

	if (a && a->cp_engaged())
		a->cp_disengage();
}

/* ---- ADSP v2 property ids (oplus_hal_adsp.h), our subset ---- */
/* usb_property_id (BC_USB_PROPERTY_* / opcode 0x32,0x33) */
#define PPS_PROP_USB_INPUT_CURR_LIMIT	5
#define PPS_PROP_USB_ADAP_TYPE		7
#define PPS_PROP_USB_REAL_TYPE		11
#define PPS_PROP_USB_VOOC_FAST_CHG_TYPE	26
#define PPS_PROP_USB_GET_PPS_TYPE	32
#define PPS_PROP_USB_GET_PPS_STATUS	33
#define PPS_PROP_USB_SET_PPS_VOLT	34
#define PPS_PROP_USB_SET_PPS_CURR	35
#define PPS_PROP_USB_GET_PPS_MAX_CURR	36
#define PPS_PROP_USB_PPS_MOS_CTRL	39
#define PPS_PROP_USB_PPS_CP_MODE_INIT	40
#define PPS_PROP_USB_PPS_VOOCPHY_ENABLE	49
/* battery_property_id (BC_BATTERY_PROPERTY_* / opcode 0x30,0x31) */
#define PPS_PROP_BATT_CHG_CTRL_LIM	10
#define PPS_PROP_BATT_TEMP		12
#define PPS_PROP_BATT_CHG_EN		24
#define PPS_PROP_BATT_AP_FASTCHG_ALLOW	46
/* ADSP battery GET subset (battmgr ids, mirror erhai_otg_glink.c) */
#define PPS_PROP_BATT_STATUS		0
#define PPS_PROP_BATT_CAPACITY		4
#define PPS_PROP_BATT_VOLT_NOW		7
#define PPS_PROP_BATT_CURR_NOW		9

#define PPS_USB_TYPE_PD_PPS		8	/* POWER_SUPPLY_USB_TYPE_PD_PPS */

/* USB_INPUT_CURR_LIMIT "unsuspend": no limit from the host (vendor
 * oplus_hal_adsp.c:6294 writes this to release WIRED_CHARGE_SUSPEND) */
#define PPS_ICL_UNSUSPEND		0xFFFFFFFF

#define PPS_USB_OP_GET			0x32
#define PPS_USB_OP_SET			0x33
#define PPS_BATT_OP_GET			0x30
#define PPS_BATT_OP_SET			0x31


/* ---- module parameters (sc8547a-style knobs) ---- */
static bool pps_enable = true;
module_param(pps_enable, bool, 0644);
MODULE_PARM_DESC(pps_enable, "master kill switch for the PPS fast-charge engine");

static unsigned int pps_poll_ms = 2000;
module_param(pps_poll_ms, uint, 0644);
MODULE_PARM_DESC(pps_poll_ms, "engine poll period (ms)");

static unsigned int pps_identify_ms = 10000;
static unsigned int pps_reprobe_ms = 15000;
module_param(pps_reprobe_ms, uint, 0644);
MODULE_PARM_DESC(pps_reprobe_ms, "re-probe interval while the adapter reads as undeployable (ADSP slow start)");
module_param(pps_identify_ms, uint, 0644);
MODULE_PARM_DESC(pps_identify_ms,
		 "wait for the serial-VOOC ladder to decline before PPS engages (ms)");

static unsigned int pps_start_volt_mv = 5500;
module_param(pps_start_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_start_volt_mv,
		 "legacy PPS entry VBUS (mV) - entry now writes the TARGET directly (validated "
		 "2026-09-24: the ADSP latches the PPS APDO range to the first 34 write, so a "
		 "5.5 V seed + step-ramp caps VBUS at ~6.6 V); kept as the ramp liveness floor");

static unsigned int pps_target_volt_mv = 8400;
module_param(pps_target_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_target_volt_mv,
		 "PPS target VBUS (mV) - Android measures 8.4-8.6 V with the 100 W Anker");

static unsigned int pps_max_volt_mv = 8900;
module_param(pps_max_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_max_volt_mv, "hard PPS VBUS ceiling (mV)");

static unsigned int pps_volt_step_mv = 400;
module_param(pps_volt_step_mv, uint, 0644);
MODULE_PARM_DESC(pps_volt_step_mv, "VBUS ramp step (mV) per poll");

static unsigned int pps_start_curr_ma = 1000;
module_param(pps_start_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_start_curr_ma, "PPS entry current (mA)");

static unsigned int pps_target_curr_ma = 4000;
module_param(pps_target_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_target_curr_ma,
		 "PPS input current target (mA), capped per plug by the adapter's own PPS max "
		 "current - lab-validated 4000 through the pump (~39 W, 32-34 C)");

static unsigned int pps_max_curr_ma = 4000;
module_param(pps_max_curr_ma, uint, 0644);
MODULE_PARM_DESC(pps_max_curr_ma, "hard PPS input current ceiling (mA)");

static unsigned int pps_cable_safe_ma = 3000;
module_param(pps_cable_safe_ma, uint, 0644);
MODULE_PARM_DESC(pps_cable_safe_ma,
		 "PPS current cap when the adapter does not report its PPS max current (mA); "
		 "a source offers more than 3 A only over a 5 A e-marked cable");

static unsigned int pps_iclim_ma = 3000;
module_param(pps_iclim_ma, uint, 0644);
MODULE_PARM_DESC(pps_iclim_ma,
		 "USB_INPUT_CURR_LIMIT written on PPS entry (mA); the ADSP honours 3000 when "
		 "the battery demand is granted (validated 2026-09-24)");

static unsigned int pps_fcc_ma = 4400;
module_param(pps_fcc_ma, uint, 0644);
MODULE_PARM_DESC(pps_fcc_ma,
		 "pack charge demand granted via BATT_CHG_CTRL_LIM (mA); Android draws ~4.4 A at 2 A/8.4 V in");

static unsigned int pps_soc_stop = 80;
module_param(pps_soc_stop, uint, 0644);
MODULE_PARM_DESC(pps_soc_stop, "exit PPS at this pSoC (%)");

static unsigned int pps_soc_restart = 75;
module_param(pps_soc_restart, uint, 0644);
MODULE_PARM_DESC(pps_soc_restart,
		 "below this pSoC a stopped PPS session may resume (%)");

static unsigned int pps_vbat_max_mv = 4490;
module_param(pps_vbat_max_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbat_max_mv, "taper/exit at this pack voltage (mV)");

static unsigned int pps_vbat_min_mv = 3300;
module_param(pps_vbat_min_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbat_min_mv, "abort below this pack voltage (mV)");

static unsigned int pps_vbus_collapse_mv = 3800;
module_param(pps_vbus_collapse_mv, uint, 0644);
MODULE_PARM_DESC(pps_vbus_collapse_mv,
		 "VBUS below this while targeting >5 V means a bad cable/adapter -> exit (mV)");

static int pps_temp_min_c = -5;
module_param(pps_temp_min_c, int, 0644);
MODULE_PARM_DESC(pps_temp_min_c, "PPS pack temp floor (C)");

static int pps_temp_max_c = 45;
module_param(pps_temp_max_c, int, 0644);
MODULE_PARM_DESC(pps_temp_max_c, "PPS pack temp ceiling (C)");

static unsigned int pps_vooc_min = 2;
module_param(pps_vooc_min, uint, 0644);
MODULE_PARM_DESC(pps_vooc_min,
		 "ADSP USB_VOOC_FAST_CHG_TYPE at or above this means serial VOOC owns the plug; never engage PPS");

static unsigned int pps_stall_polls = 15;
module_param(pps_stall_polls, uint, 0644);
MODULE_PARM_DESC(pps_stall_polls,
		 "consecutive polls with ~no pack current at >7 V VBUS before exiting (0 = never)");

static bool pps_cp_init = true;
module_param(pps_cp_init, bool, 0644);
MODULE_PARM_DESC(pps_cp_init,
		 "also write USB_PPS_CP_MODE_INIT(40)=1 + USB_PPS_MOS_CTRL(39)=1 on entry (vendor PHY init)");

/* ---- SC8547A charge-pump 2:1 (CP direct-charge from the PPS bus) ---- */
static bool pps_cp_enable = true;
module_param(pps_cp_enable, bool, 0644);
MODULE_PARM_DESC(pps_cp_enable,
		 "drive the SC8547A charge pump 2:1 from the PPS bus when the chip window is reachable "
		 "(b5; owner-testing).  Disable = pure buck path (b4 behaviour).");

static bool pps_cp_buck_suspend = true;
module_param(pps_cp_buck_suspend, bool, 0644);
MODULE_PARM_DESC(pps_cp_buck_suspend,
		 "suspend the buck (USB_INPUT_CURR_LIMIT 0) while the pump runs, as the vendor does "
		 "(WIRED_CHARGE_SUSPEND); any pump stop resumes it in the same poll");

static unsigned int pps_cp_entry_margin_mv = 450;
module_param(pps_cp_entry_margin_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_entry_margin_mv,
		 "pump entry: request 2*vbat + this before engaging (mV; vendor +450)");

static unsigned int pps_cp_lift_ms = 5000;
module_param(pps_cp_lift_ms, uint, 0644);
MODULE_PARM_DESC(pps_cp_lift_ms,
		 "time allowed for the pump to engage after the entry request, buck suspended (ms)");

static unsigned int pps_cp_rearm_ms = 10000;
module_param(pps_cp_rearm_ms, uint, 0644);
MODULE_PARM_DESC(pps_cp_rearm_ms, "hold-off after a pump drop or a failed entry (ms)");

static unsigned int pps_cp_rearm_max = 5;
module_param(pps_cp_rearm_max, uint, 0644);
MODULE_PARM_DESC(pps_cp_rearm_max, "pump attempts per plug-in; then buck path for the rest of it");

static unsigned int pps_cp_vstep_mv = 40;
module_param(pps_cp_vstep_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_vstep_mv, "pump ibus servo: PPS voltage step (mV) per poll (vendor 40)");

static unsigned int pps_cp_vstep_over_mv = 100;
module_param(pps_cp_vstep_over_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_vstep_over_mv,
		 "pump ibus servo: step down (mV) when ibus is over target + 500 mA (vendor 100)");

static unsigned int pps_cp_ibus_band_ma = 100;
module_param(pps_cp_ibus_band_ma, uint, 0644);
MODULE_PARM_DESC(pps_cp_ibus_band_ma, "pump ibus servo: deadband around the target (mA; vendor 100)");

static unsigned int pps_cp_diff_max_mv = 650;
module_param(pps_cp_diff_max_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_diff_max_mv,
		 "never step VBUS up while vbus - 2*vbat at the chip is at or above this (mV; "
		 "the pump drops at 700)");

static unsigned int pps_cp_ibat_max_ma = 9000;
module_param(pps_cp_ibat_max_ma, uint, 0644);
MODULE_PARM_DESC(pps_cp_ibat_max_ma, "pump ibus servo: step VBUS down while the pack takes more than this (mA)");

static unsigned int pps_cp_min_ibus_ma = 300;
module_param(pps_cp_min_ibus_ma, uint, 0644);
MODULE_PARM_DESC(pps_cp_min_ibus_ma,
		 "engaged pump carrying less than this for 4 polls (after pps_cp_settle_ms) is "
		 "released to the buck (mA; vendor 300)");

static unsigned int pps_cp_settle_ms = 2000;
module_param(pps_cp_settle_ms, uint, 0644);
MODULE_PARM_DESC(pps_cp_settle_ms, "grace after an engage before the low-ibus check (ms)");

static unsigned int pps_cp_max_volt_mv = 9900;
module_param(pps_cp_max_volt_mv, uint, 0644);
MODULE_PARM_DESC(pps_cp_max_volt_mv,
		 "pump PPS VBUS ceiling (mV); 9900 keeps the engaged chip under its 10 V VBUS_OVP "
		 "(lab: 9.17 V max at 4 A), vendor 10500");

static unsigned int pps_cp_feed_ms = 300;
module_param(pps_cp_feed_ms, uint, 0644);
MODULE_PARM_DESC(pps_cp_feed_ms,
		 "engine poll period (ms) while the CP is engaged - must stay under the "
		 "SC8547A 1000 ms REG_09 watchdog window");

static unsigned int pps_log_ticks = 60;
module_param(pps_log_ticks, uint, 0644);
MODULE_PARM_DESC(pps_log_ticks,
		 "log this many charging ticks per session to dmesg (default 60 = ~2 minutes at 2000 ms)");

static bool pps_voocphy_enable = true;
module_param(pps_voocphy_enable, bool, 0644);
MODULE_PARM_DESC(pps_voocphy_enable,
		 "value written to USB_PPS_VOOCPHY_ENABLE(49) on PPS entry (default 1 = vendor order; lab experiment 2b sets 0)");

/* ---- state machine ---- */
enum pps_state {
	PPS_IDLE = 0,		/* nothing attached / not a PPS adapter */
	PPS_ARMED,		/* PD_PPS seen; waiting for serial ladder to decline */
	PPS_NEGOTIATE,		/* writing profile gate + PPS props */
	PPS_RAMP,		/* stepping VBUS up toward the target */
	PPS_CHARGING,		/* servoing PPS current against vbat/pSoC */
	PPS_TAPER,		/* near CV: reducing current before exit */
	PPS_EXIT,		/* reverting to the 5 V contract */
	PPS_LAST_STATE,
};

static const char *const pps_state_names[PPS_LAST_STATE] = {
	[PPS_IDLE]	= "idle",
	[PPS_ARMED]	= "armed",
	[PPS_NEGOTIATE]	= "negotiate",
	[PPS_RAMP]	= "ramp",
	[PPS_CHARGING]	= "charging",
	[PPS_TAPER]	= "taper",
	[PPS_EXIT]	= "exit",
};

/* exit reasons (also the log tag) */
#define PPS_EXIT_REASON_DETACH		"detach"
#define PPS_EXIT_REASON_NOT_PPS		"not-a-pps-adapter"
#define PPS_EXIT_REASON_VOOC_ENGAGED	"serial-vooc-engaged"
#define PPS_EXIT_REASON_SOC		"soc-limit"
#define PPS_EXIT_REASON_VBAT_MAX	"vbat-cv"
#define PPS_EXIT_REASON_VBAT_MIN	"vbat-low"
#define PPS_EXIT_REASON_TEMP		"temp-out-of-window"
#define PPS_EXIT_REASON_COLLAPSE	"vbus-collapse"
#define PPS_EXIT_REASON_STALL		"pack-stall"
#define PPS_EXIT_REASON_SET_FAIL	"prop-set-failed"
#define PPS_EXIT_REASON_RAMP_TIMEOUT	"vbus-not-rising"
#define PPS_EXIT_REASON_FORCED		"forced"

struct erhai_pps {
	struct delayed_work	work;
	struct dentry		*debugfs;

	enum pps_state		state;
	const char		*last_reason;
	unsigned long		state_since;	/* jiffies state was entered */
	unsigned long		armed_since;
	unsigned long		ramp_since;
	unsigned int		stall_count;
	bool			plug_logged;
	unsigned long		last_reprobe;	/* adapter line printed for this plug */

	/* what we last commanded / observed */
	unsigned int		set_volt_mv;
	unsigned int		set_curr_ma;
	unsigned int		target_curr_ma;
	int			vbus_mv;
	int			vbat_mv;
	int			ibat_ma;
	int			soc;
	int			temp_c;

	/* stats */
	u64			sessions;
	u32			entries;
	unsigned int		exits;
	unsigned long		charg_ticks;
	unsigned int		cp_engages;
	unsigned int		cp_drops;

	/* pump controller (c3) */
	bool			buck_off;	/* ICL 0 written for the pump */
	bool			adc_held;	/* sc8547a ADC held on for the session */
	unsigned long		cp_lift_since;	/* entry request sent (0 = not lifting) */
	unsigned long		cp_holdoff_until;
	unsigned long		cp_engaged_since;
	unsigned int		cp_low_cnt;	/* consecutive low-ibus polls */
	bool			soc_hold;	/* exited on soc-limit, waiting to restart */
	unsigned long		cp_fed_at;	/* last successful feed */
	bool			inhibit_held;	/* policy inhibit seen this session */
	unsigned int		cp_rearms;	/* pump attempts this plug */
};

static struct erhai_pps *g_pps;
static DEFINE_MUTEX(pps_lock);

static void pps_enter(struct erhai_pps *p, enum pps_state st, const char *why);

/* optional serial-VOOC gate (sc8547a module) */
static bool pps_sc_delivering(void)
{
	static bool (*fn)(void);
	bool delivering = false;

	if (!fn) {
		fn = (bool (*)(void))symbol_get(sc8547a_fastchg_delivering);
		if (fn)
			pr_debug("erhai_pps: sc8547a delivering() gate linked\n");
	}
	if (fn)
		delivering = fn();

	return delivering;
}

/* optional "serial ladder still identifying/delivering" gate (sc8547a module) —
 * the engine must not capture the connector while the ladder is mid-identify
 * (2026-09-25): a genuine SuperVOOC brick identifies in its 5 V window and is
 * only lost when the engine's debounce laps it and lifts VBUS to PPS first. */
static bool pps_sc_identifying(void)
{
	static bool (*fn)(void);
	bool identifying = false;

	if (!fn) {
		fn = (bool (*)(void))symbol_get(sc8547a_fastchg_identifying);
		if (fn)
			pr_debug("erhai_pps: sc8547a identifying() gate linked\n");
	}
	if (fn)
		identifying = fn();

	return identifying;
}

/* optional raw REG_06/REG_07 read for the trace (sc8547a module) */
static int pps_sc_regs(u8 *r06, u8 *r07)
{
	static int (*fn)(u8 *r06, u8 *r07);

	if (!fn)
		fn = (int (*)(u8 *, u8 *))symbol_get(sc8547a_pps_regs);
	return fn ? fn(r06, r07) : -ENODEV;
}

/* optional "keep the pump ADC on for the session" (sc8547a module) */
static void pps_sc_adc_hold(bool on)
{
	static void (*fn)(bool on);

	if (!fn)
		fn = (void (*)(bool))symbol_get(sc8547a_pps_adc_hold);
	if (fn)
		fn(on);
}

/* ---- ADSP prop helpers ---- */
static int pps_usb_get(u32 prop)
{
	return erhai_otg_adsp_prop_get(PPS_USB_OP_GET, prop);
}

static int pps_usb_set(u32 prop, u32 value)
{
	return erhai_otg_adsp_prop_set(PPS_USB_OP_SET, prop, value);
}

static int pps_batt_set(u32 prop, u32 value)
{
	return erhai_otg_adsp_prop_set(PPS_BATT_OP_SET, prop, value);
}

static void pps_prop_err(struct erhai_pps *p, const char *what, int rc)
{
	pr_warn("erhai_pps: %s failed rc=%d (state %s); safe exit to 5 V\n",
		what, rc, pps_state_names[p->state]);
}

/* ---- pack reads: prefer the real pack gauge, fall back to the ADSP battery ---- */
static struct power_supply *pps_bat_psy(void)
{
	struct power_supply *psy = power_supply_get_by_name("bq27z561-0");

	if (!psy)
		psy = power_supply_get_by_name("erhai-adsp-bat");
	return psy;
}

/* the value goes through *out: a discharging pack reads a negative
 * current_now, which is a reading, not an errno (lab c3 run 1) */
static int pps_psy_int(struct power_supply *psy, enum power_supply_property prop,
		       int *out)
{
	union power_supply_propval val = { };
	int rc;

	rc = power_supply_get_property(psy, prop, &val);
	if (!rc)
		*out = val.intval;
	return rc;
}

/* fills p->vbat_mv / p->ibat_ma / p->soc / p->temp_c; returns 0 on success */
static int pps_read_battery(struct erhai_pps *p)
{
	struct power_supply *psy = pps_bat_psy();
	int v, i, s, t;

	if (!psy)
		return -ENODEV;

	if (pps_psy_int(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &v) ||
	    pps_psy_int(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &i) ||
	    pps_psy_int(psy, POWER_SUPPLY_PROP_CAPACITY, &s) ||
	    pps_psy_int(psy, POWER_SUPPLY_PROP_TEMP, &t)) {
		power_supply_put(psy);
		return -EIO;
	}
	power_supply_put(psy);

	/* sysfs battery: voltage_now uV -> mV; current_now uA -> mA; temp 0.1C -> C */
	p->vbat_mv = v / 1000;
	p->ibat_ma = i / 1000;
	p->soc = s;
	p->temp_c = t / 10;
	return 0;
}

/* reads VBUS through the ADSP (USB_VOLT_NOW in uV) */
static int pps_read_vbus(struct erhai_pps *p)
{
	int v = pps_usb_get(1);			/* USB_VOLT_NOW, uV */

	if (v < 0)
		return v;
	p->vbus_mv = v / 1000;
	return 0;
}

static bool pps_deployable(struct erhai_pps *p, bool *vooc_engaged)
{
	int real, pps_type, fct;

	*vooc_engaged = false;

	real = pps_usb_get(PPS_PROP_USB_REAL_TYPE);
	pps_type = pps_usb_get(PPS_PROP_USB_GET_PPS_TYPE);
	fct = pps_usb_get(PPS_PROP_USB_VOOC_FAST_CHG_TYPE);

	if (real != PPS_USB_TYPE_PD_PPS || pps_type != 1)
		return false;
	if (fct >= (int)pps_vooc_min || pps_sc_delivering() ||
	    pps_sc_identifying()) {
		*vooc_engaged = true;
		return false;
	}
	if (pps_read_battery(p))
		return false;
	/* Hysteresis around the stop threshold: a session that ended on
	 * soc-limit stays down until the pack falls below pps_soc_restart,
	 * otherwise the engine re-arms on the very next poll and oscillates
	 * at the boundary. */
	if (p->soc_hold) {
		if (p->soc > (int)pps_soc_restart)
			return false;
		p->soc_hold = false;
	}
	if (p->soc >= (int)pps_soc_stop)
		return false;
	if (p->vbat_mv < (int)pps_vbat_min_mv ||
	    p->vbat_mv > (int)pps_vbat_max_mv)
		return false;
	if (p->temp_c < pps_temp_min_c || p->temp_c > pps_temp_max_c)
		return false;
	return true;
}


/* ---- the 5 V revert ---- */
static void pps_revert(struct erhai_pps *p)
{
	int rc;

	/* Order matters: current+voltage back to 5 V first, then limits. */
	rc = pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, 2000);
	if (rc)
		pps_prop_err(p, "revert PPS_CURR", rc);
	rc = pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, 5000);
	if (rc)
		pps_prop_err(p, "revert PPS_VOLT", rc);
	if (pps_cp_init) {
		rc = pps_usb_set(PPS_PROP_USB_PPS_MOS_CTRL, 0);
		if (rc)
			pps_prop_err(p, "revert MOS_CTRL", rc);
	}
	/* Release the profile gate; keep charging enabled for normal 5 V. */
	rc = pps_batt_set(PPS_PROP_BATT_AP_FASTCHG_ALLOW, 0);
	if (rc)
		pps_prop_err(p, "revert FASTCHG_ALLOW", rc);
	rc = pps_batt_set(PPS_PROP_BATT_CHG_CTRL_LIM, 0);
	if (rc)
		pps_prop_err(p, "revert CHG_CTRL_LIM", rc);
	/* re-enable charging only when no policy holds the pack (bypass
	 * switch, max_soc): this write must not undo the user's inhibit */
	rc = pps_batt_set(PPS_PROP_BATT_CHG_EN, erhai_otg_charge_inhibited() ? 0 : 1);
	if (rc)
		pps_prop_err(p, "revert CHG_EN", rc);
	/* unsuspend, not a number: the ADSP takes 0 as "suspend input" and only
	 * the vendor's 0xFFFFFFFF releases it; a plain limit written after a
	 * suspend leaves the input suspended (lab, c4 build 7 bypass test) */
	rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT, PPS_ICL_UNSUSPEND);
	if (rc)
		pps_prop_err(p, "revert ICL", rc);
	rc = pps_usb_set(PPS_PROP_USB_PPS_VOOCPHY_ENABLE, 0);
	if (rc)
		pps_prop_err(p, "revert VOOCPHY", rc);

	p->set_volt_mv = 0;
	p->set_curr_ma = 0;
	p->stall_count = 0;
	p->buck_off = false;
	p->cp_lift_since = 0;
	/* the next attached poll re-runs the plug-in unsuspend */
	p->plug_logged = false;
}

/* ---- state transitions ---- */
static void pps_enter(struct erhai_pps *p, enum pps_state st, const char *why)
{
	if (p->state != st && st != PPS_EXIT)
		pr_info("erhai_pps: %s -> %s%s%s%s\n",
			pps_state_names[p->state], pps_state_names[st],
			why ? " (" : "", why ?: "", why ? ")" : "");
	p->state = st;
	p->state_since = jiffies;
	if (st == PPS_IDLE)
		p->last_reason = NULL;
	if (st == PPS_ARMED)
		p->armed_since = jiffies;
	if (st == PPS_RAMP)
		p->ramp_since = jiffies;
}

static void pps_exit(struct erhai_pps *p, const char *reason)
{
	if (p->state == PPS_IDLE || p->state == PPS_EXIT)
		return;
	p->last_reason = reason;
	p->exits++;
	p->charg_ticks = 0;
	pr_info("erhai_pps: exit: %s\n", reason);
	trace_erhai_pps_cp_event(reason, 0, -1, -1, -1, p->set_volt_mv);
	/* drop the charge pump BEFORE the 5 V revert: at 5 V the 2:1 window is
	 * gone and a latched CHG_EN has nothing to drive but also no watchdog. */
	if (pps_cp_is_engaged())
		pps_cp_drop();
	if (p->adc_held) {
		pps_sc_adc_hold(false);
		p->adc_held = false;
	}
	pps_revert(p);
	pps_enter(p, PPS_IDLE, NULL);
}

/* ---- pump controller (c3): the vendor recipe, docs/vendor-pps.md ---- */

/* PPS request as the vendor sends it: VOLT then CURR, always both
 * (oplus_chg_set_pps_config, adsp.c:10302-10303) */
static int pps_pdo_set(struct erhai_pps *p, unsigned int mv, unsigned int ma)
{
	int rc = pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, mv);

	if (rc)
		return rc;
	p->set_volt_mv = mv;
	rc = pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, ma);
	if (!rc)
		p->set_curr_ma = ma;
	return rc;
}

/* buck input suspend for the pump (vendor WIRED_CHARGE_SUSPEND -> ICL 0,
 * adsp.c:6294); resume writes the vendor's unsuspend value, 0xFFFFFFFF:
 * after a 0 the ADSP ignores an ordinary limit (lab, c4 build 7: input stuck
 * at 0.1 W after "ICL 3000000"; 0xFFFFFFFF released it at once) */
static int pps_buck_set(struct erhai_pps *p, bool off)
{
	int rc;

	if (p->buck_off == off)
		return 0;
	rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT,
			 off ? 0 : PPS_ICL_UNSUSPEND);
	if (rc) {
		pr_warn("erhai_pps: buck %s failed rc=%d\n",
			off ? "suspend" : "resume", rc);
		return rc;
	}
	p->buck_off = off;
	trace_erhai_pps_cp_event(off ? "buck-off" : "buck-on", 0, -1, -1, -1,
				 p->set_volt_mv);
	return 0;
}

/*
 * The guard: a pump that stops (drop, failed entry, taper, disabled) hands
 * the plug straight back to the buck in the same poll.  Counted stops start
 * the re-arm hold-off.  Non-zero only when the buck could not be resumed;
 * the caller then exits, and the revert writes the ICL again.
 */
static int pps_cp_release(struct erhai_pps *p, const char *why, bool count)
{
	if (pps_cp_is_engaged())
		pps_cp_drop();
	p->cp_lift_since = 0;
	if (count) {
		p->cp_rearms++;
		p->cp_holdoff_until = jiffies + msecs_to_jiffies(pps_cp_rearm_ms);
		pr_info("erhai_pps: CP %s - buck path (attempt %u/%u, next in %u ms)\n",
			why, p->cp_rearms, pps_cp_rearm_max, pps_cp_rearm_ms);
	}
	return pps_buck_set(p, false);
}

/*
 * One pump step per poll while CHARGING.  Not engaged: once the PPS current
 * is at its target (engaging during the ramp starves the pump, lab
 * Experiment 4), suspend the buck so the bus is unloaded, request
 * 2*vbat + pps_cp_entry_margin_mv, and engage when the chip window opens
 * (vendor pps.c:1881-2014).  Engaged: feed the watchdog and servo the PPS
 * voltage on the pump's own ibus, 40 mV at a time (vendor pps.c:3242-3361).
 * Returns non-zero when the engine must exit.
 */
static int pps_cp_step(struct erhai_pps *p)
{
	const struct erhai_pps_sc_api *a = pps_sc_cp_get();
	unsigned int target = p->target_curr_ma;
	int cvbus, cvbat, cibus, diff, rc, need = 0;
	unsigned int nv;

	if (!a)
		return 0;
	if (!p->adc_held) {
		pps_sc_adc_hold(true);
		p->adc_held = true;
	}
	if (a->snapshot(&cvbus, &cvbat, &cibus)) {
		/* blind: never leave a pump unfed or the buck suspended */
		if (a->cp_engaged() || p->cp_lift_since)
			return pps_cp_release(p, "adc read failed", true);
		return 0;
	}
	diff = cvbus - cvbat * 2;

	if (a->cp_engaged()) {
		rc = a->cp_feed(&cvbus, &cvbat);
		if (rc) {
			/* cvbus/cvbat are feed()'s own reads; ibus is from the
			 * snapshot just before it */
			pr_info("erhai_pps: CP feed drop rc=%d (diff %d mV ibus %d mA set %u mV)\n",
				rc, cvbus - cvbat * 2, cibus, p->set_volt_mv);
			trace_erhai_pps_cp_event("drop", rc, cvbus, cvbat, cibus,
						 p->set_volt_mv);
			p->cp_drops++;
			return pps_cp_release(p, "drop", true);
		}
		p->cp_fed_at = jiffies;
		/* switching but carrying nothing: with the buck suspended the
		 * pack would sit idle (vendor: 4 results under 300 mA) */
		if (cibus < (int)pps_cp_min_ibus_ma &&
		    time_after_eq(jiffies, p->cp_engaged_since +
				  msecs_to_jiffies(pps_cp_settle_ms))) {
			if (++p->cp_low_cnt >= 4)
				return pps_cp_release(p, "low ibus", true);
		} else {
			p->cp_low_cnt = 0;
		}
		nv = p->set_volt_mv;
		if (cibus > (int)(target + 500))
			nv -= pps_cp_vstep_over_mv;
		else if (cibus > (int)(target + pps_cp_ibus_band_ma) ||
			 p->ibat_ma > (int)pps_cp_ibat_max_ma)
			nv -= pps_cp_vstep_mv;
		else if (cibus < (int)target - (int)pps_cp_ibus_band_ma &&
			 diff < (int)pps_cp_diff_max_mv)
			nv += pps_cp_vstep_mv;
		nv = clamp_t(unsigned int, nv, cvbat * 2 + 100, pps_cp_max_volt_mv);
		if (nv != p->set_volt_mv && pps_pdo_set(p, nv, target))
			return -EIO;
		return 0;
	}

	if (p->cp_rearms >= pps_cp_rearm_max ||
	    time_before(jiffies, p->cp_holdoff_until) ||
	    p->set_curr_ma < target)
		return 0;

	if (!p->cp_lift_since) {
		if (pps_cp_buck_suspend && pps_buck_set(p, true)) {
			p->cp_rearms++;
			p->cp_holdoff_until = jiffies +
					      msecs_to_jiffies(pps_cp_rearm_ms);
			return 0;
		}
		nv = min_t(unsigned int,
			   roundup(cvbat * 2 + pps_cp_entry_margin_mv, 20),
			   pps_cp_max_volt_mv);
		p->cp_lift_since = jiffies | 1;
		pr_info("erhai_pps: CP entry: buck %s, request %u mV at %u mA (chip vbat %d mV diff %d mV)\n",
			p->buck_off ? "suspended" : "on", nv, target, cvbat, diff);
		return pps_pdo_set(p, nv, target);
	}

	rc = a->cp_engage(&need);
	if (rc == 0) {
		p->cp_lift_since = 0;
		p->cp_engaged_since = jiffies;
		p->cp_fed_at = jiffies;
		p->cp_low_cnt = 0;
		p->cp_engages++;
		pr_info("erhai_pps: CP 2:1 ENGAGED (chip vbus %d mV vbat %d mV diff %d mV ibus %d mA set %u mV)\n",
			cvbus, cvbat, diff, cibus, p->set_volt_mv);
		trace_erhai_pps_cp_event("engage", 0, cvbus, cvbat, cibus,
					 p->set_volt_mv);
		return 0;
	}
	/*
	 * Not in the pump's window yet: walk the request toward it, as the
	 * vendor does (pps.c:2020-2035), instead of waiting on one shot.  The
	 * unloaded bus overshoots the request by up to ~250 mV (c4 build 7:
	 * diff 708-783 mV for a 2*vbat + 450 request, 5 timeouts in a row).
	 */
	if (rc == -EAGAIN) {
		nv = p->set_volt_mv;
		if (diff > (int)pps_cp_diff_max_mv)
			nv -= pps_cp_vstep_over_mv;
		else if (diff < (int)pps_cp_entry_margin_mv / 2)
			nv += pps_cp_vstep_over_mv;
		nv = clamp_t(unsigned int, nv, cvbat * 2 + 100, pps_cp_max_volt_mv);
		if (nv != p->set_volt_mv && pps_pdo_set(p, nv, target))
			return -EIO;
	}
	if (time_after_eq(jiffies, p->cp_lift_since +
			  msecs_to_jiffies(pps_cp_lift_ms))) {
		pr_info("erhai_pps: CP entry timed out (diff %d mV, engage rc=%d)\n",
			diff, rc);
		return pps_cp_release(p, "entry timeout", true);
	}
	return 0;
}


/* ---- the poll ---- */
static void pps_poll(struct erhai_pps *p)
{
	int online, rc;

	if (!pps_enable)
		return;

	online = pps_usb_get(0);	/* USB_ONLINE */
	if (pps_read_vbus(p) < 0)
		online = -EINVAL;	/* ADSP not up yet -> treat as offline */

	switch (p->state) {
	case PPS_IDLE: {
		bool vooc;

		if (online <= 0) {
			p->plug_logged = false;
			return;
		}
		if (!p->plug_logged) {
			int icl = pps_usb_get(PPS_PROP_USB_INPUT_CURR_LIMIT);

			p->plug_logged = true;
			/*
			 * Every plug-in starts unsuspended: the revert at a
			 * detach runs after the cable is gone, and the ADSP appears
			 * to keep the last input limit it took while attached (0
			 * when a session ends with the pump running; "icl was" below
			 * shows what it held).  Left alone, the
			 * next fixed-PD/5 V charger never charges (lab c4 dock
			 * check: pack at -0.7..-1.1 A on the dock).
			 */
			rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT,
					 PPS_ICL_UNSUSPEND);
			pr_info("erhai_pps: adapter online: real_type %d pps_type %d pps_max_curr %d vbus %d mV (icl was %d, unsuspend rc=%d)\n",
				pps_usb_get(PPS_PROP_USB_REAL_TYPE),
				pps_usb_get(PPS_PROP_USB_GET_PPS_TYPE),
				pps_usb_get(PPS_PROP_USB_GET_PPS_MAX_CURR),
				p->vbus_mv, icl, rc);
		}
		/*
		 * Early after boot the ADSP property service may still be
		 * coming up: the first read returns garbage (real_type 6,
		 * pps_type 0) and the glink queue then stalls behind the
		 * audio graph start, so the state never changes on its own.
		 * Re-read every pps_reprobe_ms until the adapter reports as
		 * deployable instead of waiting forever for a property event.
		 */
		if (!pps_deployable(p, &vooc)) {
			if (!p->plug_logged ||
			    time_before(jiffies, p->last_reprobe +
					msecs_to_jiffies(pps_reprobe_ms)))
				return;
			p->last_reprobe = jiffies;
			p->plug_logged = false;	/* force a fresh probe log */
			return;
		}
		pps_enter(p, PPS_ARMED, "PD_PPS adapter");
		return;
	}
	case PPS_ARMED: {
		bool vooc;

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		if (pps_deployable(p, &vooc) && !vooc) {
			if (time_after_eq(jiffies, p->armed_since +
					  msecs_to_jiffies(pps_identify_ms)))
				pps_enter(p, PPS_NEGOTIATE, NULL);
			return;
		}
		if (vooc) {
			/* serial VOOC won: stand down for this plug */
			pps_exit(p, PPS_EXIT_REASON_VOOC_ENGAGED);
			return;
		}
		/* adapter vanished or no longer deployable -> back to idle.
		 * Go through pps_enter() like every other transition: the bare
		 * assignment skipped the transition log, left last_reason
		 * stale and never refreshed state_since, so the debugfs view
		 * disagreed with the engine's actual state. */
		pps_enter(p, PPS_IDLE, NULL);
		return;
	}
	case PPS_NEGOTIATE: {
		int adapter_ma;

		/*
		 * Entry sequence — ORDER IS LOAD-BEARING (live-validated 2026-09-24,
		 * 100 W Anker, b3 kernel): the ADSP charger_pd latches its PPS APDO
		 * from the first writes and grants the pack demand only after the
		 * battery profile gate is set.  The working order is:
		 *
		 *   49 PPS_EN -> 34 VOLT at TARGET (first!) -> 35 CURR -> 5 ICL,
		 * then the battery profile gate (46/24/10) and the CP PHY init
		 * (40/39) AFTER the PDO is established.
		 *
		 * Measured with the reverse/enhanced variants: CURR-before-VOLT plus
		 * either a 5.5 V seed+step-ramp (VBUS caps at ~6.64 V = ~9 W pack)
		 * or a one-shot 8.4 V request (VBUS 4.6 V); granting the gate before
		 * the PDO caps input at ~1.3 A.  The sequence below delivers
		 * 8.2-8.6 V x 3 A in ~= 24 W, pack 3.6-4 A and climbing.
		 */
		rc = pps_usb_set(PPS_PROP_USB_PPS_VOOCPHY_ENABLE,
				 pps_voocphy_enable ? 1 : 0);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, pps_target_volt_mv);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, pps_start_curr_ma);
		if (rc)
			goto fail;
		rc = pps_usb_set(PPS_PROP_USB_INPUT_CURR_LIMIT,
				 pps_iclim_ma * 1000);
		if (rc)
			goto fail;

		/* profile gate + CP PHY init AFTER the PDO (validated order) */
		rc = pps_batt_set(PPS_PROP_BATT_AP_FASTCHG_ALLOW, 1);
		if (rc)
			goto fail;
		rc = pps_batt_set(PPS_PROP_BATT_CHG_EN,
				  erhai_otg_charge_inhibited() ? 0 : 1);
		if (rc)
			goto fail;
		rc = pps_batt_set(PPS_PROP_BATT_CHG_CTRL_LIM, pps_fcc_ma * 1000);
		if (rc)
			goto fail;
		if (pps_cp_init) {
			rc = pps_usb_set(PPS_PROP_USB_PPS_CP_MODE_INIT, 1);
			if (rc)
				goto fail;
			rc = pps_usb_set(PPS_PROP_USB_PPS_MOS_CTRL, 1);
			if (rc)
				goto fail;
		}

		p->set_volt_mv = pps_target_volt_mv;
		p->set_curr_ma = pps_start_curr_ma;
		/* never ask for more than the adapter offers on this cable
		 * (vendor pps.c:1735-1774 caps by the adapter's max too) */
		adapter_ma = pps_usb_get(PPS_PROP_USB_GET_PPS_MAX_CURR);
		if (adapter_ma <= 0)
			adapter_ma = pps_cable_safe_ma;
		p->target_curr_ma = min3(pps_target_curr_ma, pps_max_curr_ma,
					 (unsigned int)adapter_ma);
		pr_info("erhai_pps: PPS current target %u mA (adapter offers %d mA)\n",
			p->target_curr_ma, adapter_ma);
		p->buck_off = false;
		p->cp_lift_since = 0;
		p->cp_holdoff_until = jiffies;
		p->cp_rearms = 0;
		p->sessions++;
		p->entries++;
		pps_enter(p, PPS_RAMP, NULL);
		return;

fail:
		pps_prop_err(p, "PPS ENTER", rc);
		pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
		return;
	}
	case PPS_RAMP: {
		unsigned int next, ci;

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		/* did we actually leave 5 V in time? */
		if (time_after_eq(jiffies, p->ramp_since +
				  msecs_to_jiffies(pps_poll_ms * 6)) &&
		    p->vbus_mv < (int)pps_start_volt_mv) {
			pps_exit(p, PPS_EXIT_REASON_RAMP_TIMEOUT);
			return;
		}
		if (p->vbus_mv < (int)pps_vbus_collapse_mv) {
			pps_exit(p, PPS_EXIT_REASON_COLLAPSE);
			return;
		}
		if (p->set_volt_mv >= pps_target_volt_mv) {
			pps_enter(p, PPS_CHARGING, "VBUS at target");
			return;
		}
		next = min(p->set_volt_mv + pps_volt_step_mv, pps_target_volt_mv);
		next = min(next, pps_max_volt_mv);
		if (pps_usb_set(PPS_PROP_USB_SET_PPS_VOLT, next)) {
			pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
			return;
		}
		/* keep the input current moving toward the target */
		if (p->set_curr_ma < p->target_curr_ma) {
			ci = min(p->set_curr_ma + 250U, p->target_curr_ma);
			if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, ci))
				p->set_curr_ma = ci;
		}
		p->set_volt_mv = next;
		pr_info("erhai_pps: ramp VBUS %u mV curr %u mA (read %d mV)\n",
			p->set_volt_mv, p->set_curr_ma, p->vbus_mv);
		return;
	}
	case PPS_CHARGING:
	case PPS_TAPER: {
		bool charging = (p->state == PPS_CHARGING);

		if (online <= 0) {
			pps_exit(p, PPS_EXIT_REASON_DETACH);
			return;
		}
		rc = pps_read_battery(p);
		if (rc) {
			/* blind: never sit with the buck suspended */
			if ((p->buck_off || pps_cp_is_engaged()) &&
			    pps_cp_release(p, "pack read failed", true))
				pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
			return;
		}

		/* safety exits first */
		if (p->vbat_mv < (int)pps_vbat_min_mv) {
			pps_exit(p, PPS_EXIT_REASON_VBAT_MIN);
			return;
		}
		if (p->soc >= (int)pps_soc_stop) {
			p->soc_hold = true;	/* latch: only pps_soc_restart clears it */
			pps_exit(p, PPS_EXIT_REASON_SOC);
			return;
		}
		if (p->temp_c < pps_temp_min_c || p->temp_c > pps_temp_max_c) {
			pps_exit(p, PPS_EXIT_REASON_TEMP);
			return;
		}
		if (p->vbus_mv < (int)pps_vbus_collapse_mv) {
			pps_exit(p, PPS_EXIT_REASON_COLLAPSE);
			return;
		}
		/* pack-stall watchdog: nothing flowing at high VBUS */
		if (pps_stall_polls && p->vbus_mv > 7000 &&
		    p->ibat_ma < 100 && !p->cp_lift_since) {
			if (++p->stall_count >= pps_stall_polls) {
				pps_exit(p, PPS_EXIT_REASON_STALL);
				return;
			}
		} else {
			p->stall_count = 0;
		}

		if (p->vbat_mv >= (int)pps_vbat_max_mv) {
			if (pps_cp_release(p, "taper", false)) {
				pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
				return;
			}
			if (charging)
				pps_enter(p, PPS_TAPER, "vbat at CV");
			/* taper: shed current in steps, then hand to 5 V */
			if (p->set_curr_ma > 500) {
				unsigned int n = max(500U,
						     p->set_curr_ma - 250U);

				if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, n))
					p->set_curr_ma = n;
			} else {
				pps_exit(p, PPS_EXIT_REASON_VBAT_MAX);
			}
			return;
		}

		/*
		 * current servo: the ADSP fills the pack up to the ICL/FCC
		 * grant; we nudge PPS curr to the input target, never yank.
		 */
		if (p->set_curr_ma < p->target_curr_ma) {
			unsigned int n = min(p->set_curr_ma + 250U,
					     p->target_curr_ma);

			if (!pps_usb_set(PPS_PROP_USB_SET_PPS_CURR, n))
				p->set_curr_ma = n;
		}

		/*
		 * CP 2:1 direct-charge: the vendor's PPS + virtual_cp recipe
		 * (pps_cp_step()).  The engine's collapse/soc/temp/vbat exits drop
		 * the pump via pps_exit BEFORE the 5 V revert.  Pump off at runtime
		 * (pps_cp_enable=0) or the API gone: hand back to the buck.
		 */
		if (erhai_otg_charge_inhibited()) {
			/* bypass / max_soc: the pump would charge straight
			 * through the ADSP inhibit; the buck obeys it */
			if (!p->inhibit_held)
				pr_info("erhai_pps: charging inhibited by policy - pump off, buck holds the pack\n");
			p->inhibit_held = true;
			rc = pps_cp_release(p, "inhibited", false);
		} else if (pps_cp_enable) {
			if (p->inhibit_held)
				pr_info("erhai_pps: charge inhibit released - pump allowed\n");
			p->inhibit_held = false;
			rc = pps_cp_step(p);
		} else {
			rc = pps_cp_release(p, "disabled", false);
		}
		if (rc) {
			pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
			return;
		}
		p->charg_ticks++;
		if (p->charg_ticks < pps_log_ticks)
			pr_info("erhai_pps: %s vbus %d mV vbat %d mV ibat %d mA soc %d%% t %dC set %u mV curr %u mA cp %s buck %s\n",
				charging ? "charging" : "taper",
				p->vbus_mv, p->vbat_mv, p->ibat_ma, p->soc,
				p->temp_c, p->set_volt_mv, p->set_curr_ma,
				pps_cp_is_engaged() ? "on" : "off",
				p->buck_off ? "off" : "on");
		return;
	}
	default:
		return;
	}
}

/*
 * The guard's backstop, run after every poll whatever path pps_poll() took:
 * a suspended buck must always have an engaged pump that was fed within the
 * 1000 ms watchdog, or be inside the entry window.  Otherwise hand the plug
 * back to the buck now (c3 run 1: an early return left the buck suspended
 * with no pump for 5.5 minutes).
 */
static void pps_cp_backstop(struct erhai_pps *p)
{
	bool stuck;

	if (!p->buck_off)
		return;
	if (pps_cp_is_engaged())
		stuck = time_after(jiffies, p->cp_fed_at + msecs_to_jiffies(1000));
	else
		stuck = !p->cp_lift_since ||
			time_after(jiffies, p->cp_lift_since +
				   msecs_to_jiffies(pps_cp_lift_ms + 1000));
	if (!stuck)
		return;
	pr_warn("erhai_pps: CP backstop: buck suspended without a working pump - resuming the buck\n");
	if (pps_cp_release(p, "backstop", true))
		pps_exit(p, PPS_EXIT_REASON_SET_FAIL);
}

/* one tracepoint sample per poll while a session is up; chip reads only
 * happen when the event is enabled */
static void pps_trace_sample(struct erhai_pps *p)
{
	const struct erhai_pps_sc_api *a;
	struct erhai_pps_trace_sample s = {
		.state		= pps_state_names[p->state],
		.adsp_vbus_mv	= p->vbus_mv,
		.set_volt_mv	= p->set_volt_mv,
		.set_curr_ma	= p->set_curr_ma,
		.vbat_mv	= p->vbat_mv,
		.ibat_ma	= p->ibat_ma,
		.soc		= p->soc,
		.temp_c		= p->temp_c,
		.buck_off	= p->buck_off,
		.chip_vbus_mv	= -1,
		.chip_vbat_mv	= -1,
		.chip_ibus_ma	= -1,
		.r06		= -1,
		.r07		= -1,
	};
	u8 r06, r07;

	if (!trace_erhai_pps_sample_enabled() || p->state == PPS_IDLE)
		return;
	a = pps_sc_cp_get();
	if (a) {
		s.cp = a->cp_engaged();
		if (a->snapshot(&s.chip_vbus_mv, &s.chip_vbat_mv,
				&s.chip_ibus_ma))
			s.chip_vbus_mv = s.chip_vbat_mv = s.chip_ibus_ma = -1;
		if (!pps_sc_regs(&r06, &r07)) {
			s.r06 = r06;
			s.r07 = r07;
		}
	}
	trace_erhai_pps_sample(&s);
}

static void pps_ws(struct work_struct *work)
{
	struct erhai_pps *p = container_of(to_delayed_work(work),
					   struct erhai_pps, work);
	bool fast;

	mutex_lock(&pps_lock);
	pps_poll(p);
	pps_cp_backstop(p);
	pps_trace_sample(p);
	fast = pps_cp_is_engaged() || p->cp_lift_since;
	mutex_unlock(&pps_lock);
	/* while the charge pump is engaged (or being entered), poll at the
	 * REG_09 watchdog feed interval (must stay under the SC8547A 1000 ms
	 * window); otherwise the normal telemetry cadence is fine.  A dropped
	 * poll = a watchdog expiry self-clears REG_07, so this can only slow the
	 * CP down, never strand it. */
	schedule_delayed_work(&p->work,
			      msecs_to_jiffies(fast ? pps_cp_feed_ms : pps_poll_ms));
}


/* ---- debugfs ---- */
static int pps_state_show(struct seq_file *s, void *v)
{
	struct erhai_pps *p = s->private;
	const char *cp_state;
	bool buck_off;

	/* pps_cp_is_engaged() lazily resolves the charge-pump symbols; take
	 * the same lock the worker uses so the one-time init cannot race. */
	mutex_lock(&pps_lock);
	cp_state = pps_cp_is_engaged() ? "on" : "off";
	buck_off = p->buck_off;
	mutex_unlock(&pps_lock);

	seq_printf(s, "pps_enable %d\nstate %s\nlast_exit %s\n",
		   pps_enable, pps_state_names[p->state],
		   p->last_reason ?: "-");
	seq_printf(s, "set volt %u mV curr %u mA target %u mA\n",
		   p->set_volt_mv, p->set_curr_ma, p->target_curr_ma);
	if (p->state > PPS_IDLE)
		seq_printf(s, "read vbus %d mV vbat %d mV ibat %d mA soc %d%% t %d C\n",
			   p->vbus_mv, p->vbat_mv, p->ibat_ma, p->soc,
			   p->temp_c);
	seq_printf(s, "sessions %llu entries %u exits %u cp %s (eng %u drop %u attempts %u) buck %s\n",
		   p->sessions, p->entries, p->exits,
		   cp_state,
		   p->cp_engages, p->cp_drops, p->cp_rearms,
		   buck_off ? "suspended" : "on");
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pps_state);

static ssize_t pps_cmd_write(struct file *f, const char __user *ubuf,
			     size_t count, loff_t *off)
{
	struct erhai_pps *p = f->private_data;
	char buf[32];
	ssize_t n;

	n = simple_write_to_buffer(buf, sizeof(buf) - 1, off, ubuf, count);
	if (n < 0)
		return n;
	buf[n] = '\0';

	mutex_lock(&pps_lock);
	if (strncmp(buf, "start", 5) == 0) {
		if (p->state == PPS_IDLE)
			pps_enter(p, PPS_ARMED, "manual");
	} else if (strncmp(buf, "stop", 4) == 0) {
		pps_exit(p, PPS_EXIT_REASON_FORCED);
	}
	mutex_unlock(&pps_lock);
	return n;
}

static const struct file_operations pps_cmd_fops = {
	.open	= simple_open,
	.write	= pps_cmd_write,
	.llseek	= noop_llseek,
};

static int __init erhai_pps_init(void)
{
	struct erhai_pps *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->state = PPS_IDLE;
	INIT_DELAYED_WORK(&p->work, pps_ws);
	g_pps = p;

	p->debugfs = debugfs_create_dir("erhai_pps", NULL);
	debugfs_create_file("state", 0444, p->debugfs, p, &pps_state_fops);
	debugfs_create_file("cmd", 0200, p->debugfs, p, &pps_cmd_fops);
	/* the engine polls even before the ADSP is up (returns cleanly) */
	schedule_delayed_work(&p->work, msecs_to_jiffies(5000));

	pr_info("erhai_pps: PPS host engine loaded (poll %u ms, target %u mV/%u mA, FCC %u mA, soc<=%u%%)\n",
		pps_poll_ms, pps_target_volt_mv, pps_target_curr_ma, pps_fcc_ma,
		pps_soc_stop);
	return 0;
}

static void __exit erhai_pps_exit(void)
{
	struct erhai_pps *p = g_pps;

	if (!p)
		return;
	/* Cancel before taking the lock: pps_ws() takes pps_lock on entry, so
	 * holding it across cancel_delayed_work_sync() deadlocks whenever the
	 * work is already running. */
	cancel_delayed_work_sync(&p->work);
	mutex_lock(&pps_lock);
	if (p->state != PPS_IDLE) {
		pps_cp_drop();
		pps_revert(p);
	}
	if (p->adc_held)
		pps_sc_adc_hold(false);
	mutex_unlock(&pps_lock);
	debugfs_remove_recursive(p->debugfs);
	kfree(p);
	g_pps = NULL;
}

module_init(erhai_pps_init);
module_exit(erhai_pps_exit);

MODULE_DESCRIPTION("erhai (OnePlus Pad 3) ADSP PPS fast-charge host engine");
MODULE_LICENSE("GPL");

