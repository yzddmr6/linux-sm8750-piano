// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xiaomi MCA charger firmware interface over Qualcomm PMIC GLINK
 *
 * Some Xiaomi devices, such as the Xiaomi Pad 8 Pro, run Xiaomi's charger
 * firmware in the charger PD of the ADSP instead of the Qualcomm battery
 * manager. It listens on the battery manager GLINK owner, but speaks the
 * property protocol of Xiaomi's "MCA" charging stack: a request names one
 * property and carries a sequence number, the reply echoes both and adds a
 * firmware return code.
 *
 * Standard power_supply telemetry and ICL/FCC writes retain their units,
 * checked replies/readback and unknown-source refusal. The complete community
 * charging policy is opt-in: source-bounded 9 V/PPS, battery-temperature FCC,
 * in-kernel MiPPS, SC8541 direct charging and temperature-bounded float voltage.
 * Optional battery thresholds use the stock charge-enable command. Firmware
 * JEITA is never disabled. All policy switches and thresholds default off.
 * USB-C port acknowledgments belong to the existing AltMode client.
 */

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/iopoll.h>
#include <linux/iio/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/hex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/ktime.h>
#include <linux/power/sc8541_charger.h>
#include <linux/mutex.h>
#include <linux/regulator/driver.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>
#include <crypto/sha2.h>

#include "xiaomi_mca_mipps.h"

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
#include <kunit/static_stub.h>
#endif

#define MCA_OPCODE_READ			1
#define MCA_OPCODE_WRITE		2

/* Charger (buck) properties, currents in mA */
#define MCA_PROP_CHGR_STATUS		0x2001
#define MCA_PROP_CHARGING_ENABLED	0x2003
#define MCA_PROP_CHARGE_CURRENT		0x2005
#define MCA_PROP_TERM_CURRENT		0x2006
#define MCA_PROP_PACK_VBAT		0x2026
#define MCA_PROP_TERM_VOLT		0x2007
#define MCA_PROP_INPUT_CURRENT_LIMIT	0x2008
#define MCA_PROP_AICL_ENABLED		0x2012
#define MCA_PROP_INPUT_SUSPEND		0x2019
#define MCA_PROP_OTG_BOOST_STATUS	0x200d
#define MCA_PROP_OTG_GATE_STATUS		0x200e
#define MCA_PROP_OTG_CFG			0x201a
#define MCA_PROP_CID_CFG			0x201b
#define MCA_PROP_CP_STATE		0x201c
#define MCA_PROP_CID_STATUS		0x2101f
#define MCA_PROP_OTG_PLUGIN		0x21020
#define MCA_PROP_CC_TOGGLE		0x21021
#define MCA_PROP_SNK_SRC_MODE		0x21022
#define MCA_PROP_TYPEC_MODE		0x21015
#define MCA_NOTIFY_TYPE_CHANGE		0
#define MCA_NOTIFY_BOOST			3
#define MCA_NOTIFY_TYPEC_CHANGE		5
#define MCA_NOTIFY_OTG_CP_CONFIG		10
#define MCA_NOTIFY_HISTORY		32

/* Piano's charger boost and PMIC GPIO controller, with no external GPIOs. */
static const __le32 xiaomi_mca_piano_otg_cfg[] = {
	cpu_to_le32(0), cpu_to_le32(1), cpu_to_le32(~0U), cpu_to_le32(~0U),
};

/*
 * Piano stock cid_gpio_int, passed unchanged by qcom_subpmic_probe()
 * (0x59c..0x5b0) and sync_cfg_work() (0xba0..0xbb4) to CID_CFG.
 * ADSP mca_misc_cid_cfg_init/register_cid_irq consumes this firmware
 * identifier. It is not an AP GPIO/IRQ number; do not translate it.
 */
#define MCA_PIANO_CID_CFG		0x708

#define MCA_PROP_AICL_STATUS		0x2029
/* USB input properties, voltages in uV, currents in uA */
#define MCA_PROP_USB_ONLINE		0x20001
#define MCA_PROP_USB_VOLTAGE_NOW	0x20002
#define MCA_PROP_USB_CURRENT_NOW	0x20004
#define MCA_PROP_USB_REAL_TYPE		0x2000c

/* USB real type: enum xm_charger_type of Xiaomi's downstream MCA stack */
#define MCA_USB_REAL_TYPE_SDP		1
#define MCA_USB_REAL_TYPE_CDP		2
#define MCA_USB_REAL_TYPE_DCP		3
#define MCA_USB_REAL_TYPE_FLOAT		4
#define MCA_USB_REAL_TYPE_HVDCP2	5
#define MCA_USB_REAL_TYPE_HVDCP3	6
#define MCA_USB_REAL_TYPE_HVDCP3_B	7
#define MCA_USB_REAL_TYPE_HVDCP3P5	8
#define MCA_USB_REAL_TYPE_TYPEC		9
#define MCA_USB_REAL_TYPE_PD		10
#define MCA_USB_REAL_TYPE_PD_VERIFY	11
#define MCA_USB_REAL_TYPE_PPS		12
#define MCA_USB_REAL_TYPE_OCP		15

/*
 * The lowest value written. The firmware starts with an input current limit
 * of 100 mA, and the downstream stack puts 100 mA back when the source goes
 * away (strategy_buckchg_reset_charge_para()); it also votes 100 mA for the
 * charge current there.
 */
#define MCA_CURRENT_MIN_MA		100
#define MCA_ICL_DEFAULT_MA		100

#define MCA_DATA_LEN			256
#define MCA_REQUEST_TIMEOUT		HZ
/* Userspace reads every attribute in turn; one firmware read per second will do. */
#define MCA_CACHE_TIME			HZ
#define MCA_POLL_INTERVAL		(2 * HZ)

/*
 * Charging policy and MiPPS relay ported function-for-function from
 * blu-sharky/linux-piano, piano_mca.c at 7a33c60fd6ed (GPL-2.0-only),
 * bluseliu50 <265648257+bluseliu50@users.noreply.github.com>:
 * 30e17de8bd23, 7bdefed13158, eae63ac2f462, bfb6b3833857,
 * 3fff8d02f29b, 42329af82bec, afdb026aed59, 7a33c60fd6ed.
 * The original checked transport, power_supply ABI and unknown-source
 * refusal remain. Port ACKs use pmic_glink_altmode, the sole PAN owner.
 * Board ceilings were checked against this unit's OS3.0.303 stock FDT;
 * the community policy's narrower operating windows remain conservative
 * software limits, not a claim of complete stock JEITA/FFC support.
 */
/* Property ids recovered from the stock charger stack */
#define MCA_PROP_PACK_TBAT	0x202a	/* degC */

/* Up to seven source PDOs, as decoded by the ADSP */
#define MCA_PROP_PDOS		0x21011	/* struct xiaomi_mca_pdo[] */
/* Piano mca_qc_quick_charge_regulation and qcom_subpmic_set_qc_volt_cmd.
 * This firmware uses 0x20/0x10, unlike the Onyx enum (1/2).
 */
#define MCA_PROP_AICL_ENABLE	0x2012	/* u8 */
#define MCA_PROP_RERUN_AICL	0x2013	/* u8 */
#define MCA_PROP_QC_PULSE	0x201d
#define MCA_QC_UP		0x20
#define MCA_QC_DOWN		0x10
#define MCA_PROP_QC_VOLT	0x2011	/* HVDCP voltage request, little-endian mV */
#define MCA_PROP_PD_FIXED_VOLT	0x21018	/* request a fixed PDO, mV */
#define MCA_PROP_PPS_SELECT	0x21017	/* request a PPS APDO, see below */
#define MCA_PROP_VERIFY_PROCESS	0x21002	/* non-zero during adapter auth */
#define MCA_PD_MAX_PDOS		7

struct xiaomi_mca_pdo {
	__le32 min_mv;
	__le32 max_mv;
	__le32 max_ma;
};

/*
 * Xiaomi charger authentication (MiPPS): the ADSP exchanges the PD
 * unstructured VDMs with the adapter; the kernel computes the digests.
 * Command n (1-8) goes to property 0x21003 + n - 1, as in the stock
 * adsp_pd_protocol_request_vdm_cmd().
 */
#define MCA_PROP_VDM_VSAFE0V	0x2100b
#define MCA_PROP_VDM_BASE	0x21003
#define MCA_PROP_VDM_VERSION	0x21003
#define MCA_PROP_VDM_VOLTAGE	0x21004
#define MCA_PROP_VDM_TEMP	0x21005
#define MCA_PROP_VDM_SEED	0x21006
#define MCA_PROP_VDM_AUTH	0x21007
#define MCA_PROP_VDM_VERIFIED	0x21008
#define MCA_PROP_VDM_REMOVE_COMP 0x21009
#define MCA_PROP_VDM_REVERSE	0x2100a
#define MCA_PROP_DATA_ROLE	0x2100c	/* 1 UFP, 2 DFP; write asks a swap */
#define MCA_PROP_PD_STATE	0x2100d
#define MCA_PD_SNK_READY		2
#define MCA_PROP_ADAPTER_ID	0x2100e
#define MCA_PROP_ADAPTER_SVID	0x2100f
#define MCA_PROP_PD_VERIFIED	0x21010
#define MCA_PROP_UVDM_STATE	0x21012
#define MCA_VDM_LEN		16
#define MCA_DATA_ROLE_UFP	1
#define MCA_DATA_ROLE_DFP	2

/*
 * Charger types as named by the stock sysfs "real_type" table; 13 and 16
 * are unnamed there, 17-19 are wireless.
 */
enum mca_real_type {
	MCA_TYPE_UNKNOWN,
	MCA_TYPE_SDP,
	MCA_TYPE_CDP,
	MCA_TYPE_DCP,
	MCA_TYPE_FLOAT,
	MCA_TYPE_HVDCP,
	MCA_TYPE_HVDCP_3,
	MCA_TYPE_HVDCP_3_B,
	MCA_TYPE_HVDCP_3P5,
	MCA_TYPE_C,
	MCA_TYPE_PD,
	MCA_TYPE_PD_VERIFY,
	MCA_TYPE_PD_PPS,
	MCA_TYPE_ACA = 14,
	MCA_TYPE_DCP_B,
	MCA_TYPE_BPP = 17,
	MCA_TYPE_EPP,
	MCA_TYPE_HPP,
	MCA_TYPE_MAX = 20,
};

static const char * const mca_real_type_names[MCA_TYPE_MAX] = {
	[MCA_TYPE_UNKNOWN]	= "Unknown",
	[MCA_TYPE_SDP]		= "SDP",
	[MCA_TYPE_CDP]		= "CDP",
	[MCA_TYPE_DCP]		= "DCP",
	[MCA_TYPE_FLOAT]	= "USB_FLOAT",
	[MCA_TYPE_HVDCP]	= "HVDCP",
	[MCA_TYPE_HVDCP_3]	= "HVDCP_3",
	[MCA_TYPE_HVDCP_3_B]	= "HVDCP_3_B",
	[MCA_TYPE_HVDCP_3P5]	= "HVDCP_3P5",
	[MCA_TYPE_C]		= "C",
	[MCA_TYPE_PD]		= "PD",
	[MCA_TYPE_PD_VERIFY]	= "PD_PPS",
	[MCA_TYPE_PD_PPS]	= "PD_PPS",
	[MCA_TYPE_ACA]		= "ACA",
	[MCA_TYPE_DCP_B]	= "DCP",
	[MCA_TYPE_BPP]		= "BPP",
	[MCA_TYPE_EPP]		= "EPP",
	[MCA_TYPE_HPP]		= "HPP",
};

/* VBUS above this means a source is attached (USB default is 5 V) */
#define MCA_VBUS_ONLINE_UV	4000000
#define MCA_POLL_MS		3000

/* Input current limits (mA) set by charge_policy, from the stock tables */
#define MCA_ICL_MIN_MA		100	/* also the ADSP power-on value */
#define MCA_ICL_5V_MAX_MA	3000	/* Piano ADSP fixed 5 V sink PDO */
#define MCA_ICL_HV_MAX_MA	2500	/* at 9 V, see xiaomi_mca_hv_icl() */
#define MCA_POLICY_ICL_DEFAULT_MA	500
#define MCA_UNKNOWN_POLLS		3
#define MCA_AICL_WATCH_MS		10000
#define MCA_FLOAT_ICL_MA		1000	/* Piano in_float */

static bool charge_policy = true;
module_param(charge_policy, bool, 0644);
MODULE_PARM_DESC(charge_policy,
		 "Raise the USB input current limit by charger type (default: on)");

/*
 * charge_current: left alone, the ADSP charges the battery at about 500 mA
 * whatever the source.  Stock sets the charge current by charger type (its
 * buck strategy chg_* values) and a JEITA table that allows far more than
 * the ceiling below between 13 and 48 degC; this sets the charger-type value,
 * capped.  Temperature only ever moves the charge current, never the input
 * limit or the bus voltage, so the system keeps running from the source:
 * outside the normal cell window the buck uses the fallback current. The
 * ADSP's own JEITA window (0-55 degC) still applies.  Detach, unbind and
 * ADSP restarts put the fallback value back.
 */
#define MCA_FCC_MIN_MA		100
#define MCA_FCC_MAX_MA		3600	/* maximum of the per-source buck profiles */
#define MCA_FCC_DEFAULT_MA	500
#define MCA_TERM_COLD_MV	4500	/* CN buck JEITA below 18 C */

static bool charge_current = true;
module_param(charge_current, bool, 0644);
MODULE_PARM_DESC(charge_current,
		 "Raise the battery charge current by charger type; needs charge_policy (default: on)");

/*
 * full_charge: use the stock normal float voltage, lowered at 48 degC with
 * 2 degC hysteresis. Missing battery telemetry selects the hot limit. The
 * firmware still stops charging outside its own 0-55 degC JEITA window.
 */
#define MCA_TERM_DEFAULT_MV	4400
#define MCA_TERM_FULL_MV	4480
#define MCA_TERM_FFC_MV		4540	/* batt_para_lwn final fast-charge step */
#define MCA_TERM_HOT_MV		4060
#define MCA_TERM_HOT_DEGC	48
#define MCA_TERM_COOL_DEGC	46

static bool full_charge = true;
module_param(full_charge, bool, 0644);
MODULE_PARM_DESC(full_charge,
		 "Use the stock full-charge voltage with thermal derating; needs charge_policy (default: on)");

/*
 * hv_charge: a PD source with a fixed 9 V PDO is asked for 9 V, as the
 * stock buck strategy does, whatever the battery temperature (that is left
 * to the charge current).  A source without one whose PPS APDO covers 9 V
 * (some power banks list only a 5-11 V APDO) is asked for 9.0 V at its
 * APDO current, capped at MCA_HV_PPS_MAX_MA; the ADSP keeps the PPS
 * contract alive by itself.  The bus has to settle within the window below in
 * a few polls and stay there. A rejected request is retried from 5 V.
 */
#define MCA_HV_MV		9000
#define MCA_HV_VBUS_MIN_UV	8000000
#define MCA_HV_VBUS_MAX_UV	9600000
#define MCA_HV_SETTLE_POLLS	3
/* A source that drops the bus while 9 V is pending is left alone this long */
#define MCA_HV_RETRY_MS		60000
/*
 * A source that goes back to 5 V by itself once 9 V was reached (a Xiaomi
 * charger does after the adapter authentication) is asked again after a
 * pause.
 */
#define MCA_HV_VBUS_5V_MAX_UV	6000000
#define MCA_HV_RENEG_MS		10000
/* Current limit asked of a PPS source */
#define MCA_HV_PPS_MIN_MA	500
#define MCA_HV_PPS_MAX_MA	3000

static bool hv_charge = true;
module_param(hv_charge, bool, 0644);
MODULE_PARM_DESC(hv_charge,
		 "Ask PD sources with a fixed 9 V PDO for 9 V; needs charge_policy (default: on)");


static bool mipps_auth = true;
module_param(mipps_auth, bool, 0644);
MODULE_PARM_DESC(mipps_auth,
		 "Authenticate Xiaomi MiPPS adapters before PPS negotiation (default: on)");

/*
 * cp_charge: direct charging through the SC8541 2:1 switched-capacitor
 * stage (driver sc8541_charger), which is how stock charges fast
 * (mca_strategy_quickchg): a PPS source is held at about twice the battery
 * voltage and the stage halves it into the battery at twice the bus
 * current, while the buck charger is held at MCA_CP_BUCK_MA.  The sequence
 * follows stock: ask the source for twice the battery voltage plus
 * MCA_CP_DELTA_MV, enable the stage, raise the voltage MCA_CP_STEP_MV at a
 * time until current flows, then regulate the voltage every MCA_CP_POLL_MS
 * so the bus current stays at its target and the battery current (both
 * gauges) below the limit of xiaomi_mca_cp_ibat_limit().  The limits follow
 * the selected stock battery table, across the enabled thermal rows.
 * Both cells and the pack must qualify for the selected voltage and
 * current. MiPPS authentication controls the source budget only. On
 * reaching CV, lower the bus request until buck can finish charging.
 * SC8541 independently enforces its protection registers and watchdog.
 */
#define MCA_CP_POLL_MS		500
#define MCA_CP_VBAT_MIN_MV	3450	/* stock min_vbat */
#define MCA_CP_VBAT_MAX_MV	4473	/* normal_volt_para18_35/35_48 */
#define MCA_CP_TAPER_FCC_MA	3000
#define MCA_CP_VBAT_HYS_MV	13	/* pps_taper_vol_hys + quick_chg_fv_hys */
#define MCA_CP_CELL_OCP_MA	7000	/* lower of stock ocp_threshold 9/7 A */
/* stock normal_volt_para18_35: 8 A, 7.23 A from 4150 mV */
#define MCA_CP_VSTEP_MV		4150
#define MCA_CP_VSTEP_MA		7230
/*
 * Battery temperature, degC, of the coolest and the warmest of the two
 * cells and the pack: stock batt_para_lwn allows 7.23 A from 13, 8 A from
 * 18 and 4.52 A from 48 to 55 degC (2 degC hysteresis on the warm side).
 */
#define MCA_CP_TBAT_MIN		8	/* batt_para_lwn first active row */
#define MCA_CP_TBAT_EXIT	55	/* first disabled row after warm */
#define MCA_CP_IBAT_MAX_MA	12000	/* stock div_max_curr[DIV2], battery side */
#define MCA_CP_MAX_POWER_MW	67000	/* stock max_power */
/*
 * Stage temperature, 0.1 degC: from MCA_CP_TDIE_HOT the bus target drops
 * MCA_CP_TDIE_STEP_MA a poll (the stage warms about 1 degC/s at 4 A), down
 * to MCA_CP_TDIE_MIN_MA, and comes back a quarter as fast below
 * MCA_CP_TDIE_HOT - MCA_CP_TDIE_HYS; MCA_CP_TDIE_MAX leaves.
 */
#define MCA_CP_TDIE_HOT		1200
#define MCA_CP_TDIE_HYS		50
#define MCA_CP_TDIE_MAX		1250	/* Piano die_temp_max */
#define MCA_CP_TDIE_STEP_MA	200
#define MCA_CP_TDIE_MIN_MA	1000
#define MCA_CP_DELTA_MV		300	/* stock div_delta_volt */
#define MCA_CP_STEP_MV		40	/* stock open path step */
/*
 * Regulation step, the PPS resolution: one step moves the bus current by
 * about 140 mA, so it is held between the target and MCA_CP_BAND_MA below.
 */
#define MCA_CP_REG_STEP_MV	20
#define MCA_CP_REG_STEP_MA	140
#define MCA_CP_REG_MAX_STEP_MV	200	/* down, when well above the target */
#define MCA_CP_BAND_MA		200
/*
 * Above twice the battery voltage: the most the stage may see, and the most
 * the source may be asked for (the cable drops up to about 0.8 V at 5 A)
 */
#define MCA_CP_VBUS_MAX_DELTA_MV 1200
#define MCA_CP_REQ_MAX_DELTA_MV	2000
#define MCA_CP_PPS_MIN_MV	6000
#define MCA_CP_PPS_MAX_MV	11000	/* stock third_pps_vbus_max */
#define MCA_CP_BUCK_MA		500	/* buck input during opening */
/* SC8541 EVM, table 3: 8 A output at 2:1. Piano div_single_curr
 * allows 12 A, with a 7 A bus OCP. Leave one regulation band below
 * the programmed 8 A battery OCP even on a higher-current contract.
 */
#define MCA_CP_STAGE_IBUS_MA	(8000 / 2 - MCA_CP_BAND_MA)
#define MCA_CP_PAR_HYS_MA	300
#define MCA_CP_PAR_FCC_MAX_MA	4000
#define MCA_CP_PPS_MAX_MA	6000	/* current limit asked of the source */
#define MCA_CP_STD_PPS_MAX_MA	5000
/* bus over-current protection of the stage, above the target */
#define MCA_CP_BUSOCP_MARGIN_MA	1000
#define MCA_CP_BUSOCP_MAX_MA	7000	/* Piano bus-ocp-threshold */
#define MCA_CP_OPEN_IBUS_MA	500	/* the stage counts as running above */
#define MCA_CP_OPEN_TRIES	10
#define MCA_CP_LOW_IBUS_MA	300	/* leaves when below for MCA_CP_LOW_POLLS */
#define MCA_CP_TRIP_IBUS_MA	500	/* leaves at once this far above the target */
#define MCA_CP_TRIP_IBAT_MA	1000
/*
 * A lowered battery limit trips only this long after the stage reached
 * its lowered target: the gauges average over 10-15 s
 */
#define MCA_CP_TRIP_GRACE_MS	30000
#define MCA_CP_LOW_POLLS	6
#define MCA_CP_LOG_POLLS	20
/* see xiaomi_mca_cp_learn_drop() */
#define MCA_CP_DROP_BACKOFF_MA	500
#define MCA_CP_DROP_FORGET_MS	10000
/* an authenticated adapter resets itself within this, see xiaomi_mca_auth_pending() */
#define MCA_AUTH_RESET_WAIT_MS	30000

static bool cp_charge = true;
module_param(cp_charge, bool, 0644);
MODULE_PARM_DESC(cp_charge,
		 "Charge through the SC8541 switched-capacitor stage from PPS sources; needs hv_charge and charge_current (default: on)");

static unsigned int cp_ibat_max;
module_param(cp_ibat_max, uint, 0444);
MODULE_PARM_DESC(cp_ibat_max,
		 "Battery current ceiling, both cells, mA (0 follows selected stock table; 1000-12000 overrides downwards)");


/* Chinese lwn tables; the _gbl node is only for non-CN hardware. */
enum xiaomi_mca_profile {
	MCA_PROFILE_NORMAL,
	MCA_PROFILE_FFC,
	MCA_PROFILE_COLD,
	MCA_PROFILE_COOL,
	MCA_PROFILE_WARM,
	MCA_PROFILE_COUNT,
};

static const char * const xiaomi_mca_profile_names[] = { "normal", "ffc", "cold", "cool", "warm" };

struct xiaomi_mca_charge_step {
	u32 mv;
	u32 ma;
};

static const struct xiaomi_mca_charge_step xiaomi_mca_profile_steps[][4] = {
	/* batt_para_lwn, normal_volt_para8_13/13_18/48_55 */
	[MCA_PROFILE_COLD] = { { 4493, 4320 } },
	[MCA_PROFILE_COOL] = { { 4493, 7100 } },
	[MCA_PROFILE_WARM] = { { 4053, 4320 } },
	[MCA_PROFILE_NORMAL] = { { 4150, 8000 }, { 4473, 7230 } },
	[MCA_PROFILE_FFC] = {
		{ 4144, 12400 }, { 4300, 10650 }, { 4475, 9040 }, { 4540, 7230 },
	},
};

static const unsigned int xiaomi_mca_profile_counts[] = { 2, 4, 1, 1, 1 };

enum xiaomi_mca_hv {
	MCA_HV_OFF,		/* 5 V, may ask for 9 V */
	MCA_HV_REQUESTED,	/* 9 V asked for, waiting for the bus */
	MCA_HV_ON,		/* bus at 9 V */
	MCA_HV_COOLDOWN,	/* renegotiate once back at 5 V */
};

enum xiaomi_mca_cp {
	MCA_CP_OFF,		/* buck charging, may start */
	MCA_CP_OPENING,		/* source asked for, stage being started */
	MCA_CP_ON,		/* stage running */
	MCA_CP_STOPPING,	/* stop failed; retain ownership until confirmed off */
	MCA_CP_COOLDOWN,	/* resume after the source settles */
};


/*
 * All fields are little endian. Requests are 276 bytes and replies 280 bytes
 * long, the layout Xiaomi's downstream mca_adsp_glink driver uses: its reply
 * handler requires 280 bytes and a zero return code at offset 16, matches the
 * sequence number at offset 20 and copies the data from offset 24. A scalar
 * value takes the first four bytes of the data area, the rest is zero.
 */
struct xiaomi_mca_request {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 seq;
	u8 data[MCA_DATA_LEN];
};

struct xiaomi_mca_reply {
	struct pmic_glink_hdr hdr;
	__le32 property;
	__le32 retcode;
	__le32 seq;
	u8 data[MCA_DATA_LEN];
};

static_assert(sizeof(struct xiaomi_mca_request) == 276);
static_assert(sizeof(struct xiaomi_mca_reply) == 280);

enum xiaomi_mca_usb_index {
	MCA_USB_ONLINE,
	MCA_USB_VOLTAGE_NOW,
	MCA_USB_CURRENT_NOW,
	MCA_USB_INPUT_CURRENT_LIMIT,
	MCA_USB_REAL_TYPE,
	MCA_USB_CHARGE_CURRENT,
	MCA_USB_COUNT,
};

static const u32 xiaomi_mca_usb_property[MCA_USB_COUNT] = {
	[MCA_USB_ONLINE] = MCA_PROP_USB_ONLINE,
	[MCA_USB_VOLTAGE_NOW] = MCA_PROP_USB_VOLTAGE_NOW,
	[MCA_USB_CURRENT_NOW] = MCA_PROP_USB_CURRENT_NOW,
	/*
	 * The limit the charger was asked for. With the input at 500 mA this
	 * reads 100, so it is not necessarily the limit in force.
	 */
	[MCA_USB_INPUT_CURRENT_LIMIT] = MCA_PROP_INPUT_CURRENT_LIMIT,
	[MCA_USB_REAL_TYPE] = MCA_PROP_USB_REAL_TYPE,
	/* Reads 0 until a charge current is written. */
	[MCA_USB_CHARGE_CURRENT] = MCA_PROP_CHARGE_CURRENT,
};

struct xiaomi_mca_limits {
	u32 input_current_ma;
	u32 charge_current_ma;
};

/*
 * The highest values written, in mA, by USB real type: the input limits
 * (in_*) and charge currents (chg_*) of the stock strategy_buckchg node of
 * the Xiaomi Pad 8 Pro, picked per type as the downstream strategy_buckchg
 * does. At 5 V that stack maps QC types to DCP before selecting limits.
 * At 5 V QC and PD are held to the
 * DCP values (in_dcp 1500, chg_dcp 2000). Types not listed cannot write.
 */
static const struct xiaomi_mca_limits xiaomi_mca_type_limits[] = {
	[MCA_USB_REAL_TYPE_SDP] = { 500, 500 },
	[MCA_USB_REAL_TYPE_CDP] = { 900, 900 },
	[MCA_USB_REAL_TYPE_DCP] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_FLOAT] = { 1000, 1000 },
	[MCA_USB_REAL_TYPE_HVDCP2] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_HVDCP3] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_HVDCP3_B] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_HVDCP3P5] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_TYPEC] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_PD] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_PD_VERIFY] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_PPS] = { 1500, 2000 },
	[MCA_USB_REAL_TYPE_OCP] = { 1000, 1000 },
};

struct xiaomi_mca_cached {
	u32 value;
	unsigned long expires;
	bool valid;
};

/* What a change notification would have announced. */
struct xiaomi_mca_state {
	u32 online;
	u32 chgr_status;
	u32 real_type;
};

struct xiaomi_mca_stats {
	u64 reads;
	u64 writes;
	u64 replies;
	u64 errors;
	u64 malformed;
	u64 timeouts;
	u64 unmatched;
	u64 notifications;
};

/* Stock mca_glink_nty_msg: notification is not hdr.opcode. */
struct xiaomi_mca_notification {
	struct pmic_glink_hdr hdr;
	__le32 id;
	u8 data[MCA_DATA_LEN];
};

struct xiaomi_mca_notify_record {
	u64 time_ns;
	u32 generation;
	u32 id;
	u32 data[4];
};

#define MCA_PROP_PEN_BOOST_ENABLE 0x200f
#define MCA_PROP_PEN_BOOST_VOLTAGE 0x2010

struct xiaomi_mca_cells {
	int vmax, vmin, ibat, imax, cold, hot;
	bool cell_ocp;
	int status;
};

struct mca_connector {
	struct gpio_desc *cutoff;
	struct iio_channel *ntc[6];
	int temp[2];	/* last good mC, INT_MIN until first valid sample */
	bool valid[2];
	ktime_t sampled;
	bool tripped;
};

struct xiaomi_mca {
	struct mca_connector connector;
	/* A policy tick uses one observation for FV, buck and pump control. */
	const struct xiaomi_mca_cells *poll_cells;
	struct device *dev;
	struct pmic_glink_client *client;
	struct power_supply *psy;
	struct dentry *debugfs;

	/* serializes requests, the cache and @req: one request in flight */
	struct mutex lock;
	struct completion ack;
	struct xiaomi_mca_request req;
	u32 seq;
	struct xiaomi_mca_cached cache[MCA_USB_COUNT];
	struct xiaomi_mca_state last;
	bool last_valid;
	/* an input current limit was written for the source in @icl_source */
	bool icl_written;
	unsigned int unknown_polls;
	u32 aicl_cap_ma;		/* observed collapse, valid for this attachment */
	u32 aicl_probe_floor_ma;	/* nonzero while watching a raised 5 V vote */
	ktime_t aicl_deadline;
	bool fcc_written;
	bool voltage_written;	/* a raised voltage request may have reached firmware */
	struct xiaomi_mca_state icl_source;

	struct regulator_dev *pen_rdev;
	bool pen_requested, pen_on, pen_dirty;
	u32 pen_generation, pen_attempt_generation;
	bool pen_attempted;
	bool piano_otg;
	bool config_attempted;
	u32 config_generation;
	int config_result;
	bool cp_absent_warned;
	bool otg_active;

	struct delayed_work poll_work;
	bool term_written;	/* restore even after a lost write acknowledgment */
	bool cp_parallel_done;
	bool cp_normal_cv;
	bool term_hot;		/* float-voltage thermal hysteresis */
	bool fv_derated;	/* pack temperature unavailable */
	unsigned int gauge_failures;
	int cell_zone;		/* coldest sensor: 0, 8, 13 or 18 C row */
	bool zone_valid, ffc_warm, cold_fv;
	bool ffc_done;
	bool ffc_voted;	/* high FV may have reached firmware */
	u32 ffc_type, ffc_attach;
	bool ffc_active;		/* checked again every poll, never latched by auth */
	/* Thresholds are shared by the two cells and protected by rx_lock. */
	unsigned int charge_start, charge_end;
	struct wakeup_source *charge_ws;
	u32 input_generation;	/* changes that invalidate the wired source sample */
	bool input_pending;	/* PM must let the worker classify this input */
	bool otg_requested;	/* firmware role request, independent of gate errors */
	bool charge_awake;	/* protected by rx_lock, like the thresholds */
	bool charge_hold;	/* SOC hysteresis, protected by lock */
	bool charge_blocked;	/* no CP/HV policy until limits can be satisfied */
	bool charge_disabled;	/* a disable may have reached firmware */
	int coolest_temp;
	bool charge_hot, charge_cold;
	struct power_supply *batteries[2];
	bool sleeping;		/* no transactions between prepare and complete */
	bool online;
	int vbus_uv;
	int ibus_ua;
	int icl_ma;
	u32 real_type;
	int icl_set_ma;		/* last limit written by charge_policy, 0 if none */
	int fcc_set_ma;		/* last current written by charge_current, 0 if none */
	enum xiaomi_mca_hv hv;
	int hv_polls;
	bool hv_pps;		/* 9 V is asked of a PPS APDO */
	u32 hv_pps_ma;		/* its requested current limit */
	u32 cp_type;	/* protocol at stage entry */
	enum xiaomi_mca_cp cp;
	const char *cp_last_exit;
	const char *cp_stop_reason;
	u32 cp_retry_ms;
	unsigned long cp_running_at;
	const char *hv_last_exit;
	u32 cp_exits;
	struct power_supply *cp_psy;	/* the stage, held while not MCA_CP_OFF */
	u32 cp_mv;		/* voltage asked of the PPS source */
	u32 cp_ma;		/* APDO current limit, with source/authentication caps */
	u32 cp_request_ma;	/* current in the most recent PPS request */
	u32 cp_max_mv;		/* the most the source offers */
	u32 cp_ibus_ma;		/* bus current target */
	u32 cp_ibus_hot;	/* the target as lowered for the stage temperature */
	u32 cp_ibat_trip;	/* battery current limit the trip checks against */
	/* jiffies when the limit was last that high, or the stage above target */
	unsigned long cp_trip_at;
	enum xiaomi_mca_profile cp_profile; /* follows the current FFC conditions */
	unsigned int cp_stage; /* monotonic while the selected table is unchanged */
	u32 cp_target_ma; /* effective battery target after table/thermal limits */
	unsigned long cp_retry_at;
	bool auth_lost;
	unsigned long auth_retry;
	u32 auth_retry_ms;
	int qc_align_mv;
	unsigned int qc_stalls;
	bool cp_die_hot;
	bool cp_fault;
	u32 cp_recharge_cv;	/* CV of the profile that tapered */
	int cp_polls;		/* while opening: tries; then: polls */
	int cp_low;		/* consecutive polls below MCA_CP_LOW_IBUS_MA */
	u32 cp_ibus_last;	/* stage and buck input current at the last step */
	unsigned long cp_cap_retry;
	u32 cp_ibus_cap;	/* learned from a source dropping out, 0 if none */
	unsigned long cp_off_at;	/* jiffies when the source went away */
	unsigned long cp_on_at;		/* jiffies when it came */
	bool auth_ok;		/* adapter response matched our fresh challenge */
	u32 auth_generation;
	u32 auth_attach_generation;
	u32 policy_attach_generation;
	u32 attach_generation;	/* physical UCSI disconnects, under rx_lock */
	int ucsi_state;
	struct notifier_block psy_nb;
	bool auth_cleanup;	/* verify_process clear still needs confirmation */
	bool mipps_attempted;	/* throttle repeated exchanges on the current cable */
	unsigned long auth_at;	/* jiffies when it did */
	bool auth_tried;	/* direct charging asked for before any reset */
	unsigned long hv_retry;	/* jiffies before which 9 V is not asked for */
	u32 service_generation;
	u32 policy_generation;
	bool policy_active;
	u32 adapter_svid;
	u32 retcode;

	/* orders the fields below against the GLINK callbacks */
	spinlock_t rx_lock;
	bool service_up;
	bool poll_ready;
	struct xiaomi_mca_notify_record notify[MCA_NOTIFY_HISTORY];
	u32 notify_head;
	u32 notify_count;
	u32 notify_pending;
	u64 notify_dropped;
	bool pending;
	bool answered;
	u32 pending_opcode;
	u32 pending_property;
	u32 pending_seq;
	int result;
	u32 value;
	u8 reply[sizeof(struct xiaomi_mca_reply)];
	size_t reply_len;
	struct xiaomi_mca_stats stats;
};

/* The poll mutex keeps the saved attachment fixed throughout an exchange. */
static bool xiaomi_mca_auth_current(struct xiaomi_mca *mca, u32 generation)
{
	return generation == READ_ONCE(mca->service_generation) &&
	       mca->auth_attach_generation == READ_ONCE(mca->attach_generation);
}

static bool xiaomi_mca_authenticated(struct xiaomi_mca *mca)
{
	return mca->auth_ok && xiaomi_mca_auth_current(mca, mca->auth_generation);
}

static int xiaomi_mca_source_type(struct xiaomi_mca *mca, u32 *type, u32 *raw);

static int xiaomi_mca_read_type(struct xiaomi_mca *mca, u32 *type)
{
	return xiaomi_mca_source_type(mca, type, NULL);
}

/* A write carries @value in the first four bytes of the data area, a read nothing. */
static void xiaomi_mca_build_request(struct xiaomi_mca_request *req, u32 opcode,
				     u32 property, u32 seq, u32 value)
{
	memset(req, 0, sizeof(*req));
	req->hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR);
	req->hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP);
	req->hdr.opcode = cpu_to_le32(opcode);
	req->property = cpu_to_le32(property);
	req->seq = cpu_to_le32(seq);
	if (opcode == MCA_OPCODE_WRITE)
		put_unaligned_le32(value, req->data);
}

/*
 * Return 0 and the first data word if @data is the reply to the request
 * @opcode for @property with sequence number @seq, -EIO if the firmware
 * failed that request, -EPROTO if the reply to it is malformed, and -ENOMSG
 * if @data answers no such request.
 */
static int xiaomi_mca_parse_reply(const void *data, size_t len, u32 opcode, u32 property,
				  u32 seq, u32 *value)
{
	const struct xiaomi_mca_reply *reply = data;

	if (len < offsetof(struct xiaomi_mca_reply, data))
		return -ENOMSG;

	if (le32_to_cpu(reply->hdr.type) != PMIC_GLINK_REQ_RESP ||
	    le32_to_cpu(reply->hdr.opcode) != opcode ||
	    le32_to_cpu(reply->seq) != seq)
		return -ENOMSG;

	if (len != sizeof(*reply) || le32_to_cpu(reply->property) != property)
		return -EPROTO;

	if (reply->retcode)
		return -EIO;

	*value = get_unaligned_le32(reply->data);

	return 0;
}

static int xiaomi_mca_usb_type(u32 real_type)
{
	switch (real_type) {
	case MCA_USB_REAL_TYPE_SDP:
		return POWER_SUPPLY_USB_TYPE_SDP;
	case MCA_USB_REAL_TYPE_CDP:
		return POWER_SUPPLY_USB_TYPE_CDP;
	case MCA_USB_REAL_TYPE_DCP:
	case MCA_USB_REAL_TYPE_HVDCP2:
	case MCA_USB_REAL_TYPE_HVDCP3:
	case MCA_USB_REAL_TYPE_HVDCP3_B:
	case MCA_USB_REAL_TYPE_HVDCP3P5:
		return POWER_SUPPLY_USB_TYPE_DCP;
	case MCA_USB_REAL_TYPE_TYPEC:
		return POWER_SUPPLY_USB_TYPE_C;
	case MCA_USB_REAL_TYPE_PD:
	case MCA_USB_REAL_TYPE_PD_VERIFY:
		return POWER_SUPPLY_USB_TYPE_PD;
	case MCA_USB_REAL_TYPE_PPS:
		return POWER_SUPPLY_USB_TYPE_PD_PPS;
	default:
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
}

static const struct xiaomi_mca_limits xiaomi_mca_pd_limits = { 1600, 3000 };
static const struct xiaomi_mca_limits xiaomi_mca_pps_limits = { 2500, 3500 };

static const struct xiaomi_mca_limits *xiaomi_mca_limits(u32 real_type)
{
	if (real_type < ARRAY_SIZE(xiaomi_mca_type_limits) &&
	    xiaomi_mca_type_limits[real_type].input_current_ma)
		return &xiaomi_mca_type_limits[real_type];

	return NULL;
}

static bool xiaomi_mca_is_qc_pump(u32 type)
{
	return type == MCA_TYPE_HVDCP_3_B || type == MCA_TYPE_HVDCP_3P5;
}

static bool xiaomi_mca_is_qc(u32 type)
{
	return type >= MCA_TYPE_HVDCP && type <= MCA_TYPE_HVDCP_3P5;
}

static const struct xiaomi_mca_limits *xiaomi_mca_hv_limits(u32 type)
{
	static const struct xiaomi_mca_limits qc2 = { 1200, 2600 };
	static const struct xiaomi_mca_limits qc3 = { 1600, 3600 };

	if (type == MCA_TYPE_HVDCP)
		return &qc2;
	if (xiaomi_mca_is_qc(type))
		return &qc3;
	if (type == MCA_TYPE_PD)
		return &xiaomi_mca_pd_limits;
	if (type == MCA_TYPE_PD_VERIFY || type == MCA_TYPE_PD_PPS)
		return &xiaomi_mca_pps_limits;
	return NULL;
}

/*
 * Return 0 and the value in mA to write for @psp set to @ua with a source of
 * type @real_type attached, -EOPNOTSUPP for an unsupported source, or -EINVAL
 * if the property or current is out of range.
 */
static int xiaomi_mca_current_ma(enum power_supply_property psp, u32 real_type, bool hv, int ua,
				 u32 *ma)
{
	const struct xiaomi_mca_limits *limits = xiaomi_mca_limits(real_type);
	u32 max;

	if (!limits)
		return -EOPNOTSUPP;
	if (hv) {
		const struct xiaomi_mca_limits *high = xiaomi_mca_hv_limits(real_type);

		if (high)
			limits = high;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		max = limits->input_current_ma;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		max = limits->charge_current_ma;
		break;
	default:
		return -EINVAL;
	}

	if (ua < 0 || ua < MCA_CURRENT_MIN_MA * MILLI || ua > max * MILLI)
		return -EINVAL;

	*ma = ua / MILLI;

	return 0;
}

static int xiaomi_mca_usb_propval(enum xiaomi_mca_usb_index index, u32 value)
{
	switch (index) {
	case MCA_USB_ONLINE:
		return !!value;
	case MCA_USB_INPUT_CURRENT_LIMIT:
	case MCA_USB_CHARGE_CURRENT:
		return value * MILLI;
	case MCA_USB_REAL_TYPE:
		return xiaomi_mca_usb_type(value);
	default:
		return (s32)value;
	}
}

static int xiaomi_mca_transfer_data(struct xiaomi_mca *mca, u32 opcode, u32 property,
				    void *data, size_t len)
{
	int ret;

	lockdep_assert_held(&mca->lock);
	if (mca->sleeping)
		return -EHOSTDOWN;
#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
	KUNIT_STATIC_STUB_REDIRECT(xiaomi_mca_transfer_data, mca, opcode, property, data, len);
#endif

	if (len > MCA_DATA_LEN)
		return -EINVAL;
	xiaomi_mca_build_request(&mca->req, opcode, property, ++mca->seq, 0);
	if (opcode == MCA_OPCODE_WRITE)
		memcpy(mca->req.data, data, len);
	reinit_completion(&mca->ack);

	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (!mca->service_up)
			return -ENODATA;

		mca->pending = true;
		mca->answered = false;
		mca->pending_opcode = opcode;
		mca->pending_property = property;
		mca->pending_seq = mca->seq;
		if (opcode == MCA_OPCODE_WRITE)
			mca->stats.writes++;
		else
			mca->stats.reads++;
	}

	ret = pmic_glink_send(mca->client, &mca->req, sizeof(mca->req));
	if (!ret)
		wait_for_completion_timeout(&mca->ack, MCA_REQUEST_TIMEOUT);

	guard(spinlock_irqsave)(&mca->rx_lock);
	mca->pending = false;
	if (ret)
		return ret;

	if (!mca->answered) {
		mca->stats.timeouts++;
		dev_err_ratelimited(mca->dev, "no reply to the %s of %#x\n",
				    opcode == MCA_OPCODE_WRITE ? "write" : "read", property);
		return -ETIMEDOUT;
	}

	if (mca->result)
		return mca->result;

	if (opcode == MCA_OPCODE_READ)
		memcpy(data, mca->reply + offsetof(struct xiaomi_mca_reply, data), len);

	return 0;
}

static int xiaomi_mca_transfer(struct xiaomi_mca *mca, u32 opcode, u32 property,
			       u32 *value)
{
	__le32 data = cpu_to_le32(*value);
	int ret = xiaomi_mca_transfer_data(mca, opcode, property, &data, sizeof(data));

	if (!ret && opcode == MCA_OPCODE_READ)
		*value = le32_to_cpu(data);
	return ret;
}

static int xiaomi_mca_read(struct xiaomi_mca *mca, u32 property, u32 *value)
{
	*value = 0;

	return xiaomi_mca_transfer(mca, MCA_OPCODE_READ, property, value);
}

static void xiaomi_mca_invalidate(struct xiaomi_mca *mca)
{
	unsigned int i;

	lockdep_assert_held(&mca->lock);

	for (i = 0; i < MCA_USB_COUNT; i++)
		mca->cache[i].valid = false;
}

/* Current requests use the same bounded, acknowledged write path as sysfs. */
static int xiaomi_mca_write_blob(struct xiaomi_mca *mca, u32 property,
				 const void *data, size_t len);

static int xiaomi_mca_write(struct xiaomi_mca *mca, u32 property, u32 value)
{
	__le32 data = cpu_to_le32(value);

	if (property != MCA_PROP_INPUT_CURRENT_LIMIT && property != MCA_PROP_CHARGE_CURRENT)
		return -EINVAL;

	return xiaomi_mca_write_blob(mca, property, &data, sizeof(data));
}

/*
 * The firmware keeps the input current limit when the source changes, and
 * the limit written may be too high for the next one. Put back the value the
 * firmware starts with, as the downstream stack does on detach. On failure
 * the next poll tries again.
 */
static int xiaomi_mca_restore_icl(struct xiaomi_mca *mca)
{
	int ret;

	lockdep_assert_held(&mca->lock);

	if (!mca->icl_written)
		return 0;

	ret = xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, MCA_ICL_DEFAULT_MA);
	if (ret) {
		dev_warn_ratelimited(mca->dev,
				     "failed to restore the input current limit: %d\n", ret);
		return ret;
	}

	mca->icl_written = false;
	mca->icl_set_ma = 0;
	dev_info(mca->dev, "input current limit restored to %u mA\n", MCA_ICL_DEFAULT_MA);
	return 0;
}

static int xiaomi_mca_read_blob(struct xiaomi_mca *mca, u32 prop, void *buf, size_t len)
{
	return xiaomi_mca_transfer_data(mca, MCA_OPCODE_READ, prop, buf, len);
}

static u32 xiaomi_mca_read_u32(struct xiaomi_mca *mca, u32 prop)
{
	__le32 val;

	if (xiaomi_mca_read_blob(mca, prop, &val, sizeof(val)))
		return U32_MAX;

	return le32_to_cpu(val);
}

/* PPS request word: voltage in 20 mV and current in 50 mA steps */
static u32 xiaomi_mca_pps_word(u32 mv, u32 ma)
{
	return (mv / 20) << 16 | ma / 50;
}

static bool xiaomi_mca_pps_word_ok(u32 val)
{
	u32 mv = (val >> 16) * 20, ma = (val & 0xffff) * 50;

	if (ma < MCA_HV_PPS_MIN_MA)
		return false;
	if (mv == MCA_HV_MV && ma <= MCA_HV_PPS_MAX_MA)
		return true;

	return cp_charge && mv >= MCA_CP_PPS_MIN_MV && mv <= MCA_CP_PPS_MAX_MV &&
	       ma <= MCA_CP_PPS_MAX_MA;
}

static bool xiaomi_mca_write_allowed(u32 prop, const void *data, size_t len)
{
	u32 val = len == sizeof(u32) ? get_unaligned_le32(data) : 0;

	switch (prop) {
	case MCA_PROP_PEN_BOOST_ENABLE:
		return len == sizeof(u32) && (!val || val == 0x20100 || val == 0x20101);
	case MCA_PROP_PEN_BOOST_VOLTAGE:
		return len == sizeof(u32) && (val == (BIT(16) | 5000) ||
			val == (BIT(16) | 5200) || val == (BIT(16) | 5500) ||
			val == (BIT(16) | 6000) || val == (BIT(16) | 6500));
	case MCA_PROP_CID_CFG:
		return len == sizeof(u32) && val == MCA_PIANO_CID_CFG;
	case MCA_PROP_CHARGING_ENABLED:
	case MCA_PROP_CP_STATE:
	case MCA_PROP_CC_TOGGLE:
		return len == 1 && *(const u8 *)data <= 1;
	case MCA_PROP_OTG_CFG:
		return len == sizeof(xiaomi_mca_piano_otg_cfg) &&
		       !memcmp(data, xiaomi_mca_piano_otg_cfg, len);
	case MCA_PROP_INPUT_CURRENT_LIMIT:
		return len == sizeof(u32) &&
		       val >= MCA_ICL_MIN_MA && val <= MCA_ICL_5V_MAX_MA;
	case MCA_PROP_TERM_VOLT:
		return len == sizeof(u32) &&
		       (val == MCA_TERM_HOT_MV || val == MCA_TERM_DEFAULT_MV ||
			val == MCA_TERM_COLD_MV || val == MCA_TERM_FULL_MV ||
			val == MCA_TERM_FFC_MV);
	case MCA_PROP_CHARGE_CURRENT:
		return len == sizeof(u32) &&
		       val >= MCA_FCC_MIN_MA && val <= MCA_CP_PAR_FCC_MAX_MA;
	case MCA_PROP_AICL_ENABLE:
	case MCA_PROP_RERUN_AICL:
		return len == 1 && *(const u8 *)data == 1;
	case MCA_PROP_QC_PULSE:
		return cp_charge && len == sizeof(u32) && (val == MCA_QC_UP || val == MCA_QC_DOWN);
	case MCA_PROP_QC_VOLT:
	case MCA_PROP_PD_FIXED_VOLT:
		return len == sizeof(u32) &&
		       (val == 5000 || (hv_charge && val == MCA_HV_MV));
	case MCA_PROP_PPS_SELECT:
		return hv_charge && len == sizeof(u32) &&
		       xiaomi_mca_pps_word_ok(val);
	case MCA_PROP_VDM_VERSION:
	case MCA_PROP_VDM_VOLTAGE:
	case MCA_PROP_VDM_TEMP:
		return mipps_auth && len == sizeof(u32);
	case MCA_PROP_VDM_SEED:
	case MCA_PROP_VDM_AUTH:
		return mipps_auth && len == MCA_VDM_LEN;
	case MCA_PROP_VDM_VERIFIED:
	case MCA_PROP_VDM_REMOVE_COMP:
		return mipps_auth && len == sizeof(u32) && val <= 1;
	case MCA_PROP_PD_VERIFIED:
	case MCA_PROP_VERIFY_PROCESS:
		return len == sizeof(u32) && val <= 1 && (mipps_auth || !val);
	default:
		return false;
	}
}

static bool xiaomi_mca_is_pd(u32 real_type);
static u32 xiaomi_mca_5v_input(struct xiaomi_mca *mca, u32 type);

static bool xiaomi_mca_probe_5v(u32 type)
{
	return type == MCA_TYPE_DCP || type == MCA_TYPE_DCP_B ||
	       type == MCA_TYPE_FLOAT || type == MCA_TYPE_UNKNOWN;
}
/* Legacy 5 V starting votes on this board. Unknown uses FLOAT until
 * classification or a contract supplies stronger evidence.
 */
static u32 xiaomi_mca_5v_floor(u32 raw)
{
	return raw == MCA_TYPE_FLOAT || raw == MCA_TYPE_UNKNOWN ? MCA_FLOAT_ICL_MA : 1500;
}

static u32 xiaomi_mca_contract_ma(struct xiaomi_mca *mca, int mv, u32 fallback);

/* All paths share the original checked transport and this refusal gate. */
static int xiaomi_mca_write_blob(struct xiaomi_mca *mca, u32 prop,
				 const void *data, size_t len)
{
	u8 buf[MCA_VDM_LEN];
	u32 type, raw, vbus, value, checked, readback, generation;
	int ret;

	if (len > sizeof(buf) || !xiaomi_mca_write_allowed(prop, data, len))
		return -EPERM;
	lockdep_assert_held(&mca->lock);
	if (prop == MCA_PROP_CHARGING_ENABLED) {
		u8 enabled;

		/* Stock qcom_subpmic_enable_charging sends one byte, not u32. */
		scoped_guard(spinlock_irqsave, &mca->rx_lock)
			generation = mca->service_generation;
		memcpy(buf, data, len);
		ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
		if (ret)
			return ret;
		ret = xiaomi_mca_read_blob(mca, prop, &enabled, sizeof(enabled));
		if (ret)
			return ret;
		scoped_guard(spinlock_irqsave, &mca->rx_lock) {
			if (generation != mca->service_generation || !mca->service_up)
				return -ECONNRESET;
		}
		return enabled == *(const u8 *)data ? 0 : -EIO;
	}
	if (prop == MCA_PROP_TERM_VOLT) {
		if (get_unaligned_le32(data) == MCA_TERM_FFC_MV &&
		    !mca->ffc_active)
			return -EPERM;
		/* Unlike source current limits, this also restores while unplugged. */
		scoped_guard(spinlock_irqsave, &mca->rx_lock)
			generation = mca->service_generation;
		mca->term_written = true;
		if (get_unaligned_le32(data) == MCA_TERM_FFC_MV)
			mca->ffc_voted = true;
		memcpy(buf, data, len);
		ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
		if (ret)
			return ret;
		ret = xiaomi_mca_read(mca, prop, &readback);
		if (ret)
			return ret;
		scoped_guard(spinlock_irqsave, &mca->rx_lock) {
			if (generation != mca->service_generation || !mca->service_up)
				return -ECONNRESET;
		}
		if (readback != 2 * get_unaligned_le32(data) - 3600)
			return -EIO;
		mca->ffc_voted = get_unaligned_le32(data) == MCA_TERM_FFC_MV;
		return 0;
	}
	if (prop == MCA_PROP_CC_TOGGLE) {
		if (!mca->piano_otg)
			return -EOPNOTSUPP;
		scoped_guard(spinlock_irqsave, &mca->rx_lock)
			generation = mca->service_generation;
		memcpy(buf, data, len);
		ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
		if (ret)
			return ret;
		ret = xiaomi_mca_read_blob(mca, prop, buf, len);
		if (ret)
			return ret;
		scoped_guard(spinlock_irqsave, &mca->rx_lock) {
			if (generation != mca->service_generation || !mca->service_up)
				return -ECONNRESET;
		}
		return buf[0] == *(const u8 *)data ? 0 : -EIO;
	}
	if (prop == MCA_PROP_CP_STATE || prop == MCA_PROP_OTG_CFG || prop == MCA_PROP_CID_CFG ||
	    prop == MCA_PROP_PEN_BOOST_ENABLE || prop == MCA_PROP_PEN_BOOST_VOLTAGE) {
		if (!mca->piano_otg)
			return -EOPNOTSUPP;
		memcpy(buf, data, len);
		return xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
	}
	/* Clearing our authentication interlocks cannot raise source limits. */
	if ((prop == MCA_PROP_VERIFY_PROCESS || prop == MCA_PROP_PD_VERIFIED) &&
	    !get_unaligned_le32(data)) {
		memcpy(buf, data, len);
		return xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
	}
	ret = xiaomi_mca_source_type(mca, &type, &raw);
	if (ret)
		return ret;
	if (!xiaomi_mca_limits(type)) {
		/* An unclassified powered input can use the 5 V AICL loop.
		 * No QC/PD or other protocol writes on that evidence alone.
		 */
		bool probe = type == MCA_TYPE_UNKNOWN && mca->real_type == type &&
			     !mca->otg_active && !mca->pen_dirty &&
			     (prop == MCA_PROP_AICL_ENABLE || prop == MCA_PROP_RERUN_AICL ||
			      prop == MCA_PROP_INPUT_CURRENT_LIMIT || prop == MCA_PROP_CHARGE_CURRENT) &&
			     xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE) == 1 &&
			     !xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus) &&
			     vbus >= MCA_VBUS_ONLINE_UV && vbus < MCA_HV_VBUS_5V_MAX_UV;

		if (prop == MCA_PROP_CHARGE_CURRENT && get_unaligned_le32(data) == MCA_FCC_DEFAULT_MA &&
		    mca->fcc_written) {
			memcpy(buf, data, len);
			return xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
		}
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT && mca->icl_written &&
		    get_unaligned_le32(data) == MCA_ICL_DEFAULT_MA) {
			memcpy(buf, data, len);
			ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
			ret = ret ?: xiaomi_mca_read(mca, prop, &readback);
			if (ret)
				return ret;
			return readback == MCA_ICL_DEFAULT_MA ? 0 : -EIO;
		}
		if (!probe)
			return -EOPNOTSUPP;
	}
	if (prop == MCA_PROP_QC_PULSE &&
	    (!xiaomi_mca_is_qc_pump(type) || type != mca->cp_type ||
	     (mca->cp != MCA_CP_OPENING && mca->cp != MCA_CP_ON)))
		return -EPERM;
	if (prop == MCA_PROP_QC_VOLT && !xiaomi_mca_is_qc(type))
		return -EOPNOTSUPP;
	if (prop >= MCA_PROP_VERIFY_PROCESS && !xiaomi_mca_is_pd(type))
		return -EOPNOTSUPP;
	if (prop == MCA_PROP_INPUT_CURRENT_LIMIT || prop == MCA_PROP_CHARGE_CURRENT) {
		ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
		if (ret)
			return ret;
		value = get_unaligned_le32(data);
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT && vbus < MCA_HV_VBUS_5V_MAX_UV &&
		    mca->aicl_cap_ma && value > mca->aicl_cap_ma)
			return -EINVAL;
		ret = xiaomi_mca_current_ma(prop == MCA_PROP_INPUT_CURRENT_LIMIT ?
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT :
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, type,
			vbus >= MCA_HV_VBUS_MIN_UV && vbus <= MCA_HV_VBUS_MAX_UV,
			value * MILLI, &checked);
		/* The stock buck votes for CP parallel operation apply only
		 * under the active PPS contract, also below the 9 V window.
		 * The public setter checks the ordinary per-source limits.
		 */
		if (vbus >= MCA_VBUS_ONLINE_UV && vbus < MCA_HV_VBUS_5V_MAX_UV &&
		    (type == MCA_TYPE_C || xiaomi_mca_probe_5v(type) || xiaomi_mca_is_pd(type))) {
			u8 enabled;

			if (value > xiaomi_mca_5v_input(mca, type))
				return -EINVAL;
			if (xiaomi_mca_probe_5v(type) && value > xiaomi_mca_5v_floor(type) &&
			    (xiaomi_mca_read_blob(mca, MCA_PROP_AICL_ENABLE, &enabled, 1) ||
			     enabled != 1))
				return -EAGAIN;
			ret = 0;
		}
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT && xiaomi_mca_is_pd(type) &&
		    mca->cp != MCA_CP_ON && vbus >= MCA_HV_VBUS_MIN_UV &&
		    vbus <= MCA_HV_VBUS_MAX_UV && !mca->hv_pps) {
			u32 ma = xiaomi_mca_contract_ma(mca, MCA_HV_MV, 1600);

			if (value > min_t(u32, ma, MCA_ICL_HV_MAX_MA))
				return -EINVAL;
			ret = 0;
		}
		if (prop == MCA_PROP_CHARGE_CURRENT && xiaomi_mca_is_pd(type) &&
		    mca->hv == MCA_HV_ON && vbus >= MCA_HV_VBUS_MIN_UV &&
		    vbus <= MCA_HV_VBUS_MAX_UV && value <= MCA_CP_PAR_FCC_MAX_MA)
			ret = 0;
		if (mca->cp == MCA_CP_ON && (xiaomi_mca_is_pd(type) ||
		    (xiaomi_mca_is_qc_pump(type) && type == mca->cp_type)) &&
		    vbus >= MCA_CP_PPS_MIN_MV * 1000 && vbus <= MCA_CP_PPS_MAX_MV * 1000 &&
		    value <= (prop == MCA_PROP_INPUT_CURRENT_LIMIT ?
			      min(mca->cp_request_ma, (u32)MCA_ICL_HV_MAX_MA) : MCA_CP_PAR_FCC_MAX_MA))
			ret = 0;
		if (ret)
			return ret;
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT) {
			u32 floor = xiaomi_mca_5v_floor(raw);

			/* Arm before sending: the source may fall out before the
			 * acknowledgement or setpoint readback comes back.
			 */
			if (vbus >= MCA_VBUS_ONLINE_UV && vbus < MCA_HV_VBUS_5V_MAX_UV &&
			    (xiaomi_mca_probe_5v(type) || type == MCA_TYPE_C || xiaomi_mca_is_pd(type)) &&
			    value > floor && value > mca->icl_set_ma && !mca->aicl_cap_ma) {
				mca->aicl_probe_floor_ma = floor;
				mca->aicl_deadline = ktime_add_ms(ktime_get_boottime(), MCA_AICL_WATCH_MS);
			}
			mca->icl_written = true;
			mca->icl_source.online = 1;
			mca->icl_source.real_type = type;
		}
	}
	memcpy(buf, data, len);
	/* A lost acknowledgment does not prove the request had no effect. */
	if (prop == MCA_PROP_CHARGE_CURRENT)
		mca->fcc_written = true;
	if (prop == MCA_PROP_PPS_SELECT ||
	    ((prop == MCA_PROP_PD_FIXED_VOLT || prop == MCA_PROP_QC_VOLT) &&
	     get_unaligned_le32(data) > 5000)) {
		mca->voltage_written = true;
		mca->aicl_probe_floor_ma = 0;
	}
	ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
	xiaomi_mca_invalidate(mca);
	if (ret)
		return ret;
	/*
	 * ICL has a readable setpoint. FCC is a request to the charger firmware:
	 * the vendor set_ichg interface only writes it, and this unit reports
	 * zero on reads even after an acknowledged write. Do not manufacture a
	 * transport failure by comparing that value with the requested current.
	 * The matching firmware reply still has to acknowledge every write;
	 * actual charging is measured separately through the fuel gauges.
	 */
	if (prop == MCA_PROP_INPUT_CURRENT_LIMIT) {
		ret = xiaomi_mca_read(mca, prop, &readback);
		if (ret)
			return ret;
		if (readback != get_unaligned_le32(data))
			return -EIO;
	}
	return 0;
}

static int xiaomi_mca_write_u32(struct xiaomi_mca *mca, u32 prop, u32 val)
{
	__le32 data = cpu_to_le32(val);

	return xiaomi_mca_write_blob(mca, prop, &data, sizeof(data));
}

static int xiaomi_mca_clear_auth(struct xiaomi_mca *mca)
{
	int ret;

	if (!mca->auth_cleanup)
		return 0;
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_VERIFY_PROCESS, 0);
	if (!ret)
		mca->auth_cleanup = false;
	return ret;
}

/*
 * MiPPS challenge/response from xiaomi-mipps-auth, with Piano's stock
 * command order. The transport and charging policy remain in this driver.
 */
static int xiaomi_mca_vdm(struct xiaomi_mca *mca, unsigned int generation,
			 unsigned int cmd, const u8 *bytes, u32 flag)
{
	__le32 payload[MCA_VDM_LEN / 4] = {};
	unsigned long deadline;
	size_t len = sizeof(payload[0]);
	__le32 state, vbus;
	int attempt, i, ret;

	if (bytes) {
		for (i = 0; i < ARRAY_SIZE(payload); i++)
			payload[i] = cpu_to_le32(get_unaligned_be32(bytes + 4 * i));
		len = sizeof(payload);
	} else {
		payload[0] = cpu_to_le32(flag);
	}

	/* Stock batterysecret retries an unanswered command once. */
	for (attempt = 0; attempt < 2; attempt++) {
		if (!xiaomi_mca_auth_current(mca, generation))
			return -ENODEV;
		ret = xiaomi_mca_write_blob(mca, MCA_PROP_VDM_BASE + cmd - 1,
				      payload, len);
		if (ret)
			return ret;
		msleep(cmd == 4 || cmd == 5 ? 500 : 5);
		deadline = jiffies + msecs_to_jiffies(1000);
		do {
			if (!xiaomi_mca_auth_current(mca, generation))
				return -ENODEV;
			ret = xiaomi_mca_read_blob(mca, MCA_PROP_USB_VOLTAGE_NOW,
						   &vbus, sizeof(vbus));
			if (ret || le32_to_cpu(vbus) < MCA_VBUS_ONLINE_UV)
				return ret ?: -ENODEV;
			ret = xiaomi_mca_read_blob(mca, MCA_PROP_UVDM_STATE, &state, sizeof(state));
			if (ret)
				return ret;
			if (le32_to_cpu(state) == cmd)
				return 0;
			msleep(20);
		} while (time_before(jiffies, deadline));
	}
	return -ETIMEDOUT;
}

static bool xiaomi_mca_mipps_digest(struct xiaomi_mca *mca,
				  const struct piano_mipps_key *key,
				  u8 message[20], const u8 response[MCA_VDM_LEN],
				  u8 digest[SHA256_DIGEST_SIZE], u32 *adapter_id,
				  unsigned int generation)
{
	struct hmac_sha256_key prepared;
	u32 id;
	bool matched = false;

	hmac_sha256_preparekey(&prepared, key->key, sizeof(key->key));
	put_unaligned_be32(*adapter_id, message + MCA_VDM_LEN);
	hmac_sha256(&prepared, message, 20, digest);
	if (*adapter_id && !memcmp(response, digest, MCA_VDM_LEN)) {
		matched = true;
		goto out;
	}

	/*
	 * Firmware can omit or retain an old ID. Recover it only from a valid
	 * answer to our fresh random challenge, never from a cached identity.
	 * Limit work to 65536 guesses; reported 32-bit IDs are checked above.
	 * No match simply leaves this source on ordinary PD/PPS.
	 */
	for (id = 0; id < 65536; id++) {
		if (!(id & 255)) {
			if (!xiaomi_mca_auth_current(mca, generation))
				break;
			cond_resched();
		}
		put_unaligned_be32(id, message + MCA_VDM_LEN);
		hmac_sha256(&prepared, message, 20, digest);
		if (!memcmp(response, digest, MCA_VDM_LEN)) {
			*adapter_id = id;
			matched = true;
			break;
		}
	}
out:
	memzero_explicit(&prepared, sizeof(prepared));
	return matched;
}

static void xiaomi_mca_mipps_authenticate(struct xiaomi_mca *mca)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	const struct piano_mipps_key *key;
	u8 message[20], digest[SHA256_DIGEST_SIZE], response[MCA_VDM_LEN];
	__le32 words[MCA_VDM_LEN / 4], version, vbus;
	unsigned int generation = READ_ONCE(mca->service_generation);
	u32 attachment = READ_ONCE(mca->attach_generation);
	u32 adapter_id;
	bool discovered = false, capable = false;
	int i, ret;

	if (!mipps_auth || mca->auth_lost || mca->mipps_attempted || xiaomi_mca_authenticated(mca) ||
	    mca->hv != MCA_HV_OFF || mca->cp != MCA_CP_OFF ||
	    (mca->real_type != MCA_TYPE_PD &&
	     mca->real_type != MCA_TYPE_PD_VERIFY &&
	     mca->real_type != MCA_TYPE_PD_PPS))
		return;
	if (xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		return;
	for (i = 0; i < MCA_PD_MAX_PDOS; i++)
		capable |= le32_to_cpu(pdos[i].max_mv) > 5000;
	if (!capable || generation != READ_ONCE(mca->service_generation) ||
	    attachment != READ_ONCE(mca->attach_generation))
		return;

	mca->auth_attach_generation = attachment;
	mca->ffc_active = false;
	mca->auth_ok = false;
	mca->mipps_attempted = true;
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_VERIFY_PROCESS, 1);
	if (ret)
		goto finish;

	/* A Xiaomi VDM query works as UFP too. Never swap PC/hub data roles. */
	ret = xiaomi_mca_vdm(mca, generation, 1, NULL, 0);
	if (ret)
		goto finish;
	ret = xiaomi_mca_read_blob(mca, MCA_PROP_VDM_VERSION, &version, sizeof(version));
	if (ret || !le32_to_cpu(version) || le32_to_cpu(version) == U32_MAX) {
		ret = ret ?: -ENODEV;
		goto finish;
	}
	discovered = true;
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PD_VERIFIED, 0);
	if (ret)
		goto finish;
	for (i = 2; i <= 3; i++) {
		ret = xiaomi_mca_vdm(mca, generation, i, NULL, 0);
		if (ret)
			goto finish;
	}

	key = &piano_mipps_keys[get_random_u32_below(ARRAY_SIZE(piano_mipps_keys))];
	get_random_bytes(message, MCA_VDM_LEN);
	ret = xiaomi_mca_vdm(mca, generation, 4, key->seed, 0);
	if (ret)
		goto finish;
	ret = xiaomi_mca_vdm(mca, generation, 5, message, 0);
	if (ret)
		goto finish;
	ret = xiaomi_mca_read_blob(mca, MCA_PROP_VDM_AUTH, words, sizeof(words));
	if (ret)
		goto finish;
	for (i = 0; i < ARRAY_SIZE(words); i++)
		put_unaligned_be32(le32_to_cpu(words[i]), response + 4 * i);
	adapter_id = xiaomi_mca_read_u32(mca, MCA_PROP_ADAPTER_ID);
	if (!xiaomi_mca_mipps_digest(mca, key, message, response, digest,
				    &adapter_id, generation)) {
		ret = -EKEYREJECTED;
		goto finish;
	}

	/* Piano batterysecret sends 7 then 6. Command 8 stalls this firmware. */
	ret = xiaomi_mca_vdm(mca, generation, 7, NULL, 1);
	ret = ret ?: xiaomi_mca_vdm(mca, generation, 6, NULL, 1);
	if (ret)
		goto finish;
	ret = xiaomi_mca_read_blob(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus, sizeof(vbus));
	if (ret || le32_to_cpu(vbus) < MCA_VBUS_ONLINE_UV ||
	    !xiaomi_mca_auth_current(mca, generation)) {
		ret = ret ?: -ENODEV;
		goto finish;
	}
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PD_VERIFIED, 1);
	if (!ret && !xiaomi_mca_auth_current(mca, generation))
		ret = -ENODEV;
	if (!ret) {
		mca->auth_generation = generation;
		mca->auth_ok = true;
		mca->auth_at = jiffies;
		mca->auth_tried = false;
	}
finish:
	mca->auth_retry_ms = !ret ? 0 : mca->auth_retry_ms ?
		min(2 * mca->auth_retry_ms, 300000U) : MCA_HV_RETRY_MS;
	mca->auth_retry = jiffies + msecs_to_jiffies(mca->auth_retry_ms);
	if (generation == READ_ONCE(mca->service_generation)) {
		if (ret && discovered) {
			xiaomi_mca_write_u32(mca, MCA_PROP_PD_VERIFIED, 0);
			if (xiaomi_mca_auth_current(mca, generation))
				xiaomi_mca_vdm(mca, generation, 7, NULL, 0);
		}
		/* An unconfirmed clear blocks voltage negotiation until it succeeds. */
		mca->auth_cleanup = true;
		xiaomi_mca_clear_auth(mca);
	}
	memzero_explicit(message, sizeof(message));
	memzero_explicit(digest, sizeof(digest));
	memzero_explicit(response, sizeof(response));
	if (!ret)
		dev_info(mca->dev, "MiPPS adapter %08x authenticated\n", adapter_id);
	else if (discovered)
		dev_info(mca->dev,
			 "MiPPS authentication failed (%d); using standard PD/PPS\n", ret);
	power_supply_changed(mca->psy);
}


static const char *xiaomi_mca_type_name(u32 type)
{
	return type < MCA_TYPE_MAX ? mca_real_type_names[type] : NULL;
}

static u32 xiaomi_mca_policy_icl(u32 real_type)
{
	const struct xiaomi_mca_limits *limits = xiaomi_mca_limits(real_type);
	u32 target;

	if (!limits)
		return 0;
	/* Initial ceilings before checking Rp or an explicit contract. */
	switch (real_type) {
	case MCA_TYPE_FLOAT:
		target = 1000;
		break;
	case MCA_TYPE_CDP:
		target = 900;
		break;
	case MCA_TYPE_DCP:
	case MCA_TYPE_DCP_B:
	case MCA_TYPE_HVDCP:
	case MCA_TYPE_HVDCP_3:
	case MCA_TYPE_HVDCP_3_B:
	case MCA_TYPE_HVDCP_3P5:
	case MCA_TYPE_PD:
	case MCA_TYPE_PD_VERIFY:
	case MCA_TYPE_PD_PPS:
		target = 1500;
		break;
	default:
		target = MCA_POLICY_ICL_DEFAULT_MA;
		break;
	}
	return min(target, limits->input_current_ma);
}

/* The buck's autonomous 5 V path uses a fixed PDO, never a PPS APDO. */
static u32 xiaomi_mca_pd_5v_ma(struct xiaomi_mca *mca)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	int i;

	if (!xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		for (i = 0; i < MCA_PD_MAX_PDOS; i++)
			if (le32_to_cpu(pdos[i].min_mv) == 5000 &&
			    le32_to_cpu(pdos[i].max_mv) == 5000 &&
			    le32_to_cpu(pdos[i].max_ma))
				return le32_to_cpu(pdos[i].max_ma);

	return MCA_POLICY_ICL_DEFAULT_MA;
}

/*
 * The ADSP keeps the last limit written for the next source, so put its
 * power-on value back on detach and unbind.  Retried by the next poll if
 * the write fails.
 */
static void xiaomi_mca_reset_icl(struct xiaomi_mca *mca)
{
	xiaomi_mca_restore_icl(mca);
}

/* The ADSP keeps the charge current too; put the fallback value back */
static int xiaomi_mca_reset_fcc(struct xiaomi_mca *mca)
{
	int ret = xiaomi_mca_write_u32(mca, MCA_PROP_CHARGE_CURRENT, MCA_FCC_DEFAULT_MA);

	if (!ret) {
		mca->fcc_set_ma = 0;
		mca->fcc_written = false;
	}
	return ret;
}

/* Stock buck strategy chg_* charge currents (mA) per charger type */
static u32 xiaomi_mca_policy_fcc(u32 real_type, bool hv)
{
	const struct xiaomi_mca_limits *limits = xiaomi_mca_limits(real_type);

	if (!limits)
		return 0;
	if (hv) {
		const struct xiaomi_mca_limits *high = xiaomi_mca_hv_limits(real_type);

		if (high)
			return high->charge_current_ma;
	}
	/* The community policy does not raise FCC for unclassified Type-C. */
	if (real_type == MCA_TYPE_C)
		return MCA_FCC_DEFAULT_MA;
	return limits->charge_current_ma;
}

/* Charge current for @full at battery temperature @tbat */
static u32 xiaomi_mca_fcc_thermal(struct xiaomi_mca *mca, u32 full, int tbat)
{
	/* CN mca_buckchg_jeita. Firmware and charge_limit_poll own stop. */
	if (tbat < -9 || tbat >= 55)
		return min_t(u32, full, MCA_FCC_DEFAULT_MA);
	if (tbat < 3)
		return min_t(u32, full, 880);
	if (tbat < 8 || (mca->zone_valid && mca->cell_zone < 8))
		return min_t(u32, full, 1768);
	/* All other active JEITA rows allow >= 4520 mA, above buck max. */
	return full;
}

static void xiaomi_mca_apply_fcc(struct xiaomi_mca *mca, int tbat, u32 icl)
{
	u32 full = min_t(u32, xiaomi_mca_policy_fcc(mca->real_type, mca->hv == MCA_HV_ON),
			 MCA_FCC_MAX_MA);
	u32 target;

	if (mca->real_type == MCA_TYPE_C || xiaomi_mca_probe_5v(mca->real_type) ||
	    (xiaomi_mca_is_pd(mca->real_type) && mca->hv != MCA_HV_ON))
		full = icl;
	else if (xiaomi_mca_is_pd(mca->real_type) && mca->hv == MCA_HV_ON)
		full = min(2 * icl, (u32)MCA_CP_PAR_FCC_MAX_MA);
	target = xiaomi_mca_fcc_thermal(mca, full, tbat);

	if (target == mca->fcc_set_ma)
		return;

	if (xiaomi_mca_write_u32(mca, MCA_PROP_CHARGE_CURRENT, target)) {
		dev_warn(mca->dev, "failed to set charge current %u mA\n", target);
		return;
	}

	dev_info(mca->dev, "%s: charge current %u mA (battery %d degC)\n",
		 xiaomi_mca_type_name(mca->real_type) ?: "Unknown", target, tbat);
	mca->fcc_set_ma = target;
}

static bool xiaomi_mca_is_pd(u32 real_type)
{
	return real_type == MCA_TYPE_PD || real_type == MCA_TYPE_PD_VERIFY ||
	       real_type == MCA_TYPE_PD_PPS;
}

enum xiaomi_mca_ucsi_state {
	MCA_UCSI_UNAVAILABLE,
	MCA_UCSI_OFFLINE,
	MCA_UCSI_CONNECTED,
	MCA_UCSI_RP_1500,
	MCA_UCSI_RP_3000,
	MCA_UCSI_PD_SINK,
};

struct xiaomi_mca_contract {
	struct xiaomi_mca *mca;
	int mv;
};

static bool xiaomi_mca_ucsi_peer(struct xiaomi_mca *mca, struct power_supply *psy)
{
	struct device *parent = psy->dev.parent;

	return psy != mca->psy && parent && parent->parent == mca->dev->parent &&
	       psy->desc->type == POWER_SUPPLY_TYPE_USB;
}

/* UCSI CURRENT_NOW is RDO operating current. A source PDO alone does not
 * authorize drawing its advertised maximum. Check voltage, role and type.
 */
static int xiaomi_mca_pd_contract(struct power_supply *psy, void *data)
{
	struct xiaomi_mca_contract *want = data;
	union power_supply_propval val;

	if (!xiaomi_mca_ucsi_peer(want->mca, psy))
		return 0;
	guard(rwsem_read)(&psy->epc_sem);
	if (psy->removing ||
	    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_ONLINE, &val) ||
	    val.intval != 1 ||
	    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_STATUS, &val) ||
	    val.intval != POWER_SUPPLY_STATUS_CHARGING ||
	    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_USB_TYPE, &val) ||
	    val.intval != POWER_SUPPLY_USB_TYPE_PD ||
	    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) ||
	    val.intval != want->mv * 1000 ||
	    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val) ||
	    val.intval < 100000 || val.intval > 5000000)
		return 0;
	return val.intval / 1000;
}

static u32 xiaomi_mca_contract_ma(struct xiaomi_mca *mca, int mv, u32 fallback)
{
	struct xiaomi_mca_contract want = { mca, mv };
	int ma = power_supply_for_each_psy(&want, xiaomi_mca_pd_contract);

	return ma > 0 ? ma : fallback;
}

/* The UCSI supply belongs to another child of this PMIC GLINK instance. */
static int xiaomi_mca_ucsi_status(struct power_supply *psy, void *data)
{
	struct xiaomi_mca *mca = data;
	union power_supply_propval val;

	if (!xiaomi_mca_ucsi_peer(mca, psy))
		return MCA_UCSI_UNAVAILABLE;
	/* UCSI frees its connector data after unregistering this supply. */
	guard(rwsem_read)(&psy->epc_sem);
	if (psy->removing)
		return MCA_UCSI_UNAVAILABLE;
	if (power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_ONLINE, &val))
		return MCA_UCSI_UNAVAILABLE;
	if (!val.intval)
		return MCA_UCSI_OFFLINE;
	if (power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
		return MCA_UCSI_CONNECTED;
	if (val.intval == POWER_SUPPLY_USB_TYPE_C) {
		if (power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_STATUS, &val) ||
		    val.intval != POWER_SUPPLY_STATUS_CHARGING ||
		    power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val))
			return MCA_UCSI_CONNECTED;
		if (val.intval == 3000000)
			return MCA_UCSI_RP_3000;
		if (val.intval == 1500000)
			return MCA_UCSI_RP_1500;
		return MCA_UCSI_CONNECTED;
	}
	if (val.intval != POWER_SUPPLY_USB_TYPE_PD && val.intval != POWER_SUPPLY_USB_TYPE_PD_PPS)
		return MCA_UCSI_CONNECTED;
	if (power_supply_get_property_direct(psy, POWER_SUPPLY_PROP_STATUS, &val) ||
	    val.intval != POWER_SUPPLY_STATUS_CHARGING)
		return MCA_UCSI_CONNECTED;
	return MCA_UCSI_PD_SINK;
}

static u32 xiaomi_mca_5v_input(struct xiaomi_mca *mca, u32 type)
{
	u32 ma = xiaomi_mca_policy_icl(type);
	int state;

	if (type == MCA_TYPE_C || xiaomi_mca_probe_5v(type)) {
		/* Piano's PMIH010x 5 V sink PDO is 3 A. Program an AICL
		 * ceiling; firmware's VBUS loop controls actual draw. An
		 * explicit Type-C or PD advertisement still takes priority.
		 */
		if (xiaomi_mca_probe_5v(type))
			ma = MCA_ICL_5V_MAX_MA;
		state = power_supply_for_each_psy(mca, xiaomi_mca_ucsi_status);
		if (state == MCA_UCSI_RP_1500)
			ma = 1500;
		else if (state == MCA_UCSI_RP_3000)
			ma = 3000;
	} else if (xiaomi_mca_is_pd(type)) {
		ma = xiaomi_mca_pd_5v_ma(mca);
		ma = min(ma, xiaomi_mca_contract_ma(mca, 5000, min(ma, 3000U)));
	}
	if (type == MCA_TYPE_UNKNOWN &&
	    (mca->unknown_polls < MCA_UNKNOWN_POLLS ||
	     mca->policy_attach_generation != READ_ONCE(mca->attach_generation)))
		ma = min_t(u32, ma, MCA_FLOAT_ICL_MA);
	if (mca->aicl_cap_ma) {
		ma = min(ma, mca->aicl_cap_ma);
		if (xiaomi_mca_probe_5v(type))
			ma = min(ma, xiaomi_mca_5v_floor(type));
	}
	return min_t(u32, ma, MCA_ICL_5V_MAX_MA);
}

static void xiaomi_mca_record_collapse(struct xiaomi_mca *mca)
{
	int state;

	if (!mca->aicl_probe_floor_ma)
		return;
	if (ktime_after(ktime_get_boottime(), mca->aicl_deadline) ||
	    mca->policy_attach_generation != READ_ONCE(mca->attach_generation)) {
		mca->aicl_probe_floor_ma = 0;
		return;
	}
	state = power_supply_for_each_psy(mca, xiaomi_mca_ucsi_status);
	if (state >= MCA_UCSI_CONNECTED &&
	    mca->policy_attach_generation == READ_ONCE(mca->attach_generation)) {
		mca->aicl_cap_ma = mca->aicl_probe_floor_ma;
		dev_info(mca->dev, "5 V input collapsed with cable attached; limiting to %u mA\n",
			 mca->aicl_cap_ma);
	}
	if (state != MCA_UCSI_UNAVAILABLE)
		mca->aicl_probe_floor_ma = 0;
}

static void xiaomi_mca_observe_aicl(struct xiaomi_mca *mca)
{
	u32 value;

	if (!mca->aicl_probe_floor_ma)
		return;
	if (ktime_after(ktime_get_boottime(), mca->aicl_deadline)) {
		mca->aicl_probe_floor_ma = 0;
		return;
	}
	if ((!xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &value) && value < MCA_VBUS_ONLINE_UV) ||
	    (!xiaomi_mca_read(mca, MCA_PROP_USB_ONLINE, &value) && !value))
		xiaomi_mca_record_collapse(mca);
}

static bool xiaomi_mca_charge_wake_locked(struct xiaomi_mca *mca, bool awake)
{
	lockdep_assert_held(&mca->rx_lock);
	awake = awake && mca->charge_ws && mca->service_up;
	if (mca->charge_awake == awake)
		return false;
	mca->charge_awake = awake;
	if (awake)
		__pm_stay_awake(mca->charge_ws);
	else
		__pm_relax(mca->charge_ws);
	return true;
}

/* Remember disconnects even when a complete replug fits between two polls. */
static void xiaomi_mca_ucsi_update(struct xiaomi_mca *mca, int state)
{
	guard(spinlock_irqsave)(&mca->rx_lock);

	if (state == MCA_UCSI_UNAVAILABLE)
		return;
	if (state != mca->ucsi_state) {
		if (state == MCA_UCSI_OFFLINE) {
			mca->attach_generation++;
			if (xiaomi_mca_charge_wake_locked(mca, false) && mca->poll_ready)
				power_supply_changed(mca->psy);
		} else if (mca->ucsi_state < MCA_UCSI_CONNECTED) {
			mca->otg_requested = false; /* UCSI ONLINE means connected as sink. */
		}
		mca->input_generation++;
		mca->input_pending = mca->service_up;
		if (mca->poll_ready)
			mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
	}
	mca->ucsi_state = state;
}

static int xiaomi_mca_psy_notifier(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct xiaomi_mca *mca = container_of(nb, struct xiaomi_mca, psy_nb);

	if (event == PSY_EVENT_PROP_CHANGED)
		xiaomi_mca_ucsi_update(mca, xiaomi_mca_ucsi_status(data, mca));
	return NOTIFY_OK;
}

static void xiaomi_mca_psy_notifier_remove(void *data)
{
	struct xiaomi_mca *mca = data;

	power_supply_unreg_notifier(&mca->psy_nb);
}

/*
 * REAL_TYPE can remain Unknown after a PD reconnect. The stock
 * qcom_adsp_pd_protocol driver names PD_STATE 2 "SNK_Ready", but the state
 * and cached PDOs alone do not establish a current PD connection. Also
 * require UCSI to report an online PD sink with its charging path enabled.
 * Classify it conservatively as ordinary PD; the existing policy still
 * checks source PDOs and temperature before negotiating voltage/current.
 */
static int xiaomi_mca_source_type(struct xiaomi_mca *mca, u32 *type, u32 *raw)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	__le32 val;
	int ret, i;

	ret = xiaomi_mca_read_blob(mca, MCA_PROP_USB_REAL_TYPE, &val, sizeof(val));
	if (ret)
		return ret;
	*type = le32_to_cpu(val);
	if (raw)
		*raw = *type;
	if (*type == MCA_TYPE_UNKNOWN || *type == MCA_TYPE_FLOAT || *type == MCA_TYPE_C ||
	    *type == MCA_TYPE_DCP) {
		int state = power_supply_for_each_psy(mca, xiaomi_mca_ucsi_status);

		if (state == MCA_UCSI_RP_1500 || state == MCA_UCSI_RP_3000) {
			*type = MCA_TYPE_C;
			return 0;
		}
		if (state != MCA_UCSI_PD_SINK)
			return 0;
	} else {
		return 0;
	}

	ret = xiaomi_mca_read_blob(mca, MCA_PROP_PD_STATE, &val, sizeof(val));
	if (ret || le32_to_cpu(val) != MCA_PD_SNK_READY)
		return ret;
	ret = xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos));
	if (ret)
		return ret;

	for (i = 0; i < MCA_PD_MAX_PDOS; i++) {
		u32 ma = le32_to_cpu(pdos[i].max_ma);

		if (le32_to_cpu(pdos[i].min_mv) == 5000 &&
		    le32_to_cpu(pdos[i].max_mv) == 5000 &&
		    ma >= MCA_ICL_MIN_MA && ma <= 5000) {
			*type = MCA_TYPE_PD;
			break;
		}
	}
	return 0;
}


/*
 * Current a PD source offers at 9 V, 0 if none: its fixed 9 V PDO when it
 * has one, as stock prefers, otherwise a PPS APDO whose range covers 9 V
 * (*pps set), capped at MCA_HV_PPS_MAX_MA.
 */
static u32 xiaomi_mca_pd_9v_ma(struct xiaomi_mca *mca, bool *pps)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 pps_ma = 0;
	int i;

	*pps = false;
	if (xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		return 0;

	for (i = 0; i < MCA_PD_MAX_PDOS; i++) {
		u32 min_mv = le32_to_cpu(pdos[i].min_mv);
		u32 max_mv = le32_to_cpu(pdos[i].max_mv);

		if (min_mv == MCA_HV_MV && max_mv == MCA_HV_MV &&
		    le32_to_cpu(pdos[i].max_ma) >= MCA_ICL_MIN_MA)
			return le32_to_cpu(pdos[i].max_ma);
		if (min_mv < max_mv && min_mv <= MCA_HV_MV && max_mv >= MCA_HV_MV)
			pps_ma = max(pps_ma, le32_to_cpu(pdos[i].max_ma));
	}

	if (pps_ma < MCA_HV_PPS_MIN_MA)
		return 0;

	*pps = true;
	return min_t(u32, pps_ma, MCA_HV_PPS_MAX_MA);
}

/*
 * Ask the source for @mv: 9 V through the kind of object it offers it in,
 * 5 V always as the fixed vSafe5V PDO, as stock does for PPS sources too.
 */
static int xiaomi_mca_request_volt(struct xiaomi_mca *mca, u32 mv)
{
	if (xiaomi_mca_is_qc(mca->real_type))
		return xiaomi_mca_write_u32(mca, MCA_PROP_QC_VOLT, mv);
	if (mv == MCA_HV_MV && mca->hv_pps)
		return xiaomi_mca_write_u32(mca, MCA_PROP_PPS_SELECT,
					   xiaomi_mca_pps_word(mv, mca->hv_pps_ma));

	return xiaomi_mca_write_u32(mca, MCA_PROP_PD_FIXED_VOLT, mv);
}

/*
 * Once authenticated, a MiPPS power bank stays at 5 V, refuses further PD
 * requests and resets itself about 15 s later; it is left alone until then,
 * after one try at direct charging (see xiaomi_mca_cp_after_auth()).
 */
static bool xiaomi_mca_auth_pending(struct xiaomi_mca *mca)
{
	return mipps_auth && xiaomi_mca_authenticated(mca) &&
	       !time_after(mca->cp_on_at, mca->auth_at) &&
	       time_before(jiffies, mca->auth_at +
				    msecs_to_jiffies(MCA_AUTH_RESET_WAIT_MS));
}

/*
 * hv_charge state machine, one step per poll.  Returns the input current
 * the source allows at 9 V once the bus is there, 0 while at 5 V.
 */
static u32 xiaomi_mca_hv_step(struct xiaomi_mca *mca)
{
	bool vbus_hv = mca->vbus_uv >= MCA_HV_VBUS_MIN_UV &&
		       mca->vbus_uv <= MCA_HV_VBUS_MAX_UV;
	bool pps = false;
	bool qc = xiaomi_mca_is_qc(mca->real_type);
	u32 ma = 0;
	int ret;

	if (!hv_charge)
		return 0;
	if (mca->hv == MCA_HV_COOLDOWN) {
		if (time_before(jiffies, mca->hv_retry))
			return 0;
		if (mca->vbus_uv >= MCA_HV_VBUS_5V_MAX_UV) {
			xiaomi_mca_request_volt(mca, 5000);
			mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_RENEG_MS);
			return 0;
		}
		mca->hv = MCA_HV_OFF;
	}

	if (xiaomi_mca_is_pd(mca->real_type))
		ma = xiaomi_mca_pd_9v_ma(mca, &pps);
	else if (qc)
		ma = xiaomi_mca_hv_limits(mca->real_type)->input_current_ma;

	/* The source must keep offering the negotiated protocol. */
	if (mca->hv != MCA_HV_OFF && pps != mca->hv_pps)
		ma = 0;

	switch (mca->hv) {
	case MCA_HV_OFF:
		/* not in the middle of an adapter authentication */
		if (!ma || time_before(jiffies, mca->hv_retry))
			return 0;
		if (!qc && (xiaomi_mca_auth_pending(mca) ||
			    xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS)))
			return 0;
		if (!qc && mca->icl_set_ma > min(ma, 1600U)) {
			u32 transition = min(ma, 1600U);

			if (xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, transition))
				return 0;
			mca->icl_set_ma = transition;
		}
		if (qc) {
			/* QC2 allows less input current at 9 V than at 5 V. Keep
			 * both limits valid throughout the voltage transition. */
			u32 transition = min(ma, xiaomi_mca_policy_icl(mca->real_type));

			if (xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, transition))
				return 0;
			mca->icl_set_ma = transition;
		}
		mca->hv_pps = pps;
		mca->hv_pps_ma = ma;
		ret = xiaomi_mca_request_volt(mca, MCA_HV_MV);
		if (ret) {
			mca->hv_last_exit = "request-refused";
			dev_warn(mca->dev, "9 V request refused: %d (adsp %d)\n",
				 ret, mca->retcode);
			break;
		}
		dev_info(mca->dev, "requested 9 V (%s %u mA)\n",
			 qc ? "QC" : pps ? "PPS" : "PDO", ma);
		mca->hv = MCA_HV_REQUESTED;
		mca->hv_polls = 0;
		return 0;
	case MCA_HV_REQUESTED:
		if (ma && vbus_hv) {
			dev_info(mca->dev, "bus at %d mV\n", mca->vbus_uv / 1000);
			mca->hv = MCA_HV_ON;
			return ma;
		}
		if (ma && ++mca->hv_polls < MCA_HV_SETTLE_POLLS)
			return 0;
		mca->hv_last_exit = "contract-not-reached";
		break;
	case MCA_HV_ON:
		if (ma && vbus_hv)
			return ma;
		mca->hv_last_exit = "contract-lost";
		if (ma && mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV) {
			dev_info(mca->dev, "source back at 5 V, asking again in %u s\n",
				 MCA_HV_RENEG_MS / 1000);
			mca->hv = MCA_HV_OFF;
			mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_RENEG_MS);
			return 0;
		}
		break;
	default:
		return 0;
	}

	/* Recover vSafe5V before another negotiation. */
	dev_info(mca->dev, "back to 5 V (bus %d mV)\n", mca->vbus_uv / 1000);
	xiaomi_mca_request_volt(mca, 5000);
	mca->hv = MCA_HV_COOLDOWN;
	mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_RENEG_MS);
	return 0;
}

/*
 * Stock buck strategy input current limits (mA) at 9 V: in_pd, and in_pps
 * for the two types the stock table names PD_PPS
 */
static u32 xiaomi_mca_hv_icl(u32 real_type)
{
	const struct xiaomi_mca_limits *limits = xiaomi_mca_hv_limits(real_type);

	return limits ? limits->input_current_ma : 0;
}

/* Piano's sub-PMIC has its own VBUS/AICL loop. Turn it on before
 * raising the input vote, then trigger a scan for the new ceiling.
 */
static int xiaomi_mca_aicl(struct xiaomi_mca *mca, bool rerun)
{
	u8 enabled = 1, value;

	if (rerun)
		return xiaomi_mca_write_blob(mca, MCA_PROP_RERUN_AICL, &enabled, sizeof(enabled));
	if (!xiaomi_mca_read_blob(mca, MCA_PROP_AICL_ENABLE, &value, sizeof(value)) && value == 1)
		return 0;
	return xiaomi_mca_write_blob(mca, MCA_PROP_AICL_ENABLE, &enabled, sizeof(enabled));
}

/*
 * charge_policy: raise the input current limit to what the detected charger
 * type allows, never above what a PD source advertises at the voltage in
 * use.  It does not follow the battery temperature (charge_current does), so
 * a warm battery is not left to carry the system; JEITA stays with the ADSP,
 * and its AICL still backs the limit off if the source sags.
 */
static void xiaomi_mca_apply_policy(struct xiaomi_mca *mca)
{
	u32 target = xiaomi_mca_policy_icl(mca->real_type);
	int tbat = mca->coolest_temp;
	u32 hv_ma;
	bool aicl, pps;
	int ret;

	if (!target && (mca->real_type != MCA_TYPE_UNKNOWN ||
			mca->vbus_uv < MCA_VBUS_ONLINE_UV || mca->vbus_uv >= MCA_HV_VBUS_5V_MAX_UV))
		return;

	hv_ma = xiaomi_mca_hv_step(mca);
	if (hv_ma && xiaomi_mca_is_pd(mca->real_type) && !mca->hv_pps)
		target = min3(hv_ma, xiaomi_mca_contract_ma(mca, MCA_HV_MV, 1600),
			      (u32)MCA_ICL_HV_MAX_MA);
	else if (hv_ma)
		target = min(hv_ma, xiaomi_mca_hv_icl(mca->real_type));
	else if (xiaomi_mca_is_qc(mca->real_type) && mca->hv == MCA_HV_REQUESTED)
		target = min(target, xiaomi_mca_hv_icl(mca->real_type));
	else if (mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV && mca->hv == MCA_HV_OFF)
		target = xiaomi_mca_5v_input(mca, mca->real_type);
	else if (xiaomi_mca_is_pd(mca->real_type))
		target = min(target, min(xiaomi_mca_pd_5v_ma(mca),
					 xiaomi_mca_pd_9v_ma(mca, &pps) ?: 500U));

	/* A QC2 source can still be at 9 V after a refused downgrade. */
	if (xiaomi_mca_is_qc(mca->real_type) && mca->vbus_uv >= MCA_HV_VBUS_5V_MAX_UV)
		target = min(target, xiaomi_mca_hv_icl(mca->real_type));
	if (mca->aicl_cap_ma && mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV)
		target = min(target, mca->aicl_cap_ma);
	aicl = mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV && mca->hv == MCA_HV_OFF &&
	       (xiaomi_mca_probe_5v(mca->real_type) || mca->real_type == MCA_TYPE_C ||
		xiaomi_mca_is_pd(mca->real_type));
	if (aicl && (target != mca->icl_set_ma || target != mca->fcc_set_ma)) {
		ret = xiaomi_mca_aicl(mca, false);
		if (ret && xiaomi_mca_probe_5v(mca->real_type)) {
			target = min(target, xiaomi_mca_5v_floor(mca->real_type));
			aicl = false;
			dev_warn_ratelimited(mca->dev, "AICL unavailable, using board input limit\n");
			ret = 0;
		}
		if (ret) {
			dev_warn_ratelimited(mca->dev, "cannot enable input current detection: %d\n", ret);
			return;
		}
	}
	if (charge_current)
		xiaomi_mca_apply_fcc(mca, min(tbat, mca->coolest_temp), target);
	if (target == mca->icl_set_ma)
		return;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, target);
	if (!ret && aicl)
		ret = xiaomi_mca_aicl(mca, true);
	if (ret && aicl && xiaomi_mca_probe_5v(mca->real_type)) {
		target = min(target, xiaomi_mca_5v_floor(mca->real_type));
		dev_warn_ratelimited(mca->dev, "AICL scan failed, using board input limit\n");
		ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, target);
	}
	if (ret) {
		dev_warn(mca->dev, "failed to set input limit %u mA: %d\n",
			 target, ret);
		return;
	}

	dev_info(mca->dev, "%s: input limit %u mA at %s (battery %d degC)\n",
		 xiaomi_mca_type_name(mca->real_type) ?: "Unknown", target,
		 hv_ma ? "9 V" : "5 V", tbat);
	mca->icl_set_ma = target;
}

static char *xiaomi_mca_supplied_to[] = { "bq27z561-0", "bq27z561-1" };

/*
 * Both fuel gauges: voltage and temperature extremes, total charging
 * current and largest absolute cell current. A low or overloaded cell
 * must not be hidden by the other cell's voltage or opposite current.
 */
static bool xiaomi_mca_cell_ocp(struct fwnode_handle *charger, struct fwnode_handle *gauge, int ua)
{
	struct fwnode_handle *pump, *pump_bus, *gauge_bus;
	bool exceeded;

	if (ua <= MCA_CP_CELL_OCP_MA * 1000)
		return false;
	if (ua > 9000000)
		return true; /* Above both Piano gauge thresholds. */

	/* The master gauge and SC8541 share SE5. Only the slave on SE11
	 * crosses its stock OCP between 7 A and 9 A.
	 */
	pump = fwnode_find_reference(charger, "xiaomi,charge-pump", 0);
	if (IS_ERR_OR_NULL(pump))
		return false;
	pump_bus = fwnode_get_parent(pump);
	gauge_bus = fwnode_get_parent(gauge);
	exceeded = pump_bus && gauge_bus && pump_bus != gauge_bus;
	fwnode_handle_put(gauge_bus);
	fwnode_handle_put(pump_bus);
	fwnode_handle_put(pump);
	return exceeded;
}

static void xiaomi_mca_sample_cells(struct xiaomi_mca *mca, struct xiaomi_mca_cells *cells)
{
	union power_supply_propval v, i, t;
	struct power_supply *psy;
	int n, ret;

	*cells = (struct xiaomi_mca_cells) { .cold = INT_MAX, .hot = INT_MIN, .vmin = INT_MAX };
	for (n = 0; n < ARRAY_SIZE(xiaomi_mca_supplied_to); n++) {
		psy = power_supply_get_by_name(xiaomi_mca_supplied_to[n]);
		if (!psy) {
			cells->status = -ENODEV;
			continue;
		}
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &t);
		if (!ret) {
			cells->hot = max(cells->hot, t.intval / 10);
			cells->cold = min(cells->cold, (t.intval < 0 ? t.intval - 9 : t.intval) / 10);
		}
		ret = ret ?: power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &v);
		ret = ret ?: power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &i);
		if (!ret)
			cells->cell_ocp |= xiaomi_mca_cell_ocp(dev_fwnode(mca->dev),
							       dev_fwnode(&psy->dev), abs(i.intval));
		power_supply_put(psy);
		if (ret) {
			cells->status = ret;
			continue;
		}
		cells->vmax = max(cells->vmax, v.intval / 1000);
		cells->vmin = min(cells->vmin, v.intval / 1000);
		cells->ibat += i.intval / 1000;
		cells->imax = max(cells->imax, abs(i.intval / 1000));
	}
}

static int xiaomi_mca_gauges(struct xiaomi_mca *mca, int *vbat, int *ibat, int *cold,
			     int *hot, int *vmin, int *imax, bool *ocp)
{
	struct xiaomi_mca_cells local;
	const struct xiaomi_mca_cells *cells = mca->poll_cells;

	if (!cells) {
		xiaomi_mca_sample_cells(mca, &local);
		cells = &local;
	}
	*vbat = cells->vmax;
	*ibat = cells->ibat;
	*cold = cells->cold;
	*hot = cells->hot;
	*vmin = cells->vmin;
	*imax = cells->imax;
	if (ocp)
		*ocp = cells->cell_ocp;
	return cells->status;
}

static void xiaomi_mca_update_zones(struct xiaomi_mca *mca, int cold, int warm)
{
	static const int boundaries[] = { 8, 13, 18 };
	int zone = 0, i;

	/* Cross promptly into a more restrictive row. Require 2 C of
	 * headroom before taking a higher-current row again.
	 */
	for (i = 0; i < ARRAY_SIZE(boundaries); i++) {
		int edge = boundaries[i];

		if (cold < edge + (mca->zone_valid && mca->cell_zone < edge ? 2 : 0))
			break;
		zone = edge;
	}
	mca->cell_zone = zone;
	/* The cold row permits a HIGHER float voltage. Enter it only once
	 * every sensor has cooled; withdraw it immediately at 18 C.
	 */
	if (!mca->zone_valid || warm >= 18 || warm <= 16)
		mca->cold_fv = warm < 18;
	if (warm >= 35)
		mca->ffc_warm = true;
	else if (warm <= 33)
		mca->ffc_warm = false;
	mca->zone_valid = true;
}

static void xiaomi_mca_select_profile(struct xiaomi_mca *mca, int tcool, int twarm)
{
	enum xiaomi_mca_profile profile;

	xiaomi_mca_update_zones(mca, tcool, twarm);
	mca->cp_normal_cv = !mca->cold_fv && mca->cell_zone < 18;

	if (twarm >= MCA_TERM_HOT_DEGC || mca->term_hot)
		profile = MCA_PROFILE_WARM;
	else if (mca->cell_zone < 13)
		profile = MCA_PROFILE_COLD;
	else if (mca->cell_zone < 18)
		profile = MCA_PROFILE_COOL;
	else
		profile = mca->ffc_active ? MCA_PROFILE_FFC : MCA_PROFILE_NORMAL;

	if (profile != mca->cp_profile) {
		mca->cp_profile = profile;
		mca->cp_stage = 0;
	}
}

/* Enter only after observing both cells actually take more than iterm.
 * A source advertisement by itself is no evidence of usable charge current.
 */
static bool xiaomi_mca_ffc_conditions(struct xiaomi_mca *mca, int tcool, int twarm,
				      int vbat, int vmin, int ibat)
{
	u32 online, type, attached = READ_ONCE(mca->attach_generation);
	int iterm = mca->ffc_warm ? 3164 : 2350;
	bool running;

	if (mca->ffc_attach != attached || vbat < 4350)
		mca->ffc_done = false;
	online = xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE);
	/* USB_ONLINE also drops during PD hard resets on the same cable. */
	if (!full_charge || mca->cell_zone < 18 || tcool < 18 || twarm >= MCA_TERM_HOT_DEGC ||
	    mca->policy_generation != READ_ONCE(mca->service_generation) ||
	    online != 1 ||
	    xiaomi_mca_read_type(mca, &type) ||
	    (!xiaomi_mca_limits(type) && type != MCA_TYPE_UNKNOWN) ||
	    type == MCA_TYPE_SDP || type == MCA_TYPE_CDP)
		return false;

	running = mca->ffc_active && mca->ffc_attach == attached &&
		  (mca->ffc_type == type || (xiaomi_mca_is_pd(mca->ffc_type) && xiaomi_mca_is_pd(type)));
	mca->ffc_type = type;
	mca->ffc_attach = attached;
	/* The cell endpoint applies even while the source changes contracts. */
	if (running && vbat >= MCA_TERM_FFC_MV - 9 && ibat <= iterm)
		mca->ffc_done = true;
	if (mca->ffc_done)
		return false;
	/* Entry proves the source can supply iterm. Its PDO and the measured
	 * current change during taper and pump teardown on the same cable.
	 * Continue monitoring the cell endpoint across that transition.
	 */
	if (running)
		return true;

	if (xiaomi_mca_is_pd(type) || type == MCA_TYPE_C || xiaomi_mca_probe_5v(type)) {
		u32 ma = 0, vbus;

		if (xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus) ||
		    vbus < MCA_VBUS_ONLINE_UV)
			return false;
		if (vbus < MCA_HV_VBUS_5V_MAX_UV) {
			ma = xiaomi_mca_5v_input(mca, type);
		} else if (xiaomi_mca_is_pd(type) && mca->cp != MCA_CP_ON &&
			   mca->cp != MCA_CP_OPENING && !mca->hv_pps) {
			bool pps;
			u32 offered, rdo;

			if (vbus < MCA_HV_VBUS_MIN_UV || vbus > MCA_HV_VBUS_MAX_UV)
				return false;
			offered = xiaomi_mca_pd_9v_ma(mca, &pps);
			if (pps && mca->hv == MCA_HV_ON && mca->hv_pps)
				rdo = mca->hv_pps_ma;
			else
				rdo = xiaomi_mca_contract_ma(mca, MCA_HV_MV, 0);
			if (!rdo || !offered)
				return false;
			ma = min3(offered, rdo, (u32)MCA_ICL_HV_MAX_MA);
		}
		if (ma && ma * (vbus / 1000) <= iterm * vmin)
			return false;
	}
	return ibat > iterm;
}

static int xiaomi_mca_restore_term_volt(struct xiaomi_mca *mca)
{
	int ret;

	if (!mca->term_written)
		return 0;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_TERM_VOLT, MCA_TERM_DEFAULT_MV);
	if (!ret) {
		mca->term_written = false;
		mca->term_hot = false;
	}
	return ret;
}

static int xiaomi_mca_full_charge_poll(struct xiaomi_mca *mca)
{
	int vbat, ibat, tcool = 0, tbat = 0, vmin, imax;
	u32 tpack, target, readback;
	int ret;

	lockdep_assert_held(&mca->lock);
	if (mca->sleeping)
		return -EHOSTDOWN;
	if (!full_charge || !charge_policy) {
		mca->ffc_active = false;
		if (charge_policy && mca->gauge_failures)
			return 0;
		return xiaomi_mca_restore_term_volt(mca);
	}

	ret = xiaomi_mca_read(mca, MCA_PROP_PACK_TBAT, &tpack);
	mca->fv_derated = !!ret;
	if (!ret && xiaomi_mca_gauges(mca, &vbat, &ibat, &tcool, &tbat, &vmin, &imax, NULL)) {
		/* Hold the last vote for one missed observation. A measured
		 * hot sensor always takes precedence over that grace period.
		 */
		tbat = max_t(int, (s32)tpack, tbat);
		if (tbat < MCA_TERM_HOT_DEGC) {
			if (tbat < MCA_TERM_COOL_DEGC)
				mca->term_hot = false;
			return 0;
		}
		ret = -ENODATA;
	}
	if (ret) {
		mca->ffc_active = false;
		mca->term_hot = true;
	} else {
		tbat = max_t(s32, tbat, tpack);
		tcool = min_t(s32, tcool, tpack);
		if (tbat >= MCA_TERM_HOT_DEGC)
			mca->term_hot = true;
		else if (tbat < MCA_TERM_COOL_DEGC)
			mca->term_hot = false;
		xiaomi_mca_update_zones(mca, tcool, tbat);
		mca->ffc_active = !mca->term_hot && xiaomi_mca_ffc_conditions(mca, tcool, tbat, vbat, vmin, ibat);
	}
	if (!ret)
		xiaomi_mca_select_profile(mca, tcool, tbat);
	target = mca->term_hot ? MCA_TERM_HOT_MV : MCA_TERM_FULL_MV;
	if (!mca->term_hot && mca->cell_zone < 18 && mca->cold_fv)
		target = MCA_TERM_COLD_MV;
	if (!mca->term_hot && mca->ffc_active)
		target = MCA_TERM_FFC_MV;

	/* Read every poll: detach or an ADSP restart can discard the vote. */
	ret = xiaomi_mca_read(mca, MCA_PROP_TERM_VOLT, &readback);
	if (!ret && readback == 2 * target - 3600) {
		mca->term_written = true;
		mca->ffc_voted = target == MCA_TERM_FFC_MV;
		return 0;
	}

	return xiaomi_mca_write_u32(mca, MCA_PROP_TERM_VOLT, target);
}

static bool xiaomi_mca_cp_active(struct xiaomi_mca *mca)
{
	return mca->cp == MCA_CP_OPENING || mca->cp == MCA_CP_ON ||
	       mca->cp == MCA_CP_STOPPING;
}

static int xiaomi_mca_cp_get(struct xiaomi_mca *mca,
			    enum power_supply_property psp, int *val)
{
	union power_supply_propval pv;
	int ret;

	ret = power_supply_get_property(mca->cp_psy, psp, &pv);
	if (!ret)
		*val = pv.intval;

	return ret;
}

static int xiaomi_mca_cp_set_online(struct xiaomi_mca *mca, bool on)
{
	union power_supply_propval pv = { .intval = on };

	return power_supply_set_property(mca->cp_psy, POWER_SUPPLY_PROP_ONLINE,
					 &pv);
}

static int xiaomi_mca_cp_set_busocp(struct xiaomi_mca *mca, u32 ma)
{
	union power_supply_propval pv = { .intval = ma * 1000 };

	return power_supply_set_property(mca->cp_psy,
					 POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
					 &pv);
}

static u32 xiaomi_mca_cp_capability_ma(u32 apdo_ma, bool verified)
{
	return min_t(u32, apdo_ma, verified ? MCA_CP_PPS_MAX_MA : MCA_CP_STD_PPS_MAX_MA);
}

/* milliamp budget at the requested PPS voltage, before buck sharing. */
static u32 xiaomi_mca_cp_power_limit(u32 source_ma, u32 mv)
{
	if (!mv)
		return 0;
	return min(source_ma, (u32)(MCA_CP_MAX_POWER_MW * 1000 / mv));
}

static u32 xiaomi_mca_cp_div2_limit(u32 source_ma)
{
	/* Clamp before multiplying so even a malformed capability cannot wrap. */
	return min_t(u32, source_ma, MCA_CP_IBAT_MAX_MA / 2) * 2;
}

/* Stock voltage resolution is 200 mV for QC3 class B and 20 mV for QC3.5.
 * Base every pulse on measured VBUS; an acknowledged pulse may not move it.
 */
static int xiaomi_mca_qc_adjust(struct xiaomi_mca *mca, u32 target)
{
	unsigned int step = mca->cp_type == MCA_TYPE_HVDCP_3P5 ? 20 : 200;
	int mv = mca->vbus_uv / 1000;
	u32 pulse;

	if (mv < 4500 || mv > mca->cp_max_mv + step)
		return -ERANGE;
	if (target > mv && mv + step <= mca->cp_max_mv)
		pulse = MCA_QC_UP;
	else if (target < mv && mv - step >= MCA_CP_PPS_MIN_MV)
		pulse = MCA_QC_DOWN;
	else
		return 0;
	mca->voltage_written = true;
	return xiaomi_mca_write_u32(mca, MCA_PROP_QC_PULSE, pulse);
}

static int xiaomi_mca_cp_request(struct xiaomi_mca *mca, u32 mv)
{
	u32 ma;
	int ret;

	mv = clamp(mv, (u32)MCA_CP_PPS_MIN_MV,
		   min_t(u32, mca->cp_max_mv, MCA_CP_PPS_MAX_MV));
	if (xiaomi_mca_is_qc_pump(mca->cp_type)) {
		mca->cp_request_ma = mca->cp_ma;
		mca->cp_mv = mca->vbus_uv / 1000;
		return xiaomi_mca_qc_adjust(mca, mv);
	}
	mv = rounddown(mv, 20);
	ma = rounddown(xiaomi_mca_cp_power_limit(mca->cp_ma, mv), 50);
	if (mv == mca->cp_mv && ma == mca->cp_request_ma)
		return 0;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PPS_SELECT, xiaomi_mca_pps_word(mv, ma));
	if (!ret) {
		mca->cp_mv = mv;
		mca->cp_request_ma = ma;
	}
	return ret;
}

/*
 * Turn the stage off and go back to the buck charger: the next poll asks
 * the source for 9 V again and puts the buck limits back.
 */
/* Buck charger share while direct charging */
static int xiaomi_mca_cp_set_buck(struct xiaomi_mca *mca, u32 icl_ma,
				 u32 fcc_ma)
{
	int ret;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, icl_ma);
	ret = ret ?: xiaomi_mca_write_u32(mca, MCA_PROP_CHARGE_CURRENT, fcc_ma);
	if (ret) {
		mca->icl_set_ma = 0;
		mca->fcc_set_ma = 0;
		return ret;
	}

	mca->icl_set_ma = icl_ma;
	mca->fcc_set_ma = fcc_ma;
	return 0;
}

static void xiaomi_mca_cp_backoff(struct xiaomi_mca *mca, const char *reason)
{
	if (mca->cp_retry_ms && mca->cp_last_exit && !strcmp(mca->cp_last_exit, reason))
		mca->cp_retry_ms = min(2 * mca->cp_retry_ms, 300000U);
	else
		mca->cp_retry_ms = MCA_HV_RENEG_MS;
	mca->cp_last_exit = reason;
	mca->cp = MCA_CP_COOLDOWN;
	mca->cp_retry_at = jiffies + msecs_to_jiffies(mca->cp_retry_ms);
}

static int xiaomi_mca_cp_stop(struct xiaomi_mca *mca, const char *why)
{
	int ret;

	if (!xiaomi_mca_cp_active(mca))
		return 0;

	if (mca->cp == MCA_CP_ON && time_after_eq(jiffies, mca->cp_running_at + 60 * HZ))
		mca->cp_retry_ms = 0;
	if (mca->cp != MCA_CP_STOPPING)
		mca->cp_stop_reason = why;
	ret = xiaomi_mca_cp_set_online(mca, false);
	if (ret == -ENODEV) {
		guard(rwsem_read)(&mca->cp_psy->epc_sem);
		if (mca->cp_psy->removing)
			ret = 0;
	}
	if (ret) {
		mca->cp = MCA_CP_STOPPING;
		dev_err(mca->dev, "failed to stop the switched-capacitor stage: %d\n", ret);
		return ret;
	}
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	mca->cp_exits++;
	why = mca->cp_stop_reason ?: why;
	mca->cp_stop_reason = NULL;
	dev_info(mca->dev, "direct charging stopped: %s\n", why);

	if (!strcmp(why, "taper complete")) {
		mca->cp = MCA_CP_OFF;
		mca->cp_last_exit = why;
		mca->cp_retry_ms = 0;
	} else {
		xiaomi_mca_cp_backoff(mca, why);
	}
	mca->cp_target_ma = 0;
	mca->hv = MCA_HV_OFF;
	mca->hv_retry = jiffies;
	return 0;
}

static int xiaomi_mca_restore_voltage(struct xiaomi_mca *mca)
{
	u32 vbus;
	int ret, wait_ret;

	if (mca->voltage_written) {
		/* Cancel a pending raise even if the bus still reads 5 V. */
		ret = xiaomi_mca_request_volt(mca, 5000);
		if (ret)
			return ret;
		/* Observe the existing voltage-settling budget; do not resend. */
		wait_ret = read_poll_timeout(xiaomi_mca_read, ret,
					     ret || vbus < MCA_HV_VBUS_5V_MAX_UV,
					     jiffies_to_usecs(MCA_POLL_INTERVAL),
					     jiffies_to_usecs(MCA_POLL_INTERVAL *
							      MCA_HV_SETTLE_POLLS),
					     false, mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
		if (ret || wait_ret)
			return ret ?: wait_ret;
	}
	mca->voltage_written = false;
	return 0;
}

/* ADSP real_type 17..19 describes wireless power. */
static bool xiaomi_mca_should_stay_awake(bool online, bool otg, u32 type)
{
	return online && !otg && type != MCA_TYPE_SDP && type != MCA_TYPE_CDP &&
	       (type < MCA_TYPE_BPP || type > MCA_TYPE_HPP);
}

/* Apply only a sample taken entirely after the last source/role or service
 * change. A rejected sample leaves the event queued for the next worker.
 */
static bool xiaomi_mca_apply_wired_sample(struct xiaomi_mca *mca, u32 generation,
					  u32 online, u32 type)
{
	guard(spinlock_irqsave)(&mca->rx_lock);
	if (generation != mca->input_generation || !mca->service_up || !mca->poll_ready)
		return false;
	if (online == U32_MAX)
		return false;
	mca->input_pending = false;
	return xiaomi_mca_charge_wake_locked(mca,
			xiaomi_mca_should_stay_awake(online == 1, mca->otg_requested, type));
}

static void xiaomi_mca_charge_wake_remove(void *data)
{
	struct xiaomi_mca *mca = data;

	__pm_relax(mca->charge_ws);
	wakeup_source_unregister(mca->charge_ws);
}

static int xiaomi_mca_charge_wake_init(struct xiaomi_mca *mca)
{
	if (!IS_ENABLED(CONFIG_PM_SLEEP))
		return 0;
	mca->charge_ws = wakeup_source_register(mca->dev, "xiaomi-mca-charge");
	if (!mca->charge_ws)
		return -ENOMEM;
	return devm_add_action_or_reset(mca->dev, xiaomi_mca_charge_wake_remove, mca);
}

static int xiaomi_mca_charge_enable(struct xiaomi_mca *mca, bool enable)
{
	u8 value = enable, readback;
	int ret;

	if (enable && mca->connector.tripped)
		return -EBUSY;
	/* Retain ownership after a lost disable acknowledgment. */
	if (!enable)
		mca->charge_disabled = true;
	ret = xiaomi_mca_read_blob(mca, MCA_PROP_CHARGING_ENABLED,
				   &readback, sizeof(readback));
	if (ret || readback != value)
		ret = xiaomi_mca_write_blob(mca, MCA_PROP_CHARGING_ENABLED,
					    &value, sizeof(value));
	if (!ret && enable)
		mca->charge_disabled = false;
	return ret;
}

static int xiaomi_mca_charge_stop(struct xiaomi_mca *mca)
{
	u32 vbus;
	int ret, buck_ret;

	ret = xiaomi_mca_cp_stop(mca, "battery protection");
	buck_ret = xiaomi_mca_charge_enable(mca, false);
	if (ret || buck_ret)
		return ret ?: buck_ret;
	/* Never reduce source voltage while CP shutdown is unconfirmed. */
	if (mca->voltage_written) {
		ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
		if (ret)
			return ret;
		/* As in quiesce, a confirmed detach needs no source request. */
		if (vbus < MCA_VBUS_ONLINE_UV)
			mca->voltage_written = false;
	}
	ret = xiaomi_mca_restore_voltage(mca);
	if (ret)
		return ret;
	mca->hv = MCA_HV_OFF;
	return 0;
}

/* Stock thermal-normal.conf and thermal-chg-only.conf VIRTUAL-SENSOR.
 * Only needed at the 60 C contact/50 C board cross-check.
 */
static int xiaomi_mca_connector_board(struct xiaomi_mca *mca)
{
	static const int weight[] = { 131, -9, 89, 232 };
	union power_supply_propval cell[2];
	s64 sum;
	int i, val, cold;

	for (i = 0; i < ARRAY_SIZE(cell); i++)
		if (!mca->batteries[i] || power_supply_get_property(mca->batteries[i],
					POWER_SUPPLY_PROP_TEMP, &cell[i]))
			return INT_MIN;
	cold = min(cell[0].intval, cell[1].intval);
	val = cold <= 150 || cold > 1000 ? cold : max(cell[0].intval, cell[1].intval);
	sum = (s64)500 * val * 100;
	for (i = 0; i < ARRAY_SIZE(weight); i++) {
		if (!IS_REACHABLE(CONFIG_IIO) ||
		    iio_read_channel_processed(mca->connector.ntc[i + 2], &val) < 0)
			return INT_MIN;
		sum += (s64)weight[i] * val;
	}
	return div_s64(sum, 1000) + 1600;
}

/* Piano connector_antiburn: the hotter contact selects its own slope. */
static bool xiaomi_mca_connector_trip(struct mca_connector *c, const int temp[2],
				      ktime_t now, bool powered, int board)
{
	s64 elapsed = ktime_ms_delta(now, c->sampled);
	bool continuous = temp[0] != INT_MIN && temp[1] != INT_MIN &&
			  c->valid[0] && c->valid[1] && elapsed > 0;
	bool rising[2];
	int i, hot;

	for (i = 0; i < 2; i++) {
		rising[i] = c->sampled && elapsed > 0 && c->valid[i] &&
			    temp[i] != INT_MIN && (s64)temp[i] - c->temp[i] >= 4 * elapsed;
		c->valid[i] = temp[i] != INT_MIN;
		if (c->valid[i])
			c->temp[i] = temp[i];
	}
	c->sampled = now;
	hot = c->temp[0] >= c->temp[1] ? 0 : 1;
	if (c->tripped)
		return !continuous || c->temp[hot] >= 55000 || rising[hot];
	return powered && (c->temp[hot] >= 65000 ||
			   (c->temp[hot] >= 60000 && (board == INT_MIN || board / 1000 <= 50)) ||
			   (c->temp[hot] >= 35000 && rising[hot]));
}

static void xiaomi_mca_connector_stop(struct xiaomi_mca *mca)
{
	u32 vbus, dummy = 0, type;
	int ret, i;

	mca->charge_blocked = true;
	ret = xiaomi_mca_cp_stop(mca, "connector overtemperature");
	if (ret)
		dev_warn_ratelimited(mca->dev, "connector pump stop failed: %d\n", ret);
	ret = xiaomi_mca_charge_enable(mca, false);
	if (ret)
		dev_warn_ratelimited(mca->dev, "connector buck stop failed: %d\n", ret);
	/* Both OTG source selectors in the Piano configuration are zero. */
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_ENABLE, 0);
	if (ret)
		dev_warn_ratelimited(mca->dev, "connector boost stop failed: %d\n", ret);

	/* Ordinary contract teardown may wait six seconds. Use the stock
	 * connector-specific rundown before operating the independent MOS.
	 */
	mca->hv = MCA_HV_OFF;
	if (!mca->voltage_written &&
	    !xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus) && vbus < 6000000)
		return;
	if (xiaomi_mca_read_type(mca, &type))
		return;
	/* The stock buck event votes for 5 V even on a PD adapter which
	 * does not implement Xiaomi's additional VSAFE0V VDM.
	 */
	if (xiaomi_mca_is_pd(type))
		xiaomi_mca_write_u32(mca, MCA_PROP_PD_FIXED_VOLT, 5000);
	for (i = 0; i < 5; i++) {
		if (xiaomi_mca_is_pd(type))
			ret = xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, MCA_PROP_VDM_VSAFE0V,
						       &dummy, 0);
		else if (xiaomi_mca_is_qc(type))
			ret = xiaomi_mca_write_u32(mca, MCA_PROP_QC_VOLT, 5000);
		else
			break;
		if (ret)
			break;
		msleep(200);
		if (xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus))
			break;
		if (vbus < 6000000) {
			mca->voltage_written = false;
			break;
		}
	}
}

static bool xiaomi_mca_connector_poll(struct xiaomi_mca *mca)
{
	struct mca_connector *c = &mca->connector;
	int temp[2], ret, i, board = INT_MIN;
	bool hot, powered;
	u32 online;
	u8 otg = 0, cid = 0;

	if (!c->cutoff)
		return false;
	for (i = 0; i < ARRAY_SIZE(temp); i++) {
		temp[i] = INT_MIN;
		if (IS_REACHABLE(CONFIG_IIO) && iio_read_channel_processed(c->ntc[i], &temp[i]) < 0)
			temp[i] = INT_MIN;
	}
	/* On a cold connector no source query is needed for protection. */
	powered = false;
	if (max(max(temp[0], temp[1]), max(c->temp[0], c->temp[1])) >= 35000 || c->tripped) {
		ret = xiaomi_mca_read_blob(mca, MCA_PROP_OTG_PLUGIN, &otg, sizeof(otg));
		/* Unknown role must not mask a hot port. */
		powered = ret || otg || READ_ONCE(mca->otg_requested);
		online = xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE);
		powered |= online != 0;
		if (!powered)
			powered = xiaomi_mca_read_blob(mca, MCA_PROP_CID_STATUS, &cid, sizeof(cid)) || cid;
	}
	if (powered && max(temp[0], temp[1]) >= 60000)
		board = xiaomi_mca_connector_board(mca);
	hot = xiaomi_mca_connector_trip(c, temp, ktime_get_boottime(), powered, board);
	if (hot) {
		c->tripped = true;
		xiaomi_mca_connector_stop(mca);
	}
	if (hot || c->tripped) {
		ret = gpiod_set_value_cansleep(c->cutoff, hot);
		if (ret) {
			dev_err_ratelimited(mca->dev, "connector cutoff failed: %d\n", ret);
			return true;
		}
		if (hot) {
			u32 vbus;

			msleep(50);
			ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
			if (ret || vbus >= 4100000)
				dev_err_ratelimited(mca->dev, "connector VBUS cutoff unconfirmed\n");
		}
		c->tripped = hot;
	}
	return hot;
}

static int xiaomi_mca_charge_limit_poll(struct xiaomi_mca *mca)
{
	union power_supply_propval val;
	struct power_supply *psy;
	unsigned int start, end;
	int soc = 0, n, ret = 0, stop_ret, pack_ret, cell_ret;
	int vbat, ibat, tcool, tbat, vmin, imax;
	u32 tpack;

	lockdep_assert_held(&mca->lock);
	if (mca->sleeping)
		return -EHOSTDOWN;
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		start = mca->charge_start;
		end = mca->charge_end;
	}
	if (end == 100)
		mca->charge_hold = false;
	if (end == 100 && !charge_policy && !mca->charge_disabled) {
		mca->charge_blocked = false;
		return 0;
	}
	if (end < 100) {
		/* One charger feeds both cells: the higher SOC controls the limit. */
		for (n = 0; n < ARRAY_SIZE(xiaomi_mca_supplied_to); n++) {
			psy = power_supply_get_by_name(xiaomi_mca_supplied_to[n]);
			if (!psy) {
				ret = -ENODEV;
				break;
			}
			ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
			power_supply_put(psy);
			if (ret)
				break;
			if (val.intval < 0 || val.intval > 100) {
				ret = -ENODATA;
				break;
			}
			soc = max(soc, val.intval);
		}
		if (!ret) {
			if (soc >= end)
				mca->charge_hold = true;
			else if (soc <= start)
				mca->charge_hold = false;
		}
	}
	pack_ret = xiaomi_mca_read(mca, MCA_PROP_PACK_TBAT, &tpack);
	cell_ret = xiaomi_mca_gauges(mca, &vbat, &ibat, &tcool, &tbat, &vmin, &imax, NULL);
	if (!pack_ret) {
		tbat = max_t(int, tbat, (s32)tpack);
		tcool = min_t(int, tcool, (s32)tpack);
	}
	/* A surviving thermometer can assert a hold. Clearing one requires
	 * a complete sample.
	 */
	if (tbat >= 55)
		mca->charge_hot = true;
	if (tcool < -9)
		mca->charge_cold = true;
	ret = ret ?: pack_ret ?: cell_ret;
	if (!ret && !(full_charge && charge_policy && mca->fv_derated)) {
		mca->coolest_temp = tcool;
		if (!full_charge)
			xiaomi_mca_update_zones(mca, tcool, tbat);
		/* Firmware JEITA remains enabled throughout. */
		if (tbat < 53)
			mca->charge_hot = false;
		if (tcool >= -7)
			mca->charge_cold = false;
	}
	if (ret || (full_charge && charge_policy && mca->fv_derated)) {
		int err = ret ?: -ENODATA;

		mca->charge_blocked = true;
		mca->gauge_failures = min(mca->gauge_failures + 1, 3U);
		if (mca->charge_hold || mca->charge_hot || mca->charge_cold) {
			stop_ret = xiaomi_mca_charge_stop(mca);
			return stop_ret ?: err;
		}
		/* No new voltage/current or watchdog kick on a single miss.
		 * The next tick uses a fresh pair of gauge readings.
		 */
		if (mca->gauge_failures < 2 && !pack_ret && !mca->fv_derated &&
		    tbat < MCA_TERM_HOT_DEGC)
			return err;

		ret = xiaomi_mca_cp_stop(mca, "gauge unavailable");
		if (!ret) {
			u32 target = mca->term_hot || tbat >= MCA_TERM_HOT_DEGC ||
				     pack_ret || mca->fv_derated ? MCA_TERM_HOT_MV : MCA_TERM_DEFAULT_MV;
			u32 readback;

			mca->ffc_active = false;
			if (xiaomi_mca_read(mca, MCA_PROP_TERM_VOLT, &readback) ||
			    readback != 2 * target - 3600)
				ret = xiaomi_mca_write_u32(mca, MCA_PROP_TERM_VOLT, target);
			if (ret) {
				xiaomi_mca_charge_enable(mca, false);
				return ret;
			}
			mca->ffc_voted = false;
		}
		if (end < 100 && mca->gauge_failures >= 3) {
			stop_ret = xiaomi_mca_charge_stop(mca);
			return ret ?: stop_ret ?: err;
		}
		if (!ret && (mca->fcc_set_ma != MCA_FCC_DEFAULT_MA || mca->fcc_written)) {
			ret = xiaomi_mca_reset_fcc(mca);
			if (!ret)
				mca->fcc_set_ma = MCA_FCC_DEFAULT_MA;
		}
		if (!ret && xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE) == 1) {
			u32 icl, type, input;

			ret = xiaomi_mca_read_type(mca, &type);
			if (ret)
				return ret;
			input = MCA_FCC_DEFAULT_MA;
			if ((type == MCA_TYPE_FLOAT || type == MCA_TYPE_UNKNOWN) &&
			    xiaomi_mca_read_u32(mca, MCA_PROP_USB_VOLTAGE_NOW) < MCA_HV_VBUS_5V_MAX_UV)
				input = MCA_FLOAT_ICL_MA;
			ret = xiaomi_mca_read(mca, MCA_PROP_INPUT_CURRENT_LIMIT, &icl);
			if (!ret && icl != input && (xiaomi_mca_limits(type) || type == MCA_TYPE_UNKNOWN)) {
				/* The write gate rechecks type, voltage and sink online. */
				if (type == MCA_TYPE_UNKNOWN)
					mca->real_type = type;
				/* Replace both the 100 mA boot vote and a stale
				 * high-current vote inherited from a previous source.
				 */
				ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, input);
				if (!ret)
					mca->icl_set_ma = input;
			}
		}
		if (!ret)
			ret = xiaomi_mca_charge_enable(mca, true);
		return ret ?: err;
	}
	mca->gauge_failures = 0;
	mca->charge_blocked = mca->charge_hold || mca->charge_hot || mca->charge_cold;
	if (mca->charge_blocked) {
		stop_ret = xiaomi_mca_charge_stop(mca);
		return ret ?: stop_ret;
	}
	/* full_charge only selects FV; it never overrides SOC/thermal stops. */
	ret = xiaomi_mca_charge_enable(mca, true);
	return ret;
}

/*
 * A source that drops out under direct charging (a hard reset; the request
 * in flight may be refused first) did not take that much current, whatever
 * it advertised: aim lower from then on, unless it is away for longer than
 * MCA_CP_DROP_FORGET_MS (a replug).  The current is what the stage and the
 * buck charger took together.
 */
static void xiaomi_mca_cp_learn_drop(struct xiaomi_mca *mca)
{
	/* not the reset an authenticated adapter may do by itself */
	if (mca->cp != MCA_CP_ON || xiaomi_mca_auth_pending(mca) ||
	    mca->cp_ibus_last <= 2 * MCA_CP_DROP_BACKOFF_MA)
		return;

	mca->cp_ibus_cap = mca->cp_ibus_last - MCA_CP_DROP_BACKOFF_MA;
	mca->cp_cap_retry = jiffies + msecs_to_jiffies(MCA_HV_RETRY_MS);
	dev_warn(mca->dev, "source dropped out at %u mA, retrying from %u mA\n",
		 mca->cp_ibus_last, mca->cp_ibus_cap);
}

/* Re-test a learned limit on the running stage only after the source has
 * carried that load. A new drop updates the ceiling and restarts backoff.
 */
static void xiaomi_mca_cp_recover_current(struct xiaomi_mca *mca)
{
	u32 cap = mca->cp_ibus_cap;

	if (!cap || mca->cp != MCA_CP_ON || time_before(jiffies, mca->cp_cap_retry) ||
	    mca->cp_ibus_last + MCA_CP_DROP_BACKOFF_MA < cap)
		return;
	cap += MCA_CP_DROP_BACKOFF_MA;
	mca->cp_ibus_cap = cap < mca->cp_ma ? cap : 0;
	mca->cp_cap_retry = jiffies + msecs_to_jiffies(MCA_HV_RETRY_MS);
}

/* What the source gives: its APDO current, less while recovering from dropout. */
static u32 xiaomi_mca_cp_src_ma(struct xiaomi_mca *mca)
{
	u32 ma = mca->cp_ibus_cap ? min(mca->cp_ma, mca->cp_ibus_cap) : mca->cp_ma;

	return xiaomi_mca_cp_power_limit(ma, max_t(u32, mca->cp_mv, MCA_CP_PPS_MIN_MV));
}

static u32 xiaomi_mca_cp_vbat_max(struct xiaomi_mca *mca)
{
	unsigned int profile = mca->cp_profile;
	u32 cv = xiaomi_mca_profile_steps[profile][xiaomi_mca_profile_counts[profile] - 1].mv;

	if (mca->cp_normal_cv)
		cv = min_t(u32, cv, MCA_CP_VBAT_MAX_MV);
	return full_charge ? cv : min_t(u32, cv, MCA_TERM_DEFAULT_MV - MCA_CP_VBAT_HYS_MV);
}

/* The PPS APDO direct charging can use, 0 if none */
static u32 xiaomi_mca_cp_apdo(struct xiaomi_mca *mca, u32 start_mv, u32 *max_mv)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 ma = 0;
	int i;

	if (xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		return 0;

	for (i = 0; i < MCA_PD_MAX_PDOS; i++) {
		u32 min_mv = le32_to_cpu(pdos[i].min_mv);
		u32 hi_mv = le32_to_cpu(pdos[i].max_mv);

		if (min_mv < hi_mv && min_mv <= start_mv &&
		    hi_mv >= start_mv &&
		    le32_to_cpu(pdos[i].max_ma) > ma) {
			ma = le32_to_cpu(pdos[i].max_ma);
			*max_mv = hi_mv;
		}
	}

	return ma;
}

/* The ADSP holds the adapter verified until the next authentication */
static bool xiaomi_mca_adapter_verified(struct xiaomi_mca *mca)
{
	return !mca->auth_lost && mipps_auth && xiaomi_mca_authenticated(mca) &&
	       xiaomi_mca_read_u32(mca, MCA_PROP_PD_VERIFIED) == 1;
}

/*
 * Direct charging after the authentication, from 5 V by PPS and without
 * asking for 9 V first.  A Xiaomi adapter does not reset: it stays at 5 V
 * and the charger firmware offers its full APDO once it is verified, so
 * that is tried at once; a source that refuses it is asked again after
 * its reset, or once the wait for it is over.
 */
static bool xiaomi_mca_cp_after_auth(struct xiaomi_mca *mca)
{
	if (mca->hv != MCA_HV_OFF || time_before(jiffies, mca->hv_retry) ||
	    mca->vbus_uv >= MCA_HV_VBUS_5V_MAX_UV)
		return false;
	if (!mipps_auth || (mca->mipps_attempted && !mca->auth_cleanup &&
			    !xiaomi_mca_authenticated(mca)))
		return true;
	if (xiaomi_mca_authenticated(mca) && time_after(mca->cp_on_at, mca->auth_at))
		return true;
	if (xiaomi_mca_auth_pending(mca) && mca->auth_tried)
		return false;
	return xiaomi_mca_adapter_verified(mca);
}

/*
 * Battery current limit (both cells) for the battery voltage and the
 * coolest cell temperature within the direct-charge window
 */
static u32 xiaomi_mca_cp_ibat_limit(struct xiaomi_mca *mca, int vbat)
{
	const struct xiaomi_mca_charge_step *steps;
	u32 ma = cp_ibat_max ? clamp(cp_ibat_max, 1000U, (u32)MCA_CP_IBAT_MAX_MA) :
		MCA_CP_IBAT_MAX_MA;

	steps = xiaomi_mca_profile_steps[mca->cp_profile];
	ma = min(ma, xiaomi_mca_cp_div2_limit(xiaomi_mca_cp_src_ma(mca)));

	while (mca->cp_stage + 1 < xiaomi_mca_profile_counts[mca->cp_profile] &&
	       vbat >= steps[mca->cp_stage].mv)
		mca->cp_stage++;
	ma = min(ma, steps[mca->cp_stage].ma);

	mca->cp_target_ma = ma;
	return ma;
}

/* Called from the poll at 9 V on the buck charger, or right after the auth */
static void xiaomi_mca_cp_try_start(struct xiaomi_mca *mca)
{
	int vbat, ibat, tcool, tbat, tpack, ret, vmin, imax;
	u32 start_mv, ma, src_ma, vbat_max, max_mv = 0;

	if (mca->cp == MCA_CP_COOLDOWN && time_after_eq(jiffies, mca->cp_retry_at))
		mca->cp = MCA_CP_OFF;
	if (mca->cp_fault || mca->ffc_done)
		return;
	if (!cp_charge || !hv_charge || !charge_current ||
	    (!xiaomi_mca_is_pd(mca->real_type) && !xiaomi_mca_is_qc_pump(mca->real_type)) ||
	    mca->cp != MCA_CP_OFF ||
	    (mca->hv != MCA_HV_ON &&
	     (!xiaomi_mca_is_pd(mca->real_type) || !xiaomi_mca_cp_after_auth(mca))))
		return;

	if (xiaomi_mca_gauges(mca, &vbat, &ibat, &tcool, &tbat, &vmin, &imax, NULL))
		return;
	if (xiaomi_mca_read(mca, MCA_PROP_PACK_TBAT, &tpack))
		return;
	xiaomi_mca_select_profile(mca, min(tcool, tpack), max(tbat, tpack));
	vbat_max = xiaomi_mca_cp_vbat_max(mca);
	if ((mca->cp_recharge_cv >= vbat_max && vbat >= 4350) || vmin < MCA_CP_VBAT_MIN_MV ||
	    vbat >= vbat_max - MCA_CP_VBAT_HYS_MV ||
	    imax >= MCA_CP_CELL_OCP_MA ||
	    mca->cell_zone < MCA_CP_TBAT_MIN ||
	    max(tbat, tpack) >= MCA_CP_TBAT_EXIT ||
	    xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS))
		return;

	start_mv = 2 * vbat + MCA_CP_DELTA_MV;
	mca->cp_type = mca->real_type;
	if (xiaomi_mca_is_qc_pump(mca->cp_type)) {
		/* Piano QC class limits. QC has no PPS current advertisement. */
		bool qc35 = mca->cp_type == MCA_TYPE_HVDCP_3P5;

		max_mv = qc35 ? 10000 : 9500;
		ma = qc35 ? 2100 : 2500;
	} else {
		ma = xiaomi_mca_cp_apdo(mca, start_mv, &max_mv);
	}
	if (ma < MCA_CP_BUCK_MA + 1000)
		return;

	/* The advertised current bounds both the buck and the pump. */
	mca->cp_ma = xiaomi_mca_cp_capability_ma(ma, xiaomi_mca_adapter_verified(mca));
	mca->cp_max_mv = max_mv;
	mca->cp_mv = start_mv;
	/* Size hardware protection for the present contract. The live
	 * regulator separately applies the learned ceiling on every step.
	 */
	src_ma = xiaomi_mca_cp_power_limit(mca->cp_ma, start_mv);
	src_ma -= min_t(u32, src_ma, MCA_CP_BUCK_MA + 200);
	mca->cp_ibus_ma = min_t(u32, MCA_CP_STAGE_IBUS_MA, src_ma);
	if (xiaomi_mca_cp_src_ma(mca) < MCA_CP_BUCK_MA + 200 + 2 * MCA_CP_OPEN_IBUS_MA) {
		/* A learned limit below the opening current cannot be tested
		 * from a running pump. Retry from the normal opening ramp.
		 */
		if (mca->cp_ibus_cap && time_after_eq(jiffies, mca->cp_cap_retry))
			mca->cp_ibus_cap = 0;
		xiaomi_mca_cp_backoff(mca, "opening current unavailable");
		return;
	}

	mca->cp_psy = power_supply_get_by_name("sc8541");
	if (!mca->cp_psy) {
		dev_warn_ratelimited(mca->dev, "SC8541 unavailable, using buck charging\n");
		xiaomi_mca_cp_backoff(mca, "pump unavailable");
		return;
	}
	if (mca->cp_die_hot) {
		int tdie;

		if (xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_TEMP, &tdie) ||
		    tdie >= MCA_CP_TDIE_HOT) {
			power_supply_put(mca->cp_psy);
			mca->cp_psy = NULL;
			xiaomi_mca_cp_backoff(mca, "stage temperature");
			return;
		}
		mca->cp_die_hot = false;
	}
	/* it must be off before the source is asked for anything */
	if (xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_ONLINE, &ret) || ret ||
	    xiaomi_mca_cp_set_busocp(mca, min(mca->cp_ibus_ma + MCA_CP_BUSOCP_MARGIN_MA,
					     MCA_CP_BUSOCP_MAX_MA))) {
		dev_warn(mca->dev, "switched-capacitor stage not ready\n");
		power_supply_put(mca->cp_psy);
		mca->cp_psy = NULL;
		xiaomi_mca_cp_backoff(mca, "pump not ready");
		return;
	}

	mca->cp_mv = 0;
	mca->cp_request_ma = 0;
	mca->cp_polls = 0;
	mca->qc_align_mv = 0;
	mca->qc_stalls = 0;
	mca->cp_low = 0;
	mca->cp_ibus_hot = mca->cp_ibus_ma;
	mca->cp_ibat_trip = 0;
	mca->cp_stage = 0;
	mca->cp_target_ma = 0;
	mca->cp_recharge_cv = 0;
	mca->cp_parallel_done = false;
	mca->cp = MCA_CP_OPENING;
	/* Confirm the new FV before asking the source to open this session. */
	ret = xiaomi_mca_full_charge_poll(mca);
	if (ret) {
		xiaomi_mca_cp_stop(mca, "profile float voltage not confirmed");
		return;
	}
	dev_info(mca->dev, "direct-charge profile %s, source/cell/thermal limits retained\n",
		 xiaomi_mca_profile_names[mca->cp_profile]);

	if (xiaomi_mca_auth_pending(mca))
		mca->auth_tried = true;

	/* the buck charger carries little meanwhile */
	ret = xiaomi_mca_cp_set_buck(mca, MCA_CP_BUCK_MA, MCA_CP_BUCK_MA);
	ret = ret ?: xiaomi_mca_cp_request(mca, start_mv);
	if (ret) {
		xiaomi_mca_cp_stop(mca, "start refused");
		return;
	}

	dev_info(mca->dev, "direct charging: %u mV %u mA (APDO to %u mV %u mA), battery %d mV %d degC, bus target %u mA\n",
		 mca->cp_mv, mca->cp_ma, max_mv, ma, vbat, tbat, mca->cp_ibus_ma);
}

/* Piano cur_max_threshold, ibus_threshold and pmih_fcc_value. */
static u32 xiaomi_mca_parallel_fcc(int ibat_max, int ibus, u32 current_fcc)
{
	static const struct { int ibat, ibus, fcc; } rows[] = {
		{ 10000, 5200, 4000 }, { 8000, 4200, 3000 }, { 5000, 2700, 2000 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(rows); i++) {
		int hyst = current_fcc >= rows[i].fcc ? MCA_CP_PAR_HYS_MA : 0;

		if (ibat_max > rows[i].ibat && ibus > rows[i].ibus - hyst)
			return rows[i].fcc;
	}
	return 1000;
}

static u32 xiaomi_mca_die_limit(struct xiaomi_mca *mca, int tdie)
{
	if (tdie >= MCA_CP_TDIE_HOT)
		return max_t(u32, mca->cp_ibus_hot - MCA_CP_TDIE_STEP_MA,
					 min_t(u32, MCA_CP_TDIE_MIN_MA, mca->cp_ibus_ma));
	if (tdie < MCA_CP_TDIE_HOT - MCA_CP_TDIE_HYS)
		return min(mca->cp_ibus_hot + MCA_CP_TDIE_STEP_MA / 4,
				       mca->cp_ibus_ma);
	return mca->cp_ibus_hot;
}

static int xiaomi_mca_cp_band(u32 type)
{
	/* One QC3 class B pulse is ten PPS voltage steps. */
	return type == MCA_TYPE_HVDCP_3_B ? 10 * MCA_CP_REG_STEP_MA : MCA_CP_BAND_MA;
}

static bool xiaomi_mca_cp_bus_trip(struct xiaomi_mca *mca, int ibus)
{
	int margin = max(xiaomi_mca_cp_band(mca->cp_type), MCA_CP_TRIP_IBUS_MA);

	if (ibus >= MCA_CP_BUSOCP_MAX_MA) {
		mca->cp_fault = true;
		return true;
	}
	return ibus > (int)mca->cp_ibus_ma + margin;
}

/* A 2:1 stage reflects an adapter increment into at most half a step
 * at the cell (ignoring cable loss). Keep the stock 9 mV CV band.
 * Apply this after the current regulator, including its opening ramp.
 */
static int xiaomi_mca_cp_cv_request(u32 type, int previous, int requested, int vbat, int cv)
{
	int step = type == MCA_TYPE_HVDCP_3_B ? 200 : MCA_CP_REG_STEP_MV;
	int headroom = cv - vbat;

	if (headroom <= 3) /* stock fv_hys_delta_mv */
		return min(requested, previous - clamp(roundup(2 * (3 - headroom), step),
						       step, MCA_CP_REG_MAX_STEP_MV));
	if (requested > previous && 2 * (headroom - 9) < max(step, requested - previous))
		return previous;
	return requested;
}

/* One step of direct charging, every MCA_CP_POLL_MS */
static void xiaomi_mca_cp_step(struct xiaomi_mca *mca)
{
	int vbat, ibat, tcool, tbat, tpack, vbus_uv, ibus_ua, tdie, on, ibus, over;
	int ibat_max, target, ibus_all, buck_ma, budget, vmin, imax;
	int band = xiaomi_mca_cp_band(mca->cp_type);
	u32 mv = mca->cp_mv, cv;
	int previous;
	bool taper, ocp;
	u32 fcc, icl;

	if (xiaomi_mca_gauges(mca, &vbat, &ibat, &tcool, &tbat, &vmin, &imax, &ocp) ||
	    xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_ONLINE, &on)) {
		xiaomi_mca_cp_stop(mca, "read failed");
		return;
	}
	if (xiaomi_mca_read(mca, MCA_PROP_PACK_TBAT, &tpack)) {
		xiaomi_mca_cp_stop(mca, "pack temperature unavailable");
		return;
	}

	xiaomi_mca_select_profile(mca, min(tcool, tpack), max(tbat, tpack));
	/* Never continue a high-current APDO after authentication is revoked. */
	if (mca->cp_ma > MCA_CP_STD_PPS_MAX_MA) {
		u32 verified;

		if (xiaomi_mca_read(mca, MCA_PROP_PD_VERIFIED, &verified)) {
			xiaomi_mca_cp_stop(mca, "authentication read failed");
			return;
		}
		if (!mipps_auth || !xiaomi_mca_authenticated(mca)) {
			mca->auth_ok = false;
			xiaomi_mca_cp_stop(mca, "high current authorization inactive");
			return;
		}
		if (verified != 1) {
			mca->auth_lost = verified == 0;
			mca->auth_ok = false;
			xiaomi_mca_cp_stop(mca, mca->auth_lost ? "adapter authentication lost" :
							     "invalid authentication status");
			return;
		}
	}
	if (imax >= MCA_CP_CELL_OCP_MA) {
		mca->cp_fault = ocp;
		xiaomi_mca_cp_stop(mca, "cell current too high");
		return;
	}
	cv = xiaomi_mca_cp_vbat_max(mca);
	if (mca->ffc_done) {
		mca->cp_recharge_cv = MCA_TERM_FFC_MV;
		xiaomi_mca_cp_stop(mca, "taper complete");
		return;
	}
	taper = vbat >= cv - 9;	/* pps_taper_vol_hys */
	/* The pump CV setpoint is below the cell table's terminal voltage.
	 * Regulate through that gap, while retaining a hard cell ceiling.
	 */
	if (vmin < MCA_CP_VBAT_MIN_MV ||
	    vbat >= (!full_charge ? MCA_TERM_DEFAULT_MV : mca->ffc_active ? 4550 : mca->term_hot ? MCA_TERM_HOT_MV :
		     mca->cold_fv ? MCA_TERM_COLD_MV : MCA_TERM_FULL_MV)) {
		xiaomi_mca_cp_stop(mca, "cell voltage outside envelope");
		return;
	}
	if (taper && (mca->cp != MCA_CP_ON || ibat <= MCA_CP_TAPER_FCC_MA)) {
		mca->cp_recharge_cv = cv;
		xiaomi_mca_cp_stop(mca, "taper complete");
		return;
	}
	if (mca->cell_zone < MCA_CP_TBAT_MIN ||
	    max(tbat, tpack) >= MCA_CP_TBAT_EXIT) {
		xiaomi_mca_cp_stop(mca, "battery temperature");
		return;
	}
	if (xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS)) {
		/* Retry authentication only after the stage is confirmed off. */
		xiaomi_mca_cp_stop(mca, "adapter authentication");
		return;
	}

	ibat_max = xiaomi_mca_cp_ibat_limit(mca, vbat);
	/* a lowered limit is regulated down to before it trips */
	if (ibat_max >= (int)mca->cp_ibat_trip) {
		mca->cp_ibat_trip = ibat_max;
		mca->cp_trip_at = jiffies;
	} else if (time_after(jiffies, mca->cp_trip_at +
				       msecs_to_jiffies(MCA_CP_TRIP_GRACE_MS))) {
		mca->cp_ibat_trip = ibat_max;
	}

	if (xiaomi_mca_is_qc_pump(mca->cp_type) && !on && mca->cp == MCA_CP_OPENING) {
		int step = mca->cp_type == MCA_TYPE_HVDCP_3P5 ? 20 : 200;
		int want = 2 * vbat + MCA_CP_DELTA_MV;
		int bus = mca->vbus_uv / 1000;

		/* Align the bus with the present cells before closing the switch. */
		if (abs(bus - want) >= step) {
			if (abs(bus - mca->qc_align_mv) >= step) {
				mca->qc_align_mv = bus;
				mca->qc_stalls = 0;
			}
			if (++mca->qc_stalls > MCA_CP_OPEN_TRIES ||
			    ++mca->cp_polls > (MCA_HV_MV - 2 * MCA_CP_VBAT_MIN_MV) / step +
					      MCA_CP_OPEN_TRIES ||
			    xiaomi_mca_qc_adjust(mca, want))
				xiaomi_mca_cp_stop(mca, "QC failed to reach opening voltage");
			return;
		}
		if (mca->qc_align_mv) {
			mca->qc_align_mv = 0;
			mca->cp_polls = 0;
		}
	}
	if (mca->cp == MCA_CP_OPENING &&
	    ++mca->cp_polls > MCA_CP_OPEN_TRIES) {
		xiaomi_mca_cp_stop(mca, "no current");
		return;
	}
	if (!on && mca->cp == MCA_CP_ON) {
		/* A watchdog timeout is recoverable. Leave hardware protection
		 * enabled on every new arm, including after a thermal trip.
		 */
		xiaomi_mca_cp_stop(mca, "stage turned itself off");
		return;
	}
	/* enables it while opening (then steps up next poll), else a kick */
	if (xiaomi_mca_cp_set_online(mca, true)) {
		xiaomi_mca_cp_stop(mca, "stage refused");
		return;
	}
	if (!on)
		return;

	if (xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbus_uv) ||
	    xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_CURRENT_NOW, &ibus_ua) ||
	    xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_TEMP, &tdie)) {
		xiaomi_mca_cp_stop(mca, "stage read failed");
		return;
	}
	ibus = ibus_ua / 1000;
	buck_ma = mca->ibus_ua / 1000;
	mca->cp_ibus_last = max(ibus + buck_ma, 0);
	if (2 * vbat > mca->cp_max_mv) {
		xiaomi_mca_cp_stop(mca, "source voltage range exhausted");
		return;
	}
	if (vbus_uv / 1000 >= MCA_CP_PPS_MAX_MV ||
	    vbus_uv / 1000 > 2 * vbat + MCA_CP_VBUS_MAX_DELTA_MV) {
		/* A relative control excursion is recoverable. The independent
		 * forward BUSOVP programmed in SC8541 is 11 V.
		 */
		mca->cp_fault = vbus_uv / 1000 >= MCA_CP_PPS_MAX_MV;
		xiaomi_mca_cp_stop(mca, "bus voltage too high");
		return;
	}
	if (tdie >= MCA_CP_TDIE_MAX) {
		mca->cp_die_hot = true;
		xiaomi_mca_cp_stop(mca, "stage temperature");
		return;
	}
	if (xiaomi_mca_cp_bus_trip(mca, ibus)) {
		xiaomi_mca_cp_stop(mca, "bus current too high");
		return;
	}
	if (ibat > MCA_CP_IBAT_MAX_MA || ibat > (int)mca->cp_ibat_trip + MCA_CP_TRIP_IBAT_MA) {
		/* div_max_curr is a regulation limit, not a pack OCP flag. */
		xiaomi_mca_cp_stop(mca, "battery current too high");
		return;
	}

	if (!taper && tdie < MCA_CP_TDIE_HOT)
		xiaomi_mca_cp_recover_current(mca);
	/* the stage warms about 1 degC/s at 4 A: back off before it trips */
	mca->cp_ibus_hot = xiaomi_mca_die_limit(mca, tdie);
	target = mca->cp_ibus_hot;

	if (xiaomi_mca_is_qc_pump(mca->cp_type))
		mv = mca->vbus_uv / 1000;
	previous = mv;
	if (mca->cp == MCA_CP_OPENING) {
		if (ibus < MCA_CP_OPEN_IBUS_MA) {
			if (mca->cp_type != MCA_TYPE_HVDCP_3_B ||
			    (ibus + band <= target && ibat + 2 * band <= ibat_max))
				mv += MCA_CP_STEP_MV;
		} else {
			dev_info(mca->dev, "direct charging at %u mV: bus %d mV %d mA, battery %d mV %d mA\n",
				 mca->cp_mv, vbus_uv / 1000, ibus,
				 vbat, ibat);
			mca->cp = MCA_CP_ON;
			mca->cp_running_at = jiffies;
			mca->cp_polls = 0;
		}
	} else {
		if (time_after_eq(jiffies, mca->cp_running_at + 60 * HZ))
			mca->cp_retry_ms = 0;
		if (ibus < MCA_CP_LOW_IBUS_MA) {
			if (++mca->cp_low >= MCA_CP_LOW_POLLS) {
				xiaomi_mca_cp_stop(mca, "bus current too low");
				return;
			}
		} else {
			mca->cp_low = 0;
		}

		/* only from a source that feeds both at that bus current */
		ibus_all = ibus + buck_ma;
		fcc = xiaomi_mca_parallel_fcc(ibat_max, ibus_all, mca->fcc_set_ma);
		icl = fcc / 2 + (fcc > 1000 ? 500 : 0);
		/* The stock vbat_threshold ends parallel boosting near CV. */
		if (vbat >= 4480 || taper)
			mca->cp_parallel_done = true;
		if (mca->cp_parallel_done)
			fcc = icl = MCA_CP_BUCK_MA;
		budget = (int)xiaomi_mca_cp_src_ma(mca) - 200;
		/* While raising a vote, allow for the load step before its ADC
		 * sample is available. Steady regulation uses measured draw.
		 */
		if (fcc > mca->fcc_set_ma)
			target = min(target, budget - (int)icl);
		if (fcc != mca->fcc_set_ma &&
		    (fcc < mca->fcc_set_ma || ibus <= target)) {
			if (xiaomi_mca_cp_set_buck(mca, icl, fcc)) {
				xiaomi_mca_cp_stop(mca, "buck charger refused");
				return;
			}
		}
		target = min(target, budget - buck_ma);
		/*
		 * The stage gives the battery about twice its bus current,
		 * next to the buck charger's share.  The gauges average over
		 * 10-15 s, so the battery limit is held through the stage's
		 * own reading and theirs only trims it slowly, once they
		 * have caught up with a lowered limit.
		 */
		target = min(target, (ibat_max - (int)mca->fcc_set_ma) / 2);

		over = ibus - target;
		if (over > 0)
			mv -= clamp(over / MCA_CP_REG_STEP_MA * MCA_CP_REG_STEP_MV,
				    MCA_CP_REG_STEP_MV, MCA_CP_REG_MAX_STEP_MV);
		else if (ibat > ibat_max && mca->cp_ibat_trip <= ibat_max)
			mv -= MCA_CP_REG_STEP_MV;
		else if (ibus + band <= target && ibat + 2 * band <= ibat_max)
			mv += MCA_CP_REG_STEP_MV;
		/* the gauges catch up with a lowered limit only from here */
		if (over > MCA_CP_BAND_MA)
			mca->cp_trip_at = jiffies;

		if (++mca->cp_polls % MCA_CP_LOG_POLLS == 0)
			dev_info(mca->dev, "direct charging at %u mV: bus %d mV %d/%d mA, battery %d mV %d/%d mA %d degC, stage %d.%d degC\n",
				 mca->cp_mv, vbus_uv / 1000, ibus, target,
				 vbat, ibat, ibat_max, max(tbat, tpack),
				 tdie / 10, tdie % 10);
	}

	mv = clamp_t(u32, mv, 2 * vbat, 2 * vbat + MCA_CP_REQ_MAX_DELTA_MV);
	mv = xiaomi_mca_cp_cv_request(mca->cp_type, previous, mv, vbat, cv);
	if (xiaomi_mca_cp_request(mca, mv)) {
		xiaomi_mca_cp_learn_drop(mca);
		xiaomi_mca_cp_stop(mca, "request refused");
	}
}

static int xiaomi_mca_read_usb(struct xiaomi_mca *mca, enum xiaomi_mca_usb_index index,
			       u32 *value)
{
	struct xiaomi_mca_cached *cached = &mca->cache[index];
	int ret;

	guard(mutex)(&mca->lock);

	if (!cached->valid || time_after_eq(jiffies, cached->expires)) {
		cached->valid = false;
		if (index == MCA_USB_REAL_TYPE)
			ret = xiaomi_mca_read_type(mca, &cached->value);
		else
			ret = xiaomi_mca_read(mca, xiaomi_mca_usb_property[index], &cached->value);
		if (ret)
			return ret;

		cached->expires = jiffies + MCA_CACHE_TIME;
		cached->valid = true;
	}

	*value = cached->value;

	return 0;
}

static int xiaomi_mca_usb_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(psy);
	enum xiaomi_mca_usb_index index;
	u32 value;
	int ret;

	if (psp == POWER_SUPPLY_PROP_AUTHENTIC) {
		guard(mutex)(&mca->lock);
		val->intval = mca->online && xiaomi_mca_authenticated(mca);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		index = MCA_USB_ONLINE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		index = MCA_USB_VOLTAGE_NOW;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		index = MCA_USB_CURRENT_NOW;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		index = MCA_USB_INPUT_CURRENT_LIMIT;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		index = MCA_USB_REAL_TYPE;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		index = MCA_USB_CHARGE_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	ret = xiaomi_mca_read_usb(mca, index, &value);
	if (ret)
		return ret;

	val->intval = xiaomi_mca_usb_propval(index, value);

	return 0;
}

static int xiaomi_mca_usb_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(psy);
	struct xiaomi_mca_state source = {};
	const char *name;
	u32 property, ma;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		property = MCA_PROP_INPUT_CURRENT_LIMIT;
		name = "input current limit";
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		property = MCA_PROP_CHARGE_CURRENT;
		name = "charge current";
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&mca->lock);

	/* The limits depend on the source attached now. */
	ret = xiaomi_mca_read(mca, MCA_PROP_USB_ONLINE, &source.online);
	if (!ret)
		ret = xiaomi_mca_read_type(mca, &source.real_type);
	if (ret)
		return ret;

	if (!source.online)
		return -ENODEV;

	ret = xiaomi_mca_current_ma(psp, source.real_type,
				    mca->hv == MCA_HV_ON, val->intval, &ma);
	if (ret)
		return ret;

	/* Restored on a change of source even if this write fails half way. */
	if (property == MCA_PROP_INPUT_CURRENT_LIMIT) {
		mca->icl_written = true;
		mca->icl_source = source;
	}

	ret = xiaomi_mca_write(mca, property, ma);
	if (ret) {
		dev_warn(mca->dev, "failed to set the %s to %u mA: %d\n", name, ma, ret);
		return ret;
	}

	if (property == MCA_PROP_CHARGE_CURRENT)
		mca->fcc_set_ma = ma;
	else
		mca->icl_set_ma = ma;

	dev_info(mca->dev, "%s set to %u mA (USB real type %u)\n", name, ma,
		 source.real_type);

	return 0;
}

static int xiaomi_mca_usb_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
	       psp == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT;
}

static const enum power_supply_property xiaomi_mca_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_AUTHENTIC,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static const struct power_supply_desc xiaomi_mca_usb_psy_desc = {
	.name = "xiaomi-mca-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = xiaomi_mca_usb_props,
	.num_properties = ARRAY_SIZE(xiaomi_mca_usb_props),
	.get_property = xiaomi_mca_usb_get_property,
	.set_property = xiaomi_mca_usb_set_property,
	.property_is_writeable = xiaomi_mca_usb_property_is_writeable,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_C) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD) |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
};

/* Optional provider: a missing or unbound SC8541 does not fail MCA probe. */
static int xiaomi_mca_otg_cp(struct xiaomi_mca *mca, enum sc8541_otg_action action)
{
	struct power_supply *psy;
	int ret;

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
	KUNIT_STATIC_STUB_REDIRECT(xiaomi_mca_otg_cp, mca, action);
#endif
	psy = power_supply_get_by_reference(dev_fwnode(mca->dev), "xiaomi,charge-pump");
	if (IS_ERR_OR_NULL(psy))
		return -ENODEV;
	ret = sc8541_otg_configure(psy, action);
	power_supply_put(psy);
	return ret;
}

/* No independent timer: only an explicit consumer or an ADSP epoch applies it. */
static void xiaomi_mca_pen_delay(unsigned int ms)
{
#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
	KUNIT_STATIC_STUB_REDIRECT(xiaomi_mca_pen_delay, ms);
#endif
	msleep(ms);
}

static int xiaomi_mca_pen_off(struct xiaomi_mca *mca)
{
	int ret;

	lockdep_assert_held(&mca->lock);
	if (!mca->pen_dirty)
		return 0;
	/* Even a failed voltage write must not prevent an attempt to stop. */
	xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_VOLTAGE, BIT(16) | 5200);
	xiaomi_mca_pen_delay(10);
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_ENABLE, 0x20100);
	if (ret)
		return ret;
	mca->pen_on = false;
	ret = xiaomi_mca_otg_cp(mca, SC8541_PEN_RESTORE);
	if (!ret)
		mca->pen_dirty = false;
	return ret;
}

static int xiaomi_mca_pen_on(struct xiaomi_mca *mca)
{
	u32 generation, vbus, mv;
	int ret;

	lockdep_assert_held(&mca->lock);
	if (mca->sleeping || mca->otg_active || mca->connector.tripped)
		return -EBUSY;
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (!mca->service_up)
			return -ENOTCONN;
		generation = mca->service_generation;
	}
	mca->pen_attempted = true;
	mca->pen_attempt_generation = generation;
	/* Do not disconnect a wired charger for an unverified source switch. */
	ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
	if (ret || vbus >= MCA_VBUS_ONLINE_UV)
		return ret ?: -EBUSY;
	if (mca->pen_on && mca->pen_generation == generation)
		return 0;
	ret = xiaomi_mca_otg_cp(mca, SC8541_PEN_ISOLATE);
	if (ret) {
		xiaomi_mca_otg_cp(mca, SC8541_PEN_RESTORE);
		return ret;
	}
	mca->pen_dirty = true;
	xiaomi_mca_pen_delay(100);
	ret = xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_VOLTAGE, BIT(16) | 5000);
	ret = ret ?: xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_ENABLE, 0x20101);
	for (mv = 5500; !ret && mv <= 6500; mv += 500) {
		xiaomi_mca_pen_delay(20);
		ret = xiaomi_mca_write_u32(mca, MCA_PROP_PEN_BOOST_VOLTAGE, BIT(16) | mv);
	}
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (!mca->service_up || generation != mca->service_generation)
			ret = -ECONNRESET;
	}
	if (ret)
		goto fail;
	mca->pen_on = true;
	mca->pen_generation = generation;
	return 0;
fail:
	xiaomi_mca_pen_off(mca);
	return ret;
}

static void xiaomi_mca_pen_supply_failed(struct xiaomi_mca *mca)
{
	if (mca->pen_requested && mca->pen_rdev) {
		mca->pen_requested = false;
		regulator_notifier_call_chain(mca->pen_rdev, REGULATOR_EVENT_FAIL, NULL);
	}
}

static int xiaomi_mca_pen_enable(struct regulator_dev *rdev)
{
	struct xiaomi_mca *mca = rdev_get_drvdata(rdev);
	int ret;

	guard(mutex)(&mca->lock);
	ret = xiaomi_mca_pen_on(mca);
	if (!ret)
		mca->pen_requested = true;
	return ret;
}

static int xiaomi_mca_pen_disable(struct regulator_dev *rdev)
{
	struct xiaomi_mca *mca = rdev_get_drvdata(rdev);

	guard(mutex)(&mca->lock);
	mca->pen_requested = false;
	return xiaomi_mca_pen_off(mca);
}

static int xiaomi_mca_pen_is_enabled(struct regulator_dev *rdev)
{
	struct xiaomi_mca *mca = rdev_get_drvdata(rdev);

	guard(mutex)(&mca->lock);
	return mca->pen_on && mca->pen_generation == READ_ONCE(mca->service_generation);
}

static const struct regulator_ops xiaomi_mca_pen_ops = {
	.enable = xiaomi_mca_pen_enable,
	.disable = xiaomi_mca_pen_disable,
	.is_enabled = xiaomi_mca_pen_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
};

static const struct regulator_desc xiaomi_mca_pen_desc = {
	.name = "piano-pen-boost",
	.of_match = "pen-boost",
	.regulators_node = "regulators",
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.ops = &xiaomi_mca_pen_ops,
	.n_voltages = 1,
	.min_uV = 6500000,
	.fixed_uV = 6500000,
};

static int xiaomi_mca_register_pen(struct xiaomi_mca *mca)
{
	struct regulator_config cfg = { .dev = mca->dev, .driver_data = mca };
	struct device_node *regulators, *pen;
	struct regulator_dev *rdev;

	regulators = of_get_child_by_name(mca->dev->of_node, "regulators");
	pen = of_get_child_by_name(regulators, "pen-boost");
	of_node_put(regulators);
	if (!pen)
		return 0;
	of_node_put(pen);
	rdev = devm_regulator_register(mca->dev, &xiaomi_mca_pen_desc, &cfg);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);
	mca->pen_rdev = rdev;
	return 0;
}

static void xiaomi_mca_cp_missing(struct xiaomi_mca *mca, int ret)
{
	if ((ret == -ENODEV || ret == -EOPNOTSUPP) && !mca->cp_absent_warned) {
		dev_warn(mca->dev, "OTG CP provider unavailable: %d\n", ret);
		mca->cp_absent_warned = true;
	}
}

/* Stock qcom_subpmic_sync_cfg_work: CP_STATE, OTG_CFG, then CID_CFG. */
static void xiaomi_mca_sync_config(struct xiaomi_mca *mca)
{
	const __le32 cid = cpu_to_le32(MCA_PIANO_CID_CFG);
	u32 generation;
	bool up;
	u8 present;
	int ret;

	if (!mca->piano_otg)
		return;
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		generation = mca->service_generation;
		up = mca->service_up;
	}
	if (!up || (mca->config_attempted && mca->config_generation == generation &&
		    !mca->config_result))
		return;
	/* As in stock/SunUEFI, a failed configuration is retried by the poll. */
	mca->config_attempted = true;
	mca->config_generation = generation;
	ret = xiaomi_mca_otg_cp(mca, SC8541_OTG_PRESENT);
	present = !ret;
	xiaomi_mca_cp_missing(mca, ret);
	ret = xiaomi_mca_write_blob(mca, MCA_PROP_CP_STATE, &present, sizeof(present));
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (mca->service_generation != generation || !mca->service_up)
			ret = -ESTALE;
	}
	if (!ret)
		ret = xiaomi_mca_write_blob(mca, MCA_PROP_OTG_CFG,
				xiaomi_mca_piano_otg_cfg, sizeof(xiaomi_mca_piano_otg_cfg));
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (mca->service_generation != generation || !mca->service_up)
			ret = -ESTALE;
	}
	if (!ret)
		ret = xiaomi_mca_write_blob(mca, MCA_PROP_CID_CFG, &cid, sizeof(cid));
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		if (mca->service_generation != generation || !mca->service_up)
			ret = -ESTALE;
	}
	mca->config_result = ret;
	if (ret)
		dev_warn_ratelimited(mca->dev, "OTG configuration sync failed: %d\n", ret);
	else
		dev_info(mca->dev, "OTG config generation %u CP present %u CID %#x result 0\n",
			 generation, present, MCA_PIANO_CID_CFG);
}

/* Process in the existing poll, preserving notification order. No retries. */
static void xiaomi_mca_process_notifications(struct xiaomi_mca *mca)
{
	struct xiaomi_mca_notify_record record;
	u32 generation;
	u8 plugin;
	int ret, budget = MCA_NOTIFY_HISTORY;

	while (budget--) {
		scoped_guard(spinlock_irqsave, &mca->rx_lock) {
			if (!mca->notify_pending)
				return;
			record = mca->notify[(mca->notify_head + MCA_NOTIFY_HISTORY -
					      mca->notify_pending) % MCA_NOTIFY_HISTORY];
			mca->notify_pending--;
			generation = mca->service_generation;
		}
		dev_info(mca->dev, "MCA notification %u data %#x generation %u%s\n",
			 record.id, record.data[0], record.generation,
			 record.generation == generation ? "" : " stale");
		if (!mca->piano_otg || record.generation != generation)
			continue;
		switch (record.id) {
		case MCA_NOTIFY_OTG_CP_CONFIG:
			if (record.data[0] && mca->connector.tripped)
				continue;
			/* Prevent the forward policy from changing a reverse path. */
			mca->otg_active = true;
			ret = xiaomi_mca_otg_cp(mca, record.data[0] ?
					SC8541_OTG_REVERSE_ON : SC8541_OTG_REVERSE_OFF);
			if (!ret && !record.data[0])
				mca->otg_active = false;
			break;
		case MCA_NOTIFY_BOOST:
			/*
			 * boost_src=0 bypasses proxy's ovpgate_with_check.
			 * Ordinary OTG only clears manual reverse on detach
			 * (strategy_buckchg_cp_revert_handler, 0x5794..0x57c8).
			 */
			ret = xiaomi_mca_read_blob(mca, MCA_PROP_OTG_PLUGIN,
						  &plugin, sizeof(plugin));
			if (!ret && !plugin)
				ret = xiaomi_mca_otg_cp(mca, SC8541_OTG_DETACH);
			break;
		default:
			continue;
		}
		xiaomi_mca_cp_missing(mca, ret);
		dev_info(mca->dev, "OTG notification %u CP result %d\n", record.id, ret);
	}
}

static bool xiaomi_mca_usb_notification(const void *data, size_t len, void *priv)
{
	const struct xiaomi_mca_notification *msg = data;

	return len == sizeof(*msg) && le32_to_cpu(msg->id) == MCA_NOTIFY_TYPE_CHANGE;
}

static void xiaomi_mca_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct xiaomi_mca *mca = priv;
	u32 value = 0;
	int ret;

	guard(spinlock_irqsave)(&mca->rx_lock);

	if (len < sizeof(*hdr)) {
		mca->stats.malformed++;
		return;
	}

	if (le32_to_cpu(hdr->type) == PMIC_GLINK_NOTIFY) {
		const struct xiaomi_mca_notification *msg = data;
		struct xiaomi_mca_notify_record *record;
		int i;

		mca->stats.notifications++;
		if (len != sizeof(*msg) ||
		    le32_to_cpu(hdr->owner) != PMIC_GLINK_OWNER_BATTMGR) {
			mca->stats.malformed++;
			return;
		}
		record = &mca->notify[mca->notify_head];
		record->time_ns = ktime_get_boottime_ns();
		record->generation = mca->service_generation;
		record->id = le32_to_cpu(msg->id);
		for (i = 0; i < ARRAY_SIZE(record->data); i++)
			record->data[i] = get_unaligned_le32(msg->data + i * sizeof(u32));
		mca->notify_head = (mca->notify_head + 1) % MCA_NOTIFY_HISTORY;
		mca->notify_count = min_t(u32, mca->notify_count + 1, MCA_NOTIFY_HISTORY);
		if (mca->notify_pending == MCA_NOTIFY_HISTORY)
			mca->notify_dropped++;
		else
			mca->notify_pending++;
		if (record->id == MCA_NOTIFY_TYPE_CHANGE || record->id == MCA_NOTIFY_TYPEC_CHANGE ||
		    record->id == MCA_NOTIFY_OTG_CP_CONFIG || record->id == MCA_NOTIFY_BOOST) {
			mca->input_generation++;
			mca->input_pending = mca->service_up;
			if (mca->piano_otg && record->id == MCA_NOTIFY_OTG_CP_CONFIG)
				mca->otg_requested = !!record->data[0];
			if (mca->poll_ready)
				mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
		}
		return;
	}

	if (!mca->pending) {
		mca->stats.unmatched++;
		return;
	}

	ret = xiaomi_mca_parse_reply(data, len, mca->pending_opcode, mca->pending_property,
				     mca->pending_seq, &value);
	if (ret == -ENOMSG) {
		mca->stats.unmatched++;
		return;
	}

	mca->stats.replies++;
	if (ret == -EIO)
		mca->stats.errors++;
	else if (ret)
		mca->stats.malformed++;

	mca->reply_len = min(len, sizeof(mca->reply));
	memcpy(mca->reply, data, mca->reply_len);
	mca->retcode = len == sizeof(struct xiaomi_mca_reply) ?
		le32_to_cpu(((const struct xiaomi_mca_reply *)data)->retcode) : 0;
	mca->result = ret;
	mca->value = value;
	mca->answered = true;
	mca->pending = false;
	complete(&mca->ack);
}

static void xiaomi_mca_pdr_notify(void *priv, int state)
{
	struct xiaomi_mca *mca = priv;

	guard(spinlock_irqsave)(&mca->rx_lock);
	if (mca->service_up != (state == SERVREG_SERVICE_STATE_UP)) {
		mca->service_generation++;
		mca->input_generation++;
		mca->input_pending = state == SERVREG_SERVICE_STATE_UP;
	}
	mca->service_up = state == SERVREG_SERVICE_STATE_UP;
	if (!mca->service_up) {
		mca->otg_requested = false;
		if (xiaomi_mca_charge_wake_locked(mca, false) && mca->poll_ready)
			power_supply_changed(mca->psy);
	}
	if (mca->poll_ready && mca->service_up)
		mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
}

static void xiaomi_mca_policy_poll(struct xiaomi_mca *mca, u32 *sampled_type)
{
	bool online;
	u32 real_type = MCA_TYPE_UNKNOWN, vbus;
	u32 attachment = READ_ONCE(mca->attach_generation);

	/* Do not change source voltage or restart a stage whose stop failed. */
	if (mca->cp == MCA_CP_STOPPING) {
		xiaomi_mca_cp_stop(mca, "pending stop");
		return;
	}

	if (attachment != mca->policy_attach_generation) {
		if (xiaomi_mca_cp_stop(mca, "physical disconnect"))
			return;
		mca->ffc_active = false;
		mca->auth_ok = false;
		mca->mipps_attempted = false;
		mca->cp = MCA_CP_OFF;
		mca->hv = MCA_HV_OFF;
		mca->cp_ibus_cap = 0;
		mca->aicl_cap_ma = 0;
		mca->aicl_probe_floor_ma = 0;
		mca->unknown_polls = 0;
		mca->online = false;
		mca->auth_lost = false;
		mca->cp_fault = false;
		mca->cp_exits = 0;
		mca->cp_last_exit = NULL;
		mca->auth_retry_ms = 0;
		mca->hv_last_exit = NULL;
		mca->cp_recharge_cv = 0;
		mca->hv_retry = jiffies;
		mca->zone_valid = false;
		mca->policy_attach_generation = attachment;
	}
	if (xiaomi_mca_clear_auth(mca))
		return;

	if (xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus)) {
		/* Missing telemetry is neither a detach nor a source overload. */
		mca->unknown_polls = 0;
		xiaomi_mca_cp_stop(mca, "VBUS read failed");
		return;
	}
	mca->vbus_uv = vbus;
	online = mca->vbus_uv >= MCA_VBUS_ONLINE_UV;
	if (!online)
		xiaomi_mca_record_collapse(mca);

	/* The rest only means something with a source attached */
	if (online) {
		mca->ibus_ua = xiaomi_mca_read_u32(mca, MCA_PROP_USB_CURRENT_NOW);
		mca->icl_ma = xiaomi_mca_read_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT);
		if (xiaomi_mca_read_type(mca, &real_type)) {
			*sampled_type = MCA_TYPE_UNKNOWN;
			mca->unknown_polls = 0;
			xiaomi_mca_cp_stop(mca, "charger type read failed");
			return;
		}
		*sampled_type = real_type;
		if (mca->ibus_ua < 0 || mca->icl_ma == -1) {
			mca->unknown_polls = 0;
			xiaomi_mca_cp_stop(mca, "telemetry read failed");
			return;
		}
	} else {
		mca->ibus_ua = 0;
		mca->icl_ma = 0;
	}


	if (online && real_type == MCA_TYPE_UNKNOWN && vbus < MCA_HV_VBUS_5V_MAX_UV &&
	    xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE) == 1)
		mca->unknown_polls = min(mca->unknown_polls + 1, (unsigned int)MCA_UNKNOWN_POLLS);
	else
		mca->unknown_polls = 0;

	if (online && !mca->online && mca->cp_ibus_cap &&
	    time_after(jiffies, mca->cp_off_at +
				msecs_to_jiffies(MCA_CP_DROP_FORGET_MS)))
		mca->cp_ibus_cap = 0;
	if (!online && mca->online)
		mca->cp_off_at = jiffies;
	if (online && !mca->online) {
		/* away longer than a reset takes: another attach */
		if (time_after(jiffies, mca->cp_off_at +
				msecs_to_jiffies(MCA_CP_DROP_FORGET_MS))) {
			mca->ffc_active = false;
			mca->auth_ok = false;
			mca->mipps_attempted = false;
		}
		mca->cp_on_at = jiffies;
	}
	if (real_type != mca->real_type && (xiaomi_mca_limits(real_type) || real_type == MCA_TYPE_UNKNOWN)) {
		/* A QC identification may appear after DCP or disappear on a
		 * new attachment. Do not inherit the old protocol's 9 V state. */
		if (xiaomi_mca_is_qc(real_type) || xiaomi_mca_is_qc(mca->real_type)) {
			mca->hv = MCA_HV_OFF;
			mca->hv_pps = false;
		}
		if (mca->icl_written && xiaomi_mca_restore_icl(mca))
			return;
		if (mca->fcc_written && xiaomi_mca_reset_fcc(mca))
			return;
	}
	mca->online = online;
	mca->real_type = real_type;

	if (!online) {
		/* Preserve an authentication reset only while its cable is connected. */
		if (power_supply_for_each_psy(mca, xiaomi_mca_ucsi_status) == MCA_UCSI_OFFLINE) {
			mca->aicl_cap_ma = 0;
			mca->aicl_probe_floor_ma = 0;
			mca->auth_lost = false;
			mca->cp_fault = false;
			mca->cp_recharge_cv = 0;
		}
		if (!xiaomi_mca_auth_pending(mca) ||
		    power_supply_for_each_psy(mca, xiaomi_mca_ucsi_status) < MCA_UCSI_CONNECTED) {
			mca->ffc_active = false;
			mca->auth_ok = false;
			mca->mipps_attempted = false;
		}
		xiaomi_mca_cp_learn_drop(mca);
		if (xiaomi_mca_cp_stop(mca, "detached"))
			return;
		mca->cp = MCA_CP_OFF;
		if (mca->icl_written)
			xiaomi_mca_reset_icl(mca);
		if (mca->fcc_written)
			xiaomi_mca_reset_fcc(mca);
		/* a hard reset in answer to the request looks like a detach */
		if (mca->hv == MCA_HV_REQUESTED)
			mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_RETRY_MS);
		mca->hv = MCA_HV_OFF;
	} else if ((!xiaomi_mca_limits(real_type) && real_type != MCA_TYPE_UNKNOWN) ||
		   (xiaomi_mca_cp_active(mca) &&
		    (xiaomi_mca_is_qc_pump(mca->cp_type) ? real_type != mca->cp_type :
							   !xiaomi_mca_is_pd(real_type)))) {
		if (!xiaomi_mca_cp_stop(mca, "unknown source") && mca->icl_written)
			xiaomi_mca_restore_icl(mca);
	} else if (real_type == MCA_TYPE_UNKNOWN) {
		if (charge_policy && mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV) {
			mca->hv = MCA_HV_OFF;
			xiaomi_mca_apply_policy(mca);
		} else if (mca->icl_written) {
			xiaomi_mca_restore_icl(mca);
		}
	} else if (charge_policy && mipps_auth && !mca->auth_lost &&
		   mca->mipps_attempted &&
		   !xiaomi_mca_authenticated(mca) && time_after_eq(jiffies, mca->auth_retry) &&
		   mca->cp == MCA_CP_OFF && mca->hv == MCA_HV_OFF &&
		   mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV && xiaomi_mca_is_pd(real_type)) {
		/* Re-enter from 5 V without dismantling a working fallback. */
		mca->mipps_attempted = false;
	} else if (charge_policy && xiaomi_mca_cp_active(mca)) {
		xiaomi_mca_cp_step(mca);
	} else if (charge_policy) {
		xiaomi_mca_mipps_authenticate(mca);
		if (mca->auth_cleanup || attachment != READ_ONCE(mca->attach_generation))
			return;
		xiaomi_mca_cp_try_start(mca);
		if (!xiaomi_mca_cp_active(mca))
			xiaomi_mca_apply_policy(mca);
	}
}

static bool xiaomi_mca_identity_changed(struct xiaomi_mca *mca)
{
	u32 svid;

	if (!mipps_auth || xiaomi_mca_read(mca, MCA_PROP_ADAPTER_SVID, &svid) ||
	    svid == mca->adapter_svid)
		return false;

	dev_info(mca->dev, "adapter SVID %#06x -> %#06x\n", mca->adapter_svid, svid);
	mca->adapter_svid = svid;
	return true;
}

/*
 * Report a change of the input state found by polling. The first state read
 * after probe, or after reads failed, is reported as a change too, so that
 * userspace sees the values that could not be read before.
 */
static void xiaomi_mca_poll(struct work_struct *work)
{
	struct xiaomi_mca *mca = container_of(to_delayed_work(work), struct xiaomi_mca,
					      poll_work);
	struct xiaomi_mca_state now = { .real_type = U32_MAX };
	struct xiaomi_mca_cells cells;
	u32 generation, input_generation;
	bool changed = false, policy_sample = false;
	int ret;

	scoped_guard(mutex, &mca->lock) {
		scoped_guard(spinlock_irqsave, &mca->rx_lock) {
			generation = mca->service_generation;
			input_generation = mca->input_generation;
		}
		if (xiaomi_mca_connector_poll(mca)) {
			xiaomi_mca_process_notifications(mca);
			goto out;
		}
		xiaomi_mca_observe_aicl(mca);
		xiaomi_mca_sync_config(mca);
		if (mca->policy_generation != generation) {
			if (xiaomi_mca_cp_stop(mca, "ADSP restart"))
				goto out;
			mca->cp = MCA_CP_OFF;
			mca->hv = MCA_HV_OFF;
			mca->icl_written = false;
			mca->icl_set_ma = 0;
			mca->fcc_set_ma = 0;
			mca->fcc_written = false;
			mca->ffc_active = false;
			mca->auth_ok = false;
			mca->mipps_attempted = false;
			mca->auth_cleanup = false;
			mca->online = false;
			mca->policy_generation = generation;
		}
		if (mca->pen_requested && (!mca->pen_attempted ||
		    mca->pen_attempt_generation != generation)) {
			int ret = xiaomi_mca_pen_on(mca);

			if (ret) {
				dev_warn(mca->dev, "pen supply restore failed: %d\n", ret);
				xiaomi_mca_pen_supply_failed(mca);
			}
		}
		/* A newly attached source has priority over optional pen charging. */
		if (mca->pen_dirty) {
			u32 vbus;

			if (xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus) ||
			    vbus >= MCA_VBUS_ONLINE_UV) {
				xiaomi_mca_pen_off(mca);
				xiaomi_mca_pen_supply_failed(mca);
			}
		}
		if (xiaomi_mca_clear_auth(mca))
			goto out;
		xiaomi_mca_process_notifications(mca);
		if (charge_policy || READ_ONCE(mca->charge_end) < 100 || mca->charge_disabled) {
			xiaomi_mca_sample_cells(mca, &cells);
			mca->poll_cells = &cells;
		}
		ret = xiaomi_mca_full_charge_poll(mca);
		if (ret) {
			dev_warn_ratelimited(mca->dev, "failed to set float voltage: %d\n", ret);
			if (!full_charge || !charge_policy)
				xiaomi_mca_cp_stop(mca, "float voltage not confirmed");
			else
				xiaomi_mca_charge_stop(mca);
			goto out;
		}
		ret = xiaomi_mca_charge_limit_poll(mca);
		if (ret) {
			dev_warn_ratelimited(mca->dev, "failed to apply battery limit: %d\n", ret);
			goto out;
		}
		if (mca->charge_blocked)
			goto out;
		if (charge_policy && !mca->otg_active && !mca->pen_dirty) {
			mca->policy_active = true;
			xiaomi_mca_policy_poll(mca, &now.real_type);
		} else if (!mca->otg_active && !mca->pen_dirty && mca->policy_active) {
			if (xiaomi_mca_cp_stop(mca, "policy disabled"))
				goto out;
			xiaomi_mca_restore_icl(mca);
			if (mca->fcc_written)
				xiaomi_mca_reset_fcc(mca);
			if (mca->voltage_written)
				xiaomi_mca_request_volt(mca, 5000);
			mca->policy_active = false;
			mca->cp = MCA_CP_OFF;
			mca->hv = MCA_HV_OFF;
		}
		policy_sample = true;
out:
		/* This is the existing USB change-reporting sample. A type read
		 * failure on an online input means unknown, including before APSD.
		 */
		now.online = xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE);
		ret = now.online == U32_MAX ? -EIO : 0;
		if (now.online == 1) {
			if (now.real_type == U32_MAX) {
				ret = xiaomi_mca_read_type(mca, &now.real_type);
				if (ret)
					now.real_type = MCA_TYPE_UNKNOWN;
			}
			now.chgr_status = xiaomi_mca_read_u32(mca, MCA_PROP_CHGR_STATUS);
			if (now.chgr_status == U32_MAX)
				ret = -EIO;
		} else {
			now.real_type = MCA_TYPE_UNKNOWN;
		}
		if (ret) {
			mca->last_valid = false;
		} else {
			if (policy_sample && !now.online)
				xiaomi_mca_record_collapse(mca);
			changed = !mca->last_valid || memcmp(&now, &mca->last, sizeof(now));
			mca->last = now;
			mca->last_valid = true;

			if (policy_sample && (now.online != mca->icl_source.online ||
					      now.real_type != mca->icl_source.real_type))
				xiaomi_mca_restore_icl(mca);
		}
		changed |= xiaomi_mca_apply_wired_sample(mca, input_generation, now.online, now.real_type);
		mca->poll_cells = NULL;
		/* Identity discovery must also reach userspace while charging is held. */
		changed |= xiaomi_mca_identity_changed(mca);
		/* The uevent must not report what was cached before the change. */
		if (changed)
			xiaomi_mca_invalidate(mca);
	}

	if (changed)
		power_supply_changed(mca->psy);

	schedule_delayed_work(&mca->poll_work,
		xiaomi_mca_cp_active(mca) ? msecs_to_jiffies(MCA_CP_POLL_MS) :
		(mca->connector.cutoff ? HZ : MCA_POLL_INTERVAL));
}

/* The ADSP's own name of the charger type, finer than usb_type */
static ssize_t charger_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(to_power_supply(dev));
	u32 type;
	const char *name;

	guard(mutex)(&mca->lock);
	if (xiaomi_mca_read_type(mca, &type))
		return -ENODATA;
	name = xiaomi_mca_type_name(type);

	if (!name)
		return sysfs_emit(buf, "Unknown (%u)\n", type);

	return sysfs_emit(buf, "%s\n", name);
}
static DEVICE_ATTR_RO(charger_type);

static ssize_t xiaomi_mca_otg_show(struct device *dev, char *buf, u32 prop)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(to_power_supply(dev));
	u32 value = 0;
	u8 plugin;
	int ret;

	guard(mutex)(&mca->lock);
	if (prop == MCA_PROP_OTG_PLUGIN || prop == MCA_PROP_CC_TOGGLE) {
		ret = xiaomi_mca_read_blob(mca, prop, &plugin, sizeof(plugin));
		if (!ret)
			value = plugin;
	} else {
		ret = xiaomi_mca_read(mca, prop, &value);
	}
	return ret ?: sysfs_emit(buf, "%u\n", value);
}

static ssize_t otg_boost_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_OTG_BOOST_STATUS);
}
static DEVICE_ATTR_RO(otg_boost_status);

static ssize_t otg_gate_status_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_OTG_GATE_STATUS);
}
static DEVICE_ATTR_RO(otg_gate_status);

static ssize_t otg_plugin_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_OTG_PLUGIN);
}
static DEVICE_ATTR_RO(otg_plugin);

static ssize_t typec_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_TYPEC_MODE);
}
static DEVICE_ATTR_RO(typec_mode);

static ssize_t cc_toggle_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_CC_TOGGLE);
}

static ssize_t cc_toggle_store(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t count)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(to_power_supply(dev));
	u8 enable;
	int value, ret;

	/* Stock cc_toggle_store -> PD class -> adsp_protocol_set_cc_toggle. */
	ret = kstrtoint(buf, 10, &value);
	if (ret)
		return ret;
	enable = !!value;
	guard(mutex)(&mca->lock);
	ret = xiaomi_mca_write_blob(mca, MCA_PROP_CC_TOGGLE, &enable, sizeof(enable));
	return ret ?: count;
}
static DEVICE_ATTR_RW(cc_toggle);

static ssize_t snk_src_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_otg_show(dev, buf, MCA_PROP_SNK_SRC_MODE);
}
static DEVICE_ATTR_RO(snk_src_mode);

static struct attribute *xiaomi_mca_usb_attrs[] = {
	&dev_attr_charger_type.attr,
	&dev_attr_otg_boost_status.attr,
	&dev_attr_otg_gate_status.attr,
	&dev_attr_otg_plugin.attr,
	&dev_attr_typec_mode.attr,
	&dev_attr_cc_toggle.attr,
	&dev_attr_snk_src_mode.attr,
	NULL,
};

static const struct attribute_group xiaomi_mca_usb_group = {
	.attrs = xiaomi_mca_usb_attrs,
};

/* Read-only protocol diagnostics. Adapter authentication is kernel-owned. */
static struct xiaomi_mca *xiaomi_mca_from_dev(struct device *dev)
{
	return power_supply_get_drvdata(to_power_supply(dev));
}

static ssize_t request_vdm_cmd_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	__le32 auth[MCA_VDM_LEN / sizeof(__le32)];
	__le32 val;
	u32 state;
	int ret;

	guard(mutex)(&mca->lock);

	ret = xiaomi_mca_read_blob(mca, MCA_PROP_UVDM_STATE, &val, sizeof(val));
	if (ret)
		return ret;
	state = le32_to_cpu(val);

	switch (state) {
	case 1 ... 3:
		ret = xiaomi_mca_read_blob(mca, MCA_PROP_VDM_BASE + state - 1,
				     &val, sizeof(val));
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u,%u\n", state, le32_to_cpu(val));
	case 5:
		ret = xiaomi_mca_read_blob(mca, MCA_PROP_VDM_AUTH, auth, sizeof(auth));
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u,%08x%08x%08x%08x\n", state,
				  le32_to_cpu(auth[0]), le32_to_cpu(auth[1]),
				  le32_to_cpu(auth[2]), le32_to_cpu(auth[3]));
	default:
		return sysfs_emit(buf, "%u,Null\n", state);
	}
}

static DEVICE_ATTR_RO(request_vdm_cmd);

static ssize_t xiaomi_mca_show_u32(struct device *dev, char *buf, u32 prop,
				  const char *fmt)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	__le32 val;
	int ret;

	guard(mutex)(&mca->lock);

	ret = xiaomi_mca_read_blob(mca, prop, &val, sizeof(val));
	if (ret)
		return ret;

	return sysfs_emit(buf, fmt, le32_to_cpu(val));
}


static ssize_t adapter_id_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_show_u32(dev, buf, MCA_PROP_ADAPTER_ID, "%08x\n");
}
static DEVICE_ATTR_RO(adapter_id);

static ssize_t adapter_svid_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_show_u32(dev, buf, MCA_PROP_ADAPTER_SVID, "%04x\n");
}
static DEVICE_ATTR_RO(adapter_svid);

/*
 * Maximum voltage of the second source PDO, zero when the adapter offers
 * only 5 V.  Stock tests the same field before starting authentication.
 */
static ssize_t pdo2_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	int ret;

	guard(mutex)(&mca->lock);

	ret = xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos,
			     sizeof(pdos));
	if (ret)
		return ret;

	return sysfs_emit(buf, "%08x\n", le32_to_cpu(pdos[1].max_mv));
}
static DEVICE_ATTR_RO(pdo2);

static ssize_t real_type_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return charger_type_show(dev, attr, buf);
}
static DEVICE_ATTR_RO(real_type);

static ssize_t pd_verifed_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_show_u32(dev, buf, MCA_PROP_PD_VERIFIED, "%u\n");
}

static DEVICE_ATTR_RO(pd_verifed);

static ssize_t verify_process_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_show_u32(dev, buf, MCA_PROP_VERIFY_PROCESS, "%u\n");
}

static DEVICE_ATTR_RO(verify_process);

static ssize_t data_role_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	__le32 val;
	int ret;

	guard(mutex)(&mca->lock);

	ret = xiaomi_mca_read_blob(mca, MCA_PROP_DATA_ROLE, &val,
			     sizeof(val));
	if (ret)
		return ret;

	switch (le32_to_cpu(val)) {
	case MCA_DATA_ROLE_UFP:
		return sysfs_emit(buf, "ufp\n");
	case MCA_DATA_ROLE_DFP:
		return sysfs_emit(buf, "dfp\n");
	default:
		return sysfs_emit(buf, "none\n");
	}
}

static DEVICE_ATTR_RO(data_role);

static ssize_t charge_profile_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);

	char t[2][16];
	int i;

	guard(mutex)(&mca->lock);
	for (i = 0; i < ARRAY_SIZE(t); i++) {
		if (!mca->connector.valid[i])
			strscpy(t[i], "unavailable", sizeof(t[i]));
		else
			snprintf(t[i], sizeof(t[i]), "%d", mca->connector.temp[i]);
	}
	return sysfs_emit(buf, "active=%s stage=%u target_ma=%u ffc=%u cp=%u source_ma=%u request_ma=%u bus_ceiling_ma=%u hv=%u hv_pps=%u request_mv=%u attach=%u type=%u pump_type=%u exits=%u retry_ms=%u aicl_cap_ma=%u hv_exit=%s last_exit=%s conn_mc=%s conn2_mc=%s antiburn=%u\n",
		 xiaomi_mca_profile_names[mca->cp_profile],
		 mca->cp_stage, mca->cp_target_ma, mca->ffc_active, mca->cp,
		 mca->cp_ma, mca->cp_request_ma, mca->cp_ibus_ma, mca->hv, mca->hv_pps,
		 xiaomi_mca_cp_active(mca) ? mca->cp_mv : 0, READ_ONCE(mca->attach_generation),
		 mca->real_type, xiaomi_mca_cp_active(mca) ? mca->cp_type : 0, mca->cp_exits,
		 mca->cp_retry_ms, mca->aicl_cap_ma, mca->hv_last_exit ?: "none", mca->cp_last_exit ?: "none",
		 t[0], t[1], mca->connector.tripped);
}
static DEVICE_ATTR_RO(charge_profile);

static struct attribute *xiaomi_mca_xiaomi_attrs[] = {
	&dev_attr_charge_profile.attr,
	&dev_attr_request_vdm_cmd.attr,
	&dev_attr_adapter_id.attr,
	&dev_attr_adapter_svid.attr,
	&dev_attr_pdo2.attr,
	&dev_attr_real_type.attr,
	&dev_attr_pd_verifed.attr,
	&dev_attr_verify_process.attr,
	&dev_attr_data_role.attr,
	NULL,
};

static umode_t xiaomi_mca_xiaomi_visible(struct kobject *kobj,
					struct attribute *attr, int n)
{
	/* Reads remain useful while disabled; the write whitelist gates auth. */
	return attr->mode;
}

static const struct attribute_group xiaomi_mca_xiaomi_group = {
	.name = "xiaomi",
	.attrs = xiaomi_mca_xiaomi_attrs,
	.is_visible = xiaomi_mca_xiaomi_visible,
};

static const struct attribute_group *xiaomi_mca_usb_groups[] = {
	&xiaomi_mca_usb_group,
	&xiaomi_mca_xiaomi_group,
	NULL,
};

static const struct {
	const char *name;
	u32 property;
} xiaomi_mca_debugfs_props[] = {
	{ "usb_online", MCA_PROP_USB_ONLINE },
	{ "usb_voltage_uv", MCA_PROP_USB_VOLTAGE_NOW },
	{ "usb_current_ua", MCA_PROP_USB_CURRENT_NOW },
	{ "usb_real_type", MCA_PROP_USB_REAL_TYPE },
	{ "chgr_status", MCA_PROP_CHGR_STATUS },
	{ "charging_enabled", MCA_PROP_CHARGING_ENABLED },
	{ "charge_current_ma", MCA_PROP_CHARGE_CURRENT },
	{ "term_voltage_raw", MCA_PROP_TERM_VOLT },
	{ "term_current_raw", MCA_PROP_TERM_CURRENT },
	{ "pack_voltage_mv", MCA_PROP_PACK_VBAT },
	{ "input_current_limit_ma", MCA_PROP_INPUT_CURRENT_LIMIT },
	{ "aicl_enabled", MCA_PROP_AICL_ENABLED },
	{ "aicl_status", MCA_PROP_AICL_STATUS },
	{ "input_suspend", MCA_PROP_INPUT_SUSPEND },
};

/* Uncached reads of the properties above; the raw values, no unit conversion. */
static int xiaomi_mca_status_show(struct seq_file *s, void *unused)
{
	struct xiaomi_mca *mca = s->private;
	unsigned int i;
	u32 value;
	int ret;

	guard(mutex)(&mca->lock);

	for (i = 0; i < ARRAY_SIZE(xiaomi_mca_debugfs_props); i++) {
		ret = xiaomi_mca_read(mca, xiaomi_mca_debugfs_props[i].property, &value);
		if (ret)
			seq_printf(s, "%s %#x error %d\n", xiaomi_mca_debugfs_props[i].name,
				   xiaomi_mca_debugfs_props[i].property, ret);
		else
			seq_printf(s, "%s %#x %u\n", xiaomi_mca_debugfs_props[i].name,
				   xiaomi_mca_debugfs_props[i].property, value);
	}

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xiaomi_mca_status);

static int xiaomi_mca_stats_show(struct seq_file *s, void *unused)
{
	struct xiaomi_mca *mca = s->private;
	struct xiaomi_mca_stats stats;
	bool service_up;

	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		stats = mca->stats;
		service_up = mca->service_up;
	}

	seq_printf(s, "service_up %d\n", service_up);
	seq_printf(s, "read_requests %llu\n", stats.reads);
	seq_printf(s, "write_requests %llu\n", stats.writes);
	seq_printf(s, "replies %llu\n", stats.replies);
	seq_printf(s, "firmware_errors %llu\n", stats.errors);
	seq_printf(s, "malformed_replies %llu\n", stats.malformed);
	seq_printf(s, "timeouts %llu\n", stats.timeouts);
	seq_printf(s, "unmatched_messages %llu\n", stats.unmatched);
	seq_printf(s, "notifications %llu\n", stats.notifications);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xiaomi_mca_stats);

/* The last request sent and the last reply matched to a request. */
static int xiaomi_mca_frames_show(struct seq_file *s, void *unused)
{
	struct xiaomi_mca *mca = s->private;
	u8 reply[sizeof(struct xiaomi_mca_reply)];
	struct xiaomi_mca_request req;
	size_t reply_len;

	scoped_guard(mutex, &mca->lock)
		req = mca->req;

	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		reply_len = mca->reply_len;
		memcpy(reply, mca->reply, reply_len);
	}

	seq_printf(s, "request %zu\n", sizeof(req));
	seq_hex_dump(s, "", DUMP_PREFIX_OFFSET, 16, 4, &req, sizeof(req), false);
	seq_printf(s, "reply %zu\n", reply_len);
	seq_hex_dump(s, "", DUMP_PREFIX_OFFSET, 16, 4, reply, reply_len, false);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xiaomi_mca_frames);

static int xiaomi_mca_notifications_show(struct seq_file *s, void *unused)
{
	struct xiaomi_mca *mca = s->private;
	struct xiaomi_mca_notify_record records[MCA_NOTIFY_HISTORY];
	u32 count, head, i;
	u64 dropped;

	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		memcpy(records, mca->notify, sizeof(records));
		count = mca->notify_count;
		head = mca->notify_head;
		dropped = mca->notify_dropped;
	}
	seq_printf(s, "dropped_pending %llu\n", dropped);
	scoped_guard(mutex, &mca->lock)
		seq_printf(s, "config_attempted %u generation %u result %d\n",
			   mca->config_attempted, mca->config_generation, mca->config_result);
	seq_puts(s, "boottime_ns generation id data0 data1 data2 data3\n");
	for (i = 0; i < count; i++) {
		struct xiaomi_mca_notify_record *r;

		r = &records[(head + MCA_NOTIFY_HISTORY - count + i) % MCA_NOTIFY_HISTORY];
		seq_printf(s, "%llu %u %u %08x %08x %08x %08x\n",
			   r->time_ns, r->generation, r->id,
			   r->data[0], r->data[1], r->data[2], r->data[3]);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(xiaomi_mca_notifications);

static void xiaomi_mca_stop_poll(void *data)
{
	struct xiaomi_mca *mca = data;

	scoped_guard(spinlock_irqsave, &mca->rx_lock)
		mca->poll_ready = false;
	disable_delayed_work_sync(&mca->poll_work);
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		mca->input_pending = false;
		if (xiaomi_mca_charge_wake_locked(mca, false) && mca->psy)
			power_supply_changed(mca->psy);
	}
}

static void xiaomi_mca_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

/* The next driver, or none, must not inherit a limit set for this source. */
static void xiaomi_mca_unbind_restore(void *data)
{
	struct xiaomi_mca *mca = data;
	int ret;

	guard(mutex)(&mca->lock);
	xiaomi_mca_pen_off(mca);
	ret = xiaomi_mca_clear_auth(mca);
	if (ret)
		dev_warn(mca->dev, "failed to clear authentication interlock: %d\n", ret);
	ret = xiaomi_mca_restore_term_volt(mca);
	if (ret)
		dev_warn(mca->dev, "failed to restore float voltage: %d\n", ret);
	if (xiaomi_mca_cp_stop(mca, "unbind")) {
		/* Unbind cannot fail; the stage driver retains its stop watcher. */
		power_supply_put(mca->cp_psy);
		mca->cp_psy = NULL;
		return;
	}
	scoped_guard(spinlock_irqsave, &mca->rx_lock)
		mca->charge_end = 100;
	ret = xiaomi_mca_charge_limit_poll(mca);
	if (ret)
		dev_warn(mca->dev, "failed to release battery limit: %d\n", ret);
	xiaomi_mca_restore_icl(mca);
	if (mca->fcc_written)
		xiaomi_mca_reset_fcc(mca);
	if (mca->voltage_written)
		xiaomi_mca_request_volt(mca, 5000);
}

/* Hand off the buck under a confirmed fixed contract. */
static int xiaomi_mca_sleep_currents(struct xiaomi_mca *mca, u32 fixed_ma)
{
	u32 type, icl, fcc, tbat;
	int ret;

	if (!mca->icl_written && !mca->fcc_written)
		return 0;

	ret = xiaomi_mca_read_type(mca, &type);
	if (ret)
		return ret;
	icl = xiaomi_mca_policy_icl(type);
	fcc = xiaomi_mca_policy_fcc(type, fixed_ma != 0);
	if (xiaomi_mca_probe_5v(type)) {
		u32 vbus;

		if (xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus) ||
		    vbus < MCA_VBUS_ONLINE_UV || vbus >= MCA_HV_VBUS_5V_MAX_UV ||
		    xiaomi_mca_read_u32(mca, MCA_PROP_USB_ONLINE) != 1 ||
		    (type == MCA_TYPE_UNKNOWN && mca->real_type != type))
			return -EOPNOTSUPP;
		ret = xiaomi_mca_aicl(mca, false);
		if (ret)
			return ret;
		icl = fcc = xiaomi_mca_5v_input(mca, type);
	}
	if (!icl || !fcc)
		return -EOPNOTSUPP;
	if (fixed_ma)
		icl = min3(fixed_ma, xiaomi_mca_contract_ma(mca, MCA_HV_MV, 1600),
			   (u32)MCA_ICL_HV_MAX_MA);
	else if (xiaomi_mca_is_qc(type))
		icl = min(icl, xiaomi_mca_hv_icl(type));
	else if (xiaomi_mca_is_pd(type) || type == MCA_TYPE_C) {
		icl = xiaomi_mca_5v_input(mca, type);
		/* Both the current and the requested 5 V contract must allow it. */
		if (mca->voltage_written)
			icl = min(icl, xiaomi_mca_hv_icl(type) ?: 500U);
		fcc = icl;
	}

	if (mca->fcc_written) {
		ret = xiaomi_mca_read(mca, MCA_PROP_PACK_TBAT, &tbat);
		if (ret)
			return ret;
		fcc = xiaomi_mca_fcc_thermal(mca, fcc, min((s32)tbat, mca->coolest_temp));
		ret = xiaomi_mca_write_u32(mca, MCA_PROP_CHARGE_CURRENT, fcc);
		if (ret)
			return ret;
		mca->fcc_set_ma = fcc;
	}
	if (mca->icl_written) {
		ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, icl);
		if (ret)
			return ret;
		mca->icl_set_ma = icl;
	}
	/* Keep the written flags: detach/resume must reset these setpoints. */
	return 0;
}

/* Stop host-controlled charging before its suppliers can suspend. */
static int xiaomi_mca_quiesce(struct xiaomi_mca *mca)
{
	u32 vbus, fixed_ma = 0;
	bool pps;
	int ret;

	ret = xiaomi_mca_pen_off(mca);
	if (ret)
		return ret;
	ret = xiaomi_mca_cp_stop(mca, "system sleep");
	if (ret)
		return ret;
	if (mca->ffc_voted) {
		ret = xiaomi_mca_write_u32(mca, MCA_PROP_TERM_VOLT,
					   mca->term_hot ? MCA_TERM_HOT_MV : MCA_TERM_FULL_MV);
		if (ret)
			return ret;
		mca->ffc_active = false;
	}
	ret = xiaomi_mca_clear_auth(mca);
	if (ret)
		return ret;

	/*
	 * With no source attached there is nothing to hand back: the limits
	 * and voltage only matter for the next source, and the attach path in
	 * xiaomi_mca_policy_poll() resets them before any policy runs.
	 * Refusing system sleep here kept a tablet that had been charged once
	 * from ever suspending on battery. An attached source of unknown type
	 * still refuses it, as no automatic write is allowed for that one.
	 */
	if (mca->icl_written || mca->fcc_written || mca->voltage_written) {
		ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
		if (ret)
			return ret;
		if (vbus < MCA_VBUS_ONLINE_UV)
			goto out;
	}

	/* Standard PD can maintain a fixed 9 V PDO without CPU keepalives.
	 * Always name the fixed PDO explicitly, even if PPS already reads 9 V. */
	if (hv_charge && mca->voltage_written && xiaomi_mca_is_pd(mca->real_type) &&
	    !mca->charge_blocked) {
		fixed_ma = xiaomi_mca_pd_9v_ma(mca, &pps);
		if (fixed_ma && !pps) {
			ret = xiaomi_mca_write_u32(mca, MCA_PROP_PD_FIXED_VOLT, MCA_HV_MV);
			if (!ret) {
				int read_ret;

				ret = read_poll_timeout(xiaomi_mca_read, read_ret,
					read_ret || (vbus >= MCA_HV_VBUS_MIN_UV && vbus <= MCA_HV_VBUS_MAX_UV),
					jiffies_to_usecs(MCA_POLL_INTERVAL),
					jiffies_to_usecs(MCA_POLL_INTERVAL * MCA_HV_SETTLE_POLLS),
					false, mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
				ret = ret ?: read_ret;
			}
			if (ret)
				fixed_ma = 0;
		} else {
			fixed_ma = 0;
		}
	}
	if (!fixed_ma) {
		/* Downrate before requesting 5 V. */
		ret = xiaomi_mca_sleep_currents(mca, 0);
		if (ret)
			return ret;
		ret = xiaomi_mca_restore_voltage(mca);
	} else {
		ret = xiaomi_mca_sleep_currents(mca, fixed_ma);
	}
	if (ret)
		return ret;

out:
	mca->cp = MCA_CP_OFF;
	mca->hv = fixed_ma ? MCA_HV_ON : MCA_HV_OFF;
	mca->voltage_written = !!fixed_ma;
	mca->hv_pps = false;
	mca->ffc_active = false;
	mca->auth_ok = false;
	mca->mipps_attempted = false;
	mca->online = false;
	mca->policy_active = false;
	return 0;
}

/* A reconnect can be reported after prepare() has drained the worker.
 * The noirq admission check uses the same cached state, without GLINK I/O.
 */
static int xiaomi_mca_check_sleep(struct device *dev)
{
	struct xiaomi_mca *mca = dev_get_drvdata(dev);

	guard(spinlock_irqsave)(&mca->rx_lock);
	return mca->charge_awake || mca->input_pending || READ_ONCE(mca->connector.tripped) ? -EBUSY : 0;
}

/*
 * All prepare callbacks precede supplier suspend, and all supplier resumes
 * finish before complete. A freezable queue alone cannot hand back the
 * host's charging policy or protect transactions from other kernel callers.
 */
static int xiaomi_mca_prepare(struct device *dev)
{
	struct xiaomi_mca *mca = dev_get_drvdata(dev);
	int ret;

	/* Direct requests can bypass wakeup_count. Leave an active worker
	 * running, especially while it is servicing the pump watchdog.
	 */
	ret = xiaomi_mca_check_sleep(dev);
	if (ret)
		return ret;
	disable_delayed_work_sync(&mca->poll_work);
	scoped_guard(mutex, &mca->lock) {
		ret = xiaomi_mca_check_sleep(dev);
		if (!ret) {
			ret = xiaomi_mca_quiesce(mca);
			if (ret)
				dev_err(dev, "cannot quiesce charging for system sleep: %d\n", ret);
		}
		if (!ret) {
			mca->sleeping = true;
			mca->last_valid = false;
			xiaomi_mca_invalidate(mca);
		}
	}
	if (ret) {
		enable_delayed_work(&mca->poll_work);
		schedule_delayed_work(&mca->poll_work, 0);
	}
	return ret;
}

static void xiaomi_mca_complete(struct device *dev)
{
	struct xiaomi_mca *mca = dev_get_drvdata(dev);

	scoped_guard(mutex, &mca->lock) {
		if (!mca->sleeping)
			return;
		mca->sleeping = false;
		/* Autonomous firmware owned the buck during sleep. Reapply even
		 * when its pre-sleep setpoint equals the current policy target. */
		mca->icl_set_ma = 0;
		mca->fcc_set_ma = 0;
		mca->pen_attempted = false;
	}
	enable_delayed_work(&mca->poll_work);
	schedule_delayed_work(&mca->poll_work, 0);
}

static const struct dev_pm_ops xiaomi_mca_pm_ops = {
	.prepare = xiaomi_mca_prepare,
	.suspend_noirq = xiaomi_mca_check_sleep,
	.complete = xiaomi_mca_complete,
};

static int xiaomi_mca_threshold_get(struct power_supply *psy,
				    const struct power_supply_ext *ext, void *data,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct xiaomi_mca *mca = data;

	guard(spinlock_irqsave)(&mca->rx_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = mca->charge_start;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = mca->charge_end;
		return 0;
	default:
		return -EINVAL;
	}
}

static int xiaomi_mca_threshold_set(struct power_supply *psy,
				    const struct power_supply_ext *ext, void *data,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct xiaomi_mca *mca = data;
	int value = val->intval;

	/* No mca->lock here: power_supply callers hold extensions_sem. */
	scoped_guard(spinlock_irqsave, &mca->rx_lock) {
		switch (psp) {
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
			if (value < 0 || value >= mca->charge_end)
				return -EINVAL;
			mca->charge_start = value;
			break;
		case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
			if (value < 1 || value > 100 || value <= mca->charge_start)
				return -EINVAL;
			mca->charge_end = value;
			break;
		default:
			return -EINVAL;
		}
		if (mca->poll_ready)
			mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
	}
	power_supply_changed(mca->batteries[0]);
	power_supply_changed(mca->batteries[1]);
	return 0;
}

static int xiaomi_mca_threshold_writeable(struct power_supply *psy,
					  const struct power_supply_ext *ext, void *data,
					  enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static const enum power_supply_property xiaomi_mca_threshold_props[] = {
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_ext xiaomi_mca_battery_ext = {
	.name = "xiaomi-mca",
	.properties = xiaomi_mca_threshold_props,
	.num_properties = ARRAY_SIZE(xiaomi_mca_threshold_props),
	.get_property = xiaomi_mca_threshold_get,
	.set_property = xiaomi_mca_threshold_set,
	.property_is_writeable = xiaomi_mca_threshold_writeable,
};

static void xiaomi_mca_battery_put(void *data)
{
	power_supply_put(data);
}

static void xiaomi_mca_battery_ext_remove(void *data)
{
	power_supply_unregister_extension(data, &xiaomi_mca_battery_ext);
}

static int xiaomi_mca_battery_ext_register(struct xiaomi_mca *mca)
{
	int n, ret;

	for (n = 0; n < ARRAY_SIZE(mca->batteries); n++) {
		mca->batteries[n] = power_supply_get_by_name(xiaomi_mca_supplied_to[n]);
		if (!mca->batteries[n])
			return -EPROBE_DEFER;
		ret = devm_add_action_or_reset(mca->dev, xiaomi_mca_battery_put,
					       mca->batteries[n]);
		if (ret)
			return ret;
	}
	for (n = 0; n < ARRAY_SIZE(mca->batteries); n++) {
		ret = power_supply_register_extension(mca->batteries[n],
						      &xiaomi_mca_battery_ext, mca->dev, mca);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(mca->dev, xiaomi_mca_battery_ext_remove,
					       mca->batteries[n]);
		if (ret)
			return ret;
	}
	return 0;
}

static int xiaomi_mca_connector_init(struct xiaomi_mca *mca)
{
	struct device *dev = mca->dev;
	struct mca_connector *c = &mca->connector;
	static const char * const names[] = { "connector", "connector2", "quiet", "charger", "wifi", "cpu" };
	int i;

	if (!device_property_present(dev, "xiaomi,connector-cutoff-gpios"))
		return mca->piano_otg ? -ENODEV : 0;
	if (!IS_REACHABLE(CONFIG_IIO))
		return -ENODEV;
	c->temp[0] = c->temp[1] = INT_MIN;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		c->ntc[i] = devm_iio_channel_get(dev, names[i]);
		if (IS_ERR(c->ntc[i]))
			return PTR_ERR(c->ntc[i]);
	}
	/* Piano GPIO9 is active high. Stock drives raw low at probe. */
	c->cutoff = devm_gpiod_get(dev, "xiaomi,connector-cutoff", GPIOD_OUT_LOW);
	return PTR_ERR_OR_ZERO(c->cutoff);
}

static int xiaomi_mca_probe(struct auxiliary_device *adev,
			    const struct auxiliary_device_id *id)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &adev->dev;
	struct xiaomi_mca *mca;
	int ret;

	mca = devm_kzalloc(dev, sizeof(*mca), GFP_KERNEL);
	if (!mca)
		return -ENOMEM;

	mca->dev = dev;
	mca->piano_otg = of_device_is_compatible(dev->of_node, "xiaomi,piano-pmic-glink");
	ret = xiaomi_mca_connector_init(mca);
	if (ret)
		return dev_err_probe(dev, ret, "connector protection unavailable\n");
	mca->hv_retry = jiffies;
	mca->charge_end = 100;
	dev_set_drvdata(dev, mca);
	init_completion(&mca->ack);
	spin_lock_init(&mca->rx_lock);
	ret = devm_mutex_init(dev, &mca->lock);
	if (ret)
		return ret;
	/* Released after polling, restore and the GLINK client are gone. */
	ret = xiaomi_mca_charge_wake_init(mca);
	if (ret)
		return ret;

	/*
	 * Allocated first, so that devres releases it last: the power supply
	 * and debugfs readers send requests through it until they are gone.
	 */
	ret = devm_device_init_wakeup(dev);
	if (ret)
		return ret;

	mca->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						   xiaomi_mca_callback,
						   xiaomi_mca_pdr_notify, mca);
	if (IS_ERR(mca->client))
		return PTR_ERR(mca->client);

	/*
	 * Released after the power supply, debugfs and polling, which can no
	 * longer write then, and before the client it sends through.
	 */
	ret = devm_add_action(dev, xiaomi_mca_unbind_restore, mca);
	if (ret)
		return ret;

	ret = xiaomi_mca_register_pen(mca);
	if (ret)
		return ret;

	psy_cfg.attr_grp = xiaomi_mca_usb_groups;
	psy_cfg.supplied_to = xiaomi_mca_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(xiaomi_mca_supplied_to);
	psy_cfg.drv_data = mca;
	psy_cfg.fwnode = dev_fwnode(dev);
	mca->psy = devm_power_supply_register(dev, &xiaomi_mca_usb_psy_desc, &psy_cfg);
	if (IS_ERR(mca->psy))
		return dev_err_probe(dev, PTR_ERR(mca->psy),
				     "failed to register USB power supply\n");

	/* Cancelled before the power supply is unregistered. */
	INIT_DELAYED_WORK(&mca->poll_work, xiaomi_mca_poll);
	mca->psy_nb.notifier_call = xiaomi_mca_psy_notifier;
	ret = power_supply_reg_notifier(&mca->psy_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, xiaomi_mca_psy_notifier_remove, mca);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, xiaomi_mca_stop_poll, mca);
	if (ret)
		return ret;

	mca->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0400, mca->debugfs, mca, &xiaomi_mca_status_fops);
	debugfs_create_file("stats", 0400, mca->debugfs, mca, &xiaomi_mca_stats_fops);
	debugfs_create_file("frames", 0400, mca->debugfs, mca, &xiaomi_mca_frames_fops);
	debugfs_create_file("notifications", 0400, mca->debugfs, mca,
			    &xiaomi_mca_notifications_fops);
	ret = devm_add_action_or_reset(dev, xiaomi_mca_debugfs_remove, mca->debugfs);
	if (ret)
		return ret;

	ret = xiaomi_mca_battery_ext_register(mca);
	if (ret)
		return dev_err_probe(dev, ret, "failed to extend batteries\n");

	scoped_guard(spinlock_irqsave, &mca->rx_lock)
		mca->poll_ready = true;
	pmic_glink_client_set_wakeup_filter(mca->client, xiaomi_mca_usb_notification);
	pmic_glink_client_register(mca->client);
	schedule_delayed_work(&mca->poll_work, 0);

	return 0;
}

static void xiaomi_mca_shutdown(struct auxiliary_device *adev)
{
	struct xiaomi_mca *mca = auxiliary_get_drvdata(adev);

	xiaomi_mca_stop_poll(mca);
	xiaomi_mca_unbind_restore(mca);
}

static const struct auxiliary_device_id xiaomi_mca_id_table[] = {
	{ .name = "pmic_glink.xiaomi-mca", },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, xiaomi_mca_id_table);

static struct auxiliary_driver xiaomi_mca_driver = {
	.name = "xiaomi_mca",
	.probe = xiaomi_mca_probe,
	.shutdown = xiaomi_mca_shutdown,
	.id_table = xiaomi_mca_id_table,
	.driver.pm = pm_sleep_ptr(&xiaomi_mca_pm_ops),
};
module_auxiliary_driver(xiaomi_mca_driver);

MODULE_DESCRIPTION("Xiaomi MCA charger firmware interface over PMIC GLINK");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
#include "xiaomi_mca_test.c"
#endif
