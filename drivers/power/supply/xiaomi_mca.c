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
 * MiPPS relay and SC8541 direct charging. Float voltage, termination, charging
 * enable and firmware JEITA are not written. All policy switches default off.
 * USB-C port acknowledgments belong to the existing AltMode client.
 */

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/devm-helpers.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/hex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/ktime.h>
#include <linux/power/sc8541_charger.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
#include <kunit/static_stub.h>
#endif

#define MCA_OPCODE_READ			1
#define MCA_OPCODE_WRITE		2

/* Charger (buck) properties, currents in mA */
#define MCA_PROP_CHGR_STATUS		0x2001
#define MCA_PROP_CHARGING_ENABLED	0x2003
#define MCA_PROP_CHARGE_CURRENT		0x2005
#define MCA_PROP_INPUT_CURRENT_LIMIT	0x2008
#define MCA_PROP_AICL_ENABLED		0x2012
#define MCA_PROP_INPUT_SUSPEND		0x2019
#define MCA_PROP_OTG_BOOST_STATUS	0x200d
#define MCA_PROP_OTG_GATE_STATUS		0x200e
#define MCA_PROP_OTG_CFG			0x201a
#define MCA_PROP_CID_CFG			0x201b
#define MCA_PROP_CP_STATE		0x201c
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
 * unstructured VDMs with the adapter, userspace computes the digests.
 * Command n (1-8) goes to property 0x21003 + n - 1, as in the stock
 * adsp_pd_protocol_request_vdm_cmd().
 */
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
	[17]			= "BPP",
	[18]			= "EPP",
	[19]			= "HPP",
};

/* VBUS above this means a source is attached (USB default is 5 V) */
#define MCA_VBUS_ONLINE_UV	4000000
#define MCA_POLL_MS		3000

/* Input current limits (mA) set by charge_policy, from the stock tables */
#define MCA_ICL_MIN_MA		100	/* also the ADSP power-on value */
#define MCA_ICL_MAX_MA		1500
#define MCA_ICL_HV_MAX_MA	2500	/* at 9 V, see xiaomi_mca_hv_icl() */
#define MCA_POLICY_ICL_DEFAULT_MA	500

static bool charge_policy;
module_param(charge_policy, bool, 0644);
MODULE_PARM_DESC(charge_policy,
		 "Raise the USB input current limit by charger type (default: off, ADSP defaults)");

/*
 * charge_current: left alone, the ADSP charges the battery at about 500 mA
 * whatever the source.  Stock sets the charge current by charger type (its
 * buck strategy chg_* values) and a JEITA table that allows far more than
 * the ceiling below between 13 and 48 degC; this sets the charger-type value,
 * capped.  Temperature only ever moves the charge current, never the input
 * limit or the bus voltage, so the system keeps running from the source:
 * above MCA_FCC_DERATE_FROM the current steps down by MCA_FCC_DERATE_STEP_MA
 * per degree to the fallback value, and comes back one step at a time, once
 * the battery is a degree cooler than that step needs and at most every
 * MCA_FCC_RAISE_MS.  Below MCA_FCC_TBAT_MIN it is the fallback value.  The
 * ADSP's own JEITA window (0-55 degC) still applies.  Detach, unbind and
 * ADSP restarts put the fallback value back.
 */
#define MCA_FCC_MIN_MA		100
#define MCA_FCC_MAX_MA		3000	/* the ADSP's own nominal value */
#define MCA_FCC_DEFAULT_MA	500
#define MCA_FCC_TBAT_MIN	15
#define MCA_FCC_DERATE_FROM	38
#define MCA_FCC_DERATE_STEP_MA	300
#define MCA_FCC_RAISE_MS	30000

static bool charge_current;
module_param(charge_current, bool, 0644);
MODULE_PARM_DESC(charge_current,
		 "Raise the battery charge current by charger type; needs charge_policy (default: off, about 500 mA)");

/*
 * hv_charge: a PD source with a fixed 9 V PDO is asked for 9 V, as the
 * stock buck strategy does, whatever the battery temperature (that is left
 * to the charge current).  A source without one whose PPS APDO covers 9 V
 * (some power banks list only a 5-11 V APDO) is asked for 9.0 V at its
 * APDO current, capped at MCA_HV_PPS_MAX_MA; the ADSP keeps the PPS
 * contract alive by itself.  The bus has to settle within the window below in
 * a few polls and stay there; otherwise the source goes back to 5 V until
 * it is detached.
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
 * pause, a few times per attach.
 */
#define MCA_HV_VBUS_5V_MAX_UV	6000000
#define MCA_HV_RENEG_MS		10000
/* with mipps_auth, 9 V waits this long after attach for the authentication */
#define MCA_HV_AUTH_WAIT_MS	15000
#define MCA_HV_RENEG_MAX	3
/* Current limit asked of a PPS source */
#define MCA_HV_PPS_MIN_MA	500
#define MCA_HV_PPS_MAX_MA	3000

static bool hv_charge;
module_param(hv_charge, bool, 0644);
MODULE_PARM_DESC(hv_charge,
		 "Ask PD sources with a fixed 9 V PDO for 9 V; needs charge_policy (default: off, 5 V)");


static bool mipps_auth;
module_param(mipps_auth, bool, 0644);
MODULE_PARM_DESC(mipps_auth,
		 "Expose the Xiaomi charger authentication (MiPPS) messages to userspace (default: off)");

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
 * stock's battery tables without FFC (the charger firmware floats at
 * 4350 mV), within a narrower window: direct charging only runs from
 * MCA_CP_VBAT_MIN_MV to MCA_CP_VBAT_MAX_MV and from MCA_CP_TBAT_MIN to
 * MCA_CP_TBAT_EXIT degC, and the buck charger does the rest.  The bus
 * target is cp_ibus_max, at most MCA_CP_IBUS_MAX_MA (stock div_max_curr)
 * from a verified Xiaomi adapter and MCA_CP_THIRD_IBUS_MA (stock
 * third_pps_ibus_max) from other PPS sources, and is lowered while the
 * stage is hot, see xiaomi_mca_cp_step().  Anything
 * unexpected (a read failure, the stage turning itself off, the bus out of
 * range, too little or too much current, detach, an ADSP restart, unbind)
 * turns the stage off and goes back to 9 V on the buck charger until the
 * source is detached.  The stage also turns itself off if it is not kicked
 * for 3 s, and its bus over-current protection is set 1 A above the target.
 */
#define MCA_CP_POLL_MS		500
#define MCA_CP_VBAT_MIN_MV	3500
#define MCA_CP_VBAT_START_MV	4100	/* only starts below this */
#define MCA_CP_VBAT_MAX_MV	4200
/* stock normal_volt_para18_35: 8 A, 7.23 A from 4150 mV */
#define MCA_CP_VSTEP_MV		4150
#define MCA_CP_VSTEP_MA		7230
/*
 * Battery temperature, degC, of the coolest and the warmest of the two
 * cells and the pack: stock batt_para_lwn allows 7.23 A from 13, 8 A from
 * 18 and 4.52 A from 48 to 55 degC (2 degC hysteresis on the warm side).
 */
#define MCA_CP_TBAT_MIN		15
#define MCA_CP_TBAT_FULL	18	/* MCA_CP_IBAT_REDUCED_MA below */
#define MCA_CP_TBAT_WARM	40	/* MCA_CP_IBAT_REDUCED_MA from */
#define MCA_CP_TBAT_EXIT	45
#define MCA_CP_TBAT_HYS		2
/*
 * The pack reading follows the current within seconds (40 to 37 degC in
 * 12 s after a step down): from MCA_CP_TBAT_WARM the limit is held
 * reduced this long, then comes back MCA_CP_TBAT_RAMP_MA at a time
 */
#define MCA_CP_TBAT_HOLD_MS	30000
#define MCA_CP_TBAT_RAMP_MA	500
#define MCA_CP_TBAT_RAMP_MS	15000
#define MCA_CP_IBAT_MAX_MA	8000
#define MCA_CP_IBAT_REDUCED_MA	4500
/*
 * Stage temperature, 0.1 degC: from MCA_CP_TDIE_HOT the bus target drops
 * MCA_CP_TDIE_STEP_MA a poll (the stage warms about 1 degC/s at 4 A), down
 * to MCA_CP_TDIE_MIN_MA, and comes back a quarter as fast below
 * MCA_CP_TDIE_HOT - MCA_CP_TDIE_HYS; MCA_CP_TDIE_MAX leaves.
 */
#define MCA_CP_TDIE_HOT		700
#define MCA_CP_TDIE_HYS		50
#define MCA_CP_TDIE_MAX		800
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
#define MCA_CP_PPS_MAX_MV	10000
#define MCA_CP_BUCK_MA		500	/* buck input limit and charge current */
/*
 * Stock (strategy_quickchg_pmic_single_cp_charging) has the buck charger
 * carry part of the battery current next to the stage once the source
 * gives more than MCA_CP_PAR_IBUS_MA (stage and buck charger together,
 * which the share moving between them leaves alone) with more than
 * MCA_CP_PAR_IBAT_MA asked of the battery: its 5000/2700 mA row, the one
 * within 8 A, at FCC 2000 and input 1500 mA.  That moves heat from the
 * stage to the PMIC.
 */
#define MCA_CP_PAR_IBUS_MA	2700
#define MCA_CP_PAR_IBAT_MA	5000
#define MCA_CP_PAR_HYS_MA	300
#define MCA_CP_PAR_FCC_MA	2000
#define MCA_CP_PAR_ICL_MA	1500
#define MCA_CP_PPS_MAX_MA	6000	/* current limit asked of the source */
#define MCA_CP_IBUS_MAX_MA	5000
#define MCA_CP_THIRD_IBUS_MA	4100
/* bus over-current protection of the stage, above the target */
#define MCA_CP_BUSOCP_MARGIN_MA	1000
#define MCA_CP_BUSOCP_MAX_MA	6000
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
/* with mipps_auth, waits this long after attach for the authentication */
#define MCA_CP_AUTH_WAIT_MS	60000
/* an authenticated adapter resets itself within this, see xiaomi_mca_auth_pending() */
#define MCA_AUTH_RESET_WAIT_MS	30000

static bool cp_charge;
module_param(cp_charge, bool, 0644);
MODULE_PARM_DESC(cp_charge,
		 "Charge through the SC8541 switched-capacitor stage from PPS sources; needs hv_charge and charge_current (default: off)");

static unsigned int cp_ibus_max = 4000;
module_param(cp_ibus_max, uint, 0444);
MODULE_PARM_DESC(cp_ibus_max,
		 "Bus current of the switched-capacitor stage, mA (500-5000, default 4000; at most what the source offers less 700, 4100 from unverified sources)");

static unsigned int cp_ibat_max = MCA_CP_IBAT_MAX_MA;
module_param(cp_ibat_max, uint, 0444);
MODULE_PARM_DESC(cp_ibat_max,
		 "Battery current while direct charging, both cells, mA (1000-8000, default 8000; lower by battery voltage and temperature)");



enum xiaomi_mca_hv {
	MCA_HV_OFF,		/* 5 V, may ask for 9 V */
	MCA_HV_REQUESTED,	/* 9 V asked for, waiting for the bus */
	MCA_HV_ON,		/* bus at 9 V */
	MCA_HV_BLOCKED,		/* back at 5 V until detach */
};

enum xiaomi_mca_cp {
	MCA_CP_OFF,		/* buck charging, may start */
	MCA_CP_OPENING,		/* source asked for, stage being started */
	MCA_CP_ON,		/* stage running */
	MCA_CP_STOPPING,	/* stop failed; retain ownership until confirmed off */
	MCA_CP_DONE,		/* buck charging until detach */
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

struct xiaomi_mca {
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
	bool fcc_written;
	bool voltage_written;	/* a raised voltage request may have reached firmware */
	struct xiaomi_mca_state icl_source;

	bool piano_otg;
	bool config_attempted;
	u32 config_generation;
	int config_result;
	bool cp_absent_warned;
	bool otg_active;

	struct delayed_work poll_work;
	bool sleeping;		/* no transactions between prepare and complete */
	bool online;
	int vbus_uv;
	int ibus_ua;
	int icl_ma;
	u32 real_type;
	int icl_set_ma;		/* last limit written by charge_policy, 0 if none */
	int fcc_set_ma;		/* last current written by charge_current, 0 if none */
	u32 fcc_full;		/* charger-type current fcc_set_ma was derived from */
	unsigned long fcc_raise_at;	/* jiffies before which it is not raised */
	enum xiaomi_mca_hv hv;
	int hv_polls;
	int hv_renegs;		/* times the source went back to 5 V on its own */
	bool hv_pps;		/* 9 V is asked of a PPS APDO */
	u32 hv_pps_ma;		/* its requested current limit */
	enum xiaomi_mca_cp cp;
	struct power_supply *cp_psy;	/* the stage, held while not MCA_CP_OFF */
	u32 cp_mv;		/* voltage asked of the PPS source */
	u32 cp_ma;		/* its current limit */
	u32 cp_max_mv;		/* the most the source offers */
	u32 cp_ibus_ma;		/* bus current target */
	u32 cp_ibus_hot;	/* the target as lowered for the stage temperature */
	u32 cp_ibat_trip;	/* battery current limit the trip checks against */
	/* jiffies when the limit was last that high, or the stage above target */
	unsigned long cp_trip_at;
	u32 cp_warm_ma;		/* battery limit as reduced for temperature */
	unsigned long cp_warm_at;	/* jiffies when it last changed */
	bool cp_buck_par;	/* buck charger at MCA_CP_PAR_* */
	int cp_polls;		/* while opening: tries; then: polls */
	int cp_low;		/* consecutive polls below MCA_CP_LOW_IBUS_MA */
	u32 cp_ibus_last;	/* stage and buck input current at the last step */
	u32 cp_ibus_cap;	/* learned from a source dropping out, 0 if none */
	unsigned long cp_off_at;	/* jiffies when the source went away */
	unsigned long cp_on_at;		/* jiffies when it came */
	bool auth_ok;		/* xiaomi-mipps-auth reported the adapter verified */
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
static const struct xiaomi_mca_limits xiaomi_mca_pps_limits = { 2500, 3000 };

static const struct xiaomi_mca_limits *xiaomi_mca_limits(u32 real_type)
{
	if (real_type < ARRAY_SIZE(xiaomi_mca_type_limits) &&
	    xiaomi_mca_type_limits[real_type].input_current_ma)
		return &xiaomi_mca_type_limits[real_type];

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
	if (hv && real_type == MCA_USB_REAL_TYPE_PD)
		limits = &xiaomi_mca_pd_limits;
	else if (hv && (real_type == MCA_USB_REAL_TYPE_PD_VERIFY ||
		       real_type == MCA_USB_REAL_TYPE_PPS))
		limits = &xiaomi_mca_pps_limits;

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
	u32 real_type;
	int ret;

	lockdep_assert_held(&mca->lock);

	if (!mca->icl_written)
		return 0;

	/* Unknown also forbids automatic writes; wait for a recognized source. */
	ret = xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &real_type);
	if (ret || !xiaomi_mca_limits(real_type))
		return ret ?: -EOPNOTSUPP;

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
	case MCA_PROP_CID_CFG:
		return len == sizeof(u32) && val == MCA_PIANO_CID_CFG;
	case MCA_PROP_CP_STATE:
	case MCA_PROP_CC_TOGGLE:
		return len == 1 && *(const u8 *)data <= 1;
	case MCA_PROP_OTG_CFG:
		return len == sizeof(xiaomi_mca_piano_otg_cfg) &&
		       !memcmp(data, xiaomi_mca_piano_otg_cfg, len);
	case MCA_PROP_INPUT_CURRENT_LIMIT:
		return len == sizeof(u32) &&
		       val >= MCA_ICL_MIN_MA && val <= MCA_ICL_HV_MAX_MA;
	case MCA_PROP_CHARGE_CURRENT:
		return len == sizeof(u32) &&
		       val >= MCA_FCC_MIN_MA && val <= MCA_FCC_MAX_MA;
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
	case MCA_PROP_VDM_REVERSE:
		return mipps_auth && len == MCA_VDM_LEN;
	case MCA_PROP_DATA_ROLE:
		return mipps_auth && len == sizeof(u32) &&
		       (val == MCA_DATA_ROLE_UFP || val == MCA_DATA_ROLE_DFP);
	case MCA_PROP_VDM_VERIFIED:
	case MCA_PROP_VDM_REMOVE_COMP:
	case MCA_PROP_PD_VERIFIED:
	case MCA_PROP_VERIFY_PROCESS:
		return mipps_auth && len == sizeof(u32) && val <= 1;
	default:
		return false;
	}
}

static bool xiaomi_mca_is_pd(u32 real_type);

/* All paths share the original checked transport and this refusal gate. */
static int xiaomi_mca_write_blob(struct xiaomi_mca *mca, u32 prop,
				 const void *data, size_t len)
{
	u8 buf[MCA_VDM_LEN];
	u32 type, vbus, value, checked, readback, generation;
	int ret;

	if (len > sizeof(buf) || !xiaomi_mca_write_allowed(prop, data, len))
		return -EPERM;
	lockdep_assert_held(&mca->lock);
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
	if (prop == MCA_PROP_CP_STATE || prop == MCA_PROP_OTG_CFG || prop == MCA_PROP_CID_CFG) {
		if (!mca->piano_otg)
			return -EOPNOTSUPP;
		memcpy(buf, data, len);
		return xiaomi_mca_transfer_data(mca, MCA_OPCODE_WRITE, prop, buf, len);
	}
	ret = xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &type);
	if (ret)
		return ret;
	if (!xiaomi_mca_limits(type))
		return -EOPNOTSUPP;
	if (prop >= MCA_PROP_VERIFY_PROCESS && !xiaomi_mca_is_pd(type))
		return -EOPNOTSUPP;
	if (prop == MCA_PROP_INPUT_CURRENT_LIMIT || prop == MCA_PROP_CHARGE_CURRENT) {
		ret = xiaomi_mca_read(mca, MCA_PROP_USB_VOLTAGE_NOW, &vbus);
		if (ret)
			return ret;
		value = get_unaligned_le32(data);
		ret = xiaomi_mca_current_ma(prop == MCA_PROP_INPUT_CURRENT_LIMIT ?
			POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT :
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, type,
			vbus >= MCA_HV_VBUS_MIN_UV && vbus <= MCA_HV_VBUS_MAX_UV,
			value * MILLI, &checked);
		if (ret)
			return ret;
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT) {
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
	    (prop == MCA_PROP_PD_FIXED_VOLT && get_unaligned_le32(data) > 5000))
		mca->voltage_written = true;
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

static u32 xiaomi_mca_read_u32(struct xiaomi_mca *mca, u32 prop)
{
	__le32 val;

	if (xiaomi_mca_read_blob(mca, prop, &val, sizeof(val)))
		return U32_MAX;

	return le32_to_cpu(val);
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
	/* Preserve the community policy's lower Type-C/FLOAT fallback. */
	switch (real_type) {
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

/*
 * Current a PD source offers at 5 V: the lowest maximum among the PDOs and
 * APDOs whose range covers 5 V (some PPS sources list only a 5-11 V APDO),
 * or the default if there is none.
 */
static u32 xiaomi_mca_pd_5v_ma(struct xiaomi_mca *mca)
{
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 ma = U32_MAX;
	int i;

	if (xiaomi_mca_read_blob(mca, MCA_PROP_PDOS, pdos, sizeof(pdos)))
		return MCA_POLICY_ICL_DEFAULT_MA;

	for (i = 0; i < MCA_PD_MAX_PDOS; i++) {
		if (le32_to_cpu(pdos[i].min_mv) <= 5000 &&
		    le32_to_cpu(pdos[i].max_mv) >= 5000)
			ma = min(ma, le32_to_cpu(pdos[i].max_ma));
	}

	return ma && ma != U32_MAX ? ma : MCA_POLICY_ICL_DEFAULT_MA;
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
	if (hv && (real_type == MCA_TYPE_PD || real_type == MCA_TYPE_PD_VERIFY ||
		   real_type == MCA_TYPE_PD_PPS))
		return MCA_FCC_MAX_MA;
	/* The community policy does not raise FCC for unclassified Type-C. */
	if (real_type == MCA_TYPE_C)
		return MCA_FCC_DEFAULT_MA;
	return limits->charge_current_ma;
}

/* Charge current for @full at battery temperature @tbat */
static u32 xiaomi_mca_fcc_thermal(u32 full, int tbat)
{
	u32 derate;

	if (tbat < MCA_FCC_TBAT_MIN)
		return MCA_FCC_DEFAULT_MA;
	if (tbat <= MCA_FCC_DERATE_FROM || full <= MCA_FCC_DEFAULT_MA)
		return full;

	derate = (tbat - MCA_FCC_DERATE_FROM) * MCA_FCC_DERATE_STEP_MA;
	return derate < full - MCA_FCC_DEFAULT_MA ? full - derate :
						     MCA_FCC_DEFAULT_MA;
}

static void xiaomi_mca_apply_fcc(struct xiaomi_mca *mca, int tbat)
{
	u32 full = min_t(u32, xiaomi_mca_policy_fcc(mca->real_type, mca->hv == MCA_HV_ON),
			 MCA_FCC_MAX_MA);
	u32 target = xiaomi_mca_fcc_thermal(full, tbat);

	/* Same charger, cooling down: one step at a time, with a degree spare */
	if (mca->fcc_set_ma && full == mca->fcc_full &&
	    target > mca->fcc_set_ma) {
		if (time_before(jiffies, mca->fcc_raise_at))
			return;
		target = min3(target, xiaomi_mca_fcc_thermal(full, tbat + 1),
			      (u32)mca->fcc_set_ma + MCA_FCC_DERATE_STEP_MA);
	}
	if (target == mca->fcc_set_ma)
		return;

	if (xiaomi_mca_write_u32(mca, MCA_PROP_CHARGE_CURRENT, target)) {
		dev_warn(mca->dev, "failed to set charge current %u mA\n", target);
		return;
	}

	dev_info(mca->dev, "%s: charge current %u mA (battery %d degC)\n",
		 xiaomi_mca_type_name(mca->real_type) ?: "Unknown", target, tbat);
	mca->fcc_set_ma = target;
	mca->fcc_full = full;
	mca->fcc_raise_at = jiffies + msecs_to_jiffies(MCA_FCC_RAISE_MS);
}

static bool xiaomi_mca_is_pd(u32 real_type)
{
	return real_type == MCA_TYPE_PD || real_type == MCA_TYPE_PD_VERIFY ||
	       real_type == MCA_TYPE_PD_PPS;
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
	return mipps_auth && mca->auth_ok &&
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
	u32 ma = 0;
	int ret;

	if (!hv_charge || mca->hv == MCA_HV_BLOCKED)
		return 0;

	if (xiaomi_mca_is_pd(mca->real_type))
		ma = xiaomi_mca_pd_9v_ma(mca, &pps);

	/* The source must keep offering 9 V the way it was asked for */
	if (mca->hv != MCA_HV_OFF && pps != mca->hv_pps)
		ma = 0;

	switch (mca->hv) {
	case MCA_HV_OFF:
		/* not in the middle of an adapter authentication */
		if (!ma || time_before(jiffies, mca->hv_retry) ||
		    xiaomi_mca_auth_pending(mca) ||
		    xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS))
			return 0;
		mca->hv_pps = pps;
		mca->hv_pps_ma = ma;
		ret = xiaomi_mca_request_volt(mca, MCA_HV_MV);
		if (ret) {
			dev_warn(mca->dev, "9 V request refused: %d (adsp %d)\n",
				 ret, mca->retcode);
			break;
		}
		dev_info(mca->dev, "requested 9 V (%s %u mA)\n",
			 pps ? "PPS" : "PDO", ma);
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
		break;
	case MCA_HV_ON:
		if (ma && vbus_hv)
			return ma;
		if (ma && mca->vbus_uv < MCA_HV_VBUS_5V_MAX_UV &&
		    mca->hv_renegs++ < MCA_HV_RENEG_MAX) {
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

	/* Not reached, dropped or refused: 5 V until detach */
	dev_info(mca->dev, "back to 5 V (bus %d mV)\n", mca->vbus_uv / 1000);
	xiaomi_mca_request_volt(mca, 5000);
	mca->hv = MCA_HV_BLOCKED;
	return 0;
}

/*
 * Stock buck strategy input current limits (mA) at 9 V: in_pd, and in_pps
 * for the two types the stock table names PD_PPS
 */
static u32 xiaomi_mca_hv_icl(u32 real_type)
{
	if (real_type == MCA_TYPE_PD)
		return xiaomi_mca_pd_limits.input_current_ma;
	if (real_type == MCA_TYPE_PD_VERIFY || real_type == MCA_TYPE_PD_PPS)
		return xiaomi_mca_pps_limits.input_current_ma;
	return 0;
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
	int tbat = (s32)xiaomi_mca_read_u32(mca, MCA_PROP_PACK_TBAT);
	u32 hv_ma;
	int ret;

	if (!target)
		return;

	if (charge_current)
		xiaomi_mca_apply_fcc(mca, tbat);

	hv_ma = xiaomi_mca_hv_step(mca);
	if (hv_ma)
		target = clamp(hv_ma, MCA_ICL_MIN_MA,
			       xiaomi_mca_hv_icl(mca->real_type));
	else if (xiaomi_mca_is_pd(mca->real_type))
		target = clamp(xiaomi_mca_pd_5v_ma(mca), MCA_ICL_MIN_MA, target);

	if (target == mca->icl_set_ma)
		return;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, target);
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
 * Both fuel gauges: the higher cell voltage (mV) and temperature (degC) and
 * the sum of the currents (mA, positive while charging)
 */
static int xiaomi_mca_gauges(int *vbat_mv, int *ibat_ma, int *tcool, int *tbat)
{
	union power_supply_propval v, i, t;
	struct power_supply *psy;
	int n, ret;

	*vbat_mv = 0;
	*ibat_ma = 0;
	*tbat = INT_MIN;
	*tcool = INT_MAX;
	for (n = 0; n < ARRAY_SIZE(xiaomi_mca_supplied_to); n++) {
		psy = power_supply_get_by_name(xiaomi_mca_supplied_to[n]);
		if (!psy)
			return -ENODEV;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &v);
		ret = ret ?: power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &i);
		ret = ret ?: power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &t);
		power_supply_put(psy);
		if (ret)
			return ret;
		*vbat_mv = max(*vbat_mv, v.intval / 1000);
		*ibat_ma += i.intval / 1000;
		*tbat = max(*tbat, t.intval / 10);
		*tcool = min(*tcool, t.intval / 10);
	}

	return 0;
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

static int xiaomi_mca_cp_request(struct xiaomi_mca *mca, u32 mv)
{
	mv = clamp(rounddown(mv, 20), (u32)MCA_CP_PPS_MIN_MV,
		   min_t(u32, mca->cp_max_mv, MCA_CP_PPS_MAX_MV));
	if (mv == mca->cp_mv)
		return 0;

	mca->cp_mv = mv;
	return xiaomi_mca_write_u32(mca, MCA_PROP_PPS_SELECT,
				   xiaomi_mca_pps_word(mv, mca->cp_ma));
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
	if (ret)
		return ret;

	mca->icl_set_ma = icl_ma;
	mca->fcc_set_ma = fcc_ma;
	return 0;
}

static int xiaomi_mca_cp_stop(struct xiaomi_mca *mca, const char *why)
{
	int ret;

	if (!xiaomi_mca_cp_active(mca))
		return 0;

	ret = xiaomi_mca_cp_set_online(mca, false);
	if (ret) {
		mca->cp = MCA_CP_STOPPING;
		dev_err(mca->dev, "failed to stop the switched-capacitor stage: %d\n", ret);
		return ret;
	}
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	dev_info(mca->dev, "direct charging stopped: %s\n", why);

	mca->cp = MCA_CP_DONE;
	mca->hv = MCA_HV_OFF;
	mca->hv_retry = jiffies;
	return 0;
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
	dev_warn(mca->dev, "source dropped out at %u mA, %u mA from now on\n",
		 mca->cp_ibus_last, mca->cp_ibus_cap);
}

/* What the source gives: its APDO current, less once it dropped out */
static u32 xiaomi_mca_cp_src_ma(struct xiaomi_mca *mca)
{
	return mca->cp_ibus_cap ? min(mca->cp_ma, mca->cp_ibus_cap) : mca->cp_ma;
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
		    hi_mv >= 2 * MCA_CP_VBAT_MAX_MV + MCA_CP_DELTA_MV &&
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
	return mipps_auth &&
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
	if (!mipps_auth || mca->hv == MCA_HV_REQUESTED ||
	    mca->vbus_uv >= MCA_HV_VBUS_5V_MAX_UV)
		return false;
	if (mca->auth_ok && time_after(mca->cp_on_at, mca->auth_at))
		return true;
	if (xiaomi_mca_auth_pending(mca) && mca->auth_tried)
		return false;
	return xiaomi_mca_adapter_verified(mca);
}

/*
 * Battery current limit (both cells) for the battery voltage and the
 * coolest and warmest temperature, see MCA_CP_TBAT_* and MCA_CP_VSTEP_*
 */
static u32 xiaomi_mca_cp_ibat_limit(struct xiaomi_mca *mca, int vbat,
				   int tcool, int twarm)
{
	u32 ma = clamp(cp_ibat_max, 1000U, (u32)MCA_CP_IBAT_MAX_MA);

	if (twarm >= MCA_CP_TBAT_WARM) {
		mca->cp_warm_ma = MCA_CP_IBAT_REDUCED_MA;
		mca->cp_warm_at = jiffies;
	} else if (mca->cp_warm_ma < MCA_CP_IBAT_MAX_MA &&
		   twarm <= MCA_CP_TBAT_WARM - MCA_CP_TBAT_HYS &&
		   time_after(jiffies, mca->cp_warm_at +
			      msecs_to_jiffies(mca->cp_warm_ma == MCA_CP_IBAT_REDUCED_MA ?
					       MCA_CP_TBAT_HOLD_MS :
					       MCA_CP_TBAT_RAMP_MS))) {
		mca->cp_warm_ma = min(mca->cp_warm_ma + MCA_CP_TBAT_RAMP_MA,
				      (u32)MCA_CP_IBAT_MAX_MA);
		mca->cp_warm_at = jiffies;
	}
	ma = min(ma, mca->cp_warm_ma);
	if (tcool < MCA_CP_TBAT_FULL)
		ma = min_t(u32, ma, MCA_CP_IBAT_REDUCED_MA);
	if (vbat >= MCA_CP_VSTEP_MV)
		ma = min_t(u32, ma, MCA_CP_VSTEP_MA);

	return ma;
}

/* Called from the poll at 9 V on the buck charger, or right after the auth */
static void xiaomi_mca_cp_try_start(struct xiaomi_mca *mca)
{
	int vbat, ibat, tcool, tbat, tpack, ret;
	u32 start_mv, ma, src_ma, max_mv = 0;

	if (!cp_charge || !hv_charge || !charge_current ||
	    !xiaomi_mca_is_pd(mca->real_type) ||
	    mca->cp != MCA_CP_OFF ||
	    (mca->hv != MCA_HV_ON && !xiaomi_mca_cp_after_auth(mca)))
		return;

	if (xiaomi_mca_gauges(&vbat, &ibat, &tcool, &tbat))
		return;
	tpack = (s32)xiaomi_mca_read_u32(mca, MCA_PROP_PACK_TBAT);
	if (vbat < MCA_CP_VBAT_MIN_MV || vbat >= MCA_CP_VBAT_START_MV ||
	    min(tcool, tpack) < MCA_CP_TBAT_MIN ||
	    max(tbat, tpack) >= MCA_CP_TBAT_WARM - MCA_CP_TBAT_HYS ||
	    xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS))
		return;
	/* PD requests in the middle of the authentication break it */
	if (mipps_auth && !mca->auth_ok &&
	    time_before(jiffies, mca->cp_on_at +
				 msecs_to_jiffies(MCA_CP_AUTH_WAIT_MS)))
		return;

	start_mv = 2 * vbat + MCA_CP_DELTA_MV;
	ma = xiaomi_mca_cp_apdo(mca, start_mv, &max_mv);
	if (ma < MCA_CP_BUCK_MA + 1000)
		return;

	mca->cp_ma = min_t(u32, ma, MCA_CP_PPS_MAX_MA);
	mca->cp_max_mv = max_mv;
	src_ma = xiaomi_mca_cp_src_ma(mca);
	src_ma -= min_t(u32, src_ma, MCA_CP_BUCK_MA + 200);
	mca->cp_ibus_ma = min(clamp(cp_ibus_max, 500U, (u32)MCA_CP_IBUS_MAX_MA),
			      src_ma);
	if (!xiaomi_mca_adapter_verified(mca))
		mca->cp_ibus_ma = min_t(u32, mca->cp_ibus_ma,
					MCA_CP_THIRD_IBUS_MA);
	if (mca->cp_ibus_ma < 2 * MCA_CP_OPEN_IBUS_MA) {
		mca->cp = MCA_CP_DONE;
		return;
	}

	mca->cp_psy = power_supply_get_by_name("sc8541");
	if (!mca->cp_psy)
		return;
	/* it must be off before the source is asked for anything */
	if (xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_ONLINE, &ret) || ret ||
	    xiaomi_mca_cp_set_busocp(mca, min(mca->cp_ibus_ma + MCA_CP_BUSOCP_MARGIN_MA,
					     MCA_CP_BUSOCP_MAX_MA))) {
		dev_warn(mca->dev, "switched-capacitor stage not ready\n");
		power_supply_put(mca->cp_psy);
		mca->cp_psy = NULL;
		mca->cp = MCA_CP_DONE;
		return;
	}

	mca->cp_mv = 0;
	mca->cp_polls = 0;
	mca->cp_low = 0;
	mca->cp_ibus_hot = mca->cp_ibus_ma;
	mca->cp_ibat_trip = 0;
	mca->cp_warm_ma = MCA_CP_IBAT_MAX_MA;
	mca->cp_buck_par = false;
	mca->cp = MCA_CP_OPENING;

	if (xiaomi_mca_auth_pending(mca))
		mca->auth_tried = true;

	/* the buck charger carries little meanwhile */
	ret = xiaomi_mca_cp_set_buck(mca, MCA_CP_BUCK_MA, MCA_CP_BUCK_MA);
	ret = ret ?: xiaomi_mca_cp_request(mca, start_mv);
	if (ret) {
		if (xiaomi_mca_cp_stop(mca, "start refused"))
			return;
		/* again after the adapter reset */
		if (xiaomi_mca_auth_pending(mca))
			mca->cp = MCA_CP_OFF;
		return;
	}

	dev_info(mca->dev, "direct charging: %u mV %u mA (APDO to %u mV %u mA), battery %d mV %d degC, bus target %u mA\n",
		 mca->cp_mv, mca->cp_ma, max_mv, ma, vbat, tbat, mca->cp_ibus_ma);
}

/* One step of direct charging, every MCA_CP_POLL_MS */
static void xiaomi_mca_cp_step(struct xiaomi_mca *mca)
{
	int vbat, ibat, tcool, tbat, tpack, vbus_uv, ibus_ua, tdie, on, ibus, over;
	int ibat_max, target, ibus_all;
	u32 mv = mca->cp_mv;
	bool par;

	if (xiaomi_mca_gauges(&vbat, &ibat, &tcool, &tbat) ||
	    xiaomi_mca_cp_get(mca, POWER_SUPPLY_PROP_ONLINE, &on)) {
		xiaomi_mca_cp_stop(mca, "read failed");
		return;
	}
	tpack = (s32)xiaomi_mca_read_u32(mca, MCA_PROP_PACK_TBAT);

	if (vbat >= MCA_CP_VBAT_MAX_MV) {
		xiaomi_mca_cp_stop(mca, "battery voltage reached");
		return;
	}
	if (min(tcool, tpack) < MCA_CP_TBAT_MIN ||
	    max(tbat, tpack) >= MCA_CP_TBAT_EXIT) {
		xiaomi_mca_cp_stop(mca, "battery temperature");
		return;
	}
	if (xiaomi_mca_read_u32(mca, MCA_PROP_VERIFY_PROCESS)) {
		xiaomi_mca_cp_stop(mca, "adapter authentication");
		/* again once it is over */
		mca->cp = MCA_CP_OFF;
		return;
	}

	ibat_max = xiaomi_mca_cp_ibat_limit(mca, vbat, min(tcool, tpack),
					   max(tbat, tpack));
	/* a lowered limit is regulated down to before it trips */
	if (ibat_max >= (int)mca->cp_ibat_trip) {
		mca->cp_ibat_trip = ibat_max;
		mca->cp_trip_at = jiffies;
	} else if (time_after(jiffies, mca->cp_trip_at +
				       msecs_to_jiffies(MCA_CP_TRIP_GRACE_MS))) {
		mca->cp_ibat_trip = ibat_max;
	}

	if (mca->cp == MCA_CP_OPENING &&
	    ++mca->cp_polls > MCA_CP_OPEN_TRIES) {
		xiaomi_mca_cp_stop(mca, "no current");
		return;
	}
	if (!on && mca->cp == MCA_CP_ON) {
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
	mca->cp_ibus_last = max(ibus + mca->ibus_ua / 1000, 0);
	if (vbus_uv / 1000 > 2 * vbat + MCA_CP_VBUS_MAX_DELTA_MV) {
		xiaomi_mca_cp_stop(mca, "bus voltage too high");
		return;
	}
	if (tdie > MCA_CP_TDIE_MAX) {
		xiaomi_mca_cp_stop(mca, "stage temperature");
		return;
	}
	if (ibus > (int)mca->cp_ibus_ma + MCA_CP_TRIP_IBUS_MA) {
		xiaomi_mca_cp_stop(mca, "bus current too high");
		return;
	}
	if (ibat > (int)mca->cp_ibat_trip + MCA_CP_TRIP_IBAT_MA) {
		xiaomi_mca_cp_stop(mca, "battery current too high");
		return;
	}

	/* the stage warms about 1 degC/s at 4 A: back off before it trips */
	if (tdie >= MCA_CP_TDIE_HOT)
		mca->cp_ibus_hot = max_t(u32, mca->cp_ibus_hot - MCA_CP_TDIE_STEP_MA,
					 min_t(u32, MCA_CP_TDIE_MIN_MA, mca->cp_ibus_ma));
	else if (tdie < MCA_CP_TDIE_HOT - MCA_CP_TDIE_HYS)
		mca->cp_ibus_hot = min(mca->cp_ibus_hot + MCA_CP_TDIE_STEP_MA / 4,
				       mca->cp_ibus_ma);
	target = mca->cp_ibus_hot;

	if (mca->cp == MCA_CP_OPENING) {
		if (ibus < MCA_CP_OPEN_IBUS_MA) {
			mv += MCA_CP_STEP_MV;
		} else {
			dev_info(mca->dev, "direct charging at %u mV: bus %d mV %d mA, battery %d mV %d mA\n",
				 mca->cp_mv, vbus_uv / 1000, ibus,
				 vbat, ibat);
			mca->cp = MCA_CP_ON;
			mca->cp_polls = 0;
		}
	} else {
		if (ibus < MCA_CP_LOW_IBUS_MA) {
			if (++mca->cp_low >= MCA_CP_LOW_POLLS) {
				xiaomi_mca_cp_stop(mca, "bus current too low");
				return;
			}
		} else {
			mca->cp_low = 0;
		}

		/* only from a source that feeds both at that bus current */
		ibus_all = ibus + mca->ibus_ua / 1000;
		par = ibat_max > MCA_CP_PAR_IBAT_MA &&
		      xiaomi_mca_cp_src_ma(mca) >=
				MCA_CP_PAR_IBUS_MA + MCA_CP_PAR_ICL_MA + 200 &&
		      (ibus_all > MCA_CP_PAR_IBUS_MA ||
		       (mca->cp_buck_par &&
			ibus_all > MCA_CP_PAR_IBUS_MA - MCA_CP_PAR_HYS_MA));
		if (par != mca->cp_buck_par) {
			if (xiaomi_mca_cp_set_buck(mca,
						  par ? MCA_CP_PAR_ICL_MA : MCA_CP_BUCK_MA,
						  par ? MCA_CP_PAR_FCC_MA : MCA_CP_BUCK_MA)) {
				xiaomi_mca_cp_stop(mca, "buck charger refused");
				return;
			}
			mca->cp_buck_par = par;
			/* make room on the bus for the buck charger's share */
			if (par)
				mv -= (MCA_CP_PAR_FCC_MA - MCA_CP_BUCK_MA) / 2 /
				      MCA_CP_REG_STEP_MA * MCA_CP_REG_STEP_MV;
			dev_info(mca->dev, "buck charger at %u mA%s\n",
				 mca->fcc_set_ma, par ? " in parallel" : "");
		}
		/* what the source gives, less the buck charger's input */
		target = min(target, (int)xiaomi_mca_cp_src_ma(mca) - 200 -
			     (mca->cp_buck_par ? MCA_CP_PAR_ICL_MA : MCA_CP_BUCK_MA));
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
		else if (ibus + MCA_CP_BAND_MA < target &&
			 ibat + MCA_CP_BAND_MA < ibat_max)
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
		ret = xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &source.real_type);
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
	if (!up || (mca->config_attempted && mca->config_generation == generation))
		return;
	/* Exactly one attempt per service generation, including failed writes. */
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
	mca->config_result = ret;
	dev_info(mca->dev, "OTG config generation %u CP present %u CID %#x result %d\n",
		 generation, present, MCA_PIANO_CID_CFG, ret);
	if (ret)
		dev_warn(mca->dev, "OTG configuration sync failed: %d\n", ret);
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
		if (mca->poll_ready &&
		    (record->id == MCA_NOTIFY_TYPE_CHANGE ||
		     record->id == MCA_NOTIFY_TYPEC_CHANGE ||
		     record->id == MCA_NOTIFY_BOOST ||
		     record->id == MCA_NOTIFY_OTG_CP_CONFIG))
			mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
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
	if (mca->service_up != (state == SERVREG_SERVICE_STATE_UP))
		mca->service_generation++;
	mca->service_up = state == SERVREG_SERVICE_STATE_UP;
	if (mca->poll_ready && mca->service_up)
		mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
}

static void xiaomi_mca_policy_poll(struct xiaomi_mca *mca)
{
	bool online;
	u32 real_type = MCA_TYPE_UNKNOWN;

	/* Do not change source voltage or restart a stage whose stop failed. */
	if (mca->cp == MCA_CP_STOPPING) {
		xiaomi_mca_cp_stop(mca, "pending stop");
		return;
	}

	mca->vbus_uv = xiaomi_mca_read_u32(mca, MCA_PROP_USB_VOLTAGE_NOW);
	online = mca->vbus_uv >= MCA_VBUS_ONLINE_UV;

	/* The rest only means something with a source attached */
	if (online) {
		mca->ibus_ua = xiaomi_mca_read_u32(mca, MCA_PROP_USB_CURRENT_NOW);
		mca->icl_ma = xiaomi_mca_read_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT);
		real_type = xiaomi_mca_read_u32(mca, MCA_PROP_USB_REAL_TYPE);
		if (mca->ibus_ua == -1 || mca->icl_ma == -1) {
			xiaomi_mca_cp_stop(mca, "telemetry read failed");
			return;
		}
	} else {
		mca->ibus_ua = 0;
		mca->icl_ma = 0;
	}


	if (online && !mca->online && mca->cp_ibus_cap &&
	    time_after(jiffies, mca->cp_off_at +
				msecs_to_jiffies(MCA_CP_DROP_FORGET_MS)))
		mca->cp_ibus_cap = 0;
	if (!online && mca->online)
		mca->cp_off_at = jiffies;
	if (online && !mca->online) {
		/* away longer than a reset takes: another attach */
		if (time_after(jiffies, mca->cp_off_at +
				msecs_to_jiffies(MCA_CP_DROP_FORGET_MS)))
			mca->auth_ok = false;
		mca->cp_on_at = jiffies;
		/* a voltage change under way breaks the identity exchange */
		if (mipps_auth)
			mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_AUTH_WAIT_MS);
	}
	if (real_type != mca->real_type && xiaomi_mca_limits(real_type)) {
		if (mca->icl_written)
			xiaomi_mca_reset_icl(mca);
		if (mca->fcc_written)
			xiaomi_mca_reset_fcc(mca);
	}
	mca->online = online;
	mca->real_type = real_type;

	if (!online) {
		xiaomi_mca_cp_learn_drop(mca);
		if (xiaomi_mca_cp_stop(mca, "detached"))
			return;
		mca->cp = MCA_CP_OFF;
		if (mca->icl_set_ma)
			xiaomi_mca_reset_icl(mca);
		if (mca->fcc_written)
			xiaomi_mca_reset_fcc(mca);
		/* a hard reset in answer to the request looks like a detach */
		if (mca->hv == MCA_HV_REQUESTED)
			mca->hv_retry = jiffies + msecs_to_jiffies(MCA_HV_RETRY_MS);
		mca->hv = MCA_HV_OFF;
		mca->hv_renegs = 0;
	} else if (!xiaomi_mca_limits(real_type) ||
		   (xiaomi_mca_cp_active(mca) && !xiaomi_mca_is_pd(real_type))) {
		xiaomi_mca_cp_stop(mca, "unknown source");
	} else if (charge_policy && xiaomi_mca_cp_active(mca)) {
		xiaomi_mca_cp_step(mca);
	} else if (charge_policy) {
		xiaomi_mca_cp_try_start(mca);
		if (!xiaomi_mca_cp_active(mca))
			xiaomi_mca_apply_policy(mca);
	}
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
	struct xiaomi_mca_state now;
	u32 generation, svid;
	bool changed = false;

	scoped_guard(mutex, &mca->lock) {
		scoped_guard(spinlock_irqsave, &mca->rx_lock)
			generation = mca->service_generation;
		xiaomi_mca_sync_config(mca);
		if (mca->policy_generation != generation) {
			if (xiaomi_mca_cp_stop(mca, "ADSP restart"))
				goto out;
			mca->cp = MCA_CP_OFF;
			mca->hv = MCA_HV_OFF;
			mca->hv_renegs = 0;
			mca->icl_written = false;
			mca->icl_set_ma = 0;
			mca->fcc_set_ma = 0;
			mca->fcc_written = false;
			mca->fcc_full = 0;
			mca->auth_ok = false;
			mca->online = false;
			mca->policy_generation = generation;
		}
		xiaomi_mca_process_notifications(mca);
		if (charge_policy && !mca->otg_active) {
			mca->policy_active = true;
			xiaomi_mca_policy_poll(mca);
		} else if (!mca->otg_active && mca->policy_active) {
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
		if (xiaomi_mca_read(mca, MCA_PROP_USB_ONLINE, &now.online) ||
		    xiaomi_mca_read(mca, MCA_PROP_CHGR_STATUS, &now.chgr_status) ||
		    xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &now.real_type)) {
			mca->last_valid = false;
		} else {
			changed = !mca->last_valid || memcmp(&now, &mca->last, sizeof(now));
			mca->last = now;
			mca->last_valid = true;

			if (now.online != mca->icl_source.online ||
			    now.real_type != mca->icl_source.real_type)
				xiaomi_mca_restore_icl(mca);
		}

		if (mipps_auth && !xiaomi_mca_read(mca, MCA_PROP_ADAPTER_SVID, &svid) &&
		    svid != mca->adapter_svid) {
			mca->adapter_svid = svid;
			changed = true;
		}

out:
		/* The uevent must not report what was cached before the change. */
		if (changed)
			xiaomi_mca_invalidate(mca);
	}

	if (changed)
		power_supply_changed(mca->psy);

	schedule_delayed_work(&mca->poll_work,
		xiaomi_mca_cp_active(mca) ? msecs_to_jiffies(MCA_CP_POLL_MS) :
		MCA_POLL_INTERVAL);
}

/* The ADSP's own name of the charger type, finer than usb_type */
static ssize_t charger_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct xiaomi_mca *mca = power_supply_get_drvdata(to_power_supply(dev));
	u32 type;
	const char *name;

	guard(mutex)(&mca->lock);
	if (xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &type))
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

/*
 * MiPPS relay, in the format of the sheng battmgr "xiaomi" nodes that the
 * xiaomi-mipps-auth tool drives:
 *
 *   request_vdm_cmd  write "<cmd>[,<hex>]": 1-3 query the adapter, 4, 5
 *                    and 8 send 16 bytes (seed, challenge, reverse digest),
 *                    6 and 7 send a 4-byte flag; read "<state>,<result>"
 *   adapter_id, adapter_svid, pdo2, real_type, pd_verifed
 *   verify_process   1 while an authentication runs, as stock batterysecret
 *                    sets it; 9 V is not newly asked for meanwhile
 *   data_role        "ufp" or "dfp"; writing asks the adapter for a PD data
 *                    role swap (stock request_vdm_cmd 200), as the ADSP
 *                    reads the adapter identity only as DFP
 */
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

static ssize_t request_vdm_cmd_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	__le32 data[MCA_VDM_LEN / sizeof(__le32)] = {};
	u8 bytes[MCA_VDM_LEN];
	char hex[2 * MCA_VDM_LEN + 1] = {};
	unsigned int cmd;
	size_t len;
	int i, ret;

	guard(mutex)(&mca->lock);

	if (sscanf(buf, "%u,%32s", &cmd, hex) < 1)
		return -EINVAL;

	switch (cmd) {
	case 1 ... 3:
		len = sizeof(u32);
		break;
	case 4:
	case 5:
	case 8:
		/* the adapter takes the 16 bytes as big-endian words */
		if (strlen(hex) != 2 * MCA_VDM_LEN ||
		    hex2bin(bytes, hex, MCA_VDM_LEN))
			return -EINVAL;
		for (i = 0; i < ARRAY_SIZE(data); i++)
			data[i] = cpu_to_le32(get_unaligned_be32(&bytes[4 * i]));
		len = MCA_VDM_LEN;
		break;
	case 6:
	case 7:
		if (strlen(hex) != 2 * sizeof(u32) ||
		    hex2bin(bytes, hex, sizeof(u32)))
			return -EINVAL;
		data[0] = cpu_to_le32(get_unaligned_le32(bytes));
		len = sizeof(u32);
		break;
	default:
		return -EINVAL;
	}

	ret = xiaomi_mca_write_blob(mca, MCA_PROP_VDM_BASE + cmd - 1, data, len);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(request_vdm_cmd);

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

static ssize_t xiaomi_mca_store_flag(struct device *dev, const char *buf,
				    size_t count, u32 prop)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	bool flag;
	int ret;

	guard(mutex)(&mca->lock);

	if (kstrtobool(buf, &flag))
		return -EINVAL;

	ret = xiaomi_mca_write_u32(mca, prop, flag);
	return ret ? ret : count;
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

static ssize_t pd_verifed_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	ssize_t ret = xiaomi_mca_store_flag(dev, buf, count, MCA_PROP_PD_VERIFIED);
	bool flag;

	guard(mutex)(&mca->lock);

	if (ret == count && !kstrtobool(buf, &flag)) {
		mca->auth_at = jiffies;
		mca->auth_ok = flag;
		mca->auth_tried = false;
	}
	return ret;
}
static DEVICE_ATTR_RW(pd_verifed);

static ssize_t verify_process_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return xiaomi_mca_show_u32(dev, buf, MCA_PROP_VERIFY_PROCESS, "%u\n");
}

static ssize_t verify_process_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	ssize_t ret = xiaomi_mca_store_flag(dev, buf, count, MCA_PROP_VERIFY_PROCESS);
	bool flag;

	/* the authentication is over: direct charging need not wait a poll */
	if (ret == count && !kstrtobool(buf, &flag) && !flag)
		mod_delayed_work(system_percpu_wq, &mca->poll_work, 0);
	return ret;
}
static DEVICE_ATTR_RW(verify_process);

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

static ssize_t data_role_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct xiaomi_mca *mca = xiaomi_mca_from_dev(dev);
	u32 role;
	int ret;

	guard(mutex)(&mca->lock);

	if (sysfs_streq(buf, "ufp"))
		role = MCA_DATA_ROLE_UFP;
	else if (sysfs_streq(buf, "dfp"))
		role = MCA_DATA_ROLE_DFP;
	else
		return -EINVAL;

	ret = xiaomi_mca_write_u32(mca, MCA_PROP_DATA_ROLE,
				  role);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(data_role);

static struct attribute *xiaomi_mca_xiaomi_attrs[] = {
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
}

static void xiaomi_mca_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

/* The next driver, or none, must not inherit a limit set for this source. */
static void xiaomi_mca_unbind_restore(void *data)
{
	struct xiaomi_mca *mca = data;

	guard(mutex)(&mca->lock);
	if (xiaomi_mca_cp_stop(mca, "unbind")) {
		/* Unbind cannot fail; the stage driver retains its stop watcher. */
		power_supply_put(mca->cp_psy);
		mca->cp_psy = NULL;
		return;
	}
	xiaomi_mca_restore_icl(mca);
	if (mca->fcc_written)
		xiaomi_mca_reset_fcc(mca);
	if (mca->voltage_written)
		xiaomi_mca_request_volt(mca, 5000);
}

/* Stop host-controlled charging before its suppliers can suspend. */
static int xiaomi_mca_quiesce(struct xiaomi_mca *mca)
{
	u32 vbus;
	int ret, wait_ret;

	ret = xiaomi_mca_cp_stop(mca, "system sleep");
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

	ret = xiaomi_mca_restore_icl(mca);
	if (!ret && mca->fcc_written)
		ret = xiaomi_mca_reset_fcc(mca);
	if (ret)
		return ret;

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

out:
	mca->cp = MCA_CP_OFF;
	mca->hv = MCA_HV_OFF;
	mca->voltage_written = false;
	mca->hv_renegs = 0;
	mca->fcc_full = 0;
	mca->auth_ok = false;
	mca->online = false;
	mca->policy_active = false;
	return 0;
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

	disable_delayed_work_sync(&mca->poll_work);
	scoped_guard(mutex, &mca->lock) {
		ret = xiaomi_mca_quiesce(mca);
		if (!ret) {
			mca->sleeping = true;
			mca->last_valid = false;
			xiaomi_mca_invalidate(mca);
		}
	}
	if (ret) {
		dev_err(dev, "cannot quiesce charging for system sleep: %d\n", ret);
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
	}
	enable_delayed_work(&mca->poll_work);
	schedule_delayed_work(&mca->poll_work, 0);
}

static const struct dev_pm_ops xiaomi_mca_pm_ops = {
	.prepare = xiaomi_mca_prepare,
	.complete = xiaomi_mca_complete,
};

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
	dev_set_drvdata(dev, mca);
	init_completion(&mca->ack);
	spin_lock_init(&mca->rx_lock);
	ret = devm_mutex_init(dev, &mca->lock);
	if (ret)
		return ret;

	/*
	 * Allocated first, so that devres releases it last: the power supply
	 * and debugfs readers send requests through it until they are gone.
	 */
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

	scoped_guard(spinlock_irqsave, &mca->rx_lock)
		mca->poll_ready = true;
	pmic_glink_client_register(mca->client);
	schedule_delayed_work(&mca->poll_work, 0);

	return 0;
}

static const struct auxiliary_device_id xiaomi_mca_id_table[] = {
	{ .name = "pmic_glink.xiaomi-mca", },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, xiaomi_mca_id_table);

static struct auxiliary_driver xiaomi_mca_driver = {
	.name = "xiaomi_mca",
	.probe = xiaomi_mca_probe,
	.id_table = xiaomi_mca_id_table,
	.driver.pm = pm_sleep_ptr(&xiaomi_mca_pm_ops),
};
module_auxiliary_driver(xiaomi_mca_driver);

MODULE_DESCRIPTION("Xiaomi MCA charger firmware interface over PMIC GLINK");
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
#include "xiaomi_mca_test.c"
#endif
