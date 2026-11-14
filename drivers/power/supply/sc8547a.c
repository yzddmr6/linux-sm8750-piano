// SPDX-License-Identifier: GPL-2.0-only
// Ported to Linux from the OnePlus Android kernel driver
//   vendor/oplus/kernel/charger/v2/voocphy/phy/oplus_sc8547a.c
// in
//   github.com/OnePlusOSS/android_kernel_modules_and_devicetree_oneplus_sm8750
//   commit e888579 (OnePlus OPD2415_15.0.1.701(EX01))
// which has license GPL-2.0-only
//   (Copyright (C) 2020-2022 Oplus. All rights reserved.)
//
/*
 * Southchip SC8547A divide-by-2 charge pump + VOOC/SVOOC PHY driver.
 *
 * Standalone mainline port of the OnePlus vendor voocphy backend
 * (vendor/oplus/kernel/charger/v2/voocphy/phy/oplus_sc8547a.c) for the
 * OnePlus Pad 3 (erhai, SM8750). The device carries a master + slave
 * SC8547A pair (AP_DUAL_CP_VOOCPHY); this driver binds one instance per
 * DT node (I2C addr distinguishes master/slave).
 *
 * This layer only owns the SC8547A hardware: charge-pump enable, 2:1/1:1
 * work mode, watchdog, ADC, and the VOOC serial-protocol register
 * primitives (predata / txbuff / rxbuff). The VOOC adapter-identify and
 * current-ramp state machine is a separate consumer of these ops.
 *
 * Copyright (C) 2022-2023 Oplus.
 * Mainline port 2026.
 */

#define pr_fmt(fmt) "sc8547a: " fmt

#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/iio/consumer.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
#include <linux/regmap.h>
#include <linux/firmware.h>
#include "ufcs/sc8547a_ufcs.h"
#include "ufcs/include/internal/ufcs_msg.h"
#endif
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/usb/wcd939x-usbss.h>

#include "sc8547a.h"
#include "erhai_cell_envelope.h"

/*
 * Android parity for slow/noisy third-party (S)VOOC adapters.  The vendor
 * engine (v2/voocphy/oplus_adsp_voocphy.c) classifies SVOOC from BOTH the
 * ADSP/PD verdict (oplus_chglib_is_pd_svooc_adapter / adapter_sid_power over
 * pmic-glink) AND the D+/D- serial ladder, and on a comm error/timeout it
 * DELAYS and RETRIES instead of declining ("abnormal adapter need delay") —
 * which is why a non-OEM "SuperVOOC" brick that this driver's old single
 * SC8547_AUTO_IDENTIFY_MS=4000 window could never pass still fast-charges
 * on Android.  These knobs mirror that behaviour: the identify window is
 * re-armed (identify_retries) only when the charger shows VOOC signs — an
 * ADSP fast-charge-type verdict (adsp_classify) or actual VOOC serial frames
 * (sc->vooc_saw_any) — so a non-VOOC charger still declines on the FIRST
 * window exactly as before (D+/D- restored promptly).
 */
static bool serial_enable = true;
module_param(serial_enable, bool, 0444);
MODULE_PARM_DESC(serial_enable, "Identify SUPERVOOC adapters on dedicated charging ports");

static DEFINE_MUTEX(sc8547_pps_lock);

static unsigned int identify_ms = 4000;
module_param(identify_ms, uint, 0644);
MODULE_PARM_DESC(identify_ms,
		 "SVOOC serial-identify window per attempt (ms, default 4000)");

static unsigned int identify_retries = 3;
module_param(identify_retries, uint, 0644);
MODULE_PARM_DESC(identify_retries,
		 "re-arm the identify ladder this many times on a slow adapter (default 3)");

static bool adsp_classify = true;
module_param(adsp_classify, bool, 0644);
MODULE_PARM_DESC(adsp_classify,
		 "back identify retries by the ADSP VOOC fast-charge type (Android parity, default Y)");

static unsigned int adsp_fastchg_type_prop = 26;	/* USB_VOOC_FAST_CHG_TYPE */
module_param(adsp_fastchg_type_prop, uint, 0644);
MODULE_PARM_DESC(adsp_fastchg_type_prop,
		 "ADSP USB property id for the VOOC fast-charge type (vendor oplus_hal_adsp.h, default 26)");

static unsigned int adsp_fastchg_svooc_min = 2;
module_param(adsp_fastchg_svooc_min, uint, 0644);
MODULE_PARM_DESC(adsp_fastchg_svooc_min,
		 "ADSP fast-charge type >= this counts as VOOC-family (default 2)");

static unsigned int adsp_real_type_prop = 11;	/* USB_REAL_TYPE (erhai_pps parity) */
module_param(adsp_real_type_prop, uint, 0644);
MODULE_PARM_DESC(adsp_real_type_prop,
		 "ADSP USB property id for USB_REAL_TYPE (default 11)");

static unsigned int adsp_vooc_real_type = 9;	/* VOOC family per lab records (PPS=8) */
module_param(adsp_vooc_real_type, uint, 0644);
MODULE_PARM_DESC(adsp_vooc_real_type,
		 "USB_REAL_TYPE value that counts as VOOC evidence (default 9)");

/*
 * Battery gauge used as the CELL-voltage reference by the SVOOC CC/CV policy.
 * Two erhai populations exist and BOTH must fast-charge:
 *   - pack-gauge SKUs carry a TI bq27z561 on AP I2C -> psy "bq27z561-0";
 *   - the QBG population runs the gauge inside the ADSP -> psy
 *     "qcom-battmgr-bat" (registered by erhai_otg_glink.c; on a board whose
 *     bq27z561 ACKs but never reports status the OTG driver detaches the dead
 *     pack gauge, so only the ADSP psy remains).
 * The list is tried in order and the first gauge that ANSWERS is used, so a
 * gauge-absent board no longer declines with "gauge-unavailable" (which would
 * disable SVOOC entirely there).  Same convention as erhai_pps.c
 * (pps_bat_psy()).  gauge_name is an explicit override for bring-up/debug.
 */
static char *gauge_name;
module_param(gauge_name, charp, 0644);
MODULE_PARM_DESC(gauge_name,
		 "battery gauge power-supply name to use first (default: bq27z561-0 then erhai-adsp-bat)");

static const char * const sc8547_gauge_names[] = {
	"bq27z561-0",		/* pack-gauge SKUs: TI bq27z561 (AP I2C) */
	"qcom-battmgr-bat",	/* QBG SKUs: gauge lives in the ADSP */
};

#include "erhai_otg_glink.h"
#include "sc8547a.h"
#include "erhai_cell_envelope.h"

/* ---- register map (charge-pump / VOOC-PHY function, I2C addr 0x6F) ---- */
#define SC8547_REG_00			0x00	/* VBAT_OVP */
#define SC8547_REG_01			0x01	/* BAT_OCP */
#define SC8547_REG_02			0x02	/* VAC_OVP */
#define SC8547_REG_04			0x04	/* VBUS_OVP */
#define SC8547_REG_05			0x05	/* IBUS_OCP / UCP */
#define   SC8547_IBUS_UCP_100MS		0x30
#define   SC8547_IBUS_UCP_5MS		0x10
#define   SC8547_IBUS_UCP_DIS_MASK	0x80	/* bit7: 1 = disable IBUS UCP */
#define SC8547_REG_06			0x06	/* converter status flags */
#define   SC8547_CP_SWITCHING_STAT_MASK	BIT(2)
#define SC8547_REG_07			0x07	/* CHG_EN / REG_RESET */
#define   SC8547_CHG_EN_MASK		BIT(7)
#define   SC8547_REG_RESET_MASK		BIT(6)
#define SC8547_REG_08			0x08	/* SS_TIMEOUT */
#define   SC8547_SS_TIMEOUT_SET_MASK	0xE0
#define   SC8547_SS_TIMEOUT_SET_SHIFT	5
#define   SC8547_SS_TIMEOUT_DISABLE	0
#define   SC8547_SS_TIMEOUT_81920MS	7
#define SC8547_REG_09			0x09	/* CHARGE_MODE / WATCHDOG */
#define   SC8547_CHARGE_MODE_MASK	BIT(7)
#define   SC8547_CHARGE_MODE_2_1	0
#define   SC8547_CHARGE_MODE_1_1	1
#define   SC8547_WATCHDOG_MASK		0x07
#define   SC8547_WATCHDOG_DIS		0
#define SC8547_REG_0D			0x0D
#define SC8547_REG_0F			0x0F	/* interrupt flag */
#define SC8547_REG_10			0x10	/* insert-irq mask */
#define SC8547_REG_11			0x11	/* ADC_CTRL */
#define   SC8547_ADC_EN_MASK		BIT(7)
#define SC8547_REG_13			0x13	/* IBUS ADC (2 bytes) */
#define SC8547_REG_15			0x15	/* VBUS ADC (2 bytes) */
#define   SC8547_VBUS_POL_H_MASK	0x0F
#define SC8547_REG_19			0x19	/* VSYS/VBAT block */
#define SC8547_REG_1B			0x1B	/* VBAT ADC (2 bytes) */
#define SC8547_REG_21			0x21	/* DPDM ctrl */
#define SC8547_REG_22			0x22
#define SC8547_REG_2B			0x2B	/* VOOC_CTRL / handshake */
#define SC8547_REG_2C			0x2C	/* TX buffer (word) */
#define SC8547_REG_2D			0x2D
#define SC8547_REG_2E			0x2E	/* RX buffer / adapter info (word) */
#define SC8547_REG_30			0x30	/* VOOC irq mask */
#define SC8547_REG_31			0x31	/* predata (word) */
#define SC8547_REG_33			0x33	/* loose-detect */
#define SC8547_REG_36			0x36	/* device id */
#define SC8547_REG_3A			0x3A
#define SC8547A_REG_OTG_EN		0x3B
#define SC8547_REG_3C			0x3C	/* slave VOOC_CTRL */

#define SC8547A_DEVICE_ID		0x67

/* ADC LSBs scaled to mV / mA (numerator/denominator kept as vendor) */
#define SC8547_VBUS_ADC_LSB_NUM		375	/* /100  -> 3.75 mV */
#define SC8547_VBUS_ADC_LSB_DEN		100
#define SC8547_IBUS_ADC_LSB_NUM		1875	/* /1000 -> 1.875 mA */
#define SC8547_IBUS_ADC_LSB_DEN		1000
#define SC8547_VBAT_ADC_LSB_NUM		125	/* /100  -> 1.25 mV */
#define SC8547_VBAT_ADC_LSB_DEN		100

/* Default protection register nibbles (VBAT_OVP / IBUS_OCP). */
#define SC8547_DEFAULT_OVP_REG		0x0
#define SC8547_DEFAULT_OCP_REG		0x0

/* ===================== VOOC/SVOOC serial protocol ====================== *
 * AP-side VOOC PHY state machine, ported standalone from the OnePlus vendor
 * engine (oplus_ap_voocphy.c). The SC8547A clocks a UART-like frame on D+/D-;
 * each received frame raises the VOOC IRQ. The AP decodes the adapter's "ask"
 * command, computes a reply and writes it back via txbuff/predata. Ladder:
 *   ASK_FASTCHG_ORNOT -> IDENTIFICATION -> ADAPTER_CHECK_COMMU -> TELL_MODEL ->
 *   ASK_BAT_MODEL (battery-sys curve) -> IS_VBUS_OK / ASK_CURRENT / GET_BATT_VOL.
 *
 * SAFETY: driving SVOOC current (>10 A) requires replying fastchg-allowed and
 * sending the battery-sys curve, which makes the adapter ramp Vbus to ~10 V and
 * start constant current into the pack. That path is GATED behind
 * ->fastchg_enable (default 0 = OBSERVATION mode: the ladder still
 * authenticates and is logged, but fastchg is refused and no curve is sent, so
 * the adapter never ramps and no high current flows). Flip it on (debugfs
 * fastchg_enable) only after the frame decode/encode is validated on-device.
 * The constant-current / CC-CV policy itself is a later stage, not yet wired.
 */

/* VOOC "adapter ask" command codes (payload nibble of a received frame). */
#define VOOC_CMD_NULL			0x00
#define VOOC_CMD_GET_BATT_VOL		0x01
#define VOOC_CMD_IS_VBUS_OK		0x02
#define VOOC_CMD_ASK_CURRENT_LEVEL	0x03
#define VOOC_CMD_ASK_FASTCHG_ORNOT	0x04
#define VOOC_CMD_TELL_MODEL		0x05
#define VOOC_CMD_TELL_USB_BAD		0x09
#define VOOC_CMD_ASK_AP_STATUS		0x0a
#define VOOC_CMD_IDENTIFICATION		0x0C
#define VOOC_CMD_ASK_BAT_MODEL		0x0D
#define VOOC_CMD_RECEIVE_DATA_0E	0x0E
#define VOOC_CMD_RECEIVE_DATA_0F	0x0F
#define VOOC_CMD_ADAPTER_CHECK_COMMU	0xF1
#define VOOC_CMD_TELL_MODEL_PROCESS	0xF2
#define VOOC_CMD_ASK_BATT_SYS_PROCESS	0xF3
#define VOOC_CMD_UNKNOWN		0xFF

/*
 * Pre-staged is_vbus_ok reply for SVOOC (vendor OPLUS_IS_VUBS_OK_PREDATA_SVOOC).
 * Written to the predata register (REG_31) once, after the battery-sys curve
 * completes and before the first is_vbus_ok frame, so the adapter's very first
 * is_vbus_ok clock gets an INSTANT "raise Vbus" answer out of hardware instead of
 * our reactive IRQ reply (which misses the tight frame window on the first frame
 * -> adapter never ramps Vbus off 5 V -> goes silent). This is the single missing
 * step that stalled every CC attempt at the is_vbus phase.
 */
#define SC8547_IS_VBUS_OK_PREDATA_SVOOC	0x2002

/* RX status flags (high byte of the REG_2E word). */
#define VOOC_RXDATA_DONE_FLAG		BIT(1)
#define VOOC_TXDATA_DONE_FLAG		BIT(0)
#define VOOC_TXDATA_WR_FAIL_FLAG	BIT(3)
#define VOOC_FIRST_FRAME		0xA8

/* Frame-head decode (top bits of the RX data byte). */
#define VOOC_HEAD_MASK			0xE0
#define VOOC_HEAD_SHIFT			0x05
#define VOOC_MOVE_HEAD_MASK		0x70
#define VOOC_MOVE_HEAD_SHIFT		0x04
#define VOOC2_HEAD			0x05
#define VOOC3_HEAD			0x03
#define SVOOC_HEAD			0x04
#define VOOC2_INVERT_HEAD		0x05
#define VOOC3_INVERT_HEAD		0x06
#define SVOOC_INVERT_HEAD		0x01
#define VOOC_INVERT_HEAD_MASK		0x07
#define VOOC_MESG_READ_MASK		0x1E
#define VOOC_MESG_MOVE_READ_MASK	0x0F

/* TX buffer bit masks / shifts (two payload bytes voocphy_tx_buff[0..1]). */
#define TX0_DET_BIT3_TO_BIT6_MASK	0x78
#define TX0_DET_BIT3_TO_BIT7_MASK	0xF8
#define TX1_DET_BIT0_TO_BIT1_MASK	0x03
#define TX0_DET_BIT3_MASK		0x08
#define TX0_DET_BIT4_MASK		0x10
#define TX0_DET_BIT5_MASK		0x20
#define TX0_DET_BIT6_MASK		0x40
#define TX0_DET_BIT7_MASK		0x80
#define TX1_DET_BIT0_MASK		0x01
#define TX1_DET_BIT1_MASK		0x02
#define TX0_DET_BIT0_SHIFT		0
#define TX0_DET_BIT1_SHIFT		1
#define TX0_DET_BIT2_SHIFT		2
#define TX0_DET_BIT3_SHIFT		3
#define TX0_DET_BIT4_SHIFT		4
#define TX1_DET_BIT0_SHIFT		0
#define TX1_DET_BIT1_SHIFT		1
#define VOOC_BIT_ACTIVE			0x1
#define AP_ALLOW_FASTCHG		0x08	/* tx0 bit3 = fastchg-allowed */

/* ---- CC-CV / vbus-servo constants (vendor oplus_ap_voocphy.c) ---- *
 * The adapter regulates Vbus in ~50 mV steps under host feedback. For a 2:1
 * SVOOC charge pump the target is Vbus ~= 2*Vbat + [100..200] mV; inside that
 * window the CP may close and deliver current. Battery voltage reported back to
 * the adapter is coded (Vbat_mV - 3404)/10, saturating at 125.
 */
#define VOOCPHY_VBUS_LOW_MV		100
#define VOOCPHY_VBUS_HIGH_MV		200
#define NORMAL_VBUS_MIN_MV		2000
#define NORMAL_VBAT_MIN_MV		2000
#define VBATT_BASE_FOR_ADAPTER		3404
#define VBATT_DIV_FOR_ADAPTER		10
#define VBATT_MAX_FOR_ADAPTER		4654

/* Seven-bit ASK_CURRENT is bus current in 100 mA units. */
#define SC8547_CURR_CODE_MIN		0x01

/*
 * Safety backstop while the CP is delivering (stage-2 only).
 *
 * VOLTAGE REFERENCE: the CP's own Vbat ADC sits on the pack's input rail, so
 * under delivery it reads the CELL voltage PLUS the IR drop of the
 * harness/pack path (~0.3 V at the 7-8 A this pad pulls: measured 2026-09-28,
 * 4.49 V on the CP vs 4.20 V on the gauge, while at rest both read 3920 mV).
 * Judging the CC/CV curve (and the CV end) on that loaded reading aborted every
 * SVOOC session 15-30 s in, at ~50% SoC.  The battery gauge reports the CELL
 * voltage — "bq27z561-0" on the pack-gauge SKUs, "qcom-battmgr-bat" on the QBG
 * population (see sc8547_gauge_names[]) — so it is the reference for the curve
 * steps, the start gate and the CV ceiling; the CP reading is kept only as an
 * over-voltage guard, set well above any legitimate loaded reading (the highest
 * seen is ~4.49 V).
 */
#define SC8547_CP_OVP_CEILING_MV	4800	/* CP-side hard guard: cell CV (4.5 V)
						 * plus the measured ~0.3 V IR drop
						 * at 8 A; a wilder reading than this
						 * is a genuine over-voltage. */
#define SC8547_IBUS_ABORT_MA		8000	/* combined rating, 4 A per leg */
#define SC8547_CP_RESTART_MARGIN_MV	110	/* min vbus-2*vbat to re-enable CP */
#define SC8547_CP_RESTART_MARGIN_CEIL_MV 700	/* refuse re-enable into an over-volted bus (runaway) */
#define SC8547_CELL_MIN_VALID_MV	2500	/* below this the gauge reading is used only as a fallback */

/*
 * Dual-CP load-sharing engage/disengage thresholds (master's own half-bus, mA).
 * The slave CP only latches its CHG_EN when Vbus sits inside its 2:1 error
 * window, which happens once the master is drawing real current (the adapter
 * then holds Vbus ~150-180 mV above 2*Vbat). Closing the slave at cold CP-close
 * (diff ~70 mV) just self-clears, so we engage it under load in the monitor and
 * retry until it latches; drop it again when total bus demand falls low.
 */
#define SC8547_SLAVE_ENGAGE_IBUS_MA	1400	/* master half this high -> close slave */
#define SC8547_SLAVE_DISENGAGE_IBUS_MA	1100	/* total bus below this -> drop slave */

/*
 * Settle grace + drop debounce (2026-10-01, bench-measured on Marcos' pad).
 * The adapter's output is shared the instant the slave closes, so the master's
 * own half falls to the slave's share BEFORE the slave ramps off its own
 * soft-start; the slave's ADC can also still read its idle value (~20 mA) on
 * the first tick after the enable.  A one-tick "total low" test therefore
 * drops a slave that was about to share: measured 22:36 — engage at master
 * 1781 mA, drop one tick later at "total 1044 mA", pack stuck at 58 %; and
 * 22:55 after a re-plug, the same pad held master 1061 + slave 1008 mA = 2.4 A
 * and charged (63->64 %).  So never judge the drop inside slave_settle_ms of
 * the engage, and require slave_drop_ticks consecutive low totals after that.
 */
static unsigned int slave_settle_ms = 4000;
module_param(slave_settle_ms, uint, 0644);
MODULE_PARM_DESC(slave_settle_ms,
		 "dual-CP: never judge the slave drop inside this window after the engage (ms, default 4000, 0 = old behaviour)");

static unsigned int slave_drop_ticks = 3;
module_param(slave_drop_ticks, uint, 0644);
MODULE_PARM_DESC(slave_drop_ticks,
		 "dual-CP: consecutive low total-bus ticks before dropping the slave (default 3, 1 = old behaviour)");

/* Project 24926 selects the domestic 67 W SC8547A curve. */
#define SC8547_AUTO_CODE_CEILING	61	/* 24926 (24924 table): 6100 mA */

#define SC8547_AUTO_IDENTIFY_MS		4000	/* window to see adapter_check_ok */
#define SC8547_AUTO_STEP_DEBOUNCE	4	/* consecutive ticks (~1 s) to advance */
/*
 * A genuine SVOOC adapter periodically resets its VOOC link (drops output and
 * waits for a fresh handshake) after tens of seconds of CC. That is not an
 * unplug: re-arm the PHY and re-servo (matches the vendor engine) rather than
 * ending fast-charge. Bound consecutive re-servos so a genuinely failing adapter
 * still falls out to normal charge; a re-servo that follows a sustained CC period
 * refills the budget, so healthy periodic resets never exhaust it.
 */
#define SC8547_AUTO_RESERVO_MAX		4	/* consecutive quick resets before give-up */
/* Oplus AP VOOC PHY: VOOC_COMMU_EVENT_TIME and VOOCPHY_UCP_SS_IBUS_MIN. */
#define SC8547_COMMU_TIMEOUT_MS		200
#define SC8547_UCP_ARM_MA		600
#define SC8547_LOW_IBUS_MA		300	/* same ADC floor as the serial taper check */
#define SC8547_AUTO_SUSTAINED_MS	5000	/* CC held this long -> reset is keep-alive */
#define SC8547_AUTO_DETACH_DEBOUNCE_MS	2500	/* longer absence window while delivering */

enum sc8547_vooc_vbus_status {
	SC_VBUS_LOW = 0,
	SC_VBUS_NORMAL,
	SC_VBUS_HIGH,
};

/*
 * Circuit-resistance detect bits replied in ASK_FASTCHG_ORNOT. The adapter
 * validates these against the negotiated protocol; the value is selected by the
 * decoded adapter type (vendor R_DET_BIT4/5=1, rest 0). The fixed 0xA8 sync
 * first-frame decodes as VOOC20 for every adapter, so the very first reply MUST
 * use the VOOC20 value — sending the SVOOC value there wedges the adapter into
 * re-sending 0xA8 forever.
 */
#define VOOC2_CIRCUIT_R_L		0x04	/* VOOC20 (VOOC2_CIRCUIT_R_L_DEF) */
#define SVOOC_CIRCUIT_R_L		0x06	/* SVOOC  (NO_VOOC2_CIRCUIT_R_L_DEF) */
#define NO_VOOC3_CIRCUIT_R_L		0x0C	/* VOOC30 (NO_VOOC3_CIRCUIT_R_L_DEF) */
#define VOOC_CIRCUIT_R_H		0x00

#define BATT_SYS_ARRAY			0x6
#define BATT_SYS_VOL_CUR_PAIR		0x0C
#define ADAPTER_CHECK_COMMU_TIMES	0x6
#define ADAPTER_MODEL_TIMES		0x3

enum sc8547_adapter_type {
	SC_ADAPTER_UNKNOWN = 0,
	SC_ADAPTER_VOOC20,
	SC_ADAPTER_VOOC30,
	SC_ADAPTER_SVOOC,
};

/* Auto charge-manager per-plug state (fastchg_auto path only). */
enum sc8547_auto_state {
	SC_AUTO_OFF = 0,	/* auto not engaged this plug */
	SC_AUTO_IDENTIFY,	/* armed observation; waiting for adapter_check_ok */
	SC_AUTO_DELIVER,	/* identified + preconditions OK; CC/CV stepping */
	SC_AUTO_DECLINED,	/* not (S)VOOC or precondition fail; normal charge */
	SC_AUTO_DONE,		/* curve exit reached; handed back to normal charge */
};

/* 24924 AP VOOC PHY: 3 entry checks 10 ms apart, 10 hot samples
 * 800 ms apart in delivery, then monitor a trip every 10 s.
 */
#define SC8547_BTB_HOT_MC	80000
#define SC8547_BTB_RUN_MS	800
#define SC8547_BTB_TRIP_MS	10000

struct sc8547_btb {
	struct iio_channel *battery, *usb;
	struct delayed_work work;
	unsigned int attachment;
	bool online, valid, admitted, tripped;
	unsigned int hot_count;
	int battery_mc, usb_mc;
	int battery_err, usb_err;
	unsigned long sampled_at;
};

struct sc8547a_device {
	struct sc8547_btb btb;
#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
	struct sc8547a_ufcs_phy ufcs_phy;
	struct ufcs_dev *ufcs;
	u64 ufcs_mode;
	unsigned int ufcs_cable_ma;
	unsigned int ufcs_granted_ma;
	unsigned long ufcs_packet_at;
	unsigned long ufcs_exit_at;
	bool ufcs_watchdog_set;
#endif
	bool ufcs_owner;
	struct i2c_client	*client;
	struct device		*dev;
	struct mutex		i2c_lock;	/* serialises multi-op RMW */
	struct dentry		*debug_root;
	int			irq;
	bool allow_sent, buck_ready;
	int pre_allow_buck_ua;
	unsigned long allow_deadline;
	u8 first_request; /* First confirmed ASK_CURRENT of this handshake. */
	u8			ovp_reg;
	u8			ocp_reg;
	bool			is_slave;

	/* --- VOOC protocol-engine state (master only) --- */
	bool			vooc_started;	/* PHY armed, ladder running */
	bool			fastchg_enable;	/* gate 1: reply allow-bit, run full ladder */
	bool			curve_sent;	/* batt-sys curve delivered: allow-bit may now assert */
	bool			fastchg_deliver;/* gate 2: permit CP-enable + real current */
	u8			fastchg_curr_code;/* requested 7-bit CP current cap */
	bool			cp_enabled;	/* CP switching (current flowing) */
	bool			hw_profile_set;	/* svooc_hw_setting applied this plug */
	int			vooc_vbus_status;/* enum sc8547_vooc_vbus_status */
	int			last_vbus;	/* last servo readings (for logging) */
	int			last_vbat;
	struct delayed_work	attach_work;	/* post-attach crossbar/arm poller */
	bool			auto_armed;	/* crossbar+PHY armed for this plug */
	enum sc8547_adapter_type adapter_type;
	u8			vooc_head;	/* invert head written into TX */
	bool			vooc_move_head;
	u8			voocphy_rx_buff;/* low byte of REG_2E word */
	u8			vooc_flag;	/* high byte: RX/TX status */
	u8			voocphy_tx_buff[2];
	u8			adapter_mesg;	/* decoded payload of RX frame */
	u8			fastchg_ask_cmd;/* current ladder command */
	u8			pre_ask_cmd;
	u8			adapter_rand_l;	/* identify challenge */
	u8			adapter_rand_h;
	bool			adapter_rand_start;
	u16			code_id_local;
	u16			code_id_far;
	u8			code_id_temp_l;
	u8			code_id_temp_h;
	u8			adapter_ask_check_count;
	u8			adapter_check_cmmu_count;
	u8			adapter_model_count;
	u8			adapter_model_ver;
	u8			adapter_cap;
	bool			model_received;
	bool			adapter_check_ok;
	u8			ask_batt_sys;

	/* --- auto charge-manager (fastchg_auto) --- */
	bool			fastchg_auto;	/* one-shot master arm (kill-switch) */
	enum sc8547_auto_state	auto_state;
	unsigned long		auto_identify_deadline;
	const struct erhai_charge_curve *auto_curve;
	int curve_temp, curve_sample_temp;
	int curve_temp_bounds[6];
	unsigned int thermal_ma;
	unsigned int curve_ma;	/* selected CC/CV table */
	int			auto_curve_idx;	/* current row (descends only) */
	bool			auto_protection;
	unsigned int responded;
	bool ufcs_handshake_tried;
	bool first_ufcs_timeout;
	bool serial_quiesced;
	bool monitor_pump_faults;
	u8 pump_faults;
	unsigned int adc_over_count;
	unsigned int leg_over_count;
	unsigned long next_leg_check;
	bool slave_check_pending, slave_trouble;
	unsigned int low_ibus_count;
	u64 irq_arrived_ns;	/* hard IRQ timestamp; IRQF_ONESHOT */
	bool irq_pending;
	u64 last_frame_ns;
	unsigned int frame_gap_us, max_frame_gap_us;
	unsigned int recoveries_answered;
	bool awaiting_rehandshake;
	unsigned long watchdog_fed_at;
	bool frame_seen;
	int master_ibus_ma, slave_ibus_ma;
	int master_enabled, slave_enabled;
	int serial_vbus_mv;
	int serial_ibat_ma, serial_reg06;
	unsigned int rehandshakes, serial_exits;
	unsigned int rx_errors, tx_errors;
	u8 model_frames[2], model_flags[2];
	const char *serial_last_exit;
	const char *serial_last_retry;
	unsigned long load_since;
	bool load_seen;
	bool serial_ucp_armed;
	u8 change_polls;
	u8 notified_code;
	int adapter_vbat_mv;
	u8 serial_request;	/* ASK_CURRENT retired without TXDATA_WR_FAIL */
	u8 pending_request;
	bool request_pending;
	bool asked_current;
	bool retry_current;
	unsigned int adjust_failures;
	unsigned int above_request_count;
	unsigned long next_current_sample;
	bool gauge_reading;
	u16 serial_predata, ask_predata;
	unsigned int missed_asks;
	u8 missed_request;
	u16 missed_predata;
	bool measure_missed_slot;
	int missed_peak_ibus_ma;
	unsigned int		auto_taper_cnt;
	int			auto_step_cnt;	/* advance debounce counter */
	int			auto_reservo_cnt;	/* consecutive adapter-reset re-servos */
	int			auto_ident_retries;	/* identify re-arms on this plug */
	bool			vooc_saw_any;	/* any decodable VOOC frame this attempt */
	/* --- dual-CP slave load-sharing state --- */
	unsigned long		slave_on_since;	/* jiffies the slave last engaged (0 = n/a) */
	unsigned int		slave_low_cnt;	/* consecutive low-total ticks since then */
};

/*
 * The slave charge pump is a second SC8547A on a separate I2C bus (i2c_hub_2),
 * probed as its own device with the "oplus,slave" property. The master's VOOC
 * delivery path drives it through this pointer for 2:1 parallel current sharing
 * so neither pump carries the full bus current (which sags Vbus into
 * VBUS_ERRORLO under CC). NULL on a single-CP board — every slave op no-ops.
 */
static struct sc8547a_device *sc8547_slave_dev;
/* master instance (the VOOC ladder / CP owner), for out-of-tree state queries */
static struct sc8547a_device *sc8547_master_dev;

/*
 * Exported for the PPS host engine (erhai_pps): is the serial VOOC ladder
 * currently DELIVERING?  The PPS path must stand down while genuine serial
 * SuperVOOC owns the plug (it drives the CP itself); this is the gate.
 */
bool sc8547a_fastchg_delivering(void)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;

	return sc && sc->fastchg_deliver;
}
EXPORT_SYMBOL_GPL(sc8547a_fastchg_delivering);

/*
 * Is the serial VOOC ladder currently in the identify or deliver phase (i.e. it
 * owns this plug and delivery is still possible/active)?  The PPS host engine
 * must not capture the connector while this is true — including DURING the
 * identify window, not just once CP delivery has latched (the engine's 10 s
 * debounce otherwise races a slow ladder's retries and lifts VBUS away from the
 * 5 V identify window).
 */
bool sc8547a_fastchg_identifying(void)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;

	return sc && (sc->auto_state == SC_AUTO_IDENTIFY ||
		      sc->auto_state == SC_AUTO_DELIVER);
}
EXPORT_SYMBOL_GPL(sc8547a_fastchg_identifying);

/* A failed pump-off transfer can have taken effect. Leave both chips
 * completely untouched for a full REG09 (1 s) watchdog period. Even a
 * register read can service this watchdog (vendor CP_WATCHDOG_RESET).
 */
static bool pump_stopping;
static unsigned long pump_off_at;

static bool sc8547_waiting_for_off(void)
{
	return READ_ONCE(pump_stopping) && time_before(jiffies, READ_ONCE(pump_off_at));
}

/* ------------------------- register accessors ------------------------- */

static int sc8547_read_byte(struct sc8547a_device *sc, u8 reg, u8 *val)
{
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_read_byte_data(sc->client, reg);

	if (ret < 0) {
		dev_err(sc->dev, "read reg 0x%02x failed: %d\n", reg, ret);
		return ret;
	}
	*val = ret;
	return 0;
}

static int sc8547_write_byte(struct sc8547a_device *sc, u8 reg, u8 val)
{
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_write_byte_data(sc->client, reg, val);

	if (ret < 0)
		dev_err(sc->dev, "write 0x%02x to reg 0x%02x failed: %d\n",
			val, reg, ret);
	return ret;
}

static int sc8547_update_bits(struct sc8547a_device *sc, u8 reg, u8 mask, u8 val)
{
	u8 tmp;
	int ret;

	mutex_lock(&sc->i2c_lock);
	ret = sc8547_read_byte(sc, reg, &tmp);
	if (ret)
		goto out;
	tmp = (tmp & ~mask) | (val & mask);
	ret = sc8547_write_byte(sc, reg, tmp);
out:
	mutex_unlock(&sc->i2c_lock);
	return ret;
}

static int sc8547_read_word(struct sc8547a_device *sc, u8 reg)
{
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_read_word_data(sc->client, reg);

	if (ret < 0)
		dev_err(sc->dev, "read word reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

static int sc8547_write_word(struct sc8547a_device *sc, u8 reg, u16 val)
{
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_write_word_data(sc->client, reg, val);

	if (ret < 0)
		dev_err(sc->dev, "write word 0x%04x to reg 0x%02x failed: %d\n",
			val, reg, ret);
	return ret;
}

/* ------------------------- charge-pump control ------------------------ */

static int sc8547_reg_reset(struct sc8547a_device *sc)
{
	sc->monitor_pump_faults = false;
	sc->pump_faults = 0;
	return sc8547_update_bits(sc, SC8547_REG_07, SC8547_REG_RESET_MASK,
				  SC8547_REG_RESET_MASK);
}

static bool sc8547_check_device_id(struct sc8547a_device *sc)
{
	u8 val = 0;

	if (sc8547_read_byte(sc, SC8547_REG_36, &val))
		return false;
	return val == SC8547A_DEVICE_ID;
}

static bool sc8547_btb_allows(const struct sc8547_btb *btb)
{
	return btb->online && btb->admitted && btb->valid && !btb->tripped;
}

static int sc8547_set_chg_enable(struct sc8547a_device *sc, bool enable)
{
	u8 stale;
	int ret;

	if (enable && (!sc8547_master_dev ||
		       !sc8547_btb_allows(&sc8547_master_dev->btb)))
		return -EAGAIN;
	if (enable && !sc->is_slave && !sc->monitor_pump_faults && sc->btb.hot_count) {
		/* A new session must pass the entry checks even on a warm replug. */
		sc->btb.admitted = false;
		sc->btb.hot_count = 0;
		mod_delayed_work(system_freezable_wq, &sc->btb.work, 0);
		return -EAGAIN;
	}
	/* IRQ and charge operations serialize on sc8547_pps_lock. REG0F
	 * can hold reset/idle flags. Drain it immediately before CHG_EN,
	 * never on a watchdog refresh of an already enabled converter.
	 */
	if (enable && !sc->monitor_pump_faults) {
		ret = sc8547_read_byte(sc, SC8547_REG_0F, &stale);
		if (ret)
			return ret;
		sc->pump_faults = 0;
	}
	ret = sc8547_write_byte(sc, SC8547_REG_07,
				(enable ? SC8547_CHG_EN_MASK : 0) | (sc->is_slave ? 1 : 0));
	if (!ret)
		sc->monitor_pump_faults = enable;
	return ret;
}

static int sc8547_set_adc_enable(struct sc8547a_device *sc, bool enable)
{
	return sc8547_write_byte(sc, SC8547_REG_11, enable ? 0x80 : 0x00);
}

/* Returns mV, or negative errno. */
static int sc8547_get_vbus(struct sc8547a_device *sc)
{
	u8 buf[2];
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_read_i2c_block_data(sc->client, SC8547_REG_15, 2, buf);

	if (ret != sizeof(buf))
		return ret < 0 ? ret : -EIO;
	return (((buf[0] & SC8547_VBUS_POL_H_MASK) << 8) | buf[1]) *
	       SC8547_VBUS_ADC_LSB_NUM / SC8547_VBUS_ADC_LSB_DEN;
}

/* Returns mA, or negative errno. Only valid while the CP is switching. */
static int sc8547_get_ibus(struct sc8547a_device *sc)
{
	u8 buf[2];
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_read_i2c_block_data(sc->client, SC8547_REG_13, 2, buf);

	if (ret != sizeof(buf))
		return ret < 0 ? ret : -EIO;
	return ((buf[0] << 8) | buf[1]) *
	       SC8547_IBUS_ADC_LSB_NUM / SC8547_IBUS_ADC_LSB_DEN;
}

/* REG19/1B share the 1.25 mV ADC scale (oplus_sc8547a.c update_data). */
static int sc8547_get_output_mv(struct sc8547a_device *sc, u8 reg)
{
	u8 buf[2];
	int ret;

	if (sc8547_waiting_for_off())
		return -EAGAIN;
	ret = i2c_smbus_read_i2c_block_data(sc->client, reg, 2, buf);

	if (ret != sizeof(buf))
		return ret < 0 ? ret : -EIO;
	return ((buf[0] << 8) | buf[1]) *
	       SC8547_VBAT_ADC_LSB_NUM / SC8547_VBAT_ADC_LSB_DEN;
}

static int sc8547_get_vbat(struct sc8547a_device *sc)
{
	return sc8547_get_output_mv(sc, SC8547_REG_1B);
}

/* --------------------- VOOC serial-protocol primitives ---------------- */

/*
 * Register-level API consumed by the VOOC adapter-identify state machine below.
 */
/* Pre-stage a reply word into the predata register (REG_31); see the is_vbus_ok
 * predata note above. Written raw, matching the vendor. */
static int sc8547_set_predata(struct sc8547a_device *sc, u16 val)
{
	int ret = sc8547_write_word(sc, SC8547_REG_31, val);

	if (!ret)
		sc->serial_predata = val;
	return ret;
}

static int sc8547_set_txbuff(struct sc8547a_device *sc, u16 val)
{
	return sc8547_write_word(sc, SC8547_REG_2C, val);
}

/* Reads the raw adapter reply word (RX buffer). Caller splits flag/payload. */
static int sc8547_get_adapter_info(struct sc8547a_device *sc)
{
	return sc8547_read_word(sc, SC8547_REG_2E);
}

/* Base hardware config, applied at probe and on reset. */
static int sc8547_init_device(struct sc8547a_device *sc)
{
	int ret, err = 0;

	ret = sc8547_write_byte(sc, SC8547_REG_11, 0x00);
	if (ret && !err)
		err = ret;	/* ADC disable */
	ret = sc8547_write_byte(sc, SC8547_REG_02, 0x01);
	if (ret && !err)
		err = ret;	/* VAC_OVP 12V */
	ret = sc8547_write_byte(sc, SC8547_REG_04, 0x50);
	if (ret && !err)
		err = ret;	/* VBUS_OVP 10V */
	ret = sc8547_write_byte(sc, SC8547_REG_00, 0x20 | (sc->ovp_reg & 0x1f));
	if (ret && !err)
		err = ret; /* VBAT_OVP */
	ret = sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	if (ret && !err)
		err = ret; /* IBUS_OCP/UCP */
	ret = sc8547_write_byte(sc, SC8547_REG_01, 0xbf);
	if (ret && !err)
		err = ret;	/* BAT_OCP */
	ret = sc8547_write_byte(sc, SC8547_REG_2B, 0x00);
	if (ret && !err)
		err = ret;	/* VOOC disable */
	ret = sc8547_write_byte(sc, SC8547_REG_3A, 0x60);
	if (ret && !err)
		err = ret;
	ret = sc8547_update_bits(sc, SC8547_REG_09, BIT(4), BIT(4));
	if (ret && !err)
		err = ret; /* IBUS_UCP_RISE mask */
	ret = sc8547_write_byte(sc, SC8547_REG_10, 0x02);
	if (ret && !err)
		err = ret;	/* mask insert irq */
	ret = sc8547_write_byte(sc, SC8547A_REG_OTG_EN, 0x04);
	if (ret && !err)
		err = ret;
	return err;
}

/* Full reset used at probe (reg_reset + base config). */
static int sc8547_hardware_init(struct sc8547a_device *sc)
{
	int ret = sc8547_reg_reset(sc);

	return ret ?: sc8547_init_device(sc);
}

/*
 * Arm the VOOC PHY: reset, base config, DPDM setup, enable the VOOC engine.
 * Mirrors vendor sc8547_init_vooc(). After this the chip runs the D+/D-
 * serial handshake; the protocol engine drives predata/txbuff/rxbuff.
 */
static int sc8547_init_vooc(struct sc8547a_device *sc)
{
	int ret = sc8547_hardware_init(sc);

	return ret ?: sc8547_write_word(sc, SC8547_REG_31, 0) ?:
		sc8547_update_bits(sc, SC8547_REG_3A, BIT(6), BIT(6)) ?:
		sc8547_write_byte(sc, SC8547_REG_21, 0x21) ?:
		sc8547_write_byte(sc, SC8547_REG_22, 0) ?:
		sc8547_write_byte(sc, SC8547_REG_33, 0xd1) ?:
		sc8547_write_byte(sc, SC8547_REG_30, 0x05);
}

/* SVOOC constant-current hardware profile (vendor sc8547_svooc_hw_setting). */
static int sc8547_svooc_hw_setting(struct sc8547a_device *sc)
{
	int ret, err = 0;

	ret = sc8547_write_byte(sc, SC8547_REG_02, 0x01);
	if (ret && !err)
		err = ret;	/* VAC_OVP 12V */
	ret = sc8547_write_byte(sc, SC8547_REG_04, 0x50);
	if (ret && !err)
		err = ret;	/* VBUS_OVP 10V */
	ret = sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	if (ret && !err)
		err = ret;
	/* watchdog (REG_09) deliberately NOT enabled here: this profile is applied
	 * at arm-time, before the VOOC servo, so enabling the 1000ms watchdog now
	 * (with the CP not yet feeding it) would expire mid-servo. The watchdog is
	 * enabled together with the CP at is_vbus-ok NORMAL, then fed by the poller. */
	ret = sc8547_write_byte(sc, SC8547_REG_11, 0x80);
	if (ret && !err)
		err = ret;	/* ADC enable */
	ret = sc8547_write_byte(sc, SC8547_REG_0D, 0x70);
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_33, 0xd1);
	if (ret && !err)
		err = ret;	/* loose_det */
	ret = sc8547_write_byte(sc, SC8547_REG_3A, 0x60);
	if (ret && !err)
		err = ret;
	/*
	 * Disable IBUS under-current protection + soft-start timeout for the SVOOC
	 * pickup (vendor set_sstimeout_ucp_enable(false)). At CP-enable the current
	 * is near zero (Vbus only ~2*Vbat+150mV) and ramps only as the adapter then
	 * raises Vbus; with UCP (5ms) armed the converter trips off immediately,
	 * REG_07 self-clears, the adapter sees no load and aborts comm. OCP/OVP and
	 * the software vbat/ibus backstop still bound the delivery.
	 */
	ret = sc8547_update_bits(sc, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			   SC8547_IBUS_UCP_DIS_MASK);
	if (ret && !err)
		err = ret;		/* UCP disable */
	ret = sc8547_update_bits(sc, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			   SC8547_SS_TIMEOUT_DISABLE);
	if (ret && !err)
		err = ret;		/* SS timeout disable */
	return err;
}

/* Tear down the VOOC PHY on unplug / fastchg exit. */
static int sc8547_reset_voocphy(struct sc8547a_device *sc)
{
	int ret, err = 0;

	ret = sc8547_set_chg_enable(sc, false);
	/* Keep the watchdog armed if the pump-off write did not complete. */
	if (ret)
		return ret;
	ret = sc8547_update_bits(sc, SC8547_REG_09, SC8547_WATCHDOG_MASK,
			   SC8547_WATCHDOG_DIS);
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_00, 0x20 | (sc->ovp_reg & 0x1f));
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_02, 0x01);
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_04, 0x50);
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_11, 0x00);
	if (ret && !err)
		err = ret;	/* ADC off */
	ret = sc8547_write_byte(sc, SC8547_REG_2C, 0x00);
	if (ret && !err)
		err = ret;	/* clear tx */
	ret = sc8547_write_byte(sc, SC8547_REG_2D, 0x00);
	if (ret && !err)
		err = ret;
	ret = sc8547_write_byte(sc, SC8547_REG_30, 0x7f);
	if (ret && !err)
		err = ret;	/* mask vooc irq */
	ret = sc8547_write_byte(sc, SC8547_REG_21, 0xc0);
	if (ret && !err)
		err = ret;	/* D+ HiZ */
	ret = sc8547_write_byte(sc, SC8547_REG_2B, 0x00);
	if (ret && !err)
		err = ret;	/* VOOC disable */
	ret = sc8547_write_word(sc, SC8547_REG_31, 0x0);
	if (ret && !err)
		err = ret;	/* predata */
	ret = sc8547_write_byte(sc, SC8547_REG_10, 0x02);
	if (ret && !err)
		err = ret;	/* mask insert irq */
	ret = sc8547_write_byte(sc, SC8547A_REG_OTG_EN, 0x04);
	if (ret && !err)
		err = ret;
	return err;
}

/* ------------------ PPS host-engine CP API (erhai_pps) ------------------ *
 * The serial VOOC ladder correctly refuses PD/PPS-only adapters, so the PPS
 * engine negotiates a high-V PPS contract through the ADSP itself.  These
 * entry points let that engine also run the SC8547A charge pump in 2:1 from
 * the PPS bus (the genuine-SVOOC delivery path minus the serial ladder): the
 * engine servos the PPS voltage so the chip input sits at ~2*vbat + margin,
 * then engages the CP through the SAME hardware sequence the VOOC frame path
 * uses (svooc profile + REG_09 1000 ms watchdog + CHG_EN) and feeds the
 * watchdog every poll.  The serial ladder keeps first ownership of the CP:
 * whenever sc8547a_fastchg_delivering() is true this path refuses to run.
 * The slave shares high-current loads after its ADC confirms delivery. The
 * engine MUST call sc8547a_pps_cp_feed() more often than the 1000 ms watchdog
 * window or REG_07 self-clears and the CP drops out on its own (safety).
 */
static bool pps_slave_engaged;
static bool pps_cp_engaged;

/* REG0F is read-to-clear. Save OVP/OCP before the shared IRQ is
 * dispatched to either PHY. UCP, adapter insertion and CHG_EN clearing
 * alone do not identify an overcurrent. Oplus oplus_voocphy.h.
 */
static int sc8547_collect_faults(struct sc8547a_device *sc)
{
	u8 flags;
	int ret;

	if (!sc)
		return 0;
	ret = sc8547_read_byte(sc, SC8547_REG_0F, &flags);
	if (!ret && sc->monitor_pump_faults)
		sc->pump_faults |= flags & GENMASK(7, 3);
	return ret;
}

static int sc8547_take_faults(void)
{
	struct sc8547a_device *master = sc8547_master_dev, *slave = sc8547_slave_dev;
	int ret, flags = 0;

	ret = sc8547_collect_faults(master);
	if (master) {
		if (master->monitor_pump_faults)
			flags = master->pump_faults;
		master->pump_faults = 0;
	}
	if (slave) {
		/* A missing slave sample cannot erase a real master fault. */
		sc8547_collect_faults(slave);
		if (slave->monitor_pump_faults)
			flags |= slave->pump_faults;
		slave->pump_faults = 0;
	}
	return flags ?: ret;
}

int sc8547a_pps_faults(void)
{
	guard(mutex)(&sc8547_pps_lock);
	return sc8547_take_faults();
}
EXPORT_SYMBOL_GPL(sc8547a_pps_faults);
static bool pps_adc_hold;	/* keep the ADC on across drops (PPS session) */
static int sc8547_slave_enabled(void);	/* defined in the slave section below */

static int sc8547_pps_disengage(void);
static int sc8547_pps_slave_off(void);

/* Snapshot the chip's own VBUS/VBAT/IBUS ADCs (V is upstream of the CP). */
int sc8547a_pps_snapshot(int *vbus_mv, int *vbat_mv, int *ibus_ma)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;
	int vbus, vbat, ibus;

	if (!sc)
		return -ENODEV;
	vbus = sc8547_get_vbus(sc);
	vbat = sc8547_get_vbat(sc);
	ibus = sc8547_get_ibus(sc);
	/* All three share the same contract: a failed read fails the call
	 * instead of being published as a negative sample. */
	if (vbus < 0 || vbat < 0 || ibus < 0)
		return -EIO;
	if (vbus_mv)
		*vbus_mv = vbus;
	if (vbat_mv)
		*vbat_mv = vbat;
	if (ibus_ma)
		*ibus_ma = ibus;
	return 0;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_snapshot);

/*
 * Raw master REG_06 (converter status) and REG_07 (CHG_EN) for the erhai_pps
 * trace: whether CHG_EN is still latched between feeds.  Read-only.  REG_0F
 * (interrupt flags) is left to the IRQ handler: a flag register may clear on
 * read, which would hide flags from it.
 */
int sc8547a_pps_regs(u8 *r06, u8 *r07)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;
	int ret;

	if (!sc)
		return -ENODEV;
	ret = sc8547_read_byte(sc, SC8547_REG_06, r06);
	if (ret)
		return ret;
	return sc8547_read_byte(sc, SC8547_REG_07, r07);
}
EXPORT_SYMBOL_GPL(sc8547a_pps_regs);

bool sc8547a_pps_cp_engaged(void)
{
	return READ_ONCE(pps_cp_engaged);
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_engaged);

/*
 * Engage the master CP for 2:1 delivery.  The chip input must be inside the
 * 2:1 error window (110 mV <= vbus - 2*vbat <= 700 mV, same constants as the
 * VOOC deliver path).  Applies the SVOOC constant-current profile (idempotent),
 * arms the 1000 ms REG_09 watchdog, then latches CHG_EN.  Returns 0 on
 * success; -EAGAIN (*need_mv = midpoint-target difference) while outside the
 * window so the caller can servo the PPS voltage toward it.
 */
int sc8547a_pps_cp_engage(int *need_mv)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;
	int vbus, vbat, diff, target, ret;

	if (!sc)
		return -ENODEV;
	if (sc->fastchg_deliver || sc8547_slave_enabled())
		return -EBUSY;	/* serial VOOC / slave owns the CP */
	if (pps_cp_engaged)
		return 0;

	vbus = sc8547_get_vbus(sc);
	vbat = sc8547_get_vbat(sc);
	if (vbus < 0 || vbat < 0)
		return -EIO;
	diff = vbus - vbat * 2;
	target = (SC8547_CP_RESTART_MARGIN_MV + SC8547_CP_RESTART_MARGIN_CEIL_MV) / 2;
	if (need_mv)
		*need_mv = target - diff;
	if (diff < SC8547_CP_RESTART_MARGIN_MV ||
	    diff > SC8547_CP_RESTART_MARGIN_CEIL_MV)
		return -EAGAIN;

	/* The UFCS backend already owns the data pins. Apply the shared
	 * pump profile (REG33/3A timing and UCP/soft-start fields) without
	 * touching REG21, the UFCS PHY enable or its packet registers.
	 */
	ret = sc8547_svooc_hw_setting(sc);
	if (!ret)
		ret = sc8547_write_byte(sc, SC8547_REG_09, 0x13);
	if (!ret) {
		pps_cp_engaged = true; /* an enable with a lost ACK may have switched */
		ret = sc8547_set_chg_enable(sc, true);
	}
	if (ret) {
		sc8547_pps_disengage();
		return ret;
	}
	pps_cp_engaged = true;
	dev_info(sc->dev,
		 "PPS CP: master engaged 2:1 (vbus %d mV vbat %d mV diff %d mV)\n",
		 vbus, vbat, diff);
	return 0;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_engage);

/*
 * Feed the 1000 ms watchdog and report state.  Call on every engine poll
 * while engaged.  Re-latches CHG_EN (a watchdog expiry self-clears REG_07)
 * and recomputes the 2:1 window: leaving it disengages the CP (safe fallback
 * to the buck path) and returns -EAGAIN so the engine stops servoing.
 */
int sc8547a_pps_cp_feed(int *vbus_mv, int *vbat_mv)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;
	int vbus, vbat, diff, ret;
	bool window = false;
	u8 control;

	if (!sc || !pps_cp_engaged)
		return -ENODEV;
	ret = sc8547_read_byte(sc, SC8547_REG_07, &control);
	if (ret)
		goto drop;
	vbus = sc8547_get_vbus(sc);
	vbat = sc8547_get_vbat(sc);
	if (vbus < 0 || vbat < 0)
		goto drop;
	if (vbus_mv)
		*vbus_mv = vbus;
	if (vbat_mv)
		*vbat_mv = vbat;
	diff = vbus - vbat * 2;
	if (diff < SC8547_CP_RESTART_MARGIN_MV ||
	    diff > SC8547_CP_RESTART_MARGIN_CEIL_MV) {
		window = true;
		goto drop;
	}
	if (!(control & SC8547_CHG_EN_MASK))
		goto drop;
	ret = sc8547_write_byte(sc, SC8547_REG_09, 0x13);
	if (!ret)
		ret = sc8547_set_chg_enable(sc, true);
	if (!ret)
		return 0;
drop:
	/* Reserve EAGAIN for an observed window excursion. I2C adapters can
	 * also return EAGAIN for arbitration loss. */
	if (sc8547_pps_disengage())
		return -EIO;
	return window ? -EAGAIN : -EIO;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_feed);

static void sc8547_wait_for_watchdog(void)
{
	WRITE_ONCE(pump_off_at, jiffies + HZ + 1);
	WRITE_ONCE(pump_stopping, true);
}

static int sc8547_pps_disengage(void)
{
	struct sc8547a_device *sc = sc8547_master_dev;
	int master = 0, slave;

	if (pump_stopping) {
		if (sc8547_waiting_for_off())
			return -EAGAIN;
		/* No register access since the failed stop. Both enables
		 * were preceded by a successful REG09=1s watchdog write.
		 */
		pump_stopping = false;
		pps_cp_engaged = pps_slave_engaged = false;
		if (sc)
			sc->monitor_pump_faults = false;
		if (sc8547_slave_dev)
			sc8547_slave_dev->monitor_pump_faults = false;
		return 0;
	}
	if (sc && pps_cp_engaged)
		master = sc8547_set_chg_enable(sc, false);
	slave = sc8547_pps_slave_off();
	if (master || slave) {
		sc8547_wait_for_watchdog();
		return -EAGAIN;
	}
	pps_cp_engaged = false;
	return 0;
}

int sc8547a_pps_cp_disengage(void)
{
	guard(mutex)(&sc8547_pps_lock);
	return sc8547_pps_disengage();
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_disengage);

/*
 * Hold the master ADC on for a whole PPS session (vendor oplus_chg_pps keeps
 * the CP ADC on from switch-check to exit).  Off again when released.
 */
int sc8547a_pps_adc_hold(bool on)
{
	guard(mutex)(&sc8547_pps_lock);
	struct sc8547a_device *sc = sc8547_master_dev;
	int ret = 0;

	if (!sc && on)
		return -ENODEV;
	if (sc && !pps_cp_engaged)
		ret = sc8547_set_adc_enable(sc, on);
	pps_adc_hold = on && !ret;
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_adc_hold);

/* ------------------ slave charge-pump (2:1 parallel) ------------------ *
 * The slave SC8547A shares the bus current in parallel with the master so
 * neither pump carries the whole ~3 A: this is how the vendor holds Vbus off
 * VBUS_ERRORLO under sustained CC (single-CP alone sags in ~4 s). The slave runs
 * NO VOOC serial protocol — the master owns D+/D-; the slave only switches. Ops
 * take the master for logging context and act on the global slave device; all
 * no-op when no slave is present (single-CP fallback). Register values mirror the
 * vendor oplus_sc8547_slave.c (sc8547a variant): CP-enable is REG_07 0x81 (CHG_EN
 * + slave-config bit0), CP-off 0x01.
 */
#define SC8547_SLAVE_CP_ON		0x81
#define SC8547_SLAVE_CP_OFF		0x01

static int sc8547_slave_hw_setting(struct sc8547a_device *master)
{
	struct sc8547a_device *sc = sc8547_slave_dev;
	int ret;

	if (!sc)
		return -ENODEV;
	sc->serial_ucp_armed = false;
	ret = sc8547_write_byte(sc, SC8547_REG_02, 0x01);		/* VAC_OVP 12V */
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_04, 0x50);		/* VBUS_OVP 10V */
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_09, 0x03);		/* watchdog 1000ms */
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_3C, 0x40);		/* VOOC_CTRL disable */
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_11, 0x80);		/* ADC enable */
	if (ret)
		return ret;
	ret = sc8547_set_chg_enable(sc, false);
	if (ret)
		return ret;
	ret = sc8547_write_byte(sc, SC8547_REG_0D, 0x70);
	if (ret)
		return ret;
	/*
	 * Same as the master: at parallel CP-close the slave current is near zero
	 * and only ramps as Vbus rises, so an armed IBUS-UCP (5 ms) or soft-start
	 * timeout trips the slave off instantly and it never shares load (reads ~0).
	 * Mask both during pickup. Restore them once this leg carries load.
	 */
	ret = sc8547_update_bits(sc, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			   SC8547_IBUS_UCP_DIS_MASK);
	if (ret)
		return ret;
	ret = sc8547_update_bits(sc, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			   SC8547_SS_TIMEOUT_DISABLE);
	if (ret)
		return ret;
	dev_info(master->dev, "slave CP: parallel hw profile applied\n");
	return 0;
}

static int sc8547_slave_cp_enable(struct sc8547a_device *master, bool on)
{
	struct sc8547a_device *sc = sc8547_slave_dev;
	int ret;

	if (!sc)
		return -ENODEV;
	if (on) {
		ret = sc8547_write_byte(sc, SC8547_REG_09, 0x13);
		ret = ret ?: sc8547_write_byte(sc, SC8547_REG_11, 0x80);
		if (ret)
			return ret;
	}
	return sc8547_set_chg_enable(sc, on);
}

/* Keep both converters alive before a blocking gauge read. During that
 * read, the serial master's regular IRQs continue servicing the pair.
 * Feed a slave with an unknown enable readback as well; REG09 cannot
 * assert CHG_EN. Never feed a pump waiting for its stop watchdog.
 */
static int sc8547_serial_feed(struct sc8547a_device *sc)
{
	int master, slave = 0;

	if (!sc->cp_enabled || sc->auto_state != SC_AUTO_DELIVER || pump_stopping)
		return 0;
	master = sc8547_write_byte(sc, SC8547_REG_09, 0x13);
	if (sc8547_slave_dev && (sc8547_slave_dev->monitor_pump_faults || sc->slave_enabled)) {
		slave = sc8547_write_byte(sc8547_slave_dev, SC8547_REG_09, 0x13);
		slave = slave ?: sc8547_set_adc_enable(sc8547_slave_dev, true);
	}
	if (!master && !slave)
		sc->watchdog_fed_at = jiffies;
	return master ?: slave;
}

/* Slave bus current (mA); 0 when no slave or its ADC/CP is idle. */
static int sc8547_slave_ibus(void)
{
	if (!sc8547_slave_dev)
		return 0;
	return sc8547_get_ibus(sc8547_slave_dev);
}

/* True if the slave CP has its CHG_EN latched (i.e. it is actually converting). */
static int sc8547_slave_enabled(void)
{
	u8 v;
	int ret;

	if (!sc8547_slave_dev)
		return false;
	ret = sc8547_read_byte(sc8547_slave_dev, SC8547_REG_07, &v);
	if (ret)
		return ret;
	return !!(v & SC8547_CHG_EN_MASK);
}

/* Drop the slave CP + ADC on teardown / hand-back. */
static int sc8547_slave_reset(void)
{
	struct sc8547a_device *sc = sc8547_slave_dev;
	int ret;

	if (!sc)
		return 0;
	ret = sc8547_set_chg_enable(sc, false);
	if (ret)
		return ret;
	return sc8547_update_bits(sc, SC8547_REG_09, SC8547_WATCHDOG_MASK,
				  SC8547_WATCHDOG_DIS) ?: sc8547_set_adc_enable(sc, false);
}

/* Serial CHG_EN writes also arm REG09 before either leg can start. */
static int sc8547_serial_off(struct sc8547a_device *sc)
{
	int master, slave;

	if (sc->serial_quiesced)
		return 0;
	if (pump_stopping) {
		if (sc8547_pps_disengage())
			return -EAGAIN;
		sc->serial_quiesced = true;
		return 0;
	}
	master = sc8547_set_chg_enable(sc, false);
	slave = sc8547_slave_reset();
	if (master || slave) {
		sc8547_wait_for_watchdog();
		return -EAGAIN;
	}
	sc->serial_quiesced = true;
	return 0;
}

/* Dual PPS sequence from TheMojoMan/oneplus-erhai-linux 110b27276c22.
 * Keep both pumps under the PPS lock and surface every failed transfer.
 * A latched enable alone cannot establish how the load is shared.
 */
static int sc8547_pps_slave_off(void)
{
	int ret;

	if (!pps_slave_engaged)
		return 0;
	ret = sc8547_slave_cp_enable(sc8547_master_dev, false);
	if (!ret)
		ret = sc8547_set_adc_enable(sc8547_slave_dev, false);
	if (!ret)
		pps_slave_engaged = false;
	return ret;
}

int sc8547a_pps_share(int master_ma, int margin_mv, unsigned int demand_ma,
		      int *slave_ma)
{
	struct sc8547a_device *sc;
	int ret;
	u8 control;

	guard(mutex)(&sc8547_pps_lock);
	sc = sc8547_slave_dev;
	*slave_ma = 0;
	if (!sc || !pps_cp_engaged)
		return 0;
	if (demand_ma < 1600)
		return sc8547_pps_slave_off();
	if (!pps_slave_engaged) {
		/* virtual_cp cp_open_curr_thr_ma / cp_close_curr_thr_ma. */
		if (demand_ma <= 1900 || master_ma <= 1900 || margin_mv < 200)
			return 0;
		ret = sc8547_slave_hw_setting(sc8547_master_dev);
		if (ret)
			return ret;
		pps_slave_engaged = true; /* a failed enable ACK can still have switched */
		return sc8547_slave_cp_enable(sc8547_master_dev, true);
	}

	ret = sc8547_read_byte(sc, SC8547_REG_07, &control);
	if (ret)
		return ret;
	if (!(control & SC8547_CHG_EN_MASK))
		return sc8547_pps_slave_off() ?: -ENOLINK;
	ret = sc8547_write_byte(sc, SC8547_REG_09, 0x13);
	ret = ret ?: sc8547_set_adc_enable(sc, true);
	if (ret)
		return ret;
	ret = sc8547_get_ibus(sc);
	if (ret < 0)
		return ret;
	*slave_ma = ret;
	return 0;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_share);

/* ==================== VOOC protocol state machine ===================== *
 * Ported from the OnePlus vendor engine (oplus_ap_voocphy.c), reduced to the
 * SVOOC adapter-authentication ladder. On each VOOC IRQ the RX word is split
 * into an adapter data byte + status flags; the data byte carries a frame head
 * (adapter type) and a 4-bit "ask" command. We decode it, build a reply into
 * voocphy_tx_buff[0..1] (masked bit writes matching the vendor), bit-reverse
 * the 10-bit payload, and write it to the TX buffer register. The reply word
 * packs tx_buff[0] as the high byte and tx_buff[1] as the low byte, exactly as
 * the vendor does, so the on-wire framing is bit-identical.
 *
 * Implemented here (the hard wire-protocol part): ASK_FASTCHG_ORNOT,
 * IDENTIFICATION, ADAPTER_CHECK_COMMU, TELL_MODEL (+process). The battery-sys
 * curve negotiation and the constant-current / CC-CV policy that actually
 * drives >10 A are stage 2b and deliberately NOT wired — those commands are
 * logged and no current path is enabled.
 */

static inline u8 vooc_shift_from_mask(u8 mask)
{
	return mask ? __ffs(mask) : 0;
}

static void vooc_write_mesg_mask(u8 mask, u8 *pdata, u8 data)
{
	data <<= vooc_shift_from_mask(mask);
	*pdata = (*pdata & ~mask) | (data & mask);
}

static void vooc_read_mesg_mask(u8 mask, u8 data, u8 *out)
{
	*out = (data & mask) >> vooc_shift_from_mask(mask);
}

/*
 * Bit-reverse the 10-bit TX payload (tx_buff[1][1:0]:tx_buff[0]) in place. A
 * "move head" frame additionally shifts the reversed word right by one, exactly
 * like the vendor convert_tx_bit0_to_bit9_for_move_model().
 */
static void vooc_convert_tx_bit0_to_bit9(struct sc8547a_device *sc, bool move)
{
	unsigned int i, src, des = 0;

	src = ((sc->voocphy_tx_buff[1] & TX1_DET_BIT0_TO_BIT1_MASK) << 8) |
	      sc->voocphy_tx_buff[0];
	for (i = 0; i < 10; i++)
		des |= ((src >> i) & 0x1) << (10 - i - 1);
	if (move)
		des >>= 1;
	sc->voocphy_tx_buff[1] = (des >> 8) & TX1_DET_BIT0_TO_BIT1_MASK;
	sc->voocphy_tx_buff[0] = des & 0xff;
}

/* Decode frame head -> adapter type, and the 4-bit ask command. */
static int vooc_get_mesg_from_adapter(struct sc8547a_device *sc)
{
	u8 head = (sc->voocphy_rx_buff & VOOC_HEAD_MASK) >> VOOC_HEAD_SHIFT;
	u8 move = (sc->voocphy_rx_buff & VOOC_MOVE_HEAD_MASK) >> VOOC_MOVE_HEAD_SHIFT;

	if (head == VOOC2_HEAD && move == VOOC3_HEAD)
		move = 0;	/* prefer vooc_head for the ambiguous case */

	sc->vooc_move_head = false;
	if (head == SVOOC_HEAD || move == SVOOC_HEAD) {
		if (head != SVOOC_HEAD)
			sc->vooc_move_head = true;
		sc->vooc_head = SVOOC_INVERT_HEAD;
		sc->adapter_type = SC_ADAPTER_SVOOC;
	} else if (head == VOOC3_HEAD || move == VOOC3_HEAD) {
		if (head != VOOC3_HEAD)
			sc->vooc_move_head = true;
		sc->vooc_head = VOOC3_INVERT_HEAD;
		sc->adapter_type = SC_ADAPTER_VOOC30;
	} else if (head == VOOC2_HEAD || move == VOOC2_HEAD) {
		if (head != VOOC2_HEAD)
			sc->vooc_move_head = true;
		/*
		 * The fixed 0xA8 sync first-frame decodes with a VOOC2 head for
		 * every adapter type. erhai only fast-charges SVOOC bricks (the
		 * ADSP flags pd_svooc before the serial handshake, which is what
		 * the vendor engine keys off to settle the type), so treat the
		 * ambiguous VOOC2 sync as SVOOC and reply with SVOOC framing.
		 * Otherwise the SVOOC adapter rejects our VOOC2-headed reply and
		 * re-sends 0xA8 forever (confirmed against the Android register
		 * trace: its ask-fastchg reply is the SVOOC-framed 0x3002).
		 * Genuine SVOOC data frames (head 0x04) hit the first branch.
		 */
		sc->vooc_head = SVOOC_INVERT_HEAD;
		sc->adapter_type = SC_ADAPTER_SVOOC;
	} else {
		dev_dbg(sc->dev, "unknown frame head, rx=0x%02x\n",
			sc->voocphy_rx_buff);
		return -EIO;
	}

	vooc_read_mesg_mask(sc->vooc_move_head ? VOOC_MESG_MOVE_READ_MASK :
			    VOOC_MESG_READ_MASK, sc->voocphy_rx_buff,
			    &sc->adapter_mesg);
	/* Evidence gate for the identify-retry loop (Android "abnormal adapter
	 * need delay"): any decodable VOOC frame marks a live VOOC adapter even
	 * if the check-commu exchange has not completed yet. */
	sc->vooc_saw_any = true;
	return 0;
}

static void vooc_handle_ask_fastchg_ornot(struct sc8547a_device *sc)
{
	/*
	 * Reply the circuit-resistance detect bits (selected by decoded adapter
	 * type, matching the vendor per-type branches), with bit3 = "AP allows
	 * fastchg". OBSERVATION keeps allow 0, so the adapter keeps polling this
	 * command and never ramps Vbus; the fastchg_enable gate flips it.
	 *
	 * The allow-bit must NOT assert on the FIRST ask_fastchg (pre-identification):
	 * Android sets fastchg_real_allow=true only at ask_bat_model, i.e. after the
	 * identification/check/model/batt-sys-curve exchange. Asserting allow up front
	 * makes the SVOOC adapter go silent (never runs identification). So gate allow
	 * on curve_sent: the adapter re-polls ask_fastchg post-curve, and THAT reply
	 * carries allow=1.
	 */
	u8 allow = (sc->fastchg_enable && sc->curve_sent && sc->buck_ready) ? VOOC_BIT_ACTIVE : 0;
	u8 r_l;

	switch (sc->adapter_type) {
	case SC_ADAPTER_VOOC20:
		r_l = VOOC2_CIRCUIT_R_L;
		break;
	case SC_ADAPTER_VOOC30:
		r_l = NO_VOOC3_CIRCUIT_R_L;
		break;
	default:	/* SC_ADAPTER_SVOOC (and unknown) */
		r_l = SVOOC_CIRCUIT_R_L;
		break;
	}

	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0],
			     r_l | allow);
	vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1],
			     VOOC_CIRCUIT_R_H);
	sc->adapter_rand_start = true;
}

/* Build the identification challenge response from the random pair. */
static void vooc_calc_identification_code(struct sc8547a_device *sc)
{
	u8 l, h;

	sc->code_id_local = (sc->adapter_rand_h << 9) | (sc->adapter_rand_l << 1);

	l = (((sc->adapter_rand_l >> 6) & 0x1) << TX0_DET_BIT0_SHIFT) |
	    (((sc->adapter_rand_l >> 4) & 0x1) << TX0_DET_BIT1_SHIFT) |
	    (((sc->adapter_rand_h >> 6) & 0x1) << TX0_DET_BIT2_SHIFT) |
	    ((sc->adapter_rand_l & 0x1) << TX0_DET_BIT3_SHIFT) |
	    ((sc->adapter_rand_h & 0x1) << TX0_DET_BIT4_SHIFT);
	h = (((sc->adapter_rand_h >> 4) & 0x1) << TX1_DET_BIT0_SHIFT) |
	    (((sc->adapter_rand_l >> 2) & 0x1) << TX1_DET_BIT1_SHIFT);
	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0], l);
	vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1], h);

	/* next round's bytes */
	sc->code_id_temp_l = ((sc->adapter_rand_l >> 3) & 0x1) |
			     (((sc->adapter_rand_h >> 5) & 0x1) << TX0_DET_BIT1_SHIFT) |
			     (((sc->adapter_rand_l >> 5) & 0x1) << TX0_DET_BIT2_SHIFT) |
			     (((sc->adapter_rand_h >> 1) & 0x1) << TX0_DET_BIT3_SHIFT) |
			     (((sc->adapter_rand_h >> 2) & 0x1) << TX0_DET_BIT4_SHIFT);
	sc->code_id_temp_h = ((sc->adapter_rand_l >> 1) & 0x1) |
			     (((sc->adapter_rand_h >> 3) & 0x1) << TX1_DET_BIT1_SHIFT);
}

static void vooc_handle_identification(struct sc8547a_device *sc)
{
	u16 rnd;

	if (sc->adapter_ask_check_count == 0 || !sc->adapter_rand_start)
		return;

	sc->adapter_ask_check_count--;
	sc->adapter_check_cmmu_count = ADAPTER_CHECK_COMMU_TIMES;
	get_random_bytes(&rnd, sizeof(rnd));
	sc->adapter_rand_l = (u8)rnd & 0x7F;
	sc->adapter_rand_h = (u8)(rnd >> 8) & 0x7F;
	sc->fastchg_ask_cmd = VOOC_CMD_ADAPTER_CHECK_COMMU;
	vooc_calc_identification_code(sc);
}

/* Multi-step: adapter echoes our code back, we compare local vs far. */
static void vooc_handle_check_commu(struct sc8547a_device *sc)
{
	switch (sc->adapter_check_cmmu_count) {
	case 5:
		vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK,
				     &sc->voocphy_tx_buff[0], sc->code_id_temp_l);
		vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK,
				     &sc->voocphy_tx_buff[1], sc->code_id_temp_h);
		break;
	case 4:
		sc->code_id_far = sc->adapter_mesg << 12;
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		break;
	case 3:
		sc->code_id_far |= sc->adapter_mesg << 8;
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		break;
	case 2:
		sc->code_id_far |= sc->adapter_mesg << 4;
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		break;
	case 1:
		sc->code_id_far |= sc->adapter_mesg;
		sc->code_id_far &= 0xfefe;
		if (sc->code_id_far == sc->code_id_local) {
			sc->adapter_check_ok = true;
			dev_info(sc->dev, "adapter check ok (code 0x%04x)\n",
				 sc->code_id_local);
		} else {
			dev_warn(sc->dev, "adapter check MISMATCH far=0x%04x local=0x%04x\n",
				 sc->code_id_far, sc->code_id_local);
		}
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		sc->fastchg_ask_cmd = VOOC_CMD_UNKNOWN;
		break;
	default:
		break;
	}
}

static void vooc_handle_tell_model(struct sc8547a_device *sc)
{
	sc->adapter_model_count = ADAPTER_MODEL_TIMES;
	sc->fastchg_ask_cmd = VOOC_CMD_TELL_MODEL_PROCESS;
	vooc_write_mesg_mask(TX0_DET_BIT4_MASK, &sc->voocphy_tx_buff[0],
			     VOOC_BIT_ACTIVE);
	vooc_write_mesg_mask(TX0_DET_BIT6_MASK, &sc->voocphy_tx_buff[0],
			     VOOC_BIT_ACTIVE);
}

static void vooc_handle_tell_model_process(struct sc8547a_device *sc)
{
	if (sc->adapter_model_count == 2) {
		sc->model_frames[0] = sc->voocphy_rx_buff;
		sc->model_flags[0] = sc->vooc_flag;
		sc->adapter_model_ver = sc->adapter_mesg << 4;
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
	} else if (sc->adapter_model_count == 1) {
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		sc->model_frames[1] = sc->voocphy_rx_buff;
		sc->model_flags[1] = sc->vooc_flag;
		sc->adapter_model_ver |= sc->adapter_mesg;
		sc->adapter_model_ver &= 0x7F;
		sc->model_received = true;
		dev_info(sc->dev, "adapter model 0x%02x from frames %02x/%02x flags=%02x/%02x (rx_err=%u tx_err=%u)\n",
			 sc->adapter_model_ver, sc->model_frames[0], sc->model_frames[1],
			 sc->model_flags[0], sc->model_flags[1], sc->rx_errors, sc->tx_errors);
		sc->fastchg_ask_cmd = VOOC_CMD_UNKNOWN;
	}
}

/*
 * erhai battery-sys curve (from OnePlus oplus-chg-erhai-common.dtsi
 * svooc_batt_sys_curve). Six 7-bit rows describing the pack's CC/CV steps
 * (4000 mA -> 4604 mV, 2600 mA -> 4604 mV, 1100 mA -> 4604 mV). We replay the
 * raw bits verbatim; the adapter uses them to run its own constant-current
 * ladder. The rows are sent to the adapter in reverse (row 5 first, row 0
 * last), driven by the ask_batt_sys countdown.
 */
static const u8 svooc_batt_sys_curve[BATT_SYS_ARRAY][7] = {
	{ 0, 1, 0, 1, 0, 0, 0 },	/* 4000 mA */
	{ 1, 1, 1, 1, 0, 0, 0 },	/* 4604 mV */
	{ 0, 0, 1, 1, 0, 1, 0 },	/* 2600 mA */
	{ 1, 1, 1, 1, 0, 0, 0 },	/* 4604 mV */
	{ 0, 0, 0, 1, 0, 1, 1 },	/* 1100 mA */
	{ 1, 1, 1, 1, 0, 0, 0 },	/* 4604 mV */
};

/*
 * ASK_BAT_MODEL: the adapter asks for the battery curve. Prime the batt-sys
 * countdown, reply the vol/cur pair count, and latch the process command so the
 * following frames stream the curve rows.
 */
static void vooc_handle_ask_bat_model(struct sc8547a_device *sc)
{
	sc->ask_batt_sys = BATT_SYS_ARRAY + 1;
	sc->fastchg_ask_cmd = VOOC_CMD_ASK_BATT_SYS_PROCESS;
	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT6_MASK, &sc->voocphy_tx_buff[0],
			     BATT_SYS_VOL_CUR_PAIR);
}

/* ASK_BATT_SYS_PROCESS: stream one curve row per frame (reverse order). */
static void vooc_handle_ask_batt_sys_process(struct sc8547a_device *sc)
{
	const u8 *row;
	u8 l, h;

	if (sc->ask_batt_sys == 0 || sc->ask_batt_sys > BATT_SYS_ARRAY)
		return;

	row = svooc_batt_sys_curve[sc->ask_batt_sys - 1];
	l = row[0] | (row[1] << 1) | (row[2] << 2) | (row[3] << 3) | (row[4] << 4);
	h = row[5] | (row[6] << 1);
	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0], l);
	vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1], h);

	if (sc->ask_batt_sys == 1) {
		bool first_curve = !sc->curve_sent;

		sc->fastchg_ask_cmd = VOOC_CMD_UNKNOWN;
		sc->curve_sent = true;
		/* The next ASK_FASTCHG can grant permission. Start the ADSP
		 * transaction now, outside the IRQ, so neither REG31 nor the
		 * allow reply can authorize a rise with the buck drawing current.
		 */
		if (first_curve && sc->fastchg_deliver)
			sc->allow_deadline = jiffies + msecs_to_jiffies(identify_ms);
	}
}

/*
 * Classify the CP input voltage against the 2:1 target window. The adapter uses
 * our reply to step Vbus toward Vbus ~= 2*Vbat + [100..200] mV; only inside that
 * window (NORMAL) may the CP close.
 */
static void sc8547_vbus_vbatt_detect(struct sc8547a_device *sc)
{
	int vbus = sc8547_get_vbus(sc);
	int vbat = sc8547_get_vbat(sc);
	int diff;

	sc->last_vbus = vbus;
	sc->last_vbat = vbat;

	if (vbus <= 0 || vbat <= 0) {
		sc->vooc_vbus_status = SC_VBUS_HIGH;
		return;
	}

	diff = vbus - vbat * 2;	/* SVOOC divide-by-2 */
	if (diff < VOOCPHY_VBUS_LOW_MV)
		sc->vooc_vbus_status = SC_VBUS_LOW;
	else if (diff > VOOCPHY_VBUS_HIGH_MV)
		sc->vooc_vbus_status = SC_VBUS_HIGH;
	else if (vbus < NORMAL_VBUS_MIN_MV || vbat < NORMAL_VBAT_MIN_MV)
		sc->vooc_vbus_status = SC_VBUS_HIGH;	/* implausible, re-step */
	else
		sc->vooc_vbus_status = SC_VBUS_NORMAL;
}

/*
 * IS_VBUS_OK: report the Vbus window (bit4 = high, bit5 = low, both = ok) so the
 * adapter servos Vbus. Enabling the charge pump (which starts current into the
 * pack) is gated behind ->fastchg_deliver: without it we never latch NORMAL, so
 * the CP stays open and no current ever flows while the ladder is exercised.
 */
static void vooc_handle_is_vbus_ok(struct sc8547a_device *sc)
{
	sc8547_vbus_vbatt_detect(sc);

	dev_dbg(sc->dev,
		"is_vbus_ok: vbus=%d vbat=%d diff=%d status=%d deliver=%d cp=%d\n",
		sc->last_vbus, sc->last_vbat, sc->last_vbus - sc->last_vbat * 2,
		sc->vooc_vbus_status, sc->fastchg_deliver, sc->cp_enabled);

	/* Stage-1 safety: never latch NORMAL unless real delivery is armed. */
	if (!sc->allow_sent || (sc->vooc_vbus_status == SC_VBUS_NORMAL && !sc->fastchg_deliver))
		sc->vooc_vbus_status = SC_VBUS_HIGH;

	switch (sc->vooc_vbus_status) {
	case SC_VBUS_HIGH:
		vooc_write_mesg_mask(TX0_DET_BIT4_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		break;
	case SC_VBUS_NORMAL:
		vooc_write_mesg_mask(TX0_DET_BIT4_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		vooc_write_mesg_mask(TX0_DET_BIT3_MASK, &sc->voocphy_tx_buff[0], 0);
		break;
	default: /* SC_VBUS_LOW */
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		break;
	}

	/*
	 * CP-enable on NORMAL is a SINGLE fast register write — the multi-write CP
	 * hardware profile (svooc_hw_setting) is applied ahead of time by the attach
	 * poller (non-IRQ context) so it never delays this timing-critical reply.
	 * Gate on hw_profile_set: if the poller hasn't applied it yet, hold off and
	 * enable on a later NORMAL frame (adapter keeps servoing meanwhile). The
	 * vendor likewise does only set_chg_enable + monitor here, not hw_setting.
	 */
	if (sc->vooc_vbus_status == SC_VBUS_NORMAL && sc->fastchg_deliver &&
	    sc->hw_profile_set && !sc->cp_enabled) {
		if (sc8547_write_byte(sc, SC8547_REG_09, 0x13) ||
		    sc8547_set_chg_enable(sc, true))
			return;
		sc->cp_enabled = true;
		/*
		 * The slave CP is NOT closed here: at cold CP-close Vbus-2*Vbat is
		 * only ~70 mV, below the slave's 2:1 error window, so its CHG_EN
		 * would just self-clear. The monitor engages the slave under load
		 * (master half > SC8547_SLAVE_ENGAGE_IBUS_MA), where the adapter
		 * holds Vbus high enough for the slave to latch and share.
		 */
		dev_info(sc->dev,
			 "master CP enabled — fastchg delivering (vbus %d mV vbat %d mV, curr code %u)\n",
			 sc->last_vbus, sc->last_vbat, sc->fastchg_curr_code);
	}
}

/*
 * ASK_CURRENT_LEVEL: request a constant-current level (7-bit code spread across
 * tx0[3:7] and tx1[0:1], MSB first). In observation / stage-1 we request 0; the
 * gated path requests the capped code.
 */
static void vooc_handle_ask_current_level(struct sc8547a_device *sc)
{
	u8 cur = sc->fastchg_deliver ? sc->fastchg_curr_code : 0;

	vooc_write_mesg_mask(TX0_DET_BIT3_MASK, &sc->voocphy_tx_buff[0], (cur >> 6) & 1);
	vooc_write_mesg_mask(TX0_DET_BIT4_MASK, &sc->voocphy_tx_buff[0], (cur >> 5) & 1);
	vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0], (cur >> 4) & 1);
	vooc_write_mesg_mask(TX0_DET_BIT6_MASK, &sc->voocphy_tx_buff[0], (cur >> 3) & 1);
	vooc_write_mesg_mask(TX0_DET_BIT7_MASK, &sc->voocphy_tx_buff[0], (cur >> 2) & 1);
	vooc_write_mesg_mask(TX1_DET_BIT0_MASK, &sc->voocphy_tx_buff[1], (cur >> 1) & 1);
	vooc_write_mesg_mask(TX1_DET_BIT1_MASK, &sc->voocphy_tx_buff[1], (cur >> 0) & 1);
}

/* The SC8547A reports a missed reply on a subsequent IRQ. A successful
 * SMBus TXBUF write alone cannot establish what the adapter clocked out.
 */
static void sc8547_request_missed(struct sc8547a_device *sc, u8 code)
{
	sc->missed_asks++;
	sc->missed_request = code;
	sc->missed_predata = sc->ask_predata;
	sc->measure_missed_slot = true;
	sc->missed_peak_ibus_ma = -1;
	/* A missed replacement did not supersede the last confirmed ASK. */
	sc->request_pending = false;
	sc->retry_current = true;
	sc->change_polls = 0;
	sc->adjust_failures++;
	dev_info_ratelimited(sc->dev, "SVOOC missed ASK_CURRENT code=%u predata=0x%04x flags=0x%02x total=%u\n",
			     code, sc->missed_predata, sc->vooc_flag, sc->missed_asks);
}

/* Oplus AP VOOC PHY, non_vooc20_handle_get_batt_vol_cmd: during CC
 * the master polls battery voltage. A seven-bit all-ones reply asks it
 * to poll ASK_CURRENT_LEVEL again. Give the vendor's five indications
 * per changed target, including changes as the second pump picks up.
 */
#define SC8547_CHANGE_CUR_POLLS	5

static void vooc_handle_get_batt_vol(struct sc8547a_device *sc)
{
	int vbat;
	u8 v, l, h;

	if (sc->cp_enabled && sc->fastchg_deliver &&
	    (sc->retry_current || sc->fastchg_curr_code != sc->serial_request)) {
		if (sc->notified_code != sc->fastchg_curr_code) {
			sc->notified_code = sc->fastchg_curr_code;
			sc->change_polls = 0;
		}
		if (sc->change_polls <= SC8547_CHANGE_CUR_POLLS)
			sc->change_polls++;
		if (sc->change_polls <= SC8547_CHANGE_CUR_POLLS && sc->adjust_failures <= 3) {
			vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0], 0x1f);
			vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1], 3);
			return;
		}
	}
	vbat = sc->cp_enabled && sc->adapter_vbat_mv > 0 ? sc->adapter_vbat_mv : sc8547_get_vbat(sc);
	if (vbat < 0)
		vbat = VBATT_BASE_FOR_ADAPTER;

	if (vbat < VBATT_BASE_FOR_ADAPTER)
		v = 0;
	else if (vbat > VBATT_MAX_FOR_ADAPTER)
		v = (VBATT_MAX_FOR_ADAPTER - VBATT_BASE_FOR_ADAPTER) / VBATT_DIV_FOR_ADAPTER;
	else
		v = (vbat - VBATT_BASE_FOR_ADAPTER) / VBATT_DIV_FOR_ADAPTER;

	l = ((v >> 6) & 1) | (((v >> 5) & 1) << 1) | (((v >> 4) & 1) << 2) |
	    (((v >> 3) & 1) << 3) | (((v >> 2) & 1) << 4);
	h = ((v >> 1) & 1) | (((v >> 0) & 1) << 1);
	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0], l);
	vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1], h);
}

/* Return whether RX was a supported command. TX completion has its own status. */
static bool vooc_reply_adapter_mesg(struct sc8547a_device *sc)
{
	bool valid = true;
	u16 reply;
	u8 cmd;

	/* clear the payload bits, then stamp the invert head */
	vooc_write_mesg_mask(TX0_DET_BIT3_TO_BIT7_MASK, &sc->voocphy_tx_buff[0], 0);
	vooc_write_mesg_mask(TX1_DET_BIT0_TO_BIT1_MASK, &sc->voocphy_tx_buff[1], 0);
	vooc_write_mesg_mask(VOOC_INVERT_HEAD_MASK, &sc->voocphy_tx_buff[0],
			     sc->vooc_head);

	/* multi-step commands keep their own ask_cmd; others take the new mesg */
	if (sc->fastchg_ask_cmd != VOOC_CMD_ADAPTER_CHECK_COMMU &&
	    sc->fastchg_ask_cmd != VOOC_CMD_ASK_BATT_SYS_PROCESS &&
	    sc->fastchg_ask_cmd != VOOC_CMD_TELL_MODEL_PROCESS)
		sc->fastchg_ask_cmd = sc->adapter_mesg;

	cmd = sc->fastchg_ask_cmd;
	switch (cmd) {
	case VOOC_CMD_ASK_FASTCHG_ORNOT:
		vooc_handle_ask_fastchg_ornot(sc);
		break;
	case VOOC_CMD_IDENTIFICATION:
		vooc_handle_identification(sc);
		break;
	case VOOC_CMD_ADAPTER_CHECK_COMMU:
		vooc_handle_check_commu(sc);
		break;
	case VOOC_CMD_TELL_MODEL:
		vooc_handle_tell_model(sc);
		break;
	case VOOC_CMD_TELL_MODEL_PROCESS:
		vooc_handle_tell_model_process(sc);
		break;
	case VOOC_CMD_ASK_BAT_MODEL:
		vooc_handle_ask_bat_model(sc);
		break;
	case VOOC_CMD_ASK_BATT_SYS_PROCESS:
		vooc_handle_ask_batt_sys_process(sc);
		break;
	case VOOC_CMD_IS_VBUS_OK:
		vooc_handle_is_vbus_ok(sc);
		break;
	case VOOC_CMD_ASK_CURRENT_LEVEL:
		vooc_handle_ask_current_level(sc);
		break;
	case VOOC_CMD_GET_BATT_VOL:
		vooc_handle_get_batt_vol(sc);
		break;
	case VOOC_CMD_TELL_USB_BAD:
	case VOOC_CMD_ASK_AP_STATUS:
		/* The vendor replies with the empty payload for both. */
		break;
	default:
		valid = false;
		break;
	}

	/* decrement the active multi-step counter, matching vendor ordering */
	if (sc->adapter_check_cmmu_count)
		sc->adapter_check_cmmu_count--;
	else if (sc->adapter_model_count)
		sc->adapter_model_count--;
	else if (sc->ask_batt_sys)
		sc->ask_batt_sys--;

	vooc_convert_tx_bit0_to_bit9(sc, sc->vooc_move_head);
	sc->vooc_move_head = false;
	/* Keep REG31 ready for the next voltage poll even if the threaded
	 * handler misses its slot (oplus_voocphy_set_txbuff, GET_BATT_VOL).
	 */
	reply = (sc->voocphy_tx_buff[0] << 8) | sc->voocphy_tx_buff[1];
	if (cmd == VOOC_CMD_ASK_CURRENT_LEVEL) {
		sc->asked_current = sc->fastchg_deliver;
		sc->ask_predata = sc->serial_predata;
	}
	if (sc8547_set_txbuff(sc, reply) ||
	    (cmd == VOOC_CMD_GET_BATT_VOL && sc8547_set_predata(sc, reply))) {
		sc->tx_errors++;
		if (cmd == VOOC_CMD_ASK_CURRENT_LEVEL)
			sc8547_request_missed(sc, sc->fastchg_curr_code);
		return valid;
	}
	/* AP VOOC PHY sets IS_VBUS_OK predata after sending allow. */
	if (cmd == VOOC_CMD_ASK_BATT_SYS_PROCESS && sc->curve_sent && !sc->buck_ready)
		erhai_charge_try_recheck();
	if (cmd == VOOC_CMD_ASK_FASTCHG_ORNOT && sc->fastchg_enable &&
	    sc->curve_sent && sc->buck_ready) {
		sc->allow_sent = true;
		sc8547_set_predata(sc, SC8547_IS_VBUS_OK_PREDATA_SVOOC);
	}
	if (cmd == VOOC_CMD_ASK_CURRENT_LEVEL) {
		sc->pending_request = sc->fastchg_deliver ? sc->fastchg_curr_code : 0;
		sc->request_pending = true;
	}
	return valid;
}

/*
 * Read pack SOC (%), temperature (0.1 C) and cell voltage (mV) from ONE named
 * gauge power-supply.  Returns -ENODEV when that psy is not registered yet.
 */
static int sc8547_read_gauge_name(const char *name, int *soc, int *temp_ddc,
				  int *vbat_mv, int *ibat_ma)
{
	struct power_supply *psy = power_supply_get_by_name(name);
	union power_supply_propval v;
	int ret;

	if (!psy)
		return -ENODEV;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &v);
	if (ret)
		goto out;
	*soc = v.intval;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &v);
	if (ret)
		goto out;
	*temp_ddc = v.intval;

	/*
	 * Cell (pack) voltage in mV. This is the DELIVERY REFERENCE: unlike the
	 * CP's own Vbat ADC it does not carry the ~0.3 V IR drop of the
	 * harness/pack path at 7-8 A (see the backstop comment below).
	 */
	if (vbat_mv) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
						&v);
		if (ret)
			goto out;
		*vbat_mv = v.intval / 1000;	/* uV -> mV */
	}
	if (ibat_ma) {
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &v);
		if (!ret)
			*ibat_ma = v.intval / 1000;
	}
out:
	power_supply_put(psy);
	return ret;
}

/*
 * Read pack SOC (%) and temperature (0.1 C) from the battery gauge. The auto
 * policy needs the gauge's own SOC/temp (not the CP's Vbat ADC) to pick the
 * curve table, and its cell voltage (mV) as the CC/CV reference.
 *
 * WHICH gauge: every candidate in sc8547_gauge_names is tried in order and the
 * first one that answers is used (gauge_name, when set, is tried first) —
 * pack-gauge SKUs answer on "bq27z561-0", the QBG population on
 * "qcom-battmgr-bat".  Returns the last error (e.g. -ENODEV when no gauge is up
 * yet), which the caller reports as "gauge-unavailable".
 */
static int sc8547_read_gauge(struct sc8547a_device *sc, int *soc, int *temp_ddc,
			     int *vbat_mv, int *ibat_ma)
{
	int last = -ENODEV;
	size_t i;

	if (gauge_name) {
		last = sc8547_read_gauge_name(gauge_name, soc, temp_ddc, vbat_mv, ibat_ma);
		if (!last)
			return 0;
	}

	for (i = 0; i < ARRAY_SIZE(sc8547_gauge_names); i++) {
		last = sc8547_read_gauge_name(sc8547_gauge_names[i], soc, temp_ddc,
					      vbat_mv, ibat_ma);
		if (!last) {
			/*
			 * Evidence line for a board whose primary pack gauge is
			 * absent or dead: name the fallback actually in use.
			 */
			if (gauge_name || i)
				dev_info_once(sc->dev,
					      "gauge: %s in use for the SVOOC CC/CV reference\n",
					      gauge_name ? gauge_name
							 : sc8547_gauge_names[i]);
			return 0;
		}
	}

	return last;
}

/* SUPERVOOC model IDs and rated watts from OnePlusOSS sm8750,
 * cc463675855a, oplus_chg_vooc.c adapter_id_table (GPL-2.0-only).
 * Use the 11 V SUPERVOOC rating
 * (100 mA per step). The serial master checks the circuit resistance
 * and regulates its output; the ADSP 5 V DCP ICL is unrelated.
 */
static u8 sc8547_adapter_cap(u8 model)
{
	static const struct { u8 first, last, watts; } ratings[] = {
		{ 0x11, 0x12, 50 },
		{ 0x14, 0x14, 65 },
		{ 0x1a, 0x1b, 33 },
		{ 0x1c, 0x1c, 45 },
		{ 0x1d, 0x1e, 44 },
		{ 0x21, 0x21, 50 },
		{ 0x22, 0x22, 44 },
		{ 0x23, 0x23, 50 },
		{ 0x24, 0x27, 55 },
		{ 0x28, 0x28, 65 },
		{ 0x2a, 0x2a, 65 },
		{ 0x2b, 0x2b, 66 },
		{ 0x2c, 0x2e, 67 },
		{ 0x31, 0x31, 50 },
		{ 0x32, 0x32, 120 },
		{ 0x33, 0x33, 50 },
		{ 0x35, 0x35, 65 },
		{ 0x36, 0x36, 66 },
		{ 0x37, 0x3a, 88 },
		{ 0x3b, 0x3e, 100 },
		{ 0x47, 0x48, 120 },
		{ 0x49, 0x4a, 33 },
		{ 0x4b, 0x4e, 80 },
		{ 0x51, 0x51, 125 },
		{ 0x61, 0x61, 33 },
		{ 0x62, 0x62, 50 },
		{ 0x63, 0x63, 65 },
		{ 0x64, 0x64, 66 },
		{ 0x65, 0x65, 80 },
		{ 0x66, 0x66, 65 },
		{ 0x67, 0x68, 125 },
		{ 0x69, 0x6a, 100 },
		{ 0x6b, 0x6b, 120 },
		{ 0x6c, 0x6d, 67 },
		{ 0x6e, 0x6e, 65 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ratings); i++)
		if (model >= ratings[i].first && model <= ratings[i].last) {
			/* 24924 explicitly specifies 11 V at 6.1 A for 67 W. */
			if (ratings[i].watts >= 67)
				return SC8547_AUTO_CODE_CEILING;
			return ratings[i].watts * 10 / 11;
		}
	return 0;
}

/* oplus_chg_vooc query_temp_range / rang_rise/drop_update. Apply
 * hysteresis to the reverse transition only, retaining it across a
 * temperature re-arm. VOOC_TEMP_RANGE_THD is 10, THD_HIGH is 20;
 * these are not the common buck's BATT_TEMP_HYST.
 */
#define SC8547_TEMP_RANGE_HYST_DDC 10
#define SC8547_TEMP_RANGE_HIGH_DDC 20
static int sc8547_curve_temperature(struct sc8547a_device *sc, int temp,
				    int soc, bool entering)
{
	static const int bounds[] = { 50, 120, 160, 210, 350, 440 };
	unsigned int region, previous;
	int low, high, margin;

	if (!sc->curve_temp_bounds[0])
		memcpy(sc->curve_temp_bounds, bounds, sizeof(bounds));
	for (region = 0; region < ARRAY_SIZE(bounds); region++)
		if ((entering ? temp : sc->curve_temp) <
		    (entering ? sc->curve_temp_bounds[region] : bounds[region]))
			break;
	previous = region;
	low = region ? sc->curve_temp_bounds[region - 1] : 0;
	high = region < ARRAY_SIZE(bounds) ? sc->curve_temp_bounds[region] : 510;
	if (!entering && temp > high && region < ARRAY_SIZE(bounds)) {
		/* NORMAL_LOW and high-SOC LITTLE_COOL transitions keep the
		 * other dynamic boundaries; the remaining rise paths reset.
		 */
		if (region < 2 || region == 5 || (region < 4 && soc <= 90))
			memcpy(sc->curve_temp_bounds, bounds, sizeof(bounds));
		margin = region == 2 && soc <= 90 ? SC8547_TEMP_RANGE_HIGH_DDC :
			 SC8547_TEMP_RANGE_HYST_DDC;
		sc->curve_temp_bounds[region] = bounds[region] - margin;
		region++;
	} else if (!entering && temp < low && region) {
		memcpy(sc->curve_temp_bounds, bounds, sizeof(bounds));
		region--;
		sc->curve_temp_bounds[region] = bounds[region] + SC8547_TEMP_RANGE_HYST_DDC;
	}
	if (!entering && region == previous)
		return sc->curve_temp;
	/* The tables use fixed region endpoints; the AP sends the region
	 * index selected with the dynamic thresholds to the stock PHY.
	 */
	low = region ? bounds[region - 1] : 0;
	high = region < ARRAY_SIZE(bounds) ? bounds[region] - 1 : 509;
	return min(max(temp, low), high);
}

/* oplus_chg_vooc rang_rise/drop_update and fastchg_range_switch.
 * The low-temperature transitions below are renegotiated up to 90%
 * SOC; crossing into/out of warm is renegotiated at any SOC. This
 * chooses between in-session derating and a new handshake, never
 * between admitting and refusing a high-SOC source.
 */
static bool sc8547_new_temperature_session(int old, int next, int soc)
{
	if (next == old || next < 0 || next >= 510)
		return false;
	if ((old < 440) != (next < 440))
		return true;
	if (soc > 90)
		return false;
	if (next < old)
		return old >= 120 && next < 120;
	return (old < 120 && next >= 120) || (old < 160 && next >= 160) ||
	       (old < 210 && next >= 210);
}

/* Re-index on a change of bin. Within a bin use the AP VOOC PHY's
 * four consecutive voltage/current samples to descend the curve.
 */
static bool sc8547_auto_update_curve(struct sc8547a_device *sc, int soc, int temp,
				     int mv, int ibus, bool entering)
{
	const struct erhai_charge_curve *curve;
	const struct erhai_charge_step *step;
	const struct erhai_charge_curve *raw;
	unsigned int limit;
	int last_temp = sc->curve_temp;
	bool new_table;

	raw = erhai_curve_select(svooc_curves, soc, temp);
	sc->curve_sample_temp = temp;
	sc->curve_temp = sc8547_curve_temperature(sc, temp, soc, entering || !sc->auto_curve);
	curve = erhai_curve_select(svooc_curves, soc, sc->curve_temp);
	/* vooc_high_temp (entry) and over_high_temp (running). */
	if (!raw || !curve || mv <= SC8547_CELL_MIN_VALID_MV ||
	    mv >= min(erhai_cell_cv(temp), erhai_cell_cv(sc->curve_temp)))
		return false;

	new_table = entering || !sc->auto_curve || sc->auto_curve->steps != curve->steps ||
		    last_temp != sc->curve_temp;
	if (new_table) {
		sc->auto_curve = curve;
		sc->auto_curve_idx = erhai_curve_start(curve, mv, true);
		sc->auto_step_cnt = 0;
		sc->auto_taper_cnt = 0;
	} else {
		step = &curve->steps[sc->auto_curve_idx];
		if (!step->last && mv > step->mv && ibus < step->ma + 1000) {
			if (++sc->auto_step_cnt >= SC8547_AUTO_STEP_DEBOUNCE) {
				sc->auto_curve_idx++;
				sc->auto_step_cnt = 0;
			}
		} else {
			sc->auto_step_cnt = 0;
		}
	}
	step = &curve->steps[sc->auto_curve_idx];
	/* Let even a one-row high-SOC curve deliver. Only a measured
	 * low-current tail on a running pump hands the cell to the buck.
	 */
	if (!entering && sc->cp_enabled && step->last && ibus < 300) {
		if (++sc->auto_taper_cnt >= SC8547_AUTO_STEP_DEBOUNCE)
			return false;
	} else {
		sc->auto_taper_cnt = 0;
	}
	/* AP PHY current_max descends with the curve. current_batt_temp is
	 * a separate, reversible vote (oplus_ap_voocphy.c:5316-5324).
	 * Never burn a transient raw-temperature clamp into current_max.
	 */
	limit = curve->steps[sc->auto_curve_idx].ma;
	sc->curve_ma = entering ? limit : min(sc->curve_ma, limit);
	sc->thermal_ma = raw->steps != curve->steps ?
		raw->steps[erhai_curve_start(raw, mv, true)].ma : curve->steps[0].ma;
	limit = min(sc->curve_ma, sc->thermal_ma) / 100;
	limit = min_t(unsigned int, limit, sc->adapter_cap);
	/* A source can remember the first ASK_CURRENT as the session ceiling.
	 * The worker renegotiates if a recovered temperature needs more.
	 */
	if (sc->first_request && !entering)
		limit = min_t(unsigned int, limit, sc->first_request);
	sc->fastchg_curr_code = limit;
	return true;
}

/* A sysfs read never consumes REG0F/RX or blocks an IRQ on I2C. -1 means
 * there is no valid measurement for this handshake yet.
 */
static void sc8547_serial_clear_sample(struct sc8547a_device *sc)
{
	sc->frame_seen = false;
	memset(sc->model_frames, 0, sizeof(sc->model_frames));
	memset(sc->model_flags, 0, sizeof(sc->model_flags));
	sc->frame_gap_us = sc->max_frame_gap_us = 0;
	sc->voocphy_rx_buff = sc->vooc_flag = 0;
	sc->master_ibus_ma = sc->slave_ibus_ma = -1;
	sc->master_enabled = sc->slave_enabled = -1;
	sc->serial_vbus_mv = sc->serial_reg06 = -1;
	sc->serial_ibat_ma = 0;
	sc->first_request = sc->serial_request = sc->pending_request = 0;
	sc->request_pending = sc->asked_current = sc->retry_current = false;
	sc->measure_missed_slot = false;
	sc->adjust_failures = sc->above_request_count = 0;
	sc->next_current_sample = jiffies;
	sc->serial_predata = sc->ask_predata = 0;
}

static void sc8547_serial_end(struct sc8547a_device *sc, const char *why)
{
	sc->serial_last_exit = why;
	sc->serial_exits++;
	dev_info(sc->dev, "SVOOC end: %s, code=%u ibus=%d/%d mA en=%d/%d vbus=%d mV retries=%u last_rx=%02x flags=%02x\n",
		 why, sc->serial_request, sc->master_ibus_ma, sc->slave_ibus_ma,
		 sc->master_enabled, sc->slave_enabled, sc->serial_vbus_mv,
		 sc->rehandshakes, sc->voocphy_rx_buff, sc->vooc_flag);
	sc->serial_request = 0;
	sc->request_pending = sc->measure_missed_slot = false;
	sc->master_ibus_ma = sc->slave_ibus_ma = -1;
	sc->master_enabled = sc->slave_enabled = sc->serial_quiesced ? 0 : -1;
}

static void sc8547_vooc_reset_state(struct sc8547a_device *sc)
{
	sc->adapter_type = SC_ADAPTER_UNKNOWN;
	sc->adapter_check_ok = false;
	sc->adapter_rand_start = false;
	sc->fastchg_ask_cmd = VOOC_CMD_UNKNOWN;
	sc->pre_ask_cmd = VOOC_CMD_UNKNOWN;
	sc->adapter_ask_check_count = 3;
	sc->adapter_check_cmmu_count = 0;
	sc->adapter_model_count = 0;
	sc->model_received = false;
	sc->adapter_cap = 0;
	sc->ask_batt_sys = 0;
	sc->curve_sent = sc->allow_sent = sc->buck_ready = false;
	sc->pre_allow_buck_ua = -ENODATA;
	sc->vooc_head = SVOOC_INVERT_HEAD;
	sc->vooc_saw_any = false;
	/* delivery-phase state (gates fastchg_deliver / fastchg_curr_code persist) */
	sc->cp_enabled = false;
	sc8547_serial_clear_sample(sc);
	sc->notified_code = 0;
	sc->change_polls = 0;
	sc->adapter_vbat_mv = 0;
	sc->serial_ucp_armed = false;
	sc->low_ibus_count = 0;
	sc->load_seen = false;
	sc->slave_check_pending = sc->slave_trouble = false;
	sc->leg_over_count = 0;
	sc->next_leg_check = jiffies;
	sc->hw_profile_set = false;
	sc->vooc_vbus_status = SC_VBUS_LOW;
}

/*
 * Flip the WCD939x USBSS crossbar to the charge-pump D+/D- path and arm the
 * VOOC PHY. erhai routes the connector D+/D- to this charge pump through the
 * USBSS analog switch, not straight to the SoC, so without this the VOOC serial
 * handshake never reaches the adapter (total silence). This MUST run AFTER the
 * Type-C mux has programmed the USBSS for the new attach, else the mux
 * reprograms the switch back to the SoC USB route and clobbers it; the attach
 * poller enforces that ordering. -ENODEV (no USBSS) is non-fatal for a
 * discrete-DPDM board. One USB-C port exists, so no analog-headset collision.
 */
static void sc8547_vooc_arm_hw(struct sc8547a_device *sc)
{
	int ret;

	sc->serial_quiesced = false;

	sc8547_vooc_reset_state(sc);
	ret = wcd939x_usbss_dpdm_switch_fast_charge(true);
	if (!ret)
		ret = sc8547_init_vooc(sc);
	if (!ret)
		ret = sc8547_svooc_hw_setting(sc);
	if (!ret)
		ret = sc8547_slave_hw_setting(sc);
	if (!ret)
		ret = sc8547_write_byte(sc, SC8547_REG_2B, 0x81);
	if (ret) {
		sc8547_reset_voocphy(sc);
		sc8547_slave_reset();
		wcd939x_usbss_dpdm_switch_fast_charge(false);
		sc->auto_state = SC_AUTO_DECLINED;
		sc8547_serial_end(sc, "phy-arm-failed");
		return;
	}
	sc->hw_profile_set = true;
}

/*
 * Auto path: decline fast-charge for this plug and restore the normal USB/PD
 * charge route. Battmgr can retry after backoff when the adapter or
 * battery conditions recover.
 */
static void sc8547_auto_decline(struct sc8547a_device *sc, const char *why)
{
	sc8547_slave_reset();			/* drop parallel CP */
	sc8547_reset_voocphy(sc);		/* stop VOOC frames, CP off */
	wcd939x_usbss_dpdm_switch_fast_charge(false);	/* D+/D- back to SoC */
	sc8547_set_adc_enable(sc, true);
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc->cp_enabled = false;
	sc->hw_profile_set = false;
	sc->auto_state = SC_AUTO_DECLINED;
	sc8547_serial_end(sc, why);
}

/*
 * Auto path: end fast-charge delivery and hand back to the normal charge path.
 * Called on curve exit, on a safety backstop trip, or on a margin-low stop.
 * The adapter drops to 5 V once VOOC frames stop; restoring the crossbar
 * returns D+/D- to the SoC for BC1.2/PD, so the pack keeps charging over the
 * normal battmgr path. Re-evaluate known bricks after a holdoff; a
 * protection trip keeps the pumps off for the rest of the session.
 */
static void sc8547_auto_finish(struct sc8547a_device *sc, const char *why)
{
	if (!sc8547_serial_off(sc)) {
		sc8547_reset_voocphy(sc);
		wcd939x_usbss_dpdm_switch_fast_charge(false);
	}
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc->cp_enabled = false;
	sc->hw_profile_set = false;
	sc->auto_state = SC_AUTO_DONE;
	sc8547_serial_end(sc, why);
}

/*
 * Auto path re-servo: an SVOOC adapter periodically resets its VOOC link mid-CC
 * (drops output below the CP margin so CHG_EN self-clears; Vbus returns to ~5 V
 * and the adapter waits for a fresh handshake). Below the CV ceiling that is a
 * keep-alive reset, not an unplug (the >=CV case was already caught by the hard
 * backstop), so instead of ending fast-charge we drop the CP, re-arm the PHY and
 * return to identify to re-handshake and resume CC — matching the vendor engine.
 * The crossbar stays on the CP path (no USB re-enumeration churn). A re-servo
 * that follows a sustained CC period refills the retry budget; a burst of quick
 * collapses without progress exhausts it and hands back to normal charge.
 */
static void sc8547_auto_reservo(struct sc8547a_device *sc, const char *why)
{
	/* Stop both legs before touching the crossbar, resetting or feeding.
	 * A lost ACK is fenced by the existing pump-off watchdog path.
	 */
	if (sc->auto_protection || sc8547_take_faults() > 0) {
		sc->auto_protection = true;
		sc8547_auto_finish(sc, "hw-ovp-ocp");
		return;
	}
	sc->serial_last_retry = why;
	if (sc8547_serial_off(sc)) {
		sc8547_auto_finish(sc, "pump-stop-pending");
		return;
	}

	if (sc->auto_reservo_cnt >= SC8547_AUTO_RESERVO_MAX) {
		sc8547_auto_finish(sc, "retry-exhausted");
		return;
	}
	sc->auto_reservo_cnt++;
	sc->auto_ident_retries = 0; /* This recovery owns a fresh set of identify windows. */
	sc->rehandshakes++;
	sc->awaiting_rehandshake = true;

	dev_info(sc->dev, "SVOOC re-handshake %d/%d: %s, request=%u ibus=%d/%d mA en=%d/%d vbus=%d mV rx_age=%u ms last_rx=0x%02x flags=0x%02x reg06=%d pack_ma=%d\n",
		 sc->auto_reservo_cnt, SC8547_AUTO_RESERVO_MAX, why, sc->serial_request,
		 sc->master_ibus_ma, sc->slave_ibus_ma, sc->master_enabled, sc->slave_enabled,
		 sc->serial_vbus_mv, (unsigned int)div_u64(ktime_get_ns() - sc->last_frame_ns, NSEC_PER_MSEC),
		 sc->voocphy_rx_buff, sc->vooc_flag, sc->serial_reg06, sc->serial_ibat_ma);
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc8547_vooc_arm_hw(sc);
	if (sc->auto_state == SC_AUTO_DECLINED)
		return;
	sc->auto_state = SC_AUTO_IDENTIFY;
	sc->auto_identify_deadline = jiffies +
		msecs_to_jiffies(SC8547_AUTO_IDENTIFY_MS);
}

/*
 * Android parity for a slow/noisy serial ladder: when the identify window
 * expires without adapter_check_ok, RE-ARM the PHY + handshake (the vendor
 * engine's "abnormal adapter need delay" retry) up to identify_retries times,
 * instead of the old permanent single-shot decline.  The ADSP fast-charge
 * verdict is consulted (adsp_classify) as a second signal — the vendor keys
 * the retry off oplus_chglib_is_pd_svooc_adapter()/get_adapter_sid_power()
 * [>= 80 W] — but even without it we still give the serial ladder its bounded
 * chance, because the final decision stays with the serial exchange.
 * Returns true when re-armed (stay in IDENTIFY), false when declining.
 */
static bool sc8547_auto_identify_retry(struct sc8547a_device *sc)
{
	int ft, rt;

	if (sc->auto_ident_retries >= identify_retries)
		return false;

	/* A new arm clears the frame decoder, not the charger's identity.
	 * Consult the PMIC only for an as-yet unidentified, silent source.
	 */
	if (!(sc->responded & ERHAI_SERIAL) && !sc->vooc_saw_any && adsp_classify) {
		ft = erhai_otg_adsp_usb_prop(adsp_fastchg_type_prop);
		rt = erhai_otg_adsp_usb_prop(adsp_real_type_prop);
		if ((ft >= 0 && (ft & 0x7f) >= adsp_fastchg_svooc_min) ||
		    rt == adsp_vooc_real_type)
			sc->responded |= ERHAI_SERIAL;
	}
	if (!(sc->responded & ERHAI_SERIAL) && !sc->vooc_saw_any)
		return false;

	dev_info(sc->dev, "SVOOC re-arming identify %u/%u (responded=%u frames=%d)\n",
		 sc->auto_ident_retries + 1, identify_retries, sc->responded, sc->vooc_saw_any);
	sc->auto_ident_retries++;
	sc8547_vooc_arm_hw(sc);		/* re-init PHY + resend handshake */
	if (sc->auto_state == SC_AUTO_DECLINED)	/* DP alt-mode owns the switch */
		return false;
	sc->auto_state = SC_AUTO_IDENTIFY;
	sc->auto_identify_deadline = jiffies + msecs_to_jiffies(identify_ms);
	return true;
}

/*
 * Auto path identify tick: run while armed in observation, before any current.
 * Once the adapter has authenticated as a genuine (S)VOOC adapter, read the
 * gauge, pick the CC/CV curve for this SOC/temp and check vbat headroom, then
 * escalate to delivery in place (the single ladder is still pre-curve, so
 * setting fastchg_enable/deliver here is seen when the allow-bit is later read).
 * Decline only on identify-window exhaustion (after the Android-parity retry
 * loop) or any failed precondition.
 */
static void sc8547_auto_identify_tick(struct sc8547a_device *sc, int gret,
				      int soc, int temp_ddc, int cell_mv)
{
	if (!sc->adapter_check_ok || !sc->model_received) {
		if (time_before(jiffies, sc->auto_identify_deadline))
			return;
		if (sc8547_auto_identify_retry(sc))
			return;
		sc8547_auto_decline(sc, "no-vooc-response");
		return;
	}

	sc->adapter_cap = sc8547_adapter_cap(sc->adapter_model_ver);
	if (sc->adapter_type != SC_ADAPTER_SVOOC || !sc->adapter_cap) {
		sc8547_auto_decline(sc, "unsupported-model");
		return;
	}

	if (gret) {
		sc8547_auto_decline(sc, "gauge-unavailable");
		return;
	}

	/*
	 * Gate and index the curve on the CELL voltage (gauge). The CP's own
	 * reading is inflated by the delivery IR drop and would make a half-full
	 * pack look like it is already at CV.
	 */
	if (!sc8547_auto_update_curve(sc, soc, temp_ddc, cell_mv, 0, true)) {
		sc8547_auto_decline(sc, "outside-cell-curve");
		return;
	}
	if (!sc->hw_profile_set) {		/* normally pre-applied at arm */
		sc8547_svooc_hw_setting(sc);
		sc->hw_profile_set = true;
	}
	sc->fastchg_enable = true;
	sc->fastchg_deliver = true;
	sc->fastchg_auto = false;	/* one-shot per arm consumed */
	sc->auto_state = SC_AUTO_DELIVER;
	if (sc->curve_sent) {
		sc->allow_deadline = jiffies + msecs_to_jiffies(identify_ms);
		erhai_charge_recheck();
	}
	if (sc->awaiting_rehandshake) {
		sc->recoveries_answered++;
		sc->awaiting_rehandshake = false;
		dev_info(sc->dev, "SVOOC re-handshake answered: model=0x%02x window=%u\n",
			 sc->adapter_model_ver, sc->auto_ident_retries);
	}
	dev_info(sc->dev,
		 "auto fast-charge: SVOOC identified (soc %d%% temp %d.%d C cell %d mV cp %d mV) — deliver curve[%d] code %u\n",
		 soc, temp_ddc / 10, temp_ddc % 10, cell_mv, sc8547_get_vbat(sc), sc->auto_curve_idx,
		 sc->fastchg_curr_code);
}

#define SC8547_ATTACH_SETTLE_MS	300
#define SC8547_ATTACH_POLL_MS	250

/* Restore stock REG05 UCP and REG08 soft-start timeout after pickup.
 * The slave starts with its own ADC near zero; arm it on its own load.
 */
static int sc8547_serial_arm_ucp(struct sc8547a_device *sc)
{
	int ret;

	if (sc->serial_ucp_armed)
		return 0;
	ret = sc8547_update_bits(sc, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK, 0);
	ret = ret ?: sc8547_update_bits(sc, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
					SC8547_SS_TIMEOUT_81920MS << SC8547_SS_TIMEOUT_SET_SHIFT);
	if (!ret)
		sc->serial_ucp_armed = true;
	return ret;
}

/* 24924 copycat_vooc_support, non-bidirectional CP: compare gauge
 * charge current / 2 against the transmitted ASK_CURRENT value. See
 * oplus_ap_voocphy.c:3392-3396, 5782-5806 and VOOC_CURR_EVENT_TIME.
 */
#define SC8547_REQUEST_MARGIN_MA	400
#define SC8547_OVER_REQUEST_SAMPLES	10

/* AP VOOC PHY IBUS_CHECK: three 100 ms samples over the board's
 * 4500 mA per-leg limit reduce current_max to code 30. IRQs keep this
 * running during gauge transactions. Neither ADC replaces REG0F OCP.
 */
static void sc8547_check_leg_current(struct sc8547a_device *sc, int master, int slave)
{
	if (master < 0 || slave < 0 || time_before(jiffies, sc->next_leg_check))
		return;
	sc->next_leg_check = jiffies + msecs_to_jiffies(100);
	if (master <= 4500 && slave <= 4500) {
		sc->leg_over_count = 0;
		return;
	}
	if (++sc->leg_over_count == 3) {
		sc->curve_ma = min(sc->curve_ma, 3000U);
		sc->fastchg_curr_code = min_t(u8, sc->fastchg_curr_code, 30);
	}
}

static bool sc8547_current_in_range(struct sc8547a_device *sc, int ibat_ma)
{
	int bus_ma = ibat_ma / 2;

	if (time_before(jiffies, sc->next_current_sample))
		return true;
	sc->next_current_sample = jiffies + msecs_to_jiffies(1000);
	if (!sc->cp_enabled || !sc->serial_request || bus_ma <= 1500 ||
	    bus_ma <= sc->serial_request * 100 + SC8547_REQUEST_MARGIN_MA) {
		sc->above_request_count = 0;
		return true;
	}
	/* Vendor count starts at ten, prompts at three remaining, reloads
	 * ten and exits on the following excessive sample with zero left.
	 */
	if (++sc->above_request_count == SC8547_OVER_REQUEST_SAMPLES - 3) {
		sc->retry_current = true;
		sc->change_polls = sc->adjust_failures = 0;
		dev_info(sc->dev, "SVOOC over confirmed code %u: pack %d mA; asking again\n",
			 sc->serial_request, ibat_ma);
	}
	return sc->above_request_count < 2 * SC8547_OVER_REQUEST_SAMPLES - 2;
}

/* A late frame can arrive between monitor polls. It belongs to a broken
 * exchange and must not refresh the watchdog for the old contract.
 */
static bool sc8547_serial_timed_out(struct sc8547a_device *sc, u64 now_ns)
{
	return sc->auto_state == SC_AUTO_DELIVER && sc->frame_seen &&
	       now_ns >= sc->last_frame_ns + SC8547_COMMU_TIMEOUT_MS * NSEC_PER_MSEC;
}

static void sc8547_attach_work(struct work_struct *work)
{
	struct sc8547a_device *sc =
		container_of(work, struct sc8547a_device, attach_work.work);
	u64 sampled_at;
	int gsoc = -1, gtemp = 0, gvbat = -1, ibat = 0;
	int gret = -ENODATA, vbus;

	guard(mutex)(&sc8547_pps_lock);
	if (!sc->vooc_started)
		return;
	if (sc8547_serial_feed(sc)) {
		sc8547_auto_reservo(sc, "watchdog-feed-io");
		goto reschedule;
	}
	/* A pack/gauge GLINK round trip can outlast a PHY reply slot. The
	 * gauge belongs to the pack, so it can be read outside the port
	 * lock. Recheck ownership before touching either converter.
	 */
	if (sc->auto_state == SC_AUTO_DELIVER ||
	    (sc->auto_armed && sc->auto_state == SC_AUTO_IDENTIFY &&
	     sc->adapter_check_ok && sc->model_received)) {
		sc->gauge_reading = true;
		mutex_unlock(&sc8547_pps_lock);
		gret = sc8547_read_gauge(sc, &gsoc, &gtemp, &gvbat, &ibat);
		mutex_lock(&sc8547_pps_lock);
		sc->gauge_reading = false;
		if (!sc->vooc_started || !sc->auto_armed ||
		    (sc->auto_state != SC_AUTO_IDENTIFY && sc->auto_state != SC_AUTO_DELIVER))
			return;
	}
	sampled_at = ktime_get_ns();

	vbus = sc8547_get_vbus(sc);
	if (!sc->auto_armed) {
		sc8547_vooc_arm_hw(sc);
		sc->auto_armed = true;
		sc->auto_identify_deadline = jiffies + msecs_to_jiffies(identify_ms);
	}

	/* Auto path: watch for identify, escalate to delivery or decline. */
	if (sc->auto_armed && sc->auto_state == SC_AUTO_IDENTIFY)
		sc8547_auto_identify_tick(sc, gret, gsoc, gtemp, gvbat);

	/*
	 * Stage-2 safety backstop while the CP is delivering: keep the hardware
	 * watchdog fed and hard-stop on an over-voltage / over-current reading.
	 * Latches fastchg_deliver off so a trip needs an explicit re-arm.
	 */
	if (sc->auto_state == SC_AUTO_DELIVER) {
		int cpbat = sc8547_get_vbat(sc);	/* cell + delivery IR drop */
		int cibus = sc8547_get_ibus(sc);
		int cvbat = gvbat;
		int previous_temp = sc->curve_temp;

		/*
		 * Under dual-CP the master REG_13 reads only its half of the bus
		 * current; the CC taper compare and the over-current backstop both
		 * need the TOTAL bus, so sum in the slave's half (0 if single-CP).
		 */
		int slave_ma = sc8547_slave_ibus();
		int cibus_total = cibus + slave_ma;
		int margin = vbus - 2 * cpbat;
		u8 r6 = 0, r7 = 0;
		int faults = sc8547_take_faults();
		int vsys = sc8547_get_output_mv(sc, SC8547_REG_19);

		/* 24926 has no external_gauge_support. In the stock AP PHY
		 * GET_BATT_VOL uses REG1B for gauge_vbatt and both REG13s
		 * for cp_ichg. The pack gauge still owns the cell curve.
		 */
		sc->master_ibus_ma = cibus < 0 ? -1 : cibus;
		sc->slave_ibus_ma = slave_ma < 0 ? -1 : slave_ma;
		if (sc->measure_missed_slot && cibus >= 0 && slave_ma >= 0)
			sc->missed_peak_ibus_ma = max(sc->missed_peak_ibus_ma, cibus_total);
		sc->serial_vbus_mv = vbus > 0 ? vbus : -1;
		sc->serial_ibat_ma = gret ? 0 : ibat;
		sc->serial_reg06 = sc8547_read_byte(sc, SC8547_REG_06, &r6) ? -1 : r6;
		sc->slave_enabled = sc8547_slave_enabled();
		sc->master_enabled = sc8547_read_byte(sc, SC8547_REG_07, &r7) ?
			-1 : !!(r7 & SC8547_CHG_EN_MASK);
		if (cpbat > 0 && vsys > 0 && vbus > 0 && cibus >= 0 && slave_ma >= 0)
			sc->adapter_vbat_mv = vbus / 2 - (vsys - cpbat) / 4 -
					      cibus_total * 75 / 1000;
		else
			sc->adapter_vbat_mv = 0;

		if (faults > 0) {
			sc->auto_protection = true;
			sc8547_auto_finish(sc, "hw-ovp-ocp");
			goto reschedule;
		}
		if (sc->curve_sent && !sc->buck_ready && time_after_eq(jiffies, sc->allow_deadline)) {
			sc8547_auto_reservo(sc, "buck-still-drawing");
			goto reschedule;
		}
		if (sc->cp_enabled)
			sc8547_check_leg_current(sc, cibus, slave_ma);
		if (cpbat >= SC8547_CP_OVP_CEILING_MV || cibus_total >= SC8547_IBUS_ABORT_MA || (!gret && ibat > 13600))
			sc->adc_over_count++;
		else
			sc->adc_over_count = 0;
		if (faults < 0 || sc->adc_over_count > 1) {
			sc8547_auto_finish(sc, "adc-limit-or-io");
			goto reschedule;
		}
		/* Four low readings on a discharging cell cannot qualify as a charge taper. */
		if (ibat <= 0)
			sc->auto_taper_cnt = 0;
		if (gret || cpbat <= 0 || vbus <= 0 || cibus < 0 || slave_ma < 0 ||
		    !sc8547_auto_update_curve(sc, gsoc, gtemp, cvbat, cibus_total, false)) {
			if (!gret && gtemp >= 0 && gtemp < 510 && gvbat > 0 &&
			    (gvbat >= min(erhai_cell_cv(gtemp), erhai_cell_cv(sc->curve_temp)) ||
			     sc->auto_taper_cnt >= SC8547_AUTO_STEP_DEBOUNCE)) {
				erhai_charge_tail_complete(ERHAI_SERIAL);
				sc8547_auto_finish(sc, "buck-tail");
			} else {
				sc8547_auto_finish(sc, "cell-envelope-or-adc");
			}
			goto reschedule;
		}
		if (sc8547_new_temperature_session(previous_temp, sc->curve_temp, gsoc)) {
			sc8547_auto_reservo(sc, "temperature-range");
			goto reschedule;
		}
		if (sc->first_request &&
		    min_t(unsigned int, min(sc->curve_ma, sc->thermal_ma) / 100, sc->adapter_cap) > sc->first_request) {
			sc8547_auto_reservo(sc, "temperature-ceiling");
			goto reschedule;
		}
		/* RX_DONE frames are clocked by the source. CHG_EN can remain
		 * set indefinitely with no source current while UCP is masked.
		 */
		/* A pending IRQ may be a frame that arrived before the deadline.
		 * Its thread checks the captured edge against the previous valid
		 * one; a non-frame IRQ never refreshes that deadline.
		 */
		if (sc8547_serial_timed_out(sc, sampled_at) &&
		    !(smp_load_acquire(&sc->irq_pending) &&
		      !sc8547_serial_timed_out(sc, READ_ONCE(sc->irq_arrived_ns)))) {
			sc8547_auto_reservo(sc, "frame-timeout");
			goto reschedule;
		}
		if (sc->adc_over_count) {
			sc->fastchg_curr_code = max_t(int, SC8547_CURR_CODE_MIN, sc->fastchg_curr_code - 1);
			sc->curve_ma = min(sc->curve_ma, sc->fastchg_curr_code * 100U);
		}
		if (!sc8547_current_in_range(sc, ibat)) {
			sc8547_auto_reservo(sc, "above-request");
			goto reschedule;
		}
		if (cibus_total >= SC8547_LOW_IBUS_MA && !sc->above_request_count && sc->serial_request) {
			sc->low_ibus_count = 0;
			if (!sc->load_seen) {
				sc->load_since = jiffies;
				sc->load_seen = true;
			}
			if (time_after_eq(jiffies, sc->load_since + msecs_to_jiffies(SC8547_AUTO_SUSTAINED_MS)))
				sc->auto_reservo_cnt = 0;
		} else {
			sc->load_seen = false;
			if (!sc->asked_current || cibus_total >= SC8547_LOW_IBUS_MA)
				sc->low_ibus_count = 0;
			else if (++sc->low_ibus_count >= SC8547_AUTO_STEP_DEBOUNCE) {
				dev_info(sc->dev, "SVOOC current lost: code=%u master=%d slave=%d mA vbus=%d cp_vbat=%d en=%d/%d\n",
					 sc->serial_request, cibus, slave_ma, vbus, cpbat, sc->master_enabled, sc->slave_enabled);
				sc8547_auto_reservo(sc, "no-load");
				goto reschedule;
			}
		}

		if (!sc->cp_enabled)
			goto reschedule;

		if (sc->master_enabled < 0 || sc->slave_enabled < 0 || sc->serial_reg06 < 0) {
			sc8547_auto_reservo(sc, "pump-status-io");
			goto reschedule;
		}

		if (sc->vooc_vbus_status == SC_VBUS_NORMAL && cibus_total > SC8547_UCP_ARM_MA &&
		    (sc8547_serial_arm_ucp(sc) || (slave_ma > SC8547_UCP_ARM_MA &&
		    sc8547_serial_arm_ucp(sc8547_slave_dev)))) {
			sc8547_auto_reservo(sc, "ucp-restore-io");
			goto reschedule;
		}

		if (!(r7 & SC8547_CHG_EN_MASK)) {
			/*
			 * CHG_EN self-cleared in hardware (VBUS_ERRORLO / UCP
			 * transient). Recover only when the Vbus margin is healthy
			 * and we are below the CV ceiling; otherwise stop clean
			 * rather than fake a live CP or oscillate into a fault.
			 */
			if (margin >= SC8547_CP_RESTART_MARGIN_MV &&
			    margin <= SC8547_CP_RESTART_MARGIN_CEIL_MV) {
				if (sc8547_write_byte(sc, SC8547_REG_09, 0x13) ||
				    sc8547_set_chg_enable(sc, true)) {
					sc8547_auto_reservo(sc, "pump-enable-io");
					goto reschedule;
				}
				dev_warn(sc->dev,
					 "fastchg: CHG_EN self-cleared, re-enabled (vbus %d mV vbat %d mV margin %d mV REG06=0x%02x)\n",
					 vbus, cvbat, margin, r6);
			} else {
				sc8547_auto_reservo(sc, "converter-window");
				goto reschedule;
			}
		} else {
			bool slave_on = sc->slave_enabled > 0;

			u8 switching;

			/* Stock checks EN, switching, minimum slave load and pair
			 * deviation after 300 ms, for up to 60 more 50 ms slots.
			 * Observe it without sleeping in the port lock.
			 */
			if (sc->slave_check_pending && time_after_eq(jiffies,
			    sc->slave_on_since + msecs_to_jiffies(300))) {
				if (slave_on && slave_ma >= 500 && abs(cibus - slave_ma) <= 1000 &&
				    !sc8547_read_byte(sc8547_slave_dev, SC8547_REG_06, &switching) &&
				    (switching & BIT(2)))
					sc->slave_check_pending = false;
				else if (time_after_eq(jiffies,
					 sc->slave_on_since + msecs_to_jiffies(300 + 60 * 50)))
					sc->slave_trouble = true;
			}
			if (!slave_on && !sc->slave_check_pending && !sc->slave_trouble &&
			    sc->vooc_vbus_status == SC_VBUS_NORMAL &&
			    cibus > SC8547_SLAVE_ENGAGE_IBUS_MA) {
				if (sc8547_slave_cp_enable(sc, true)) {
					sc->slave_trouble = true;
				} else {
					sc->slave_check_pending = true;
					sc->slave_on_since = jiffies;
					sc->slave_low_cnt = 0;
					dev_info_ratelimited(sc->dev, "dual-CP: requested slave under %d mA master load\n",
							     cibus);
				}
			}
			if (sc->slave_trouble) {
				sc->curve_ma = min(sc->curve_ma, 3000U);
				sc->fastchg_curr_code = min_t(u8, sc->fastchg_curr_code, 30);
				sc->slave_check_pending = false;
				if (slave_on && sc8547_slave_cp_enable(sc, false)) {
					sc8547_auto_reservo(sc, "slave-stop-io");
					goto reschedule;
				}
			} else if (slave_on && slave_settle_ms &&
				   jiffies - sc->slave_on_since <
					   msecs_to_jiffies(slave_settle_ms)) {
				/*
				 * Settling: the adapter's output is already
				 * split, but the slave has not ramped yet (its
				 * ADC may still read its idle ~20 mA).  Feed
				 * it; never judge the drop inside this window.
				 */
				dev_dbg(sc->dev,
					"dual-CP: slave settling (%u ms on, total %d mA)\n",
					jiffies_to_msecs(jiffies - sc->slave_on_since),
					cibus_total);
			} else if (slave_on &&
				   cibus_total < SC8547_SLAVE_DISENGAGE_IBUS_MA) {
				if (++sc->slave_low_cnt >= slave_drop_ticks) {
					sc8547_slave_cp_enable(sc, false);
					sc->slave_low_cnt = 0;
					dev_info(sc->dev,
						 "dual-CP: dropping slave (total %d mA low for %u ticks, %u ms on)\n",
						 cibus_total, slave_drop_ticks,
						 jiffies_to_msecs(jiffies - sc->slave_on_since));
				}
			} else if (slave_on) {
				sc->slave_low_cnt = 0;
			}

			dev_dbg(sc->dev, "fastchg: vbus %d mV vbat %d mV ibus %d mA (master %d slave %d) margin %d mV REG06=0x%02x REG07=0x%02x slave_on=%d\n",
				 vbus, cvbat, cibus_total, cibus, sc8547_slave_ibus(),
				 margin, r6, r7, slave_on);
		}
	}

reschedule:
	if (sc->auto_state == SC_AUTO_IDENTIFY || sc->auto_state == SC_AUTO_DELIVER)
		schedule_delayed_work(&sc->attach_work,
				      msecs_to_jiffies(SC8547_ATTACH_POLL_MS));
}

static const char * const serial_state_names[] = {
	"off", "identify", "deliver", "declined", "done",
};

ssize_t sc8547a_serial_describe(char *buf, ssize_t off, bool *active)
{
	struct sc8547a_device *sc;
	int ibus, pump;
	long long age;

	guard(mutex)(&sc8547_pps_lock);
	sc = sc8547_master_dev;
	*active = sc && sc->vooc_started && sc->auto_state != SC_AUTO_OFF;
	if (!sc)
		return off;
	off += sysfs_emit_at(buf, off, "btb_batt_mc=%d btb_usb_mc=%d btb_valid=%d btb_hot=%d "
			     "btb_samples=%u btb_age_ms=%lld btb_err=%d/%d ",
			     sc->btb.online && !sc->btb.battery_err ? sc->btb.battery_mc : INT_MIN,
			     sc->btb.online && !sc->btb.usb_err ? sc->btb.usb_mc : INT_MIN,
			     sc->btb.valid, sc->btb.tripped, sc->btb.hot_count,
			     sc->btb.online && sc->btb.sampled_at ?
				(long long)jiffies_to_msecs(jiffies - sc->btb.sampled_at) : -1LL,
			     sc->btb.battery_err, sc->btb.usb_err);
	off += sysfs_emit_at(buf, off, "svooc_last_exit=%s svooc_last_retry=%s rehandshakes=%u rehandshakes_answered=%u missed_asks=%u missed_code=%u missed_predata=0x%04x missed_peak_ibus_ma=%d ",
			     sc->serial_last_exit ?: "none", sc->serial_last_retry ?: "none",
			     sc->rehandshakes, sc->recoveries_answered, sc->missed_asks, sc->missed_request, sc->missed_predata,
			     sc->missed_asks ? sc->missed_peak_ibus_ma : -1);
	if (!*active)
		return off;

	age = sc->frame_seen ? (long long)(unsigned int)div_u64(ktime_get_ns() - sc->last_frame_ns, NSEC_PER_MSEC) : -1;
	ibus = sc->master_ibus_ma < 0 || sc->slave_ibus_ma < 0 ? -1 :
		sc->master_ibus_ma + sc->slave_ibus_ma;
	pump = sc->master_enabled > 0 || sc->slave_enabled > 0 ? 1 :
		sc->master_enabled < 0 || sc->slave_enabled < 0 ? -1 : 0;
	return off + sysfs_emit_at(buf, off,
		"curve_temp=%d sample_temp=%d curve_code=%u thermal_code=%u curve_idx=%u warm_boundary=%d "
		"protocol=supervooc state=%s model=%d allow_sent=%d pre_allow_buck_ua=%d first_request_code=%u target_code=%u request_code=%u request_ma=%u "
		"pending_code=%d predata=0x%04x above_samples=%u model_rx=0x%02x%02x model_flags=0x%02x%02x "
		"vbus_mv=%d pump=%d pump_ibus_ma=%d master_ibus_ma=%d slave_ibus_ma=%d "
		"master_pump=%d slave_pump=%d frame_age_ms=%lld last_rx=0x%02x flags=0x%02x reg06=%d rx_errors=%u tx_errors=%u frame_gap_us=%u max_gap_us=%u "
		"exits=%u last_exit=%s\n",
		sc->curve_temp, sc->curve_sample_temp, sc->curve_ma / 100, sc->thermal_ma / 100,
		sc->auto_curve_idx, sc->curve_temp_bounds[5],
		serial_state_names[sc->auto_state], sc->model_received ? sc->adapter_model_ver : -1,
		sc->allow_sent, sc->pre_allow_buck_ua, sc->first_request,
		sc->fastchg_deliver ? sc->fastchg_curr_code : 0, sc->serial_request, sc->serial_request * 100,
		sc->request_pending ? sc->pending_request : -1, sc->serial_predata, sc->above_request_count,
		sc->model_frames[0], sc->model_frames[1], sc->model_flags[0], sc->model_flags[1],
		sc->serial_vbus_mv, pump, ibus, sc->master_ibus_ma, sc->slave_ibus_ma,
		sc->master_enabled, max(sc->slave_enabled, -1), age, sc->voocphy_rx_buff,
		sc->vooc_flag, sc->serial_reg06, sc->rx_errors, sc->tx_errors, sc->frame_gap_us, sc->max_frame_gap_us,
		sc->serial_exits, sc->serial_last_exit ?: "none");
}
EXPORT_SYMBOL_GPL(sc8547a_serial_describe);

/* Clipped table endpoints include open/short circuit. The ADC5 Gen4
 * 100k table (adcmap7_100k) uses these exact processed values.
 */
static int sc8547_btb_read(struct iio_channel *channel, int *mc)
{
	int ret = iio_read_channel_processed(channel, mc);

	if (ret < 0)
		return ret;
	if (*mc <= -40960 || *mc >= 130048)
		return -ERANGE;
	return 0;
}

/* A bad conversion withdraws pump permission immediately. It cannot
 * manufacture either an ambient temperature or a confirmed thermal trip.
 */
static unsigned int sc8547_btb_sample(struct sc8547_btb *b, int battery_err,
				      int battery_mc, int usb_err, int usb_mc)
{
	bool hot;

	b->sampled_at = jiffies;
	b->battery_err = battery_err;
	b->usb_err = usb_err;
	b->battery_mc = battery_err ? INT_MIN : battery_mc;
	b->usb_mc = usb_err ? INT_MIN : usb_mc;
	b->valid = !battery_err && !usb_err;
	if (!b->valid) {
		b->admitted = false;
		b->hot_count = 0;
		return b->tripped ? SC8547_BTB_TRIP_MS : SC8547_BTB_RUN_MS;
	}
	hot = battery_mc >= SC8547_BTB_HOT_MC || usb_mc >= SC8547_BTB_HOT_MC;
	if (!hot) {
		b->hot_count = 0;
		b->tripped = false;
		b->admitted = true;
		return SC8547_BTB_RUN_MS;
	}
	if (b->tripped)
		return SC8547_BTB_TRIP_MS;
	if (++b->hot_count >= (b->admitted ? 10 : 3)) {
		b->tripped = true;
		b->admitted = false;
		return SC8547_BTB_TRIP_MS;
	}
	return b->admitted ? SC8547_BTB_RUN_MS : 10;
}

static void sc8547_btb_work(struct work_struct *work)
{
	struct sc8547a_device *sc = container_of(to_delayed_work(work), struct sc8547a_device,
						 btb.work);
	struct sc8547_btb *b = &sc->btb;
	unsigned int attachment, delay;
	unsigned long due, started = jiffies;
	int battery = 0, usb = 0, battery_err, usb_err;
	bool allowed, tripped, changed;

	scoped_guard(mutex, &sc8547_pps_lock) {
		if (!b->online)
			return;
		attachment = b->attachment;
	}
	/* ADC conversions can sleep. Keep both PHY and watchdog locks free. */
	battery_err = sc8547_btb_read(b->battery, &battery);
	usb_err = sc8547_btb_read(b->usb, &usb);
	scoped_guard(mutex, &sc8547_pps_lock) {
		if (!b->online || b->attachment != attachment)
			return;
		allowed = sc8547_btb_allows(b);
		tripped = b->tripped;
		delay = sc8547_btb_sample(b, battery_err, battery, usb_err, usb);
		changed = allowed != sc8547_btb_allows(b) || tripped != b->tripped;
		if (!sc8547_btb_allows(b)) {
			if (sc->vooc_started && (sc->auto_state == SC_AUTO_IDENTIFY ||
						 sc->auto_state == SC_AUTO_DELIVER))
				sc8547_auto_finish(sc, b->tripped ? "connector-hot" : "connector-adc");
			else if (pps_cp_engaged || pps_slave_engaged)
				sc8547_pps_disengage();
		}
		if (changed)
			dev_info(sc->dev, "connector protection: battery=%d usb=%d mC errors=%d/%d hot=%d admitted=%d\n",
				 b->battery_mc, b->usb_mc, battery_err, usb_err, b->tripped,
				 sc8547_btb_allows(b));
		due = started + msecs_to_jiffies(delay);
		mod_delayed_work(system_freezable_wq, &b->work,
				 time_after(due, jiffies) ? due - jiffies : 0);
	}
	if (changed)
		erhai_charge_recheck();
}

/* Called by the battery manager under its charge policy lock. Stop is
 * synchronous with both frame handling and the pump watchdog worker.
 */
void sc8547a_probe_status(struct erhai_probe_status *status)
{
	struct sc8547a_device *sc;

	guard(mutex)(&sc8547_pps_lock);
	sc = sc8547_master_dev;
	if (!sc)
		return;
	if (status->wired_online && !sc->btb.online) {
		sc->btb.online = true;
		mod_delayed_work(system_freezable_wq, &sc->btb.work,
				 sc->btb.tripped ? msecs_to_jiffies(SC8547_BTB_TRIP_MS) : 0);
	}
	status->connector_blocked = !sc8547_btb_allows(&sc->btb);
	if (status->connector_blocked)
		status->protected |= ERHAI_SERIAL | ERHAI_UFCS | ERHAI_PPS;
	status->ready |= ERHAI_PPS;
	if (sc->irq <= 0)
		return;
	if (serial_enable && sc8547_slave_dev)
		status->ready |= ERHAI_SERIAL;
#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
	if (sc->ufcs)
		status->ready |= ERHAI_UFCS;
#endif
	status->responded = sc->responded;
	status->first_ufcs_timeout = sc->first_ufcs_timeout;
	if (sc->auto_state == SC_AUTO_DELIVER && sc->fastchg_deliver) {
		status->delivering = ERHAI_SERIAL;
		status->serial_needs_buck_off = sc->curve_sent;
		status->serial_buck_ready = sc->buck_ready;
	}
	if (sc->auto_protection)
		status->protected |= ERHAI_SERIAL;
}
EXPORT_SYMBOL_GPL(sc8547a_probe_status);

/* Invalidate in-flight conversions before unplug or sleep. Keep a thermal
 * trip across suspend so waking up cannot bypass the 10 s recheck.
 */
static void sc8547_btb_offline(struct sc8547_btb *b)
{
	b->online = false;
	b->valid = false;
	b->admitted = false;
	b->battery_err = b->usb_err = -ENODATA;
	b->sampled_at = 0;
	b->hot_count = 0;
	b->attachment++;
	cancel_delayed_work(&b->work);
}

void sc8547a_detached(void)
{
	guard(mutex)(&sc8547_pps_lock);
	if (sc8547_master_dev) {
		struct sc8547_btb *b = &sc8547_master_dev->btb;

		sc8547_btb_offline(b);
		b->tripped = false;
		sc8547_master_dev->responded = 0;
		sc8547_master_dev->rehandshakes = sc8547_master_dev->recoveries_answered = 0;
		sc8547_master_dev->missed_asks = 0;
		sc8547_master_dev->rx_errors = sc8547_master_dev->tx_errors = 0;
		sc8547_master_dev->ufcs_handshake_tried = false;
		sc8547_master_dev->first_ufcs_timeout = false;
		sc8547_master_dev->auto_protection = false;
		sc8547_master_dev->pump_faults = 0;
	}
	if (sc8547_slave_dev)
		sc8547_slave_dev->pump_faults = 0;
}
EXPORT_SYMBOL_GPL(sc8547a_detached);

/* Called by battmgr only after the ADSP acknowledged USB ICL=0. */
void sc8547a_serial_buck_sample(int ua)
{
	struct sc8547a_device *sc;

	guard(mutex)(&sc8547_pps_lock);
	sc = sc8547_master_dev;
	if (sc && sc->auto_state == SC_AUTO_DELIVER && sc->curve_sent && !sc->buck_ready) {
		sc->pre_allow_buck_ua = ua;
		/* The suspended PMIC reports zero or one 3 mA ADC step. */
		sc->buck_ready = ua >= 0 && ua <= 3000;
	}
}
EXPORT_SYMBOL_GPL(sc8547a_serial_buck_sample);

int sc8547a_serial_session(bool enable)
{
	struct sc8547a_device *sc;
	int ret, route;

	guard(mutex)(&sc8547_pps_lock);
	sc = sc8547_master_dev;
	if (!sc || !serial_enable)
		return enable ? -EOPNOTSUPP : 0;
	if (enable && !sc8547_btb_allows(&sc->btb))
		return -EAGAIN;
	if (enable && sc->auto_protection)
		return -EPERM;
	if (enable == sc->vooc_started) {
		if (enable && (sc->auto_state == SC_AUTO_DECLINED || sc->auto_state == SC_AUTO_DONE))
			return -ENODATA;
		return 0;
	}
	if (enable) {
		if (sc->ufcs_owner || !sc8547_slave_dev || sc->irq <= 0 || pps_cp_engaged || pps_adc_hold)
			return -EAGAIN;
		sc->auto_state = SC_AUTO_IDENTIFY;
		memset(sc->curve_temp_bounds, 0, sizeof(sc->curve_temp_bounds));
		sc->fastchg_auto = true;
		sc->serial_quiesced = false;
		sc->adc_over_count = 0;
		sc->auto_armed = false;
		sc->auto_ident_retries = 0;
		sc->awaiting_rehandshake = false;
		sc8547_serial_clear_sample(sc);
		sc->model_received = false;
		sc->auto_reservo_cnt = 0;
		sc->slave_low_cnt = 0;
		sc->vooc_started = true;
		mod_delayed_work(system_wq, &sc->attach_work,
				 msecs_to_jiffies(SC8547_ATTACH_SETTLE_MS));
		return 0;
	}

	cancel_delayed_work(&sc->attach_work);
	if (sc->auto_state == SC_AUTO_IDENTIFY || sc->auto_state == SC_AUTO_DELIVER)
		sc8547_serial_end(sc, "route-released");
	sc->auto_state = SC_AUTO_OFF;
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc->cp_enabled = false;
	ret = sc8547_serial_off(sc);
	if (ret)
		return ret;
	sc8547_reset_voocphy(sc);
	route = wcd939x_usbss_dpdm_switch_fast_charge(false);
	if (!ret && !route)
		sc->vooc_started = false;
	return ret ?: route;
}
EXPORT_SYMBOL_GPL(sc8547a_serial_session);

/* IRQ-time frame processing. Returns true if a frame was handled. */
static bool sc8547_voocphy_isr(struct sc8547a_device *sc, u64 arrived_ns)
{
	int word = sc8547_get_adapter_info(sc);
	bool valid_rx;

	if (word < 0)
		return false;

	sc->voocphy_rx_buff = word & 0xff;	/* REG_2E: data */
	sc->vooc_flag = (word >> 8) & 0xff;	/* REG_2F: status */

	if (sc->vooc_flag & VOOC_TXDATA_WR_FAIL_FLAG) {
		sc->tx_errors++;
		if (sc->request_pending || sc->pre_ask_cmd == VOOC_CMD_ASK_CURRENT_LEVEL) {
			sc8547_request_missed(sc, sc->pending_request);
			sc->pre_ask_cmd = VOOC_CMD_UNKNOWN;
		}
	} else if (sc->request_pending &&
		   (sc->vooc_flag & (VOOC_TXDATA_DONE_FLAG | VOOC_RXDATA_DONE_FLAG))) {
		if (sc->serial_request != sc->pending_request)
			sc->above_request_count = 0;
		if (!sc->first_request && sc->pending_request)
			sc->first_request = sc->pending_request;
		sc->serial_request = sc->pending_request;
		sc->request_pending = sc->retry_current = false;
		sc->measure_missed_slot = false;
		sc->adjust_failures = sc->change_polls = 0;
	}
	if (!(sc->vooc_flag & VOOC_RXDATA_DONE_FLAG))
		return !!(sc->vooc_flag & (VOOC_TXDATA_DONE_FLAG | VOOC_TXDATA_WR_FAIL_FLAG));

	if (sc8547_serial_timed_out(sc, arrived_ns)) {
		/* The monitor checks latched OCP/OVP and CV before rearming. */
		mod_delayed_work(system_wq, &sc->attach_work, 0);
		return true;
	}
	if (vooc_get_mesg_from_adapter(sc)) {
		sc->rx_errors++;
		dev_dbg(sc->dev, "VOOC frame decode failed (rx 0x%02x)\n",
			sc->voocphy_rx_buff);
		return false;
	}

	valid_rx = vooc_reply_adapter_mesg(sc);
	if (sc->fastchg_ask_cmd == VOOC_CMD_TELL_USB_BAD) {
		/* The peer has withdrawn permission even if our reply missed. */
		sc8547_auto_finish(sc, "adapter-reported-bad-connection");
		return true;
	}
	if (valid_rx) {
		if (sc->adapter_type == SC_ADAPTER_SVOOC)
			sc->responded |= ERHAI_SERIAL;
		/* A combined RX/WR_FAIL IRQ still needs its normal reply.
		 * For an ASK on that IRQ REG31 may already have won the slot.
		 */
		if (sc->request_pending && (sc->vooc_flag & VOOC_TXDATA_WR_FAIL_FLAG))
			sc8547_request_missed(sc, sc->pending_request);
		if (sc->frame_seen) {
			sc->frame_gap_us = div_u64(arrived_ns - sc->last_frame_ns, NSEC_PER_USEC);
			sc->max_frame_gap_us = max(sc->max_frame_gap_us, sc->frame_gap_us);
		}
		sc->frame_seen = true;
		sc->last_frame_ns = arrived_ns;
		/* Apply the stock 100 ms per-leg check after replying. */
		if (sc->cp_enabled && time_after_eq(jiffies, sc->next_leg_check))
			sc8547_check_leg_current(sc, sc8547_get_ibus(sc), sc8547_slave_ibus());
		if (sc->gauge_reading && sc->cp_enabled &&
		    time_after_eq(jiffies, sc->watchdog_fed_at + msecs_to_jiffies(SC8547_ATTACH_POLL_MS))) {
			int master, slave;

			if (sc8547_serial_feed(sc)) {
				sc8547_auto_reservo(sc, "watchdog-feed-io");
				return true;
			}
			master = sc8547_get_ibus(sc);
			slave = sc8547_slave_ibus();
			if (master < 0 || slave < 0) {
				sc8547_auto_reservo(sc, "pump-adc-io");
				return true;
			}
			sc->master_ibus_ma = master;
			sc->slave_ibus_ma = slave;
			if (sc->measure_missed_slot)
				sc->missed_peak_ibus_ma = max(sc->missed_peak_ibus_ma, master + slave);
			/* This IRQ has no fresh gauge sample for the 1 s contract check. */
		}
	}
	dev_dbg(sc->dev, "VOOC mesg=0x%02x ask=0x%02x tx=%02x%02x\n",
		sc->adapter_mesg, sc->fastchg_ask_cmd,
		sc->voocphy_tx_buff[0], sc->voocphy_tx_buff[1]);
	sc->pre_ask_cmd = sc->fastchg_ask_cmd == VOOC_CMD_ASK_CURRENT_LEVEL && !sc->request_pending ?
		VOOC_CMD_UNKNOWN : sc->fastchg_ask_cmd;
	return true;
}

/* Serialize synchronous protocol transactions against unbind. The
 * threaded IRQ only takes pps_lock; the engine waits for IRQs with
 * that lock released. The charger worker owns the route and pumps.
 */
static DEFINE_MUTEX(sc8547_ufcs_lock);

#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
static int sc8547_ufcs_prepare(void *ctx)
{
	struct sc8547a_device *sc = ctx;
	int ret;

	guard(mutex)(&sc8547_pps_lock);
	if (!sc->ufcs_owner || sc->vooc_started || pps_cp_engaged)
		return -EBUSY;
	ret = sc8547_reg_reset(sc);
	if (ret)
		return ret;
	msleep(10); /* vendor SC8547A reset settling time */
	return sc8547_init_device(sc);
}

static void sc8547_ufcs_unregister(void *arg)
{
	ufcs_device_unregister(arg);
}

static int sc8547_ufcs_init(struct sc8547a_device *sc)
{
	static const struct regmap_config config = { .reg_bits = 8, .val_bits = 8 };

	if (sc->is_slave || sc->irq <= 0)
		return 0;
	sc->ufcs_phy.map = devm_regmap_init_i2c(sc->client, &config);
	if (IS_ERR(sc->ufcs_phy.map))
		return PTR_ERR(sc->ufcs_phy.map);
	sc->ufcs_phy.ctx = sc;
	sc->ufcs_phy.prepare = sc8547_ufcs_prepare;
	sc->ufcs = sc8547a_ufcs_register(sc->dev, &sc->ufcs_phy);
	if (IS_ERR(sc->ufcs))
		return PTR_ERR(sc->ufcs);
	return devm_add_action_or_reset(sc->dev, sc8547_ufcs_unregister, sc->ufcs);
}

/* Stop the pumps before disabling the PHY watchdog or releasing D+/D-. */
static int sc8547_ufcs_end(struct sc8547a_device *sc)
{
	int ret;

	if (!sc || !sc->ufcs_owner)
		return 0;
	scoped_guard(mutex, &sc8547_pps_lock) {
		ret = sc8547_pps_disengage();
		if (ret)
			return ret;
		pps_cp_engaged = false;
		pps_adc_hold = false;
	}
	if (!sc->ufcs_exit_at) {
		ret = ufcs_force_exit(sc->ufcs);
		if (ret && sc->ufcs_watchdog_set) {
			sc->ufcs_exit_at = (jiffies + 3 * HZ + 1) ?: 1;
			return ret;
		}
	}
	if (sc->ufcs_exit_at && time_before(jiffies, sc->ufcs_exit_at))
		return -EAGAIN;
	/* A successful EXIT reply may precede the change at the connector.
	 * Retain the route while a live source still supplies a high VBUS.
	 */
	ret = erhai_otg_adsp_usb_prop(0);
	if (ret < 0)
		return ret;
	if (ret) {
		ret = erhai_otg_adsp_usb_prop(1);
		if (ret < 0 || ret > 5500000)
			return ret < 0 ? ret : -EAGAIN;
	}
	guard(mutex)(&sc8547_pps_lock);
	/* Converters are off, VBUS is safe. Reset is best effort; the
	 * independent crossbar physically isolates an unresponsive PHY.
	 */
	sc8547_reset_voocphy(sc);
	ret = wcd939x_usbss_dpdm_switch_fast_charge(false);
	if (!ret) {
		sc->ufcs_owner = false;
		sc->ufcs_watchdog_set = false;
		sc->ufcs_exit_at = 0;
	}
	return ret;
}

int sc8547a_ufcs_stop(void)
{
	guard(mutex)(&sc8547_ufcs_lock);
	return sc8547_ufcs_end(sc8547_master_dev);
}

/* oplus_chg_ufcs.c uses UFCS_VERIFY_CURR_THR_MA for unverified
 * Oplus sources. Standard PDOs and cable limits apply in both modes.
 * A transport error requires a new negotiation.
 */
#define SC8547_UFCS_UNVERIFIED_MA	4000

static int sc8547_ufcs_authenticate(struct sc8547a_device *sc)
{
	const struct firmware *key;
	u64 info;
	int ret;

	ret = ufcs_intf_get_device_info(sc->ufcs, &info);
	if (ret)
		return ret;
	if (UFCS_DEVICE_INFO_IC_VENDOR(info) ||
	    UFCS_DEVICE_INFO_DEV_VENDOR(info) != UFCS_OPLUS_DEV_ID)
		return INT_MAX;
	ret = request_firmware_direct(&key, "oneplus/erhai/ufcs-auth.bin", sc->dev);
	if (ret)
		return ret == -ENOENT ? SC8547_UFCS_UNVERIFIED_MA : ret;
	if (key->size == UFCS_VERIFY_AUTH_DATA_SIZE)
		ret = ufcs_intf_verify_adapter(sc->ufcs, 1, (u8 *)key->data, key->size);
	else
		ret = 0;
	release_firmware(key);
	if (ret < 0)
		return ret;
	return ret == 1 ? INT_MAX : SC8547_UFCS_UNVERIFIED_MA;
}

/* Stock oplus_ufcs_get_emark_ability: UFCS_EMARK_POWER_DEFAULT
 * is 4 A for an unidentified cable. An explicit cable rating wins.
 * The default is used only inside our board's <=9.9 V operating range.
 */
static int sc8547_ufcs_cable_limit(int rc, u64 cable, unsigned int mv)
{
	if (rc < 0)
		return 4000;
	if (UFCS_CABLE_INFO_VOL_MAX(cable) < mv)
		return 0;
	return UFCS_CABLE_INFO_CURR_MAX(cable);
}

int sc8547a_ufcs_start(unsigned int min_mv, unsigned int max_mv)
{
	u64 modes[UFCS_OUTPUT_MODE_MAX], cable = 0;
	int count, ret, i, cable_ma, source_ma, best = 0;
	struct sc8547a_device *sc;

	guard(mutex)(&sc8547_ufcs_lock);
	sc = sc8547_master_dev;
	if (!sc || !sc->ufcs)
		return -EOPNOTSUPP;
	scoped_guard(mutex, &sc8547_pps_lock) {
		if (!sc8547_btb_allows(&sc->btb))
			return -EAGAIN;
		if (sc->vooc_started || pps_cp_engaged || pps_adc_hold || sc->ufcs_owner)
			return -EBUSY;
		sc->ufcs_owner = true;
		ret = wcd939x_usbss_dpdm_switch_fast_charge(true);
	}
	if (ret)
		goto out;
	ret = ufcs_handshake(sc->ufcs);
	scoped_guard(mutex, &sc8547_pps_lock) {
		sc->first_ufcs_timeout = !sc->ufcs_handshake_tried && ret == -ETIMEDOUT &&
			READ_ONCE(sc->ufcs->handshake_state) == UFCS_HS_TIMEOUT;
		sc->ufcs_handshake_tried = true;
		if (!ret)
			sc->responded |= ERHAI_UFCS;
	}
	if (ret)
		goto out;
	/* Stock sets the CP watchdog to 5 s after PHY handshake. The
	 * source packet watchdog is a separate 3 s contract.
	 */
	ret = sc->ufcs->ops->watchdog_config(sc->ufcs, 5000);
	if (ret)
		goto out;
	ret = ufcs_intf_config_watchdog(sc->ufcs, 3000);
	if (ret)
		goto out;
	sc->ufcs_watchdog_set = true;
	count = ufcs_intf_get_pdo_info(sc->ufcs, modes, ARRAY_SIZE(modes));
	if (count <= 0) {
		ret = count ?: -ENODATA;
		goto out;
	}
	/* Standard, non-proprietary UFCS modes. A larger cable budget
	 * requires a successful cable information exchange.
	 */
	ret = ufcs_intf_get_cable_info(sc->ufcs, &cable);
	cable_ma = sc8547_ufcs_cable_limit(ret, cable, max_mv);
	if (!cable_ma || !ufcs_handshake_success(sc->ufcs)) {
		ret = -ENOLINK;
		goto out;
	}
	source_ma = sc8547_ufcs_authenticate(sc);
	if (source_ma < 0) {
		ret = source_ma;
		goto out;
	}
	for (i = 0; i < count; i++) {
		int ma = min_t(int, UFCS_OUTPUT_MODE_CURR_MAX(modes[i]), cable_ma);

		ma = min(ma, source_ma);
		if (UFCS_OUTPUT_MODE_CURR_MIN(modes[i]) > ma)
			continue;

		if (UFCS_OUTPUT_MODE_VOL_MIN(modes[i]) > min_mv ||
		    UFCS_OUTPUT_MODE_VOL_MAX(modes[i]) < max_mv ||
		    ma <= best)
			continue;
		sc->ufcs_mode = modes[i];
		best = ma;
	}
	if (best) {
		sc->ufcs_cable_ma = min(cable_ma, source_ma);
		sc->ufcs_granted_ma = 0;
		sc->ufcs_packet_at = jiffies;
		return best;
	}
	ret = -ENODATA;
out:
	return sc8547_ufcs_end(sc) ?: ret;
}

int sc8547a_ufcs_request(unsigned int *mv, unsigned int *ma)
{
	struct sc8547a_device *sc;
	u64 mode;
	unsigned int base_mv, base_ma;
	int applied, ret;

	guard(mutex)(&sc8547_ufcs_lock);
	sc = sc8547_master_dev;
	if (!sc || !sc->ufcs_owner || !ufcs_handshake_success(sc->ufcs))
		return -ENOLINK;
	mode = sc->ufcs_mode;
	base_mv = UFCS_OUTPUT_MODE_VOL_MIN(mode);
	base_ma = UFCS_OUTPUT_MODE_CURR_MIN(mode);
	if (*mv < base_mv || *ma < base_ma)
		return -ERANGE;
	*mv -= (*mv - base_mv) % UFCS_OUTPUT_MODE_VOL_STEP(mode);
	ret = ufcs_request_limited(sc->ufcs, *mv, sc->ufcs_cable_ma,
				   8000, *ma, &applied);
	if (ret)
		return ret;
	*ma = applied;
	sc->ufcs_granted_ma = applied;
	sc->ufcs_packet_at = jiffies;
	return 0;
}
/* Source watchdog is set to 3 s; monitor the live contract before
 * that deadline even when the servo has no voltage change to request.
 */
int sc8547a_ufcs_monitor(void)
{
	struct sc8547a_device *sc;
	u32 changes[UFCS_OUTPUT_MODE_MAX];
	u64 info;
	int i, ret;
	u8 control;

	guard(mutex)(&sc8547_ufcs_lock);
	sc = sc8547_master_dev;
	if (!sc || !sc->ufcs_owner || !ufcs_handshake_success(sc->ufcs))
		return -ENOLINK;
	/* oplus_hal_sc8547a.c: CP_WATCHDOG_RESET reads REG07. Service
	 * the PHY even after a recoverable drop has returned load to buck.
	 * A still-running 2:1 pump retains its tighter 1 s watchdog.
	 */
	ret = sc8547_read_byte(sc, SC8547_REG_07, &control);
	if (ret)
		return ret;
	ret = ufcs_intf_get_power_change_info(sc->ufcs, changes, ARRAY_SIZE(changes));
	if (ret < 0)
		return ret;
	for (i = 0; i < ret; i++) {
		if (UFCS_POWER_CHANGE_INDEX(changes[i]) == UFCS_OUTPUT_MODE_INDEX(sc->ufcs_mode) &&
		    UFCS_POWER_CHANGE_CURR_MAX(changes[i]) < sc->ufcs_granted_ma)
			return -EAGAIN;
	}
	if (time_before(jiffies, sc->ufcs_packet_at + HZ))
		return 0;
	ret = ufcs_intf_get_source_info(sc->ufcs, &info);
	if (!ret)
		sc->ufcs_packet_at = jiffies;
	return ret;
}
#else
static int sc8547_ufcs_init(struct sc8547a_device *sc) { return 0; }
int sc8547a_ufcs_start(unsigned int min_mv, unsigned int max_mv) { return -EOPNOTSUPP; }
int sc8547a_ufcs_request(unsigned int *mv, unsigned int *ma) { return -EOPNOTSUPP; }
int sc8547a_ufcs_stop(void) { return 0; }
int sc8547a_ufcs_monitor(void) { return -EOPNOTSUPP; }
#endif
EXPORT_SYMBOL_GPL(sc8547a_ufcs_start);
EXPORT_SYMBOL_GPL(sc8547a_ufcs_request);
EXPORT_SYMBOL_GPL(sc8547a_ufcs_stop);
EXPORT_SYMBOL_GPL(sc8547a_ufcs_monitor);

/* ------------------------------- IRQ ---------------------------------- */

static irqreturn_t sc8547_irq_arrival(int irq, void *data)
{
	struct sc8547a_device *sc = data;

	WRITE_ONCE(sc->irq_arrived_ns, ktime_get_ns());
	smp_store_release(&sc->irq_pending, true);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t sc8547_irq_handler(int irq, void *data)
{
	struct sc8547a_device *sc = data;
	u8 state = 0;

	/* IRQF_ONESHOT keeps this edge stable until the thread finishes. */
	guard(mutex)(&sc8547_pps_lock);
	WRITE_ONCE(sc->irq_pending, false);
	if (sc8547_waiting_for_off())
		return IRQ_HANDLED;
	sc8547_collect_faults(sc);
#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
	if (sc->ufcs_owner) {
		sc8547a_ufcs_irq(sc->ufcs);
		return IRQ_HANDLED;
	}
#endif
	if (sc->vooc_started && sc->auto_armed &&
	    (sc->auto_state == SC_AUTO_IDENTIFY || sc->auto_state == SC_AUTO_DELIVER)) {
		if (sc8547_voocphy_isr(sc, READ_ONCE(sc->irq_arrived_ns)))
			return IRQ_HANDLED;
	}

	sc8547_read_byte(sc, SC8547_REG_06, &state);
	dev_dbg(sc->dev, "irq: int_flag=0x%02x status=0x%02x\n", sc->pump_faults, state);
	return IRQ_HANDLED;
}

/* ------------------------------ debugfs ------------------------------- */

static int sc8547_dump_show(struct seq_file *m, void *unused)
{
	struct sc8547a_device *sc = m->private;
	u8 val;
	int addr;

	for (addr = 0x00; addr <= 0x3C; addr++) {
		/*
		 * 0x0f is the interrupt flag register and 0x2e/0x2f are the
		 * VOOC RX buffer: a plain read clears the flag and consumes a
		 * pending frame, which desynchronises the ladder while a
		 * session is running.  sc8547a_pps_regs() already refuses to
		 * touch them for the same reason; a debug dump must not be a
		 * way around that.
		 */
		if (addr == 0x0f || addr == 0x2e || addr == 0x2f) {
			seq_printf(m, "reg[0x%02x] = (not read: read-clear/RX)\n",
				   addr);
			continue;
		}
		if (sc8547_read_byte(sc, addr, &val) == 0)
			seq_printf(m, "reg[0x%02x] = 0x%02x\n", addr, val);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc8547_dump);

static int sc8547_adc_show(struct seq_file *m, void *unused)
{
	struct sc8547a_device *sc = m->private;

	seq_printf(m, "vbus_mV %d\nibus_mA %d\nvbat_mV %d\n",
		   sc8547_get_vbus(sc), sc8547_get_ibus(sc), sc8547_get_vbat(sc));
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc8547_adc);

/* Read-only auto/fast-charge status for userspace. */
static int sc8547_state_show(struct seq_file *m, void *unused)
{
	struct sc8547a_device *sc = m->private;
	guard(mutex)(&sc8547_pps_lock);

	seq_printf(m, "auto_armed %d state %s code %u reservo %d/%d ident_retry %d/%u vbus %d mV vbat %d mV ibus %d mA (master %d slave %d) cp %s\n",
		   sc->fastchg_auto, serial_state_names[sc->auto_state], sc->fastchg_curr_code,
		   sc->auto_reservo_cnt, SC8547_AUTO_RESERVO_MAX,
		   sc->auto_ident_retries, identify_retries,
		   sc8547_get_vbus(sc), sc8547_get_vbat(sc),
		   sc8547_get_ibus(sc) + sc8547_slave_ibus(),
		   sc8547_get_ibus(sc), sc8547_slave_ibus(),
		   sc8547_slave_dev ? "dual" : "single");
	if (sc->auto_state == SC_AUTO_DELIVER && sc->auto_curve)
		seq_printf(m, "curve idx %d/%d target_vbat %d code %u\n",
			   sc->auto_curve_idx, sc->auto_curve->count - 1,
			   sc->auto_curve->steps[sc->auto_curve_idx].mv,
			   sc->auto_curve->steps[sc->auto_curve_idx].ma / 100);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc8547_state);

static void sc8547_create_debugfs(struct sc8547a_device *sc)
{
	sc->debug_root = debugfs_create_dir(dev_name(sc->dev), NULL);
	debugfs_create_file("registers", 0444, sc->debug_root, sc,
			    &sc8547_dump_fops);
	debugfs_create_file("adc", 0444, sc->debug_root, sc, &sc8547_adc_fops);
	if (!sc->is_slave)
		debugfs_create_file("fastchg_state", 0444, sc->debug_root, sc,
				    &sc8547_state_fops);
}

/* ------------------------------- probe -------------------------------- */

static int sc8547a_probe(struct i2c_client *client)
{
	struct sc8547a_device *sc;
	int ret;
	u32 threshold;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return dev_err_probe(&client->dev, -ENODEV,
				     "required SMBus functions unsupported\n");

	sc = devm_kzalloc(&client->dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->client = client;
	sc->dev = &client->dev;
	sc->irq = client->irq;
	sc->is_slave = device_property_read_bool(sc->dev, "southchip,secondary") ||
		       device_property_read_bool(sc->dev, "oplus,slave");
	sc->fastchg_curr_code = SC8547_CURR_CODE_MIN;
	sc->ovp_reg = SC8547_DEFAULT_OVP_REG;
	sc->ocp_reg = SC8547_DEFAULT_OCP_REG;
	device_property_read_u8(sc->dev, "sc,vbat-ovp-reg", &sc->ovp_reg);
	device_property_read_u8(sc->dev, "sc,ibus-ocp-reg", &sc->ocp_reg);
	if (!device_property_read_u32(sc->dev, "southchip,battery-overvoltage-microvolt",
				      &threshold)) {
		if (threshold < 3500000 || threshold > 5075000 || threshold % 25000)
			return -EINVAL;
		sc->ovp_reg = (threshold - 3500000) / 25000;
	}
	if (!device_property_read_u32(sc->dev, "southchip,input-overcurrent-microamp",
				      &threshold)) {
		if (threshold < 1200000 || threshold > 5700000 || threshold % 300000)
			return -EINVAL;
		sc->ocp_reg = (threshold - 1200000) / 300000;
	}
	if (!device_property_read_u32(sc->dev, "southchip,ac-overvoltage-microvolt",
				      &threshold) && threshold != 12000000)
		return -EINVAL;
	if (!device_property_read_u32(sc->dev, "southchip,input-overvoltage-microvolt",
				      &threshold) && threshold != 10000000)
		return -EINVAL;
	if (!sc->is_slave) {
		sc->btb.battery = devm_iio_channel_get(sc->dev, "battery-connector");
		if (IS_ERR(sc->btb.battery))
			return dev_err_probe(sc->dev, PTR_ERR(sc->btb.battery), "battery connector ADC\n");
		sc->btb.usb = devm_iio_channel_get(sc->dev, "usb-connector");
		if (IS_ERR(sc->btb.usb))
			return dev_err_probe(sc->dev, PTR_ERR(sc->btb.usb), "USB connector ADC\n");
		sc->btb.battery_err = sc->btb.usb_err = -ENODATA;
		INIT_DELAYED_WORK(&sc->btb.work, sc8547_btb_work);
	}
	mutex_init(&sc->i2c_lock);
	INIT_DELAYED_WORK(&sc->attach_work, sc8547_attach_work);
	i2c_set_clientdata(client, sc);

	if (!sc8547_check_device_id(sc))
		return dev_err_probe(sc->dev, -ENODEV,
				     "SC8547A device id mismatch (reg 0x36)\n");

	ret = sc8547_hardware_init(sc);
	if (ret)
		return dev_err_probe(sc->dev, ret, "hardware init failed\n");

	ret = sc8547_ufcs_init(sc);
	if (ret)
		return dev_err_probe(sc->dev, ret, "UFCS PHY registration failed\n");

	if (sc->irq > 0) {
		ret = devm_request_threaded_irq(sc->dev, sc->irq, sc8547_irq_arrival,
						sc8547_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						dev_name(sc->dev), sc);
		if (ret)
			return dev_err_probe(sc->dev, ret,
					     "failed to request irq\n");
	}

	sc8547_create_debugfs(sc);

	guard(mutex)(&sc8547_ufcs_lock);
	mutex_lock(&sc8547_pps_lock);
	if (sc->is_slave)
		sc8547_slave_dev = sc;	/* master delivery path drives it */
	else
		sc8547_master_dev = sc;
	mutex_unlock(&sc8547_pps_lock);

	dev_info(sc->dev, "SC8547A %s charge pump probed (addr 0x%02x)\n",
		 sc->is_slave ? "slave" : "master", client->addr);
	return 0;
}

static void sc8547a_remove(struct i2c_client *client)
{
	struct sc8547a_device *sc = i2c_get_clientdata(client);

	if (!sc->is_slave)
		disable_delayed_work_sync(&sc->btb.work);
	sc8547a_serial_session(false);
	cancel_delayed_work_sync(&sc->attach_work);
	debugfs_remove_recursive(sc->debug_root);
	guard(mutex)(&sc8547_ufcs_lock);
#if IS_REACHABLE(CONFIG_OPLUS_UFCS_CLASS)
	if (!sc->is_slave)
		sc8547_ufcs_end(sc);
#endif
	guard(mutex)(&sc8547_pps_lock);
	sc8547_reset_voocphy(sc);
	if (sc->is_slave) {
		pps_slave_engaged = false;
		sc8547_slave_dev = NULL;
	} else {
		sc8547_pps_slave_off();
		sc8547_master_dev = NULL;
		pps_cp_engaged = false;
		pps_adc_hold = false;
	}
}

static int sc8547a_suspend(struct device *dev)
{
	struct sc8547a_device *sc = dev_get_drvdata(dev);

	int ret = sc8547a_ufcs_stop() ?: sc8547a_serial_session(false);

	if (ret)
		return ret;
	cancel_delayed_work_sync(&sc->attach_work);
	guard(mutex)(&sc8547_pps_lock);
	if (pps_slave_engaged && sc8547_pps_slave_off())
		return -EIO;
	if (!sc->is_slave) {
		sc8547_btb_offline(&sc->btb);
		sc->auto_state = SC_AUTO_OFF;
		sc->fastchg_deliver = false;
		sc->fastchg_enable = false;
		pps_adc_hold = false;
		pps_cp_engaged = false;
	}
	return sc8547_reset_voocphy(sc);
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc8547a_pm_ops, sc8547a_suspend, NULL);

static void sc8547a_shutdown(struct i2c_client *client)
{
	sc8547a_suspend(&client->dev);
}

static const struct of_device_id sc8547a_of_match[] = {
	{ .compatible = "oplus,sc8547a" },
	{ .compatible = "southchip,sc8547a" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc8547a_of_match);

static const struct i2c_device_id sc8547a_i2c_id[] = {
	{ "sc8547a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc8547a_i2c_id);

static struct i2c_driver sc8547a_driver = {
	.driver = {
		.name = "sc8547a",
		.of_match_table = sc8547a_of_match,
		.pm = pm_sleep_ptr(&sc8547a_pm_ops),
	},
	.probe = sc8547a_probe,
	.remove = sc8547a_remove,
	.shutdown = sc8547a_shutdown,
	.id_table = sc8547a_i2c_id,
};
module_i2c_driver(sc8547a_driver);

MODULE_FIRMWARE("oneplus/erhai/ufcs-auth.bin");
MODULE_DESCRIPTION("Southchip SC8547A charge pump / VOOC PHY driver");
MODULE_LICENSE("GPL");
