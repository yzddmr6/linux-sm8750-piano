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
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/usb/wcd939x-usbss.h>

#include "sc8547a.h"

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
 *     "erhai-adsp-bat" (registered by erhai_otg_glink.c; on a board whose
 *     bq27z561 ACKs but never reports status the OTG driver detaches the dead
 *     pack gauge, so only the ADSP psy remains).
 * The list is tried in order and the first gauge that ANSWERS is used, so a
 * gauge-absent board no longer declines with "gauge not ready" (which would
 * disable SVOOC entirely there).  Same convention as erhai_pps.c
 * (pps_bat_psy()).  gauge_name is an explicit override for bring-up/debug.
 */
static char *gauge_name;
module_param(gauge_name, charp, 0644);
MODULE_PARM_DESC(gauge_name,
		 "battery gauge power-supply name to use first (default: bq27z561-0 then erhai-adsp-bat)");

static const char * const sc8547_gauge_names[] = {
	"bq27z561-0",		/* pack-gauge SKUs: TI bq27z561 (AP I2C) */
	"erhai-adsp-bat",	/* QBG SKUs: gauge lives in the ADSP */
};

#include "erhai_otg_glink.h"
#include "sc8547a.h"

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

/*
 * Requested constant-current level: the 7-bit ask-current code the adapter
 * reads as its CC setpoint. Android ground truth: code 73 <-> target_ibus
 * 7300 mA, i.e. code = ibus(bus-side) / 100 mA. The CP is divide-by-2, so
 * battery-side current is ~2x the bus-side ibus. A useful CC setpoint must
 * exceed the passive (vbus-2*vbat)/R bleed, else the adapter backs vbus off,
 * the margin collapses as vbat rises, VBUS_ERRORLO fires and CHG_EN self-
 * clears. Knob range [MIN,MAX]; MAX bounds the bus-side current under the
 * 6000 mA IBUS backstop. Default MIN keeps a bare arm near-zero.
 */
#define SC8547_CURR_CODE_MIN		0x01	/* 100 mA bus (~0.2 A pack) */
#define SC8547_CURR_CODE_MAX		0x49	/* 7300 mA bus (full SVOOC rung 73) */

/*
 * Safety backstop while the CP is delivering (stage-2 only).
 *
 * VOLTAGE REFERENCE: the CP's own Vbat ADC sits on the pack's input rail, so
 * under delivery it reads the CELL voltage PLUS the IR drop of the
 * harness/pack path (~0.3 V at the 7-8 A this pad pulls: measured 2026-09-28,
 * 4.49 V on the CP vs 4.20 V on the gauge, while at rest both read 3920 mV).
 * Judging the CC/CV curve (and the CV end) on that loaded reading aborted every
 * SVOOC session 15-30 s in, at ~50% SoC.  The battery gauge reports the CELL
 * voltage — "bq27z561-0" on the pack-gauge SKUs, "erhai-adsp-bat" on the QBG
 * population (see sc8547_gauge_names[]) — so it is the reference for the curve
 * steps, the start gate and the CV ceiling; the CP reading is kept only as an
 * over-voltage guard, set well above any legitimate loaded reading (the highest
 * seen is ~4.49 V).
 */
#define SC8547_VBAT_CV_CEILING_MV	4500	/* cell (gauge) CV end */
#define SC8547_CP_OVP_CEILING_MV	4800	/* CP-side hard guard: cell CV (4.5 V)
						 * plus the measured ~0.3 V IR drop
						 * at 8 A; a wilder reading than this
						 * is a genuine over-voltage. */
#define SC8547_IBUS_ABORT_MA		8000	/* total-bus (master+slave) hard abort; > code-73 7300 mA */
#define SC8547_TEMP_ABORT_DDC		450	/* pack temp 45.0 C -> stop (0.1 C) */
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
#define SC8547_SLAVE_ENGAGE_IBUS_MA	1500	/* master half this high -> close slave */
#define SC8547_SLAVE_DISENGAGE_IBUS_MA	1200	/* total bus below this -> drop slave */

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

/*
 * Auto charge-manager (fastchg_auto) policy bounds. On charger attach the auto
 * path arms the PHY in observation, waits for the adapter to identify as a
 * genuine (S)VOOC adapter, checks battery preconditions, then steps the CC
 * ask-current code down the Android host curve as vbat rises. Every step is
 * clamped to SC8547_AUTO_CODE_CEILING. Now the full curve max (code 73 =
 * 7300 mA bus, the Android SVOOC peak rung) so the auto path delivers full
 * ~88 W SuperVOOC at low SoC. Safe because the dual charge-pump shares the bus
 * ~50/50 (each pump ~3.65 A, under its proven solo capability, validated to
 * 5.9 A bus with pack temp flat), and delivery is bounded by the CELL 4500 mV CV
 * ceiling, the CP-side 4800 mV over-voltage guard
 * (SC8547_CP_OVP_CEILING_MV), the 8000 mA total-bus
 * over-current abort, and the 45 C pack-temp backstop. Auto delivery only ever
 * descends the curve.
 *
 * NOTE (2026-09-28): "vbat" everywhere in this policy means the CELL voltage,
 * i.e. the gauge reading — NOT the charge pump's own Vbat ADC, which is
 * inflated by the delivery IR drop.  See the backstop comment above.
 */
#define SC8547_AUTO_CODE_CEILING	0x49	/* full curve: code 73 = 7300 mA bus */
#define SC8547_MAX_SOC_DEFAULT		80	/* stop fast-charge at this SoC % (longevity) */

#define SC8547_AUTO_IDENTIFY_MS		4000	/* window to see adapter_check_ok */
#define SC8547_AUTO_TEMP_MIN_DDC	150	/* 15.0 C: below -> normal charge */
#define SC8547_AUTO_TEMP_MAX_DDC	440	/* 44.0 C: above -> normal charge */
#define SC8547_AUTO_TEMP_NL_NH_DDC	350	/* <350 -> normal_low col, else _high */
#define SC8547_AUTO_STEP_DEBOUNCE	4	/* consecutive ticks (~1 s) to advance */
#define SC8547_AUTO_STEP_I_HYST_MA	200	/* ibus below setpoint by this -> step */
#define SC8547_AUTO_STEP_V_HYST_MV	10	/* vbat over target by this -> step */
/*
 * A genuine SVOOC adapter periodically resets its VOOC link (drops output and
 * waits for a fresh handshake) after tens of seconds of CC. That is not an
 * unplug: re-arm the PHY and re-servo (matches the vendor engine) rather than
 * ending fast-charge. Bound consecutive re-servos so a genuinely failing adapter
 * still falls out to normal charge; a re-servo that follows a sustained CC period
 * refills the budget, so healthy periodic resets never exhaust it.
 */
#define SC8547_AUTO_RESERVO_MAX		4	/* consecutive quick resets before give-up */
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

/* One host CC/CV curve row: {target_vbat mV, ask-current code, exit}. */
struct sc8547_curve_row {
	u16	target_vbat;
	u8	code;		/* bus-side ibus / 100 mA */
	u8	exit;
};

struct sc8547a_device {
	struct i2c_client	*client;
	struct device		*dev;
	struct mutex		i2c_lock;	/* serialises multi-op RMW */
	struct dentry		*debug_root;
	int			irq;
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
	bool			vbus_was_present;/* attach/detach edge tracking (debounced) */
	unsigned long		vbus_absent_since;/* jiffies of first absent poll, 0 = present */
	bool			auto_armed;	/* crossbar+PHY armed for this plug */
	unsigned long		arm_at;		/* jiffies: settle deadline post-attach */
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
	bool			adapter_check_ok;
	u8			ask_batt_sys;

	/* --- auto charge-manager (fastchg_auto) --- */
	bool			fastchg_auto;	/* one-shot master arm (kill-switch) */
	int			max_soc;	/* stop fast-charge at this SoC % (default 80) */
	enum sc8547_auto_state	auto_state;
	unsigned long		auto_identify_deadline;
	const struct sc8547_curve_row *auto_curve;	/* selected CC/CV table */
	int			auto_curve_len;
	int			auto_curve_idx;	/* current row (descends only) */
	int			auto_step_cnt;	/* advance debounce counter */
	int			auto_reservo_cnt;	/* consecutive adapter-reset re-servos */
	int			auto_ident_retries;	/* identify re-arms on this plug */
	bool			vooc_saw_any;	/* any decodable VOOC frame this attempt */
	unsigned long		auto_deliver_since;	/* jiffies of last (re)escalation */
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
	struct sc8547a_device *sc = sc8547_master_dev;

	return sc && (sc->auto_state == SC_AUTO_IDENTIFY ||
		      sc->auto_state == SC_AUTO_DELIVER);
}
EXPORT_SYMBOL_GPL(sc8547a_fastchg_identifying);

/* ------------------------- register accessors ------------------------- */

static int sc8547_read_byte(struct sc8547a_device *sc, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(sc->client, reg);

	if (ret < 0) {
		dev_err(sc->dev, "read reg 0x%02x failed: %d\n", reg, ret);
		return ret;
	}
	*val = ret;
	return 0;
}

static int sc8547_write_byte(struct sc8547a_device *sc, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(sc->client, reg, val);

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
	int ret = i2c_smbus_read_word_data(sc->client, reg);

	if (ret < 0)
		dev_err(sc->dev, "read word reg 0x%02x failed: %d\n", reg, ret);
	return ret;
}

static int sc8547_write_word(struct sc8547a_device *sc, u8 reg, u16 val)
{
	int ret = i2c_smbus_write_word_data(sc->client, reg, val);

	if (ret < 0)
		dev_err(sc->dev, "write word 0x%04x to reg 0x%02x failed: %d\n",
			val, reg, ret);
	return ret;
}

/* ------------------------- charge-pump control ------------------------ */

static int sc8547_reg_reset(struct sc8547a_device *sc)
{
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

static int sc8547_set_chg_enable(struct sc8547a_device *sc, bool enable)
{
	return sc8547_write_byte(sc, SC8547_REG_07, enable ? 0x80 : 0x00);
}

static int sc8547_get_chg_enable(struct sc8547a_device *sc, bool *enabled)
{
	u8 val = 0;
	int ret = sc8547_read_byte(sc, SC8547_REG_07, &val);

	if (ret)
		return ret;
	*enabled = !!(val & SC8547_CHG_EN_MASK);
	return 0;
}

static int sc8547_set_charge_mode(struct sc8547a_device *sc, bool half)
{
	return sc8547_update_bits(sc, SC8547_REG_09, SC8547_CHARGE_MODE_MASK,
				  half ? SC8547_CHARGE_MODE_MASK : 0);
}

static int sc8547_set_adc_enable(struct sc8547a_device *sc, bool enable)
{
	return sc8547_write_byte(sc, SC8547_REG_11, enable ? 0x80 : 0x00);
}

/* Returns mV, or negative errno. */
static int sc8547_get_vbus(struct sc8547a_device *sc)
{
	u8 buf[2];
	int ret = i2c_smbus_read_i2c_block_data(sc->client, SC8547_REG_15, 2, buf);

	if (ret < 0)
		return ret;
	return (((buf[0] & SC8547_VBUS_POL_H_MASK) << 8) | buf[1]) *
	       SC8547_VBUS_ADC_LSB_NUM / SC8547_VBUS_ADC_LSB_DEN;
}

/* Returns mA, or negative errno. Only valid while the CP is switching. */
static int sc8547_get_ibus(struct sc8547a_device *sc)
{
	u8 buf[2];
	int ret = i2c_smbus_read_i2c_block_data(sc->client, SC8547_REG_13, 2, buf);

	if (ret < 0)
		return ret;
	return ((buf[0] << 8) | buf[1]) *
	       SC8547_IBUS_ADC_LSB_NUM / SC8547_IBUS_ADC_LSB_DEN;
}

/* Returns mV, or negative errno. */
static int sc8547_get_vbat(struct sc8547a_device *sc)
{
	u8 buf[2];
	int ret = i2c_smbus_read_i2c_block_data(sc->client, SC8547_REG_1B, 2, buf);

	if (ret < 0)
		return ret;
	return ((buf[0] << 8) | buf[1]) *
	       SC8547_VBAT_ADC_LSB_NUM / SC8547_VBAT_ADC_LSB_DEN;
}

/* --------------------- VOOC serial-protocol primitives ---------------- */

/*
 * Register-level API consumed by the VOOC adapter-identify state machine below.
 */
/* Pre-stage a reply word into the predata register (REG_31); see the is_vbus_ok
 * predata note above. Written raw, matching the vendor. */
static int sc8547_set_predata(struct sc8547a_device *sc, u16 val)
{
	return sc8547_write_word(sc, SC8547_REG_31, val);
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

static void sc8547_send_handshake(struct sc8547a_device *sc)
{
	sc8547_write_byte(sc, SC8547_REG_2B, 0x81);
}

/* Base hardware config, applied at probe and on reset. */
static int sc8547_init_device(struct sc8547a_device *sc)
{
	sc8547_write_byte(sc, SC8547_REG_11, 0x00);	/* ADC disable */
	sc8547_write_byte(sc, SC8547_REG_02, 0x01);	/* VAC_OVP 12V */
	sc8547_write_byte(sc, SC8547_REG_04, 0x50);	/* VBUS_OVP 10V */
	sc8547_write_byte(sc, SC8547_REG_00, 0x20 | (sc->ovp_reg & 0x1f)); /* VBAT_OVP */
	sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f)); /* IBUS_OCP/UCP */
	sc8547_write_byte(sc, SC8547_REG_01, 0xbf);	/* BAT_OCP */
	sc8547_write_byte(sc, SC8547_REG_2B, 0x00);	/* VOOC disable */
	sc8547_write_byte(sc, SC8547_REG_3A, 0x60);
	sc8547_update_bits(sc, SC8547_REG_09, BIT(4), BIT(4)); /* IBUS_UCP_RISE mask */
	sc8547_write_byte(sc, SC8547_REG_10, 0x02);	/* mask insert irq */
	sc8547_write_byte(sc, SC8547A_REG_OTG_EN, 0x04);
	return 0;
}

/* Full reset used at probe (reg_reset + base config). */
static int sc8547_hardware_init(struct sc8547a_device *sc)
{
	sc8547_reg_reset(sc);
	return sc8547_init_device(sc);
}

/*
 * Arm the VOOC PHY: reset, base config, DPDM setup, enable the VOOC engine.
 * Mirrors vendor sc8547_init_vooc(). After this the chip runs the D+/D-
 * serial handshake; the protocol engine drives predata/txbuff/rxbuff.
 */
static int sc8547_init_vooc(struct sc8547a_device *sc)
{
	u8 value = 0;

	sc8547_reg_reset(sc);
	sc8547_init_device(sc);

	sc8547_write_word(sc, SC8547_REG_31, 0x0);	/* predata */
	sc8547_read_byte(sc, SC8547_REG_3A, &value);
	value |= BIT(6);
	sc8547_write_byte(sc, SC8547_REG_3A, value);

	sc8547_write_byte(sc, SC8547_REG_21, 0x21);	/* DPDM */
	sc8547_write_byte(sc, SC8547_REG_22, 0x00);
	sc8547_write_byte(sc, SC8547_REG_33, 0xD1);	/* loose_det */

	sc8547_write_byte(sc, SC8547_REG_30, 0x05);	/* VOOC irq enable */
	return 0;
}

/* SVOOC constant-current hardware profile (vendor sc8547_svooc_hw_setting). */
static int sc8547_svooc_hw_setting(struct sc8547a_device *sc)
{
	sc8547_write_byte(sc, SC8547_REG_02, 0x01);	/* VAC_OVP 12V */
	sc8547_write_byte(sc, SC8547_REG_04, 0x50);	/* VBUS_OVP 10V */
	sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	/* watchdog (REG_09) deliberately NOT enabled here: this profile is applied
	 * at arm-time, before the VOOC servo, so enabling the 1000ms watchdog now
	 * (with the CP not yet feeding it) would expire mid-servo. The watchdog is
	 * enabled together with the CP at is_vbus-ok NORMAL, then fed by the poller. */
	sc8547_write_byte(sc, SC8547_REG_11, 0x80);	/* ADC enable */
	sc8547_write_byte(sc, SC8547_REG_0D, 0x70);
	sc8547_write_byte(sc, SC8547_REG_33, 0xd1);	/* loose_det */
	sc8547_write_byte(sc, SC8547_REG_3A, 0x60);
	/*
	 * Disable IBUS under-current protection + soft-start timeout for the SVOOC
	 * phase (vendor set_sstimeout_ucp_enable(false)). At CP-enable the current
	 * is near zero (Vbus only ~2*Vbat+150mV) and ramps only as the adapter then
	 * raises Vbus; with UCP (5ms) armed the converter trips off immediately,
	 * REG_07 self-clears, the adapter sees no load and aborts comm. OCP/OVP and
	 * the software vbat/ibus backstop still bound the delivery.
	 */
	sc8547_update_bits(sc, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			   SC8547_IBUS_UCP_DIS_MASK);		/* UCP disable */
	sc8547_update_bits(sc, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			   SC8547_SS_TIMEOUT_DISABLE);		/* SS timeout disable */
	return 0;
}

/* Tear down the VOOC PHY on unplug / fastchg exit. */
static int sc8547_reset_voocphy(struct sc8547a_device *sc)
{
	sc8547_write_byte(sc, SC8547_REG_07, 0x00);	/* CP off */
	sc8547_update_bits(sc, SC8547_REG_09, SC8547_WATCHDOG_MASK,
			   SC8547_WATCHDOG_DIS);
	sc8547_write_byte(sc, SC8547_REG_00, 0x20 | (sc->ovp_reg & 0x1f));
	sc8547_write_byte(sc, SC8547_REG_02, 0x01);
	sc8547_write_byte(sc, SC8547_REG_04, 0x50);
	sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	sc8547_write_byte(sc, SC8547_REG_11, 0x00);	/* ADC off */
	sc8547_write_byte(sc, SC8547_REG_2C, 0x00);	/* clear tx */
	sc8547_write_byte(sc, SC8547_REG_2D, 0x00);
	sc8547_write_byte(sc, SC8547_REG_30, 0x7f);	/* mask vooc irq */
	sc8547_write_byte(sc, SC8547_REG_21, 0xc0);	/* D+ HiZ */
	sc8547_write_byte(sc, SC8547_REG_2B, 0x00);	/* VOOC disable */
	sc8547_write_word(sc, SC8547_REG_31, 0x0);	/* predata */
	sc8547_write_byte(sc, SC8547_REG_10, 0x02);	/* mask insert irq */
	sc8547_write_byte(sc, SC8547A_REG_OTG_EN, 0x04);
	return 0;
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
 * Gated on the master instance; slave stays idle (single-CP delivery).  The
 * engine MUST call sc8547a_pps_cp_feed() more often than the 1000 ms watchdog
 * window or REG_07 self-clears and the CP drops out on its own (safety).
 */
static bool pps_cp_engaged;
static bool pps_adc_hold;	/* keep the ADC on across drops (PPS session) */
static bool sc8547_slave_is_on(void);	/* defined in the slave section below */

void sc8547a_pps_cp_disengage(void);	/* defined below, called by feed()'s drop path */

/* Snapshot the chip's own VBUS/VBAT/IBUS ADCs (V is upstream of the CP). */
int sc8547a_pps_snapshot(int *vbus_mv, int *vbat_mv, int *ibus_ma)
{
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
	return pps_cp_engaged;
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
	struct sc8547a_device *sc = sc8547_master_dev;
	int vbus, vbat, diff, target;

	if (!sc)
		return -ENODEV;
	if (sc8547a_fastchg_delivering() || sc8547_slave_is_on())
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

	sc8547_svooc_hw_setting(sc);
	sc8547_write_byte(sc, SC8547_REG_09, 0x13);	/* watchdog 1000 ms */
	sc8547_set_chg_enable(sc, true);
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
	struct sc8547a_device *sc = sc8547_master_dev;
	int vbus, vbat, diff;

	if (!sc || !pps_cp_engaged)
		return -ENODEV;
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
	    diff > SC8547_CP_RESTART_MARGIN_CEIL_MV)
		goto drop;
	sc8547_write_byte(sc, SC8547_REG_09, 0x13);
	sc8547_set_chg_enable(sc, true);
	return 0;
drop:
	sc8547a_pps_cp_disengage();
	return -EAGAIN;
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_feed);

void sc8547a_pps_cp_disengage(void)
{
	struct sc8547a_device *sc = sc8547_master_dev;

	pps_cp_engaged = false;
	if (!sc)
		return;
	/* full teardown mirrors the VOOC exit path (reset_voocphy): CP off,
	 * watchdog off, OCP/OVP and ADC back to base defaults. */
	sc8547_reset_voocphy(sc);
	/* ...except the ADC while the PPS session is still up: engage() and the
	 * engine steer on these ADCs, and with the ADC off they read the last
	 * conversion forever (lab Experiment 4: 855 identical samples). */
	if (pps_adc_hold)
		sc8547_set_adc_enable(sc, true);
	dev_info(sc->dev, "PPS CP: master disengaged - buck path restored\n");
}
EXPORT_SYMBOL_GPL(sc8547a_pps_cp_disengage);

/*
 * Hold the master ADC on for a whole PPS session (vendor oplus_chg_pps keeps
 * the CP ADC on from switch-check to exit).  Off again when released.
 */
void sc8547a_pps_adc_hold(bool on)
{
	struct sc8547a_device *sc = sc8547_master_dev;

	pps_adc_hold = on;
	if (sc && !pps_cp_engaged)
		sc8547_set_adc_enable(sc, on);
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

static void sc8547_slave_hw_setting(struct sc8547a_device *master)
{
	struct sc8547a_device *sc = sc8547_slave_dev;

	if (!sc)
		return;
	sc8547_write_byte(sc, SC8547_REG_02, 0x01);		/* VAC_OVP 12V */
	sc8547_write_byte(sc, SC8547_REG_04, 0x50);		/* VBUS_OVP 10V */
	sc8547_write_byte(sc, SC8547_REG_05, 0x10 | (sc->ocp_reg & 0x0f));
	sc8547_write_byte(sc, SC8547_REG_09, 0x03);		/* watchdog 1000ms */
	sc8547_write_byte(sc, SC8547_REG_3C, 0x40);		/* VOOC_CTRL disable */
	sc8547_write_byte(sc, SC8547_REG_11, 0x80);		/* ADC enable */
	sc8547_write_byte(sc, SC8547_REG_07, SC8547_SLAVE_CP_OFF);
	sc8547_write_byte(sc, SC8547_REG_0D, 0x70);
	/*
	 * Same as the master: at parallel CP-close the slave current is near zero
	 * and only ramps as Vbus rises, so an armed IBUS-UCP (5 ms) or soft-start
	 * timeout trips the slave off instantly and it never shares load (reads ~0).
	 * Disable both; OCP/OVP still bound it.
	 */
	sc8547_update_bits(sc, SC8547_REG_05, SC8547_IBUS_UCP_DIS_MASK,
			   SC8547_IBUS_UCP_DIS_MASK);
	sc8547_update_bits(sc, SC8547_REG_08, SC8547_SS_TIMEOUT_SET_MASK,
			   SC8547_SS_TIMEOUT_DISABLE);
	dev_info(master->dev, "slave CP: parallel hw profile applied\n");
}

static void sc8547_slave_cp_enable(struct sc8547a_device *master, bool on)
{
	struct sc8547a_device *sc = sc8547_slave_dev;

	if (!sc)
		return;
	if (on) {
		sc8547_write_byte(sc, SC8547_REG_09, 0x13);	/* WD 1000ms (as master) */
		sc8547_write_byte(sc, SC8547_REG_11, 0x80);	/* ADC on (else ibus reads 0) */
	}
	sc8547_write_byte(sc, SC8547_REG_07,
			  on ? SC8547_SLAVE_CP_ON : SC8547_SLAVE_CP_OFF);
}

/*
 * Keep the slave alive + measurable while it is enabled (mirror of the master's
 * REG_09=0x13 feed): the slave arms a 1000 ms watchdog at CP-close and its ADC
 * self-clears, so without a periodic refresh it would eventually disable itself
 * and its ibus would read 0. Call wherever the master watchdog is fed. NOTE: this
 * does NOT by itself make the slave share current — two SC8547 in parallel do not
 * auto-split; the vendor engages the slave as overload relief and runs an active
 * current-balance loop (oplus_switching / ibus_check deviation), not yet ported.
 */
static void sc8547_slave_feed_wd(void)
{
	if (!sc8547_slave_dev)
		return;
	sc8547_write_byte(sc8547_slave_dev, SC8547_REG_09, 0x13);
	sc8547_write_byte(sc8547_slave_dev, SC8547_REG_11, 0x80);	/* keep ADC on */
}

/* Slave bus current (mA); 0 when no slave or its ADC/CP is idle. */
static int sc8547_slave_ibus(void)
{
	int ibus;

	if (!sc8547_slave_dev)
		return 0;
	ibus = sc8547_get_ibus(sc8547_slave_dev);
	return ibus > 0 ? ibus : 0;
}

/* True if the slave CP has its CHG_EN latched (i.e. it is actually converting). */
static bool sc8547_slave_is_on(void)
{
	u8 v = 0;

	if (!sc8547_slave_dev)
		return false;
	if (sc8547_read_byte(sc8547_slave_dev, SC8547_REG_07, &v))
		return false;
	return !!(v & SC8547_CHG_EN_MASK);
}

/* Drop the slave CP + ADC on teardown / hand-back. */
static void sc8547_slave_reset(void)
{
	struct sc8547a_device *sc = sc8547_slave_dev;

	if (!sc)
		return;
	sc8547_write_byte(sc, SC8547_REG_07, SC8547_SLAVE_CP_OFF);	/* CP off */
	sc8547_update_bits(sc, SC8547_REG_09, SC8547_WATCHDOG_MASK,
			   SC8547_WATCHDOG_DIS);
	sc8547_write_byte(sc, SC8547_REG_11, 0x00);			/* ADC off */
}

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
	u8 allow = (sc->fastchg_enable && sc->curve_sent) ? VOOC_BIT_ACTIVE : 0;
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
		sc->adapter_model_ver = sc->adapter_mesg << 4;
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
	} else if (sc->adapter_model_count == 1) {
		vooc_write_mesg_mask(TX0_DET_BIT5_MASK, &sc->voocphy_tx_buff[0],
				     VOOC_BIT_ACTIVE);
		sc->adapter_model_ver |= sc->adapter_mesg;
		sc->adapter_model_ver &= 0x7F;
		dev_info(sc->dev, "adapter model 0x%02x\n", sc->adapter_model_ver);
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
		sc->fastchg_ask_cmd = VOOC_CMD_UNKNOWN;
		sc->curve_sent = true;	/* allow-bit may now assert on next ask_fastchg */
		/*
		 * Pre-stage the is_vbus_ok reply (vendor set_is_vbus_ok_predata,
		 * one-shot here) so the first is_vbus_ok frame is answered instantly
		 * from hardware and the adapter ramps Vbus. Only when delivering — in
		 * observation mode we must not stage a raise. REG_2C (reactive txbuff)
		 * for the intervening 0x4/0xa frames is untouched, so REG_31 survives
		 * to the first is_vbus_ok clock.
		 */
		if (sc->fastchg_deliver)
			sc8547_set_predata(sc, SC8547_IS_VBUS_OK_PREDATA_SVOOC);
		dev_info(sc->dev, "battery-sys curve sent%s\n",
			 sc->fastchg_deliver ? " (is_vbus_ok predata staged)" : "");
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

	if (vbus < 0 || vbat < 0) {
		sc->vooc_vbus_status = SC_VBUS_LOW;
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
	if (sc->vooc_vbus_status == SC_VBUS_NORMAL && !sc->fastchg_deliver)
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
		sc8547_write_byte(sc, SC8547_REG_09, 0x13);	/* watchdog 1000ms */
		sc8547_set_chg_enable(sc, true);
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

/*
 * GET_BATT_VOL: report the measured pack voltage so the adapter can run its
 * curve. We report the CP's own Vbat ADC directly (coded (mV-3404)/10). The
 * vendor applies an IR-drop compensation formula here; reporting the raw
 * measurement is conservative (slightly under-reports under load) and safe.
 */
static void vooc_handle_get_batt_vol(struct sc8547a_device *sc)
{
	int vbat = sc8547_get_vbat(sc);
	u8 v, l, h;

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

/* Build + physically send the reply for the current RX frame. */
static void vooc_reply_adapter_mesg(struct sc8547a_device *sc)
{
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

	switch (sc->fastchg_ask_cmd) {
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
	default:
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
	sc8547_set_txbuff(sc, (sc->voocphy_tx_buff[0] << 8) | sc->voocphy_tx_buff[1]);
}

/* Reset the ladder-tracking state for a fresh plug. */
/*
 * Host-side SVOOC CC/CV curves, transcribed from the erhai Android device tree
 * (oplus,sc8547a@6F/svooc_charge_strategy). Rows are {target_vbat, ask-current
 * code, exit}, ordered big->small current. The policy picks the first row whose
 * target_vbat >= the live vbat, then descends as vbat rises / current tapers;
 * the terminal exit row hands charging back to the normal (battmgr) path. Only
 * the normal-temperature columns are carried — any other temperature range
 * declines auto fast-charge and lets normal charging run. Codes above the auto
 * ceiling are clamped at use, so several leading high-code rows collapse to the
 * same clamped setpoint (harmless; stepping still tracks vbat).
 */
static const struct sc8547_curve_row sc_curve_soc0_50_nl[] = {
	{ 4050, 73, 0 }, { 4230, 61, 0 }, { 4230, 50, 0 }, { 4230, 40, 0 },
	{ 4380, 39, 0 }, { 4380, 35, 0 }, { 4380, 30, 0 }, { 4535, 29, 0 },
	{ 4535, 25, 0 }, { 4535, 20, 0 }, { 4535, 15, 0 }, { 4545, 10, 1 },
};
static const struct sc8547_curve_row sc_curve_soc0_50_nh[] = {
	{ 4200, 61, 0 }, { 4200, 60, 0 }, { 4200, 50, 0 }, { 4230, 48, 0 },
	{ 4230, 45, 0 }, { 4230, 40, 0 }, { 4380, 39, 0 }, { 4380, 35, 0 },
	{ 4380, 30, 0 }, { 4535, 29, 0 }, { 4535, 25, 0 }, { 4535, 20, 0 },
	{ 4535, 15, 0 }, { 4545, 14, 1 },
};
static const struct sc8547_curve_row sc_curve_soc50_75_nl[] = {
	{ 4380, 30, 0 }, { 4535, 29, 0 }, { 4535, 25, 0 }, { 4535, 20, 0 },
	{ 4535, 15, 0 }, { 4545, 10, 1 },
};
static const struct sc8547_curve_row sc_curve_soc50_75_nh[] = {
	{ 4380, 35, 0 }, { 4380, 30, 0 }, { 4535, 29, 0 }, { 4535, 25, 0 },
	{ 4535, 20, 0 }, { 4535, 15, 0 }, { 4545, 14, 1 },
};
static const struct sc8547_curve_row sc_curve_soc75_85_nl[] = {
	{ 4535, 29, 0 }, { 4535, 25, 0 }, { 4535, 20, 0 }, { 4535, 15, 0 },
	{ 4545, 10, 1 },
};
static const struct sc8547_curve_row sc_curve_soc75_85_nh[] = {
	{ 4380, 30, 0 }, { 4535, 29, 0 }, { 4535, 25, 0 }, { 4535, 20, 0 },
	{ 4535, 15, 0 }, { 4545, 14, 1 },
};
static const struct sc8547_curve_row sc_curve_soc85_90_nl[] = {
	{ 4535, 20, 0 }, { 4535, 15, 0 }, { 4545, 10, 1 },
};
static const struct sc8547_curve_row sc_curve_soc85_90_nh[] = {
	{ 4535, 25, 0 }, { 4535, 20, 0 }, { 4535, 15, 0 }, { 4545, 10, 1 },
};

#define SC_CURVE(arr)	(arr), ARRAY_SIZE(arr)

/*
 * Read pack SOC (%), temperature (0.1 C) and cell voltage (mV) from ONE named
 * gauge power-supply.  Returns -ENODEV when that psy is not registered yet.
 */
static int sc8547_read_gauge_name(const char *name, int *soc, int *temp_ddc,
				  int *vbat_mv)
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
 * "erhai-adsp-bat".  Returns the last error (e.g. -ENODEV when no gauge is up
 * yet), which the caller reports as "gauge not ready".
 */
static int sc8547_read_gauge(struct sc8547a_device *sc, int *soc, int *temp_ddc,
			     int *vbat_mv)
{
	int last = -ENODEV;
	size_t i;

	if (gauge_name) {
		last = sc8547_read_gauge_name(gauge_name, soc, temp_ddc, vbat_mv);
		if (!last)
			return 0;
	}

	for (i = 0; i < ARRAY_SIZE(sc8547_gauge_names); i++) {
		last = sc8547_read_gauge_name(sc8547_gauge_names[i], soc, temp_ddc,
					      vbat_mv);
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

/*
 * Select the CC/CV curve for the current SOC + temperature. Returns 0 and fills
 * curve/len on success, or -ERANGE if this SOC/temp is outside the auto
 * envelope (SOC > 90, or temperature outside the normal window) — the caller
 * then declines auto fast-charge and leaves normal charging running.
 */
static int sc8547_auto_select_curve(int soc, int temp_ddc,
				    const struct sc8547_curve_row **curve, int *len)
{
	bool nh = temp_ddc >= SC8547_AUTO_TEMP_NL_NH_DDC;

	if (temp_ddc < SC8547_AUTO_TEMP_MIN_DDC ||
	    temp_ddc > SC8547_AUTO_TEMP_MAX_DDC)
		return -ERANGE;

	if (soc <= 50)
		nh ? (*curve = sc_curve_soc0_50_nh, *len = ARRAY_SIZE(sc_curve_soc0_50_nh))
		   : (*curve = sc_curve_soc0_50_nl, *len = ARRAY_SIZE(sc_curve_soc0_50_nl));
	else if (soc <= 75)
		nh ? (*curve = sc_curve_soc50_75_nh, *len = ARRAY_SIZE(sc_curve_soc50_75_nh))
		   : (*curve = sc_curve_soc50_75_nl, *len = ARRAY_SIZE(sc_curve_soc50_75_nl));
	else if (soc <= 85)
		nh ? (*curve = sc_curve_soc75_85_nh, *len = ARRAY_SIZE(sc_curve_soc75_85_nh))
		   : (*curve = sc_curve_soc75_85_nl, *len = ARRAY_SIZE(sc_curve_soc75_85_nl));
	else if (soc <= 90)
		nh ? (*curve = sc_curve_soc85_90_nh, *len = ARRAY_SIZE(sc_curve_soc85_90_nh))
		   : (*curve = sc_curve_soc85_90_nl, *len = ARRAY_SIZE(sc_curve_soc85_90_nl));
	else
		return -ERANGE;			/* SOC > 90: no fast-charge */

	return 0;
}

/* First curve row whose target_vbat >= vbat (else the last, terminal row). */
static int sc8547_auto_init_idx(const struct sc8547_curve_row *curve, int len,
				int vbat)
{
	int i;

	for (i = 0; i < len; i++)
		if (vbat <= curve[i].target_vbat)
			return i;
	return len - 1;
}

/* Clamp an Android curve code to the auto ceiling. */
static u8 sc8547_auto_clamp_code(u8 code)
{
	return code > SC8547_AUTO_CODE_CEILING ? SC8547_AUTO_CODE_CEILING : code;
}

/*
 * Advance the CC/CV curve one row when the pack has moved past the current row:
 * vbat has climbed over target (voltage step) or, at the CV plateau, the bus
 * current has tapered below the clamped setpoint (current step). Debounced so a
 * single noisy sample can't step. Returns true once the terminal exit row is
 * reached (caller stops fast-charge and hands back to normal charging).
 */
static bool sc8547_auto_step(struct sc8547a_device *sc, int vbat, int ibus)
{
	const struct sc8547_curve_row *row = &sc->auto_curve[sc->auto_curve_idx];
	u8 clamped = sc8547_auto_clamp_code(row->code);
	bool advance;

	if (row->exit) {
		sc->fastchg_curr_code = clamped;
		return true;
	}

	advance = (vbat > row->target_vbat + SC8547_AUTO_STEP_V_HYST_MV) ||
		  (vbat >= row->target_vbat &&
		   ibus < clamped * 100 - SC8547_AUTO_STEP_I_HYST_MA);

	if (advance && sc->auto_curve_idx < sc->auto_curve_len - 1) {
		if (++sc->auto_step_cnt >= SC8547_AUTO_STEP_DEBOUNCE) {
			sc->auto_curve_idx++;
			sc->auto_step_cnt = 0;
			row = &sc->auto_curve[sc->auto_curve_idx];
			clamped = sc8547_auto_clamp_code(row->code);
			dev_info(sc->dev,
				 "auto: step to curve[%d] target_vbat %d code %u (clamped %u)%s\n",
				 sc->auto_curve_idx, row->target_vbat, row->code,
				 clamped, row->exit ? " EXIT" : "");
		}
	} else {
		sc->auto_step_cnt = 0;
	}

	sc->fastchg_curr_code = clamped;
	return sc->auto_curve[sc->auto_curve_idx].exit;
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
	sc->ask_batt_sys = 0;
	sc->curve_sent = false;
	sc->vooc_head = SVOOC_INVERT_HEAD;
	sc->vooc_saw_any = false;
	/* delivery-phase state (gates fastchg_deliver / fastchg_curr_code persist) */
	sc->cp_enabled = false;
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
	int ret = wcd939x_usbss_dpdm_switch_fast_charge(true);

	/*
	 * -EBUSY = DisplayPort alt-mode owns the USBSS switches. A DP sink is
	 * never a VOOC adapter, and stealing the switch block kills DP AUX, so
	 * stay out and charge over the normal path for this plug.
	 */
	if (ret == -EBUSY) {
		dev_info(sc->dev, "DP alt-mode active — skipping VOOC arm\n");
		sc->auto_state = SC_AUTO_DECLINED;
		return;
	}

	if (ret && ret != -ENODEV)
		dev_warn(sc->dev, "DPDM switch to charge path failed: %d\n", ret);

	sc8547_vooc_reset_state(sc);
	sc8547_init_vooc(sc);
	sc8547_set_adc_enable(sc, true);	/* init_vooc cleared it; keep VBUS ADC */

	/*
	 * Apply the CP hardware profile NOW, in the settle window before any VOOC
	 * frame flows, when delivery is armed. svooc_hw_setting writes CP config
	 * (REG_0D/33/3A etc.) that disturbs the D+/D- path if done mid-servo, and is
	 * too many I2C writes to do in the timing-critical is_vbus reply. Doing it
	 * here means the NORMAL frame only needs watchdog-enable + chg-enable.
	 * The auto path arms in observation (fastchg_deliver still 0) but will
	 * escalate to delivery mid-ladder, so pre-apply the profile now too —
	 * doing it here (before any frame) avoids disturbing the D+/D- servo,
	 * and it is harmless if the adapter is later declined (CP never closes).
	 */
	if (sc->fastchg_deliver || sc->fastchg_auto) {
		sc8547_svooc_hw_setting(sc);
		sc->hw_profile_set = true;
		sc8547_slave_hw_setting(sc);	/* pre-arm parallel CP (stays off) */
	}

	sc8547_send_handshake(sc);
}

/*
 * Auto path: decline fast-charge for this plug and restore the normal USB/PD
 * charge route. Called when no (S)VOOC adapter identified, or a battery
 * precondition (SOC/temp/vbat) is outside the auto envelope. The charger stays
 * plugged and charges over the normal battmgr path; auto stays declined until
 * the adapter is physically removed.
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
	dev_info(sc->dev, "auto fast-charge declined (%s) — normal charging\n", why);
}

/*
 * Auto path: end fast-charge delivery and hand back to the normal charge path.
 * Called on curve exit, on a safety backstop trip, or on a margin-low stop.
 * The adapter drops to 5 V once VOOC frames stop; restoring the crossbar
 * returns D+/D- to the SoC for BC1.2/PD, so the pack keeps charging over the
 * normal battmgr path. Stays DONE until physical detach so it does not re-arm
 * on this plug.
 */
static void sc8547_auto_finish(struct sc8547a_device *sc, const char *why)
{
	sc8547_slave_reset();		/* drop parallel CP first */
	sc8547_set_chg_enable(sc, false);
	sc8547_reset_voocphy(sc);
	wcd939x_usbss_dpdm_switch_fast_charge(false);
	sc8547_set_adc_enable(sc, true);
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc->cp_enabled = false;
	sc->hw_profile_set = false;
	sc->auto_state = SC_AUTO_DONE;
	dev_info(sc->dev, "auto fast-charge ended (%s) — normal charging\n", why);
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
static void sc8547_auto_reservo(struct sc8547a_device *sc)
{
	if (time_after(jiffies, sc->auto_deliver_since +
		       msecs_to_jiffies(SC8547_AUTO_SUSTAINED_MS)))
		sc->auto_reservo_cnt = 0;	/* sustained CC: legit keep-alive reset */

	if (sc->auto_reservo_cnt >= SC8547_AUTO_RESERVO_MAX) {
		sc8547_auto_finish(sc, "adapter reset budget exhausted");
		return;
	}
	sc->auto_reservo_cnt++;

	sc8547_slave_cp_enable(sc, false);	/* drop parallel CP before re-arm */
	sc->fastchg_enable = false;
	sc->fastchg_deliver = false;
	sc8547_vooc_arm_hw(sc);			/* re-init PHY + resend handshake */
	sc->auto_state = SC_AUTO_IDENTIFY;
	sc->auto_identify_deadline = jiffies +
		msecs_to_jiffies(SC8547_AUTO_IDENTIFY_MS);
	dev_info(sc->dev,
		 "auto fast-charge: adapter reset — re-handshaking (retry %d/%d)\n",
		 sc->auto_reservo_cnt, SC8547_AUTO_RESERVO_MAX);
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
	bool adsp_vooc = false;
	bool real_vooc = false;
	int ft = -ENODEV;
	int rt = -ENODEV;

	if (sc->auto_ident_retries >= identify_retries)
		return false;

	if (adsp_classify) {
		ft = erhai_otg_adsp_usb_prop(adsp_fastchg_type_prop);
		adsp_vooc = ft >= 0 && (ft & 0x7f) >= adsp_fastchg_svooc_min;
		if (ft >= 0)
			dev_info(sc->dev,
				 "slow adapter: ADSP fast-chg type %d (%s)\n",
				 ft & 0x7f,
				 adsp_vooc ? "VOOC-class" : "not VOOC");
	}

	/*
	 * erhai: the ADSP never runs a VOOC session, so prop26 stays 0 and
	 * adsp_vooc alone starves the ladder.  But the ADSP still classifies
	 * the brick on its own DPDM leg — USB_REAL_TYPE in the VOOC family
	 * (9 on this board; PPS heads read 8) — and that verdict must count
	 * as evidence, or the serial ladder gets a single 4 s window and
	 * identify_retries is dead weight (vooc-debug 调试笔记 §5).
	 */
	if (adsp_classify) {
		rt = erhai_otg_adsp_usb_prop(adsp_real_type_prop);
		real_vooc = rt == adsp_vooc_real_type;
		if (rt >= 0)
			dev_info(sc->dev,
				 "slow adapter: USB real_type %d (%s)\n",
				 rt, real_vooc ? "VOOC family" : "not VOOC");
	}

	/*
	 * Only keep the ladder alive on VOOC evidence: an ADSP VOOC-family
	 * verdict (Android's is_pd_svooc_adapter / sid_power), the ADSP
	 * real_type VOOC family, or actual VOOC serial frames.  A plain
	 * PD/5 V charger has neither and declines on the FIRST window —
	 * identical to the old single-shot behaviour, so D+/D- and normal
	 * charging are not held hostage by the retry loop.
	 */
	if (!adsp_vooc && !real_vooc && !sc->vooc_saw_any) {
		dev_info(sc->dev,
			 "no VOOC evidence (adsp %d, real %d, frames %d) — declining\n",
			 ft, rt, sc->vooc_saw_any);
		return false;
	}

	dev_info(sc->dev, "slow adapter%s: re-arming identify %d/%u\n",
		 adsp_vooc ? " (ADSP VOOC)" :
		 (real_vooc ? " (real_type VOOC)" :
		 (sc->vooc_saw_any ? " (serial frames)" : "")),
		 sc->auto_ident_retries + 1, identify_retries);

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
static void sc8547_auto_identify_tick(struct sc8547a_device *sc)
{
	const struct sc8547_curve_row *curve;
	int soc, temp_ddc, vbat, cell_mv = -1, vref, len, idx, ret;

	if (!sc->adapter_check_ok) {
		if (time_before(jiffies, sc->auto_identify_deadline))
			return;
		if (sc8547_auto_identify_retry(sc))
			return;
		sc8547_auto_decline(sc, "no VOOC adapter");
		return;
	}

	if (sc->adapter_type != SC_ADAPTER_SVOOC) {
		sc8547_auto_decline(sc, "not an SVOOC adapter");
		return;
	}

	ret = sc8547_read_gauge(sc, &soc, &temp_ddc, &cell_mv);
	if (ret) {
		sc8547_auto_decline(sc, "gauge not ready");
		return;
	}

	if (soc >= sc->max_soc) {
		sc8547_auto_decline(sc, "SoC already at limit");
		return;
	}

	/*
	 * Gate and index the curve on the CELL voltage (gauge). The CP's own
	 * reading is inflated by the delivery IR drop and would make a half-full
	 * pack look like it is already at CV.
	 */
	vbat = sc8547_get_vbat(sc);
	vref = (cell_mv > SC8547_CELL_MIN_VALID_MV) ? cell_mv : vbat;
	/* A failed CP ADC read leaves vref negative; indexing the curve with
	 * that errno would pick the highest-current row (index 0) and engage
	 * the pump flat out.  Skip the tick instead: one bad sample is not a
	 * reason to abandon fast charging for the rest of the plug, and a
	 * persistently unreadable ADC still ends in a decline when the
	 * identify window runs out — which restores the normal charge route
	 * rather than leaving the crossbar armed. */
	if (vref <= 0) {
		if (time_before(jiffies, sc->auto_identify_deadline))
			return;
		sc8547_auto_decline(sc, "CP voltage unreadable");
		return;
	}
	if (vref >= SC8547_VBAT_CV_CEILING_MV) {
		sc8547_auto_decline(sc, "vbat at CV already");
		return;
	}

	if (sc8547_auto_select_curve(soc, temp_ddc, &curve, &len)) {
		sc8547_auto_decline(sc, "SOC/temp outside auto envelope");
		return;
	}

	idx = sc8547_auto_init_idx(curve, len, vref);
	if (curve[idx].exit) {
		sc8547_auto_decline(sc, "already at curve exit");
		return;
	}

	/* Escalate to delivery. One-shot: consume the master arm now. */
	sc->auto_curve = curve;
	sc->auto_curve_len = len;
	sc->auto_curve_idx = idx;
	sc->auto_step_cnt = 0;
	sc->fastchg_curr_code = sc8547_auto_clamp_code(curve[idx].code);
	if (!sc->hw_profile_set) {		/* normally pre-applied at arm */
		sc8547_svooc_hw_setting(sc);
		sc->hw_profile_set = true;
	}
	sc->fastchg_enable = true;
	sc->fastchg_deliver = true;
	sc->fastchg_auto = false;	/* one-shot per arm consumed */
	sc->auto_state = SC_AUTO_DELIVER;
	sc->auto_deliver_since = jiffies;	/* for re-servo sustained-CC refill */
	dev_info(sc->dev,
		 "auto fast-charge: SVOOC identified (soc %d%% temp %d.%d C cell %d mV cp %d mV) — deliver curve[%d] code %u\n",
		 soc, temp_ddc / 10, temp_ddc % 10, vref, vbat, idx,
		 sc->fastchg_curr_code);
}

#define SC8547_VBUS_PRESENT_MV	3000
#define SC8547_ATTACH_SETTLE_MS	300
#define SC8547_ATTACH_POLL_MS	250
/*
 * Once the ladder reaches is_vbus_ok the SVOOC adapter transiently collapses
 * Vbus on protocol resets/timeouts. A single absent poll must NOT be read as an
 * unplug (that would restore the USB route + re-flip the crossbar → audible USB
 * re-enumeration churn). Require Vbus absent for this sustained window before
 * declaring detach; a real unplug stays absent and still trips it.
 */
#define SC8547_DETACH_DEBOUNCE_MS	600

/*
 * Attach poller: while armed, watch the CP's own VBUS ADC for charger
 * plug/unplug. On the attach edge, wait a short settle window (so the Type-C
 * mux has finished reprogramming the USBSS) then arm the crossbar + VOOC
 * handshake; on unplug, restore the normal USB route. This makes a single
 * `echo 1 > vooc` self-manage every plug cycle without manual re-arm timing.
 */
static void sc8547_attach_work(struct work_struct *work)
{
	struct sc8547a_device *sc =
		container_of(work, struct sc8547a_device, attach_work.work);
	int vbus;
	bool present;

	if (!sc->vooc_started)
		return;

	vbus = sc8547_get_vbus(sc);

	/*
	 * Present source: prefer the ADSP connector state (USB_ONLINE, prop 0)
	 * while reachable — it tracks the PHYSICAL plug even when the erhai_pps
	 * engine holds an internal PPS rail that keeps the SC8547 VBUS ADC above
	 * the present threshold across a real detach.  Relying on the ADC alone
	 * stranded the serial-VOOC ladder in `declined` for the whole boot on the
	 * engine kernels (2026-09-25): the SC8547-VBUS attach edge never fired on
	 * a physical replug, so the historical "replug = fresh 5 V identify"
	 * recovery was dead and a genuine SuperVOOC brick stayed locked to PPS.
	 * Fall back to the SC8547 VBUS ADC while the ADSP is not yet reachable
	 * (early boot / firmware absent).
	 */
	{
		int adsp_online = erhai_otg_adsp_usb_prop(0);	/* USB_ONLINE */

		if (adsp_online >= 0)
			present = adsp_online > 0;
		else
			present = vbus > SC8547_VBUS_PRESENT_MV;
	}

	if (!present) {
		/*
		 * Debounce absence: a raw-absent state only becomes a logical
		 * detach after the debounce window of continuous absence, so brief
		 * SVOOC protocol Vbus-dips during the servo don't flip
		 * vbus_was_present (which would fire a spurious detach→attach edge
		 * and re-flip the crossbar). A real unplug stays absent past the
		 * window → detach fires.
		 *
		 * Widen the absence window while auto is delivering or re-servoing:
		 * an SVOOC keep-alive reset can hold Vbus low longer than the plain
		 * detach debounce, and tearing down there would flip the crossbar
		 * and lose the re-handshake. A real unplug still trips, just slower.
		 */
		unsigned long debounce_ms = SC8547_DETACH_DEBOUNCE_MS;

		if (sc->auto_state == SC_AUTO_DELIVER ||
		    (sc->auto_state == SC_AUTO_IDENTIFY && sc->auto_reservo_cnt > 0))
			debounce_ms = SC8547_AUTO_DETACH_DEBOUNCE_MS;

		if (!sc->vbus_was_present) {
			present = false;	/* already detached, stay so */
			sc->vbus_absent_since = 0;
		} else if (!sc->vbus_absent_since) {
			sc->vbus_absent_since = jiffies;
			present = true;		/* debounce in progress */
		} else {
			present = !time_after(jiffies, sc->vbus_absent_since +
					      msecs_to_jiffies(debounce_ms));
		}
	} else {
		sc->vbus_absent_since = 0;
	}

	if (present && !sc->vbus_was_present) {
		/* attach edge: start settle timer, arm once it expires */
		sc->arm_at = jiffies + msecs_to_jiffies(SC8547_ATTACH_SETTLE_MS);
		sc->auto_armed = false;
		sc->slave_low_cnt = 0;
		sc->slave_on_since = jiffies;	/* fresh plug: full settle grace */
		/* Hands-free: re-arm the auto charge-manager on every fresh attach. */
		sc->fastchg_auto = true;
		if (sc->fastchg_auto) {
			/*
			 * Auto charge-manager engaged: arm in observation, gate
			 * real delivery behind adapter-identify. Force the manual
			 * gates off so the settle-window arm is observation-only
			 * until sc8547_auto_identify_tick escalates.
			 */
			sc->auto_state = SC_AUTO_IDENTIFY;
			sc->fastchg_enable = false;
			sc->fastchg_deliver = false;
			sc->auto_curve = NULL;
			sc->auto_step_cnt = 0;
			sc->auto_reservo_cnt = 0;	/* fresh plug: full budget */
			sc->auto_ident_retries = 0;	/* fresh plug: full retry budget */
			dev_info(sc->dev,
				 "auto fast-charge: attach, identifying adapter\n");
		}
		dev_info(sc->dev, "charger attach (vbus %d mV), settling\n", vbus);
	} else if (!present && sc->vbus_was_present) {
		/* detach edge: restore USB route, disarm the PHY */
		sc8547_reset_voocphy(sc);	/* also drops REG_07 (CP off) */
		wcd939x_usbss_dpdm_switch_fast_charge(false);
		sc8547_set_adc_enable(sc, true);	/* re-arm ADC to sense next attach */
		sc->auto_armed = false;
		sc->cp_enabled = false;
		sc->hw_profile_set = false;
		sc->auto_state = SC_AUTO_OFF;
		sc->auto_reservo_cnt = 0;
		sc->vbus_absent_since = 0;
		dev_info(sc->dev, "charger removed, USB route restored\n");
	}

	if (present && !sc->auto_armed && time_after_eq(jiffies, sc->arm_at)) {
		sc8547_vooc_arm_hw(sc);
		sc->auto_armed = true;
		if (sc->auto_state == SC_AUTO_IDENTIFY)
			sc->auto_identify_deadline = jiffies +
				msecs_to_jiffies(identify_ms);
		dev_info(sc->dev, "VOOC armed post-attach, vbus %d mV (%s)\n", vbus,
			 sc->fastchg_enable ? "fastchg-enabled" : "observation");
	}

	/* Auto path: watch for identify, escalate to delivery or decline. */
	if (sc->auto_armed && sc->auto_state == SC_AUTO_IDENTIFY)
		sc8547_auto_identify_tick(sc);

	/*
	 * Stage-2 safety backstop while the CP is delivering: keep the hardware
	 * watchdog fed and hard-stop on an over-voltage / over-current reading.
	 * Latches fastchg_deliver off so a trip needs an explicit re-arm.
	 */
	if (sc->cp_enabled) {
		int cpbat = sc8547_get_vbat(sc);	/* cell + delivery IR drop */
		int cibus = sc8547_get_ibus(sc);
		int gsoc = -1, gtemp = 0, gvbat = -1;
		int gret;
		int cvbat;				/* the reference for CC/CV */

		gret = sc8547_read_gauge(sc, &gsoc, &gtemp, &gvbat);
		/*
		 * CC/CV reference = the CELL voltage (gauge) while it is readable;
		 * the CP's own reading only as the fallback.  See the backstop
		 * comment above: on the loaded CP reading a half-full pack looks
		 * like it is already at CV and every session dies in ~20 s.
		 */
		cvbat = (gret == 0 && gvbat > SC8547_CELL_MIN_VALID_MV) ? gvbat
								       : cpbat;
		/*
		 * Under dual-CP the master REG_13 reads only its half of the bus
		 * current; the CC taper compare and the over-current backstop both
		 * need the TOTAL bus, so sum in the slave's half (0 if single-CP).
		 */
		int cibus_total = cibus + sc8547_slave_ibus();
		int margin = vbus - 2 * cpbat;
		u8 r6 = 0, r7 = 0;

		/*
		 * Auto CC/CV: step the ask-current code down the curve as vbat
		 * rises / current tapers. The terminal exit row ends fast-charge
		 * and hands back to normal charging.
		 */
		if (sc->auto_state == SC_AUTO_DELIVER &&
		    sc8547_auto_step(sc, cvbat, cibus_total)) {
			sc8547_auto_finish(sc, "curve exit");
			goto reschedule;
		}

		sc8547_read_byte(sc, SC8547_REG_06, &r6);
		sc8547_read_byte(sc, SC8547_REG_07, &r7);

		/*
		 * Pack-temperature backstop + max-SoC charge limit. High-current
		 * SVOOC CC dumps real heat into the pack (gauge temp = the
		 * battery-safety sensor); and fast-charge stops once SoC reaches the
		 * user-set ceiling (max_soc, default 80 %, longevity limit). A failed
		 * gauge read returns <=0 and is ignored (other backstops still bound
		 * delivery).
		 */
		{
			if (!gret && gsoc >= sc->max_soc) {
				sc8547_slave_cp_enable(sc, false);
				sc8547_set_chg_enable(sc, false);
				sc->cp_enabled = false;
				sc->fastchg_deliver = false;
				dev_info(sc->dev,
					 "fastchg stop: SoC %d%% >= limit %d%% — handing back to normal charge\n",
					 gsoc, sc->max_soc);
				if (sc->auto_state == SC_AUTO_DELIVER)
					sc8547_auto_finish(sc, "SoC limit reached");
				goto reschedule;
			}
			if (!gret && gtemp >= SC8547_TEMP_ABORT_DDC) {
				sc8547_slave_cp_enable(sc, false);
				sc8547_set_chg_enable(sc, false);
				sc->cp_enabled = false;
				sc->fastchg_deliver = false;
				dev_warn(sc->dev,
					 "fastchg thermal backstop (pack %d.%d C) — CP off, deliver disarmed\n",
					 gtemp / 10, gtemp % 10);
				if (sc->auto_state == SC_AUTO_DELIVER)
					sc8547_auto_finish(sc, "thermal backstop");
				goto reschedule;
			}
		}

		if (cvbat >= SC8547_VBAT_CV_CEILING_MV ||
		    cpbat >= SC8547_CP_OVP_CEILING_MV ||
		    cibus_total >= SC8547_IBUS_ABORT_MA) {
			/* Hard backstop: over-voltage / over-current -> latch off. */
			sc8547_slave_cp_enable(sc, false);
			sc8547_set_chg_enable(sc, false);
			sc->cp_enabled = false;
			sc->fastchg_deliver = false;
			dev_warn(sc->dev,
				 "fastchg backstop tripped (cell %d mV cp %d mV ibus %d mA total, master %d) — CP off, deliver disarmed\n",
				 cvbat, cpbat, cibus_total, cibus);
			if (sc->auto_state == SC_AUTO_DELIVER) {
				sc8547_auto_finish(sc, "backstop (CV reached)");
				goto reschedule;
			}
		} else if (!(r7 & SC8547_CHG_EN_MASK)) {
			/*
			 * CHG_EN self-cleared in hardware (VBUS_ERRORLO / UCP
			 * transient). Recover only when the Vbus margin is healthy
			 * and we are below the CV ceiling; otherwise stop clean
			 * rather than fake a live CP or oscillate into a fault.
			 */
			if (margin >= SC8547_CP_RESTART_MARGIN_MV &&
			    margin <= SC8547_CP_RESTART_MARGIN_CEIL_MV &&
			    cvbat < SC8547_VBAT_CV_CEILING_MV) {
				sc8547_write_byte(sc, SC8547_REG_09, 0x13);
				sc8547_set_chg_enable(sc, true);
				dev_warn(sc->dev,
					 "fastchg: CHG_EN self-cleared, re-enabled (vbus %d mV vbat %d mV margin %d mV REG06=0x%02x)\n",
					 vbus, cvbat, margin, r6);
			} else {
				sc8547_slave_cp_enable(sc, false);
				sc8547_set_chg_enable(sc, false);
				sc->cp_enabled = false;
				if (sc->auto_state == SC_AUTO_DELIVER) {
					/*
					 * CHG_EN self-cleared below the CV ceiling (the
					 * >=CV case finished in the hard backstop above):
					 * the SVOOC adapter dropped its output for a
					 * link reset. Re-servo (re-handshake) instead of
					 * ending, bounded by the retry budget.
					 */
					dev_info(sc->dev,
						 "auto: link drop (vbus %d mV vbat %d mV margin %d mV) — re-servo\n",
						 vbus, cvbat, margin);
					sc8547_auto_reservo(sc);
					goto reschedule;
				}
				dev_warn(sc->dev,
					 "fastchg: CHG_EN self-cleared, margin %d mV too thin (vbat %d mV) — CP stopped\n",
					 margin, cvbat);
			}
		} else {
			bool slave_on = sc8547_slave_is_on();

			sc8547_write_byte(sc, SC8547_REG_09, 0x13);	/* feed watchdog */

			/*
			 * Dual-CP load-sharing (the vendor's threshold engage). Once
			 * the master's own half-bus climbs past the engage threshold
			 * the adapter is holding Vbus high enough that the slave's 2:1
			 * error window is valid, so latch it in and let the two pumps
			 * split the bus — this keeps a single pump off its OCP and
			 * stops the adapter over-volting Vbus to force current through
			 * one converter (the low-SoC code-73 collapse). Retried every
			 * tick until CHG_EN sticks; dropped when total demand falls.
			 */
			if (!slave_on && sc->vooc_vbus_status == SC_VBUS_NORMAL &&
			    cibus >= SC8547_SLAVE_ENGAGE_IBUS_MA) {
				sc8547_slave_cp_enable(sc, true);
				sc->slave_on_since = jiffies;
				sc->slave_low_cnt = 0;
				dev_info(sc->dev,
					 "dual-CP: engaging slave under load (master %d mA vbus %d mV vbat %d mV)\n",
					 cibus, vbus, cvbat);
			} else if (slave_on && slave_settle_ms &&
				   jiffies - sc->slave_on_since <
					   msecs_to_jiffies(slave_settle_ms)) {
				/*
				 * Settling: the adapter's output is already
				 * split, but the slave has not ramped yet (its
				 * ADC may still read its idle ~20 mA).  Feed
				 * it; never judge the drop inside this window.
				 */
				sc8547_slave_feed_wd();
				dev_dbg(sc->dev,
					"dual-CP: slave settling (%u ms on, total %d mA)\n",
					jiffies_to_msecs(jiffies - sc->slave_on_since),
					cibus_total);
			} else if (slave_on &&
				   cibus_total < SC8547_SLAVE_DISENGAGE_IBUS_MA) {
				if (++sc->slave_low_cnt < slave_drop_ticks) {
					sc8547_slave_feed_wd();
				} else {
					sc8547_slave_cp_enable(sc, false);
					sc->slave_low_cnt = 0;
					dev_info(sc->dev,
						 "dual-CP: dropping slave (total %d mA low for %u ticks, %u ms on)\n",
						 cibus_total, slave_drop_ticks,
						 jiffies_to_msecs(jiffies - sc->slave_on_since));
				}
			} else if (slave_on) {
				sc->slave_low_cnt = 0;
				sc8547_slave_feed_wd();			/* keep the slave alive */
			}

			dev_dbg(sc->dev, "fastchg: vbus %d mV vbat %d mV ibus %d mA (master %d slave %d) margin %d mV REG06=0x%02x REG07=0x%02x slave_on=%d\n",
				 vbus, cvbat, cibus_total, cibus, sc8547_slave_ibus(),
				 margin, r6, r7, slave_on);
		}
	}

reschedule:
	sc->vbus_was_present = present;

	if (sc->vooc_started)
		schedule_delayed_work(&sc->attach_work,
				      msecs_to_jiffies(SC8547_ATTACH_POLL_MS));
}

/* Arm / disarm the auto VOOC bring-up (bench control via debugfs). */
static void sc8547_vooc_start(struct sc8547a_device *sc)
{
	sc8547_vooc_reset_state(sc);
	sc->vbus_was_present = false;
	sc->vbus_absent_since = 0;
	sc->auto_armed = false;
	sc->auto_state = SC_AUTO_OFF;
	sc->auto_reservo_cnt = 0;
	sc->arm_at = jiffies;
	sc->vooc_started = true;
	sc8547_set_adc_enable(sc, true);	/* VBUS ADC on for the attach poll */
	schedule_delayed_work(&sc->attach_work, 0);
	dev_info(sc->dev, "VOOC PHY armed (auto, %s)\n",
		 sc->fastchg_enable ? "fastchg-enabled" : "observation");
}

static void sc8547_vooc_stop(struct sc8547a_device *sc)
{
	sc->vooc_started = false;
	cancel_delayed_work_sync(&sc->attach_work);
	sc8547_slave_reset();		/* drop parallel CP */
	sc8547_reset_voocphy(sc);	/* CP off */
	/* Return the D+/D- crossbar to the normal USB path. */
	wcd939x_usbss_dpdm_switch_fast_charge(false);
	sc->auto_armed = false;
	sc->cp_enabled = false;
	sc->hw_profile_set = false;
	sc->auto_state = SC_AUTO_OFF;
	sc->auto_reservo_cnt = 0;
	sc->fastchg_auto = false;
	sc->fastchg_deliver = false;	/* full disarm: re-arm needs explicit re-enable */
	dev_info(sc->dev, "VOOC PHY stopped\n");
}

/* IRQ-time frame processing. Returns true if a frame was handled. */
static bool sc8547_voocphy_isr(struct sc8547a_device *sc)
{
	int word = sc8547_get_adapter_info(sc);

	if (word < 0)
		return false;

	sc->voocphy_rx_buff = word & 0xff;	/* REG_2E: data */
	sc->vooc_flag = (word >> 8) & 0xff;	/* REG_2F: status */

	if (!(sc->vooc_flag & VOOC_RXDATA_DONE_FLAG))
		return false;

	if (vooc_get_mesg_from_adapter(sc)) {
		dev_dbg(sc->dev, "VOOC frame decode failed (rx 0x%02x)\n",
			sc->voocphy_rx_buff);
		return false;
	}

	vooc_reply_adapter_mesg(sc);
	dev_dbg(sc->dev, "VOOC mesg=0x%02x ask=0x%02x tx=%02x%02x\n",
		sc->adapter_mesg, sc->fastchg_ask_cmd,
		sc->voocphy_tx_buff[0], sc->voocphy_tx_buff[1]);
	sc->pre_ask_cmd = sc->fastchg_ask_cmd;
	return true;
}

/* ------------------------------- IRQ ---------------------------------- */

static irqreturn_t sc8547_irq_handler(int irq, void *data)
{
	struct sc8547a_device *sc = data;
	u8 flag = 0, state = 0;

	/* When the PHY is armed, an IRQ is an adapter VOOC frame. */
	if (sc->vooc_started && !sc->is_slave) {
		if (sc8547_voocphy_isr(sc))
			return IRQ_HANDLED;
	}

	sc8547_read_byte(sc, SC8547_REG_0F, &flag);
	sc8547_read_byte(sc, SC8547_REG_06, &state);
	dev_dbg(sc->dev, "irq: int_flag=0x%02x status=0x%02x\n", flag, state);
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

static int sc8547_enable_get(void *data, u64 *val)
{
	struct sc8547a_device *sc = data;
	bool en = false;
	int ret = sc8547_get_chg_enable(sc, &en);

	if (ret)
		return ret;
	*val = en;
	return 0;
}

static int sc8547_enable_set(void *data, u64 val)
{
	return sc8547_set_chg_enable(data, !!val);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_enable_fops, sc8547_enable_get,
			 sc8547_enable_set, "%llu\n");

static int sc8547_initvooc_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	if (val)
		return sc8547_init_vooc(sc);
	sc8547_slave_reset();		/* never leave the slave switching alone */
	return sc8547_reset_voocphy(sc);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_initvooc_fops, NULL, sc8547_initvooc_set, "%llu\n");

static int sc8547_adcen_set(void *data, u64 val)
{
	return sc8547_set_adc_enable(data, !!val);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_adcen_fops, NULL, sc8547_adcen_set, "%llu\n");

/* 0 = 2:1 divide-by-2, 1 = 1:1 bypass */
static int sc8547_mode_set(void *data, u64 val)
{
	return sc8547_set_charge_mode(data, !!val);
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_mode_fops, NULL, sc8547_mode_set, "%llu\n");

static int sc8547_svooc_set(void *data, u64 val)
{
	return val ? sc8547_svooc_hw_setting(data) : 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_svooc_fops, NULL, sc8547_svooc_set, "%llu\n");

/* Arm/disarm the VOOC protocol engine (bench control). */
static int sc8547_vooc_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	if (val)
		sc8547_vooc_start(sc);
	else
		sc8547_vooc_stop(sc);
	return 0;
}
static int sc8547_vooc_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->vooc_started;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_vooc_fops, sc8547_vooc_get, sc8547_vooc_set, "%llu\n");

/*
 * Stage-2b gate. 0 = observation (adapter authenticates but fastchg is refused,
 * no Vbus ramp, no current). Do NOT set 1 on a real battery until the
 * constant-current policy lands — enabling only tells the adapter fastchg is
 * allowed; nothing here manages the resulting Vbus/current yet.
 */
static int sc8547_fastchg_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	sc->fastchg_enable = !!val;
	if (sc->fastchg_enable)
		dev_warn(sc->dev,
			 "fastchg_enable set: adapter will ramp Vbus; CC policy (stage 2b) not yet wired\n");
	return 0;
}
static int sc8547_fastchg_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->fastchg_enable;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_fastchg_fops, sc8547_fastchg_get,
			 sc8547_fastchg_set, "%llu\n");

/*
 * Stage-2 gate. 0 = the full ladder runs but the charge pump is never enabled
 * (Vbus may ramp, but zero current reaches the pack). Set 1 ONLY for a
 * supervised gated real-current test: it lets the CP close inside the Vbus
 * window and pull up to fastchg_current into the pack, backed by the vbat/ibus
 * backstop. Requires fastchg_enable=1 as well.
 */
static int sc8547_deliver_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	sc->fastchg_deliver = !!val;
	if (sc->fastchg_deliver) {
		dev_warn(sc->dev,
			 "fastchg_deliver ARMED: CP will deliver current at Vbus window (curr code %u); backstop vbat %d mV\n",
			 sc->fastchg_curr_code, SC8547_VBAT_CV_CEILING_MV);
	} else if (sc->cp_enabled) {
		sc8547_set_chg_enable(sc, false);
		sc->cp_enabled = false;
		dev_info(sc->dev, "fastchg_deliver disarmed: CP off\n");
	}
	return 0;
}
static int sc8547_deliver_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->fastchg_deliver;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_deliver_fops, sc8547_deliver_get,
			 sc8547_deliver_set, "%llu\n");

/* Requested CC current setpoint (ask-current code = bus-side ibus / 100 mA). */
static int sc8547_curr_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	if (val < SC8547_CURR_CODE_MIN || val > SC8547_CURR_CODE_MAX)
		return -EINVAL;
	sc->fastchg_curr_code = val;
	return 0;
}
static int sc8547_curr_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->fastchg_curr_code;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_curr_fops, sc8547_curr_get, sc8547_curr_set, "%llu\n");

/*
 * Max-SoC charge limit (%). Fast-charge is declined at arm and stopped mid-CC
 * once the gauge SoC reaches this value; normal charging then continues. Default
 * 80 % (battery longevity). Set 100 to disable the limit.
 */
static int sc8547_maxsoc_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	if (val < 1 || val > 100)
		return -EINVAL;
	sc->max_soc = val;
	dev_info(sc->dev, "fast-charge max SoC set to %d%%\n", sc->max_soc);
	return 0;
}
static int sc8547_maxsoc_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->max_soc;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_maxsoc_fops, sc8547_maxsoc_get, sc8547_maxsoc_set, "%llu\n");

/*
 * Auto charge-manager master arm (one-shot, kill-switch). Setting 1 arms the
 * next charger attach for hands-free fast-charge: the driver identifies the
 * adapter and, only if it is a genuine SVOOC adapter and the battery is within
 * the auto envelope (SOC <= 90, normal temperature, vbat below CV), delivers
 * real current stepped down the Android CC/CV curve (clamped to code
 * SC8547_AUTO_CODE_CEILING) and hands back to normal charging at the exit row.
 * A non-VOOC adapter or any failed precondition declines and leaves normal
 * charging running. One-shot: consumed on the escalation to delivery, so a
 * human is in the loop for each fast-charge session. Setting 0 disarms.
 * Requires the PHY started (`echo 1 > vooc`).
 */
static int sc8547_auto_set(void *data, u64 val)
{
	struct sc8547a_device *sc = data;

	sc->fastchg_auto = !!val;
	if (sc->fastchg_auto)
		dev_warn(sc->dev,
			 "fastchg_auto ARMED (one-shot): next SVOOC attach will auto-deliver real current (curve, <= code %u); backstop vbat %d mV / ibus %d mA\n",
			 SC8547_AUTO_CODE_CEILING, SC8547_VBAT_CV_CEILING_MV,
			 SC8547_IBUS_ABORT_MA);
	return 0;
}
static int sc8547_auto_get(void *data, u64 *val)
{
	*val = ((struct sc8547a_device *)data)->fastchg_auto;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(sc8547_auto_fops, sc8547_auto_get, sc8547_auto_set, "%llu\n");

/* Read-only auto/fast-charge status for userspace. */
static int sc8547_state_show(struct seq_file *m, void *unused)
{
	struct sc8547a_device *sc = m->private;
	static const char * const names[] = {
		"off", "identify", "deliver", "declined", "done",
	};

	seq_printf(m, "auto_armed %d state %s code %u max_soc %d%% reservo %d/%d ident_retry %d/%u vbus %d mV vbat %d mV ibus %d mA (master %d slave %d) cp %s\n",
		   sc->fastchg_auto, names[sc->auto_state], sc->fastchg_curr_code,
		   sc->max_soc, sc->auto_reservo_cnt, SC8547_AUTO_RESERVO_MAX,
		   sc->auto_ident_retries, identify_retries,
		   sc8547_get_vbus(sc), sc8547_get_vbat(sc),
		   sc8547_get_ibus(sc) + sc8547_slave_ibus(),
		   sc8547_get_ibus(sc), sc8547_slave_ibus(),
		   sc8547_slave_dev ? "dual" : "single");
	if (sc->auto_state == SC_AUTO_DELIVER && sc->auto_curve)
		seq_printf(m, "curve idx %d/%d target_vbat %d code %u\n",
			   sc->auto_curve_idx, sc->auto_curve_len - 1,
			   sc->auto_curve[sc->auto_curve_idx].target_vbat,
			   sc->auto_curve[sc->auto_curve_idx].code);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sc8547_state);

static void sc8547_create_debugfs(struct sc8547a_device *sc)
{
	sc->debug_root = debugfs_create_dir(dev_name(sc->dev), NULL);
	debugfs_create_file("registers", 0444, sc->debug_root, sc,
			    &sc8547_dump_fops);
	debugfs_create_file("adc", 0444, sc->debug_root, sc, &sc8547_adc_fops);
	debugfs_create_file("cp_enable", 0644, sc->debug_root, sc,
			    &sc8547_enable_fops);
	debugfs_create_file("init_vooc", 0200, sc->debug_root, sc,
			    &sc8547_initvooc_fops);
	debugfs_create_file("adc_enable", 0200, sc->debug_root, sc,
			    &sc8547_adcen_fops);
	debugfs_create_file("charge_mode", 0200, sc->debug_root, sc,
			    &sc8547_mode_fops);
	debugfs_create_file("svooc_setting", 0200, sc->debug_root, sc,
			    &sc8547_svooc_fops);
	if (!sc->is_slave) {
		debugfs_create_file("vooc", 0644, sc->debug_root, sc,
				    &sc8547_vooc_fops);
		debugfs_create_file("fastchg_enable", 0644, sc->debug_root, sc,
				    &sc8547_fastchg_fops);
		debugfs_create_file("fastchg_deliver", 0644, sc->debug_root, sc,
				    &sc8547_deliver_fops);
		debugfs_create_file("fastchg_current", 0644, sc->debug_root, sc,
				    &sc8547_curr_fops);
		debugfs_create_file("max_soc", 0644, sc->debug_root, sc,
				    &sc8547_maxsoc_fops);
		debugfs_create_file("fastchg_auto", 0644, sc->debug_root, sc,
				    &sc8547_auto_fops);
		debugfs_create_file("fastchg_state", 0444, sc->debug_root, sc,
				    &sc8547_state_fops);
	}
}

/* ------------------------------- probe -------------------------------- */

static int sc8547a_probe(struct i2c_client *client)
{
	struct sc8547a_device *sc;
	int ret;

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
	sc->is_slave = device_property_read_bool(sc->dev, "oplus,slave");
	sc->fastchg_curr_code = SC8547_CURR_CODE_MIN;
	sc->max_soc = SC8547_MAX_SOC_DEFAULT;
	sc->ovp_reg = SC8547_DEFAULT_OVP_REG;
	sc->ocp_reg = SC8547_DEFAULT_OCP_REG;
	device_property_read_u8(sc->dev, "sc,vbat-ovp-reg", &sc->ovp_reg);
	device_property_read_u8(sc->dev, "sc,ibus-ocp-reg", &sc->ocp_reg);
	mutex_init(&sc->i2c_lock);
	INIT_DELAYED_WORK(&sc->attach_work, sc8547_attach_work);
	i2c_set_clientdata(client, sc);

	if (!sc8547_check_device_id(sc))
		return dev_err_probe(sc->dev, -ENODEV,
				     "SC8547A device id mismatch (reg 0x36)\n");

	ret = sc8547_hardware_init(sc);
	if (ret)
		return dev_err_probe(sc->dev, ret, "hardware init failed\n");

	if (sc->irq > 0) {
		ret = devm_request_threaded_irq(sc->dev, sc->irq, NULL,
						sc8547_irq_handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						dev_name(sc->dev), sc);
		if (ret)
			return dev_err_probe(sc->dev, ret,
					     "failed to request irq\n");
	}

	sc8547_create_debugfs(sc);

	if (sc->is_slave)
		sc8547_slave_dev = sc;	/* master delivery path drives it */
	else
		sc8547_master_dev = sc;

	/*
	 * Hands-free: start the master's attach poller now so a SuperVOOC adapter
	 * that is already plugged (or plugged later) auto-arms and fast-charges
	 * without any debugfs poke. The poller waits for the attach + settle
	 * window, identifies the adapter (declines non-SVOOC to normal charge),
	 * and is bounded by the max_soc / vbat-CV / ibus / thermal backstops. A
	 * not-yet-ready DPDM crossbar (-ENODEV) is non-fatal and simply retried.
	 */
	if (!sc->is_slave)
		sc8547_vooc_start(sc);

	dev_info(sc->dev, "SC8547A %s charge pump probed (addr 0x%02x)\n",
		 sc->is_slave ? "slave" : "master", client->addr);
	return 0;
}

static void sc8547a_remove(struct i2c_client *client)
{
	struct sc8547a_device *sc = i2c_get_clientdata(client);

	sc->vooc_started = false;
	cancel_delayed_work_sync(&sc->attach_work);
	debugfs_remove_recursive(sc->debug_root);
	sc8547_reset_voocphy(sc);
	if (sc->is_slave && sc8547_slave_dev == sc)
		sc8547_slave_dev = NULL;
	if (!sc->is_slave && sc8547_master_dev == sc)
		sc8547_master_dev = NULL;
}

static void sc8547a_shutdown(struct i2c_client *client)
{
	struct sc8547a_device *sc = i2c_get_clientdata(client);

	sc8547_write_byte(sc, SC8547_REG_11, 0x00);	/* ADC off */
	sc8547_write_byte(sc, SC8547_REG_21, 0x00);	/* DPDM off */
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
	},
	.probe = sc8547a_probe,
	.remove = sc8547a_remove,
	.shutdown = sc8547a_shutdown,
	.id_table = sc8547a_i2c_id,
};
module_i2c_driver(sc8547a_driver);

MODULE_DESCRIPTION("Southchip SC8547A charge pump / VOOC PHY driver");
MODULE_LICENSE("GPL");
