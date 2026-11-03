// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for the CPS8601 wireless pen charger found on the OnePlus Pad 2
 * Pro (erhai, SM8750).
 *
 * The chip is an 8051-core MCU driving the inductive TX coil that charges
 * the capacitive stylus. It sits behind a geni i2c-master-hub SE (0x98c000)
 * at I2C address 0x41 with five TLMM control lines. Register access is
 * 16-bit register addresses with little-endian multi-byte values.
 *
 * Register map, IRQ bits, power-on/off sequences and the RX data-frame
 * protocol were reverse engineered from the vendor module
 * oplus_wireless_pen.ko (GPL v2, "Wireless Pen Charger Cps8601") extracted
 * from the stock vendor_dlkm; see ~/erhai-pen/disasm-keyfuncs.txt and
 * ~/erhai-pen/fw-update-notes.md.
 *
 * The firmware SRAM update path (vendor fw_update_work_func) is implemented
 * as well: wireless_pen/<project>/cps8601_firmware.bin is requested, its
 * header version is compared against the running firmware, and on mismatch
 * (or with the fw_force module parameter) the embedded bootloader is loaded
 * into chip SRAM and the image is flashed and verified.  The vendor module
 * carries the whole machinery but nothing ever schedules it - the factory
 * image ships the matching version, so the path stays dormant there.  A
 * missing image or a failed update is not fatal: init keeps going with the
 * firmware the chip currently runs.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/soc/qcom/pmic_glink.h>
#include "erhai_otg_glink.h"

#define CPS8601_REG_CHIP_ID		0x00	/* 2B, expect 0x8601 */
#define CPS8601_REG_FW_VER		0x02	/* 2B */
#define CPS8601_REG_IRQ_CLEAR		0x05	/* 2B, write 0xffff */
#define CPS8601_REG_IRQ_STATUS		0x07	/* 2B */
#define CPS8601_REG_IRQ_ACK		0x09	/* 2B, write back status */
#define CPS8601_REG_CTRL			0x0b	/* 1B, RMW command bits */
#define CPS8601_REG_PING_CNT		0x10	/* 1B */
#define CPS8601_REG_PING_WIDTH		0x11	/* 2B */
#define CPS8601_REG_OCP_TH			0x28	/* 2B W, DT qcom,ocp-threshold-us */
#define CPS8601_REG_LVP_TH			0x2c	/* 2B W, DT lvp */
#define CPS8601_REG_OVP_TH			0x2a	/* 2B W, DT ovp */
#define CPS8601_REG_FOD_TH			0x2f	/* 2B W, DT fod */
#define CPS8601_REG_TX_VOLTAGE		0x34	/* 2B R */
#define CPS8601_REG_TX_CURRENT		0x38	/* 2B R */
#define CPS8601_REG_CHG_STATUS		0x3e	/* 1B R */
#define CPS8601_REG_Q_CALI_RES		0x13	/* 1B R, 1 = calibrated ok */
#define CPS8601_REG_Q_CALI_MODE		0x15	/* 1B W, 0xAA enter / 0xFF exit */
#define CPS8601_CALI_ENTER		0xAA
#define CPS8601_CALI_EXIT		0xFF
#define CPS8601_CMD_START_Q_CALI	0x40	/* CTRL bit 6, vendor RMW */
#define CPS8601_REG_RX_FRAME		0x40	/* 11B burst */

#define CPS8601_CHIP_ID			0x8601

/*
 * The pen coil rail is fed by the PMIC hboost converter, gated on the ADSP
 * side: the chip stays dead on I2C until the rail is requested over
 * pmic_glink (stock property qcom,pen_is_supply_by_hboost). Vendor message:
 * owner 0x8011, PMIC_GLINK_REQ_RESP, opcode 0x10007, one voltage code byte
 * computed as (mV - 2000) / 50 - attach voltage 10010 mV -> code 160.
 */
#define CPS8601_HBOOST_OWNER		0x8011
#define CPS8601_HBOOST_OPCODE		0x10007
#define CPS8601_HBOOST_ATTACH_CODE	160	/* 10010 mV */
#define CPS8601_HBOOST_DEFAULT_CODE	100	/* 7000 mV */

struct cps8601_hboost_msg {
	struct pmic_glink_hdr	hdr;
	u8			code;
	u8			reserved[3];
} __packed;

/*
 * Firmware SRAM update registers (vendor cps_do_fw_update /
 * cps_load_bootloader).  These live in a separate address space the chip
 * only decodes once program mode has been armed, and they are addressed
 * with 32-bit register addresses - the vendor module keeps a second
 * regmap_config with reg_bits = 32 just for them while the normal map is
 * 16-bit.  On the wire both maps are big-endian in the address and the
 * "word" values go out little-endian over four consecutive registers.
 */
#define CPS8601_PROG_EN		0xffffff00	/* program mode gate: 0x0e arm, 0 release */
#define CPS8601_PROG_KEY	0x40040030	/* flash controller key: 0xffffffff unlock, 0 lock */
#define CPS8601_PROG_CFG	0x40040070	/* flash controller config, written 0xa061 */
#define CPS8601_PROG_MODE	0x40040008	/* 0x08 load SRAM, 0x20 boot SRAM, 0x01 run app */
#define CPS8601_PROG_MASK	0x40040048	/* flash write mask, written 0xfff */
#define CPS8601_PROG_AUX	0x4000e000	/* written 0 in the entry sequence */
#define CPS8601_PROG_BOOT_LATCH	0x40040014	/* 1: remap SRAM over flash, 0: back to flash */

#define CPS8601_SRAM_BOOT_BASE	0x20000000	/* bootloader load address */
#define CPS8601_SRAM_STATUS	0x200009f8	/* boot mailbox status, 4B LE */
#define CPS8601_SRAM_CMD	0x200009fc	/* boot mailbox command, 4B LE */
#define CPS8601_SRAM_WINDOW_A	0x20000a00	/* 256B data window A / cmd arg + result */
#define CPS8601_SRAM_WINDOW_B	0x20000b00	/* 256B data window B */

/* boot mailbox commands (written to CPS8601_SRAM_CMD) */
#define CPS8601_BOOT_CMD_CRC_SRAM	0xb0	/* CRC the SRAM image, length in window A */
#define CPS8601_BOOT_CMD_ERASE		0x70	/* erase the flash image */
#define CPS8601_BOOT_CMD_VERIFY		0x90	/* CRC the flash image, length in window A */
#define CPS8601_BOOT_CMD_COMMIT		0x80	/* flash the verified image */
#define CPS8601_BOOT_CMD_PUT_A		0x10	/* flash data window A */
#define CPS8601_BOOT_CMD_PUT_B		0x20	/* flash data window B */

/* CPS8601_SRAM_STATUS (byte0 & 0xf0): 0x10 busy, 0x20 done, 0x30/0x40 error */
#define CPS8601_BOOT_ST_BUSY		0x10
#define CPS8601_BOOT_ST_DONE		0x20

#define CPS8601_FW_RETRIES		5	/* vendor retry count, update and request alike */
#define CPS8601_FW_BIN_VER_OFF		0xc4	/* image header: big-endian version */
#define CPS8601_FW_BLOCK		0x100	/* SRAM window / flash block size */

/* the serial bootloader, extracted verbatim from the vendor module
 * (oplus_wireless_pen.ko .data, symbol cps8601_bl) */
#include "cps8601_bl.h"

static bool fw_force;
module_param(fw_force, bool, 0644);
MODULE_PARM_DESC(fw_force,
	"reflash the pen charger firmware even if the version matches (unbrick switch)");

/* vendor get_project(): the stock kernel log shows wireless_pen/24926/
 * (bugreport pen-bugreport-analysis.md; the 24976 note was wrong) */
static unsigned int fw_project_id = 24926;
module_param(fw_project_id, uint, 0444);
MODULE_PARM_DESC(fw_project_id,
	"project id used in the wireless_pen/<id>/cps8601_firmware.bin path");

/* IRQ status bits (reg 0x07-0x08), from the vendor handler bit tests */
#define CPS8601_IRQ_Q_CALI		BIT(12)
#define CPS8601_IRQ_PING_OK		BIT(1)
#define CPS8601_IRQ_RX_FRAME		BIT(5)
#define CPS8601_IRQ_CHG_STATE		BIT(6)
#define CPS8601_IRQ_RX_ATTACH		BIT(9)
#define CPS8601_IRQ_RX_DETACH		BIT(10)
#define CPS8601_IRQ_TX_WAKEUP		BIT(0)
#define CPS8601_IRQ_MISLOCATED		BIT(13)

/* chg status anomalies (reg 0x3e) */
#define CPS8601_CHG_ERR_MASK		0xec0

struct cps8601 {
	struct i2c_client	*client;
	struct regmap		*regmap;
	/* program-mode map, 32-bit register addresses (fw update only) */
	struct regmap		*progmap;
	struct mutex		prog_lock;
	struct mutex		io_lock;
	struct device		*dev;

	struct gpio_desc	*int_gpiod;
	struct gpio_desc	*sleep_gpiod;
	struct gpio_desc	*scan_gpiod;
	struct gpio_desc	*off_state_gpiod;
	struct gpio_desc	*sw_en_gpiod;

	struct work_struct	init_work;
	struct delayed_work	ready_work;
	struct work_struct	attach_work;
	struct notifier_block pm_nb;
	bool suspended;
	bool hboost_active;
	struct completion	hboost_ack;
	struct pmic_glink_client *pgclient;
	bool			service_up;

	int			irq;
	bool			irq_armed;
	unsigned int		init_retries;

	u16			fw_version;
	u16			chip_id;
	int			pen_soc;
	bool			present;
	bool			ready;
	bool			removing;	/* unbound: init/ready must not reschedule */

	u32			ocp_uv;
	u32			ovp_uv;
	u32			lvp_uv;
	u32			fod_uv;
};

/* GLINK dispatches by owner, so the pen needs its own response callback. */
static void cps8601_glink_rx(const void *data, size_t len, void *priv)
{
	struct cps8601 *chip = priv;

	const struct pmic_glink_hdr *hdr = data;

	if (len >= sizeof(*hdr) &&
	    le32_to_cpu(hdr->owner) == CPS8601_HBOOST_OWNER &&
	    le32_to_cpu(hdr->opcode) == CPS8601_HBOOST_OPCODE)
		complete(&chip->hboost_ack);
}

static void cps8601_pdr_notify(void *priv, int state)
{
	struct cps8601 *chip = priv;

	/* SERVREG_SERVICE_STATE_UP == 2; charging works long before this
	 * callback fires, so never gate the send loop on it — just note it */
	WRITE_ONCE(chip->service_up, state == 2);
}

static int cps8601_hboost_req(struct cps8601 *chip, u8 code)
{
	struct cps8601_hboost_msg msg = { };
	int ret, i;

	/* Without our own glink client the only route left is the
	 * battery_charger channel with a forged owner, and the ADSP drops
	 * those: 60 x 512 ms of silence was measured (ge3816d8d8 boot,
	 * 2026-10-25), and the reply path is not even wired here --
	 * erhai_otg_glink_set_rx_hook() has no caller.  Retrying 60 rounds
	 * would just make every caller (including remove()) wait ~30 s for
	 * a result that cannot arrive, so say so and give up. */
	if (!chip->pgclient) {
		dev_warn_once(chip->dev,
			      "hboost %u skipped: no glink client (pen charging unavailable)\n",
			      code);
		return -ENODEV;
	}

	msg.hdr.owner = cpu_to_le32(CPS8601_HBOOST_OWNER);
	msg.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP);
	msg.hdr.opcode = cpu_to_le32(CPS8601_HBOOST_OPCODE);
	msg.code = code;

	/*
	 * Vendor set_boost_work_func: re-send every 512 ms until the ADSP
	 * ACKs, up to ~60 rounds (~30 s — charger_pd is still attaching for
	 * the first seconds after boot and silently drops 0x8011 requests
	 * until then).  The Android bugreport shows retry=59 before the
	 * first success; a blind send plus a fixed sleep just NACKs the
	 * chip.  Gate on the routed ACK instead of sleeping.
	 */
	for (i = 0; i < (code ? 60 : 1); i++) {
		if (code && (READ_ONCE(chip->suspended) || READ_ONCE(chip->removing)))
			return -ESHUTDOWN;
		reinit_completion(&chip->hboost_ack);
		ret = pmic_glink_send(chip->pgclient, (void *)&msg, sizeof(msg));
		if (ret && ret != -ECONNRESET)
			return ret;
		if (wait_for_completion_timeout(&chip->hboost_ack,
						msecs_to_jiffies(512))) {
			chip->hboost_active = !!code;
			dev_info(chip->dev, "hboost %u ACKed (round %d, own client %d)\n",
				 code, i + 1, !!chip->pgclient);
			return 0;
		}
	}
	dev_warn(chip->dev, "hboost %u: no ADSP ACK in %d rounds (own client %d)\n",
		 code, i, !!chip->pgclient);
	return -ETIMEDOUT;
}

/*
 * Vendor power-on (init_work_func after firmware check):
 *   sw_en high -> wait 2.5 s for the MCU to boot -> poll chip ID
 *   -> read fw version -> write protection thresholds
 *   -> sleep line high, one scan burst, enable IRQ.
 */
static void cps8601_power_on(struct cps8601 *chip)
{
	/*
	 * Vendor order (init_work_func +0x264): raise the boost rail and
	 * the scan line first, then sw_en, settle 2.5 s for the MCU. The
	 * stock DT calls this line "off_state" but the vendor drives it
	 * high for the whole session: it is the boost enable that powers
	 * the MCU, not an off request.
	 */
	gpiod_set_value(chip->off_state_gpiod, 1);
	gpiod_set_value(chip->scan_gpiod, 1);
	gpiod_set_value(chip->sleep_gpiod, 0);
	gpiod_set_value(chip->sw_en_gpiod, 1);
	msleep(2500);
}

static void cps8601_power_off(struct cps8601 *chip)
{
	/*
	 * Deliberately NOT the vendor shutdown register writes (fod 3000,
	 * ctrl 0x02|0x08|0x10, 0x4b=31, 0x4c=172): they latch the MCU off
	 * and only a real rail power cycle revives it, which the ADSP does
	 * not grant on a warm re-request. Gate the control lines instead;
	 * the MCU idles, and the next probe re-runs the default->attach
	 * rail dance.
	 */
	gpiod_set_value(chip->sw_en_gpiod, 0);
	gpiod_set_value(chip->sleep_gpiod, 0);
	gpiod_set_value(chip->scan_gpiod, 0);
	gpiod_set_value(chip->off_state_gpiod, 0);
}

static void cps8601_scan(struct cps8601 *chip)
{
	gpiod_set_value(chip->scan_gpiod, 1);
	msleep(50);
	gpiod_set_value(chip->scan_gpiod, 0);
}

/*
 * Vendor q_cali_work_func: the TX tank needs a resonance calibration
 * before the chip will radiate.  Without it the scan pulse is a no-op
 * (TX voltage stays 0 -- 2026-10-29 measured).  Sequence:
 *   GP_0 (sleep) high 2.5 s  -- wake the TX subsystem,
 *   0x15 = 0xAA              -- enter calibration mode,
 *   GP_1 (scan) high,
 *   CTRL 0x0b |= 0x40        -- START_Q_CALI,
 *   poll 0x13 for 1          -- calibration result,
 *   0x15 = 0xFF, GP_0 low.
 */
/*
 * Vendor-exact q_cali_work_func (oplus_cps8601.c:1348-1397).  The 8051
 * sleeps at idle: every command sequence must start with the wake
 * handshake -- raise the sleep line (tlmm 106), wait for the
 * TX_WAKEUP interrupt (INT_FLAG bit 0) -- or the writes go nowhere.
 * The earlier blind-msleep variant NAKed precisely because the core
 * was never confirmed awake (2026-11-01 saga).
 */
static int cps8601_read16(struct cps8601 *chip, u16 reg, u16 *val);
static int cps8601_write16(struct cps8601 *chip, u16 reg, u16 val);

static int cps8601_q_cali(struct cps8601 *chip)
{
	unsigned int ctrl = 0, res = 0, cnt = 0;
	u16 flag = 0;
	int i, ret;

	/* Wake handshake: the vendor raises its GP_0 (sleep) line, but on
	 * this board raising tlmm-106 kills the whole I2C bus (measured
	 * twice, 2026-11-01).  The firmware-update path instead drives the
	 * INT pin high for the session -- that handshake is proven to work
	 * here (hundreds of bootloader writes landed through it), so use
	 * the same one. */
	ret = gpiod_direction_output(chip->int_gpiod, 1);
	if (ret)
		return ret;
	msleep(50);
	/* vendor writes INT_EN = 0xffff in every irq pass */
	cps8601_write16(chip, CPS8601_REG_IRQ_CLEAR, 0xffff);
	cps8601_write16(chip, 0x05, 0xffff);

	ret = regmap_write(chip->regmap, CPS8601_REG_Q_CALI_MODE,
			   CPS8601_CALI_ENTER);
	if (ret) {
		gpiod_direction_input(chip->int_gpiod);
		return ret;
	}

	gpiod_set_value(chip->scan_gpiod, 1);

	regmap_read(chip->regmap, CPS8601_REG_CTRL, &ctrl);
	ctrl |= CPS8601_CMD_START_Q_CALI;
	regmap_write(chip->regmap, CPS8601_REG_CTRL, ctrl);

	/* Wait for the Q_CALI interrupt (INT_FLAG bit 12) <= 5 s. */
	for (i = 0; i < 200; i++) {
		usleep_range(20000, 30000);
		ret = cps8601_read16(chip, CPS8601_REG_IRQ_STATUS, &flag);
		if (!ret && (flag & CPS8601_IRQ_Q_CALI))
			break;
	}
	regmap_read(chip->regmap, CPS8601_REG_Q_CALI_RES, &res);
	regmap_read(chip->regmap, 0x0d, &cnt);

	regmap_write(chip->regmap, CPS8601_REG_Q_CALI_MODE,
		     CPS8601_CALI_EXIT);

	if (res == 1)
		dev_info(chip->dev,
			 "q-cali success (flag %#x, times %u, %u ms)\n",
			 flag, cnt, i * 25);
	else
		dev_warn(chip->dev,
			 "q-cali failed (res %u, flag %#x, times %u)\n",
			 res, flag, cnt);

	ret = gpiod_direction_input(chip->int_gpiod);
	return ret ?: (res == 1 ? 0 : -EIO);
}

static int cps8601_read16(struct cps8601 *chip, u16 reg, u16 *val)
{
	unsigned int v;
	int ret;

	/* multi-byte values are little-endian: the first register carries
	 * the low byte (chip id bytes read back 01,86 -> 0x8601) */
	ret = regmap_read(chip->regmap, reg, &v);
	if (ret)
		return ret;
	*val = v;
	ret = regmap_read(chip->regmap, reg + 1, &v);
	if (ret)
		return ret;
	*val |= v << 8;
	return 0;
}

static int cps8601_write16(struct cps8601 *chip, u16 reg, u16 val)
{
	int ret;

	/* little-endian like the reads */
	ret = regmap_write(chip->regmap, reg, val & 0xff);
	if (ret)
		return ret;
	return regmap_write(chip->regmap, reg + 1, val >> 8);
}

/* ---- firmware SRAM update (vendor fw_update_work_func) ---- */

/* 32-bit "word" values go out little-endian over four consecutive
 * registers, exactly like the vendor raw-writes a stack slot */
static int cps8601_prog_write_word(struct cps8601 *chip, u32 reg, u32 val)
{
	u8 buf[4] = { val, val >> 8, val >> 16, val >> 24 };

	return regmap_raw_write(chip->progmap, reg, buf, sizeof(buf));
}

static int cps8601_prog_read_word(struct cps8601 *chip, u32 reg, u32 *val)
{
	u8 buf[4];
	int ret;

	ret = regmap_raw_read(chip->progmap, reg, buf, sizeof(buf));
	if (ret)
		return ret;
	*val = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
	return 0;
}

/* SRAM data writes are capped at 32 bytes per i2c burst
 * (vendor cps_wls_program_sram) */
static int cps8601_prog_write_sram(struct cps8601 *chip, u32 addr,
				   const u8 *buf, size_t len)
{
	int ret;

	while (len > 32) {
		ret = regmap_raw_write(chip->progmap, addr, buf, 32);
		if (ret)
			return ret;
		addr += 32;
		buf += 32;
		len -= 32;
	}
	return regmap_raw_write(chip->progmap, addr, buf, len);
}

/*
 * Poll the boot mailbox status until the command completes: 0x10 keeps
 * working, 0x20 means done, anything else is a failure (vendor
 * cps_wls_program_wait_cmd_done, ~3 s of 1 ms polls).
 */
static int cps8601_boot_wait_done(struct cps8601 *chip)
{
	int tries = 3001;

	while (tries--) {
		u32 st = 0;
		int ret;

		ret = cps8601_prog_read_word(chip, CPS8601_SRAM_STATUS, &st);
		if (ret)
			return ret;
		st &= 0xf0;
		if (st == CPS8601_BOOT_ST_DONE)
			return 0;
		if (st != CPS8601_BOOT_ST_BUSY) {
			dev_warn(chip->dev, "boot cmd failed, status %#x\n", st);
			return -EIO;
		}
		msleep(1);
	}
	dev_warn(chip->dev, "boot cmd timeout\n");
	return -ETIMEDOUT;
}

/*
 * CRC-16, poly 0x1021, init 0, MSB first, no final xor.  This is the
 * bit-exact algorithm both the vendor update path and the chip's
 * bootloader use; crc_ccitt() uses a reflected table with the same
 * parameters but the init value mapping differs, so keep the plain loop.
 */
static u16 cps8601_crc16(const u8 *buf, size_t len)
{
	u16 crc = 0;
	size_t i;
	int b;

	for (i = 0; i < len; i++) {
		crc ^= (u16)buf[i] << 8;
		for (b = 0; b < 8; b++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
		}
	}
	return crc;
}

/*
 * MCU reset into boot mode and bootloader download (vendor
 * cps_load_bootloader): arm program mode, unlock the flash controller,
 * push the bootloader into SRAM, latch it over flash and let the MCU
 * boot it.  The bootloader verifies itself first: its CRC over the loaded
 * image must match the host's, then it answers mailbox commands.
 */
static int cps8601_load_bootloader(struct cps8601 *chip)
{
	size_t bl_len = sizeof(cps8601_bl_blob);
	u32 crc = 0;
	int ret;

	ret = cps8601_prog_write_word(chip, CPS8601_PROG_EN, 0x0e);
	if (ret)
		return ret;
	msleep(110);
	ret = cps8601_prog_write_word(chip, CPS8601_PROG_KEY, 0xffffffff);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_CFG, 0xa061);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_MODE, 0x08);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_AUX, 0);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_MASK, 0xfff);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_KEY, 0);
	if (ret)
		return ret;

	ret = cps8601_prog_write_sram(chip, CPS8601_SRAM_BOOT_BASE,
				      cps8601_bl_blob, bl_len);
	if (ret)
		return ret;

	/* latch SRAM over flash and reset the MCU into the bootloader */
	ret = cps8601_prog_write_word(chip, CPS8601_PROG_KEY, 0xffffffff);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_BOOT_LATCH, 1);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_MODE, 0x20);
	if (ret)
		return ret;
	msleep(10);

	/* re-arm program mode: the MCU reset cleared the gate */
	ret = cps8601_prog_write_word(chip, CPS8601_PROG_EN, 0x0e);
	if (ret)
		return ret;
	msleep(110);
	ret = cps8601_prog_write_word(chip, CPS8601_PROG_KEY, 0xffffffff);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_CFG, 0xa061);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_SRAM_WINDOW_A, bl_len);
	ret = ret ?: cps8601_prog_write_word(chip, CPS8601_SRAM_CMD,
					     CPS8601_BOOT_CMD_CRC_SRAM);
	ret = ret ?: cps8601_boot_wait_done(chip);
	if (ret)
		return ret;
	ret = cps8601_prog_read_word(chip, CPS8601_SRAM_WINDOW_A, &crc);
	if (ret)
		return ret;
	if (crc != cps8601_crc16(cps8601_bl_blob, bl_len)) {
		dev_warn(chip->dev, "bootloader crc mismatch (%#x)\n", crc);
		return -EIO;
	}
	return 0;
}

/*
 * Flash one firmware image.  Caller holds chip->prog_lock.  Every failure
 * restarts the whole sequence from the bootloader download, like the
 * vendor retry loop.
 */
static int cps8601_do_fw_update(struct cps8601 *chip, const u8 *fw,
				size_t fw_size)
{
	int retry;

	if (fw_size < CPS8601_FW_BLOCK)
		return -EINVAL;

	for (retry = 0; retry < CPS8601_FW_RETRIES; retry++) {
		size_t off;
		u32 crc = 0;
		u16 host_crc;
		int ret;

		ret = cps8601_load_bootloader(chip);
		if (ret) {
			dev_warn(chip->dev,
				 "bootloader load failed (try %d): %d\n",
				 retry + 1, ret);
			continue;
		}

		/* erase, then stream the image through the two SRAM windows:
		 * each window is written while the bootloader is still busy
		 * flashing the other one, double-buffered */
		ret = cps8601_prog_write_word(chip, CPS8601_PROG_EN, 0x0e);
		if (ret)
			continue;
		msleep(110);
		ret = cps8601_prog_write_word(chip, CPS8601_PROG_KEY,
					      0xffffffff);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_SRAM_CMD,
						CPS8601_BOOT_CMD_ERASE);
		if (ret)
			continue;
		msleep(10);
		ret = cps8601_boot_wait_done(chip);
		if (ret)
			continue;

		for (off = 0; off < fw_size; off += CPS8601_FW_BLOCK) {
			size_t n = min_t(size_t, CPS8601_FW_BLOCK,
					 fw_size - off);
			bool win_b = off & CPS8601_FW_BLOCK;

			ret = cps8601_prog_write_sram(chip,
					win_b ? CPS8601_SRAM_WINDOW_B :
						CPS8601_SRAM_WINDOW_A,
					fw + off, n);
			if (ret)
				break;
			/* the window just written is only handed over once
			 * the previous flash command finished; block 0 goes
			 * out immediately after the erase */
			if (off) {
				ret = cps8601_boot_wait_done(chip);
				if (ret)
					break;
			}
			ret = cps8601_prog_write_word(chip, CPS8601_SRAM_CMD,
					win_b ? CPS8601_BOOT_CMD_PUT_B :
						CPS8601_BOOT_CMD_PUT_A);
			if (ret)
				break;
		}
		if (ret)
			continue;
		ret = cps8601_boot_wait_done(chip);
		if (ret)
			continue;

		/* verify: the bootloader CRCs what it flashed, the host
		 * compares against its own CRC over the image (vendor
		 * cps_do_fw_update) */
		ret = cps8601_prog_write_word(chip, CPS8601_SRAM_WINDOW_A,
					      fw_size);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_SRAM_CMD,
						CPS8601_BOOT_CMD_VERIFY);
		ret = ret ?: cps8601_boot_wait_done(chip);
		if (ret)
			continue;
		host_crc = cps8601_crc16(fw, fw_size);
		ret = cps8601_prog_read_word(chip, CPS8601_SRAM_WINDOW_A,
					     &crc);
		if (ret)
			continue;
		if (crc != host_crc) {
			dev_warn(chip->dev,
				 "verify failed: chip %#x, host %#x\n",
				 crc, host_crc);
			continue;
		}

		/* commit, then release the MCU back into the application:
		 * unlock, run-app mode, drop the boot latch, lock and leave
		 * program mode (vendor exit sequence) */
		ret = cps8601_prog_write_word(chip, CPS8601_SRAM_CMD,
						CPS8601_BOOT_CMD_COMMIT);
		ret = ret ?: cps8601_boot_wait_done(chip);
		if (ret)
			continue;
		ret = cps8601_prog_write_word(chip, CPS8601_PROG_EN, 0x0e);
		if (ret)
			continue;
		msleep(100);
		ret = cps8601_prog_write_word(chip, CPS8601_PROG_KEY,
					      0xffffffff);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_CFG,
						     0xa061);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_MODE,
						     0x01);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_CFG, 0);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_KEY, 0);
		ret = ret ?: cps8601_prog_write_word(chip, CPS8601_PROG_EN, 0);
		if (ret)
			continue;
		return 0;
	}
	return -EIO;
}

/*
 * Vendor entry point (fw_update_work_func head): fetch the image, compare
 * the header version against the running firmware and reflash on
 * mismatch.  Everything here is best-effort: a missing image or a failed
 * update just leaves the chip with its current firmware.
 */
static void cps8601_fw_update_check(struct cps8601 *chip)
{
	const struct firmware *fw = NULL;
	char path[64];
	u16 bin_ver, sys_ver = 0;
	int ret = -ENOENT, tries;

	snprintf(path, sizeof(path), "wireless_pen/%u/cps8601_firmware.bin",
		 fw_project_id);

	/* the vendor requests the image five times: during early probe the
	 * firmware usermode helper can lose the race */
	for (tries = 0; tries < CPS8601_FW_RETRIES; tries++) {
		release_firmware(fw);
		ret = request_firmware(&fw, path, chip->dev);
		if (!ret)
			break;
		fw = NULL;
	}
	if (ret) {
		dev_info(chip->dev,
			 "%s missing, keeping factory firmware\n", path);
		return;
	}

	if (fw->size < CPS8601_FW_BIN_VER_OFF + 2) {
		dev_warn(chip->dev, "%s too short (%zu)\n", path, fw->size);
		goto out;
	}

	/* image header: big-endian version at 0xc4..0xc5 (0x011e on the
	 * shipped image); the MCU reports the same version as a
	 * little-endian word at reg 0x02 */
	bin_ver = (fw->data[CPS8601_FW_BIN_VER_OFF] << 8) |
		  fw->data[CPS8601_FW_BIN_VER_OFF + 1];
	cps8601_read16(chip, CPS8601_REG_FW_VER, &sys_ver);

	/* vendor handshake: the INT pin is driven high while the update
	 * session runs (cps_set_gpio_value(chip, 0, 1)); the stock module
	 * then waits on a completion nothing ever signals, so just settle
	 * briefly and move on */
	gpiod_direction_output(chip->int_gpiod, 1);
	msleep(50);

	if (!fw_force && bin_ver == sys_ver) {
		dev_dbg(chip->dev, "firmware up to date (0x%04x)\n", sys_ver);
	} else {
		dev_info(chip->dev, "fw update: chip 0x%04x, image 0x%04x%s\n",
			 sys_ver, bin_ver, fw_force ? " (forced)" : "");
		mutex_lock(&chip->prog_lock);
		ret = cps8601_do_fw_update(chip, fw->data, fw->size);
		mutex_unlock(&chip->prog_lock);
		if (ret)
			dev_warn(chip->dev,
				 "fw update failed (%d), keeping current firmware\n",
				 ret);
		else
			dev_info(chip->dev, "fw update complete\n");
	}

	gpiod_direction_input(chip->int_gpiod);
out:
	release_firmware(fw);
}

static void cps8601_handle_rx_frame(struct cps8601 *chip)
{
	u8 frame[11];
	unsigned int v;
	int i;

	/* frame FIFO at 0x40..0x4a; head (0x28,0x17) carries the pen SOC */
	for (i = 0; i < 11; i++) {
		if (regmap_read(chip->regmap, CPS8601_REG_RX_FRAME + i, &v))
			return;
		frame[i] = v;
	}

	if (frame[0] == 0x28 && frame[1] == 0x17) {
		chip->pen_soc = frame[2];
		if (chip->pen_soc >= 100)
			dev_info(chip->dev, "pen full\n");
		else
			dev_info(chip->dev, "pen soc %d%%\n", chip->pen_soc);
	} else if (frame[0] == 0x48 && (frame[1] == 0xb6 || frame[1] == 0xc1)) {
		dev_dbg(chip->dev, "mac frame %*ph\n", 11, frame);
	}
}

/* RX_ATTACH runs in hard/atomic context but the 10 V rail request sleeps
 * (ACK-gated glink round-trip) — vendor sends code 160 only from the attach
 * event, never during init. */
static void cps8601_attach_work(struct work_struct *work)
{
	struct cps8601 *chip = container_of(work, struct cps8601, attach_work);

	if (!READ_ONCE(chip->suspended) && !READ_ONCE(chip->removing))
		cps8601_hboost_req(chip, CPS8601_HBOOST_ATTACH_CODE);
}

static irqreturn_t cps8601_irq(int irq, void *data)
{
	struct cps8601 *chip = data;
	u16 status = 0;
	u16 active;


	/* vendor order: clear pending first, then read status, then ack by
	 * writing the status back (cps_comm_reg e3/e4/e5) */
	cps8601_write16(chip, CPS8601_REG_IRQ_CLEAR, 0xffff);
	if (cps8601_read16(chip, CPS8601_REG_IRQ_STATUS, &status)) {

		return IRQ_HANDLED;
	}
	active = status;
	cps8601_write16(chip, CPS8601_REG_IRQ_ACK, status);

	if (active & CPS8601_IRQ_RX_ATTACH) {
		u16 width = 0;

		cps8601_read16(chip, CPS8601_REG_PING_WIDTH, &width);
		chip->present = true;
		schedule_work(&chip->attach_work);
		dev_info(chip->dev, "pen attached (ping width %d)\n", width);
	}
	if (active & CPS8601_IRQ_RX_DETACH) {
		chip->present = false;
		chip->pen_soc = -1;
		dev_info(chip->dev, "pen removed\n");
	}
	if (active & CPS8601_IRQ_RX_FRAME)
		cps8601_handle_rx_frame(chip);
	if (active & CPS8601_IRQ_CHG_STATE) {
		unsigned int chg = 0;

		regmap_read(chip->regmap, CPS8601_REG_CHG_STATUS, &chg);
		if (chg & CPS8601_CHG_ERR_MASK)
			dev_warn(chip->dev, "charge anomaly, status %#x\n", chg);
		else
			dev_dbg(chip->dev, "charge status %#x\n", chg);
	}
	if (active & CPS8601_IRQ_PING_OK)
		dev_dbg(chip->dev, "ping detected\n");
	if (active & CPS8601_IRQ_MISLOCATED)
		dev_warn(chip->dev, "pen mislocated\n");

	return IRQ_HANDLED;
}

static void cps8601_init_work(struct work_struct *work)
{
	struct cps8601 *chip = container_of(work, struct cps8601, init_work);
	u16 id = 0;
	int ret, tries;


	guard(mutex)(&chip->io_lock);
	/* power the coil rail through the ADSP before touching the MCU.
	 * Vendor bring-up only raises the DEFAULT (7 V) code here — the
	 * 10 V attach code is sent from the RX_ATTACH IRQ, once a pen is
	 * actually detected.  The ACK-gated request mirrors the vendor's
	 * 512 ms × ~60 retry loop. */
	if (READ_ONCE(chip->suspended) || READ_ONCE(chip->removing))
		return;
	ret = cps8601_hboost_req(chip, CPS8601_HBOOST_DEFAULT_CODE);
	if (ret) {
		dev_warn(chip->dev, "hboost default failed: %d\n", ret);
		return;
	}
	/* vendor: first I2C lands ~3.2 s after the rail ACK; the MCU needs
	 * that much after rail-up before it decodes the bus */
	msleep(1500);

	cps8601_power_on(chip);

	for (tries = 0; tries < 10; tries++) {
		ret = cps8601_read16(chip, CPS8601_REG_CHIP_ID, &id);
		if (!ret && id == CPS8601_CHIP_ID)
			break;
		msleep(100);
	}
	if (id != CPS8601_CHIP_ID) {
		dev_warn(chip->dev,
			 "chip id mismatch: %#x (ret %d), retry %u/3 in 15 s\n",
			 id, ret, chip->init_retries + 1);
		if (chip->init_retries++ < 3 && !(READ_ONCE(chip->removing) || READ_ONCE(chip->suspended))) {
			gpiod_set_value(chip->sw_en_gpiod, 0);
			gpiod_set_value(chip->scan_gpiod, 0);
			gpiod_set_value(chip->off_state_gpiod, 0);
			schedule_delayed_work(&chip->ready_work,
					      msecs_to_jiffies(15000));
		}
		goto out_nopm;
	}

	/* vendor fw_update_work_func: reflash when the image version differs
	 * (or fw_force is set); on failure keep going with what runs */
	cps8601_fw_update_check(chip);

	ret = cps8601_read16(chip, CPS8601_REG_FW_VER, &chip->fw_version);
	if (ret)
		goto out_nopm;
	ret = cps8601_write16(chip, CPS8601_REG_OCP_TH, chip->ocp_uv);
	ret = ret ?: cps8601_write16(chip, CPS8601_REG_LVP_TH, chip->lvp_uv);
	ret = ret ?: cps8601_write16(chip, CPS8601_REG_OVP_TH, chip->ovp_uv);
	ret = ret ?: cps8601_write16(chip, CPS8601_REG_FOD_TH, chip->fod_uv);
	if (ret)
		goto out_nopm;

	/* vendor raises the sleep line only after the thresholds are set */
	gpiod_set_value(chip->sleep_gpiod, 1);

	chip->ready = true;
	dev_info(chip->dev, "CPS8601 ready, fw 0x%04x\n", chip->fw_version);

	/* IRQ is armed only once the chip is up: the 2.5 s MCU boot window
	 * would otherwise feed INT glitches into the handler before the
	 * chip can answer, and the vendor driver registers it here too. */
	if (READ_ONCE(chip->removing) || READ_ONCE(chip->suspended))
		goto out_nopm;
	ret = request_threaded_irq(chip->irq, NULL, cps8601_irq,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "cps8601", chip);
	if (ret) {
		dev_err(chip->dev, "irq request failed: %d\n", ret);
		goto out_nopm;
	}
	chip->irq_armed = true;

	if (READ_ONCE(chip->removing) || READ_ONCE(chip->suspended))
		goto out_nopm;

	/*
	 * Vendor post-ready sequence (init_work_func +0x5a4..): drop the
	 * boost line, then after 1.25 s release sw_en and pulse scan 0->1
	 * with a 50 ms gap. Holding sw_en/boost high (as the bring-up v1
	 * did) leaves the MCU in its setup state; only this handoff lets it
	 * enter ping/TX mode.
	 */
	gpiod_set_value(chip->off_state_gpiod, 0);
	schedule_delayed_work(&chip->ready_work, msecs_to_jiffies(1250));

	return;

out_nopm:
	chip->ready = false;
	cps8601_power_off(chip);
	cps8601_hboost_req(chip, 0);
}

static void cps8601_ready_work(struct work_struct *work)
{
	struct cps8601 *chip = container_of(work, struct cps8601,
					    ready_work.work);

	guard(mutex)(&chip->io_lock);
	if (!(READ_ONCE(chip->removing) || READ_ONCE(chip->suspended)) && !chip->ready) {
		/* retry entry: the first power-on raced a slow-starting
		 * ADSP (hboost service swallows early rail requests), so
		 * re-run the whole rail + power-on + probe sequence */
		schedule_work(&chip->init_work);
		return;
	}
	if (READ_ONCE(chip->removing) || READ_ONCE(chip->suspended))
		return;

	/* vendor terminal state keeps sw_en HIGH (it gates the MCU supply);
	 * dropping it here killed the bus right after "ready" and made every
	 * later register write NAK (2026-11-01 endgame). */
	gpiod_set_value(chip->scan_gpiod, 0);
	msleep(50);
	gpiod_set_value(chip->scan_gpiod, 1);
	dev_info(chip->dev, "scan armed (vendor post-ready state)\n");

}

static ssize_t present_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%d\n", chip->present);
}
static DEVICE_ATTR_RO(present);

static ssize_t pen_soc_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "%d\n", chip->pen_soc);
}
static DEVICE_ATTR_RO(pen_soc);

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));

	return sysfs_emit(buf, "0x%04x\n", chip->fw_version);
}
static DEVICE_ATTR_RO(fw_version);

static ssize_t tx_voltage_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));
	u16 v = 0;
	int ret;

	guard(mutex)(&chip->io_lock);
	if (chip->suspended || chip->removing || !chip->ready)
		return -EHOSTDOWN;
	ret = cps8601_read16(chip, CPS8601_REG_TX_VOLTAGE, &v);
	return ret ?: sysfs_emit(buf, "%u\n", v);
}
static DEVICE_ATTR_RO(tx_voltage);

static ssize_t tx_current_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));
	u16 v = 0;
	int ret;

	guard(mutex)(&chip->io_lock);
	if (chip->suspended || chip->removing || !chip->ready)
		return -EHOSTDOWN;
	ret = cps8601_read16(chip, CPS8601_REG_TX_CURRENT, &v);
	return ret ?: sysfs_emit(buf, "%u\n", v);
}
static DEVICE_ATTR_RO(tx_current);

static ssize_t scan_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));
	bool v;
	int ret;

	guard(mutex)(&chip->io_lock);
	if (chip->suspended || chip->removing || !chip->ready)
		return -EHOSTDOWN;
	ret = kstrtobool(buf, &v);
	if (ret)
		return ret;
	if (v)
		cps8601_scan(chip);
	else {
		if (chip->irq_armed) {
			free_irq(chip->irq, chip);
			chip->irq_armed = false;
		}
		ret = cps8601_q_cali(chip);
		if (!READ_ONCE(chip->suspended) && !READ_ONCE(chip->removing)) {
			int irq_ret;

			irq_ret = request_threaded_irq(chip->irq, NULL, cps8601_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "cps8601", chip);
			if (!irq_ret)
				chip->irq_armed = true;
			ret = ret ?: irq_ret;
		}
	}
	return ret ?: count;
}
static DEVICE_ATTR_WO(scan);

/*
 * Vendor FSK broadcast (cps_wls_send_fsk_packet with {0x1f, 0xac}, the
 * remove-path wake packet): the pen's BLE wakes and advertises for a
 * short window on receiving it over the coil link.  Userspace pairs
 * the pen inside that window.
 */
static int cps8601_send_wake_fsk(struct cps8601 *chip)
{
	unsigned int ctrl = 0;
	int ret;

	ret = regmap_write(chip->regmap, 0x4b, 0x1f);
	if (ret)
		return ret;
	ret = regmap_write(chip->regmap, 0x4c, 0xac);
	if (ret)
		return ret;
	regmap_read(chip->regmap, CPS8601_REG_CTRL, &ctrl);
	return regmap_write(chip->regmap, CPS8601_REG_CTRL,
			    ctrl | 0x10 /* TX_CMD_SEND_FSK */);
}

static ssize_t fsk_store(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct cps8601 *chip = i2c_get_clientdata(to_i2c_client(dev));
	bool v;
	int ret;

	guard(mutex)(&chip->io_lock);
	if (chip->suspended || chip->removing || !chip->ready)
		return -EHOSTDOWN;
	ret = kstrtobool(buf, &v);
	if (ret)
		return ret;
	if (v)
		ret = cps8601_send_wake_fsk(chip);
	return ret ?: count;
}
static DEVICE_ATTR_WO(fsk);

static struct attribute *cps8601_attrs[] = {
	&dev_attr_present.attr,
	&dev_attr_pen_soc.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_tx_voltage.attr,
	&dev_attr_tx_current.attr,
	&dev_attr_scan.attr,
	&dev_attr_fsk.attr,
	NULL
};
ATTRIBUTE_GROUPS(cps8601);

static const struct regmap_config cps8601_regmap_cfg = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
};

/* program-mode space (fw update): 32-bit register addresses on the wire */
static const struct regmap_config cps8601_progmap_cfg = {
	.reg_bits	= 32,
	.val_bits	= 8,
	.max_register	= 0xffffffff,
};

/* No wake source or IRQ wake: the coil rail is released before suspend. */
static int cps8601_quiesce(struct cps8601 *chip)
{
	int ret;
	bool active;

	WRITE_ONCE(chip->suspended, true);
	cancel_delayed_work_sync(&chip->ready_work);
	cancel_work_sync(&chip->init_work);
	cancel_delayed_work_sync(&chip->ready_work);
	cancel_work_sync(&chip->init_work);
	guard(mutex)(&chip->io_lock);
	if (chip->irq_armed) {
		free_irq(chip->irq, chip);
		chip->irq_armed = false;
	}
	cancel_work_sync(&chip->attach_work);
	cps8601_power_off(chip);
	chip->ready = false;
	chip->present = false;
	chip->pen_soc = -1;
	active = chip->hboost_active;
	ret = cps8601_hboost_req(chip, 0);
	/* An unavailable optional charger must not prevent system sleep. */
	return active ? ret : 0;
}

static int cps8601_pm_notify(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	struct cps8601 *chip = container_of(nb, struct cps8601, pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		if (cps8601_quiesce(chip)) {
			WRITE_ONCE(chip->suspended, false);
			schedule_work(&chip->init_work);
			return NOTIFY_BAD;
		}
		break;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		WRITE_ONCE(chip->suspended, false);
		chip->init_retries = 0;
		schedule_work(&chip->init_work);
		break;
	}
	return NOTIFY_DONE;
}

static int cps8601_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct cps8601 *chip;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->dev = dev;
	chip->pen_soc = -1;
	i2c_set_clientdata(client, chip);

	chip->regmap = devm_regmap_init_i2c(client, &cps8601_regmap_cfg);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "regmap init failed\n");

	/* second map for the program-mode address space, 32-bit register
	 * addresses; the vendor module does exactly the same */
	chip->progmap = devm_regmap_init_i2c(client, &cps8601_progmap_cfg);
	if (IS_ERR(chip->progmap))
		return dev_err_probe(dev, PTR_ERR(chip->progmap),
				     "program regmap init failed\n");
	mutex_init(&chip->prog_lock);
	mutex_init(&chip->io_lock);

	chip->int_gpiod = devm_gpiod_get(dev, "int", GPIOD_IN);
	chip->sleep_gpiod = devm_gpiod_get(dev, "sleep", GPIOD_OUT_LOW);
	chip->scan_gpiod = devm_gpiod_get(dev, "scan", GPIOD_OUT_LOW);
	chip->off_state_gpiod = devm_gpiod_get(dev, "off-state", GPIOD_OUT_LOW);
	chip->sw_en_gpiod = devm_gpiod_get(dev, "sw-en", GPIOD_OUT_LOW);
	if (IS_ERR(chip->int_gpiod) || IS_ERR(chip->sleep_gpiod) ||
	    IS_ERR(chip->scan_gpiod) || IS_ERR(chip->off_state_gpiod) ||
	    IS_ERR(chip->sw_en_gpiod))
		return dev_err_probe(dev, -EINVAL, "control gpio missing\n");

	device_property_read_u32(dev, "oplus,ocp-threshold", &chip->ocp_uv);
	device_property_read_u32(dev, "oplus,ovp-threshold", &chip->ovp_uv);
	device_property_read_u32(dev, "oplus,lvp-threshold", &chip->lvp_uv);
	device_property_read_u32(dev, "oplus,fod-threshold", &chip->fod_uv);
	if (!chip->ocp_uv || chip->ocp_uv > 3000 ||
	    !chip->lvp_uv || chip->lvp_uv > 5000 ||
	    chip->ovp_uv < 3000 || chip->ovp_uv > 20000 ||
	    !chip->fod_uv || chip->fod_uv > U16_MAX)
		return dev_err_probe(dev, -EINVAL, "invalid protection thresholds\n");

	chip->irq = gpiod_to_irq(chip->int_gpiod);
	if (chip->irq < 0)
		return dev_err_probe(dev, chip->irq, "no irq for int line\n");

	/* the IRQ itself is requested from init_work once the chip is ready */

	INIT_WORK(&chip->init_work, cps8601_init_work);
	INIT_DELAYED_WORK(&chip->ready_work, cps8601_ready_work);
	INIT_WORK(&chip->attach_work, cps8601_attach_work);
	init_completion(&chip->hboost_ack);

	/* Own 0x8011 client so the ADSP both accepts our requests and can
	 * route the ACKs back (see the comment above cps8601_glink_rx).
	 * Registration never blocks; sends just fail with -ECONNRESET
	 * until the ADSP service is up, which the retry loop tolerates. */
	/* devm_pmic_glink_client_alloc() is only valid for pmic_glink child
	 * devices — from this i2c device it would mutex-lock garbage (see
	 * erhai_otg_glink.c).  Register through the real child instead. */
	chip->pgclient = erhai_otg_glink_client_register_sibling(dev,
					CPS8601_HBOOST_OWNER,
					cps8601_glink_rx,
					cps8601_pdr_notify, chip);
	if (IS_ERR(chip->pgclient))
		return dev_err_probe(dev, PTR_ERR(chip->pgclient),
				     "cannot register pen GLINK client\n");

	chip->pm_nb.notifier_call = cps8601_pm_notify;
	ret = register_pm_notifier(&chip->pm_nb);
	if (ret) {
		pmic_glink_client_neuter(chip->pgclient);
		return ret;
	}

	schedule_work(&chip->init_work);

	return 0;
}

static void cps8601_remove(struct i2c_client *client)
{
	struct cps8601 *chip = i2c_get_clientdata(client);

	unregister_pm_notifier(&chip->pm_nb);
	WRITE_ONCE(chip->removing, true);
	cps8601_quiesce(chip);
	pmic_glink_client_neuter(chip->pgclient);
}

static void cps8601_shutdown(struct i2c_client *client)
{
	struct cps8601 *chip = i2c_get_clientdata(client);

	WRITE_ONCE(chip->removing, true);
	cps8601_quiesce(chip);
}

static const struct of_device_id cps8601_of_match[] = {
	{ .compatible = "oplus,wls-charger-cps8601" },
	{ }
};
MODULE_DEVICE_TABLE(of, cps8601_of_match);

static struct i2c_driver cps8601_driver = {
	.driver = {
		.name		= "cps8601-pen-charger",
		.of_match_table	= cps8601_of_match,
		.dev_groups	= cps8601_groups,
	},
	.probe		= cps8601_probe,
	.remove		= cps8601_remove,
	.shutdown	= cps8601_shutdown,
};
module_i2c_driver(cps8601_driver);

MODULE_AUTHOR("erhai mainline port");
MODULE_DESCRIPTION("CPS8601 wireless pen charger driver");
MODULE_LICENSE("GPL v2");
