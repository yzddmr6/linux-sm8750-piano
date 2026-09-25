// SPDX-License-Identifier: GPL-2.0-only
/*
 * Keyboard cover of the OnePlus Pad 2 Pro, attached through pogo pins
 *
 * The keyboard's microcontroller shares one data line with the tablet: the
 * transmit and receive pins of the UART meet on the connector through a
 * direction switch that the host turns on only while it transmits, so every
 * frame the host sends comes back to it.  The host switches the connector
 * supply and, while the supply is off, senses an attached keyboard on a
 * separate line that the keyboard pulls low.  A powered keyboard announces
 * itself with a sync packet and then keeps sending heartbeats; the host only
 * transmits after that first packet.
 *
 * Frame on the line:
 *
 *   0x55 x8 | 0xf1 (0xf2 when repeated) | src | dst | cmd | len | data[len] |
 *   crc16 (big endian) | 0xfe | 0xaa x4
 *
 * src and dst are 0xa1 for the keyboard and 0xa2 for the host.  The CRC uses
 * polynomial 0x8005 and initial value 0xc596, most significant bit first,
 * without reflection or final XOR, over the start code through the data.
 *
 * Protocol, timing and the attach sequence follow the vendor driver published
 * in OnePlusOSS android_kernel_modules_and_devicetree_oneplus_sm8750, commit
 * cc463675855a, vendor/oplus/kernel/device_info/pogo_keyboard/.  The HID usage
 * table is the one of usbkbd.c with that driver's four additions.
 */

#include <linux/bits.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/serdev.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define POGO_KB_SYNC_HEAD		0x55
#define POGO_KB_START			0xf1
#define POGO_KB_START_REPEAT		0xf2
#define POGO_KB_END			0xfe
#define POGO_KB_SYNC_TAIL		0xaa
#define POGO_KB_ADDR_KEYBOARD		0xa1
#define POGO_KB_ADDR_HOST		0xa2

#define POGO_KB_HEAD_SYNC_LEN		8
#define POGO_KB_HEAD_SYNC_MIN		4
#define POGO_KB_TAIL_SYNC_LEN		4
/* start code, source, destination, command, length */
#define POGO_KB_HDR_LEN			5
/* CRC, end code and the two tail sync bytes a receiver waits for */
#define POGO_KB_TRAILER_LEN		5
#define POGO_KB_RX_MAX			(POGO_KB_HDR_LEN + U8_MAX + POGO_KB_TRAILER_LEN)
#define POGO_KB_TX_MAX_DATA		8
#define POGO_KB_TX_MAX			(POGO_KB_HEAD_SYNC_LEN + POGO_KB_HDR_LEN + \
					 POGO_KB_TX_MAX_DATA + 3 + POGO_KB_TAIL_SYNC_LEN)

#define POGO_KB_CRC_POLY		0x8005
#define POGO_KB_CRC_INIT		0xc596

#define POGO_KB_CMD_KEY			0x01
#define POGO_KB_CMD_MEDIA		0x02
#define POGO_KB_CMD_TOUCHPAD		0x03
#define POGO_KB_CMD_PARAM_SET		0x20
#define POGO_KB_CMD_SYNC		0x2f
#define POGO_KB_CMD_PASSTHROUGH		0x38
#define POGO_KB_CMD_GENERAL		0x3a

/* data[0] and data[1] of POGO_KB_CMD_SYNC */
#define POGO_KB_SYNC_POWER_ON		0x01
#define POGO_KB_SYNC_HEARTBEAT		0x05
#define POGO_KB_SYNC_FORMAT		0x02
#define POGO_KB_STATUS_CAPSLOCK		BIT(0)

#define POGO_KB_GENERAL_SLEEP		0x02
#define POGO_KB_PASSTHROUGH_VERSION	0x08

/* Attach detection, supply off: sample period, samples to attach, to give up */
#define POGO_KB_DETECT_PERIOD_MS	20
#define POGO_KB_DETECT_ATTACH		6
#define POGO_KB_DETECT_ABORT		3
#define POGO_KB_RECHECK_PERIOD_MS	50
#define POGO_KB_RECHECK_ATTACH		12
#define POGO_KB_RECHECK_DETACH		20
#define POGO_KB_MAX_POWER_CYCLES	20

/* Supply on: the keyboard is dropped after this long without a frame */
#define POGO_KB_MONITOR_PERIOD_MS	50
#define POGO_KB_SILENCE_MS		500
#define POGO_KB_GRACE_MS		2000

#define POGO_KB_TX_EN_SETUP_US		450
#define POGO_KB_TX_EN_HOLD_US		300
#define POGO_KB_TX_TIMEOUT_MS		20
#define POGO_KB_ECHO_TIMEOUT_MS		50
#define POGO_KB_REPLY_TIMEOUT_MS	100
#define POGO_KB_LED_DELAY_MS		15
#define POGO_KB_LED_RESYNC_MS		1000

#define POGO_KB_SPEED			921600

#define POGO_KB_TP_SLOTS		5
#define POGO_KB_TP_SLOT_LEN		5
#define POGO_KB_TP_X_MAX		2764
#define POGO_KB_TP_Y_MAX		1630
#define POGO_KB_TP_RES			23	/* units per mm */

#define POGO_KB_VENDOR			0x22d9
#define POGO_KB_PRODUCT			0x3869

enum pogo_kb_state {
	POGO_KB_IDLE,		/* supply off, waiting for the attach edge */
	POGO_KB_DETECT,		/* supply off, debouncing an attach edge */
	POGO_KB_RECHECK,	/* supply off, sampling the sense line */
	POGO_KB_POWERED,	/* supply on, waiting for the first packet */
	POGO_KB_CONNECTED,	/* input devices registered */
};

struct pogo_kb_model {
	const char *keyboard;
	const char *touchpad;
};

/* Indexed by the brand byte of the sync packet, which starts at 1 */
static const struct pogo_kb_model pogo_kb_models[] = {
	{ "OPPO Pad 4 Pro Keyboard", "OPPO Pad 4 Pro Keyboard Touchpad" },
	{ "OnePlus Pad 2 Pro Keyboard", "OnePlus Pad 2 Pro Keyboard Touchpad" },
	{ "OnePlus Pad 3 Keyboard", "OnePlus Pad 3 Keyboard Touchpad" },
};

static const struct pogo_kb_model pogo_kb_model_unknown = {
	"Pogo Pin Keyboard", "Pogo Pin Keyboard Touchpad"
};

static const unsigned char pogo_kb_hid_keys[256] = {
	  0,   0,   0,   0,  30,  48,  46,  32,  18,  33,  34,  35,  23,  36,  37,  38,
	 50,  49,  24,  25,  16,  19,  31,  20,  22,  47,  17,  45,  21,  44,   2,   3,
	  4,   5,   6,   7,   8,   9,  10,  11,  28,   1,  14,  15,  57,  12,  13,  26,
	 27,  43,  43,  39,  40,  41,  51,  52,  53,  58,  59,  60,  61,  62,  63,  64,
	 65,  66,  67,  68,  87,  88,  99,  70, 119, 110, 102, 104, 111, 107, 109, 106,
	105, 108, 103,  69,  98,  55,  74,  78,  96,  79,  80,  81,  75,  76,  77,  71,
	 72,  73,  82,  83,  86, 127, 116, 117, 183, 184, 185, 186, 187, 188, 189, 190,
	191, 192, 193, 194, 134, 138, 130, 132, 128, 129, 131, 137, 133, 135, 136, 113,
	115, 114,   0,   0,   0, 121,   0,  89,  93, 124,  92,  94,  95,   0,   0,   0,
	122, 123,  90,  91,  85,   0,   0,   0,   0,   0,   0,   0, 111,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0, 179, 180,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0, 111,   0,   0,   0,   0,   0,   0,   0,
	 29,  42,  56, 125,  97,  54, 100, 126, 164, 166, 165, 163, 161, 115, 114, 113,
	150, 158, 159, 128, 136, 177, 178, 176, 142, 152, 173, 140,   0,   0,   0,   0,
};

#define POGO_KB_HID_MODIFIERS		0xe0

/*
 * Usages of the media key report.  The 0x38e..0x399 block is private to the
 * keyboard firmware; the vendor driver gave those keys private key codes,
 * which map to standard ones here where one exists (lock screen, language
 * switch, microphone mute, touchpad toggle, search, full screen and two
 * user-assignable keys).  The split screen and device interconnect keys
 * have no standard equivalent and are not reported.
 */
static const struct {
	u16 usage;
	u16 code;
} pogo_kb_media_keys[] = {
	{ 0x006f, KEY_BRIGHTNESSUP },
	{ 0x0070, KEY_BRIGHTNESSDOWN },
	{ 0x00b5, KEY_NEXTSONG },
	{ 0x00b6, KEY_PREVIOUSSONG },
	{ 0x00cd, KEY_PLAYPAUSE },
	{ 0x00e2, KEY_MUTE },
	{ 0x00e9, KEY_VOLUMEUP },
	{ 0x00ea, KEY_VOLUMEDOWN },
	{ 0x01d0, KEY_FN },
	{ 0x0224, KEY_BACK },
	{ 0x0244, KEY_APPSELECT },
	{ 0x02fa, KEY_ASSISTANT },
	{ 0x038e, KEY_SCREENLOCK },
	{ 0x0390, KEY_LANGUAGE },
	{ 0x0391, KEY_MICMUTE },
	{ 0x0392, KEY_TOUCHPAD_TOGGLE },
	{ 0x0393, KEY_SEARCH },
	{ 0x0394, KEY_FULL_SCREEN },
	{ 0x0398, KEY_PROG1 },
	{ 0x0399, KEY_PROG2 },
};

struct pogo_kb {
	struct serdev_device *serdev;
	struct device *dev;
	struct regulator *vdd;
	struct gpio_desc *tx_en;
	struct gpio_desc *detect;
	const char *phys_kbd;
	const char *phys_tp;
	int irq;
	u32 speed;

	/* State machine, supply, serial port and transmission */
	struct mutex lock;
	enum pogo_kb_state state;
	bool irq_enabled;
	bool irq_wake;
	unsigned int asserted;
	unsigned int released;
	unsigned int power_cycles;
	unsigned long grace_start;
	unsigned long led_sent;
	struct delayed_work poll_work;
	struct work_struct attach_work;
	struct work_struct led_work;

	/* Receive path: parser, input devices and the transfer in flight */
	spinlock_t rx_lock;
	u8 rx_buf[POGO_KB_RX_MAX];
	unsigned int rx_len;
	unsigned int rx_sync;
	unsigned long last_rx;
	u8 brand;
	struct input_dev *kbd;
	struct input_dev *tp;
	u8 keys[8];
	u16 media[2];
	bool touching;
	bool caps_on;

	u8 xfer_cmd;
	u16 xfer_crc;
	u8 reply_cmd;
	u8 reply_sub;
	u8 reply_len;
	u8 reply[U8_MAX];
	struct completion echo_done;
	struct completion reply_done;

	u16 keymap[ARRAY_SIZE(pogo_kb_hid_keys)];
};

static u16 pogo_kb_crc16(const u8 *buf, size_t len)
{
	u16 crc = POGO_KB_CRC_INIT;
	unsigned int bit;

	while (len--) {
		crc ^= *buf++ << 8;
		for (bit = 0; bit < 8; bit++)
			crc = crc & 0x8000 ? (crc << 1) ^ POGO_KB_CRC_POLY : crc << 1;
	}

	return crc;
}

static size_t pogo_kb_build_frame(u8 *frame, u8 cmd, const u8 *data, u8 len, u16 *crc)
{
	u8 *p = frame;

	memset(p, POGO_KB_SYNC_HEAD, POGO_KB_HEAD_SYNC_LEN);
	p += POGO_KB_HEAD_SYNC_LEN;
	*p++ = POGO_KB_START;
	*p++ = POGO_KB_ADDR_HOST;
	*p++ = POGO_KB_ADDR_KEYBOARD;
	*p++ = cmd;
	*p++ = len;
	memcpy(p, data, len);
	p += len;
	*crc = pogo_kb_crc16(frame + POGO_KB_HEAD_SYNC_LEN, POGO_KB_HDR_LEN + len);
	put_unaligned_be16(*crc, p);
	p += 2;
	*p++ = POGO_KB_END;
	memset(p, POGO_KB_SYNC_TAIL, POGO_KB_TAIL_SYNC_LEN);
	p += POGO_KB_TAIL_SYNC_LEN;

	return p - frame;
}

static unsigned int pogo_kb_media_code(u16 usage)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pogo_kb_media_keys); i++)
		if (pogo_kb_media_keys[i].usage == usage)
			return pogo_kb_media_keys[i].code;

	return KEY_RESERVED;
}

static void pogo_kb_report_keys(struct pogo_kb *kb, const u8 *report, u8 len)
{
	struct input_dev *input = kb->kbd;
	unsigned int i;

	if (!input || len < sizeof(kb->keys))
		return;

	for (i = 0; i < 8; i++)
		input_report_key(input, kb->keymap[POGO_KB_HID_MODIFIERS + i],
				 report[0] & BIT(i));

	/* Usages 1..3 are the boot protocol's error codes, not keys */
	for (i = 2; i < sizeof(kb->keys); i++) {
		if (kb->keys[i] > 3 && !memchr(report + 2, kb->keys[i], 6))
			input_report_key(input, kb->keymap[kb->keys[i]], 0);

		if (report[i] > 3 && !memchr(kb->keys + 2, report[i], 6))
			input_report_key(input, kb->keymap[report[i]], 1);
	}

	input_sync(input);
	memcpy(kb->keys, report, sizeof(kb->keys));
}

static void pogo_kb_report_media(struct pogo_kb *kb, const u8 *data, u8 len)
{
	struct input_dev *input = kb->kbd;
	u16 usage[2];
	unsigned int i;

	if (!input || len < sizeof(usage))
		return;

	usage[0] = get_unaligned_le16(data);
	usage[1] = get_unaligned_le16(data + 2);

	for (i = 0; i < ARRAY_SIZE(usage); i++) {
		if (kb->media[i] && kb->media[i] != usage[0] && kb->media[i] != usage[1])
			input_report_key(input, pogo_kb_media_code(kb->media[i]), 0);

		if (usage[i] && usage[i] != kb->media[0] && usage[i] != kb->media[1])
			input_report_key(input, pogo_kb_media_code(usage[i]), 1);
	}

	input_sync(input);
	memcpy(kb->media, usage, sizeof(kb->media));
}

/*
 * data: one five byte slot per contact (id << 4 | down << 1 | flag, x and y
 * as little endian 16 bit values), then the number of slots, then the button
 * byte.  The low bit of the first slot byte is not understood yet and is
 * ignored.
 */
static void pogo_kb_report_touchpad(struct pogo_kb *kb, const u8 *data, u8 len)
{
	struct input_dev *input = kb->tp;
	unsigned int count, i;
	bool touching = false;

	if (!input || len < 2)
		return;

	count = data[len - 2];
	if (count > POGO_KB_TP_SLOTS || count * POGO_KB_TP_SLOT_LEN + 2 > len) {
		dev_dbg(kb->dev, "malformed touchpad report\n");
		return;
	}

	for (i = 0; i < count; i++) {
		const u8 *slot = &data[i * POGO_KB_TP_SLOT_LEN];
		unsigned int id = slot[0] >> 4;
		bool down = slot[0] & BIT(1);

		if (id >= POGO_KB_TP_SLOTS)
			continue;

		input_mt_slot(input, id);
		if (input_mt_report_slot_state(input, MT_TOOL_FINGER, down)) {
			input_report_abs(input, ABS_MT_POSITION_X,
					 get_unaligned_le16(&slot[1]));
			input_report_abs(input, ABS_MT_POSITION_Y,
					 get_unaligned_le16(&slot[3]));
		}
	}

	for (i = 0; i < POGO_KB_TP_SLOTS; i++)
		touching |= input_mt_is_active(&input->mt->slots[i]);
	kb->touching = touching;

	input_mt_sync_frame(input);
	/* A clickpad: either physical button is the one button */
	input_report_key(input, BTN_LEFT, data[len - 1] & (BIT(0) | BIT(1)));
	input_sync(input);
}

static void pogo_kb_release_touches(struct pogo_kb *kb)
{
	struct input_dev *input = kb->tp;
	unsigned int i;

	if (!input || !kb->touching)
		return;

	for (i = 0; i < POGO_KB_TP_SLOTS; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_inactive(input);
	}
	input_mt_sync_frame(input);
	input_report_key(input, BTN_LEFT, 0);
	input_sync(input);
	kb->touching = false;
}

static void pogo_kb_release_all(struct pogo_kb *kb)
{
	static const u8 none[8];

	pogo_kb_report_keys(kb, none, sizeof(none));
	pogo_kb_report_media(kb, none, 4);
	pogo_kb_release_touches(kb);
}

static void pogo_kb_rx_sync(struct pogo_kb *kb, const u8 *data, u8 len)
{
	bool heartbeat;

	if (len < 3)
		return;

	heartbeat = data[0] == POGO_KB_SYNC_HEARTBEAT && data[1] == POGO_KB_SYNC_FORMAT;

	if (!kb->kbd) {
		/* Any sync packet announces a powered keyboard */
		if (data[0] == POGO_KB_SYNC_POWER_ON && data[1] == POGO_KB_SYNC_FORMAT)
			kb->brand = data[2];
		else if (heartbeat && len >= 4)
			kb->brand = data[3];
		queue_work(system_freezable_wq, &kb->attach_work);
		return;
	}

	if (data[0] == POGO_KB_SYNC_POWER_ON) {
		/* The keyboard restarted while it stayed powered */
		dev_dbg(kb->dev, "keyboard restarted\n");
		pogo_kb_release_all(kb);
		return;
	}

	if (!heartbeat)
		return;

	/* Like the vendor driver, end any contact still open on a heartbeat */
	pogo_kb_release_touches(kb);

	if (!!(data[2] & POGO_KB_STATUS_CAPSLOCK) != READ_ONCE(kb->caps_on) &&
	    time_after(jiffies, READ_ONCE(kb->led_sent) +
				msecs_to_jiffies(POGO_KB_LED_RESYNC_MS)))
		queue_work(system_freezable_wq, &kb->led_work);
}

static void pogo_kb_rx_frame(struct pogo_kb *kb, const u8 *frame, unsigned int len)
{
	u8 cmd = frame[3], data_len = frame[4];
	const u8 *data = &frame[POGO_KB_HDR_LEN];
	const u8 *trailer = data + data_len;
	u16 crc = get_unaligned_be16(trailer);

	if (trailer[2] != POGO_KB_END || trailer[3] != POGO_KB_SYNC_TAIL ||
	    trailer[4] != POGO_KB_SYNC_TAIL) {
		dev_dbg_ratelimited(kb->dev, "frame without end code\n");
		return;
	}

	if (pogo_kb_crc16(frame, POGO_KB_HDR_LEN + data_len) != crc) {
		dev_dbg_ratelimited(kb->dev, "CRC error, command %#x\n", cmd);
		return;
	}

	if (frame[1] == POGO_KB_ADDR_HOST && frame[2] == POGO_KB_ADDR_KEYBOARD) {
		/* The shared line looped back what the host sent */
		if (kb->xfer_cmd && cmd == kb->xfer_cmd && crc == kb->xfer_crc) {
			kb->xfer_cmd = 0;
			complete(&kb->echo_done);
		}
		return;
	}

	if (frame[1] != POGO_KB_ADDR_KEYBOARD || frame[2] != POGO_KB_ADDR_HOST)
		return;

	WRITE_ONCE(kb->last_rx, jiffies);

	if (kb->reply_cmd && cmd == kb->reply_cmd && data_len &&
	    data[0] == kb->reply_sub) {
		memcpy(kb->reply, data, data_len);
		kb->reply_len = data_len;
		kb->reply_cmd = 0;
		complete(&kb->reply_done);
		return;
	}

	switch (cmd) {
	case POGO_KB_CMD_KEY:
		pogo_kb_report_keys(kb, data, data_len);
		break;
	case POGO_KB_CMD_MEDIA:
		pogo_kb_report_media(kb, data, data_len);
		break;
	case POGO_KB_CMD_TOUCHPAD:
		pogo_kb_report_touchpad(kb, data, data_len);
		break;
	case POGO_KB_CMD_SYNC:
		pogo_kb_rx_sync(kb, data, data_len);
		break;
	default:
		dev_dbg_ratelimited(kb->dev, "ignoring command %#x\n", cmd);
		break;
	}
}

static void pogo_kb_rx_byte(struct pogo_kb *kb, u8 byte)
{
	unsigned int len;

	if (!kb->rx_len) {
		if (byte == POGO_KB_SYNC_HEAD) {
			kb->rx_sync = min(kb->rx_sync + 1, POGO_KB_HEAD_SYNC_LEN);
			return;
		}

		if ((byte == POGO_KB_START || byte == POGO_KB_START_REPEAT) &&
		    kb->rx_sync >= POGO_KB_HEAD_SYNC_MIN)
			kb->rx_buf[kb->rx_len++] = byte;
		kb->rx_sync = 0;
		return;
	}

	kb->rx_buf[kb->rx_len++] = byte;
	if (kb->rx_len < POGO_KB_HDR_LEN)
		return;

	len = POGO_KB_HDR_LEN + kb->rx_buf[4] + POGO_KB_TRAILER_LEN;
	if (kb->rx_len < len)
		return;

	pogo_kb_rx_frame(kb, kb->rx_buf, len);
	kb->rx_len = 0;
}

static size_t pogo_kb_receive_buf(struct serdev_device *serdev, const u8 *buf,
				  size_t count)
{
	struct pogo_kb *kb = serdev_device_get_drvdata(serdev);
	size_t i;

	spin_lock(&kb->rx_lock);
	for (i = 0; i < count; i++)
		pogo_kb_rx_byte(kb, buf[i]);
	spin_unlock(&kb->rx_lock);

	return count;
}

static const struct serdev_device_ops pogo_kb_serdev_ops = {
	.receive_buf = pogo_kb_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

/*
 * Send one frame.  The direction switch is on only around the frame, with
 * the settling times of the vendor driver, and the frame counts as sent once
 * its echo came back intact.  With @reply, also wait for the keyboard's
 * answer, which is the command plus one and repeats data[0].
 */
static int pogo_kb_xfer(struct pogo_kb *kb, u8 cmd, const u8 *data, u8 len,
			bool reply)
{
	u8 frame[POGO_KB_TX_MAX];
	ssize_t written;
	size_t size;
	u16 crc;
	int ret = 0;

	lockdep_assert_held(&kb->lock);

	if (WARN_ON(!len || len > POGO_KB_TX_MAX_DATA))
		return -EINVAL;

	size = pogo_kb_build_frame(frame, cmd, data, len, &crc);

	spin_lock(&kb->rx_lock);
	kb->xfer_cmd = cmd;
	kb->xfer_crc = crc;
	kb->reply_cmd = reply ? cmd + 1 : 0;
	kb->reply_sub = data[0];
	kb->reply_len = 0;
	reinit_completion(&kb->echo_done);
	reinit_completion(&kb->reply_done);
	spin_unlock(&kb->rx_lock);

	gpiod_set_value_cansleep(kb->tx_en, 1);
	usleep_range(POGO_KB_TX_EN_SETUP_US, POGO_KB_TX_EN_SETUP_US + 50);
	written = serdev_device_write(kb->serdev, frame, size,
				      msecs_to_jiffies(POGO_KB_TX_TIMEOUT_MS));
	serdev_device_wait_until_sent(kb->serdev,
				      msecs_to_jiffies(POGO_KB_TX_TIMEOUT_MS));
	usleep_range(POGO_KB_TX_EN_HOLD_US, POGO_KB_TX_EN_HOLD_US + 50);
	gpiod_set_value_cansleep(kb->tx_en, 0);

	if (written < 0)
		ret = written;
	else if (written != size)
		ret = -EIO;
	else if (!wait_for_completion_timeout(&kb->echo_done,
					      msecs_to_jiffies(POGO_KB_ECHO_TIMEOUT_MS)))
		ret = -EIO;
	else if (reply &&
		 !wait_for_completion_timeout(&kb->reply_done,
					      msecs_to_jiffies(POGO_KB_REPLY_TIMEOUT_MS)))
		ret = -ETIMEDOUT;

	spin_lock(&kb->rx_lock);
	kb->xfer_cmd = 0;
	kb->reply_cmd = 0;
	spin_unlock(&kb->rx_lock);

	if (ret == -EIO)
		dev_dbg(kb->dev, "command %#x was not echoed\n", cmd);

	return ret;
}

static int pogo_kb_send_leds(struct pogo_kb *kb)
{
	const u8 data[] = {
		0x0d, 0x04, READ_ONCE(kb->caps_on), 0x00,
		0x00,	/* mute */
		0x00,	/* microphone mute */
	};

	return pogo_kb_xfer(kb, POGO_KB_CMD_PARAM_SET, data, sizeof(data), false);
}

static int pogo_kb_set_awake(struct pogo_kb *kb, bool awake)
{
	const u8 data[] = { POGO_KB_GENERAL_SLEEP, 0x01, awake ? 0x00 : 0x01 };
	int ret;

	ret = pogo_kb_xfer(kb, POGO_KB_CMD_GENERAL, data, sizeof(data), true);
	if (ret)
		return ret;

	/* POGO_KB_GENERAL_SLEEP, 0x02, the requested state, 1 for done */
	if (kb->reply_len < 4 || kb->reply[2] != data[2] || kb->reply[3] != 1)
		return -EPROTO;

	return 0;
}

static void pogo_kb_log_version(struct pogo_kb *kb)
{
	static const u8 data[] = { POGO_KB_PASSTHROUGH_VERSION, 0x01, 0x01 };
	int len, ret;

	ret = pogo_kb_xfer(kb, POGO_KB_CMD_PASSTHROUGH, data, sizeof(data), true);
	if (ret) {
		dev_dbg(kb->dev, "no firmware version: %d\n", ret);
		return;
	}

	/* POGO_KB_PASSTHROUGH_VERSION, string length + 1, string */
	if (kb->reply_len < 2 || kb->reply[1] < 2 ||
	    kb->reply[1] - 1 > kb->reply_len - 2)
		return;

	len = kb->reply[1] - 1;
	dev_info(kb->dev, "keyboard firmware %*pE\n", len, &kb->reply[2]);
}

static int pogo_kb_event(struct input_dev *input, unsigned int type,
			 unsigned int code, int value)
{
	struct pogo_kb *kb = input_get_drvdata(input);

	if (type == EV_LED && code == LED_CAPSL) {
		WRITE_ONCE(kb->caps_on, !!value);
		queue_work(system_freezable_wq, &kb->led_work);
	}

	return 0;
}

static int pogo_kb_register_inputs(struct pogo_kb *kb, u8 brand)
{
	const struct pogo_kb_model *model = &pogo_kb_model_unknown;
	struct input_dev *kbd, *tp;
	unsigned int i;
	int ret;

	if (brand && brand <= ARRAY_SIZE(pogo_kb_models))
		model = &pogo_kb_models[brand - 1];

	kbd = input_allocate_device();
	if (!kbd)
		return -ENOMEM;

	kbd->name = model->keyboard;
	kbd->phys = kb->phys_kbd;
	kbd->id.bustype = BUS_HOST;
	kbd->id.vendor = POGO_KB_VENDOR;
	kbd->id.product = POGO_KB_PRODUCT;
	kbd->dev.parent = kb->dev;
	kbd->event = pogo_kb_event;
	kbd->keycode = kb->keymap;
	kbd->keycodesize = sizeof(kb->keymap[0]);
	kbd->keycodemax = ARRAY_SIZE(kb->keymap);
	input_set_drvdata(kbd, kb);

	__set_bit(EV_KEY, kbd->evbit);
	__set_bit(EV_REP, kbd->evbit);
	for (i = 0; i < ARRAY_SIZE(kb->keymap); i++)
		__set_bit(kb->keymap[i], kbd->keybit);
	for (i = 0; i < ARRAY_SIZE(pogo_kb_media_keys); i++)
		__set_bit(pogo_kb_media_keys[i].code, kbd->keybit);
	__clear_bit(KEY_RESERVED, kbd->keybit);
	input_set_capability(kbd, EV_LED, LED_CAPSL);

	tp = input_allocate_device();
	if (!tp) {
		ret = -ENOMEM;
		goto err_free_kbd;
	}

	tp->name = model->touchpad;
	tp->phys = kb->phys_tp;
	tp->id = kbd->id;
	tp->dev.parent = kb->dev;

	input_set_abs_params(tp, ABS_MT_POSITION_X, 0, POGO_KB_TP_X_MAX, 0, 0);
	input_set_abs_params(tp, ABS_MT_POSITION_Y, 0, POGO_KB_TP_Y_MAX, 0, 0);
	input_abs_set_res(tp, ABS_MT_POSITION_X, POGO_KB_TP_RES);
	input_abs_set_res(tp, ABS_MT_POSITION_Y, POGO_KB_TP_RES);
	input_set_capability(tp, EV_KEY, BTN_LEFT);
	__set_bit(INPUT_PROP_BUTTONPAD, tp->propbit);

	ret = input_mt_init_slots(tp, POGO_KB_TP_SLOTS, INPUT_MT_POINTER);
	if (ret)
		goto err_free_tp;

	ret = input_register_device(kbd);
	if (ret)
		goto err_free_tp;

	ret = input_register_device(tp);
	if (ret) {
		input_unregister_device(kbd);
		input_free_device(tp);
		return ret;
	}

	spin_lock(&kb->rx_lock);
	memset(kb->keys, 0, sizeof(kb->keys));
	memset(kb->media, 0, sizeof(kb->media));
	kb->touching = false;
	kb->kbd = kbd;
	kb->tp = tp;
	spin_unlock(&kb->rx_lock);

	dev_info(kb->dev, "%s attached\n", model->keyboard);

	return 0;

err_free_tp:
	input_free_device(tp);
err_free_kbd:
	input_free_device(kbd);
	return ret;
}

static void pogo_kb_unregister_inputs(struct pogo_kb *kb)
{
	struct input_dev *kbd, *tp;

	spin_lock(&kb->rx_lock);
	kbd = kb->kbd;
	tp = kb->tp;
	kb->kbd = NULL;
	kb->tp = NULL;
	spin_unlock(&kb->rx_lock);

	if (tp)
		input_unregister_device(tp);
	if (kbd)
		input_unregister_device(kbd);
}

static void pogo_kb_schedule(struct pogo_kb *kb, unsigned int ms)
{
	mod_delayed_work(system_freezable_wq, &kb->poll_work, msecs_to_jiffies(ms));
}

static void pogo_kb_enter_idle(struct pogo_kb *kb)
{
	kb->state = POGO_KB_IDLE;
	if (!kb->irq_enabled) {
		kb->irq_enabled = true;
		enable_irq(kb->irq);
	}
}

static void pogo_kb_enter_recheck(struct pogo_kb *kb)
{
	kb->state = POGO_KB_RECHECK;
	kb->asserted = 0;
	kb->released = 0;
	pogo_kb_schedule(kb, POGO_KB_RECHECK_PERIOD_MS);
}

static void pogo_kb_power_down(struct pogo_kb *kb)
{
	int ret;

	pogo_kb_unregister_inputs(kb);

	ret = regulator_disable(kb->vdd);
	if (ret)
		dev_err(kb->dev, "failed to switch off the connector: %d\n", ret);

	gpiod_set_value_cansleep(kb->tx_en, 0);
	serdev_device_close(kb->serdev);
}

static void pogo_kb_power_up(struct pogo_kb *kb)
{
	unsigned int speed;
	int ret;

	kb->power_cycles++;

	ret = serdev_device_open(kb->serdev);
	if (ret) {
		dev_err(kb->dev, "failed to open the serial port: %d\n", ret);
		goto recheck;
	}

	speed = serdev_device_set_baudrate(kb->serdev, kb->speed);
	if (speed != kb->speed)
		dev_warn(kb->dev, "line runs at %u baud instead of %u\n",
			 speed, kb->speed);
	serdev_device_set_flow_control(kb->serdev, false);

	spin_lock(&kb->rx_lock);
	kb->rx_len = 0;
	kb->rx_sync = 0;
	spin_unlock(&kb->rx_lock);

	ret = regulator_enable(kb->vdd);
	if (ret) {
		dev_err(kb->dev, "failed to switch on the connector: %d\n", ret);
		serdev_device_close(kb->serdev);
		goto recheck;
	}

	kb->state = POGO_KB_POWERED;
	kb->grace_start = jiffies;
	WRITE_ONCE(kb->last_rx, kb->grace_start);
	pogo_kb_schedule(kb, POGO_KB_MONITOR_PERIOD_MS);
	return;

recheck:
	pogo_kb_enter_recheck(kb);
}

static void pogo_kb_sample(struct pogo_kb *kb, unsigned int attach,
			   unsigned int detach, unsigned int period_ms)
{
	int attached;

	attached = gpiod_get_value_cansleep(kb->detect);
	if (attached < 0) {
		dev_err(kb->dev, "failed to read the sense line: %d\n", attached);
		attached = 0;
	}

	if (attached) {
		kb->asserted++;
		kb->released = 0;
	} else {
		kb->released++;
		kb->asserted = 0;
	}

	if (kb->released >= detach) {
		pogo_kb_enter_idle(kb);
	} else if (kb->asserted < attach) {
		pogo_kb_schedule(kb, period_ms);
	} else if (kb->power_cycles < POGO_KB_MAX_POWER_CYCLES) {
		pogo_kb_power_up(kb);
	} else {
		dev_warn(kb->dev, "attached keyboard does not answer, waiting for it to be attached again\n");
		pogo_kb_enter_idle(kb);
	}
}

static void pogo_kb_monitor(struct pogo_kb *kb)
{
	unsigned long now = jiffies;

	if (time_before(now, READ_ONCE(kb->last_rx) + msecs_to_jiffies(POGO_KB_SILENCE_MS)) ||
	    time_before(now, kb->grace_start + msecs_to_jiffies(POGO_KB_GRACE_MS))) {
		pogo_kb_schedule(kb, POGO_KB_MONITOR_PERIOD_MS);
		return;
	}

	if (kb->state == POGO_KB_CONNECTED)
		dev_info(kb->dev, "keyboard went silent, switching the connector off\n");
	else
		dev_info(kb->dev, "no packet after switching the connector on\n");

	pogo_kb_power_down(kb);
	pogo_kb_enter_recheck(kb);
}

static void pogo_kb_poll_work(struct work_struct *work)
{
	struct pogo_kb *kb = container_of(to_delayed_work(work), struct pogo_kb,
					  poll_work);

	mutex_lock(&kb->lock);

	switch (kb->state) {
	case POGO_KB_IDLE:
		break;
	case POGO_KB_DETECT:
		pogo_kb_sample(kb, POGO_KB_DETECT_ATTACH, POGO_KB_DETECT_ABORT,
			       POGO_KB_DETECT_PERIOD_MS);
		break;
	case POGO_KB_RECHECK:
		pogo_kb_sample(kb, POGO_KB_RECHECK_ATTACH, POGO_KB_RECHECK_DETACH,
			       POGO_KB_RECHECK_PERIOD_MS);
		break;
	case POGO_KB_POWERED:
	case POGO_KB_CONNECTED:
		pogo_kb_monitor(kb);
		break;
	}

	mutex_unlock(&kb->lock);
}

static void pogo_kb_attach_work(struct work_struct *work)
{
	struct pogo_kb *kb = container_of(work, struct pogo_kb, attach_work);
	u8 brand;
	int ret;

	mutex_lock(&kb->lock);

	if (kb->state != POGO_KB_POWERED)
		goto out;

	spin_lock(&kb->rx_lock);
	brand = kb->brand;
	spin_unlock(&kb->rx_lock);

	ret = pogo_kb_register_inputs(kb, brand);
	if (ret) {
		dev_err(kb->dev, "failed to register input devices: %d\n", ret);
		pogo_kb_power_down(kb);
		pogo_kb_enter_recheck(kb);
		goto out;
	}

	kb->state = POGO_KB_CONNECTED;
	kb->power_cycles = 0;
	kb->grace_start = jiffies;
	WRITE_ONCE(kb->led_sent,
		   kb->grace_start - msecs_to_jiffies(POGO_KB_LED_RESYNC_MS));
	pogo_kb_log_version(kb);
out:
	mutex_unlock(&kb->lock);
}

static void pogo_kb_led_work(struct work_struct *work)
{
	struct pogo_kb *kb = container_of(work, struct pogo_kb, led_work);
	int ret;

	mutex_lock(&kb->lock);

	if (kb->state == POGO_KB_CONNECTED) {
		/* Leave the line to the report that toggled the LED */
		msleep(POGO_KB_LED_DELAY_MS);
		ret = pogo_kb_send_leds(kb);
		WRITE_ONCE(kb->led_sent, jiffies);
		if (ret)
			dev_warn_ratelimited(kb->dev, "failed to set the LEDs: %d\n", ret);
	}

	mutex_unlock(&kb->lock);
}

static irqreturn_t pogo_kb_detect_irq(int irq, void *data)
{
	struct pogo_kb *kb = data;

	mutex_lock(&kb->lock);

	if (kb->state == POGO_KB_IDLE) {
		disable_irq_nosync(irq);
		kb->irq_enabled = false;
		kb->state = POGO_KB_DETECT;
		kb->asserted = 0;
		kb->released = 0;
		kb->power_cycles = 0;
		pogo_kb_schedule(kb, POGO_KB_DETECT_PERIOD_MS);
	}

	mutex_unlock(&kb->lock);

	return IRQ_HANDLED;
}

static int pogo_kb_probe(struct serdev_device *serdev)
{
	struct device *dev = &serdev->dev;
	struct pogo_kb *kb;
	unsigned int i;
	int ret;

	kb = devm_kzalloc(dev, sizeof(*kb), GFP_KERNEL);
	if (!kb)
		return -ENOMEM;

	kb->serdev = serdev;
	kb->dev = dev;
	kb->state = POGO_KB_IDLE;
	spin_lock_init(&kb->rx_lock);
	init_completion(&kb->echo_done);
	init_completion(&kb->reply_done);
	INIT_DELAYED_WORK(&kb->poll_work, pogo_kb_poll_work);
	INIT_WORK(&kb->attach_work, pogo_kb_attach_work);
	INIT_WORK(&kb->led_work, pogo_kb_led_work);

	ret = devm_mutex_init(dev, &kb->lock);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(kb->keymap); i++)
		kb->keymap[i] = pogo_kb_hid_keys[i];

	kb->speed = POGO_KB_SPEED;
	device_property_read_u32(dev, "current-speed", &kb->speed);

	kb->phys_kbd = devm_kasprintf(dev, GFP_KERNEL, "%s/input0", dev_name(dev));
	kb->phys_tp = devm_kasprintf(dev, GFP_KERNEL, "%s/input1", dev_name(dev));
	if (!kb->phys_kbd || !kb->phys_tp)
		return -ENOMEM;

	kb->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(kb->vdd))
		return dev_err_probe(dev, PTR_ERR(kb->vdd), "failed to get the supply\n");

	kb->tx_en = devm_gpiod_get(dev, "tx-enable", GPIOD_OUT_LOW);
	if (IS_ERR(kb->tx_en))
		return dev_err_probe(dev, PTR_ERR(kb->tx_en),
				     "failed to get the direction switch\n");

	kb->detect = devm_gpiod_get(dev, "detect", GPIOD_IN);
	if (IS_ERR(kb->detect))
		return dev_err_probe(dev, PTR_ERR(kb->detect),
				     "failed to get the sense line\n");

	kb->irq = fwnode_irq_get(dev_fwnode(dev), 0);
	if (kb->irq < 0)
		return dev_err_probe(dev, kb->irq, "failed to get the sense interrupt\n");

	serdev_device_set_drvdata(serdev, kb);
	serdev_device_set_client_ops(serdev, &pogo_kb_serdev_ops);

	ret = devm_request_threaded_irq(dev, kb->irq, NULL, pogo_kb_detect_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"pogo-keyboard-detect", kb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the sense interrupt\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
	}

	/* The supply starts off; look for a keyboard that is already attached */
	mutex_lock(&kb->lock);
	pogo_kb_enter_recheck(kb);
	mutex_unlock(&kb->lock);

	return 0;
}

static void pogo_kb_remove(struct serdev_device *serdev)
{
	struct pogo_kb *kb = serdev_device_get_drvdata(serdev);

	disable_irq(kb->irq);
	disable_delayed_work_sync(&kb->poll_work);
	disable_work_sync(&kb->attach_work);
	disable_work_sync(&kb->led_work);

	mutex_lock(&kb->lock);
	if (kb->state == POGO_KB_POWERED || kb->state == POGO_KB_CONNECTED)
		pogo_kb_power_down(kb);
	kb->state = POGO_KB_IDLE;
	mutex_unlock(&kb->lock);
}

static int pogo_kb_suspend(struct device *dev)
{
	struct pogo_kb *kb = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&kb->poll_work);
	cancel_work_sync(&kb->attach_work);
	cancel_work_sync(&kb->led_work);

	mutex_lock(&kb->lock);

	switch (kb->state) {
	case POGO_KB_CONNECTED:
		ret = pogo_kb_set_awake(kb, false);
		if (ret)
			dev_warn(dev, "keyboard did not confirm sleep: %d\n", ret);
		break;
	case POGO_KB_POWERED:
		pogo_kb_power_down(kb);
		fallthrough;
	case POGO_KB_DETECT:
	case POGO_KB_RECHECK:
		/* Restart the attach sequence after resume */
		kb->state = POGO_KB_RECHECK;
		kb->asserted = 0;
		kb->released = 0;
		break;
	case POGO_KB_IDLE:
		if (device_may_wakeup(dev) && !enable_irq_wake(kb->irq))
			kb->irq_wake = true;
		break;
	}

	mutex_unlock(&kb->lock);

	return 0;
}

static int pogo_kb_resume(struct device *dev)
{
	struct pogo_kb *kb = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&kb->lock);

	if (kb->irq_wake) {
		disable_irq_wake(kb->irq);
		kb->irq_wake = false;
	}

	if (kb->state == POGO_KB_CONNECTED) {
		spin_lock(&kb->rx_lock);
		pogo_kb_release_all(kb);
		spin_unlock(&kb->rx_lock);

		ret = pogo_kb_set_awake(kb, true);
		if (ret)
			dev_warn(dev, "keyboard did not confirm wake-up: %d\n", ret);

		kb->grace_start = jiffies;
		WRITE_ONCE(kb->last_rx, kb->grace_start);
	}

	if (kb->state != POGO_KB_IDLE)
		pogo_kb_schedule(kb, POGO_KB_MONITOR_PERIOD_MS);

	mutex_unlock(&kb->lock);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pogo_kb_pm_ops, pogo_kb_suspend, pogo_kb_resume);

static const struct of_device_id pogo_kb_of_match[] = {
	{ .compatible = "oneplus,pad2pro-pogo-keyboard" },
	{ }
};
MODULE_DEVICE_TABLE(of, pogo_kb_of_match);

static struct serdev_device_driver pogo_kb_driver = {
	.probe = pogo_kb_probe,
	.remove = pogo_kb_remove,
	.driver = {
		.name = "oneplus-pogo-keyboard",
		.of_match_table = pogo_kb_of_match,
		.pm = pm_sleep_ptr(&pogo_kb_pm_ops),
	},
};
module_serdev_device_driver(pogo_kb_driver);

MODULE_DESCRIPTION("OnePlus Pad 2 Pro pogo pin keyboard");
MODULE_LICENSE("GPL");
