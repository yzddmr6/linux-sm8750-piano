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
 * This driver reports the USB input as a power supply and a few charger
 * properties in debugfs. Two settings can be written through the power
 * supply: the input current limit and the battery charge current, each only
 * up to the value Xiaomi's own charging stack uses for the charger type the
 * firmware detected. Nothing else is written; float voltage, termination,
 * temperature limits and enabling charging stay with the firmware. The
 * driver does not request notifications. The battery is reported by the
 * fuel gauges on I2C.
 *
 * The firmware sends no message on its own and refuses the battery manager
 * notification request, so the input state is polled to notice changes.
 */

#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/devm-helpers.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/seq_file.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#define MCA_OPCODE_READ			1
#define MCA_OPCODE_WRITE		2

/* Charger (buck) properties, currents in mA */
#define MCA_PROP_CHGR_STATUS		0x2001
#define MCA_PROP_CHARGING_ENABLED	0x2003
#define MCA_PROP_CHARGE_CURRENT		0x2005
#define MCA_PROP_INPUT_CURRENT_LIMIT	0x2008
#define MCA_PROP_AICL_ENABLED		0x2012
#define MCA_PROP_INPUT_SUSPEND		0x2019
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
 * This driver never requests a higher voltage; QC and PD are held to the
 * 5 V DCP values (in_dcp 1500, chg_dcp 2000). Types not listed cannot write.
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
	struct xiaomi_mca_state icl_source;

	struct delayed_work poll_work;

	/* orders the fields below against the GLINK callbacks */
	spinlock_t rx_lock;
	bool service_up;
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
static int xiaomi_mca_current_ma(enum power_supply_property psp, u32 real_type, int ua,
				 u32 *ma)
{
	const struct xiaomi_mca_limits *limits = xiaomi_mca_limits(real_type);
	u32 max;

	if (!limits)
		return -EOPNOTSUPP;

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

static int xiaomi_mca_transfer(struct xiaomi_mca *mca, u32 opcode, u32 property,
			       u32 *value)
{
	int ret;

	lockdep_assert_held(&mca->lock);

	xiaomi_mca_build_request(&mca->req, opcode, property, ++mca->seq, *value);
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

	*value = mca->value;

	return 0;
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

/*
 * Write @value to @property and read it back. The firmware acknowledges a
 * write with a return code only, so the value it then reports is checked.
 */
static int xiaomi_mca_write(struct xiaomi_mca *mca, u32 property, u32 value)
{
	u32 data = value;
	int ret;

	if (property != MCA_PROP_INPUT_CURRENT_LIMIT && property != MCA_PROP_CHARGE_CURRENT)
		return -EINVAL;

	ret = xiaomi_mca_transfer(mca, MCA_OPCODE_WRITE, property, &data);
	/* Whatever the outcome, cached values may be stale now. */
	xiaomi_mca_invalidate(mca);
	if (ret)
		return ret;

	ret = xiaomi_mca_read(mca, property, &data);
	if (ret)
		return ret;

	if (data != value) {
		dev_warn(mca->dev, "%#x reads %u after writing %u\n", property, data, value);
		return -EIO;
	}

	return 0;
}

/*
 * The firmware keeps the input current limit when the source changes, and
 * the limit written may be too high for the next one. Put back the value the
 * firmware starts with, as the downstream stack does on detach. On failure
 * the next poll tries again.
 */
static void xiaomi_mca_restore_icl(struct xiaomi_mca *mca)
{
	u32 real_type;
	int ret;

	lockdep_assert_held(&mca->lock);

	if (!mca->icl_written)
		return;

	/* Unknown also forbids automatic writes; wait for a recognized source. */
	ret = xiaomi_mca_read(mca, MCA_PROP_USB_REAL_TYPE, &real_type);
	if (ret || !xiaomi_mca_limits(real_type))
		return;

	ret = xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, MCA_ICL_DEFAULT_MA);
	if (ret) {
		dev_warn_ratelimited(mca->dev,
				     "failed to restore the input current limit: %d\n", ret);
		return;
	}

	mca->icl_written = false;
	dev_info(mca->dev, "input current limit restored to %u mA\n", MCA_ICL_DEFAULT_MA);
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

	ret = xiaomi_mca_current_ma(psp, source.real_type, val->intval, &ma);
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

static void xiaomi_mca_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct xiaomi_mca *mca = priv;
	u32 value = 0;
	int ret;

	guard(spinlock_irqsave)(&mca->rx_lock);

	if (le32_to_cpu(hdr->type) == PMIC_GLINK_NOTIFY) {
		mca->stats.notifications++;
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
	mca->service_up = state == SERVREG_SERVICE_STATE_UP;
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
	bool changed = false;

	scoped_guard(mutex, &mca->lock) {
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

		/* The uevent must not report what was cached before the change. */
		if (changed)
			xiaomi_mca_invalidate(mca);
	}

	if (changed)
		power_supply_changed(mca->psy);

	schedule_delayed_work(&mca->poll_work, MCA_POLL_INTERVAL);
}

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

static void xiaomi_mca_debugfs_remove(void *data)
{
	debugfs_remove_recursive(data);
}

/* The next driver, or none, must not inherit a limit set for this source. */
static void xiaomi_mca_unbind_restore(void *data)
{
	struct xiaomi_mca *mca = data;

	guard(mutex)(&mca->lock);
	xiaomi_mca_restore_icl(mca);
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

	psy_cfg.drv_data = mca;
	psy_cfg.fwnode = dev_fwnode(dev);
	mca->psy = devm_power_supply_register(dev, &xiaomi_mca_usb_psy_desc, &psy_cfg);
	if (IS_ERR(mca->psy))
		return dev_err_probe(dev, PTR_ERR(mca->psy),
				     "failed to register USB power supply\n");

	/* Cancelled before the power supply is unregistered. */
	ret = devm_delayed_work_autocancel(dev, &mca->poll_work, xiaomi_mca_poll);
	if (ret)
		return ret;

	mca->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0400, mca->debugfs, mca, &xiaomi_mca_status_fops);
	debugfs_create_file("stats", 0400, mca->debugfs, mca, &xiaomi_mca_stats_fops);
	debugfs_create_file("frames", 0400, mca->debugfs, mca, &xiaomi_mca_frames_fops);
	ret = devm_add_action_or_reset(dev, xiaomi_mca_debugfs_remove, mca->debugfs);
	if (ret)
		return ret;

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
};
module_auxiliary_driver(xiaomi_mca_driver);

MODULE_DESCRIPTION("Xiaomi MCA charger firmware interface over PMIC GLINK");
MODULE_LICENSE("GPL");

#if IS_ENABLED(CONFIG_CHARGER_XIAOMI_MCA_KUNIT_TEST)
#include "xiaomi_mca_test.c"
#endif
