// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the MCA request and reply handling, included by xiaomi_mca.c.
 *
 * The frames are those exchanged with the charger firmware of a Xiaomi Pad 8
 * Pro (7.2.6, 2026-10-09) and logged by a diagnostic battery manager build:
 * the GLINK header of each reply and its 268-byte payload as hex words. Only
 * the first four payload words are listed here; in all 121 logged read
 * replies the remaining 63 words were zero. The one write request, of the
 * input current limit, was logged as its first eight words with the note
 * that the rest of the 256-byte data area is zero; its reply as the 67
 * payload words, all zero after the fourth.
 */
#include <kunit/test.h>

#define MCA_TEST_WORDS	(sizeof(struct xiaomi_mca_reply) / sizeof(u32))

struct xiaomi_mca_test_frame {
	const char *name;
	u32 type;
	u32 opcode;
	/* property, return code, sequence number, first data word */
	u32 payload[4];
};

/* Replies to MCA reads (opcode 1) */
static const struct xiaomi_mca_test_frame mca_test_icl_seq1 = {
	"input current limit", PMIC_GLINK_REQ_RESP, 1, { 0x2008, 0, 1, 100 },
};

static const struct xiaomi_mca_test_frame mca_test_ibus_seq9 = {
	"USB current", PMIC_GLINK_REQ_RESP, 1, { 0x20004, 0, 9, 394000 },
};

static const struct xiaomi_mca_test_frame mca_test_vbus_seq10 = {
	"USB voltage", PMIC_GLINK_REQ_RESP, 1, { 0x20002, 0, 10, 5041000 },
};

static const struct xiaomi_mca_test_frame mca_test_online_seq11 = {
	"USB online", PMIC_GLINK_REQ_RESP, 1, { 0x20001, 0, 11, 1 },
};

static const struct xiaomi_mca_test_frame mca_test_real_type_seq5 = {
	"USB real type", PMIC_GLINK_REQ_RESP, 1, { 0x2000c, 0, 5, 1 },
};

static const struct xiaomi_mca_test_frame mca_test_ibus_done_seq48 = {
	"USB current, charge done", PMIC_GLINK_REQ_RESP, 1, { 0x20004, 0, 48, 0 },
};

static const struct xiaomi_mca_test_frame mca_test_icl_seq41 = {
	"input current limit after a write", PMIC_GLINK_REQ_RESP, 1, { 0x2008, 0, 41, 500 },
};

/* Other messages the firmware sent on the battery manager owner */
static const struct xiaomi_mca_test_frame mca_test_battmgr_get = {
	"battery manager property read", PMIC_GLINK_REQ_RESP, 0x30, { 0, 1, 0, 0 },
};

static const struct xiaomi_mca_test_frame mca_test_notification = {
	"notification request", PMIC_GLINK_NOTIFY, 0x4, { 0, 1, 0, 0 },
};

static const struct xiaomi_mca_test_frame mca_test_write_seq34 = {
	"MCA write", PMIC_GLINK_REQ_RESP, 2, { 0x2008, 0, 34, 1 },
};

static void *mca_test_reply(struct kunit *test, const struct xiaomi_mca_test_frame *frame)
{
	__le32 *w = kunit_kzalloc(test, sizeof(struct xiaomi_mca_reply), GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, w);
	w[0] = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR);
	w[1] = cpu_to_le32(frame->type);
	w[2] = cpu_to_le32(frame->opcode);
	for (i = 0; i < ARRAY_SIZE(frame->payload); i++)
		w[3 + i] = cpu_to_le32(frame->payload[i]);

	return w;
}

static int mca_test_parse_read(const void *reply, size_t len, u32 property, u32 seq,
			       u32 *value)
{
	return xiaomi_mca_parse_reply(reply, len, MCA_OPCODE_READ, property, seq, value);
}

/* The read request the diagnostic build sent first: property 0x2008, sequence 1. */
static void xiaomi_mca_test_build_read(struct kunit *test)
{
	static const u32 sent[] = { 0x800a, 1, 1, 0x2008, 1 };
	struct xiaomi_mca_request req, expected = {};
	__le32 *w = (__le32 *)&expected;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sent); i++)
		w[i] = cpu_to_le32(sent[i]);

	memset(&req, 0xa5, sizeof(req));
	xiaomi_mca_build_request(&req, MCA_OPCODE_READ, 0x2008, 1, 0);

	KUNIT_EXPECT_EQ(test, sizeof(req), 276);
	KUNIT_EXPECT_MEMEQ(test, &req, &expected, sizeof(req));
}

/*
 * The one write the firmware accepted, in the format of the stock driver:
 * input current limit 500 mA, sequence number 34. The encoder must reproduce
 * all 276 bytes of it.
 */
static void xiaomi_mca_test_build_write(struct kunit *test)
{
	static const u32 sent[] = { 0x800a, 1, 2, 0x2008, 0x22, 0x1f4, 0, 0 };
	struct xiaomi_mca_request req, expected = {};
	__le32 *w = (__le32 *)&expected;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sent); i++)
		w[i] = cpu_to_le32(sent[i]);

	memset(&req, 0xa5, sizeof(req));
	xiaomi_mca_build_request(&req, MCA_OPCODE_WRITE, 0x2008, 34, 500);

	KUNIT_EXPECT_MEMEQ(test, &req, &expected, sizeof(req));
}

/* The firmware's reply to that write: return code 0 and a first data word of 1. */
static void xiaomi_mca_test_write_reply(struct kunit *test)
{
	void *reply = mca_test_reply(test, &mca_test_write_seq34);
	u32 value = 0xdeadbeef;

	KUNIT_EXPECT_EQ(test, xiaomi_mca_parse_reply(reply, 280, MCA_OPCODE_WRITE, 0x2008, 34,
						     &value), 0);
	KUNIT_EXPECT_EQ(test, value, 1);

	/* not the reply to another write, nor to the read-back that follows */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_parse_reply(reply, 280, MCA_OPCODE_WRITE, 0x2008, 35,
						     &value), -ENOMSG);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_parse_reply(reply, 280, MCA_OPCODE_WRITE, 0x2005, 34,
						     &value), -EPROTO);

	reply = mca_test_reply(test, &mca_test_icl_seq41);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_parse_reply(reply, 280, MCA_OPCODE_WRITE, 0x2008, 41,
						     &value), -ENOMSG);

	reply = mca_test_reply(test, &mca_test_write_seq34);
	((__le32 *)reply)[4] = cpu_to_le32(1);
	value = 0xdeadbeef;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_parse_reply(reply, 280, MCA_OPCODE_WRITE, 0x2008, 34,
						     &value), -EIO);
	KUNIT_EXPECT_EQ(test, value, 0xdeadbeef);
}

static void xiaomi_mca_test_read_replies(struct kunit *test)
{
	static const struct xiaomi_mca_test_frame *frames[] = {
		&mca_test_icl_seq1, &mca_test_ibus_seq9, &mca_test_vbus_seq10,
		&mca_test_online_seq11, &mca_test_real_type_seq5,
		&mca_test_ibus_done_seq48, &mca_test_icl_seq41,
	};
	unsigned int i;

	KUNIT_EXPECT_EQ(test, MCA_TEST_WORDS, 70);

	for (i = 0; i < ARRAY_SIZE(frames); i++) {
		const struct xiaomi_mca_test_frame *f = frames[i];
		void *reply = mca_test_reply(test, f);
		u32 value = 0xdeadbeef;

		KUNIT_EXPECT_EQ_MSG(test, mca_test_parse_read(reply, 280, f->payload[0],
							      f->payload[2], &value),
				    0, "%s", f->name);
		KUNIT_EXPECT_EQ_MSG(test, value, f->payload[3], "%s", f->name);
	}
}

static void xiaomi_mca_test_other_request(struct kunit *test)
{
	void *reply = mca_test_reply(test, &mca_test_vbus_seq10);
	u32 value = 0xdeadbeef;

	/* a late reply to an earlier read is not the answer */
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 280, 0x20002, 11, &value),
			-ENOMSG);
	/* the right sequence number but another property */
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 280, 0x20004, 10, &value),
			-EPROTO);
	KUNIT_EXPECT_EQ(test, value, 0xdeadbeef);
}

static void xiaomi_mca_test_not_a_read(struct kunit *test)
{
	static const struct xiaomi_mca_test_frame *frames[] = {
		&mca_test_battmgr_get, &mca_test_notification, &mca_test_write_seq34,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(frames); i++) {
		const struct xiaomi_mca_test_frame *f = frames[i];
		void *reply = mca_test_reply(test, f);
		u32 value = 0xdeadbeef;

		KUNIT_EXPECT_EQ_MSG(test, mca_test_parse_read(reply, 280, f->payload[0],
							      f->payload[2], &value),
				    -ENOMSG, "%s", f->name);
		KUNIT_EXPECT_EQ(test, value, 0xdeadbeef);
	}
}

/*
 * The firmware fails a request with a non-zero return code; it did so for
 * every battery manager request, in the same word.
 */
static void xiaomi_mca_test_firmware_error(struct kunit *test)
{
	__le32 *reply = mca_test_reply(test, &mca_test_icl_seq1);
	u32 value = 0xdeadbeef;

	reply[4] = cpu_to_le32(1);
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 280, 0x2008, 1, &value), -EIO);
	KUNIT_EXPECT_EQ(test, value, 0xdeadbeef);
}

static void xiaomi_mca_test_short_reply(struct kunit *test)
{
	void *reply = mca_test_reply(test, &mca_test_icl_seq1);
	u32 value = 0xdeadbeef;

	/* the 268-byte payload the battery manager driver expected as 12 */
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 276, 0x2008, 1, &value), -EPROTO);
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 281, 0x2008, 1, &value), -EPROTO);
	/* too short to carry a sequence number */
	KUNIT_EXPECT_EQ(test, mca_test_parse_read(reply, 23, 0x2008, 1, &value), -ENOMSG);
	KUNIT_EXPECT_EQ(test, value, 0xdeadbeef);
}

static void xiaomi_mca_test_usb_values(struct kunit *test)
{
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_ONLINE, 1), 1);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_ONLINE, 0), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_VOLTAGE_NOW, 5041000), 5041000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_CURRENT_NOW, 394000), 394000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_INPUT_CURRENT_LIMIT, 100), 100000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_INPUT_CURRENT_LIMIT, 500), 500000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_REAL_TYPE, 1),
			POWER_SUPPLY_USB_TYPE_SDP);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_REAL_TYPE, 0),
			POWER_SUPPLY_USB_TYPE_UNKNOWN);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_REAL_TYPE, 2),
			POWER_SUPPLY_USB_TYPE_UNKNOWN);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_CHARGE_CURRENT, 0), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_propval(MCA_USB_CHARGE_CURRENT, 2000), 2000000);
}

struct mca_test_current {
	enum power_supply_property psp;
	u32 real_type;
	int ua;
	int ret;
	u32 ma;
};

#define MCA_ICL	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT
#define MCA_FCC	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT

/* Writes stay within the stock values for the charger type, 5 V types only. */
static void xiaomi_mca_test_current_limits(struct kunit *test)
{
	static const struct mca_test_current cases[] = {
		/* SDP: in_sdp 500, chg_sdp 500 */
		{ MCA_ICL, 1, 500000, 0, 500 },
		{ MCA_ICL, 1, 501000, -EINVAL },
		{ MCA_FCC, 1, 500000, 0, 500 },
		{ MCA_FCC, 1, 1000000, -EINVAL },
		/* CDP: in_cdp 900, chg_cdp 900 */
		{ MCA_ICL, 2, 900000, 0, 900 },
		{ MCA_ICL, 2, 1000000, -EINVAL },
		/* DCP: in_dcp 1500, chg_dcp 2000 */
		{ MCA_ICL, 3, 1500000, 0, 1500 },
		{ MCA_ICL, 3, 1500001, -EINVAL },
		{ MCA_ICL, 3, 1600000, -EINVAL },
		{ MCA_FCC, 3, 2000000, 0, 2000 },
		{ MCA_FCC, 3, 2001000, -EINVAL },
		{ MCA_FCC, 3, 3000000, -EINVAL },
		/* sub-mA values round down */
		{ MCA_ICL, 3, 1234567, 0, 1234 },
		/* float: in_float 1000, chg_float 1000 */
		{ MCA_ICL, 4, 1000000, 0, 1000 },
		{ MCA_ICL, 4, 1100000, -EINVAL },
		/* HVDCP2: in_hvdcp 1200 is below in_dcp */
		{ MCA_ICL, 5, 1200000, 0, 1200 },
		{ MCA_ICL, 5, 1300000, -EINVAL },
		/* PD and PPS at 5 V: in_pd 1600, in_pps 2500, chg_pd 3000 held to DCP */
		{ MCA_ICL, 10, 1500000, 0, 1500 },
		{ MCA_ICL, 10, 1600000, -EINVAL },
		{ MCA_ICL, 12, 2500000, -EINVAL },
		{ MCA_FCC, 10, 2000000, 0, 2000 },
		{ MCA_FCC, 12, 3500000, -EINVAL },
		/* OCP like float */
		{ MCA_ICL, 15, 1000000, 0, 1000 },
		/* unknown, ACA, unnamed and out-of-table types: SDP values */
		{ MCA_ICL, 0, 500000, 0, 500 },
		{ MCA_ICL, 0, 900000, -EINVAL },
		{ MCA_ICL, 13, 900000, -EINVAL },
		{ MCA_ICL, 14, 900000, -EINVAL },
		{ MCA_ICL, 16, 900000, -EINVAL },
		{ MCA_ICL, 0xffffffff, 900000, -EINVAL },
		/* the 100 mA floor */
		{ MCA_ICL, 3, 100000, 0, 100 },
		{ MCA_ICL, 3, 99999, -EINVAL },
		{ MCA_FCC, 3, 0, -EINVAL },
		{ MCA_FCC, 3, -500000, -EINVAL },
		{ MCA_ICL, 3, INT_MIN, -EINVAL },
		{ MCA_ICL, 3, INT_MAX, -EINVAL },
		/* nothing else is written */
		{ POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, 3, 1000000, -EINVAL },
		{ POWER_SUPPLY_PROP_ONLINE, 3, 1000000, -EINVAL },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		const struct mca_test_current *c = &cases[i];
		u32 ma = 0xdeadbeef;

		KUNIT_EXPECT_EQ_MSG(test, xiaomi_mca_current_ma(c->psp, c->real_type, c->ua, &ma),
				    c->ret, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, ma, c->ret ? 0xdeadbeef : c->ma, "case %u", i);
	}
}

/* A pending write is completed by its reply only, not by a read reply. */
static void xiaomi_mca_test_callback_write(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mca);
	init_completion(&mca->ack);
	spin_lock_init(&mca->rx_lock);
	mca->pending = true;
	mca->pending_opcode = MCA_OPCODE_WRITE;
	mca->pending_property = 0x2008;
	mca->pending_seq = 34;

	xiaomi_mca_callback(mca_test_reply(test, &mca_test_icl_seq41), 280, mca);
	KUNIT_EXPECT_FALSE(test, mca->answered);

	xiaomi_mca_callback(mca_test_reply(test, &mca_test_write_seq34), 280, mca);
	KUNIT_EXPECT_TRUE(test, mca->answered);
	KUNIT_EXPECT_TRUE(test, completion_done(&mca->ack));
	KUNIT_EXPECT_EQ(test, mca->result, 0);
	KUNIT_EXPECT_EQ(test, mca->stats.replies, 1);
	KUNIT_EXPECT_EQ(test, mca->stats.unmatched, 1);
}

/* Only the reply to the pending read completes it, and only once. */
static void xiaomi_mca_test_callback(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mca);
	init_completion(&mca->ack);
	spin_lock_init(&mca->rx_lock);
	mca->pending = true;
	mca->pending_opcode = MCA_OPCODE_READ;
	mca->pending_property = 0x2008;
	mca->pending_seq = 1;

	xiaomi_mca_callback(mca_test_reply(test, &mca_test_battmgr_get), 280, mca);
	xiaomi_mca_callback(mca_test_reply(test, &mca_test_notification), 280, mca);
	xiaomi_mca_callback(mca_test_reply(test, &mca_test_vbus_seq10), 280, mca);
	KUNIT_EXPECT_FALSE(test, mca->answered);
	KUNIT_EXPECT_FALSE(test, completion_done(&mca->ack));

	xiaomi_mca_callback(mca_test_reply(test, &mca_test_icl_seq1), 280, mca);
	KUNIT_EXPECT_TRUE(test, mca->answered);
	KUNIT_EXPECT_TRUE(test, completion_done(&mca->ack));
	KUNIT_EXPECT_EQ(test, mca->result, 0);
	KUNIT_EXPECT_EQ(test, mca->value, 100);
	KUNIT_EXPECT_EQ(test, mca->reply_len, 280);

	xiaomi_mca_callback(mca_test_reply(test, &mca_test_icl_seq1), 280, mca);

	KUNIT_EXPECT_EQ(test, mca->stats.replies, 1);
	KUNIT_EXPECT_EQ(test, mca->stats.unmatched, 3);
	KUNIT_EXPECT_EQ(test, mca->stats.notifications, 1);
	KUNIT_EXPECT_EQ(test, mca->stats.errors, 0);
	KUNIT_EXPECT_EQ(test, mca->stats.malformed, 0);
}

static struct kunit_case xiaomi_mca_test_cases[] = {
	KUNIT_CASE(xiaomi_mca_test_build_read),
	KUNIT_CASE(xiaomi_mca_test_build_write),
	KUNIT_CASE(xiaomi_mca_test_read_replies),
	KUNIT_CASE(xiaomi_mca_test_write_reply),
	KUNIT_CASE(xiaomi_mca_test_other_request),
	KUNIT_CASE(xiaomi_mca_test_not_a_read),
	KUNIT_CASE(xiaomi_mca_test_firmware_error),
	KUNIT_CASE(xiaomi_mca_test_short_reply),
	KUNIT_CASE(xiaomi_mca_test_usb_values),
	KUNIT_CASE(xiaomi_mca_test_current_limits),
	KUNIT_CASE(xiaomi_mca_test_callback),
	KUNIT_CASE(xiaomi_mca_test_callback_write),
	{}
};

static struct kunit_suite xiaomi_mca_test_suite = {
	.name = "xiaomi-mca",
	.test_cases = xiaomi_mca_test_cases,
};
kunit_test_suite(xiaomi_mca_test_suite);
