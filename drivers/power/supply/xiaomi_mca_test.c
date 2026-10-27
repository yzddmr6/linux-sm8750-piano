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
#include <kunit/device.h>

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
	xiaomi_mca_build_request(&req, MCA_OPCODE_READ, 0x2008, 1, 0xdeadbeef);

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
	static const int types[] = {
		POWER_SUPPLY_USB_TYPE_UNKNOWN, POWER_SUPPLY_USB_TYPE_SDP,
		POWER_SUPPLY_USB_TYPE_CDP, POWER_SUPPLY_USB_TYPE_DCP,
		POWER_SUPPLY_USB_TYPE_UNKNOWN, POWER_SUPPLY_USB_TYPE_DCP,
		POWER_SUPPLY_USB_TYPE_DCP, POWER_SUPPLY_USB_TYPE_DCP,
		POWER_SUPPLY_USB_TYPE_DCP, POWER_SUPPLY_USB_TYPE_C,
		POWER_SUPPLY_USB_TYPE_PD, POWER_SUPPLY_USB_TYPE_PD,
		POWER_SUPPLY_USB_TYPE_PD_PPS, POWER_SUPPLY_USB_TYPE_UNKNOWN,
		POWER_SUPPLY_USB_TYPE_UNKNOWN, POWER_SUPPLY_USB_TYPE_UNKNOWN,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(types); i++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_type(i), types[i]);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_usb_type(U32_MAX), POWER_SUPPLY_USB_TYPE_UNKNOWN);

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
			POWER_SUPPLY_USB_TYPE_CDP);
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
		/* At 5 V downstream treats all QC types as DCP. */
		{ MCA_ICL, 5, 1500000, 0, 1500 },
		{ MCA_ICL, 5, 1500001, -EINVAL },
		/* PD and PPS at 5 V: in_pd 1600, in_pps 2500, chg_pd 3000 held to DCP */
		{ MCA_ICL, 10, 1500000, 0, 1500 },
		{ MCA_ICL, 10, 1600000, -EINVAL },
		{ MCA_ICL, 12, 2500000, -EINVAL },
		{ MCA_FCC, 10, 2000000, 0, 2000 },
		{ MCA_FCC, 12, 3500000, -EINVAL },
		/* OCP like float */
		{ MCA_ICL, 15, 1000000, 0, 1000 },
		/* No fallback current for unknown or unsupported types. */
		{ MCA_ICL, 0, 500000, -EOPNOTSUPP },
		{ MCA_FCC, 0, 100000, -EOPNOTSUPP },
		{ MCA_ICL, 13, 100000, -EOPNOTSUPP },
		{ MCA_FCC, 14, 100000, -EOPNOTSUPP },
		{ MCA_ICL, 16, 100000, -EOPNOTSUPP },
		{ MCA_FCC, 0xffffffff, 100000, -EOPNOTSUPP },
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

		KUNIT_EXPECT_EQ_MSG(test, xiaomi_mca_current_ma(c->psp, c->real_type,
							     false, c->ua, &ma),
				    c->ret, "case %u", i);
		KUNIT_EXPECT_EQ_MSG(test, ma, c->ret ? 0xdeadbeef : c->ma, "case %u", i);
	}
}

/* The FCC frame uses the same vendor int/mA encoding as ICL. */
static void xiaomi_mca_test_build_fcc(struct kunit *test)
{
	static const u32 sent[] = { 0x800a, 1, 2, 0x2005, 35, 2000 };
	struct xiaomi_mca_request req, expected = {};
	__le32 *w = (__le32 *)&expected;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sent); i++)
		w[i] = cpu_to_le32(sent[i]);
	xiaomi_mca_build_request(&req, MCA_OPCODE_WRITE, 0x2005, 35, 2000);
	KUNIT_EXPECT_MEMEQ(test, &req, &expected, sizeof(req));
}

static void xiaomi_mca_test_all_limits(struct kunit *test)
{
	/* Raw types 0..16, from this unit's stock buck configuration at 5 V. */
	static const u32 icl[] = {
		0, 500, 900, 1500, 1000, 1500, 1500, 1500, 1500,
		1500, 1500, 1500, 1500, 0, 0, 1000, 0,
	};
	static const u32 fcc[] = {
		0, 500, 900, 2000, 1000, 2000, 2000, 2000, 2000,
		2000, 2000, 2000, 2000, 0, 0, 1000, 0,
	};
	unsigned int type, prop;

	for (type = 0; type < ARRAY_SIZE(icl); type++) {
		for (prop = 0; prop < 2; prop++) {
			enum power_supply_property psp = prop ? MCA_FCC : MCA_ICL;
			u32 cap = prop ? fcc[type] : icl[type];
			u32 ma = 0xdeadbeef;

			if (!cap) {
				KUNIT_EXPECT_EQ(test,
					xiaomi_mca_current_ma(psp, type, false, 100000, &ma),
					-EOPNOTSUPP);
				KUNIT_EXPECT_EQ(test, ma, 0xdeadbeef);
				continue;
			}
			KUNIT_EXPECT_EQ(test,
				xiaomi_mca_current_ma(psp, type, false, cap * 1000, &ma), 0);
			KUNIT_EXPECT_EQ(test, ma, cap);
			KUNIT_EXPECT_EQ(test,
				xiaomi_mca_current_ma(psp, type, false, cap * 1000 + 1, &ma),
				-EINVAL);
			KUNIT_EXPECT_EQ(test,
				xiaomi_mca_current_ma(psp, type, false, 99999, &ma), -EINVAL);
		}
	}
}

static void xiaomi_mca_test_readonly_session(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mutex_init(&mca->lock);
	mutex_lock(&mca->lock);
	/* No client: this must return without sending even a read. */
	xiaomi_mca_restore_icl(mca);
	mutex_unlock(&mca->lock);
	KUNIT_EXPECT_EQ(test, mca->seq, 0);
	KUNIT_EXPECT_EQ(test, mca->stats.writes, 0);
	KUNIT_EXPECT_EQ(test, mca->stats.reads, 0);
	mutex_destroy(&mca->lock);
}

static void xiaomi_mca_test_writable_properties(struct kunit *test)
{
	unsigned int psp;

	/* Raw protocol writes cannot bypass the two-property whitelist. */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(NULL, 0x2003, 1), -EINVAL);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(NULL, 0x2007, 4350), -EINVAL);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(NULL, 0x21018, 9000), -EINVAL);

	for (psp = 0; psp <= POWER_SUPPLY_PROP_SERIAL_NUMBER; psp++)
		KUNIT_EXPECT_EQ(test,
			xiaomi_mca_usb_property_is_writeable(NULL, psp),
			psp == MCA_ICL || psp == MCA_FCC);
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
	/* The fixture allocator is reply-sized; a notification is 272 bytes. */
	xiaomi_mca_callback(mca_test_reply(test, &mca_test_notification),
			    sizeof(struct xiaomi_mca_notification), mca);
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

/* Community-policy tests use the production paths with only GLINK stubbed. */
struct mca_policy_fixture {
	struct xiaomi_mca mca;
	struct power_supply psy;
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 type, vbus, icl, fcc, tbat, verify;
	u32 last_prop;
	u8 last_data[MCA_VDM_LEN];
	unsigned int writes;
	unsigned int fcc_reads;
	bool fcc_read_zero;
	bool icl_read_stale;
	int error;
	int write_error;
	bool voltage_stuck;
	bool stage_on;
	int stage_error;
	unsigned int stage_stops;
	unsigned int voltage_requests;
	bool voltage_before_stop;
};

static int mca_policy_xfer(struct xiaomi_mca *mca, u32 op, u32 prop,
			   void *data, size_t len)
{
	struct mca_policy_fixture *f = container_of(mca, struct mca_policy_fixture, mca);
	u32 value = 0;

	if (f->error)
		return f->error;
	if (op == MCA_OPCODE_WRITE) {
		f->writes++;
		if (f->write_error)
			return f->write_error;
		f->last_prop = prop;
		memcpy(f->last_data, data, len);
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT)
			f->icl = get_unaligned_le32(data);
		if (prop == MCA_PROP_CHARGE_CURRENT)
			f->fcc = get_unaligned_le32(data);
		if (prop == MCA_PROP_PD_FIXED_VOLT && get_unaligned_le32(data) == 5000) {
			f->voltage_requests++;
			f->voltage_before_stop |= f->stage_on;
			if (!f->voltage_stuck)
				f->vbus = 5000000;
		}
		return 0;
	}
	if (prop == MCA_PROP_PDOS) {
		memcpy(data, f->pdos, len);
		return 0;
	}
	switch (prop) {
	case MCA_PROP_USB_REAL_TYPE:
		value = f->type;
		break;
	case MCA_PROP_USB_VOLTAGE_NOW:
		value = f->vbus;
		break;
	case MCA_PROP_INPUT_CURRENT_LIMIT:
		value = f->icl_read_stale ? 100 : f->icl;
		break;
	case MCA_PROP_CHARGE_CURRENT:
		f->fcc_reads++;
		value = f->fcc_read_zero ? 0 : f->fcc;
		break;
	case MCA_PROP_PACK_TBAT:
		value = f->tbat;
		break;
	case MCA_PROP_VERIFY_PROCESS:
		value = f->verify;
		break;
	default:
		break;
	}
	put_unaligned_le32(value, data);
	return 0;
}

static void mca_policy_cleanup(void *data)
{
	struct mca_policy_fixture *f = data;

	cancel_delayed_work_sync(&f->mca.poll_work);
	if (f->mca.cp_psy)
		power_supply_put(f->mca.cp_psy);
}

static struct mca_policy_fixture *mca_policy_fixture(struct kunit *test)
{
	struct mca_policy_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->mca.dev = kunit_device_register(test, "mca-policy-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->mca.dev);
	mutex_init(&f->mca.lock);
	spin_lock_init(&f->mca.rx_lock);
	init_completion(&f->mca.ack);
	INIT_DELAYED_WORK(&f->mca.poll_work, xiaomi_mca_poll);
	kunit_add_action_or_reset(test, mca_policy_cleanup, f);
	dev_set_drvdata(f->mca.dev, &f->mca);
	f->psy.drv_data = &f->mca;
	f->type = MCA_TYPE_PD;
	f->vbus = 5000000;
	f->icl = 100;
	f->fcc = 500;
	f->tbat = 25;
	f->mca.real_type = f->type;
	f->mca.vbus_uv = f->vbus;
	f->mca.cp_warm_ma = MCA_CP_IBAT_MAX_MA;
	kunit_activate_static_stub(test, xiaomi_mca_transfer_data, mca_policy_xfer);
	return f;
}

static void xiaomi_mca_test_policy_defaults(struct kunit *test)
{
	__le32 value = cpu_to_le32(9000);

	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_C), 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_FLOAT), 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_fcc(MCA_TYPE_C, false), 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_UNKNOWN), 0);
	KUNIT_EXPECT_FALSE(test, charge_policy);
	KUNIT_EXPECT_FALSE(test, charge_current);
	KUNIT_EXPECT_FALSE(test, hv_charge);
	KUNIT_EXPECT_FALSE(test, mipps_auth);
	KUNIT_EXPECT_FALSE(test, cp_charge);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_PD_FIXED_VOLT,
							&value, sizeof(value)));
}

static void xiaomi_mca_test_hv_fixed(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	hv_charge = true;
	f->pdos[0] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(2000) };
	mutex_lock(&mca->lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_REQUESTED);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_PD_FIXED_VOLT);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 9000);
	f->vbus = mca->vbus_uv = 9000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 2000);
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_ON);
	f->vbus = mca->vbus_uv = 10000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_BLOCKED);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 5000);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_hv_pps(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	hv_charge = true;
	f->pdos[0] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(5000), cpu_to_le32(11000), cpu_to_le32(5000) };
	mutex_lock(&mca->lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_PPS_SELECT);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), (450U << 16) | 60U);
	KUNIT_EXPECT_TRUE(test, mca->hv_pps);
	KUNIT_EXPECT_EQ(test, mca->hv_pps_ma, 3000);
	/* Auth activity (or a failed read) must prevent a voltage request. */
	mca->hv = MCA_HV_OFF;
	f->verify = 1;
	f->writes = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->error = -EIO;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_policy_refusal(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	unsigned int type;

	hv_charge = true;
	mutex_lock(&mca->lock);
	for (type = 0; type < 20; type++) {
		if (xiaomi_mca_limits(type))
			continue;
		f->type = type;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write_u32(mca, MCA_PROP_PD_FIXED_VOLT,
							 9000), -EOPNOTSUPP);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 500),
				-EOPNOTSUPP);
	}
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->type = MCA_TYPE_DCP;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_u32(mca, MCA_PROP_PD_FIXED_VOLT, 9000),
			-EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->type = MCA_TYPE_PD;
	f->vbus = 9000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 1600), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 1601), -EINVAL);
	f->type = MCA_TYPE_PD_PPS;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2500), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3000), 0);
	KUNIT_EXPECT_LT(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3500), 0);
	f->vbus = 5000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3000), -EINVAL);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_auth_relay(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	const char *seed = "4,0102030405060708090a0b0c0d0e0f10";
	const char *flag = "7,01000000";

	KUNIT_EXPECT_EQ(test, request_vdm_cmd_store(&f->psy.dev, NULL, seed,
						  strlen(seed)), -EPERM);
	mipps_auth = true;
	KUNIT_EXPECT_EQ(test, request_vdm_cmd_store(&f->psy.dev, NULL, seed,
						  strlen(seed)), strlen(seed));
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_VDM_SEED);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 0x01020304);
	KUNIT_EXPECT_EQ(test, request_vdm_cmd_store(&f->psy.dev, NULL, flag,
						  strlen(flag)), strlen(flag));
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_VDM_REMOVE_COMP);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 1);
	KUNIT_EXPECT_EQ(test, pd_verifed_store(&f->psy.dev, NULL, "1", 1), 1);
	KUNIT_EXPECT_TRUE(test, f->mca.auth_ok);
	f->type = 0;
	KUNIT_EXPECT_EQ(test, pd_verifed_store(&f->psy.dev, NULL, "1", 1), -EOPNOTSUPP);
	mipps_auth = false;
}

static void xiaomi_mca_test_temperature_follow(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca->hv = MCA_HV_ON;
	f->vbus = 9000000;
	mutex_lock(&mca->lock);
	xiaomi_mca_apply_fcc(mca, 25);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000);
	xiaomi_mca_apply_fcc(mca, 42);
	KUNIT_EXPECT_EQ(test, f->fcc, 1800);
	xiaomi_mca_apply_fcc(mca, 25);
	KUNIT_EXPECT_EQ(test, f->fcc, 1800); /* no immediate thermal oscillation */
	mca->fcc_raise_at = jiffies - 1;
	xiaomi_mca_apply_fcc(mca, 25);
	KUNIT_EXPECT_EQ(test, f->fcc, 2100);
	xiaomi_mca_apply_fcc(mca, -1);
	KUNIT_EXPECT_EQ(test, f->fcc, 500);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_CHARGE_CURRENT);
	mutex_unlock(&mca->lock);
}

static void xiaomi_mca_test_cp_limits(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 25, 25), 8000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4150, 25, 25), 7230);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 17, 25), 4500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 25, 40), 4500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 25, 25), 4500);
	mca->cp_warm_at = jiffies - msecs_to_jiffies(MCA_CP_TBAT_HOLD_MS) - 1;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 25, 25), 5000);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(8000, 4000)));
	cp_charge = true;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(8000, 4000)));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(11000, 4000)));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(8000, 6050)));
	cp_charge = false;
}

static void xiaomi_mca_test_fcc_ack_without_readback(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	u32 reported;

	f->fcc_read_zero = true;
	guard(mutex)(&f->mca.lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_read(&f->mca, MCA_PROP_CHARGE_CURRENT, &reported), 0);
	KUNIT_ASSERT_EQ(test, reported, 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_CHARGE_CURRENT, 500), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 1);
	KUNIT_EXPECT_EQ(test, f->fcc_reads, 1);
	/* A readable ICL setpoint must still agree with its request. */
	f->icl_read_stale = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 500), -EIO);
}

static void xiaomi_mca_test_fcc_request_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	f->error = -ETIMEDOUT;
	mca->fcc_set_ma = 900;
	guard(mutex)(&mca->lock);
	xiaomi_mca_reset_fcc(mca);
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 900);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->error = 0;
	f->write_error = -EIO;
	xiaomi_mca_reset_fcc(mca);
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 900);
	KUNIT_EXPECT_EQ(test, f->writes, 1);
}

static int mca_test_stage_get(struct power_supply *psy, enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct mca_policy_fixture *f = power_supply_get_drvdata(psy);

	val->intval = f->stage_on;
	return 0;
}

static int mca_test_stage_set(struct power_supply *psy, enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	struct mca_policy_fixture *f = power_supply_get_drvdata(psy);

	f->stage_stops++;
	if (f->stage_error)
		return f->stage_error;
	f->stage_on = val->intval;
	return 0;
}

static void mca_test_stage(struct kunit *test, struct mca_policy_fixture *f)
{
	static const enum power_supply_property props[] = { POWER_SUPPLY_PROP_ONLINE };
	static const struct power_supply_desc desc = {
		.name = "mca-pm-test-cp",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = props,
		.num_properties = ARRAY_SIZE(props),
		.get_property = mca_test_stage_get,
		.set_property = mca_test_stage_set,
	};
	struct power_supply_config cfg = { .drv_data = f };
	struct power_supply *psy;

	psy = devm_power_supply_register(f->mca.dev, &desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, psy);
	f->mca.cp_psy = power_supply_get_by_name(desc.name);
	KUNIT_ASSERT_PTR_EQ(test, f->mca.cp_psy, psy);
	f->mca.cp = MCA_CP_ON;
	f->stage_on = true;
}

static void xiaomi_mca_test_pm_gate(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	u32 value;

	f->mca.cache[MCA_USB_VOLTAGE_NOW].valid = true;
	f->mca.last_valid = true;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_FALSE(test, f->mca.last_valid);
	KUNIT_EXPECT_FALSE(test, schedule_delayed_work(&f->mca.poll_work, 0));
	KUNIT_EXPECT_FALSE(test, mod_delayed_work(system_percpu_wq, &f->mca.poll_work, 0));
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->mca.poll_work));
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_usb(&f->mca, MCA_USB_VOLTAGE_NOW, &value),
			-EHOSTDOWN);
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_CHARGE_CURRENT, 500),
				-EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	xiaomi_mca_complete(f->mca.dev);
	cancel_delayed_work_sync(&f->mca.poll_work);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_usb(&f->mca, MCA_USB_VOLTAGE_NOW, &value), 0);
	KUNIT_EXPECT_EQ(test, value, 5000000);
	KUNIT_EXPECT_TRUE(test, schedule_delayed_work(&f->mca.poll_work, 60 * HZ));
}

static void xiaomi_mca_test_pm_quiesce(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	mca_test_stage(test, f);
	f->vbus = 9000000;
	f->icl = 1600;
	f->fcc = 3000;
	f->mca.icl_written = true;
	f->mca.fcc_set_ma = f->fcc;
	f->mca.fcc_written = true;
	f->mca.policy_active = true;
	f->mca.voltage_written = true;
	/* Disabling new HV requests must not prevent return to 5 V. */
	KUNIT_ASSERT_FALSE(test, hv_charge);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_EQ(test, f->stage_stops, 1);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, NULL);
	KUNIT_EXPECT_EQ(test, f->icl, MCA_ICL_DEFAULT_MA);
	KUNIT_EXPECT_EQ(test, f->fcc, MCA_FCC_DEFAULT_MA);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_FALSE(test, f->voltage_before_stop);
	KUNIT_EXPECT_EQ(test, f->vbus, 5000000);
	KUNIT_EXPECT_TRUE(test, f->mca.sleeping);
}

static void xiaomi_mca_test_cp_stop_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct power_supply *psy;

	mca_test_stage(test, f);
	psy = f->mca.cp_psy;
	f->stage_error = -EIO;
	guard(mutex)(&f->mca.lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(&f->mca), -EIO);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, psy);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	xiaomi_mca_policy_poll(&f->mca);
	KUNIT_EXPECT_EQ(test, f->stage_stops, 2);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->stage_error = 0;
	xiaomi_mca_policy_poll(&f->mca);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, NULL);
}

static void xiaomi_mca_test_pm_refused(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	f->mca.fcc_set_ma = 900;
	f->mca.fcc_written = true;
	f->write_error = -EIO;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), -EIO);
	cancel_delayed_work_sync(&f->mca.poll_work);
	KUNIT_EXPECT_FALSE(test, f->mca.sleeping);
	KUNIT_EXPECT_EQ(test, f->mca.fcc_set_ma, 900);
	KUNIT_EXPECT_TRUE(test, schedule_delayed_work(&f->mca.poll_work, 60 * HZ));
	/* An attached source of unknown type must not silently drop a pending ICL restore. */
	f->mca.fcc_set_ma = 0;
	f->mca.fcc_written = false;
	f->mca.icl_written = true;
	f->write_error = 0;
	f->type = MCA_TYPE_UNKNOWN;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), -EOPNOTSUPP);
	cancel_delayed_work_sync(&f->mca.poll_work);
	KUNIT_EXPECT_TRUE(test, f->mca.icl_written);
	KUNIT_EXPECT_FALSE(test, f->mca.sleeping);
}

/* Charged once, then unplugged: no VBUS, and the firmware reports an Unknown source. */
static void xiaomi_mca_test_pm_no_source(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	f->type = MCA_TYPE_UNKNOWN;
	f->vbus = 0;
	f->icl = 500;
	f->fcc = 900;
	f->mca.icl_written = true;
	f->mca.icl_set_ma = 500;
	f->mca.fcc_set_ma = 900;
	f->mca.fcc_written = true;
	f->mca.voltage_written = true;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_TRUE(test, f->mca.sleeping);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	/* Nothing is dropped: the next recognized source gets the defaults. */
	KUNIT_EXPECT_TRUE(test, f->mca.icl_written);
	KUNIT_EXPECT_TRUE(test, f->mca.fcc_written);
	xiaomi_mca_complete(f->mca.dev);
	cancel_delayed_work_sync(&f->mca.poll_work);
	f->type = MCA_TYPE_SDP;
	f->vbus = 5000000;
	scoped_guard(mutex, &f->mca.lock)
		xiaomi_mca_policy_poll(&f->mca);
	KUNIT_EXPECT_EQ(test, f->icl, MCA_ICL_DEFAULT_MA);
	KUNIT_EXPECT_EQ(test, f->fcc, MCA_FCC_DEFAULT_MA);
	KUNIT_EXPECT_FALSE(test, f->mca.icl_written);
	KUNIT_EXPECT_FALSE(test, f->mca.fcc_written);
}

static void xiaomi_mca_test_pm_voltage_timeout(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	f->mca.policy_active = true;
	f->vbus = 9000000;
	f->mca.voltage_written = true;
	f->voltage_stuck = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), -ETIMEDOUT);
	cancel_delayed_work_sync(&f->mca.poll_work);
	KUNIT_EXPECT_FALSE(test, f->mca.sleeping);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_TRUE(test, schedule_delayed_work(&f->mca.poll_work, 60 * HZ));
}

static void xiaomi_mca_test_pm_lost_ack(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	f->write_error = -ETIMEDOUT;
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_CHARGE_CURRENT, 1500),
				-ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, f->mca.fcc_written);
	KUNIT_EXPECT_EQ(test, f->mca.fcc_set_ma, 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), -ETIMEDOUT);
	cancel_delayed_work_sync(&f->mca.poll_work);
	KUNIT_EXPECT_FALSE(test, f->mca.sleeping);
	f->write_error = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_FALSE(test, f->mca.fcc_written);
	KUNIT_EXPECT_EQ(test, f->fcc, MCA_FCC_DEFAULT_MA);
	xiaomi_mca_complete(f->mca.dev);
	cancel_delayed_work_sync(&f->mca.poll_work);

	/* Firmware may also have accepted a voltage request whose ACK was lost. */
	hv_charge = true;
	f->write_error = -ETIMEDOUT;
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_request_volt(&f->mca, 9000), -ETIMEDOUT);
	hv_charge = false;
	KUNIT_EXPECT_TRUE(test, f->mca.voltage_written);
	KUNIT_EXPECT_EQ(test, f->mca.hv, MCA_HV_OFF);
	f->write_error = 0;
	/* Still at 5 V does not mean the pending raise was cancelled. */
	f->vbus = 5000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_EQ(test, f->vbus, 5000000);
	KUNIT_EXPECT_FALSE(test, f->mca.voltage_written);
}


struct mca_otg_fixture {
	struct xiaomi_mca mca;
	u32 props[8];
	u8 data[8][16];
	size_t lengths[8];
	int writes;
	int fail_write;
	int cp_result;
	enum sc8541_otg_action actions[16];
	int calls;
	u8 plugin;
	u8 cc_toggle;
	bool restart_on_read;
	int reads;
	int read_error;
	u32 read_prop;
	size_t read_len;
	int polls;
};

static void mca_otg_test_poll(struct work_struct *work)
{
	struct xiaomi_mca *mca = container_of(to_delayed_work(work), struct xiaomi_mca,
					      poll_work);
	struct mca_otg_fixture *f = container_of(mca, struct mca_otg_fixture, mca);

	f->polls++;
}

static int mca_otg_test_cp(struct xiaomi_mca *mca, enum sc8541_otg_action action)
{
	struct mca_otg_fixture *f = container_of(mca, struct mca_otg_fixture, mca);

	if (f->calls < ARRAY_SIZE(f->actions))
		f->actions[f->calls++] = action;
	return f->cp_result;
}

static int mca_otg_test_xfer(struct xiaomi_mca *mca, u32 op, u32 prop,
			   void *data, size_t len)
{
	struct mca_otg_fixture *f = container_of(mca, struct mca_otg_fixture, mca);
	int i = f->writes;

	if (op == MCA_OPCODE_READ) {
		f->reads++;
		f->read_prop = prop;
		f->read_len = len;
		if (f->read_error)
			return f->read_error;
		if (prop == MCA_PROP_CC_TOGGLE && len == 1) {
			*(u8 *)data = f->cc_toggle;
			if (f->restart_on_read)
				mca->service_generation++;
			return 0;
		}
		if (prop == MCA_PROP_SNK_SRC_MODE && len == sizeof(u32)) {
			put_unaligned_le32(0x12345678, data);
			return 0;
		}
	}
	if (op == MCA_OPCODE_READ && prop == MCA_PROP_OTG_PLUGIN && len == 1) {
		*(u8 *)data = f->plugin;
		return 0;
	}
	if (op != MCA_OPCODE_WRITE || len > 16 || i >= ARRAY_SIZE(f->props))
		return -EINVAL;
	f->props[i] = prop;
	f->lengths[i] = len;
	memcpy(f->data[i], data, len);
	f->writes++;
	return f->fail_write == f->writes ? -EIO : 0;
}

static struct mca_otg_fixture *mca_otg_fixture(struct kunit *test)
{
	struct mca_otg_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->mca.dev = kunit_device_register(test, "mca-otg-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->mca.dev);
	mutex_init(&f->mca.lock);
	spin_lock_init(&f->mca.rx_lock);
	INIT_DELAYED_WORK(&f->mca.poll_work, mca_otg_test_poll);
	kunit_add_action_or_reset(test, xiaomi_mca_stop_poll, &f->mca);
	f->mca.piano_otg = true;
	f->mca.service_up = true;
	f->mca.service_generation = 1;
	f->cc_toggle = 0xa5;
	kunit_activate_static_stub(test, xiaomi_mca_transfer_data, mca_otg_test_xfer);
	kunit_activate_static_stub(test, xiaomi_mca_otg_cp, mca_otg_test_cp);
	return f;
}

static void mca_otg_send(struct xiaomi_mca *mca, u32 id, u32 data)
{
	struct xiaomi_mca_notification msg = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		/* Deliberately different from id to catch decoding hdr.opcode. */
		.hdr.opcode = cpu_to_le32(0x77),
		.id = cpu_to_le32(id),
	};

	put_unaligned_le32(data, msg.data);
	put_unaligned_le32(0x12345678, msg.data + 12);
	xiaomi_mca_callback(&msg, sizeof(msg), mca);
}

static void xiaomi_mca_test_otg_config(struct kunit *test)
{
	static const u8 expected[] = {
		0, 0, 0, 0, 1, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255,
	};
	static const u8 expected_cid[] = { 8, 7, 0, 0 };
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	u8 invalid = 2;

	guard(mutex)(&f->mca.lock);
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_ASSERT_EQ(test, f->writes, 3);
	KUNIT_EXPECT_EQ(test, f->props[0], 0x201c);
	KUNIT_EXPECT_EQ(test, f->lengths[0], 1);
	KUNIT_EXPECT_EQ(test, f->data[0][0], 1);
	KUNIT_EXPECT_EQ(test, f->props[1], 0x201a);
	KUNIT_EXPECT_EQ(test, f->lengths[1], 16);
	KUNIT_EXPECT_MEMEQ(test, f->data[1], expected, sizeof(expected));
	KUNIT_EXPECT_EQ(test, f->props[2], 0x201b);
	KUNIT_EXPECT_EQ(test, f->lengths[2], 4);
	KUNIT_EXPECT_MEMEQ(test, f->data[2], expected_cid, sizeof(expected_cid));
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_EXPECT_EQ(test, f->writes, 3);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x201c, &invalid, 1));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x201a, expected, 4));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x200f, expected, 4));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x2010, expected, 4));
}

static void xiaomi_mca_test_otg_restart(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);

	guard(mutex)(&f->mca.lock);
	xiaomi_mca_sync_config(&f->mca);
	xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_DOWN);
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_EXPECT_EQ(test, f->writes, 3);
	xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_UP);
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_ASSERT_EQ(test, f->writes, 6);
	KUNIT_EXPECT_EQ(test, f->props[3], 0x201c);
	KUNIT_EXPECT_EQ(test, f->props[4], 0x201a);
	KUNIT_EXPECT_EQ(test, f->props[5], 0x201b);
	KUNIT_EXPECT_MEMEQ(test, f->data[5], f->data[2], 4);
	KUNIT_EXPECT_EQ(test, f->mca.config_generation, 3);
}

static void xiaomi_mca_test_otg_config_failure(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	int fail;

	guard(mutex)(&f->mca.lock);
	for (fail = 1; fail <= 3; fail++) {
		f->writes = 0;
		f->fail_write = fail;
		xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_DOWN);
		xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_UP);
		xiaomi_mca_sync_config(&f->mca);
		KUNIT_EXPECT_EQ(test, f->writes, fail);
		KUNIT_EXPECT_EQ(test, f->mca.config_result, -EIO);
		xiaomi_mca_sync_config(&f->mca);
		KUNIT_EXPECT_EQ(test, f->writes, fail);
	}
}

static void xiaomi_mca_test_otg_absent(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);

	f->cp_result = -ENODEV;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_ASSERT_EQ(test, f->writes, 3);
	KUNIT_EXPECT_EQ(test, f->data[0][0], 0);
	KUNIT_EXPECT_TRUE(test, f->mca.cp_absent_warned);
	mca_otg_send(&f->mca, 10, 1);
	xiaomi_mca_process_notifications(&f->mca);
	KUNIT_EXPECT_EQ(test, f->calls, 2);
	KUNIT_EXPECT_EQ(test, f->actions[1], SC8541_OTG_REVERSE_ON);
}

static void xiaomi_mca_test_otg_dispatch(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);

	mca_otg_send(&f->mca, 10, 2);
	mca_otg_send(&f->mca, 10, 0);
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_process_notifications(&f->mca);
	KUNIT_ASSERT_EQ(test, f->calls, 2);
	KUNIT_EXPECT_EQ(test, f->actions[0], SC8541_OTG_REVERSE_ON);
	KUNIT_EXPECT_EQ(test, f->actions[1], SC8541_OTG_REVERSE_OFF);
	KUNIT_EXPECT_FALSE(test, f->mca.otg_active);
	KUNIT_EXPECT_EQ(test, f->mca.notify[0].id, 10);
	KUNIT_EXPECT_EQ(test, f->mca.notify[0].data[3], 0x12345678);
	KUNIT_EXPECT_GT(test, f->mca.notify[0].time_ns, 0);
	f->plugin = 1;
	mca_otg_send(&f->mca, 3, 1);
	xiaomi_mca_process_notifications(&f->mca);
	KUNIT_EXPECT_EQ(test, f->calls, 2);
	f->plugin = 0;
	mca_otg_send(&f->mca, 3, 0);
	xiaomi_mca_process_notifications(&f->mca);
	KUNIT_ASSERT_EQ(test, f->calls, 3);
	KUNIT_EXPECT_EQ(test, f->actions[2], SC8541_OTG_DETACH);
}

static void xiaomi_mca_test_otg_poll_and_stop(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);

	f->mca.poll_ready = true;
	mca_otg_send(&f->mca, 5, 0);
	flush_delayed_work(&f->mca.poll_work);
	KUNIT_EXPECT_EQ(test, f->polls, 1);
	xiaomi_mca_stop_poll(&f->mca);
	mca_otg_send(&f->mca, 5, 0);
	KUNIT_EXPECT_FALSE(test, delayed_work_pending(&f->mca.poll_work));
	KUNIT_EXPECT_EQ(test, f->polls, 1);
}

static void xiaomi_mca_test_otg_bad_and_stale(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct xiaomi_mca_notification msg = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.id = cpu_to_le32(10),
	};

	xiaomi_mca_callback(&msg, sizeof(msg) - 1, &f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.stats.malformed, 1);
	KUNIT_EXPECT_EQ(test, f->mca.notify_pending, 0);
	mca_otg_send(&f->mca, 10, 1);
	xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_DOWN);
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_process_notifications(&f->mca);
	KUNIT_EXPECT_EQ(test, f->calls, 0);
}

static void xiaomi_mca_test_cid_allowlist(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	__le32 cid = cpu_to_le32(0x708);

	KUNIT_EXPECT_TRUE(test, xiaomi_mca_write_allowed(0x201b, &cid, 4));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x201b, &cid, 1));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x201b, &cid, 3));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x21021, &cid, 1));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x21022, &cid, 4));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x202b, &cid, 4));
	guard(mutex)(&f->mca.lock);
	f->mca.piano_otg = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, 0x201b, &cid, 4),
			-EOPNOTSUPP);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	cid = cpu_to_le32(0x709);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(0x201b, &cid, 4));
}

static void xiaomi_mca_test_port_role_telemetry(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct power_supply *psy = kunit_kzalloc(test, sizeof(*psy), GFP_KERNEL);
	char *buf = kunit_kzalloc(test, PAGE_SIZE, GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, psy);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	psy->drv_data = &f->mca;
	KUNIT_EXPECT_EQ(test, cc_toggle_show(&psy->dev, NULL, buf), 4);
	KUNIT_EXPECT_STREQ(test, buf, "165\n");
	KUNIT_EXPECT_EQ(test, f->read_prop, 0x21021);
	KUNIT_EXPECT_EQ(test, f->read_len, 1);
	KUNIT_EXPECT_EQ(test, snk_src_mode_show(&psy->dev, NULL, buf), 10);
	KUNIT_EXPECT_STREQ(test, buf, "305419896\n");
	KUNIT_EXPECT_EQ(test, f->read_prop, 0x21022);
	KUNIT_EXPECT_EQ(test, f->read_len, 4);
	f->read_error = -EIO;
	KUNIT_EXPECT_EQ(test, cc_toggle_show(&psy->dev, NULL, buf), -EIO);
	KUNIT_EXPECT_EQ(test, snk_src_mode_show(&psy->dev, NULL, buf), -EIO);
	KUNIT_EXPECT_EQ(test, dev_attr_cc_toggle.attr.mode, 0644);
	KUNIT_EXPECT_EQ(test, dev_attr_snk_src_mode.attr.mode, 0444);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
}

static void xiaomi_mca_test_cc_toggle_values(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct power_supply psy = { .drv_data = &f->mca };
	static const char * const inputs[] = { "0\n", "1\n", "2\n", "-7\n", "+1\n" };
	static const u8 header[] = {
		0x0a, 0x80, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0,
		0x21, 0x10, 2, 0, 1, 0, 0, 0,
	};
	struct xiaomi_mca_request req;
	int i;

	for (i = 0; i < ARRAY_SIZE(inputs); i++) {
		f->cc_toggle = !!i;
		KUNIT_ASSERT_EQ(test, cc_toggle_store(&psy.dev, NULL, inputs[i],
						    strlen(inputs[i])), strlen(inputs[i]));
		KUNIT_ASSERT_EQ(test, f->writes, i + 1);
		KUNIT_EXPECT_EQ(test, f->props[i], 0x21021);
		KUNIT_ASSERT_EQ(test, f->lengths[i], 1);
		KUNIT_EXPECT_EQ(test, f->data[i][0], !!i);
		KUNIT_EXPECT_EQ(test, f->read_prop, 0x21021);
		KUNIT_EXPECT_EQ(test, f->read_len, 1);
		KUNIT_EXPECT_EQ(test, f->reads, i + 1);
		/* Same request builder and payload copy used by the transport. */
		xiaomi_mca_build_request(&req, MCA_OPCODE_WRITE, f->props[i], 1, 0);
		memcpy(req.data, f->data[i], f->lengths[i]);
		KUNIT_EXPECT_MEMEQ(test, &req, header, sizeof(header));
		KUNIT_EXPECT_EQ(test, req.data[0], !!i);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(req.data + 1, 0,
						    sizeof(req.data) - 1), NULL);
	}
}

static void xiaomi_mca_test_cc_toggle_errors(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct power_supply psy = { .drv_data = &f->mca };

	f->cc_toggle = 0;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -EIO);
	f->cc_toggle = 1;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "0", 1), -EIO);
	f->read_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -ETIMEDOUT);
	f->fail_write = 4;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -EIO);
	KUNIT_EXPECT_EQ(test, f->reads, 3); /* Failed writes have no readback. */
	f->read_error = 0;
	f->restart_on_read = true;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -ECONNRESET);
	f->mca.sleeping = true;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, f->writes, 5);
}

static void xiaomi_mca_test_cc_toggle_refusal(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct power_supply psy = { .drv_data = &f->mca };
	static const char * const invalid[] = { "", "true", "0x1", "1 0", "1junk" };
	__le32 word = cpu_to_le32(1);
	u8 value = 2;
	int i;

	for (i = 0; i < ARRAY_SIZE(invalid); i++)
		KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, invalid[i],
						    strlen(invalid[i])), -EINVAL);
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "2147483648", 10), -ERANGE);
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "-2147483649", 11), -ERANGE);
	f->mca.piano_otg = false;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -EOPNOTSUPP);
	guard(mutex)(&f->mca.lock);
	f->mca.piano_otg = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, 0x21021, &value, 1), -EPERM);
	value = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, 0x21021, &value, 0), -EPERM);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, 0x21021, &word, 4), -EPERM);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, 0x21022, &value, 1), -EPERM);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	KUNIT_EXPECT_EQ(test, f->reads, 0);
}

static void xiaomi_mca_test_cc_toggle_no_replay(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	struct power_supply psy = { .drv_data = &f->mca };
	int i;

	scoped_guard(mutex, &f->mca.lock)
		xiaomi_mca_sync_config(&f->mca);
	KUNIT_ASSERT_EQ(test, f->writes, 3);
	f->cc_toggle = 1;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), 1);
	guard(mutex)(&f->mca.lock);
	KUNIT_ASSERT_EQ(test, f->writes, 4);
	xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_DOWN);
	xiaomi_mca_pdr_notify(&f->mca, SERVREG_SERVICE_STATE_UP);
	KUNIT_EXPECT_EQ(test, f->writes, 4);
	xiaomi_mca_sync_config(&f->mca);
	KUNIT_ASSERT_EQ(test, f->writes, 7);
	for (i = 4; i < f->writes; i++)
		KUNIT_EXPECT_NE(test, f->props[i], 0x21021);
}

static struct kunit_case xiaomi_mca_test_cases[] = {
	KUNIT_CASE(xiaomi_mca_test_cc_toggle_values),
	KUNIT_CASE(xiaomi_mca_test_cc_toggle_errors),
	KUNIT_CASE(xiaomi_mca_test_cc_toggle_refusal),
	KUNIT_CASE(xiaomi_mca_test_cc_toggle_no_replay),
	KUNIT_CASE(xiaomi_mca_test_cid_allowlist),
	KUNIT_CASE(xiaomi_mca_test_port_role_telemetry),
	KUNIT_CASE(xiaomi_mca_test_otg_bad_and_stale),
	KUNIT_CASE(xiaomi_mca_test_otg_poll_and_stop),
	KUNIT_CASE(xiaomi_mca_test_otg_dispatch),
	KUNIT_CASE(xiaomi_mca_test_otg_absent),
	KUNIT_CASE(xiaomi_mca_test_otg_config_failure),
	KUNIT_CASE(xiaomi_mca_test_otg_restart),
	KUNIT_CASE(xiaomi_mca_test_otg_config),
	KUNIT_CASE(xiaomi_mca_test_policy_defaults),
	KUNIT_CASE(xiaomi_mca_test_hv_fixed),
	KUNIT_CASE(xiaomi_mca_test_hv_pps),
	KUNIT_CASE(xiaomi_mca_test_policy_refusal),
	KUNIT_CASE(xiaomi_mca_test_auth_relay),
	KUNIT_CASE(xiaomi_mca_test_temperature_follow),
	KUNIT_CASE(xiaomi_mca_test_cp_limits),
	KUNIT_CASE(xiaomi_mca_test_fcc_ack_without_readback),
	KUNIT_CASE(xiaomi_mca_test_fcc_request_failure),
	KUNIT_CASE(xiaomi_mca_test_pm_gate),
	KUNIT_CASE(xiaomi_mca_test_pm_quiesce),
	KUNIT_CASE(xiaomi_mca_test_cp_stop_failure),
	KUNIT_CASE(xiaomi_mca_test_pm_refused),
	KUNIT_CASE(xiaomi_mca_test_pm_no_source),
	KUNIT_CASE_SLOW(xiaomi_mca_test_pm_voltage_timeout),
	KUNIT_CASE(xiaomi_mca_test_pm_lost_ack),

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
	KUNIT_CASE(xiaomi_mca_test_build_fcc),
	KUNIT_CASE(xiaomi_mca_test_all_limits),
	KUNIT_CASE(xiaomi_mca_test_readonly_session),
	KUNIT_CASE(xiaomi_mca_test_writable_properties),
	{}
};

static struct kunit_suite xiaomi_mca_test_suite = {
	.name = "xiaomi-mca",
	.test_cases = xiaomi_mca_test_cases,
};
kunit_test_suite(xiaomi_mca_test_suite);
