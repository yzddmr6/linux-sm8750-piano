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
	struct { u32 prop, value; } trace[256];
	u32 trace_count;
	struct xiaomi_mca mca;
	struct power_supply psy;
	struct xiaomi_mca_pdo pdos[MCA_PD_MAX_PDOS];
	u32 type, vbus, icl, fcc, tbat, verify, verified, svid;
	u32 pd_state, vdm_state, adapter_id, actual_id, vdm_version;
	int ucsi_online, ucsi_type, ucsi_status, ucsi_charge_type;
	__le32 auth_response[4];
	u32 vdm_order[16], vdm_count, seed_index;
	bool simulate_auth, bad_digest, fail_verify, fail_clear;
	bool restart_auth, detach_auth;
	u32 last_prop;
	u8 last_data[MCA_VDM_LEN];
	unsigned int writes;
	unsigned int fcc_reads;
	unsigned int pack_reads;
	bool fcc_read_zero;
	bool icl_read_stale;
	u32 term_readback, term_current, pack_vbat;
	u8 input_suspend; /* legacy direct-HIZ state, getter is not implemented */
	u32 hiz_clear_writes, hiz_assert_writes, aicl_restarts;
	u32 fail_pmic_prop;
	int pack_vbat_error;
	bool resumed_before_stop;
	unsigned int term_writes;
	bool term_stuck;
	bool fg_full_ready;
	unsigned int fg_commands[2][3];
	int fg_command_error[2];
	bool term_restart;
	int pack_error;
	int vbus_error;
	u32 usb_online;
	int online_error;
	u8 charging_enabled;
	bool enable_stuck;
	unsigned int enable_writes;
	struct {
		int temp;
		int capacity;
		int voltage;
		int current_ua;
		int charge_now, charge_full;
		int error;
	} cells[2];
	int error;
	int write_error;
	bool voltage_stuck;
	int ucsi_rp_ua, ucsi_mv;
	u32 chgr_status;
	unsigned int transfers, online_reads, type_reads;
	int type_error;
	u8 aicl_enabled;
	u32 fail_aicl_prop;
	unsigned int aicl_runs;
	int stage_mv, stage_ma, stage_temp;
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

	f->transfers++;
	if (f->error)
		return f->error;
	if (prop == MCA_PROP_AICL_ENABLE || prop == MCA_PROP_RERUN_AICL ||
	    prop == MCA_PROP_RESTART_AICL) {
		if (len != 1)
			return -EINVAL;
		if (f->fail_aicl_prop == prop)
			return -EIO;
		if (op == MCA_OPCODE_READ)
			*(u8 *)data = f->aicl_enabled;
		else if (prop == MCA_PROP_AICL_ENABLE)
			f->aicl_enabled = *(u8 *)data;
		else if (prop == MCA_PROP_RESTART_AICL)
			f->aicl_restarts++;
		else
			f->aicl_runs++;
		return 0;
	}
	if (op == MCA_OPCODE_WRITE) {
		if (f->trace_count < ARRAY_SIZE(f->trace)) {
			f->trace[f->trace_count].prop = prop;
			f->trace[f->trace_count++].value = len == 1 ? *(u8 *)data : get_unaligned_le32(data);
		}
		f->writes++;
		if (prop == MCA_PROP_TERM_VOLT)
			f->term_writes++;
		if (f->write_error)
			return f->write_error;
		if (prop == MCA_PROP_TERM_VOLT) {
			if (f->term_restart)
				mca->service_generation++;
			if (!f->term_stuck)
				f->term_readback = 2 * get_unaligned_le32(data) - 3600;
		}
		if (prop == MCA_PROP_TERM_CURRENT)
			f->term_current = get_unaligned_le32(data);
		if (prop == MCA_PROP_INPUT_SUSPEND) {
			f->input_suspend = *(u8 *)data;
			if (f->input_suspend)
				f->hiz_assert_writes++;
			else
				f->hiz_clear_writes++;
		}
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT)
			f->icl = get_unaligned_le32(data);
		/* Model a write taking effect before its response is lost. */
		if (prop == f->fail_pmic_prop)
			return -EIO;

		if (prop == MCA_PROP_CHARGING_ENABLED) {
			f->enable_writes++;
			if (!f->enable_stuck)
				f->charging_enabled = *(u8 *)data;
		}
		if (prop == MCA_PROP_VERIFY_PROCESS) {
			f->verify = get_unaligned_le32(data);
			if (f->verify ? f->fail_verify : f->fail_clear)
				return -EIO; /* The write took effect but its ACK was lost. */
		}
		if (prop == MCA_PROP_PD_VERIFIED)
			f->verified = get_unaligned_le32(data);
		if (f->simulate_auth && prop >= MCA_PROP_VDM_BASE &&
		    prop <= MCA_PROP_VDM_REVERSE) {
			u8 bytes[MCA_VDM_LEN];
			int i;

			f->vdm_state = prop - MCA_PROP_VDM_BASE + 1;
			if (f->vdm_count < ARRAY_SIZE(f->vdm_order))
				f->vdm_order[f->vdm_count++] = f->vdm_state;
			if (len == MCA_VDM_LEN)
				for (i = 0; i < ARRAY_SIZE(f->auth_response); i++)
					put_unaligned_be32(get_unaligned_le32(data + 4 * i),
							   bytes + 4 * i);
			if (f->vdm_state == 4) {
				for (i = 0; i < ARRAY_SIZE(piano_mipps_keys); i++)
					if (!memcmp(bytes, piano_mipps_keys[i].seed, sizeof(bytes)))
						break;
				if (i == ARRAY_SIZE(piano_mipps_keys))
					return -EINVAL;
				f->seed_index = i;
			}
			if (f->vdm_state == 5) {
				u8 message[20], digest[SHA256_DIGEST_SIZE];

				memcpy(message, bytes, sizeof(bytes));
				put_unaligned_be32(f->actual_id, message + sizeof(bytes));
				hmac_sha256_usingrawkey(piano_mipps_keys[f->seed_index].key,
						       32, message, sizeof(message), digest);
				if (f->bad_digest)
					digest[0] ^= 1;
				for (i = 0; i < ARRAY_SIZE(f->auth_response); i++)
					f->auth_response[i] =
						cpu_to_le32(get_unaligned_be32(digest + 4 * i));
				if (f->restart_auth)
					mca->service_generation++;
				if (f->detach_auth)
					xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
			}
		}
		f->last_prop = prop;
		memcpy(f->last_data, data, len);
		if (prop == MCA_PROP_INPUT_CURRENT_LIMIT) {
			f->icl = get_unaligned_le32(data);
			if (f->icl && mca->cp_input_suspended)
				f->resumed_before_stop |= f->stage_on;
		}
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
	if (prop == MCA_PROP_CHARGING_ENABLED) {
		*(u8 *)data = f->charging_enabled;
		return 0;
	}
	if (prop == MCA_PROP_INPUT_SUSPEND) {
		*(u8 *)data = 0; /* no stock get_hiz operation */
		return 0;
	}
	if (prop == MCA_PROP_PDOS) {
		memcpy(data, f->pdos, len);
		return 0;
	}
	if (prop == MCA_PROP_VDM_AUTH) {
		memcpy(data, f->auth_response, len);
		return 0;
	}
	switch (prop) {
	case MCA_PROP_USB_ONLINE:
		f->online_reads++;
		if (f->online_error)
			return f->online_error;
		value = f->usb_online;
		break;
	case MCA_PROP_CHGR_STATUS:
		value = f->chgr_status;
		break;
	case MCA_PROP_USB_REAL_TYPE:
		f->type_reads++;
		if (f->type_error)
			return f->type_error;
		value = f->type;
		break;
	case MCA_PROP_USB_VOLTAGE_NOW:
		if (f->vbus_error)
			return f->vbus_error;
		value = f->vbus;
		break;
	case MCA_PROP_INPUT_CURRENT_LIMIT:
		value = f->icl_read_stale ? 100 : f->icl;
		break;
	case MCA_PROP_CHARGE_CURRENT:
		f->fcc_reads++;
		value = f->fcc_read_zero ? 0 : f->fcc;
		break;
	case MCA_PROP_TERM_VOLT:
		value = f->term_readback;
		break;
	case MCA_PROP_TERM_CURRENT:
		value = f->term_current;
		break;
	case MCA_PROP_PACK_VBAT:
		if (f->pack_vbat_error)
			return f->pack_vbat_error;
		value = f->pack_vbat;
		break;
	case MCA_PROP_PACK_TBAT:
		f->pack_reads++;
		if (f->pack_error)
			return f->pack_error;
		value = f->tbat;
		break;
	case MCA_PROP_VERIFY_PROCESS:
		value = f->verify;
		break;
	case MCA_PROP_PD_VERIFIED:
		value = f->verified;
		break;
	case MCA_PROP_ADAPTER_SVID:
		value = f->svid;
		break;
	case MCA_PROP_PD_STATE:
		value = f->pd_state;
		break;
	case MCA_PROP_VDM_VERSION:
		value = f->vdm_version;
		break;
	case MCA_PROP_UVDM_STATE:
		value = f->vdm_state;
		break;
	case MCA_PROP_ADAPTER_ID:
		value = f->adapter_id;
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
	f->mca.hv_retry = jiffies;
	f->mca.dev = kunit_device_register(test, "mca-policy-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->mca.dev);
	mutex_init(&f->mca.lock);
	spin_lock_init(&f->mca.rx_lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_wake_init(&f->mca), 0);
	init_completion(&f->mca.ack);
	INIT_DELAYED_WORK(&f->mca.poll_work, xiaomi_mca_poll);
	kunit_add_action_or_reset(test, mca_policy_cleanup, f);
	dev_set_drvdata(f->mca.dev, &f->mca);
	f->psy.drv_data = &f->mca;
	f->mca.charge_end = 100;
	f->mca.coolest_temp = 25;
	f->charging_enabled = 1;
	f->term_current = 100;
	f->pack_vbat = 4300;
	f->usb_online = 1;
	f->type = MCA_TYPE_PD;
	f->pdos[0] = (struct xiaomi_mca_pdo) { cpu_to_le32(5000), cpu_to_le32(5000), cpu_to_le32(1500) };
	f->vbus = 5000000;
	f->icl = 100;
	f->fcc = 500;
	f->tbat = 25;
	f->mca.real_type = f->type;
	f->mca.vbus_uv = f->vbus;
	f->mca.cp_ma = 6000;
	kunit_activate_static_stub(test, xiaomi_mca_transfer_data, mca_policy_xfer);
	return f;
}

static void xiaomi_mca_test_policy_defaults(struct kunit *test)
{
	__le32 value = cpu_to_le32(9000);

	struct mca_policy_fixture *f = mca_policy_fixture(test);

	mipps_auth = true;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_after_auth(&f->mca));
	f->mca.mipps_attempted = true;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_cp_after_auth(&f->mca));
	f->mca.auth_cleanup = true;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_after_auth(&f->mca));
	mipps_auth = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_C), 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_FLOAT), 1000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_fcc(MCA_TYPE_C, false), 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_icl(MCA_TYPE_UNKNOWN), 0);
	KUNIT_EXPECT_FALSE(test, charge_policy);
	KUNIT_EXPECT_FALSE(test, charge_current);
	KUNIT_EXPECT_FALSE(test, full_charge);
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
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_COOLDOWN);
	f->vbus = mca->vbus_uv = 5000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_COOLDOWN);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 5000);
	mca->hv_retry = jiffies;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->hv, MCA_HV_REQUESTED);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_qc_voltage(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	unsigned int type;

	hv_charge = true;
	mutex_lock(&mca->lock);
	for (type = MCA_TYPE_HVDCP; type <= MCA_TYPE_HVDCP_3P5; type++) {
		f->type = mca->real_type = type;
		mca->hv_retry = jiffies;
		f->vbus = mca->vbus_uv = 5000000;
		mca->hv = MCA_HV_OFF;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
		KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_QC_VOLT);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 9000);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_fcc(type, false), 2000U);
		f->vbus = mca->vbus_uv = 9000000;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), type == MCA_TYPE_HVDCP ? 1200 : 1600);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_policy_fcc(type, true),
				type == MCA_TYPE_HVDCP ? 2600U : 3600U);
		f->vbus = mca->vbus_uv = 10000000;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
		KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_QC_VOLT);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 5000);
	}
	f->type = MCA_TYPE_DCP;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_request_volt(mca, MCA_HV_MV), -EOPNOTSUPP);
	f->type = MCA_TYPE_PD;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_request_volt(mca, MCA_HV_MV), -EOPNOTSUPP);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_aicl(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	static const u32 types[] = { MCA_TYPE_DCP, MCA_TYPE_FLOAT, MCA_TYPE_UNKNOWN };
	unsigned int i;

	charge_current = true;
	guard(mutex)(&f->mca.lock);
	for (i = 0; i < ARRAY_SIZE(types); i++) {
		f->mca.real_type = f->type = types[i];
		f->mca.unknown_polls = MCA_UNKNOWN_POLLS;
		f->aicl_enabled = 0;
		f->mca.fcc_set_ma = f->mca.icl_set_ma = 0;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT,
						      3000), -EAGAIN);
		xiaomi_mca_apply_policy(&f->mca);
		KUNIT_EXPECT_EQ(test, f->aicl_enabled, 1);
		KUNIT_EXPECT_EQ(test, f->aicl_runs, i + 1);
		KUNIT_EXPECT_EQ(test, f->icl, 3000);
		KUNIT_EXPECT_EQ(test, f->fcc, 3000);
		xiaomi_mca_apply_policy(&f->mca);
		KUNIT_EXPECT_EQ(test, f->aicl_runs, i + 1);
	}
	for (i = 0; i < 2; i++) {
		f->mca.icl_set_ma = f->aicl_enabled = 0;
		f->fail_aicl_prop = i ? MCA_PROP_RERUN_AICL : MCA_PROP_AICL_ENABLE;
		xiaomi_mca_apply_policy(&f->mca);
		KUNIT_EXPECT_EQ(test, f->icl, 1000);
		f->fail_aicl_prop = 0;
		xiaomi_mca_apply_policy(&f->mca);
		KUNIT_EXPECT_EQ(test, f->icl, 3000);
	}
	f->vbus = 9000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 3000),
			 -EOPNOTSUPP);
	f->vbus = 5000000;
	f->usb_online = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 3000),
			 -EOPNOTSUPP);
	charge_current = false;
}

static void xiaomi_mca_test_qc_pulses(struct kunit *test)
{
	static const struct { int type, delta, cmd; } cases[] = {
		{ MCA_TYPE_HVDCP_3_B, 200, 0x20 }, { MCA_TYPE_HVDCP_3_B, -200, 0x10 },
		{ MCA_TYPE_HVDCP_3P5, 20, 0x20 }, { MCA_TYPE_HVDCP_3P5, -20, 0x10 },
	};
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	int i;

	cp_charge = true;
	f->mca.cp = MCA_CP_OPENING;
	f->mca.cp_max_mv = 9500;
	f->mca.vbus_uv = 8200000;
	guard(mutex)(&f->mca.lock);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		f->mca.cp_type = f->type = cases[i].type;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_qc_adjust(&f->mca, 8200 + cases[i].delta), 0);
		KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_QC_PULSE);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), cases[i].cmd);
	}
	/* A hold at a non-PPS-aligned ADC measurement sends no QC pulse. */
	f->mca.vbus_uv = 8217000;
	f->writes = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_request(&f->mca, 8217), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->type = MCA_TYPE_DCP;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_qc_adjust(&f->mca, 8400), -EPERM);
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
	mca->hv_pps = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2500), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3000), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3500), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3501), -EINVAL);
	f->type = MCA_TYPE_HVDCP_3;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3600), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3601), -EINVAL);
	f->type = MCA_TYPE_PD;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3500), -EINVAL);
	f->vbus = 5000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_CHARGE_CURRENT, 3000), -EINVAL);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_auth_read_only(struct kunit *test)
{
	__le32 val = cpu_to_le32(1);

	KUNIT_EXPECT_EQ(test, dev_attr_request_vdm_cmd.attr.mode, 0444);
	KUNIT_EXPECT_EQ(test, dev_attr_pd_verifed.attr.mode, 0444);
	KUNIT_EXPECT_EQ(test, dev_attr_verify_process.attr.mode, 0444);
	KUNIT_EXPECT_EQ(test, dev_attr_data_role.attr.mode, 0444);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_DATA_ROLE, &val, 4));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_VDM_REVERSE, &val, 4));
}

static void xiaomi_mca_test_temperature_follow(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca->hv = MCA_HV_ON;
	f->vbus = 9000000;
	mutex_lock(&mca->lock);
	xiaomi_mca_apply_fcc(mca, 25, 1500);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000);
	xiaomi_mca_apply_fcc(mca, 47, 1500);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000);
	xiaomi_mca_apply_fcc(mca, 48, 1500);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000);
	xiaomi_mca_apply_fcc(mca, -1, 1500);
	KUNIT_EXPECT_EQ(test, f->fcc, 880);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_CHARGE_CURRENT);
	mutex_unlock(&mca->lock);
}

static void xiaomi_mca_test_jeita_hysteresis(struct kunit *test)
{
	static const int rows[][2] = {
		{ 7, 0 }, { 8, 0 }, { 10, 8 }, { 13, 8 }, { 15, 13 },
		{ 18, 13 }, { 20, 18 }, { 17, 13 }, { 12, 8 }, { 7, 0 },
	};
	struct xiaomi_mca *mca = &mca_policy_fixture(test)->mca;
	int i;

	for (i = 0; i < ARRAY_SIZE(rows); i++) {
		xiaomi_mca_update_zones(mca, rows[i][0], 25);
		KUNIT_EXPECT_EQ(test, mca->cell_zone, rows[i][1]);
	}
	xiaomi_mca_update_zones(mca, 25, 35);
	xiaomi_mca_update_zones(mca, 25, 34);
	KUNIT_EXPECT_TRUE(test, mca->ffc_warm);
	xiaomi_mca_update_zones(mca, 25, 33);
	KUNIT_EXPECT_FALSE(test, mca->ffc_warm);
	xiaomi_mca_update_zones(mca, 7, 25);
	xiaomi_mca_update_zones(mca, 9, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_fcc_thermal(mca, 3000, 9), 1768);
	xiaomi_mca_update_zones(mca, 10, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_fcc_thermal(mca, 3000, 10), 3000);
}

static void xiaomi_mca_test_profile_switch(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	static const struct { int cold, hot, ma, mv; } rows[] = {
		{ 8, 12, 4320, 4493 }, { 13, 17, 7100, 4493 },
		{ 12, 18, 4320, 4493 }, { 18, 47, 8000, 4473 },
		{ 12, 48, 4320, 4053 }, { 48, 54, 4320, 4053 },
	};
	int n;

	full_charge = true;
	for (n = 0; n < ARRAY_SIZE(rows); n++) {
		mca->zone_valid = false;
		xiaomi_mca_select_profile(mca, rows[n].cold, rows[n].hot);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), rows[n].ma);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_profile_steps[mca->cp_profile][xiaomi_mca_profile_counts[mca->cp_profile] - 1].mv, rows[n].mv);
	}
	full_charge = false;
	/* CN temperature rows and qualified FFC. */
	mca->cp_ibus_ma = 4000;
	mca->cp_ibus_hot = 3000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_die_limit(mca, 1149), 3050);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_die_limit(mca, 1199), 3000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_die_limit(mca, 1200), 2800);
	KUNIT_EXPECT_EQ(test, MCA_PROFILE_COUNT, 5);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 8000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4150, 0), 7230);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4100, 0), 7230);
	mca->ffc_active = true;
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4143, 0), 12000);
	KUNIT_EXPECT_EQ(test, mca->cp_profile, MCA_PROFILE_FFC);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4144, 0), 10650);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4300, 0), 9040);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4475, 0), 7230);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 7230);
	/* Revocation changes the table in the current session, not next attach. */
	mca->ffc_active = false;
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4300, 0), 7230);
	KUNIT_EXPECT_EQ(test, mca->cp_profile, MCA_PROFILE_NORMAL);
	mca->ffc_active = true;
	xiaomi_mca_select_profile(mca, 17, 17);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 7100);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 12000);
	mca->ffc_active = false;
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 8000);
}

static void xiaomi_mca_test_source_limits(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	static const struct { int ibat, ibus, fcc; } rows[] = {
		{ 12000, 5201, 4000 }, { 10000, 5201, 3000 }, { 9040, 4201, 3000 },
		{ 8000, 4201, 2000 }, { 7230, 2701, 2000 }, { 5000, 2701, 1000 },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(rows); i++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_parallel_fcc(rows[i].ibat, rows[i].ibus),
				rows[i].fcc);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_capability_ma(6700, true), 6000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_capability_ma(6700, false), 5000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_capability_ma(3000, true), 3000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_div2_limit(0), 0U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_div2_limit(5000), 10000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_div2_limit(6000), 12000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_div2_limit(U32_MAX), 12400U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_power_limit(10000, 11000), 6090U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_power_limit(6000, 8000), 6000U);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_power_limit(6000, 0), 0U);
	mca->ffc_active = true;
	mca->cp_ma = 5000;
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 10000);
	mca->cp_ibus_cap = 4000;
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 8000);
}

static void xiaomi_mca_test_recover_learned_limit(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca->cp = MCA_CP_ON;
	mca->cp_ma = 3000;
	mca->cp_ibus_last = 2500;
	xiaomi_mca_cp_learn_drop(mca);
	KUNIT_ASSERT_EQ(test, mca->cp_ibus_cap, 2000);
	xiaomi_mca_cp_recover_current(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_ibus_cap, 2000);
	mca->cp_cap_retry = jiffies;
	mca->cp_ibus_last = 1000;
	xiaomi_mca_cp_recover_current(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_ibus_cap, 2000);
	mca->cp_ibus_last = 1900;
	xiaomi_mca_cp_recover_current(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_ibus_cap, 2500);
	mca->cp_cap_retry = jiffies;
	mca->cp_ibus_last = 2400;
	xiaomi_mca_cp_recover_current(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_ibus_cap, 0);
}

static void xiaomi_mca_test_bus_margin(struct kunit *test)
{
	struct xiaomi_mca *mca = &mca_policy_fixture(test)->mca;

	KUNIT_EXPECT_EQ(test, MCA_CP_STAGE_IBUS_MA, 3800);
	mca->cp_ibus_ma = MCA_CP_STAGE_IBUS_MA;

	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_bus_trip(mca, 4080));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_cp_bus_trip(mca, 4501));
	KUNIT_EXPECT_FALSE(test, mca->cp_fault);
	mca->cp_type = MCA_TYPE_HVDCP_3_B;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_band(mca->cp_type), 1400);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_bus_trip(mca, 5100));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_cp_bus_trip(mca, 7000));
	KUNIT_EXPECT_TRUE(test, mca->cp_fault);
}

static void xiaomi_mca_test_cp_limits(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 8000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4150, 0), 7230);
	xiaomi_mca_select_profile(mca, 25, 25);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4140, 0), 7230);
	xiaomi_mca_select_profile(mca, 17, 17);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4000, 0), 7100);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(8000, 4000)));
	cp_charge = true;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(8000, 4000)));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(11000, 4000)));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pps_word_ok(xiaomi_mca_pps_word(11020, 4000)));
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
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 500), 0);
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

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE: val->intval = f->stage_on; break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: val->intval = f->stage_mv * 1000; break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: val->intval = f->stage_ma * 1000; break;
	case POWER_SUPPLY_PROP_TEMP: val->intval = f->stage_temp; break;
	default: return -EINVAL;
	}
	return 0;
}

static int mca_test_stage_set(struct power_supply *psy, enum power_supply_property psp,
			      const union power_supply_propval *val)
{
	struct mca_policy_fixture *f = power_supply_get_drvdata(psy);

	if (f->trace_count < ARRAY_SIZE(f->trace)) {
		f->trace[f->trace_count].prop = 0xff0001;
		f->trace[f->trace_count++].value = val->intval;
	}
	f->stage_stops++;
	if (f->stage_error)
		return f->stage_error;
	if (psp == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT)
		return 0;
	f->stage_on = val->intval;
	return 0;
}

static int mca_test_cp_finish(struct xiaomi_mca *mca, enum sc8541_otg_action action)
{
	struct mca_policy_fixture *f = container_of(mca, struct mca_policy_fixture, mca);

	if (f->trace_count < ARRAY_SIZE(f->trace)) {
		f->trace[f->trace_count].prop = 0xff0002;
		f->trace[f->trace_count++].value = action;
	}
	if (mca->cp_psy->removing)
		return -ENODEV;
	if (f->stage_on)
		return -EBUSY;
	return 0;
}

static void mca_test_named_stage(struct kunit *test, struct mca_policy_fixture *f, const char *name)
{
	static const enum power_supply_property props[] = {
		POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	};
	static const struct power_supply_desc template = {
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = props,
		.num_properties = ARRAY_SIZE(props),
		.get_property = mca_test_stage_get,
		.set_property = mca_test_stage_set,
	};
	struct power_supply_config cfg = { .drv_data = f };
	struct power_supply *psy;
	struct power_supply_desc *desc;

	desc = devm_kmemdup(f->mca.dev, &template, sizeof(template), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, desc);
	desc->name = name;
	psy = devm_power_supply_register(f->mca.dev, desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, psy);
	f->mca.cp_psy = power_supply_get_by_name(desc->name);
	KUNIT_ASSERT_PTR_EQ(test, f->mca.cp_psy, psy);
	f->mca.cp = MCA_CP_ON;
	f->stage_on = true;
	kunit_activate_static_stub(test, xiaomi_mca_cp_finish, mca_test_cp_finish);
}

static void mca_test_stage(struct kunit *test, struct mca_policy_fixture *f)
{
	f->mca.service_up = true;
	mca_test_named_stage(test, f, "mca-pm-test-cp");
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
	f->type = MCA_TYPE_PD;
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
	KUNIT_EXPECT_EQ(test, f->icl, 1500U);
	KUNIT_EXPECT_EQ(test, f->fcc, 1500U);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_FALSE(test, f->voltage_before_stop);
	KUNIT_EXPECT_EQ(test, f->vbus, 5000000);
	KUNIT_EXPECT_TRUE(test, f->mca.sleeping);
}

static void mca_test_idle_poll(struct work_struct *work);

static void xiaomi_mca_test_pm_source_currents(struct kunit *test)
{
	static const struct {
		u32 type, icl, fcc;
	} cases[] = {
		{ MCA_TYPE_SDP, 500, 500 },
		{ MCA_TYPE_CDP, 900, 900 },
		{ MCA_TYPE_DCP, 3000, 3000 },
		{ MCA_TYPE_C, 500, 500 },
		{ MCA_TYPE_PD, 900, 900 },
		{ MCA_TYPE_PD_VERIFY, 900, 900 },
		{ MCA_TYPE_PD_PPS, 900, 900 },
	};
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	int i;

	INIT_DELAYED_WORK(&f->mca.poll_work, mca_test_idle_poll);

	f->pdos[0].min_mv = cpu_to_le32(5000);
	f->pdos[0].max_mv = cpu_to_le32(5000);
	f->pdos[0].max_ma = cpu_to_le32(900);
	/* A programmable 5-11 V/0.5 A offer cannot lower the fixed 5 V PDO. */
	f->pdos[1].min_mv = cpu_to_le32(5000);
	f->pdos[1].max_mv = cpu_to_le32(11000);
	f->pdos[1].max_ma = cpu_to_le32(500);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		f->type = cases[i].type;
		f->mca.icl_written = true;
		f->mca.fcc_written = true;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
		KUNIT_EXPECT_EQ(test, f->icl, cases[i].icl);
		KUNIT_EXPECT_EQ(test, f->fcc, cases[i].fcc);
		KUNIT_EXPECT_TRUE(test, f->mca.icl_written);
		KUNIT_EXPECT_TRUE(test, f->mca.fcc_written);
		xiaomi_mca_complete(f->mca.dev);
		cancel_delayed_work_sync(&f->mca.poll_work);
		KUNIT_EXPECT_EQ(test, f->mca.fcc_set_ma, 0);
		KUNIT_EXPECT_EQ(test, f->mca.icl_set_ma, 0);
		KUNIT_EXPECT_TRUE(test, f->mca.icl_written);
	}

	/* The 5 V handoff retains the existing warm-battery derating. */
	f->tbat = 48;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_EQ(test, f->fcc, xiaomi_mca_fcc_thermal(&f->mca, 900, 48));
}

static void xiaomi_mca_test_sleep_fixed_pd(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	hv_charge = true;
	mca->hv = MCA_HV_ON;
	mca->hv_pps = true;
	mca->voltage_written = true;
	mca->icl_written = true;
	mca->fcc_written = true;
	f->vbus = 9000000;
	f->pdos[0] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(1200) };
	mutex_lock(&mca->lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(mca), 0);
	KUNIT_EXPECT_EQ(test, f->icl, 1200U);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000U);
	KUNIT_EXPECT_TRUE(test, mca->voltage_written);
	KUNIT_EXPECT_FALSE(test, mca->hv_pps);
	/* Only a programmable offer covers 9 V now. Fall back to fixed 5 V. */
	f->pdos[0].min_mv = cpu_to_le32(5000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(mca), 0);
	KUNIT_EXPECT_EQ(test, f->vbus, 5000000);
	KUNIT_EXPECT_EQ(test, f->icl, 500U);
	KUNIT_EXPECT_FALSE(test, mca->voltage_written);
	mutex_unlock(&mca->lock);
	hv_charge = false;
}

static void xiaomi_mca_test_retry_schedule(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	static const char * const reasons[] = { "taper complete", "stage temperature", "read failed" };
	int n;

	for (n = 0; n < ARRAY_SIZE(reasons); n++) {
		xiaomi_mca_cp_session_done(mca, reasons[n]);
		KUNIT_EXPECT_EQ(test, mca->cp, n ? MCA_CP_OFF : MCA_CP_DONE);
		KUNIT_EXPECT_STREQ(test, mca->cp_last_exit, reasons[n]);
		xiaomi_mca_cp_try_start(mca);
		KUNIT_EXPECT_EQ(test, mca->cp, n ? MCA_CP_OFF : MCA_CP_DONE);
	}
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
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->stage_stops, 2);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->stage_error = 0;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, NULL);
}

static void xiaomi_mca_test_stage_removed(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct power_supply *psy;

	mca_test_stage(test, f);
	psy = f->mca.cp_psy;
	f->stage_error = -ENODEV;
	guard(mutex)(&f->mca.lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(&f->mca, "stage read failed"), -ENODEV);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	scoped_guard(rwsem_write, &psy->epc_sem)
		psy->removing = true;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, NULL);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF);
	KUNIT_EXPECT_STREQ(test, f->mca.cp_last_exit, "stage read failed");
	scoped_guard(rwsem_write, &psy->epc_sem)
		psy->removing = false;
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
		xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
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
	KUNIT_EXPECT_TRUE(test, f->mca.fcc_written);
	KUNIT_EXPECT_EQ(test, f->fcc, 1500U);
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
	u32 props[32];
	u8 data[32][16];
	size_t lengths[32];
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
	u32 pen_vbus;
	int pen_restart_write;
	unsigned int delays[16], delay_count;
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
		if (prop == MCA_PROP_USB_VOLTAGE_NOW && len == sizeof(u32)) {
			put_unaligned_le32(f->pen_vbus, data);
			return 0;
		}
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
	if (f->pen_restart_write == f->writes)
		mca->service_generation++;
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
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_write_allowed(0x200f, expected, 4));
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
		KUNIT_EXPECT_EQ(test, f->writes, fail + 3);
		KUNIT_EXPECT_EQ(test, f->mca.config_result, 0);
		xiaomi_mca_sync_config(&f->mca);
		KUNIT_EXPECT_EQ(test, f->writes, fail + 3);
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
		KUNIT_EXPECT_EQ(test, f->read_prop, 0);
		KUNIT_EXPECT_EQ(test, f->read_len, 0);
		KUNIT_EXPECT_EQ(test, f->reads, 0);
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
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), 1);
	f->cc_toggle = 1;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "0", 1), 1);
	f->read_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), 1);
	f->fail_write = 4;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), -EIO);
	KUNIT_EXPECT_EQ(test, f->reads, 0); /* Failed writes have no readback. */
	f->read_error = 0;
	f->restart_on_read = true;
	KUNIT_EXPECT_EQ(test, cc_toggle_store(&psy.dev, NULL, "1", 1), 1);
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

static int mca_test_cell_get(struct power_supply *psy, enum power_supply_property psp,
			     union power_supply_propval *val)
{
	struct mca_policy_fixture *f = power_supply_get_drvdata(psy);
	int n = !strcmp(psy->desc->name, xiaomi_mca_supplied_to[1]);

	if (f->cells[n].error)
		return f->cells[n].error;
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = f->cells[n].charge_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = f->cells[n].charge_full;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = f->cells[n].capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = f->cells[n].temp;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = f->cells[n].voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = f->cells[n].current_ua;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void mca_full_charge_cleanup(void *unused)
{
	full_charge = false;
	charge_policy = false;
	mipps_auth = false;
}

static int mca_test_gauge_command(struct xiaomi_mca *mca, unsigned int n, u16 cmd)
{
	struct mca_policy_fixture *f = container_of(mca, struct mca_policy_fixture, mca);

	f->fg_commands[n][cmd == 0x31 ? 2 : cmd == 0x3e]++;

	return f->fg_command_error[n];
}

static bool mca_test_gauges_full(struct xiaomi_mca *mca)
{
	struct mca_policy_fixture *f = container_of(mca, struct mca_policy_fixture, mca);

	return f->fg_full_ready;
}

static struct mca_policy_fixture *mca_full_charge_fixture(struct kunit *test)
{
	static const enum power_supply_property props[] = {
		POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_CAPACITY,
		POWER_SUPPLY_PROP_CHARGE_NOW, POWER_SUPPLY_PROP_CHARGE_FULL,
	};
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct power_supply_config cfg = { .drv_data = f };
	struct power_supply_desc *desc;
	struct power_supply *psy;
	int n;

	for (n = 0; n < ARRAY_SIZE(f->cells); n++) {
		desc = devm_kzalloc(f->mca.dev, sizeof(*desc), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, desc);
		desc->name = xiaomi_mca_supplied_to[n];
		desc->type = POWER_SUPPLY_TYPE_BATTERY;
		desc->properties = props;
		desc->num_properties = ARRAY_SIZE(props);
		desc->get_property = mca_test_cell_get;
		f->cells[n].current_ua = 2000000;
		f->cells[n].temp = 250;
		f->cells[n].voltage = 4000000;
		f->cells[n].charge_now = 4000000;
		f->cells[n].charge_full = 4600000;
		psy = devm_power_supply_register(f->mca.dev, desc, &cfg);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, psy);
	}
	f->mca.service_up = true;
	f->pdos[0].max_ma = cpu_to_le32(3000);
	f->term_readback = 5200;
	kunit_add_action_or_reset(test, mca_full_charge_cleanup, NULL);
	f->fg_full_ready = true;
	kunit_activate_static_stub(test, xiaomi_mca_gauge_command, mca_test_gauge_command);
	kunit_activate_static_stub(test, xiaomi_mca_gauges_full, mca_test_gauges_full);
	return f;
}

static void xiaomi_mca_test_full_charge_temperature(struct kunit *test)
{
	static const struct {
		int pack, cell0, cell1, pack_error, cell0_error, cell1_error;
		u32 readback;
	} cases[] = {
		{ 25, 250, 250, 0, 0, 0, 5580 },
		{ 47, 479, 250, 0, 0, 0, 5590 },
		{ 48, 250, 250, 0, 0, 0, 5580 },
		{ 47, 250, 250, 0, 0, 0, 5580 },
		{ 46, 250, 250, 0, 0, 0, 5580 },
		{ 45, 459, 250, 0, 0, 0, 5590 },
		{ 25, 480, 250, 0, 0, 0, 4580 },
		{ 25, 460, 250, 0, 0, 0, 4580 },
		{ 25, 459, 250, 0, 0, 0, 5590 },
		{ 25, 250, 480, 0, 0, 0, 4580 },
		{ 25, 250, 460, 0, 0, 0, 4580 },
		{ 25, 250, 459, 0, 0, 0, 5590 },
		{ 25, 250, 250, -ETIMEDOUT, 0, 0, 5580 },
		{ 47, 250, 250, 0, 0, 0, 5580 },
		{ 25, 250, 250, 0, 0, 0, 5580 },
		{ 25, 250, 250, 0, -ENODATA, 0, 5580 },
		{ 25, 250, 250, 0, 0, 0, 5580 },
		{ 25, 250, 250, 0, 0, -EIO, 5580 },
		{ 55, 550, 550, 0, 0, 0, 4580 },
		{ 60, 600, 600, 0, 0, 0, 4580 },
		{ 0, 0, 0, 0, 0, 0, 5400 },
		{ -5, -50, -50, 0, 0, 0, 5400 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int n;

	full_charge = true;
	charge_policy = true;
	guard(mutex)(&f->mca.lock);
	for (n = 0; n < ARRAY_SIZE(cases); n++) {
		f->tbat = cases[n].pack;
		f->cells[0].temp = cases[n].cell0;
		f->cells[1].temp = cases[n].cell1;
		f->pack_error = cases[n].pack_error;
		f->cells[0].error = cases[n].cell0_error;
		f->cells[1].error = cases[n].cell1_error;
		KUNIT_EXPECT_EQ_MSG(test, xiaomi_mca_full_charge_poll(&f->mca), 0,
				    "case %d", n);
		KUNIT_EXPECT_EQ_MSG(test, f->term_readback, cases[n].readback,
				    "case %d", n);
	}
}

static void xiaomi_mca_test_cold_float_hysteresis(struct kunit *test)
{
	static const int samples[][3] = {
		{ 16, 16, 5400 }, { 17, 17, 5400 }, { 18, 18, 5360 },
		{ 17, 17, 5360 }, { 18, 18, 5360 }, { 16, 18, 5360 },
		{ 16, 16, 5400 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int i;

	f->type = MCA_TYPE_SDP;
	full_charge = charge_policy = true;
	guard(mutex)(&f->mca.lock);
	for (i = 0; i < ARRAY_SIZE(samples); i++) {
		f->tbat = samples[i][0];
		f->cells[0].temp = samples[i][0] * 10;
		f->cells[1].temp = samples[i][1] * 10;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, f->term_readback, samples[i][2]);
	}
	KUNIT_EXPECT_EQ(test, f->term_writes, 3);
}

static void xiaomi_mca_test_full_charge_readback(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = true;
	mca_test_stage(test, f);
	guard(mutex)(&mca->lock);
	/* cycle3: ACK 4690, diagnostic 5580. No fake error or pump stop. */
	f->term_stuck = true;
	f->term_readback = 5580;
	f->cells[0].voltage = f->cells[1].voltage = 3844000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1500000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->pmic_fv_acked, 4690);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_writes, 1);
	xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_DOWN);
	xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_UP);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_writes, 2);
	f->term_restart = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_u32(mca, MCA_PROP_TERM_VOLT, 4480), -ECONNRESET);
}

static void xiaomi_mca_test_full_charge_restore(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct auxiliary_device adev = {};

	full_charge = true;
	scoped_guard(mutex, &f->mca.lock) {
		/* The feature requires charge_policy; untouched sessions stay read-only. */
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, f->term_writes, 0);
		charge_policy = true;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		full_charge = false;
		f->write_error = -ETIMEDOUT;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), -ETIMEDOUT);
		KUNIT_EXPECT_TRUE(test, f->mca.term_written);
		f->write_error = 0;
		f->type = MCA_TYPE_UNKNOWN;
		f->vbus = 0;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, f->term_readback, 5200);
		KUNIT_EXPECT_FALSE(test, f->mca.term_written);
		full_charge = true;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		charge_policy = false;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, f->term_readback, 5200);
		charge_policy = true;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	}
	xiaomi_mca_unbind_restore(&f->mca);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5200);
	KUNIT_EXPECT_FALSE(test, f->mca.term_written);
	scoped_guard(mutex, &f->mca.lock) {
		/* A lost iterm ACK still needs the default pair at shutdown. */
		f->write_error = -ETIMEDOUT;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), -ETIMEDOUT);
		KUNIT_EXPECT_TRUE(test, f->mca.iterm_written);
		f->write_error = 0;
	}
	auxiliary_set_drvdata(&adev, &f->mca);
	xiaomi_mca_shutdown(&adev);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5200);
	KUNIT_EXPECT_FALSE(test, f->mca.term_written);
}

static void mca_test_resume_work(struct work_struct *work)
{
	struct xiaomi_mca *mca = container_of(to_delayed_work(work), struct xiaomi_mca,
					      poll_work);

	complete(&mca->ack);
}

static void xiaomi_mca_test_full_charge_sleep(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	unsigned int writes;

	full_charge = true;
	charge_policy = true;
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	writes = f->term_writes;
	INIT_DELAYED_WORK(&f->mca.poll_work, mca_test_resume_work);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5360);
	KUNIT_EXPECT_EQ(test, f->term_writes, writes + 1);
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), -EHOSTDOWN);
	KUNIT_EXPECT_EQ(test, f->term_writes, writes + 1);
	/* The next awake evaluation must use fresh temperatures. */
	f->cells[1].temp = 480;
	xiaomi_mca_complete(f->mca.dev);
	flush_delayed_work(&f->mca.poll_work);
	KUNIT_EXPECT_TRUE(test, completion_done(&f->mca.ack));
	KUNIT_EXPECT_FALSE(test, f->mca.sleeping);
	scoped_guard(mutex, &f->mca.lock)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 4580);
}

static void xiaomi_mca_test_full_charge_allowlist(struct kunit *test)
{
	static const u32 allowed[] = { 4060, 4400, 4480, 4500 };
	static const u32 rejected[] = { 0, 4059, 4061, 4350, 4399, 4401, 4479, 4481,
					4550, 5200, 5360, U32_MAX };
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	__le32 value = cpu_to_le32(4480);
	int n;

	for (n = 0; n < ARRAY_SIZE(allowed); n++) {
		value = cpu_to_le32(allowed[n]);
		KUNIT_EXPECT_TRUE(test, xiaomi_mca_write_allowed(MCA_PROP_TERM_VOLT,
							       &value, sizeof(value)));
	}
	for (n = 0; n < ARRAY_SIZE(rejected); n++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write_u32(&f->mca, MCA_PROP_TERM_VOLT,
							 rejected[n]), -EPERM);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_blob(&f->mca, MCA_PROP_TERM_VOLT,
						  &value, 1), -EPERM);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
}

static void xiaomi_mca_test_auth_stop_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct power_supply *psy;

	mca_test_stage(test, f);
	psy = f->mca.cp_psy;
	f->verify = 1;
	f->stage_error = -EIO;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_cp_step(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, psy);
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_EQ(test, f->stage_stops, 2);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->stage_error = 0;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_PTR_EQ(test, f->mca.cp_psy, NULL);
	KUNIT_EXPECT_GT(test, f->writes, 0);
}

static void xiaomi_mca_test_vbus_error_not_detach(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);

	mca_test_stage(test, f);
	f->mca.online = true;
	f->mca.cp_off_at = 123;
	f->mca.cp_on_at = 456;
	f->mca.cp_ibus_cap = 3000;
	f->mca.cp_ibus_last = 2000;
	f->mca.hv = MCA_HV_ON;
	f->vbus_error = -ETIMEDOUT;
	f->stage_error = -EIO;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_TRUE(test, f->mca.online);
	KUNIT_EXPECT_EQ(test, f->mca.real_type, MCA_TYPE_PD);
	KUNIT_EXPECT_EQ(test, f->mca.cp_off_at, 123);
	KUNIT_EXPECT_EQ(test, f->mca.cp_on_at, 456);
	KUNIT_EXPECT_EQ(test, f->mca.cp_ibus_cap, 3000);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->stage_error = 0;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_TRUE(test, f->mca.online);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1); /* stage is already off */
	/* A successful read of 0 V must still run the real detach path. */
	f->vbus_error = 0;
	f->vbus = 0;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->mca.online);
	KUNIT_EXPECT_EQ(test, f->mca.real_type, MCA_TYPE_UNKNOWN);
	KUNIT_EXPECT_NE(test, f->mca.cp_off_at, 123);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF); /* no physical UCSI detach */
}

static void xiaomi_mca_test_threshold_interface(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	union power_supply_propval val;

	KUNIT_ASSERT_EQ(test, xiaomi_mca_battery_ext_register(&f->mca), 0);
	val.intval = 75;
	KUNIT_ASSERT_EQ(test, power_supply_set_property(f->mca.batteries[0],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &val), 0);
	val.intval = 80;
	KUNIT_ASSERT_EQ(test, power_supply_set_property(f->mca.batteries[1],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &val), 0);
	KUNIT_ASSERT_EQ(test, power_supply_get_property(f->mca.batteries[0],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &val), 0);
	KUNIT_EXPECT_EQ(test, val.intval, 80);
	KUNIT_ASSERT_EQ(test, power_supply_get_property(f->mca.batteries[1],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &val), 0);
	KUNIT_EXPECT_EQ(test, val.intval, 75);
	val.intval = 80;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[0],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &val), -EINVAL);
	val.intval = -1;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[0],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &val), -EINVAL);
	val.intval = 75;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[1],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &val), -EINVAL);
	val.intval = 101;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[1],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &val), -EINVAL);
	val.intval = 0;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[0],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD, &val), 0);
	val.intval = 100;
	KUNIT_EXPECT_EQ(test, power_supply_set_property(f->mca.batteries[1],
		POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD, &val), 0);
}

static void xiaomi_mca_test_threshold_hysteresis(struct kunit *test)
{
	static const struct { int soc0, soc1; bool enabled; } cases[] = {
		{ 74, 79, true }, { 79, 80, false }, { 79, 79, false },
		{ 75, 76, false }, { 75, 75, true }, { 79, 79, true },
		{ 81, 75, false },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int n;

	f->mca.charge_start = 75;
	f->mca.charge_end = 80;
	full_charge = charge_policy = true;
	guard(mutex)(&f->mca.lock);
	for (n = 0; n < ARRAY_SIZE(cases); n++) {
		f->cells[0].capacity = cases[n].soc0;
		f->cells[1].capacity = cases[n].soc1;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ_MSG(test, f->charging_enabled, cases[n].enabled, "case %d", n);
		KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
	}
	f->mca.charge_end = 100;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_FALSE(test, f->mca.charge_disabled);
}

static void xiaomi_mca_test_gauge_fallback(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int n;

	full_charge = charge_policy = true;
	mca_test_stage(test, f);
	f->mca.fcc_set_ma = f->fcc = 3500;
	guard(mutex)(&f->mca.lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	f->cells[0].error = -ENODEV;
	for (n = 0; n < 4; n++) {
		if (n == 2)
			f->icl = 2500;
		if (n == 3)
			f->type = MCA_TYPE_UNKNOWN;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODEV);
		KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
		KUNIT_EXPECT_EQ(test, f->stage_on, n < 2);
		KUNIT_EXPECT_EQ(test, f->fcc, n >= 2 ? 500 : 3500);
		KUNIT_EXPECT_EQ(test, f->icl, n == 3 ? 1000 : n >= 2 ? 500 : 100);
		KUNIT_EXPECT_EQ(test, f->term_readback, n >= 2 ? 5200 : 5780);
		KUNIT_EXPECT_TRUE(test, f->mca.charge_blocked);
	}
	/* Temperature remains independently observable during the outage. */
	f->cells[1].temp = 480;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODEV);
	KUNIT_EXPECT_EQ(test, f->term_readback, 4580);
	f->cells[1].temp = 450;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODEV);
	KUNIT_EXPECT_EQ(test, f->term_readback, 4580);
	f->cells[0].error = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_FALSE(test, f->mca.charge_blocked);
}

static void xiaomi_mca_test_threshold_thermal_release(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);

	f->mca.charge_start = 75;
	f->mca.charge_end = 80;
	f->cells[0].capacity = 80;
	full_charge = charge_policy = true;
	guard(mutex)(&f->mca.lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	f->mca.charge_end = 100;
	f->cells[1].temp = 550;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 4580);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[1].temp = 530;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[1].temp = 520;
	f->cells[0].error = -EIO;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -EIO);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[0].error = 0;
	f->cells[0].temp = -100;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[0].temp = -80;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[0].temp = -70;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	/* Releasing protection at 52 C must not raise JEITA's float voltage. */
	KUNIT_EXPECT_EQ(test, f->term_readback, 4580);
}

static void xiaomi_mca_test_threshold_stop_failure_sleep(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);

	mca_test_stage(test, f);
	f->mca.charge_start = 75;
	f->mca.charge_end = 80;
	f->cells[0].capacity = 80;
	f->mca.voltage_written = true;
	f->stage_error = -EIO;
	guard(mutex)(&f->mca.lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -EIO);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 0);
	f->stage_error = 0;
	f->vbus_error = -ETIMEDOUT;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1); /* off, source 5 V before ICL */
	f->vbus_error = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_FALSE(test, f->voltage_before_stop);
	/* Firmware reset discarding the disable is corrected on the next poll. */
	f->charging_enabled = 1;
	f->enable_stuck = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_TRUE(test, f->mca.charge_disabled);
	f->enable_stuck = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	f->cells[0].capacity = 70;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	/* Handover itself never disables buck charging below the threshold. */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	/* Invalid SOC cannot resume charging. */
	f->cells[1].capacity = -1;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODATA);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODATA);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(&f->mca), -ENODATA);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	/* Unplug after HV charging must still allow protected battery sleep. */
	f->vbus = 0;
	f->type = MCA_TYPE_UNKNOWN;
	f->mca.voltage_written = true;
	f->mca.icl_written = true;
	f->mca.fcc_written = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 1);
	KUNIT_EXPECT_FALSE(test, f->mca.voltage_written);
}

static void xiaomi_mca_test_connector_stop(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	guard(mutex)(&mca->lock);
	mca_test_stage(test, f);
	mca->piano_otg = true;
	mca->connector.tripped = true;
	mca->otg_requested = true;
	mca->voltage_written = false;
	f->vbus = 5000000;
	xiaomi_mca_connector_stop(mca);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_TRUE(test, mca->charge_disabled);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_PEN_BOOST_ENABLE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_enable(mca, true), -EBUSY);
	/* A pending high-voltage request must be cancelled even at 5 V. */
	mca->otg_requested = false;
	mca->voltage_written = true;
	xiaomi_mca_connector_stop(mca);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_VDM_VSAFE0V);
	KUNIT_EXPECT_FALSE(test, mca->voltage_written);
	KUNIT_EXPECT_EQ(test, f->voltage_requests, 2U);
	KUNIT_EXPECT_FALSE(test, f->voltage_before_stop);
	mca->connector.tripped = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
}

static void xiaomi_mca_test_connector(struct kunit *test)
{
	static const struct {
		int prev[2], temp[2];
		bool active, wired, otg, trip;
		int board;
	} cases[] = {
		{ { 30000, 30000 }, { 65000, 30000 }, false, true, false, true },
		{ { 30000, 60000 }, { 30000, 60000 }, false, false, true, true },
		{ { 59999, 30000 }, { 59999, 30000 }, false, true, false, false },
		{ { 31000, 29000 }, { 35000, 30000 }, false, true, false, true },
		{ { 31001, 31000 }, { 35000, 35000 }, false, true, false, false },
		{ { 42000, 30000 }, { 42000, 36000 }, false, true, false, false },
		{ { 31000, 31000 }, { 34999, 32000 }, false, true, false, false },
		{ { 30000, 30000 }, { 70000, 30000 }, false, false, false, false },
		{ { 60000, 59000 }, { 60000, 59000 }, false, true, false, false, 51000 },
		{ { 65000, 59000 }, { 65000, 59000 }, false, true, false, true, 51000 },
		{ { 60000, 59000 }, { 60000, 59000 }, false, true, false, true, 50000 },
		{ { 55000, 54500 }, { 55000, 54500 }, true, true, false, true },
		{ { 55000, 54500 }, { 54999, 54500 }, true, true, false, false },
		{ { 55000, 54500 }, { INT_MIN, 54000 }, true, false, false, true },
		{ { 50000, INT_MIN }, { INT_MIN, 65000 }, false, true, false, true },
		{ { INT_MIN, 30000 }, { 45000, 31000 }, false, true, false, false },
		{ { 48000, 44000 }, { 53000, 44000 }, true, false, true, true },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		struct mca_connector c = { .sampled = ms_to_ktime(1000),
					   .tripped = cases[i].active };

		memcpy(c.temp, cases[i].prev, sizeof(c.temp));
		c.valid[0] = c.temp[0] != INT_MIN;
		c.valid[1] = c.temp[1] != INT_MIN;
		KUNIT_EXPECT_EQ_MSG(test, xiaomi_mca_connector_trip(&c, cases[i].temp,
				    ms_to_ktime(2000), cases[i].wired || cases[i].otg, cases[i].board),
				    cases[i].trip, "case %d", i);
	}
}

static void xiaomi_mca_test_wake_policy(struct kunit *test)
{
	/* USB_REAL_TYPE names in Piano firmware order, including its unnamed
	 * values 13 and 16 and wireless BPP/EPP/HPP. Unknown IDs are wired.
	 */
	static const bool charger[] = {
		true, false, false, true, true, true, true, true, true, true,
		true, true, true, true, true, true, true, false, false, false,
	};
	int type, online, otg;

	BUILD_BUG_ON(ARRAY_SIZE(charger) != MCA_TYPE_MAX);
	for (type = 0; type <= ARRAY_SIZE(charger); type++)
		for (online = 0; online <= 1; online++)
			for (otg = 0; otg <= 1; otg++)
				KUNIT_EXPECT_EQ_MSG(test,
					xiaomi_mca_should_stay_awake(online, otg, type),
					online && !otg && (type == MCA_TYPE_MAX || charger[type]),
					"type %d online %d otg %d", type, online, otg);
}

static void mca_test_idle_poll(struct work_struct *work)
{
	/* PM queue enable/disable is real; GLINK stubs are local to this test. */
}

static void xiaomi_mca_test_ffc_conditions(struct kunit *test)
{
	static const int socs[] = { 20, 84, 94, 95, 99, 100 };
	static const int temps[] = { 10, 15, 25, 40, 50 };
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int type, soc, t;

	charge_policy = full_charge = true;
	guard(mutex)(&f->mca.lock);
	for (type = MCA_TYPE_SDP; type <= MCA_TYPE_C; type++)
		for (soc = 0; soc < ARRAY_SIZE(socs); soc++)
			for (t = 0; t < ARRAY_SIZE(temps); t++) {
				bool want = type != MCA_TYPE_SDP && type != MCA_TYPE_CDP &&
					socs[soc] < 95 && temps[t] >= 18 && temps[t] < 48;

				f->mca.ffc_active = f->mca.ffc_done = false;
				f->mca.zone_valid = f->mca.term_hot = false;
				f->cells[0].capacity = f->cells[1].capacity = socs[soc];
				f->cells[0].temp = f->cells[1].temp = temps[t] * 10;
				f->tbat = temps[t];
				f->type = type;
				KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
				KUNIT_EXPECT_EQ_MSG(test, f->mca.ffc_active, want,
					"type=%d soc=%d temp=%d", type, socs[soc], temps[t]);
			}
	f->type = MCA_TYPE_PD_PPS;
	f->cells[0].capacity = f->cells[1].capacity = 94;
	f->cells[0].temp = f->cells[1].temp = 250;
	f->tbat = 25;
	f->mca.zone_valid = f->mca.term_hot = false;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_ASSERT_TRUE(test, f->mca.ffc_active);
	f->cells[0].capacity = f->cells[1].capacity = 100;
	f->cells[0].current_ua = f->cells[1].current_ua = 500000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_TRUE(test, f->mca.ffc_active); /* session remains latched */
	f->mca.attach_generation++;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
	KUNIT_EXPECT_FALSE(test, f->mca.ffc_active); /* no new FFC at 100% */
}

static void xiaomi_mca_test_cp_cell_guards(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int vbat, ibat, tcool, tbat, vmin, imax;

	full_charge = true;
	f->cells[0].voltage = 3449000;
	f->cells[1].voltage = 4400000;
	f->cells[0].current_ua = 7000000;
	f->cells[1].current_ua = -1000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_gauges(&f->mca, &vbat, &ibat, &tcool, &tbat,
						&vmin, &imax, NULL), 0);
	KUNIT_EXPECT_EQ(test, vbat, 4400);
	KUNIT_EXPECT_EQ(test, vmin, 3449);
	KUNIT_EXPECT_EQ(test, ibat, 6000);
	KUNIT_EXPECT_EQ(test, imax, 7000);
	f->mca.cp_ma = 3000;
	f->cells[0].current_ua = 0;
	mca_test_stage(test, f);
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_cp_step(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_FALSE(test, f->mca.cp_fault);
	xiaomi_mca_cp_try_start(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF);
	xiaomi_mca_cp_try_start(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF);
	f->cells[0].voltage = 4400000;
	f->cells[0].current_ua = 7000000;
	f->mca.cp_psy = power_supply_get_by_name("mca-pm-test-cp");
	f->mca.cp = MCA_CP_ON;
	f->stage_on = true;
	kunit_activate_static_stub(test, xiaomi_mca_cp_finish, mca_test_cp_finish);
	xiaomi_mca_cp_step(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.cp, MCA_CP_OFF);
	KUNIT_EXPECT_FALSE(test, f->mca.cp_fault);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	f->cells[0].current_ua = 9001000;
	f->mca.cp_psy = power_supply_get_by_name("mca-pm-test-cp");
	f->mca.cp = MCA_CP_ON;
	f->stage_on = true;
	kunit_activate_static_stub(test, xiaomi_mca_cp_finish, mca_test_cp_finish);
	xiaomi_mca_cp_step(&f->mca);
	KUNIT_EXPECT_TRUE(test, f->mca.cp_fault);
}

static void xiaomi_mca_test_auth_loss_latch(struct kunit *test)
{
	static const struct { bool enabled, valid; u32 firmware; bool latch; } cases[] = {
		{ false, true, 1, false }, { true, false, 1, false },
		{ true, true, 2, false }, { true, true, 0, true },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int i;

	mca_test_stage(test, f);
	guard(mutex)(&mca->lock);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		if (i)
			mca->cp_psy = power_supply_get_by_name("mca-pm-test-cp");
		mca->cp = MCA_CP_ON;
		f->stage_on = true;
		mca->cp_ma = 6000;
		mipps_auth = cases[i].enabled;
		mca->auth_ok = cases[i].valid;
		f->verified = cases[i].firmware;
		xiaomi_mca_cp_step(mca);
		KUNIT_EXPECT_EQ(test, mca->cp, cases[i].latch ? MCA_CP_DONE : MCA_CP_OFF);
		KUNIT_EXPECT_FALSE(test, f->stage_on);
		KUNIT_EXPECT_EQ(test, mca->auth_lost, cases[i].latch);
	}
}

static void xiaomi_mca_test_gauge_ocp_identity(struct kunit *test)
{
	static const struct software_node se5 = { .name = "mca-test-se5" };
	static const struct software_node se11 = { .name = "mca-test-se11" };
	static const struct software_node pump = { .name = "cp", .parent = &se5 };
	static const struct software_node master = { .name = "fg", .parent = &se5 };
	static const struct software_node slave = { .name = "fg", .parent = &se11 };
	const struct property_entry props[] = {
		PROPERTY_ENTRY_REF("xiaomi,charge-pump", &pump), {}
	};
	const struct software_node charger = { .name = "mca-test", .properties = props };
	const struct software_node *group[] = { &se5, &se11, &pump, &master, &slave,
						       &charger, NULL };
	static const struct { int ua; bool master, slave; } cases[] = {
		{ 7000000, false, false }, { 7000001, false, true },
		{ 9000000, false, true }, { 9000001, true, true },
	};
	struct fwnode_handle *node;
	int i;

	KUNIT_ASSERT_EQ(test, software_node_register_node_group(group), 0);
	node = software_node_fwnode(&charger);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cell_ocp(node, software_node_fwnode(&master),
							 cases[i].ua), cases[i].master);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cell_ocp(node, software_node_fwnode(&slave),
							 cases[i].ua), cases[i].slave);
	}
	software_node_unregister_node_group(group);
}

static void xiaomi_mca_test_pack_limit_recovers(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca_test_named_stage(test, f, "sc8541");
	cp_charge = charge_current = hv_charge = charge_policy = full_charge = true;
	f->pdos[1] = (struct xiaomi_mca_pdo) { cpu_to_le32(5000), cpu_to_le32(11000),
					      cpu_to_le32(5000) };
	mca->cp_ma = 5000;
	mca->cp_max_mv = 11000;
	mca->cp_ibat_trip = 12000;
	mca->cp_ibus_ma = 3800;
	f->stage_mv = 8400;
	f->stage_ma = 1500;
	f->cells[0].current_ua = f->cells[1].current_ua = 6300000;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OFF);
	KUNIT_EXPECT_STREQ(test, mca->cp_last_exit, "battery current too high");
	KUNIT_EXPECT_FALSE(test, mca->cp_fault);
	f->cells[0].current_ua = f->cells[1].current_ua = 2000000;
	mca->vbus_uv = f->vbus = 9000000;
	mca->hv = MCA_HV_ON;
	xiaomi_mca_cp_try_start(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OPENING);
	xiaomi_mca_cp_try_start(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OPENING);
	KUNIT_EXPECT_EQ(test, mca->attach_generation, 0);
	xiaomi_mca_cp_stop(mca, "test complete");
}

static void xiaomi_mca_test_bus_voltage_fault(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n;

	mca_test_stage(test, f);
	mca->cp_max_mv = 11000;
	mca->cp_ma = 3000;
	guard(mutex)(&mca->lock);
	for (n = 0; n < 2; n++) {
		if (n) {
			mca->cp_psy = power_supply_get_by_name("mca-pm-test-cp");
			mca->cp = MCA_CP_ON;
			f->stage_on = true;
		}
		f->stage_mv = n ? 11000 : 9300;
		xiaomi_mca_cp_step(mca);
		KUNIT_EXPECT_EQ(test, mca->cp, n ? MCA_CP_DONE : MCA_CP_OFF);
		KUNIT_EXPECT_EQ(test, mca->cp_fault, !!n);
		KUNIT_EXPECT_STREQ(test, mca->cp_last_exit, "bus voltage too high");
	}
}

static void xiaomi_mca_test_qc_regulation(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	cp_charge = hv_charge = full_charge = true;
	mca_test_stage(test, f);
	f->mca.cp_type = f->type = MCA_TYPE_HVDCP_3_B;
	f->mca.real_type = f->type;
	mca->vbus_uv = f->vbus = 8400000;
	mca->cp_mv = f->stage_mv = 8400;
	mca->cp_max_mv = 9500;
	mca->cp_request_ma = mca->cp_ma = 2500;
	mca->cp_ibus_hot = mca->cp_ibus_ma = 1800;
	mca->icl_set_ma = 500;
	mca->fcc_set_ma = 1000;
	f->cells[0].voltage = f->cells[1].voltage = 4000000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1000000;
	guard(mutex)(&mca->lock);
	f->stage_ma = 401;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_NE(test, f->last_prop, MCA_PROP_QC_PULSE);
	f->stage_ma = 400;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, f->last_prop, MCA_PROP_QC_PULSE);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->last_data), MCA_QC_UP);
	mca->vbus_uv = f->vbus = 8600000;
	f->stage_mv = 8600;
	f->stage_ma = 1800;
	f->writes = 0;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	KUNIT_EXPECT_FALSE(test, mca->cp_fault);
	xiaomi_mca_cp_stop(mca, "test complete");
}

static void xiaomi_mca_test_measured_buck_share(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = hv_charge = cp_charge = mipps_auth = true;
	mca_test_stage(test, f);
	mca->auth_ok = true;
	f->verified = 1;
	mca->ffc_active = true;
	mca->cp_ma = mca->cp_request_ma = 6000;
	mca->cp_mv = f->stage_mv = 8400;
	mca->cp_max_mv = 11000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	mca->icl_set_ma = 2500;
	mca->fcc_set_ma = 4000;
	mca->ibus_ua = 2000000;
	f->stage_ma = 3600;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, mca->cp_mv, 8420);
	mca->ibus_ua = 2500000;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_LT(test, mca->cp_mv, 8420);
	xiaomi_mca_cp_stop(mca, "test complete");
}

/* Recorded transition: both cells near 4.47 V; the buck share fell at
 * 4480 mV and the previous policy increased the PPS request to 9800 mV.
 */
static void xiaomi_mca_test_tail_current_transfer(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int previous;

	full_charge = charge_policy = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->ffc_active = true;
	mca->cp_ma = 5000;
	mca->cp_mv = 9660;
	mca->cp_max_mv = 10000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	mca->icl_set_ma = 1500;
	mca->fcc_set_ma = 2000;
	f->cells[0].current_ua = 2012000;
	f->cells[1].current_ua = 2005000;
	f->cells[0].voltage = 4504000;
	f->cells[1].voltage = 4503000;
	f->stage_ma = 2740;
	f->stage_mv = 9412;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_TRUE(test, mca->cp_parallel_done);
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 2000);
	/* Removing the buck share must not impose a new pump current cap.
	 * Voltage headroom still permits a step at 4504 mV.
	 */
	previous = mca->cp_mv;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_GT(test, mca->cp_mv, previous);
	/* The parallel decision stays made when the gauge relaxes. */
	f->cells[0].voltage = f->cells[1].voltage = 4470000;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 2000);
	/* Near CV, the same current deficit must not push the voltage up. */
	f->cells[0].voltage = f->cells[1].voltage = 4529000;
	previous = mca->cp_mv;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_LE(test, mca->cp_mv, previous);
	/* One 20 mV PPS step would consume the remaining cell headroom. */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_cv_request(MCA_TYPE_PD_PPS, 9660, 9680,
						       4529, 4540), 9660);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_cv_request(MCA_TYPE_HVDCP_3_B, 9200, 9220,
						       4410, 4473), 9200);
	xiaomi_mca_cp_stop(mca, "test complete");
}

static void xiaomi_mca_test_buck_enable_readback(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	charge_policy = true;
	guard(mutex)(&mca->lock);
	f->charging_enabled = 0;
	KUNIT_ASSERT_FALSE(test, mca->charge_disabled);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, f->enable_writes, 1);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->enable_writes, 2);
	f->charging_enabled = 0;
	f->enable_stuck = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
}

static void xiaomi_mca_test_ffc_cutoff_in_pump(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = true;
	mca_test_stage(test, f);
	mca->ffc_active = true;
	f->type = mca->real_type = MCA_TYPE_PD_PPS;
	f->chgr_status = 5;
	f->cells[0].voltage = f->cells[1].voltage = 4539000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1175000;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	KUNIT_EXPECT_FALSE(test, mca->charging_done);
	KUNIT_EXPECT_FALSE(test, mca->ffc_done);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5780);
	KUNIT_EXPECT_EQ(test, f->enable_writes, 1); /* repeat enable, ignore FW termination in CP */
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
}

static void xiaomi_mca_test_taper_handoff(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = hv_charge = cp_charge = true;
	mca_test_stage(test, f);
	mca->ffc_active = true;
	mca->cp_max_mv = 11000;
	mca->cp_mv = 9500;
	mca->cp_ma = 5000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 4000;
	mca->icl_set_ma = mca->fcc_set_ma = 500;
	f->stage_mv = 9400;
	f->stage_ma = 2000;
	f->cells[0].voltage = f->cells[1].voltage = 4540000;
	f->cells[0].current_ua = f->cells[1].current_ua = 2000000;
	guard(mutex)(&mca->lock);
	for (int n = 0; n < 4 && mca->cp == MCA_CP_ON; n++)
		xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_LT(test, mca->cp_mv, 9500);
	f->cells[0].current_ua = f->cells[1].current_ua = 1499000;
	for (int n = 0; n < 4 && mca->cp == MCA_CP_ON; n++)
		xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	KUNIT_EXPECT_FALSE(test, mca->cp_fault);
}

static void xiaomi_mca_test_identity_edges(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);

	mipps_auth = true;
	f->mca.charge_blocked = true;
	guard(mutex)(&f->mca.lock);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_identity_changed(&f->mca));
	f->svid = 0x2717;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_identity_changed(&f->mca));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_identity_changed(&f->mca));
	f->error = -EIO;
	f->svid = 0;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_identity_changed(&f->mca));
	KUNIT_EXPECT_EQ(test, f->mca.adapter_svid, 0x2717);
	f->error = 0;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_identity_changed(&f->mca));
}

static int mca_auth_supply_get(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	val->intval = 1;
	return 0;
}

static void mca_auth_cleanup(void *unused)
{
	mipps_auth = false;
}

static struct mca_policy_fixture *mca_auth_fixture(struct kunit *test)
{
	static const enum power_supply_property props[] = { POWER_SUPPLY_PROP_ONLINE };
	static const struct power_supply_desc desc = {
		.name = "mca-auth-test", .type = POWER_SUPPLY_TYPE_USB,
		.properties = props, .num_properties = ARRAY_SIZE(props),
		.get_property = mca_auth_supply_get,
	};
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct power_supply_config cfg = { .drv_data = &f->mca };

	f->mca.psy = devm_power_supply_register(f->mca.dev, &desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->mca.psy);
	f->mca.online = true;
	f->mca.service_up = true;
	f->simulate_auth = true;
	f->actual_id = 0x1234;
	f->vdm_version = 1;
	f->pdos[0] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(5000), cpu_to_le32(5000), cpu_to_le32(3000) };
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(3000) };
	mipps_auth = true;
	kunit_add_action_or_reset(test, mca_auth_cleanup, NULL);
	return f;
}

struct mca_wake_events {
	struct notifier_block nb;
	struct power_supply *usb;
	unsigned int count;
};

static int mca_test_wake_event(struct notifier_block *nb, unsigned long event, void *psy)
{
	struct mca_wake_events *events = container_of(nb, struct mca_wake_events, nb);

	if (event == PSY_EVENT_PROP_CHANGED && psy == events->usb)
		events->count++;
	return NOTIFY_OK;
}

/* Run the production worker synchronously; its timer uses an inert work item. */
static void mca_test_wired_tick(struct mca_policy_fixture *f)
{
	xiaomi_mca_poll(&f->mca.poll_work.work);
	cancel_delayed_work_sync(&f->mca.poll_work);
	flush_work(&f->mca.psy->changed_work);
}

static void xiaomi_mca_test_wired_wake_transitions(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);
	struct mca_wake_events events = {
		.nb.notifier_call = mca_test_wake_event, .usb = f->mca.psy,
	};
	struct xiaomi_mca_notification msg = { .id = cpu_to_le32(2) };
	unsigned int count, transfers;

	INIT_DELAYED_WORK(&f->mca.poll_work, mca_test_idle_poll);
	flush_delayed_work(&f->mca.psy->deferred_register_work);
	flush_work(&f->mca.psy->changed_work);
	KUNIT_ASSERT_EQ(test, power_supply_reg_notifier(&events.nb), 0);
	f->mca.poll_ready = true;
	f->mca.piano_otg = true;
	f->mca.charge_end = 100;
	f->type = MCA_TYPE_FLOAT;
	f->chgr_status = 5;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, f->mca.charge_awake);
	KUNIT_EXPECT_EQ(test, f->online_reads, 1);
	KUNIT_EXPECT_EQ(test, f->type_reads, 1);
	count = events.count;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_EQ(test, events.count, count);
	transfers = f->transfers;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), -EBUSY);
	KUNIT_EXPECT_EQ(test, f->transfers, transfers);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_usb_notification(&msg, sizeof(msg), &f->mca));

	f->mca.charge_end = 80;
	f->type = MCA_TYPE_SDP;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_FALSE(test, f->mca.charge_awake);
	KUNIT_EXPECT_EQ(test, events.count, count + 1);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(f->mca.dev), 0);
	xiaomi_mca_complete(f->mca.dev);
	f->type_error = -EIO; /* Online with an unknown type holds the lock. */
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, f->mca.charge_awake);
	f->type_error = 0;
	f->usb_online = 0;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_FALSE(test, f->mca.charge_awake);
	f->usb_online = 1;
	f->type = MCA_TYPE_DCP;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, f->mca.charge_awake);
	xiaomi_mca_stop_poll(&f->mca);
	KUNIT_EXPECT_FALSE(test, f->mca.charge_awake);
	power_supply_unreg_notifier(&events.nb);
}

static void xiaomi_mca_test_wired_wake_reconnect(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	u32 old;

	INIT_DELAYED_WORK(&mca->poll_work, mca_test_idle_poll);
	mca->poll_ready = true;
	f->type = MCA_TYPE_DCP;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, mca->charge_awake);
	old = mca->input_generation;
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
	KUNIT_EXPECT_FALSE(test, mca->charge_awake);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_apply_wired_sample(mca, old, 1, MCA_TYPE_DCP));
	KUNIT_EXPECT_FALSE(test, mca->charge_awake);
	f->usb_online = 0;
	mca_test_wired_tick(f);
	/* A complete replug after prepare must be caught before platform sleep. */
	KUNIT_ASSERT_EQ(test, xiaomi_mca_prepare(mca->dev), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pm_ops.suspend_noirq(mca->dev), 0);
	old = mca->input_generation;
	f->usb_online = 1;
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_CONNECTED);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_apply_wired_sample(mca, old, 0, MCA_TYPE_CDP));
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pm_ops.suspend_noirq(mca->dev), -EBUSY);
	xiaomi_mca_complete(mca->dev);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(mca->dev), -EBUSY);
	f->type = MCA_TYPE_UNKNOWN;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, mca->charge_awake);

	old = mca->input_generation;
	xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_DOWN);
	KUNIT_EXPECT_FALSE(test, mca->charge_awake);
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
	KUNIT_EXPECT_FALSE(test, mca->input_pending);
	xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_UP);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_apply_wired_sample(mca, old, 0, MCA_TYPE_SDP));
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(mca->dev), -EBUSY);
	f->online_error = -EIO;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_prepare(mca->dev), -EBUSY);
	f->online_error = 0;
	f->type = MCA_TYPE_PD;
	mca_test_wired_tick(f);
	KUNIT_EXPECT_TRUE(test, mca->charge_awake);
	mca->piano_otg = true;
	mca_otg_send(mca, MCA_NOTIFY_OTG_CP_CONFIG, 1);
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_apply_wired_sample(mca, mca->input_generation, 1, f->type));
	KUNIT_EXPECT_FALSE(test, mca->charge_awake);
	mca->otg_active = true; /* Gate-off error must not hide a subsequent sink. */
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_CONNECTED);
	KUNIT_EXPECT_FALSE(test, mca->otg_requested);
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_apply_wired_sample(mca, mca->input_generation, 1, f->type));
	KUNIT_EXPECT_TRUE(test, mca->charge_awake);
	xiaomi_mca_stop_poll(mca);
}

static void xiaomi_mca_test_mipps_exchange(struct kunit *test)
{
	static const u32 order[] = { 1, 2, 3, 4, 5, 7, 6 };
	struct mca_policy_fixture *f = mca_auth_fixture(test);

	guard(mutex)(&f->mca.lock);
	/* A partner/5 V event must not consume the attachment's attempt. */
	f->pdos[1].max_mv = cpu_to_le32(5000);
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_FALSE(test, f->mca.mipps_attempted);
	KUNIT_EXPECT_EQ(test, f->vdm_count, 0);
	f->pdos[1].max_mv = cpu_to_le32(9000);
	/* Zero SVID and missing ID: only the fresh challenge establishes identity. */
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_authenticated(&f->mca));
	KUNIT_EXPECT_EQ(test, f->verified, 1);
	KUNIT_EXPECT_EQ(test, f->verify, 0);
	KUNIT_EXPECT_EQ(test, f->vdm_count, ARRAY_SIZE(order));
	KUNIT_EXPECT_MEMEQ(test, f->vdm_order, order, sizeof(order));
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_EQ(test, f->vdm_count, ARRAY_SIZE(order));
	/* A fast physical replug invalidates proof even if VBUS never sampled low. */
	xiaomi_mca_ucsi_update(&f->mca, MCA_UCSI_OFFLINE);
	xiaomi_mca_ucsi_update(&f->mca, MCA_UCSI_PD_SINK);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_authenticated(&f->mca));
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->mca.mipps_attempted);
	KUNIT_EXPECT_EQ(test, f->mca.policy_attach_generation, f->mca.attach_generation);
}

static void xiaomi_mca_test_mipps_bad_digest(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);

	f->bad_digest = true;
	f->verified = 1; /* Stale firmware authentication is not trusted. */
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_authenticated(&f->mca));
	KUNIT_EXPECT_EQ(test, f->verified, 0);
	KUNIT_EXPECT_EQ(test, f->verify, 0);
	KUNIT_EXPECT_EQ(test, f->vdm_order[f->vdm_count - 1], 7);
}

static void xiaomi_mca_test_mipps_lost_ack(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);

	f->fail_verify = f->fail_clear = true;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_EQ(test, f->vdm_count, 0);
	KUNIT_EXPECT_TRUE(test, f->mca.auth_cleanup);
	KUNIT_EXPECT_FALSE(test, f->mca.auth_ok);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_quiesce(&f->mca), -EIO);
	f->fail_clear = false;
	mipps_auth = false; /* Cleanup also works after disabling the feature. */
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->mca.auth_cleanup);
	KUNIT_EXPECT_EQ(test, f->verify, 0);
}

static void xiaomi_mca_test_auth_retry_contract(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);
	int n;

	charge_policy = hv_charge = true;
	f->mca.mipps_attempted = true;
	f->mca.auth_retry = jiffies - 300 * HZ;
	f->mca.adapter_svid = 0; /* identity discovery can be late */
	f->mca.hv = MCA_HV_ON;
	f->vbus = 9000000;
	guard(mutex)(&f->mca.lock);
	for (n = 0; n < 3; n++) {
		xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
		KUNIT_EXPECT_EQ(test, f->mca.hv, MCA_HV_ON);
		KUNIT_EXPECT_TRUE(test, f->mca.mipps_attempted);
		KUNIT_EXPECT_EQ(test, f->voltage_requests, 0);
	}
	f->vbus = 5000000;
	f->mca.hv = MCA_HV_OFF;
	xiaomi_mca_policy_poll(&f->mca, &(u32){ 0 });
	KUNIT_EXPECT_FALSE(test, f->mca.mipps_attempted);
	f->fail_verify = true;
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.auth_retry_ms, 60000);
	f->mca.mipps_attempted = false;
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_EQ(test, f->mca.auth_retry_ms, 120000);
}

static void xiaomi_mca_test_mipps_restart(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);

	f->restart_auth = true;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_authenticated(&f->mca));
	KUNIT_EXPECT_EQ(test, f->verified, 0);
	KUNIT_EXPECT_EQ(test, f->vdm_order[f->vdm_count - 1], 5);
}

static void xiaomi_mca_test_mipps_detach(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);

	f->detach_auth = true;
	guard(mutex)(&f->mca.lock);
	xiaomi_mca_mipps_authenticate(&f->mca);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_authenticated(&f->mca));
	KUNIT_EXPECT_EQ(test, f->verified, 0);
	KUNIT_EXPECT_EQ(test, f->verify, 0);
	KUNIT_EXPECT_EQ(test, f->vdm_order[f->vdm_count - 1], 5);
}

static int mca_ucsi_supply_get(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct mca_policy_fixture *f = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = f->ucsi_online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = f->ucsi_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 1000 * f->ucsi_mv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = f->ucsi_rp_ua;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = f->ucsi_charge_type;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = f->ucsi_status;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void mca_register_ucsi_peer(struct kunit *test, struct mca_policy_fixture *f)
{
	static const enum power_supply_property props[] = {
		POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_USB_TYPE,
		POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_CHARGE_TYPE,
	};
	static const struct power_supply_desc desc = {
		.name = "mca-test-ucsi", .type = POWER_SUPPLY_TYPE_USB,
		.properties = props, .num_properties = ARRAY_SIZE(props),
		.usb_types = BIT(POWER_SUPPLY_USB_TYPE_PD),
		.get_property = mca_ucsi_supply_get,
	};
	struct power_supply_config cfg = { .drv_data = f };
	struct device *peer_dev = kunit_device_register(test, "mca-test-ucsi-parent");
	struct power_supply *peer;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, peer_dev);
	peer = devm_power_supply_register(peer_dev, &desc, &cfg);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, peer);
}

static void xiaomi_mca_test_aicl_collapse(struct kunit *test)
{
	static const struct {
		u32 type, vbus, floor;
		bool online;
	} cases[] = {
		{ MCA_TYPE_FLOAT, 3999000, 1000, true },
		{ MCA_TYPE_DCP, 5000000, 1500, false },
		{ MCA_TYPE_UNKNOWN, 0, 1000, false },
	};
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	ktime_t deadline;
	int i, n;

	mca_register_ucsi_peer(test, f);
	f->ucsi_type = POWER_SUPPLY_USB_TYPE_C;
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	charge_policy = charge_current = true;
	guard(mutex)(&mca->lock);
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		f->type = cases[i].type;
		f->ucsi_online = f->usb_online = 1;
		xiaomi_mca_ucsi_update(mca, MCA_UCSI_CONNECTED);
		f->vbus = 5000000;
		for (n = 1; n <= MCA_UNKNOWN_POLLS; n++) {
			xiaomi_mca_policy_poll(mca, &(u32){ 0 });
			if (f->type == MCA_TYPE_UNKNOWN && n < MCA_UNKNOWN_POLLS)
				KUNIT_EXPECT_EQ(test, f->icl, 1000);
		}
		KUNIT_ASSERT_EQ(test, f->icl, 3000);
		KUNIT_ASSERT_EQ(test, mca->aicl_probe_floor_ma, cases[i].floor);
		deadline = mca->aicl_deadline;
		f->vbus = 4000000;
		xiaomi_mca_observe_aicl(mca);
		xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0);
		KUNIT_EXPECT_EQ(test, mca->aicl_deadline, deadline);
		f->vbus = cases[i].vbus;
		f->usb_online = cases[i].online;
		if (cases[i].vbus < MCA_VBUS_ONLINE_UV)
			xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		else
			xiaomi_mca_observe_aicl(mca);
		KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, cases[i].floor);
		xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		f->vbus = 5000000;
		f->usb_online = 1;
		xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		KUNIT_EXPECT_EQ(test, f->icl, cases[i].floor);
		KUNIT_EXPECT_EQ(test, f->fcc, cases[i].floor);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_write(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 3000),
				-EINVAL);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_sleep_currents(mca, 0), 0);
		KUNIT_EXPECT_EQ(test, f->icl, cases[i].floor);
		f->type = MCA_TYPE_DCP;
		xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		KUNIT_EXPECT_LE(test, f->icl, cases[i].floor);
		f->ucsi_online = f->usb_online = 0;
		f->vbus = 0;
		xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
		xiaomi_mca_policy_poll(mca, &(u32){ 0 });
		KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0);
	}
}

static void xiaomi_mca_test_aicl_evidence(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	u32 generation;

	charge_policy = true;
	f->type = MCA_TYPE_FLOAT;
	guard(mutex)(&mca->lock);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	f->vbus = 3500000;
	xiaomi_mca_observe_aicl(mca);
	KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0); /* No UCSI provider. */
	mca_register_ucsi_peer(test, f);
	f->vbus = 5000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_restore_icl(mca), 0);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	f->vbus = 3500000;
	xiaomi_mca_observe_aicl(mca);
	KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0); /* Explicit UCSI offline. */
	KUNIT_EXPECT_EQ(test, mca->aicl_probe_floor_ma, 0);
	f->vbus = 5000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_restore_icl(mca), 0);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_ASSERT_NE(test, mca->aicl_probe_floor_ma, 0);
	f->ucsi_online = 1;
	f->vbus_error = f->online_error = -EIO;
	xiaomi_mca_observe_aicl(mca);
	KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0);
	f->vbus_error = f->online_error = 0;
	mca->aicl_deadline = ktime_sub_ms(ktime_get_boottime(), 1);
	xiaomi_mca_observe_aicl(mca);
	KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0);
	KUNIT_EXPECT_EQ(test, mca->aicl_probe_floor_ma, 0);

	f->vbus = 5000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_restore_icl(mca), 0);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_ASSERT_EQ(test, mca->aicl_probe_floor_ma, 1000);
	generation = mca->attach_generation;
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
	xiaomi_mca_ucsi_update(mca, MCA_UCSI_CONNECTED);
	f->vbus = 0;
	xiaomi_mca_observe_aicl(mca);
	KUNIT_EXPECT_NE(test, generation, mca->attach_generation);
	KUNIT_EXPECT_EQ(test, mca->aicl_cap_ma, 0);
}

static void xiaomi_mca_test_unknown_debounce(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_policy_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	f->type = MCA_TYPE_UNKNOWN;
	charge_policy = true;
	guard(mutex)(&mca->lock);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->icl, 1000);
	KUNIT_EXPECT_EQ(test, mca->unknown_polls, 1);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_sleep_currents(mca, 0), 0);
	KUNIT_EXPECT_EQ(test, f->icl, 1000);
	KUNIT_EXPECT_EQ(test, mca->unknown_polls, 1);
	f->type = MCA_TYPE_SDP;
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, mca->unknown_polls, 0);
	KUNIT_EXPECT_EQ(test, f->icl, 500);
	f->type = MCA_TYPE_UNKNOWN;
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->icl, 1000);
	f->online_error = -EIO;
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, mca->unknown_polls, 0);
	f->online_error = 0;
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->icl, 1000);
	xiaomi_mca_policy_poll(mca, &(u32){ 0 });
	KUNIT_EXPECT_EQ(test, f->icl, 3000);
}

/* Five-second observations from the first MiPPS session, 84% through the
 * last good sample before autonomous pump shutdown. The physical trace
 * ends at 87%; the explicitly marked tail below tests software handover.
 */
static void xiaomi_mca_test_pmic_compensation(struct kunit *test)
{
	static const struct { bool ffc, warm, comp_warm; u32 fv, iterm; } cases[] = {
		{ false, false, false, 4480, 246 }, { false, true, true, 4480, 246 },
		{ true, false, false, 4590, 1250 }, { true, true, true, 4595, 1650 },
		{ true, true, false, 4590, 1657 }, /* 35 -> 34 C: row hysteresis only */
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int i;

	full_charge = charge_policy = true;
	mca->cell_zone = 18;
	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		mca->ffc_active = cases[i].ffc;
		mca->pmic_iterm_warm = cases[i].warm;
		mca->pmic_comp_warm = cases[i].comp_warm;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_fv(mca), cases[i].fv);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_iterm(mca), cases[i].iterm);
	}
	full_charge = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_iterm(mca), 100);
}

static void xiaomi_mca_test_pmic_temperature_votes(struct kunit *test)
{
	static const struct { int master, slave, fv, iterm; } rows[] = {
		{ 350, 340, 4595, 1243 }, /* compensation uses FG hot; iterm uses slave */
		{ 350, 350, 4595, 1650 },
		{ 340, 330, 4590, 1657 }, /* slave's original 2 C hysteresis */
		{ 329, 329, 4590, 1250 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	int i;

	full_charge = charge_policy = true;
	f->type = MCA_TYPE_PD_PPS;
	guard(mutex)(&f->mca.lock);
	for (i = 0; i < ARRAY_SIZE(rows); i++) {
		f->cells[0].temp = rows[i].master;
		f->cells[1].temp = rows[i].slave;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(&f->mca), 0);
		KUNIT_EXPECT_EQ(test, f->term_readback, 2 * rows[i].fv - 3600);
		KUNIT_EXPECT_EQ(test, f->term_current, rows[i].iterm);
	}
}

static void xiaomi_mca_test_pmic_headroom(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);
	static const int packs[] = { 4300, 4487, 4550, 4566, -1 };
	int i;

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mca->cell_zone = 18;
	mca->ffc_active = true;
	for (i = 0; i < ARRAY_SIZE(packs); i++) {
		mca->cp = MCA_CP_ON;
		mca->pmic_pack_mv = packs[i];
		xiaomi_mca_adjust_pmic_fv(mca);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_fv(mca), 4690);
	}
	mca->cp = MCA_CP_STOPPING;
	xiaomi_mca_adjust_pmic_fv(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_fv_base, 4650); /* withdrawn LAST by stop */
	mca->cp_fv_base = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_fv(mca), 4590);
	mca->ffc_active = false;
	mca->cp = MCA_CP_ON;
	xiaomi_mca_adjust_pmic_fv(mca);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_fv(mca), 4480);
}

static void xiaomi_mca_test_pmic_handover(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->cp_ma = 5000;
	mca->cp_mv = 9700;
	mca->cp_max_mv = 10000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	f->vbus = mca->vbus_uv = 9500000;
	f->stage_mv = 9500;
	f->stage_ma = 2700;
	f->pack_vbat = 4568;
	f->cells[0].voltage = f->cells[1].voltage = 4490000;
	f->cells[0].current_ua = f->cells[1].current_ua = 2000000;
	guard(mutex)(&mca->lock);
	for (n = 0; n < 5; n++)
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5780); /* 4560 + 40 */
	KUNIT_EXPECT_EQ(test, f->term_current, 1250);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_TRUE(test, mca->cp_input_suspended);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	f->pack_vbat_error = -EIO;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5780);
	KUNIT_EXPECT_EQ(test, mca->pmic_pack_mv, -1);
	f->pack_vbat_error = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "taper complete"), 0);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	KUNIT_EXPECT_EQ(test, f->input_suspend, 0);
	KUNIT_EXPECT_FALSE(test, f->resumed_before_stop);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580); /* override withdrawn */
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 0);     /* next PDO recomputes FCC */
	KUNIT_EXPECT_EQ(test, f->fcc, 500);
}

static void xiaomi_mca_test_pmic_suspend_threshold(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->cp_ma = 5000;
	mca->cp_mv = 9700;
	mca->cp_max_mv = 10000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	f->vbus = mca->vbus_uv = 9500000;
	f->stage_mv = 9500;
	f->stage_ma = 2700;
	f->cells[0].current_ua = f->cells[1].current_ua = 2000000;
	f->cells[0].voltage = f->cells[1].voltage = 4470000;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, mca->fcc_set_ma, 0);
	mca->ffc_active = true;
	f->cells[0].voltage = f->cells[1].voltage = 4480000;
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	f->cells[0].voltage = f->cells[1].voltage = 4470000;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "test complete"), 0);
	KUNIT_EXPECT_EQ(test, f->input_suspend, 0);
}

static void xiaomi_mca_test_pmic_suspend_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = true;
	mca_test_stage(test, f);
	guard(mutex)(&mca->lock);
	f->fail_pmic_prop = MCA_PROP_INPUT_CURRENT_LIMIT;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), -EIO);
	KUNIT_EXPECT_TRUE(test, mca->cp_input_suspended);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "test"), -EIO);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_STOPPING);
	KUNIT_EXPECT_NOT_NULL(test, mca->cp_psy);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_FALSE(test, f->resumed_before_stop);
	f->fail_pmic_prop = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "pending stop"), 0);
	KUNIT_EXPECT_FALSE(test, mca->cp_input_suspended);
	KUNIT_EXPECT_PTR_EQ(test, mca->cp_psy, NULL);
}

static void xiaomi_mca_test_pmic_cell_ceiling(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = true;
	f->type = MCA_TYPE_PD_PPS;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_TRUE(test, mca->ffc_active);
	f->cells[0].voltage = 4551000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->voltage_hold);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
}

static void xiaomi_mca_test_normal_pump_termination_replay(struct kunit *test)
{
	/* 23:16:55..23:17:15: observed old-policy frames, not a battery model. */
	static const struct { int v0, v1, i0, i1, pack, bus, ibus; } rows[] = {
		{ 4316, 4333, -489, -405, 4381, 8775, 1365 },
		{ 4438, 4435, -279,  921, 4427, 9000,  752 },
		{ 4446, 4445,  883,  909, 4487, 9123, 1350 },
		{ 4376, 4375,  720,  759, 4372, 9288,    5 },
		{ 4374, 4373,  503,  530, 4371, 9288,    5 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	mca->cp = MCA_CP_OPENING;
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->cp_ma = 5000;
	mca->cp_mv = 9060;
	mca->cp_max_mv = 10000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), 0);
	for (n = 0; n < ARRAY_SIZE(rows); n++) {
		f->cells[0].voltage = rows[n].v0 * 1000;
		f->cells[1].voltage = rows[n].v1 * 1000;
		f->cells[0].current_ua = rows[n].i0 * 1000;
		f->cells[1].current_ua = rows[n].i1 * 1000;
		f->pack_vbat = rows[n].pack;
		f->stage_mv = rows[n].bus;
		f->stage_ma = rows[n].ibus;
		f->vbus = mca->vbus_uv = rows[n].bus * 1000;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		xiaomi_mca_cp_step(mca);
		KUNIT_EXPECT_EQ_MSG(test, mca->cp, MCA_CP_ON, "row %d", n);
		KUNIT_EXPECT_EQ(test, f->icl, 0);
		KUNIT_EXPECT_EQ(test, mca->cp_exits, 0);
	}
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "test complete"), 0);
	KUNIT_EXPECT_EQ(test, f->input_suspend, 0);
}

static void xiaomi_mca_test_taper_direction(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);
	int n;

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mca->cp = MCA_CP_OPENING;
	mca->cp_profile = MCA_PROFILE_FFC;
	mca->cp_stage = 3;
	for (n = 0; n < 5; n++)
		KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_taper_complete(mca, true, 2000));
	mca->cp = MCA_CP_ON;
	for (n = 1; n <= 4; n++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_taper_complete(mca, true, 2999), n == 4);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_taper_complete(mca, false, 2000));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_taper_complete(mca, true, 3000));
	mca->ffc_warm = true;
	for (n = 1; n <= 4; n++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_taper_complete(mca, true, 3499), n == 4);
	mca->cp_stage = 2;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_taper_complete(mca, true, 1000));
}

static void xiaomi_mca_test_legacy_hiz_recovery(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	u8 value = 0;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->fcc = 3200;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), 0);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 3200); /* Piano open_path has ONE ICL vote */
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_INPUT_SUSPEND, &value, 1));
	KUNIT_EXPECT_EQ(test, f->hiz_clear_writes + f->hiz_assert_writes, 0);
}

static void xiaomi_mca_test_pause_voter_start(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_named_stage(test, f, "sc8541");
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	mca->cp = MCA_CP_OFF;
	f->stage_on = false;
	f->type = mca->real_type = MCA_TYPE_PD_PPS;
	f->vbus = mca->vbus_uv = 9001000;
	mca->hv = MCA_HV_ON;
	f->pdos[1] = (struct xiaomi_mca_pdo) { cpu_to_le32(5000), cpu_to_le32(10000),
						cpu_to_le32(6100) };
	f->cells[0].voltage = f->cells[1].voltage = 4340000;
	f->cells[0].current_ua = f->cells[1].current_ua = -45000;
	f->stage_mv = 9000;
	f->stage_ma = 1800;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_try_start(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_OPENING);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 500);
	KUNIT_EXPECT_GT(test, mca->cp_request_ma, 0);
	KUNIT_EXPECT_EQ(test, f->hiz_assert_writes, 0);
	xiaomi_mca_cp_step(mca);
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_TRUE(test, f->stage_on);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "test complete"), 0);
	KUNIT_EXPECT_GT(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 500); /* normal CP owns ICL=0 only */
	KUNIT_EXPECT_GT(test, f->aicl_restarts, 0);
	KUNIT_EXPECT_TRUE(test, mca->buck_resume_pending);
	KUNIT_EXPECT_FALSE(test, f->resumed_before_stop);
}

static void xiaomi_mca_test_zero_vote_ack(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca_test_stage(test, f);
	guard(mutex)(&mca->lock);
	f->icl_read_stale = true; /* no strict numeric readback for zero command */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), 0);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_restore_icl(mca), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_reset_fcc(mca), 0);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 500);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write_u32(mca, MCA_PROP_INPUT_CURRENT_LIMIT, 500), -EBUSY);
	f->icl_read_stale = false;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "test complete"), 0);
	KUNIT_EXPECT_GT(test, f->icl, 0);
	KUNIT_EXPECT_EQ(test, f->hiz_assert_writes, 0);
}

static void xiaomi_mca_test_paused_detach(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca_test_stage(test, f);
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), 0);
	f->usb_online = 0;
	f->type = MCA_TYPE_UNKNOWN;
	f->vbus = 0;
	f->fail_aicl_prop = MCA_PROP_RESTART_AICL;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "physical disconnect"), 0);
	KUNIT_EXPECT_EQ(test, f->icl, MCA_ICL_DEFAULT_MA);
	KUNIT_EXPECT_EQ(test, f->fcc, MCA_FCC_DEFAULT_MA);
	KUNIT_EXPECT_EQ(test, f->aicl_restarts, 0);
	KUNIT_EXPECT_FALSE(test, mca->cp_input_suspended);
	KUNIT_EXPECT_PTR_EQ(test, mca->cp_psy, NULL);
}

static void xiaomi_mca_test_fw_recharge(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	u32 raw;
	int n, writes;

	full_charge = charge_policy = true;
	f->chgr_status = 5;
	f->type = mca->real_type = MCA_TYPE_PD_PPS;
	mca->ffc_active = true;
	mca->cell_zone = 18;
	f->term_readback = 5480;
	f->term_current = 0;
	f->pack_vbat = 4368;
	for (n = 0; n < 2; n++) {
		f->cells[n].voltage = 4370000;
		f->cells[n].current_ua = 0;
		f->cells[n].capacity = 93;
		f->cells[n].charge_now = 3999000;
		f->cells[n].charge_full = 4300000;
	}
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_raw_soc(mca, &raw), 0);
	KUNIT_EXPECT_EQ(test, raw, 9300);
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_fw_recharge_allowed(5, 9700));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_fw_recharge_allowed(5, 9701));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_fw_recharge_allowed(7, 9300));
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5360);
	KUNIT_EXPECT_EQ(test, f->term_current, 246);
	KUNIT_EXPECT_EQ(test, f->enable_writes, 2);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 1);
	KUNIT_EXPECT_FALSE(test, mca->ffc_active);
	writes = f->enable_writes;
	f->chgr_status = 3;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->enable_writes, writes + 1);
	f->chgr_status = 3;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->fw_recharge, 0);
	f->chgr_status = 5;
	f->cells[0].charge_now = f->cells[1].charge_now = 4250000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->enable_writes, writes + 3); /* full hold, above recharge gate */
	KUNIT_EXPECT_TRUE(test, mca->charging_done);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[1].charge_full = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_raw_soc(mca, &raw), -ENODATA);
}

static void xiaomi_mca_test_fw_recharge_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = true;
	f->chgr_status = 5;
	guard(mutex)(&mca->lock);
	/* No enable transition while a configured SOC limit is holding. */
	mca->charge_end = 80;
	f->cells[0].capacity = f->cells[1].capacity = 93;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 0);
	mca->charge_end = 100;
	f->fail_pmic_prop = MCA_PROP_CHARGING_ENABLED;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(mca), -EIO);
	KUNIT_EXPECT_EQ(test, mca->fw_recharge, 1);
	f->fail_pmic_prop = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 1);
	KUNIT_EXPECT_EQ(test, mca->fw_recharge, 3);
}

static void xiaomi_mca_test_pmic_disabled_snapshot(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca->last = (struct xiaomi_mca_state) { .online = 1, .chgr_status = 3 };
	mca->last_valid = true;
	mca->pmic_pack_mv = 4400;
	f->pack_vbat = 4588;
	f->term_current = 1250;
	f->term_readback = 5580;
	guard(mutex)(&mca->lock);
	xiaomi_mca_pmic_telemetry(mca, 7);
	KUNIT_EXPECT_EQ(test, mca->pmic_pack_mv, 4588);
	KUNIT_EXPECT_EQ(test, mca->pmic_iterm_raw, 1250);
	KUNIT_EXPECT_EQ(test, mca->pmic_fv_raw, 5580);
	f->pack_vbat_error = -EIO;
	xiaomi_mca_pmic_telemetry(mca, 7);
	KUNIT_EXPECT_EQ(test, mca->pmic_pack_mv, -1);
}

static void xiaomi_mca_test_pmic_disabled_transition(struct kunit *test)
{
	int before, after;

	for (before = 0; before <= 8; before++)
		for (after = 0; after <= 8; after++)
			KUNIT_EXPECT_EQ(test, xiaomi_mca_pmic_disabled(before, after),
				       after == 7 && (before <= 4 || before == 8));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_pmic_disabled(U32_MAX, 7));
}

static void xiaomi_mca_test_mipps_tail_replay(struct kunit *test)
{
	static const struct {
		int soc0, soc1, mv0, mv1, ma0, ma1, bus_mv, bus_ma;
	} recorded[] = {
		{ 84, 84, 4321, 4320, 524, 519, 8778, 670 }, /* 21:53:28 */
		{ 84, 84, 4351, 4349, 659, 653, 8868, 985 }, /* 21:53:33 */
		{ 84, 84, 4376, 4374, 844, 839, 8958, 1235 }, /* 21:53:38 */
		{ 84, 84, 4402, 4399, 1065, 1058, 9056, 1547 }, /* 21:53:43 */
		{ 84, 84, 4430, 4427, 1311, 1302, 9150, 1857 }, /* 21:53:48 */
		{ 84, 84, 4455, 4452, 1578, 1566, 9255, 2177 }, /* 21:53:53 */
		{ 84, 84, 4477, 4473, 1853, 1838, 9333, 2575 }, /* 21:53:58 */
		{ 84, 84, 4481, 4477, 2066, 2052, 9322, 2675 }, /* 21:54:03 */
		{ 84, 84, 4472, 4468, 2171, 2157, 9300, 2572 }, /* 21:54:08 */
		{ 84, 84, 4465, 4462, 2207, 2193, 9281, 2570 }, /* 21:54:13 */
		{ 84, 84, 4465, 4462, 2189, 2177, 9258, 2495 }, /* 21:54:18 */
		{ 84, 84, 4466, 4461, 2188, 2169, 9281, 2527 }, /* 21:54:23 */
		{ 84, 84, 4466, 4462, 2165, 2147, 9277, 2570 }, /* 21:54:28 */
		{ 84, 84, 4467, 4464, 2147, 2131, 9281, 2590 }, /* 21:54:33 */
		{ 84, 84, 4467, 4464, 2133, 2120, 9277, 2585 }, /* 21:54:38 */
		{ 85, 85, 4466, 4464, 2117, 2104, 9266, 2427 }, /* 21:54:43 */
		{ 85, 85, 4467, 4465, 2100, 2087, 9266, 2415 }, /* 21:54:48 */
		{ 85, 85, 4467, 4464, 2085, 2073, 9262, 2402 }, /* 21:54:53 */
		{ 85, 85, 4467, 4465, 2071, 2058, 9285, 2580 }, /* 21:54:58 */
		{ 85, 85, 4468, 4465, 2061, 2048, 9270, 2452 }, /* 21:55:03 */
		{ 85, 85, 4469, 4467, 2053, 2040, 9307, 2622 }, /* 21:55:08 */
		{ 85, 85, 4468, 4466, 2044, 2031, 9277, 2485 }, /* 21:55:13 */
		{ 85, 85, 4470, 4467, 2038, 2027, 9273, 2475 }, /* 21:55:18 */
		{ 85, 85, 4469, 4466, 2028, 2016, 9288, 2610 }, /* 21:55:23 */
		{ 85, 85, 4469, 4467, 2022, 2007, 9266, 2420 }, /* 21:55:28 */
		{ 85, 85, 4469, 4467, 2013, 1998, 9273, 2420 }, /* 21:55:33 */
		{ 85, 85, 4470, 4467, 2006, 1991, 9288, 2557 }, /* 21:55:38 */
		{ 85, 85, 4470, 4468, 2000, 1985, 9262, 2480 }, /* 21:55:43 */
		{ 85, 85, 4470, 4468, 1993, 1979, 9273, 2487 }, /* 21:55:48 */
		{ 86, 85, 4470, 4468, 1988, 1973, 9277, 1250 }, /* 21:55:53 */
		{ 86, 86, 4471, 4468, 1983, 1966, 9285, 2480 }, /* 21:55:58 */
		{ 86, 86, 4471, 4468, 1977, 1960, 9281, 2495 }, /* 21:56:03 */
		{ 86, 86, 4471, 4469, 1971, 1955, 9285, 2460 }, /* 21:56:08 */
		{ 86, 86, 4471, 4469, 1966, 1950, 9270, 2510 }, /* 21:56:13 */
		{ 86, 86, 4471, 4469, 1962, 1945, 9277, 2512 }, /* 21:56:18 */
		{ 86, 86, 4472, 4470, 1958, 1941, 9285, 2445 }, /* 21:56:23 */
		{ 86, 86, 4472, 4470, 1954, 1937, 9288, 2492 }, /* 21:56:28 */
		{ 86, 86, 4472, 4470, 1949, 1932, 9273, 2537 }, /* 21:56:33 */
		{ 86, 86, 4472, 4470, 1944, 1928, 9277, 2492 }, /* 21:56:38 */
		{ 86, 86, 4474, 4471, 1943, 1925, 9281, 2572 }, /* 21:56:43 */
		{ 86, 86, 4473, 4471, 1938, 1920, 9285, 2492 }, /* 21:56:48 */
		{ 86, 86, 4473, 4471, 1933, 1915, 9288, 2467 }, /* 21:56:53 */
		{ 86, 86, 4473, 4471, 1928, 1910, 9288, 2507 }, /* 21:56:58 */
		{ 86, 86, 4473, 4471, 1923, 1905, 9292, 2535 }, /* 21:57:03 */
		{ 86, 86, 4472, 4472, 1917, 1901, 9292, 2487 }, /* 21:57:08 */
		{ 86, 86, 4473, 4470, 1912, 1896, 9285, 2530 }, /* 21:57:13 */
		{ 87, 86, 4504, 4503, 2012, 2005, 9412, 2740 }, /* 21:57:18 */
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n, before;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_register_ucsi_peer(test, f);
	mca_test_stage(test, f);
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->cp_ma = 5000;
	mca->cp_max_mv = 10000;
	mca->cp_mv = 8940;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	mca->icl_set_ma = mca->fcc_set_ma = 500;
	mca->cp_ibat_trip = 12000;
	guard(mutex)(&mca->lock);
	for (n = 0; n < ARRAY_SIZE(recorded); n++) {
		f->cells[0].capacity = recorded[n].soc0;
		f->cells[1].capacity = recorded[n].soc1;
		f->cells[0].voltage = recorded[n].mv0 * 1000;
		f->cells[1].voltage = recorded[n].mv1 * 1000;
		f->cells[0].current_ua = recorded[n].ma0 * 1000;
		f->cells[1].current_ua = recorded[n].ma1 * 1000;
		mca->vbus_uv = f->vbus = recorded[n].bus_mv * 1000;
		f->stage_mv = recorded[n].bus_mv;
		f->stage_ma = recorded[n].bus_ma;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		xiaomi_mca_cp_step(mca);
		KUNIT_ASSERT_EQ_MSG(test, mca->cp, MCA_CP_ON, "row %d", n);
		KUNIT_EXPECT_EQ(test, mca->cp_exits, 0);
	}
	KUNIT_ASSERT_TRUE(test, mca->ffc_active);
	/* The fixed old ADC trace does not respond to larger CC requests.
	 * It can now reach the APDO ceiling; it must never exceed it. The
	 * following CV observations still require reduction and handoff.
	 */
	KUNIT_EXPECT_LE(test, mca->cp_mv, mca->cp_max_mv);

	/* Constructed future observations, not measurements of a charged pack. */
	f->cells[0].capacity = f->cells[1].capacity = 95;
	f->cells[0].voltage = f->cells[1].voltage = 4540000;
	f->cells[0].current_ua = f->cells[1].current_ua = 2000000;
	before = mca->cp_mv;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_LT(test, mca->cp_mv, before);
	f->cells[0].voltage = f->cells[1].voltage = 4535000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1499000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	for (int n = 0; n < 4; n++)
		xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_DONE);
	KUNIT_EXPECT_EQ(test, mca->cp_exits, 1);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);

	f->type = mca->real_type = MCA_TYPE_PD;
	mca->vbus_uv = f->vbus = 9000000;
	mca->hv = MCA_HV_ON;
	mca->hv_pps = false;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(3000) };
	f->ucsi_online = 1;
	f->ucsi_type = POWER_SUPPLY_USB_TYPE_PD;
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	f->ucsi_mv = 9000;
	f->ucsi_rp_ua = 3000000;
	f->charging_enabled = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	xiaomi_mca_apply_policy(mca);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, f->icl, 2500);
	KUNIT_EXPECT_EQ(test, f->fcc, 4000);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
	f->cells[0].capacity = f->cells[1].capacity = 100;
	f->cells[0].voltage = f->cells[1].voltage = 4539000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1175000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->ffc_done);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
	xiaomi_mca_cp_try_start(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	/* Synthetic continuation of the recorded 84..87% prefix: buck CV,
	 * AP full vote at the cell endpoint, then the board's raw97 recharge.
	 */
	f->chgr_status = 4;
	f->cells[0].voltage = f->cells[1].voltage = 4542000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1100000;
	f->cells[0].charge_now = f->cells[1].charge_now = 4554000;
	for (int n = 0; n < 6; n++) {
		mca->term_at = jiffies - MCA_POLL_INTERVAL;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	}
	KUNIT_EXPECT_TRUE(test, mca->charging_done);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	f->cells[0].voltage = f->cells[1].voltage = 4450000;
	f->cells[0].charge_now = f->cells[1].charge_now = 4462000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->charging_done);
	KUNIT_EXPECT_FALSE(test, mca->ffc_active);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);

}

static void xiaomi_mca_test_ffc_handoff(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	mca_register_ucsi_peer(test, f);
	full_charge = charge_policy = hv_charge = cp_charge = true;
	mca_test_stage(test, f);
	f->tbat = 25;
	f->type = mca->cp_type = MCA_TYPE_PD_PPS;
	f->vbus = 9700000;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(3000) };
	mca->cp_max_mv = 11000;
	mca->cp_ma = 5000;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_TRUE(test, mca->ffc_active);
	/* Previous CV observation was 4 A, before falling to the threshold. */
	KUNIT_ASSERT_FALSE(test, xiaomi_mca_cp_taper_complete(mca, true, 4000));
	f->cells[0].voltage = 4539000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1499000;
	for (int n = 0; n < 4 && mca->cp == MCA_CP_ON; n++)
		xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	f->type = MCA_TYPE_PD;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	f->vbus = 8900000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	f->ucsi_online = 1;
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	f->ucsi_type = POWER_SUPPLY_USB_TYPE_PD;
	f->ucsi_mv = 9000;
	f->ucsi_rp_ua = 3000000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	f->ucsi_online = 0;
	f->pdos[1].min_mv = cpu_to_le32(5000);
	f->pdos[1].max_mv = cpu_to_le32(11000);
	mca->hv = MCA_HV_ON;
	mca->hv_pps = true;
	mca->hv_pps_ma = 3000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	mca->hv = MCA_HV_OFF;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);

	/* The cell endpoint wins even while contract telemetry is absent. */
	f->cells[0].current_ua = f->cells[1].current_ua = 1175000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->ffc_done);
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
}

static void xiaomi_mca_test_pd_recovery(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_auth_fixture(test);
	u32 type;

	mca_register_ucsi_peer(test, f);
	f->ucsi_online = 1;
	f->ucsi_type = POWER_SUPPLY_USB_TYPE_C;
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	f->ucsi_rp_ua = 1500000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_5v_input(&f->mca, MCA_TYPE_C), 1500);
	f->ucsi_rp_ua = 3000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_5v_input(&f->mca, MCA_TYPE_C), 3000);
	f->ucsi_rp_ua = 500000;
	f->ucsi_charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_5v_input(&f->mca, MCA_TYPE_UNKNOWN), 1000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_5v_input(&f->mca, MCA_TYPE_FLOAT), 3000);
	f->ucsi_rp_ua = 3000000;
	f->ucsi_status = POWER_SUPPLY_STATUS_DISCHARGING;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_5v_input(&f->mca, MCA_TYPE_C), 500);
	f->ucsi_type = POWER_SUPPLY_USB_TYPE_PD;
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	f->type = MCA_TYPE_UNKNOWN;
	f->pd_state = MCA_PD_SNK_READY;
	guard(mutex)(&f->mca.lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), 0);
	KUNIT_EXPECT_EQ(test, type, MCA_TYPE_PD);
	/* The refusal gate must use the same evidence as the policy poll. */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 1500), 0);
	f->vbus = 9000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 1600), 0);
	f->ucsi_mv = 9000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_contract_ma(&f->mca, 9000, 1600), 3000);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2500), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2501), -EINVAL);
	f->ucsi_rp_ua = 2000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2000), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2001), -EINVAL);
	f->ucsi_online = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_contract_ma(&f->mca, 9000, 1600), 1600);
	f->ucsi_online = 1;
	f->ucsi_rp_ua = 1600000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_write(&f->mca, MCA_PROP_INPUT_CURRENT_LIMIT, 2500),
			-EINVAL);
	f->ucsi_online = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), 0);
	KUNIT_EXPECT_EQ(test, type, MCA_TYPE_UNKNOWN);
	f->ucsi_online = 1;
	f->ucsi_status = POWER_SUPPLY_STATUS_DISCHARGING;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), 0);
	KUNIT_EXPECT_EQ(test, type, MCA_TYPE_UNKNOWN);
	f->ucsi_status = POWER_SUPPLY_STATUS_CHARGING;
	f->pd_state = 1;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), 0);
	KUNIT_EXPECT_EQ(test, type, MCA_TYPE_UNKNOWN);
	f->pd_state = MCA_PD_SNK_READY;
	f->pdos[0].max_ma = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), 0);
	KUNIT_EXPECT_EQ(test, type, MCA_TYPE_UNKNOWN);
	f->error = -EIO;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_read_type(&f->mca, &type), -EIO);
}

static void mca_pen_test_delay(unsigned int ms)
{
	struct mca_otg_fixture *f = kunit_get_current_test()->priv;

	if (f->delay_count < ARRAY_SIZE(f->delays))
		f->delays[f->delay_count++] = ms;
}

static void xiaomi_mca_test_pen_boost(struct kunit *test)
{
	static const unsigned int delays[] = { 100, 20, 20, 20, 10 };
	static const u32 values[] = { 0x11388, 0x20101, 0x1157c, 0x11770,
				      0x11964, 0x11450, 0x20100 };
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	int i;

	test->priv = f;
	kunit_activate_static_stub(test, xiaomi_mca_pen_delay, mca_pen_test_delay);
	mutex_lock(&f->mca.lock);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_off(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	KUNIT_EXPECT_EQ(test, f->calls, 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_pen_on(&f->mca), 0);
	KUNIT_EXPECT_TRUE(test, f->mca.pen_on);
	KUNIT_EXPECT_EQ(test, f->mca.pen_generation, 1U);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_pen_on(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 5);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_pen_off(&f->mca), 0);
	KUNIT_EXPECT_FALSE(test, f->mca.pen_on);
	KUNIT_EXPECT_FALSE(test, f->mca.pen_dirty);
	KUNIT_EXPECT_EQ(test, f->actions[0], SC8541_PEN_ISOLATE);
	KUNIT_EXPECT_EQ(test, f->actions[1], SC8541_PEN_RESTORE);
	KUNIT_EXPECT_MEMEQ(test, f->delays, delays, sizeof(delays));
	for (i = 0; i < ARRAY_SIZE(values); i++) {
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->data[i]), values[i]);
		KUNIT_EXPECT_EQ(test, f->props[i], (i == 1 || i == 6) ?
			MCA_PROP_PEN_BOOST_ENABLE : MCA_PROP_PEN_BOOST_VOLTAGE);
	}
	mutex_unlock(&f->mca.lock);
}

static void xiaomi_mca_test_pen_failures(struct kunit *test)
{
	struct mca_otg_fixture *f = mca_otg_fixture(test);
	int i;

	test->priv = f;
	kunit_activate_static_stub(test, xiaomi_mca_pen_delay, mca_pen_test_delay);
	mutex_lock(&f->mca.lock);
	/* Failed ramp writes are rolled back, including a lost enable ACK. */
	for (i = 1; i <= 5; i++) {
		f->writes = f->calls = f->delay_count = 0;
		f->fail_write = i;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), -EIO);
		KUNIT_EXPECT_FALSE(test, f->mca.pen_on);
		KUNIT_EXPECT_FALSE(test, f->mca.pen_dirty);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(f->data[f->writes - 1]), 0x20100U);
	}
	f->fail_write = f->writes = f->calls = 0;
	f->cp_result = -EBUSY;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), -EBUSY);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->cp_result = 0;
	f->pen_vbus = 5000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), -EBUSY);
	KUNIT_EXPECT_EQ(test, f->writes, 0);
	f->pen_vbus = 0;
	f->pen_restart_write = 3;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), -ECONNRESET);
	KUNIT_EXPECT_FALSE(test, f->mca.pen_on);
	f->pen_restart_write = 0;
	f->writes = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->mca.pen_generation, f->mca.service_generation);
	f->mca.service_generation++;
	f->writes = 0;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_on(&f->mca), 0);
	KUNIT_EXPECT_EQ(test, f->writes, 5);
	KUNIT_EXPECT_EQ(test, f->mca.pen_generation, f->mca.service_generation);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_pen_off(&f->mca), 0);
	mutex_unlock(&f->mca.lock);
}

static void xiaomi_mca_test_pen_allowlist(struct kunit *test)
{
	__le32 value = cpu_to_le32(0x20101);

	KUNIT_EXPECT_TRUE(test, xiaomi_mca_write_allowed(MCA_PROP_PEN_BOOST_ENABLE,
						      &value, sizeof(value)));
	value = cpu_to_le32(0x20102);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_PEN_BOOST_ENABLE,
						       &value, sizeof(value)));
	value = cpu_to_le32(BIT(16) | 6600);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_PEN_BOOST_VOLTAGE,
						       &value, sizeof(value)));
	value = cpu_to_le32(6500);
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_write_allowed(MCA_PROP_PEN_BOOST_VOLTAGE,
						       &value, sizeof(value)));
}

/* Ordered actual-board teardown, including ICL-only pause and the final FV. */
static void xiaomi_mca_test_stock_handoff_order(struct kunit *test)
{
	static const struct { u32 prop, value; } order[] = {
		{ 0xff0001, 0 },
		{ MCA_PROP_PD_FIXED_VOLT, 5000 }, { MCA_PROP_PD_FIXED_VOLT, 9000 },
		{ 0xff0002, SC8541_CHARGE_MODE_RESET },
		{ MCA_PROP_INPUT_CURRENT_LIMIT, 500 }, { MCA_PROP_CHARGE_CURRENT, 500 },
		{ 0xff0002, SC8541_CHARGE_ADC_OFF }, { MCA_PROP_TERM_VOLT, 4590 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int i, next = 0;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->type = mca->real_type = MCA_TYPE_PD_PPS;
	f->vbus = mca->vbus_uv = 9000000;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(9000), cpu_to_le32(9000), cpu_to_le32(3000) };
	mca->ffc_active = true;
	mca->cell_zone = 18;
	mca->cp_fv_base = 4650;
	f->term_readback = 5780;
	f->fcc = 3000;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_input_suspend(mca, true), 0);
	KUNIT_EXPECT_EQ(test, f->fcc, 3000);
	f->trace_count = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "taper complete"), 0);
	for (i = 0; i < f->trace_count; i++) {
		KUNIT_EXPECT_NE(test, f->trace[i].prop, (u32)MCA_PROP_INPUT_SUSPEND);
		if (next < ARRAY_SIZE(order) && f->trace[i].prop == order[next].prop &&
		    f->trace[i].value == order[next].value)
			next++;
	}
	KUNIT_EXPECT_EQ(test, next, (int)ARRAY_SIZE(order));
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	KUNIT_EXPECT_FALSE(test, mca->cp_input_suspended);
}

/* 9ccb 822.44 s: one failed read at 24 C must never manufacture 4060 mV. */
static void xiaomi_mca_test_822s_read_failure(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	static const struct { int uptime, mv, ma, soc; } samples[] = {
		{ 822, 4460, 550, 95 }, { 684, 4464, 380, 97 },
	};
	u32 writes;
	int n;

	full_charge = charge_policy = charge_current = hv_charge = true;
	f->type = mca->real_type = MCA_TYPE_PD;
	f->vbus = mca->vbus_uv = 9000000;
	f->chgr_status = 4;
	f->tbat = 24;
	f->cells[0].capacity = f->cells[1].capacity = 95;
	f->cells[0].temp = f->cells[1].temp = 240;
	f->cells[0].voltage = f->cells[1].voltage = 4460000;
	f->cells[0].current_ua = f->cells[1].current_ua = 550000;
	f->pack_vbat = 4469;
	mca->cp = MCA_CP_DONE;
	guard(mutex)(&mca->lock);
	for (n = 0; n < ARRAY_SIZE(samples); n++) {
		f->cells[0].voltage = f->cells[1].voltage = samples[n].mv * 1000;
		f->cells[0].current_ua = f->cells[1].current_ua = samples[n].ma * 1000;
		f->cells[0].capacity = f->cells[1].capacity = samples[n].soc;
		f->vbus = mca->vbus_uv = 9000000;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		KUNIT_ASSERT_EQ(test, f->term_readback, 5360);
		writes = f->term_writes;
		f->pack_error = -EIO;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
		xiaomi_mca_pmic_telemetry(mca, f->chgr_status);
		KUNIT_EXPECT_FALSE(test, mca->pmic_pack_tbat_valid);
		KUNIT_EXPECT_EQ(test, mca->gauge_failures, 0);
		KUNIT_EXPECT_EQ(test, f->term_writes, writes);
		KUNIT_EXPECT_EQ(test, f->term_readback, 5360);
		KUNIT_EXPECT_FALSE(test, mca->term_hot);
		KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
		/* Replay the old firmware-disabled result too: correction must rearm it. */
		f->pack_error = 0;
		f->chgr_status = 7;
		f->trace_count = 0;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
		KUNIT_EXPECT_EQ(test, f->trace[0].prop, (u32)MCA_PROP_PD_FIXED_VOLT);
		KUNIT_EXPECT_EQ(test, f->trace[0].value, 5000);
		KUNIT_EXPECT_EQ(test, f->trace[1].prop, (u32)MCA_PROP_CHARGING_ENABLED);
		KUNIT_EXPECT_EQ(test, f->trace[1].value, 0);
		KUNIT_EXPECT_EQ(test, f->trace[2].prop, (u32)MCA_PROP_CHARGING_ENABLED);
		KUNIT_EXPECT_EQ(test, f->trace[2].value, 1);
		KUNIT_EXPECT_TRUE(test, mca->recover_5v);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_hv_step(mca), 0);
		f->chgr_status = 4;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
		KUNIT_EXPECT_FALSE(test, mca->recover_5v);
	}
}

static void xiaomi_mca_test_full_recharge_matrix(struct kunit *test)
{
	static const u32 types[] = { MCA_TYPE_SDP, MCA_TYPE_DCP, MCA_TYPE_PD,
		MCA_TYPE_PD_PPS, MCA_TYPE_PD_VERIFY, MCA_TYPE_HVDCP_3_B, MCA_TYPE_HVDCP_3P5 };
	static const int temps[] = { 10, 15, 25, 40, 50 };
	static const int socs[] = { 84, 94, 95, 100 };
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int type, t, soc, event, n;

	full_charge = charge_policy = true;
	guard(mutex)(&mca->lock);
	/* source x SOC x JEITA x {start, full, recharge, firmware 6/7}. */
	for (type = 0; type < ARRAY_SIZE(types); type++)
	 for (t = 0; t < ARRAY_SIZE(temps); t++)
	  for (soc = 0; soc < ARRAY_SIZE(socs); soc++)
	   for (event = 0; event < 5; event++) {
		bool ffc = types[type] != MCA_TYPE_SDP && socs[soc] < 95 &&
			temps[t] >= 18 && temps[t] < 48;
		u32 fv;

		f->type = mca->real_type = types[type];
		f->vbus = mca->vbus_uv = 5000000;
		f->tbat = temps[t];
		f->chgr_status = event >= 3 ? event + 3 : 3;
		mca->cell_zone = 0;
		mca->term_hot = mca->zone_valid = mca->ffc_active = mca->ffc_done = false;
		mca->charging_done = mca->charge_hold = mca->voltage_hold = false;
		mca->charge_hot = mca->charge_cold = mca->charge_blocked = false;
		mca->cp = MCA_CP_DONE;
		mca->cp_input_suspended = mca->charge_disabled = false;
		mca->term_count = mca->buck_cv_steps = mca->buck_cv_fcc = 0;
		mca->cp_fv_base = 0;
		for (n = 0; n < 2; n++) {
			f->cells[n].temp = temps[t] * 10;
			f->cells[n].voltage = (temps[t] >= 48 ? 3900 : 4000) * 1000;
			f->cells[n].current_ua = 2000000;
			f->cells[n].capacity = socs[soc];
			f->cells[n].charge_full = 4600000;
			f->cells[n].charge_now = 4554000; /* raw 99%, full must hold */
		}
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		KUNIT_EXPECT_EQ(test, mca->ffc_active, ffc);
		fv = xiaomi_mca_cell_fv(mca);
		if (event == 1 || event == 2) {
			f->chgr_status = 5;
			f->cells[0].current_ua = f->cells[1].current_ua = 0;
			KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
			KUNIT_EXPECT_EQ(test, mca->charging_done, temps[t] < 48);
			KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
			if (event == 2) {
				f->cells[0].charge_now = f->cells[1].charge_now = 4462000; /* 97% */
				KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
				KUNIT_EXPECT_FALSE(test, mca->charging_done);
				KUNIT_EXPECT_FALSE(test, mca->ffc_active);
				KUNIT_EXPECT_EQ(test, f->charging_enabled, temps[t] < 48);
			}
		} else {
			KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
			KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
			KUNIT_EXPECT_FALSE(test, mca->charging_done);
		}
		KUNIT_EXPECT_GE(test, fv, (u32)MCA_TERM_HOT_MV);
		KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
	   }
}

static void xiaomi_mca_test_software_full_samples(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n;

	mca->cell_zone = 18;
	for (n = 1; n <= 6; n++) {
		mca->term_at = jiffies - MCA_POLL_INTERVAL;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_software_full(mca, 4472, 450, 225), n == 6);
	}
	mca->term_count = 0;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4472, 450, 225));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4472, 450, 225));
	KUNIT_EXPECT_EQ(test, mca->term_count, 1); /* no repeated use of a sample */
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4368, 0, 0)); /* unequal/low cell */
	KUNIT_EXPECT_EQ(test, mca->term_count, 0);
	/* Opposing cell currents must not masquerade as a tapered pack. */
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4472, 100, 3000));
	KUNIT_EXPECT_EQ(test, xiaomi_mca_weighted_soc((u32[]){4714, 4016},
		(u32[]){4900, 4100}), 9701U);

}

#ifdef MCA_CHARGE_EXPERIMENT
static void xiaomi_mca_test_experiment(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	INIT_DELAYED_WORK(&mca->poll_work, mca_test_idle_poll);
	f->cells[0].capacity = f->cells[1].capacity = 50;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_experiment_low(3500, 9, 0));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_experiment_low(3499, 50, 0));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_experiment_low(4000, 8, 0));
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_experiment_low(4000, 50, -EIO));
	f->cells[1].voltage = 3499000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_experiment_command(mca, "discharge"), -ERANGE);
	f->cells[1].voltage = 4000000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_experiment_command(mca, "resume"), -EINVAL);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_experiment_command(mca, "enable 0"), -EPERM);
	mca_test_stage(test, f);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_experiment_command(mca, "observe"), -EBUSY);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_experiment_command(mca, "discharge"), 0);
	KUNIT_EXPECT_TRUE(test, mca->experiment);
	KUNIT_EXPECT_FALSE(test, f->stage_on);
	KUNIT_EXPECT_EQ(test, f->icl, 0);
	KUNIT_EXPECT_NE(test, f->fcc, 0); /* only the factory ICL vote */
	mca->fg_full_mask = 3;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_experiment_command(mca, "resume"), 0);
	KUNIT_EXPECT_EQ(test, mca->fg_full_mask, 0);
	cancel_delayed_work_sync(&mca->poll_work);
	KUNIT_EXPECT_FALSE(test, mca->experiment);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OFF);
	KUNIT_EXPECT_EQ(test, f->icl, 500);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_experiment_command(mca, "observe"), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_experiment_command(mca, "enable 0"), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_experiment_command(mca, "enable 1"), 0);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	mca->otg_active = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_experiment_command(mca, "icl 500"), -EBUSY);
}
#endif

static void xiaomi_mca_test_cycle_latch_and_sample(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	struct xiaomi_mca_cells cells;
	full_charge = charge_policy = true;
	guard(mutex)(&mca->lock);
	xiaomi_mca_sample_cells(mca, &cells);
	mca->poll_cells = &cells;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_TRUE(test, mca->ffc_active);
	KUNIT_EXPECT_EQ(test, f->pack_reads, 0);
	f->pack_error = -EIO;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->pack_reads, 0); /* PMIC diagnostic is not a control sensor */
	xiaomi_mca_pmic_telemetry(mca, 3);
	KUNIT_EXPECT_EQ(test, f->pack_reads, 1);
	KUNIT_EXPECT_FALSE(test, mca->pmic_pack_tbat_valid);
	f->pack_error = 0;
	f->tbat = 60;
	xiaomi_mca_pmic_telemetry(mca, 3);
	KUNIT_EXPECT_TRUE(test, mca->pmic_pack_tbat_valid);
	KUNIT_EXPECT_EQ(test, mca->pmic_pack_tbat, 60);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->term_hot); /* both real cells remain at 25 C */
	mca->poll_cells = NULL;
	mca->charging_done = true;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active); /* only recharge clears FFC */
	KUNIT_EXPECT_EQ(test, f->term_readback, 5580);
	f->cells[0].voltage = f->cells[1].voltage = 4500000;
	f->cells[0].charge_now = f->cells[1].charge_now = 4400000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->ffc_active);
	KUNIT_EXPECT_TRUE(test, mca->voltage_hold);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0); /* no normal restart above FV */
}

static void xiaomi_mca_test_firmware_error_identity(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);
	struct xiaomi_mca_test_frame frame = {
		"failed pack temperature", PMIC_GLINK_REQ_RESP, MCA_OPCODE_READ,
		{ MCA_PROP_PACK_TBAT, 0x21, 7, 0 },
	};

	KUNIT_ASSERT_NOT_NULL(test, mca);
	spin_lock_init(&mca->rx_lock);
	init_completion(&mca->ack);
	mca->pending = true;
	mca->pending_opcode = MCA_OPCODE_READ;
	mca->pending_property = MCA_PROP_PACK_TBAT;
	mca->pending_seq = 7;
	xiaomi_mca_callback(mca_test_reply(test, &frame), sizeof(struct xiaomi_mca_reply), mca);
	KUNIT_EXPECT_EQ(test, mca->failed_property, (u32)MCA_PROP_PACK_TBAT);
	KUNIT_EXPECT_EQ(test, mca->failed_retcode, 0x21U);
	KUNIT_EXPECT_EQ(test, mca->failed_errno, -EIO);
	KUNIT_EXPECT_EQ(test, mca->stats.errors, 1);
	frame.payload[0] = MCA_PROP_PACK_VBAT;
	frame.payload[1] = 0;
	frame.payload[2] = 8;
	frame.payload[3] = 4469;
	mca->pending = true;
	mca->pending_property = MCA_PROP_PACK_VBAT;
	mca->pending_seq = 8;
	xiaomi_mca_callback(mca_test_reply(test, &frame), sizeof(struct xiaomi_mca_reply), mca);
	KUNIT_EXPECT_EQ(test, mca->failed_property, (u32)MCA_PROP_PACK_TBAT);
	KUNIT_EXPECT_EQ(test, mca->failed_retcode, 0x21U); /* later success cannot erase evidence */
}


static void xiaomi_mca_test_stock_entry_matrix(struct kunit *test)
{
	static const u32 types[] = { MCA_TYPE_SDP, MCA_TYPE_CDP, MCA_TYPE_DCP,
		MCA_TYPE_PD_PPS, MCA_TYPE_PD_VERIFY, MCA_TYPE_HVDCP_3_B, MCA_TYPE_HVDCP_3P5 };
	static const int temps[] = { 10, 25, 50 };
	static const int socs[] = { 84, 98 };
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int type, t, soc;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_named_stage(test, f, "sc8541");
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	f->stage_on = false;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(5000), cpu_to_le32(10000), cpu_to_le32(5000) };
	guard(mutex)(&mca->lock);
	for (type = 0; type < ARRAY_SIZE(types); type++)
	 for (t = 0; t < ARRAY_SIZE(temps); t++)
	  for (soc = 0; soc < ARRAY_SIZE(socs); soc++) {
		bool pump = type >= 3 && socs[soc] < 91;

		mca->cp = MCA_CP_OFF;
		mca->hv = MCA_HV_ON;
		mca->cp_fault = mca->charging_done = false;
		mca->ffc_active = mca->ffc_done = mca->term_hot = mca->zone_valid = false;
		f->type = mca->real_type = types[type];
		f->vbus = mca->vbus_uv = 9000000;
		f->tbat = temps[t];
		f->cells[0].temp = f->cells[1].temp = temps[t] * 10;
		f->cells[0].capacity = f->cells[1].capacity = socs[soc];
		f->cells[0].voltage = f->cells[1].voltage = (temps[t] >= 48 ? 3900 : 4000) * 1000;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
		xiaomi_mca_cp_try_start(mca);
		KUNIT_EXPECT_EQ_MSG(test, mca->cp, pump ? MCA_CP_OPENING : MCA_CP_OFF,
			"source=%u SOC=%d temp=%d", types[type], socs[soc], temps[t]);
		if (pump) {
			KUNIT_EXPECT_EQ(test, f->icl, 0);
			KUNIT_EXPECT_NE(test, f->fcc, 0);
			KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "taper complete"), 0);
			KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_DONE);
		}
	  }
}

static void xiaomi_mca_test_stock_cc_cv_stages(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mca->cp_profile = MCA_PROFILE_FFC;
	mca->cp_ma = 6000;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4144, 12000), 10650); /* skip first CV */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4300, 10000), 10650); /* hold CV */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4300, 9040), 9040);   /* next CC */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4475, 8000), 9040);   /* hold CV */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ibat_limit(mca, 4475, 7230), 7230);
	KUNIT_EXPECT_EQ(test, mca->cp_stage, 3U);
}

static void xiaomi_mca_test_provider_late_start(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	f->type = mca->real_type = MCA_TYPE_PD_PPS;
	f->vbus = mca->vbus_uv = 9000000;
	mca->hv = MCA_HV_ON;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(5000), cpu_to_le32(11000), cpu_to_le32(5000) };
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	xiaomi_mca_cp_try_start(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OFF);
	KUNIT_EXPECT_PTR_EQ(test, mca->cp_psy, NULL);
	mca_test_named_stage(test, f, "sc8541");
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	mca->cp = MCA_CP_OFF;
	f->stage_on = false;
	xiaomi_mca_cp_try_start(mca);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OPENING);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, "test complete"), 0);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OFF);
}

static void xiaomi_mca_test_warm_termination_is_not_full(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int writes;

	full_charge = charge_policy = hv_charge = true;
	f->type = mca->real_type = MCA_TYPE_PD;
	mca->cp = MCA_CP_DONE;
	f->chgr_status = 5;
	f->tbat = 50;
	f->cells[0].temp = f->cells[1].temp = 500;
	f->cells[0].voltage = f->cells[1].voltage = 4058000;
	f->cells[0].charge_now = f->cells[1].charge_now = 1000000;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->voltage_hold);
	KUNIT_EXPECT_FALSE(test, mca->charging_done);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	writes = f->enable_writes;
	f->chgr_status = 7;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, f->enable_writes, writes + 1); /* repeat disable, never rearm */
	f->tbat = 25;
	f->cells[0].temp = f->cells[1].temp = 250;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_FALSE(test, mca->voltage_hold);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 1);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 0); /* thermal recovery, not full */
}

static void xiaomi_mca_test_real_full_v5(struct kunit *test)
{
	/* 01:21..01:36, normal: each cell 4467..4470 mV, 130..169 mA. */
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	static const int samples[][3] = {
		{ 4467, 335, 169 },
		{ 4468, 328, 165 },
		{ 4468, 322, 162 },
		{ 4468, 318, 160 },
		{ 4468, 310, 156 },
		{ 4468, 305, 154 },
	};
	int i;

	mca->cell_zone = 18;
	for (i = 0; i < 6; i++) {
		mca->term_at = jiffies - MCA_POLL_INTERVAL;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_software_full(mca,
			samples[i][0], samples[i][1], samples[i][2]), i == 5);
	}
	mca->term_count = 0;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4457, 260, 130));
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4470, 450, 256));
	mca->ffc_active = true;
	for (i = 0; i < 6; i++) {
		mca->term_at = jiffies - MCA_POLL_INTERVAL;
		KUNIT_EXPECT_EQ(test, xiaomi_mca_software_full(mca, 4540, 2200, 1100), i == 5);
	}
	mca->term_count = 0;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_software_full(mca, 4535, 2200, 1100));
	for (i = 0; i < 2; i++) {
		f->cells[i].capacity = 100;
		f->cells[i].voltage = 4410000;
	}
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_soc_ready(mca));
	f->cells[0].capacity = f->cells[1].capacity = 90;
	KUNIT_EXPECT_TRUE(test, xiaomi_mca_cp_soc_ready(mca));
	f->cells[1].capacity = 91;
	KUNIT_EXPECT_FALSE(test, xiaomi_mca_cp_soc_ready(mca));
}

static void xiaomi_mca_test_exit_retry_class(struct kunit *test)
{
	static const char * const transient[] = { "float voltage write failed",
		"read failed", "stage turned itself off", "stage temperature",
		"battery protection", "request refused", "ADSP restart" };
	struct xiaomi_mca mca = {};
	int i;

	for (i = 0; i < ARRAY_SIZE(transient); i++) {
		mca.cp = MCA_CP_ON;
		xiaomi_mca_cp_session_done(&mca, transient[i]);
		KUNIT_EXPECT_EQ(test, mca.cp, MCA_CP_OFF);
		KUNIT_EXPECT_STREQ(test, mca.cp_last_exit, transient[i]);
	}
	xiaomi_mca_cp_session_done(&mca, "taper complete");
	KUNIT_EXPECT_EQ(test, mca.cp, MCA_CP_DONE);
	mca.cp_fault = true;
	xiaomi_mca_cp_session_done(&mca, "bus current too high");
	KUNIT_EXPECT_EQ(test, mca.cp, MCA_CP_DONE);
	mca.cp_fault = false;
	mca.auth_lost = true;
	xiaomi_mca_cp_session_done(&mca, "adapter authentication lost");
	KUNIT_EXPECT_EQ(test, mca.cp, MCA_CP_DONE);
}

static void xiaomi_mca_test_stock_failure_budget(struct kunit *test)
{
	static const struct { const char *reason; int limit; } cases[] = {
		{ "adapter request refused", 11 }, { "stage turned itself off", 5 },
	};
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	u32 writes, sample;
	int row, n;

	full_charge = charge_policy = charge_current = cp_charge = hv_charge = true;
	mca_test_named_stage(test, f, "sc8541");
	power_supply_put(mca->cp_psy);
	mca->cp_psy = NULL;
	mca->cp = MCA_CP_OFF;
	f->stage_on = false;
	f->pdos[1] = (struct xiaomi_mca_pdo) {
		cpu_to_le32(5000), cpu_to_le32(11000), cpu_to_le32(5000) };
	guard(mutex)(&mca->lock);
	mca->cp = MCA_CP_OPENING;
	xiaomi_mca_cp_count_failure(mca, "stage read failed");
	KUNIT_EXPECT_EQ(test, mca->cp_total_errors, 1);
	KUNIT_EXPECT_EQ(test, mca->cp_chip_errors, 0);
	mca->cp = MCA_CP_ON;
	xiaomi_mca_cp_count_failure(mca, "stage read failed");
	KUNIT_EXPECT_EQ(test, mca->cp_total_errors, 2);
	KUNIT_EXPECT_EQ(test, mca->cp_chip_errors, 1);
	mca->cp = MCA_CP_OFF;
	mca->cp_total_errors = mca->cp_chip_errors = 0;
	for (row = 0; row < ARRAY_SIZE(cases); row++) {
		for (n = 0; n < cases[row].limit; n++) {
			mca->hv = MCA_HV_ON;
			f->vbus = mca->vbus_uv = 9000000;
			f->type = mca->real_type = MCA_TYPE_PD;
			f->usb_online = 1;
			xiaomi_mca_cp_try_start(mca);
			KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_OPENING);
			KUNIT_EXPECT_EQ(test, f->icl, 0);
			if (n == 2) {
				/* A successful running observation is not a reset. */
				mca->cp = MCA_CP_ON;
				f->stage_on = true;
				f->stage_mv = 8400;
				f->stage_ma = 1500;
				xiaomi_mca_cp_step(mca);
				KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
				KUNIT_EXPECT_EQ(test, mca->cp_total_errors, n);
			}
			if (!row && !n) {
				f->stage_error = -EIO;
				KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, cases[row].reason), -EIO);
				KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stop(mca, "pending stop"), -EIO);
				KUNIT_EXPECT_EQ(test, mca->cp_total_errors, 1);
				f->stage_error = 0;
			}
			KUNIT_ASSERT_EQ(test, xiaomi_mca_cp_stop(mca, cases[row].reason), 0);
			KUNIT_EXPECT_EQ(test, mca->cp_total_errors, n + 1);
			KUNIT_EXPECT_EQ(test, mca->cp_chip_errors, row ? n + 1 : 0);
			KUNIT_EXPECT_GT(test, f->icl, 0);
			KUNIT_EXPECT_EQ(test, mca->cp,
				n + 1 == cases[row].limit ? MCA_CP_DONE : MCA_CP_OFF);
		}
		xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_DOWN);
		xiaomi_mca_pdr_notify(mca, SERVREG_SERVICE_STATE_UP);
		KUNIT_EXPECT_EQ(test, mca->cp_total_errors, cases[row].limit);
		writes = f->writes;
		for (n = 0; n < 3; n++)
			xiaomi_mca_cp_try_start(mca);
		KUNIT_EXPECT_EQ(test, f->writes, writes); /* no more zero ICL */
		KUNIT_EXPECT_GT(test, f->icl, 0);
		/* Actual detach handling clears the budget, not a VBUS dip/PDR. */
		f->vbus = f->usb_online = 0;
		xiaomi_mca_ucsi_update(mca, MCA_UCSI_OFFLINE);
		xiaomi_mca_ucsi_update(mca, MCA_UCSI_PD_SINK);
		xiaomi_mca_policy_poll(mca, &sample);
		KUNIT_EXPECT_EQ(test, mca->cp_total_errors, 0);
		KUNIT_EXPECT_EQ(test, mca->cp_chip_errors, 0);
		KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_OFF);
	}
}

static void xiaomi_mca_test_cycle4_intermediate_cv(struct kunit *test)
{
	struct xiaomi_mca *mca = kunit_kzalloc(test, sizeof(*mca), GFP_KERNEL);
	static const struct { int stage, mv, ma, cv; } rows[] = {
		{ 1, 4294, 9380, 4300 }, { 1, 4295, 7367, 4300 },
		{ 1, 4296, 5055, 4300 }, { 2, 4469, 6422, 4475 },
		{ 2, 4466, 4093, 4475 },
	};
	int n;

	KUNIT_ASSERT_NOT_NULL(test, mca);
	mca->cp_profile = MCA_PROFILE_FFC;
	mca->cp_ma = 6000;
	mca->cp_type = MCA_TYPE_PD_PPS;
	for (n = 0; n < ARRAY_SIZE(rows); n++) {
		mca->cp_stage = rows[n].stage;
		xiaomi_mca_cp_ibat_limit(mca, rows[n].mv, rows[n].ma);
		KUNIT_EXPECT_EQ(test, mca->cp_stage, rows[n].stage);
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_row_cv_request(mca, 9100, 9120,
			rows[n].mv, rows[n].cv), 9120);
		/* Once voltage actually reaches the row, native current condition applies. */
		xiaomi_mca_cp_ibat_limit(mca, rows[n].cv, rows[n].ma);
		KUNIT_EXPECT_EQ(test, mca->cp_stage,
			rows[n].stage + (rows[n].ma <= (rows[n].stage == 1 ? 9040 : 7230)));
	}
	mca->cp_stage = 3;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_row_cv_request(mca, 9600, 9620, 4538, 4540),
		xiaomi_mca_cp_cv_request(mca->cp_type, 9600, 9620, 4538, 4540));
	mca->cp_stage = 0;
	xiaomi_mca_cp_ibat_limit(mca, 3900, 11000);
	KUNIT_EXPECT_EQ(test, mca->cp_stage, 0);
}

struct mca_gauge_io_fixture {
	struct bq27xxx_device_info di;
	int count, fail;
	u8 reg[2], len[2], data[2][4];
};

static int mca_gauge_io_write(struct bq27xxx_device_info *di, u8 reg, u8 *data, int len)
{
	struct mca_gauge_io_fixture *f = container_of(di, struct mca_gauge_io_fixture, di);
	int n = f->count++;

	if (n >= 2)
		return -EINVAL;
	f->reg[n] = reg;
	f->len[n] = len;
	memcpy(f->data[n], data, len);
	return f->fail == n + 1 ? -EIO : 0;
}

static void xiaomi_mca_test_gauge_mac_packets(struct kunit *test)
{
	struct mca_gauge_io_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	static const u8 en[] = { 0x3e, 0, 1, 0 }, dis[] = { 0x3f, 0, 0, 0 };
	static const u8 tail[] = { 0xc0, 6 }, full[] = { 0x31, 0 };

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->di.bus.write_bulk = mca_gauge_io_write;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_gauge_mac_write(&f->di, 0x3e), 0);
	KUNIT_EXPECT_EQ(test, f->reg[0], 0x3e);
	KUNIT_EXPECT_MEMEQ(test, f->data[0], en, sizeof(en));
	KUNIT_EXPECT_EQ(test, f->reg[1], 0x60);
	KUNIT_EXPECT_MEMEQ(test, f->data[1], tail, sizeof(tail));
	f->count = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_gauge_mac_write(&f->di, 0x3f), 0);
	KUNIT_EXPECT_MEMEQ(test, f->data[0], dis, sizeof(dis));
	KUNIT_EXPECT_MEMEQ(test, f->data[1], tail, sizeof(tail));
	f->count = 0;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_gauge_mac_write(&f->di, 0x31), 0);
	KUNIT_EXPECT_EQ(test, f->count, 1);
	KUNIT_EXPECT_MEMEQ(test, f->data[0], full, sizeof(full));
	f->count = 0;
	f->fail = 1;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_gauge_mac_write(&f->di, 0x3e), -EIO);
	KUNIT_EXPECT_EQ(test, f->count, 1);
}

static void xiaomi_mca_test_cycle4_gauge_full(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int n;

	full_charge = charge_policy = true;
	mca->cp = MCA_CP_DONE;
	mca->ffc_active = true;
	f->chgr_status = 4;
	f->cells[0].capacity = f->cells[1].capacity = 94;
	f->cells[0].charge_now = f->cells[1].charge_now = 4324000;
	f->cells[0].voltage = f->cells[1].voltage = 4542000;
	f->cells[0].current_ua = 1291000;
	f->cells[1].current_ua = 1324000;
	f->fg_full_ready = false;
	guard(mutex)(&mca->lock);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_full_charge_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->fg_fast_mask, 3);
	for (n = 0; n < 6; n++) {
		mca->term_at = jiffies - MCA_POLL_INTERVAL;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	}
	KUNIT_EXPECT_EQ(test, mca->fg_full_mask, 3);
	KUNIT_EXPECT_FALSE(test, mca->charging_done); /* ACK is not gauge FC */
	/* A separate subsequent hardware sample reports FC and updated RM.
	 * The command's ACK alone deliberately did not change these values.
	 */
	f->fg_full_ready = true;
	f->cells[0].charge_now = f->cells[0].charge_full;
	f->cells[1].charge_now = f->cells[1].charge_full;
	f->cells[0].capacity = f->cells[1].capacity = 100;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->charging_done);
	KUNIT_EXPECT_EQ(test, f->charging_enabled, 0);
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_TRUE(test, mca->ffc_active);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 0);
	/* Only a later real decrease of RM/FCC permits recharge. */
	f->cells[0].charge_now = f->cells[1].charge_now = 4462000;
	f->cells[0].voltage = f->cells[1].voltage = 4450000;
	KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_limit_poll(mca), 0);
	KUNIT_EXPECT_EQ(test, mca->fg_fast_mask, 0);
	KUNIT_EXPECT_FALSE(test, mca->ffc_active);
	KUNIT_EXPECT_EQ(test, mca->fw_recharges, 1);
	f->fg_command_error[1] = -EIO;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_gauge_mode(mca, true), -EIO);
	KUNIT_EXPECT_EQ(test, mca->fg_fast_mask, 1);
	f->fg_command_error[1] = 0;
	n = f->fg_commands[0][1];
	KUNIT_EXPECT_EQ(test, xiaomi_mca_gauge_mode(mca, true), 0);
	KUNIT_EXPECT_EQ(test, mca->fg_fast_mask, 3);
	KUNIT_EXPECT_EQ(test, f->fg_commands[0][1], n); /* retry only failed peer */
}

static void xiaomi_mca_test_stock_ramp_steps(struct kunit *test)
{
	static const struct { int gap, mv; } rows[] = {
		{ 0, 20 }, { 599, 20 }, { 600, 40 }, { 999, 40 },
		{ 1000, 80 }, { 1999, 80 }, { 2000, 140 }, { 2999, 140 },
		{ 3000, 160 }, { 3999, 160 }, { 4000, 200 }, { 12000, 200 },
	};
	int n, sign;

	for (n = 0; n < ARRAY_SIZE(rows); n++) {
		for (sign = -1; sign <= 1; sign += 2) {
			int gap = sign * rows[n].gap;

			KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stock_vstep(gap, false), rows[n].mv);
			KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_stock_vstep(gap, true), min(rows[n].mv, 80));
			/* Bus excess overrides a still-positive (averaged) cell deficit. */
			KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(-1, gap, 100, false,
				false, 0, 0), -rows[n].mv);
		}
	}
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(1, 5000, 11, false, false, 0, 0), 200);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(1, 5000, 10, false, false, 0, 0), 20);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(1, 5000, 0, false, false, 0, 0), 0);
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(0, 5000, 100, false, false, 0, 0), 0);
}

static void xiaomi_mca_test_ramp_bus_overshoot(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;

	full_charge = charge_policy = cp_charge = hv_charge = true;
	mca_test_stage(test, f);
	f->type = mca->real_type = mca->cp_type = MCA_TYPE_PD_PPS;
	mca->cp_ma = 5000;
	mca->cp_mv = 8200;
	mca->cp_max_mv = 10000;
	mca->cp_ibus_ma = mca->cp_ibus_hot = 3800;
	f->cells[0].voltage = f->cells[1].voltage = 4000000;
	f->cells[0].current_ua = f->cells[1].current_ua = 1000000;
	f->stage_ma = 2000;
	f->stage_mv = 8200;
	guard(mutex)(&mca->lock);
	xiaomi_mca_cp_step(mca);
	KUNIT_ASSERT_EQ(test, mca->cp, MCA_CP_ON);
	KUNIT_EXPECT_EQ(test, mca->cp_mv, 8400);
	KUNIT_EXPECT_TRUE(test, mca->cp_ramp_poll);
	/* ADC sees an overshoot while gauges still report a 6 A deficit.
	 * Native control subtracts its full 200 mV, not the old 20 mV.
	 */
	f->stage_ma = 4000;
	xiaomi_mca_cp_step(mca);
	KUNIT_EXPECT_EQ(test, mca->cp_mv, 8200);
	KUNIT_EXPECT_TRUE(test, mca->cp_boost_done);
	KUNIT_EXPECT_FALSE(test, mca->cp_fault);
	KUNIT_EXPECT_EQ(test, mca->cp, MCA_CP_ON);
	xiaomi_mca_cp_stop(mca, "test complete");
}

static void xiaomi_mca_test_stock_boost_constraints(struct kunit *test)
{
	static const struct { int direction, gap, boost_ms, buck_ms, want; } rows[] = {
		{ 1, 400, 1999, 1000, 0 }, { 1, 400, 2000, 999, 0 },
		{ 1, 400, 2000, 1000, 20 }, { -1, -400, 5000, 999, 0 },
		{ -1, -400, 5000, 1000, -20 },
		/* Large corrections are not subject to the project-7 restriction. */
		{ 1, 4000, 700, 700, 200 }, { -1, 4000, 700, 700, -200 },
	};
	int n;

	for (n = 0; n < ARRAY_SIZE(rows); n++)
		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(rows[n].direction, rows[n].gap,
			100, false, true, rows[n].boost_ms, rows[n].buck_ms), rows[n].want);
	/* Before the first downward correction there is no boost delay. */
	KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(1, 400, 100, false, false, 0, 0), 20);
}

static void xiaomi_mca_test_cycle4_ramp(struct kunit *test)
{
	/* Real 05:05:41..05:07:47 samples, compressed to one row per ten
	 * seconds plus the first 9 A sample. Old ADCs are inputs, not a
	 * prediction of how cells respond to the new voltage requests.
	 */
	static const struct { int vbat, ibat, target, cv, step; } rows[] = {
		{ 3912, -1074, 8000, 4150, 200 }, /* 05:05:41 */
		{ 3966, 1291, 8000, 4150, 200 }, /* 05:05:51 */
		{ 4009, 2284, 8000, 4150, 200 }, /* 05:06:20 */
		{ 4065, 3636, 12000, 4144, 200 }, /* 05:06:30 */
		{ 4110, 5081, 12000, 4144, 200 }, /* 05:06:40 */
		{ 4137, 6418, 12000, 4144, 20 }, /* 05:06:50 */
		{ 4166, 7414, 10650, 4300, 160 }, /* 05:07:00 */
		{ 4198, 8560, 10650, 4300, 140 }, /* 05:07:10 */
		{ 4201, 9029, 10650, 4300, 80 }, /* 05:07:16 */
		{ 4207, 9268, 10650, 4300, 80 }, /* 05:07:20 */
		{ 4209, 9609, 10650, 4300, 80 }, /* 05:07:30 */
		{ 4212, 9775, 10650, 4300, 40 }, /* 05:07:40 */
		{ 4213, 9842, 10650, 4300, 40 }, /* 05:07:47 */
	};
	int n;

	for (n = 0; n < ARRAY_SIZE(rows); n++) {
		int gap = rows[n].target - rows[n].ibat;

		KUNIT_EXPECT_EQ(test, xiaomi_mca_cp_ramp_step(1, gap,
			rows[n].cv - rows[n].vbat, false, false, 0, 0), rows[n].step);
		/* Same gauge sample but an actual IBUS over target must lower
		 * voltage immediately, never continue the upward ramp.
		 */
		KUNIT_EXPECT_LT(test, xiaomi_mca_cp_ramp_step(-1, gap,
			rows[n].cv - rows[n].vbat, false, false, 0, 0), 0);
	}
}

static void xiaomi_mca_test_enable_compound_setter(struct kunit *test)
{
	struct mca_policy_fixture *f = mca_full_charge_fixture(test);
	struct xiaomi_mca *mca = &f->mca;
	int enable, n = 0;

	guard(mutex)(&mca->lock);
	for (enable = 0; enable <= 1; enable++) {
		f->charging_enabled = enable;
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_enable(mca, enable), 0);
		KUNIT_EXPECT_EQ(test, f->enable_writes, ++n);
		KUNIT_ASSERT_EQ(test, xiaomi_mca_charge_enable(mca, enable), 0);
		KUNIT_EXPECT_EQ(test, f->enable_writes, ++n);
	}
	f->fail_pmic_prop = MCA_PROP_CHARGING_ENABLED;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_enable(mca, true), -EIO);
	f->fail_pmic_prop = 0;
	mca->connector.tripped = true;
	KUNIT_EXPECT_EQ(test, xiaomi_mca_charge_enable(mca, true), -EBUSY);
}

static struct kunit_case xiaomi_mca_test_cases[] = {
	KUNIT_CASE(xiaomi_mca_test_enable_compound_setter),
	KUNIT_CASE(xiaomi_mca_test_stock_ramp_steps),
	KUNIT_CASE(xiaomi_mca_test_ramp_bus_overshoot),
	KUNIT_CASE(xiaomi_mca_test_stock_boost_constraints),
	KUNIT_CASE(xiaomi_mca_test_cycle4_ramp),
	KUNIT_CASE(xiaomi_mca_test_warm_termination_is_not_full),
	KUNIT_CASE(xiaomi_mca_test_provider_late_start),
	KUNIT_CASE(xiaomi_mca_test_gauge_mac_packets),
	KUNIT_CASE(xiaomi_mca_test_cycle4_gauge_full),
	KUNIT_CASE(xiaomi_mca_test_cycle4_intermediate_cv),
	KUNIT_CASE(xiaomi_mca_test_stock_failure_budget),
	KUNIT_CASE(xiaomi_mca_test_exit_retry_class),
	KUNIT_CASE(xiaomi_mca_test_real_full_v5),
	KUNIT_CASE(xiaomi_mca_test_stock_entry_matrix),
	KUNIT_CASE(xiaomi_mca_test_stock_cc_cv_stages),
#ifdef MCA_CHARGE_EXPERIMENT
	KUNIT_CASE(xiaomi_mca_test_experiment),
#endif



	KUNIT_CASE(xiaomi_mca_test_firmware_error_identity),
	KUNIT_CASE(xiaomi_mca_test_cycle_latch_and_sample),
	KUNIT_CASE(xiaomi_mca_test_stock_handoff_order),
	KUNIT_CASE(xiaomi_mca_test_822s_read_failure),
	KUNIT_CASE(xiaomi_mca_test_full_recharge_matrix),
	KUNIT_CASE(xiaomi_mca_test_software_full_samples),
	KUNIT_CASE(xiaomi_mca_test_pen_boost),
	KUNIT_CASE(xiaomi_mca_test_pen_failures),
	KUNIT_CASE(xiaomi_mca_test_pen_allowlist),

	KUNIT_CASE(xiaomi_mca_test_profile_switch),
	KUNIT_CASE(xiaomi_mca_test_jeita_hysteresis),
	KUNIT_CASE(xiaomi_mca_test_source_limits),
	KUNIT_CASE(xiaomi_mca_test_pd_recovery),
	KUNIT_CASE(xiaomi_mca_test_mipps_exchange),
	KUNIT_CASE(xiaomi_mca_test_mipps_bad_digest),
	KUNIT_CASE(xiaomi_mca_test_mipps_lost_ack),
	KUNIT_CASE(xiaomi_mca_test_mipps_restart),
	KUNIT_CASE(xiaomi_mca_test_auth_retry_contract),
	KUNIT_CASE(xiaomi_mca_test_mipps_detach),
	KUNIT_CASE(xiaomi_mca_test_identity_edges),
	KUNIT_CASE(xiaomi_mca_test_ffc_conditions),
	KUNIT_CASE(xiaomi_mca_test_ffc_handoff),
	KUNIT_CASE(xiaomi_mca_test_cp_cell_guards),
	KUNIT_CASE(xiaomi_mca_test_auth_loss_latch),
	KUNIT_CASE(xiaomi_mca_test_gauge_ocp_identity),
	KUNIT_CASE(xiaomi_mca_test_pack_limit_recovers),
	KUNIT_CASE(xiaomi_mca_test_bus_voltage_fault),
	KUNIT_CASE(xiaomi_mca_test_qc_regulation),
	KUNIT_CASE(xiaomi_mca_test_measured_buck_share),
	KUNIT_CASE(xiaomi_mca_test_taper_handoff),
	KUNIT_CASE(xiaomi_mca_test_pmic_compensation),
	KUNIT_CASE(xiaomi_mca_test_pmic_headroom),
	KUNIT_CASE(xiaomi_mca_test_pmic_temperature_votes),
	KUNIT_CASE(xiaomi_mca_test_pmic_handover),
	KUNIT_CASE(xiaomi_mca_test_pmic_suspend_failure),
	KUNIT_CASE(xiaomi_mca_test_pmic_suspend_threshold),
	KUNIT_CASE(xiaomi_mca_test_pmic_cell_ceiling),
	KUNIT_CASE(xiaomi_mca_test_pmic_disabled_transition),
	KUNIT_CASE(xiaomi_mca_test_pmic_disabled_snapshot),
	KUNIT_CASE(xiaomi_mca_test_fw_recharge),
	KUNIT_CASE(xiaomi_mca_test_legacy_hiz_recovery),
	KUNIT_CASE(xiaomi_mca_test_pause_voter_start),
	KUNIT_CASE(xiaomi_mca_test_zero_vote_ack),
	KUNIT_CASE(xiaomi_mca_test_paused_detach),
	KUNIT_CASE(xiaomi_mca_test_normal_pump_termination_replay),
	KUNIT_CASE(xiaomi_mca_test_taper_direction),
	KUNIT_CASE(xiaomi_mca_test_fw_recharge_failure),
	KUNIT_CASE(xiaomi_mca_test_mipps_tail_replay),
	KUNIT_CASE(xiaomi_mca_test_buck_enable_readback),
	KUNIT_CASE(xiaomi_mca_test_ffc_cutoff_in_pump),
	KUNIT_CASE(xiaomi_mca_test_tail_current_transfer),
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
	KUNIT_CASE(xiaomi_mca_test_full_charge_temperature),
	KUNIT_CASE(xiaomi_mca_test_cold_float_hysteresis),
	KUNIT_CASE(xiaomi_mca_test_full_charge_readback),
	KUNIT_CASE(xiaomi_mca_test_full_charge_restore),
	KUNIT_CASE(xiaomi_mca_test_full_charge_sleep),
	KUNIT_CASE(xiaomi_mca_test_full_charge_allowlist),
	KUNIT_CASE(xiaomi_mca_test_hv_fixed),
	KUNIT_CASE(xiaomi_mca_test_hv_pps),
	KUNIT_CASE(xiaomi_mca_test_qc_voltage),
	KUNIT_CASE(xiaomi_mca_test_qc_pulses),
	KUNIT_CASE(xiaomi_mca_test_aicl),
	KUNIT_CASE(xiaomi_mca_test_aicl_collapse),
	KUNIT_CASE(xiaomi_mca_test_aicl_evidence),
	KUNIT_CASE(xiaomi_mca_test_unknown_debounce),
	KUNIT_CASE(xiaomi_mca_test_policy_refusal),
	KUNIT_CASE(xiaomi_mca_test_auth_read_only),
	KUNIT_CASE(xiaomi_mca_test_temperature_follow),
	KUNIT_CASE(xiaomi_mca_test_cp_limits),
	KUNIT_CASE(xiaomi_mca_test_bus_margin),
	KUNIT_CASE(xiaomi_mca_test_recover_learned_limit),
	KUNIT_CASE(xiaomi_mca_test_fcc_ack_without_readback),
	KUNIT_CASE(xiaomi_mca_test_fcc_request_failure),
	KUNIT_CASE(xiaomi_mca_test_pm_gate),
	KUNIT_CASE(xiaomi_mca_test_pm_quiesce),
	KUNIT_CASE(xiaomi_mca_test_pm_source_currents),
	KUNIT_CASE(xiaomi_mca_test_sleep_fixed_pd),
	KUNIT_CASE(xiaomi_mca_test_cp_stop_failure),
	KUNIT_CASE(xiaomi_mca_test_retry_schedule),
	KUNIT_CASE(xiaomi_mca_test_auth_stop_failure),
	KUNIT_CASE(xiaomi_mca_test_vbus_error_not_detach),
	KUNIT_CASE(xiaomi_mca_test_threshold_interface),
	KUNIT_CASE(xiaomi_mca_test_connector),
	KUNIT_CASE(xiaomi_mca_test_connector_stop),
	KUNIT_CASE(xiaomi_mca_test_wake_policy),
	KUNIT_CASE(xiaomi_mca_test_wired_wake_transitions),
	KUNIT_CASE(xiaomi_mca_test_wired_wake_reconnect),
	KUNIT_CASE(xiaomi_mca_test_threshold_hysteresis),
	KUNIT_CASE(xiaomi_mca_test_threshold_thermal_release),
	KUNIT_CASE(xiaomi_mca_test_gauge_fallback),
	KUNIT_CASE(xiaomi_mca_test_threshold_stop_failure_sleep),
	KUNIT_CASE(xiaomi_mca_test_stage_removed),
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

static bool * const mca_test_switches[] = {
	&charge_policy, &charge_current, &full_charge, &hv_charge, &mipps_auth, &cp_charge,
};

static void mca_test_restore_switches(void *data)
{
	bool *saved = data;
	int i;

	for (i = 0; i < ARRAY_SIZE(mca_test_switches); i++)
		*mca_test_switches[i] = saved[i];
}

static int mca_test_init(struct kunit *test)
{
	bool *saved = kunit_kcalloc(test, ARRAY_SIZE(mca_test_switches), sizeof(bool), GFP_KERNEL);
	int i;

	if (!saved)
		return -ENOMEM;
	for (i = 0; i < ARRAY_SIZE(mca_test_switches); i++) {
		saved[i] = *mca_test_switches[i];
		*mca_test_switches[i] = false;
	}
	return kunit_add_action_or_reset(test, mca_test_restore_switches, saved);
}

static struct kunit_suite xiaomi_mca_test_suite = {
	.init = mca_test_init,
	.name = "xiaomi-mca",
	.test_cases = xiaomi_mca_test_cases,
};
kunit_test_suite(xiaomi_mca_test_suite);
