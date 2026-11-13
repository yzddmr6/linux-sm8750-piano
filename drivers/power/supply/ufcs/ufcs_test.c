// SPDX-License-Identifier: GPL-2.0-only
#include <crypto/sha2.h>
#include <kunit/test.h>
#include <kunit/device.h>
#include <linux/unaligned.h>
#include "ufcs_class.h"
#include "ufcs_core.h"

struct peer {
	struct kunit *test;
	struct ufcs_dev *dev;
	u8 frame[64];
	int size;
	u8 next_ctrl;
	u8 nonce[16];
	bool silent, corrupt, bad_length;
	unsigned int attempts, disabled;
	u64 request;
};

static void signal_phy(struct peer *p, unsigned int flags)
{
	ufcs_set_error_flag(p->dev, flags);
	ufcs_msg_handler(p->dev);
}

static int peer_handshake(struct ufcs_dev *dev)
{
	struct peer *p = dev->drv_data;

	p->attempts++;
	if (!p->silent) {
		dev->handshake_state = UFCS_HS_SUCCESS;
		signal_phy(p, 0);
	}
	return 0;
}

static void control(struct peer *p, u8 cmd)
{
	put_unaligned_be16(0x4008, p->frame);
	p->frame[2] = cmd;
	p->size = 3;
}

static int peer_write(struct ufcs_dev *dev, u8 *buf, int n)
{
	struct peer *p = dev->drv_data;
	unsigned int flags = BIT(UFCS_RECV_ERR_SENT_CMP);
	u8 cmd = buf[2];
	u8 input[48] = {}, digest[32];
	int i;

	KUNIT_EXPECT_EQ(p->test, buf[0] >> 5, 1);
	if ((buf[1] & 7) == UFCS_CTRL_MSG) {
		KUNIT_EXPECT_EQ(p->test, n, 3);
		if (cmd == CTRL_MSG_GET_OUTPUT_CAPABILITIES) {
			put_unaligned_be16(0x4009, p->frame);
			p->frame[2] = DATA_MSG_OUTPUT_CAPABILITIES;
			p->frame[3] = p->bad_length ? 9 : 8;
			put_unaligned_be64(0x1004b001f401f432ULL, p->frame + 4);
			p->size = 12;
		}
	} else if (cmd == DATA_MSG_REQUEST && n == 12) {
		p->request = get_unaligned_be64(buf + 4);
		control(p, CTRL_MSG_ACCEPT);
		p->next_ctrl = CTRL_MSG_POWER_READY;
	} else if (cmd == DATA_MSG_VERIFY_REQUEST && n == 21) {
		KUNIT_EXPECT_EQ(p->test, buf[4], 1);
		for (i = 0; i < sizeof(p->nonce); i++)
			p->nonce[i] = buf[20 - i];
		control(p, CTRL_MSG_ACCEPT);
		p->next_ctrl = CTRL_MSG_GET_DEVICE_INFO;
	} else if (cmd == DATA_MSG_DEVICE_INFO && n == 12) {
		/* Source nonce = 0, test credential = 0, sink nonce from wire. */
		memcpy(input + 32, p->nonce, 16);
		sha256(input, sizeof(input), digest);
		put_unaligned_be16(0x4009, p->frame);
		p->frame[2] = DATA_MSG_VERIFY_RESPONSE;
		p->frame[3] = 48;
		for (i = 0; i < 32; i++)
			p->frame[4 + i] = digest[31 - i];
		p->frame[4] ^= p->corrupt;
		memset(p->frame + 36, 0, 16);
		p->size = 52;
	}
	if (p->size)
		flags |= BIT(UFCS_RECV_ERR_DATA_READY);
	signal_phy(p, flags);
	return 0;
}

static int peer_read(struct ufcs_dev *dev, u8 *buf, int len)
{
	struct peer *p = dev->drv_data;
	int size = p->size;

	if (size > len)
		return -EMSGSIZE;
	memcpy(buf, p->frame, size);
	p->size = 0;
	if (p->next_ctrl) {
		control(p, p->next_ctrl);
		p->next_ctrl = 0;
		ufcs_set_error_flag(dev, BIT(UFCS_RECV_ERR_DATA_READY));
	}
	return size;
}

static int noop(struct ufcs_dev *dev)
{
	return 0;
}

static int disable(struct ufcs_dev *dev)
{
	struct peer *p = dev->drv_data;

	p->disabled++;
	return 0;
}

static int baud(struct ufcs_dev *dev, enum ufcs_baud_rate rate)
{
	return rate == UFCS_BAUD_115200 ? 0 : -EINVAL;
}

static struct ufcs_dev_ops ops = {
	.init = noop, .enable = noop, .disable = disable,
	.handshake = peer_handshake, .source_hard_reset = noop,
	.set_baud_rate = baud, .read_msg = peer_read, .write_msg = peer_write,
};

static int setup(struct kunit *test)
{
	struct ufcs_config config = { .handshake_hard_retry = true };
	struct device *parent = kunit_device_register(test, "ufcs-peer");
	struct peer *p = kunit_kzalloc(test, sizeof(*p), GFP_KERNEL);

	if (IS_ERR(parent))
		return PTR_ERR(parent);
	if (!p)
		return -ENOMEM;
	p->test = test;
	test->priv = p;
	p->dev = ufcs_device_register(parent, &ops, p, &config);
	return PTR_ERR_OR_ZERO(p->dev);
}

static void teardown(struct kunit *test)
{
	struct peer *p = test->priv;

	ufcs_device_unregister(p->dev);
}

static void negotiate(struct kunit *test)
{
	struct peer *p = test->priv;
	u64 mode;
	u8 key[16] = {};
	int applied;

	KUNIT_ASSERT_EQ(test, ufcs_handshake(p->dev), 0);
	KUNIT_ASSERT_EQ(test, ufcs_intf_get_pdo_info(p->dev, &mode, 1), 1);
	KUNIT_EXPECT_EQ(test, UFCS_OUTPUT_MODE_CURR_MAX(mode), 5000);
	KUNIT_EXPECT_LT(test, ufcs_intf_pdo_set(p->dev, 10000, 5010), 0);
	KUNIT_ASSERT_EQ(test, ufcs_intf_pdo_set(p->dev, 10000, 5000), 0);
	KUNIT_EXPECT_EQ(test, p->request, 0x1000000003e801f4ULL);
	KUNIT_ASSERT_EQ(test, ufcs_request_limited(p->dev, 10000, 4900, 5000, 4617, &applied), 0);
	KUNIT_EXPECT_EQ(test, applied, 4610);
	KUNIT_EXPECT_EQ(test, p->request, 0x1000000003e801cdULL);
	KUNIT_EXPECT_EQ(test, ufcs_intf_verify_adapter(p->dev, 1, key, 16), 1);
	p->corrupt = true;
	KUNIT_EXPECT_EQ(test, ufcs_intf_verify_adapter(p->dev, 1, key, 16), 0);
}

static void timeout_and_reenter(struct kunit *test)
{
	struct peer *p = test->priv;

	p->silent = true;
	KUNIT_ASSERT_LT(test, ufcs_handshake(p->dev), 0);
	KUNIT_EXPECT_EQ(test, p->attempts, 1);
	KUNIT_EXPECT_GT(test, p->disabled, 0);
	p->silent = false;
	KUNIT_EXPECT_EQ(test, ufcs_handshake(p->dev), 0);
}

static void leave_peer_test_mode(struct kunit *test)
{
	struct peer *p = test->priv;
	u64 mode;

	KUNIT_ASSERT_EQ(test, ufcs_handshake(p->dev), 0);
	ufcs_pe_test_request_handle(p->dev->class, BIT(15) | BIT(14));
	KUNIT_ASSERT_TRUE(test, ufcs_is_test_mode(p->dev));
	KUNIT_ASSERT_TRUE(test, ufcs_is_vol_acc_test_mode(p->dev));
	KUNIT_ASSERT_EQ(test, ufcs_force_exit(p->dev), 0);
	flush_work(&p->dev->class->test_handle_work);
	KUNIT_ASSERT_FALSE(test, ufcs_is_test_mode(p->dev));
	KUNIT_ASSERT_FALSE(test, ufcs_is_vol_acc_test_mode(p->dev));
	KUNIT_ASSERT_EQ(test, ufcs_handshake(p->dev), 0);
	KUNIT_ASSERT_EQ(test, ufcs_intf_get_pdo_info(p->dev, &mode, 1), 1);
	KUNIT_EXPECT_EQ(test, ufcs_intf_pdo_set(p->dev, 10000, 5000), 0);
}

static void malformed_capabilities(struct kunit *test)
{
	struct peer *p = test->priv;
	u64 mode = 0;

	KUNIT_ASSERT_EQ(test, ufcs_handshake(p->dev), 0);
	p->bad_length = true;
	KUNIT_EXPECT_LT(test, ufcs_intf_get_pdo_info(p->dev, &mode, 1), 0);
	KUNIT_EXPECT_EQ(test, mode, 0);
}

static struct kunit_case cases[] = {
	KUNIT_CASE(negotiate),
	KUNIT_CASE(timeout_and_reenter),
	KUNIT_CASE(leave_peer_test_mode),
	KUNIT_CASE(malformed_capabilities),
	{}
};

static struct kunit_suite suite = {
	.name = "ufcs-sink",
	.init = setup,
	.exit = teardown,
	.test_cases = cases,
};
kunit_test_suite(suite);
