// SPDX-License-Identifier: GPL-2.0-only
/* Exercise the real I2C/regmap driver and Type-C APIs on a synthetic bus. */
#include <kunit/test.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>

struct fsa_test_ctx {
	struct i2c_adapter adapter;
	struct i2c_client *client;
	struct typec_switch *sw;
	struct typec_mux *mux;
	u8 regs[256];
	int fail_write;
	unsigned int max_written;
};

static const struct property_entry switch_props[] = {
	PROPERTY_ENTRY_BOOL("orientation-switch"),
	PROPERTY_ENTRY_BOOL("mode-switch"),
	{ }
};

static const struct software_node switch_node = {
	.name = "fsa4480-test-switch",
	.properties = switch_props,
};

static const struct property_entry consumer_props[] = {
	PROPERTY_ENTRY_REF("orientation-switch", &switch_node),
	PROPERTY_ENTRY_REF("mode-switch", &switch_node),
	{ }
};

static const struct software_node consumer_node = {
	.name = "fsa4480-test-consumer",
	.properties = consumer_props,
};

static const struct software_node * const nodes[] = {
	&switch_node, &consumer_node, NULL,
};

static u32 fsa_test_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_SMBUS_BYTE_DATA;
}

static int fsa_test_xfer(struct i2c_adapter *adapter, u16 addr,
			unsigned short flags, char read_write, u8 command,
			int size, union i2c_smbus_data *data)
{
	struct fsa_test_ctx *ctx = i2c_get_adapdata(adapter);

	if (addr != 0x42 || size != I2C_SMBUS_BYTE_DATA)
		return -EOPNOTSUPP;
	if (read_write == I2C_SMBUS_READ) {
		data->byte = ctx->regs[command];
		return 0;
	}
	if (command == ctx->fail_write)
		return -EIO;
	ctx->regs[command] = data->byte;
	ctx->max_written = max(ctx->max_written, command);
	return 0;
}

static const struct i2c_algorithm fsa_test_algo = {
	.functionality = fsa_test_functionality,
	.smbus_xfer = fsa_test_xfer,
};

static void fsa_test_remove_nodes(void *unused)
{
	software_node_unregister_node_group(nodes);
}

static void fsa_test_remove_adapter(void *adapter)
{
	i2c_del_adapter(adapter);
}

static void fsa_test_remove_client(void *client)
{
	i2c_unregister_device(client);
}

static void fsa_test_put_handles(void *data)
{
	struct fsa_test_ctx *ctx = data;

	typec_mux_put(ctx->mux);
	typec_switch_put(ctx->sw);
}

static struct fsa_test_ctx *fsa_test_client_id(struct kunit *test,
					    const char *type, int fail_write, u8 id)
{
	struct i2c_board_info info = { I2C_BOARD_INFO("", 0x42) };
	struct fsa_test_ctx *ctx;
	int ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->fail_write = fail_write;
	ctx->regs[0] = id;
	ret = software_node_register_node_group(nodes);
	KUNIT_ASSERT_EQ(test, ret, 0);
	kunit_add_action_or_reset(test, fsa_test_remove_nodes, NULL);
	ctx->adapter.owner = THIS_MODULE;
	ctx->adapter.algo = &fsa_test_algo;
	strscpy(ctx->adapter.name, "fsa4480-kunit");
	i2c_set_adapdata(&ctx->adapter, ctx);
	ret = i2c_add_adapter(&ctx->adapter);
	KUNIT_ASSERT_EQ(test, ret, 0);
	kunit_add_action_or_reset(test, fsa_test_remove_adapter, &ctx->adapter);
	strscpy(info.type, type);
	info.swnode = &switch_node;
	ctx->client = i2c_new_client_device(&ctx->adapter, &info);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->client);
	kunit_add_action_or_reset(test, fsa_test_remove_client, ctx->client);
	return ctx;
}

static struct fsa_test_ctx *fsa_test_client(struct kunit *test,
					 const char *type, int fail_write)
{
	return fsa_test_client_id(test, type, fail_write,
				 !strcmp(type, "dio4485") ? 0xf6 : 0x09);
}

static void fsa_test_handles(struct kunit *test, struct fsa_test_ctx *ctx)
{
	KUNIT_ASSERT_NOT_NULL(test, ctx->client->dev.driver);
	ctx->sw = fwnode_typec_switch_get(software_node_fwnode(&consumer_node));
	kunit_add_action_or_reset(test, fsa_test_put_handles, ctx);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->sw);
	ctx->mux = fwnode_typec_mux_get(software_node_fwnode(&consumer_node));
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, ctx->mux);
}

static int fsa_test_mode(struct fsa_test_ctx *ctx, unsigned long mode)
{
	struct typec_altmode alt = { .svid = USB_TYPEC_DP_SID };
	struct typec_mux_state state = { .mode = mode };

	if (mode == TYPEC_DP_STATE_C || mode == TYPEC_DP_STATE_D)
		state.alt = &alt;
	return typec_mux_set(ctx->mux, &state);
}

static void fsa_test_legacy_probe(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "fsa4480", -1);

	fsa_test_handles(test, ctx);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0x98);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x18);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x10], 0x09);
}

static void fsa_test_cycle(struct kunit *test, const char *type)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, type, -1);

	fsa_test_handles(test, ctx);
	KUNIT_ASSERT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_NORMAL), 0);
	KUNIT_ASSERT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_D), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0xf8);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x18);
	KUNIT_ASSERT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_REVERSE), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x78);
	KUNIT_ASSERT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_C), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0xe0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x60);
	KUNIT_ASSERT_EQ(test, fsa_test_mode(ctx, TYPEC_STATE_SAFE), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0x98);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x18);
}

static void fsa_test_legacy_cycle(struct kunit *test)
{
	fsa_test_cycle(test, "fsa4480");
}

static void fsa_test_orientation_error(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "fsa4480", -1);

	fsa_test_handles(test, ctx);
	KUNIT_ASSERT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_NORMAL), 0);
	KUNIT_ASSERT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_D), 0);
	ctx->fail_write = 0x05;
	KUNIT_EXPECT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_REVERSE), -EIO);
	ctx->fail_write = -1;
	/* A subsequent caller request must program hardware, not use a failed cache. */
	KUNIT_EXPECT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_REVERSE), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x78);
}

static void fsa_test_mode_error(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "fsa4480", -1);

	fsa_test_handles(test, ctx);
	KUNIT_ASSERT_EQ(test, typec_switch_set(ctx->sw, TYPEC_ORIENTATION_REVERSE), 0);
	ctx->fail_write = 0x05;
	KUNIT_EXPECT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_C), -EIO);
	ctx->fail_write = -1;
	KUNIT_EXPECT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_C), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x60);
}

static void fsa_test_probe_error(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "fsa4480", 0x08);

	KUNIT_EXPECT_PTR_EQ(test, ctx->client->dev.driver, NULL);
}

static void fsa_test_dio_probe(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "dio4485", -1);
	struct typec_mux_state audio = { .mode = TYPEC_MODE_AUDIO };

	fsa_test_handles(test, ctx);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x21], 0x0f);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x12], 0x08);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0x98);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x05], 0x18);
	KUNIT_EXPECT_LE(test, ctx->max_written, 0x21);
	/* Audio's numeric mode overlaps DP D; it must carry no alternate-mode SVID. */
	KUNIT_ASSERT_EQ(test, typec_mux_set(ctx->mux, &audio), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x12], 0x09);
}

static void fsa_test_svid_transition(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "fsa4480", -1);
	struct typec_mux_state audio = { .mode = TYPEC_MODE_AUDIO };

	fsa_test_handles(test, ctx);
	KUNIT_ASSERT_EQ(test, fsa_test_mode(ctx, TYPEC_DP_STATE_D), 0);
	KUNIT_ASSERT_EQ(test, typec_mux_set(ctx->mux, &audio), 0);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x04], 0x99);
	KUNIT_EXPECT_EQ(test, ctx->regs[0x12], 0x01);
}

static void fsa_test_dio_wrong_id(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client_id(test, "dio4485", -1, 0x09);

	KUNIT_EXPECT_PTR_EQ(test, ctx->client->dev.driver, NULL);
	KUNIT_EXPECT_EQ(test, ctx->max_written, 0);
}

static void fsa_test_dio_init_error(struct kunit *test)
{
	struct fsa_test_ctx *ctx = fsa_test_client(test, "dio4485", 0x21);

	KUNIT_EXPECT_PTR_EQ(test, ctx->client->dev.driver, NULL);
}

static void fsa_test_dio_cycle(struct kunit *test)
{
	fsa_test_cycle(test, "dio4485");
}

static struct kunit_case fsa_test_cases[] = {
	KUNIT_CASE(fsa_test_legacy_probe),
	KUNIT_CASE(fsa_test_legacy_cycle),
	KUNIT_CASE(fsa_test_orientation_error),
	KUNIT_CASE(fsa_test_mode_error),
	KUNIT_CASE(fsa_test_probe_error),
	KUNIT_CASE(fsa_test_dio_probe),
	KUNIT_CASE(fsa_test_dio_cycle),
	KUNIT_CASE(fsa_test_svid_transition),
	KUNIT_CASE(fsa_test_dio_wrong_id),
	KUNIT_CASE(fsa_test_dio_init_error),
	{ }
};

static struct kunit_suite fsa_test_suite = {
	.name = "fsa4480",
	.test_cases = fsa_test_cases,
};
kunit_test_suite(fsa_test_suite);

MODULE_LICENSE("GPL");
