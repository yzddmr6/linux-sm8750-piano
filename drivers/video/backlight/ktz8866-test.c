// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

struct ktz8866_test_context {
	struct ktz8866 ktz;
	struct i2c_client client;
	struct backlight_device backlight;
	unsigned int registers[REG_MAX + 1];
	unsigned int writes;
	int fail_read;
	int fail_write;
};

static int ktz8866_test_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ktz8866_test_context *ctx = context;

	if (reg == ctx->fail_read)
		return -EIO;
	*val = ctx->registers[reg];
	return 0;
}

static int ktz8866_test_write(void *context, unsigned int reg, unsigned int val)
{
	struct ktz8866_test_context *ctx = context;

	if (reg == ctx->fail_write)
		return -EIO;
	ctx->registers[reg] = val;
	ctx->writes++;
	return 0;
}

static const struct regmap_bus ktz8866_test_bus = {
	.reg_read = ktz8866_test_read,
	.reg_write = ktz8866_test_write,
};

static int ktz8866_test_init(struct kunit *test)
{
	struct ktz8866_test_context *ctx;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->fail_read = -1;
	ctx->fail_write = -1;
	ctx->ktz.client = &ctx->client;
	ctx->ktz.regmap = regmap_init(NULL, &ktz8866_test_bus, ctx, &ktz8866_regmap_config);
	if (IS_ERR(ctx->ktz.regmap))
		return PTR_ERR(ctx->ktz.regmap);
	dev_set_drvdata(&ctx->backlight.dev, &ctx->ktz);
	ctx->backlight.props.brightness = DEFAULT_BRIGHTNESS;
	test->priv = ctx;
	return 0;
}

static void ktz8866_test_exit(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	regmap_exit(ctx->ktz.regmap);
}

static void ktz8866_test_brightness(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, ktz8866_init(&ctx->ktz), 0);
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_EN], 0x7fU);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_BRT_LSB], DEFAULT_BRIGHTNESS & 7U);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_BRT_MSB], DEFAULT_BRIGHTNESS >> 3);

	ctx->backlight.props.power = BACKLIGHT_POWER_OFF;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_EN], 0x3fU);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_BRT_LSB], 0U);
	KUNIT_EXPECT_EQ(test, ctx->registers[BL_BRT_MSB], 0U);
}

struct ktz8866_error_case {
	const char *name;
	int fail_read;
	int fail_write;
	bool initial_on;
	bool disable;
	bool expected_on;
	unsigned int writes;
};

static const struct ktz8866_error_case ktz8866_errors[] = {
	{ "enable-read", BL_EN, -1, false, false, false, 2 },
	{ "enable-write", -1, BL_EN, false, false, false, 2 },
	{ "brightness-lsb", -1, BL_BRT_LSB, false, false, false, 0 },
	{ "brightness-msb", -1, BL_BRT_MSB, false, false, false, 1 },
	{ "running-lsb", -1, BL_BRT_LSB, true, false, true, 0 },
	{ "running-msb", -1, BL_BRT_MSB, true, false, true, 1 },
	{ "disable-read", BL_EN, -1, true, true, true, 0 },
	{ "disable-write", -1, BL_EN, true, true, true, 0 },
};

KUNIT_ARRAY_PARAM_DESC(ktz8866_error, ktz8866_errors, name);

static void ktz8866_test_error(struct kunit *test)
{
	const struct ktz8866_error_case *param = test->param_value;
	struct ktz8866_test_context *ctx = test->priv;

	ctx->fail_read = param->fail_read;
	ctx->fail_write = param->fail_write;
	ctx->ktz.led_on = param->initial_on;
	ctx->registers[BL_EN] = param->initial_on ? 0x7f : 0x3f;
	if (param->disable)
		ctx->backlight.props.brightness = 0;

	KUNIT_EXPECT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->ktz.led_on, param->expected_on);
	KUNIT_EXPECT_EQ(test, ctx->writes, param->writes);

	/* A later caller request can succeed; the driver does not retry silently. */
	ctx->fail_read = -1;
	ctx->fail_write = -1;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_EQ(test, ctx->ktz.led_on, !param->disable);
}

static void ktz8866_test_init_error(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	ctx->fail_write = BL_EN;
	KUNIT_EXPECT_EQ(test, ktz8866_init(&ctx->ktz), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->writes, 0U);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);
}

static struct kunit_case ktz8866_test_cases[] = {
	KUNIT_CASE(ktz8866_test_brightness),
	KUNIT_CASE_PARAM(ktz8866_test_error, ktz8866_error_gen_params),
	KUNIT_CASE(ktz8866_test_init_error),
	{}
};

static struct kunit_suite ktz8866_test_suite = {
	.name = "ktz8866",
	.init = ktz8866_test_init,
	.exit = ktz8866_test_exit,
	.test_cases = ktz8866_test_cases,
};

kunit_test_suite(ktz8866_test_suite);
