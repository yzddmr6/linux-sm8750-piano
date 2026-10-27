// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

/* Register file of one simulated chip */
struct ktz8866_test_chip {
	unsigned int registers[REG_MAX + 1];
	unsigned int writes;
	int fail_read;
	int fail_write;
};

struct ktz8866_test_context {
	struct ktz8866 ktz;
	struct ktz8866 secondary;
	struct i2c_client client;
	struct backlight_device backlight;
	struct ktz8866_test_chip chip;
	struct ktz8866_test_chip chip2;
};

static int ktz8866_test_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ktz8866_test_chip *ctx = context;

	if (reg == ctx->fail_read)
		return -EIO;
	*val = ctx->registers[reg];
	return 0;
}

static int ktz8866_test_write(void *context, unsigned int reg, unsigned int val)
{
	struct ktz8866_test_chip *ctx = context;

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
	ctx->chip.fail_read = -1;
	ctx->chip.fail_write = -1;
	ctx->chip2.fail_read = -1;
	ctx->chip2.fail_write = -1;
	ctx->ktz.client = &ctx->client;
	ctx->ktz.regmap = regmap_init(NULL, &ktz8866_test_bus, &ctx->chip,
				      &ktz8866_regmap_config);
	if (IS_ERR(ctx->ktz.regmap))
		return PTR_ERR(ctx->ktz.regmap);
	/* Only the grouping tests set ctx->ktz.secondary. */
	ctx->secondary.regmap = regmap_init(NULL, &ktz8866_test_bus, &ctx->chip2,
					    &ktz8866_regmap_config);
	if (IS_ERR(ctx->secondary.regmap)) {
		regmap_exit(ctx->ktz.regmap);
		return PTR_ERR(ctx->secondary.regmap);
	}
	dev_set_drvdata(&ctx->backlight.dev, &ctx->ktz);
	ctx->backlight.props.brightness = DEFAULT_BRIGHTNESS;
	test->priv = ctx;
	return 0;
}

static void ktz8866_test_exit(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	regmap_exit(ctx->secondary.regmap);
	regmap_exit(ctx->ktz.regmap);
}

static void ktz8866_test_brightness(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, ktz8866_init(&ctx->ktz), 0);
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_EN], 0x7fU);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_BRT_LSB], DEFAULT_BRIGHTNESS & 7U);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_BRT_MSB], DEFAULT_BRIGHTNESS >> 3);

	ctx->backlight.props.power = BACKLIGHT_POWER_OFF;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_EN], 0x3fU);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_BRT_LSB], 0U);
	KUNIT_EXPECT_EQ(test, ctx->chip.registers[BL_BRT_MSB], 0U);
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

	ctx->chip.fail_read = param->fail_read;
	ctx->chip.fail_write = param->fail_write;
	ctx->ktz.led_on = param->initial_on;
	ctx->chip.registers[BL_EN] = param->initial_on ? 0x7f : 0x3f;
	if (param->disable)
		ctx->backlight.props.brightness = 0;

	KUNIT_EXPECT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->ktz.led_on, param->expected_on);
	KUNIT_EXPECT_EQ(test, ctx->chip.writes, param->writes);

	/* A later caller request can succeed; the driver does not retry silently. */
	ctx->chip.fail_read = -1;
	ctx->chip.fail_write = -1;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_EQ(test, ctx->ktz.led_on, !param->disable);
}

static void ktz8866_test_init_error(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	ctx->chip.fail_write = BL_EN;
	KUNIT_EXPECT_EQ(test, ktz8866_init(&ctx->ktz), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->chip.writes, 0U);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);
}

static void ktz8866_test_set_chip(struct ktz8866_test_chip *chip, unsigned int en,
				  unsigned int brightness)
{
	chip->registers[BL_EN] = en;
	chip->registers[BL_BRT_LSB] = brightness & 7;
	chip->registers[BL_BRT_MSB] = brightness >> 3;
}

static void ktz8866_test_expect_chip(struct kunit *test, struct ktz8866_test_chip *chip,
				     unsigned int en, unsigned int brightness)
{
	KUNIT_EXPECT_EQ(test, chip->registers[BL_EN], en);
	KUNIT_EXPECT_EQ(test, chip->registers[BL_BRT_LSB], brightness & 7);
	KUNIT_EXPECT_EQ(test, chip->registers[BL_BRT_MSB], brightness >> 3);
}

/* Both chips are configured and follow every brightness change. */
static void ktz8866_test_secondary_brightness(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	ctx->ktz.secondary = &ctx->secondary;
	KUNIT_ASSERT_EQ(test, ktz8866_init(&ctx->ktz), 0);
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_TRUE(test, ctx->secondary.led_on);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x7f, DEFAULT_BRIGHTNESS);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x7f, DEFAULT_BRIGHTNESS);

	ctx->backlight.props.brightness = 1234;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x7f, 1234);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x7f, 1234);

	ctx->backlight.props.power = BACKLIGHT_POWER_OFF;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_FALSE(test, ctx->secondary.led_on);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x3f, 0);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x3f, 0);
}

/* A failure on the secondary chip is reported and a later request recovers. */
static void ktz8866_test_secondary_error(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	ctx->ktz.secondary = &ctx->secondary;
	ctx->chip2.fail_write = BL_BRT_MSB;
	ctx->backlight.props.brightness = 1000;
	KUNIT_EXPECT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), -EIO);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x40, 1000);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_FALSE(test, ctx->secondary.led_on);
	KUNIT_EXPECT_EQ(test, ctx->chip2.registers[BL_EN], 0U);

	ctx->chip2.fail_write = -1;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	KUNIT_EXPECT_TRUE(test, ctx->secondary.led_on);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x40, 1000);

	/* A failed primary write does not leave the secondary ahead of it. */
	ctx->chip.fail_write = BL_BRT_LSB;
	ctx->chip2.writes = 0;
	ctx->backlight.props.brightness = 500;
	KUNIT_EXPECT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->chip2.writes, 0U);

	/* Initialization errors on the secondary chip are reported too. */
	ctx->chip.fail_write = -1;
	ctx->chip2.fail_write = BL_EN;
	ctx->chip2.registers[BL_EN] = 0;
	KUNIT_EXPECT_EQ(test, ktz8866_init(&ctx->ktz), -EIO);
}

/* A backlight the bootloader left on is kept as it is: nothing is written. */
static void ktz8866_test_boot_brightness(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;
	unsigned int brightness;
	bool sync;

	ctx->ktz.secondary = &ctx->secondary;
	ktz8866_test_set_chip(&ctx->chip, 0x7f, 1234);
	ktz8866_test_set_chip(&ctx->chip2, 0x7f, 1234);

	KUNIT_ASSERT_EQ(test, ktz8866_get_boot_brightness(&ctx->ktz, &brightness, &sync), 0);
	KUNIT_EXPECT_EQ(test, brightness, 1234U);
	KUNIT_EXPECT_FALSE(test, sync);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
	KUNIT_EXPECT_TRUE(test, ctx->secondary.led_on);

	KUNIT_ASSERT_EQ(test, ktz8866_init(&ctx->ktz), 0);
	KUNIT_EXPECT_EQ(test, ctx->chip.writes, 0U);
	KUNIT_EXPECT_EQ(test, ctx->chip2.writes, 0U);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x7f, 1234);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x7f, 1234);
}

/* The chips are written when the boot state is off or the two disagree. */
static void ktz8866_test_boot_sync(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;
	unsigned int brightness;
	bool sync;

	ctx->ktz.secondary = &ctx->secondary;
	ktz8866_test_set_chip(&ctx->chip, 0x7f, 1234);
	ktz8866_test_set_chip(&ctx->chip2, 0x7f, 1000);
	KUNIT_ASSERT_EQ(test, ktz8866_get_boot_brightness(&ctx->ktz, &brightness, &sync), 0);
	KUNIT_EXPECT_EQ(test, brightness, 1234U);
	KUNIT_EXPECT_TRUE(test, sync);

	ctx->backlight.props.brightness = brightness;
	KUNIT_ASSERT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), 0);
	ktz8866_test_expect_chip(test, &ctx->chip, 0x7f, 1234);
	ktz8866_test_expect_chip(test, &ctx->chip2, 0x7f, 1234);

	/* Off at boot: the default brightness is used and written. */
	ktz8866_test_set_chip(&ctx->chip, 0x3f, 0);
	ktz8866_test_set_chip(&ctx->chip2, 0x3f, 0);
	KUNIT_ASSERT_EQ(test, ktz8866_get_boot_brightness(&ctx->ktz, &brightness, &sync), 0);
	KUNIT_EXPECT_EQ(test, brightness, (unsigned int)DEFAULT_BRIGHTNESS);
	KUNIT_EXPECT_TRUE(test, sync);
	KUNIT_EXPECT_FALSE(test, ctx->ktz.led_on);

	/* A read error on the secondary chip fails instead of guessing. */
	ctx->chip2.fail_read = BL_BRT_MSB;
	KUNIT_EXPECT_EQ(test, ktz8866_get_boot_brightness(&ctx->ktz, &brightness, &sync), -EIO);
}

/*
 * The fade time comes from the register, because the bootloader may have
 * programmed it, and the slower chip of a mirrored pair decides how long
 * the panel stays lit.
 */
static void ktz8866_test_off_ramp(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;
	unsigned int ramp;

	/* Code 0 means 512 us, codes 1..15 mean 1..16384 ms. */
	KUNIT_EXPECT_EQ(test, ktz8866_off_ramp_delay(0), 512U);
	KUNIT_EXPECT_EQ(test, ktz8866_off_ramp_delay(1), 1000U);
	KUNIT_EXPECT_EQ(test, ktz8866_off_ramp_delay(2), 2000U);
	KUNIT_EXPECT_EQ(test, ktz8866_off_ramp_delay(15), 16384000U);

	ctx->chip.registers[BL_DIMMING] = 0x40 | 3;
	KUNIT_ASSERT_EQ(test, ktz8866_get_off_ramp(&ctx->ktz, &ramp), 0);
	KUNIT_EXPECT_EQ(test, ramp, 3U);

	ctx->ktz.secondary = &ctx->secondary;
	ctx->chip2.registers[BL_DIMMING] = 0x70 | 7;
	KUNIT_ASSERT_EQ(test, ktz8866_get_off_ramp(&ctx->ktz, &ramp), 0);
	KUNIT_EXPECT_EQ(test, ramp, 7U);

	/* A register error is reported instead of guessing a fade time. */
	ctx->chip.fail_read = BL_DIMMING;
	KUNIT_EXPECT_EQ(test, ktz8866_get_off_ramp(&ctx->ktz, &ramp), -EIO);
}

/*
 * The fade time is read before the light is turned off, so a register error
 * cannot leave the panel lit after the video link is already gone.
 */
static void ktz8866_test_disable_ramp_error(struct kunit *test)
{
	struct ktz8866_test_context *ctx = test->priv;

	ctx->ktz.led_on = true;
	ctx->chip.registers[BL_EN] = 0x7f;
	ctx->backlight.props.brightness = 0;
	ctx->chip.fail_read = BL_DIMMING;

	KUNIT_EXPECT_EQ(test, ktz8866_backlight_update_status(&ctx->backlight), -EIO);
	KUNIT_EXPECT_EQ(test, ctx->chip.writes, 0U);
	KUNIT_EXPECT_TRUE(test, ctx->ktz.led_on);
}

static struct kunit_case ktz8866_test_cases[] = {
	KUNIT_CASE(ktz8866_test_brightness),
	KUNIT_CASE_PARAM(ktz8866_test_error, ktz8866_error_gen_params),
	KUNIT_CASE(ktz8866_test_init_error),
	KUNIT_CASE(ktz8866_test_secondary_brightness),
	KUNIT_CASE(ktz8866_test_secondary_error),
	KUNIT_CASE(ktz8866_test_boot_brightness),
	KUNIT_CASE(ktz8866_test_boot_sync),
	KUNIT_CASE(ktz8866_test_off_ramp),
	KUNIT_CASE(ktz8866_test_disable_ramp_error),
	{}
};

static struct kunit_suite ktz8866_test_suite = {
	.name = "ktz8866",
	.init = ktz8866_test_init,
	.exit = ktz8866_test_exit,
	.test_cases = ktz8866_test_cases,
};

kunit_test_suite(ktz8866_test_suite);
