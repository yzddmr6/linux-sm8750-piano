// SPDX-License-Identifier: GPL-2.0-only
/* Real regmap + SC8541 paths, with register storage in RAM (no I2C hardware). */
#include <kunit/device.h>
#include <kunit/test.h>

struct sc8541_test {
	struct sc8541 sc;
	u8 regs[SC8541_MAX_REG + 1];
	int fail_reg;
};

static int sc8541_test_read(void *ctx, unsigned int reg, unsigned int *value)
{
	struct sc8541_test *f = ctx;

	*value = f->regs[reg];
	return 0;
}

static int sc8541_test_write(void *ctx, unsigned int reg, unsigned int value)
{
	struct sc8541_test *f = ctx;

	if (reg == f->fail_reg)
		return -EIO;
	f->regs[reg] = value;
	return 0;
}

static const struct regmap_bus sc8541_test_bus = {
	.reg_read = sc8541_test_read,
	.reg_write = sc8541_test_write,
};

static void sc8541_test_cleanup(void *data)
{
	struct sc8541_test *f = data;

	cancel_delayed_work_sync(&f->sc.watch);
	regmap_exit(f->sc.regmap);
}

static struct sc8541_test *sc8541_test_init(struct kunit *test)
{
	struct sc8541_test *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->sc.dev = kunit_device_register(test, "sc8541-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->sc.dev);
	f->fail_reg = -1;
	f->sc.busocp_ma = SC8541_BUSOCP_MA;
	mutex_init(&f->sc.lock);
	INIT_DELAYED_WORK(&f->sc.watch, sc8541_watch);
	f->sc.regmap = regmap_init(f->sc.dev, &sc8541_test_bus, f, &sc8541_regmap_config);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, f->sc.regmap);
	kunit_add_action_or_reset(test, sc8541_test_cleanup, f);
	return f;
}

static void sc8541_test_protections(struct kunit *test)
{
	struct sc8541_test *f = sc8541_test_init(test);

	KUNIT_ASSERT_EQ(test, sc8541_enable(&f->sc), 0);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_BATOVP], 46);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_BATOVP_ALM], 41);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_BUSOVP], 80);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_BUSOVP_ALM], 70);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_BUSOCP] & 0x9f, 10);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_CTRL2] & SC8541_EN_BYPASS, 0);
	KUNIT_EXPECT_NE(test, f->regs[SC8541_CTRL2] & SC8541_CHG_EN, 0);
	/* Expired software kick must stop switching through the real worker. */
	cancel_delayed_work_sync(&f->sc.watch);
	sc8541_watch(&f->sc.watch.work);
	KUNIT_EXPECT_FALSE(test, f->sc.enabled);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_CTRL2] & SC8541_CHG_EN, 0);
}

static void sc8541_test_failed_arm(struct kunit *test)
{
	struct sc8541_test *f = sc8541_test_init(test);

	f->fail_reg = SC8541_BUSOVP;
	KUNIT_EXPECT_EQ(test, sc8541_enable(&f->sc), -EIO);
	KUNIT_EXPECT_FALSE(test, f->sc.enabled);
	KUNIT_EXPECT_EQ(test, f->regs[SC8541_CTRL2] & SC8541_CHG_EN, 0);
}

static void sc8541_test_adc(struct kunit *test)
{
	struct sc8541_test *f = sc8541_test_init(test);
	int value;

	f->regs[SC8541_ADC_BASE] = 1;
	f->regs[SC8541_ADC_BASE + 1] = 0;
	KUNIT_ASSERT_EQ(test, sc8541_read_adc(&f->sc, SC8541_ADC_IBUS, &value), 0);
	KUNIT_EXPECT_EQ(test, value, 640000);
	f->regs[SC8541_ADC_BASE + 2 * SC8541_ADC_TDIE + 1] = 100;
	KUNIT_ASSERT_EQ(test, sc8541_read_adc(&f->sc, SC8541_ADC_TDIE, &value), 0);
	KUNIT_EXPECT_EQ(test, value, 500);
}

static struct kunit_case sc8541_test_cases[] = {
	KUNIT_CASE(sc8541_test_protections),
	KUNIT_CASE(sc8541_test_failed_arm),
	KUNIT_CASE(sc8541_test_adc),
	{}
};

static struct kunit_suite sc8541_test_suite = {
	.name = "sc8541-charger",
	.test_cases = sc8541_test_cases,
};
kunit_test_suite(sc8541_test_suite);
