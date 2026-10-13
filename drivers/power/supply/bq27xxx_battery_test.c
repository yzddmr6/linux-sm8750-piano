// SPDX-License-Identifier: GPL-2.0-only
#include <kunit/device.h>
#include <kunit/test.h>

struct bq27xxx_pm_test {
	struct bq27xxx_device_info di;
	struct power_supply psy;
	atomic_t reads;
};

static int bq27xxx_test_read(struct bq27xxx_device_info *di, u8 reg,
			    bool single)
{
	struct bq27xxx_pm_test *ctx = container_of(di, struct bq27xxx_pm_test, di);

	atomic_inc(&ctx->reads);
	return -ENODEV;
}

static void bq27xxx_test_stop(void *data)
{
	struct bq27xxx_device_info *di = data;

	scoped_guard(mutex, &di->lock)
		di->removed = true;
	cancel_delayed_work_sync(&di->work);
}

static int bq27xxx_pm_test_init(struct kunit *test)
{
	struct bq27xxx_pm_test *ctx;
	struct bq27xxx_device_info *di;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	test->priv = ctx;
	di = &ctx->di;
	di->dev = kunit_device_register(test, "bq27xxx-pm");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, di->dev);
	dev_set_drvdata(di->dev, di);
	di->chip = BQ27Z561;
	di->regs = bq27xxx_chip_data[di->chip].regs;
	di->bus.read = bq27xxx_test_read;
	di->cache.flags = -ENODEV;
	di->bat = &ctx->psy;
	ctx->psy.drv_data = di;
	mutex_init(&di->lock);
	INIT_DELAYED_WORK(&di->work, bq27xxx_battery_poll);
	return kunit_add_action_or_reset(test, bq27xxx_test_stop, di);
}

static void bq27xxx_notification_after_suspend(struct kunit *test)
{
	struct bq27xxx_pm_test *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, bq27xxx_battery_suspend(ctx->di.dev), 0);
	bq27xxx_external_power_changed(&ctx->psy);
	flush_delayed_work(&ctx->di.work);
	KUNIT_EXPECT_EQ(test, atomic_read(&ctx->reads), 0);
}

static void bq27xxx_update_after_suspend(struct kunit *test)
{
	struct bq27xxx_pm_test *ctx = test->priv;

	KUNIT_ASSERT_EQ(test, bq27xxx_battery_suspend(ctx->di.dev), 0);
	bq27xxx_battery_update(&ctx->di);
	KUNIT_EXPECT_EQ(test, atomic_read(&ctx->reads), 0);
}

static void bq27xxx_property_after_suspend(struct kunit *test)
{
	struct bq27xxx_pm_test *ctx = test->priv;
	union power_supply_propval value;

	ctx->di.last_update = jiffies - 6 * HZ;
	KUNIT_ASSERT_EQ(test, bq27xxx_battery_suspend(ctx->di.dev), 0);
	KUNIT_EXPECT_EQ(test, bq27xxx_battery_get_property(&ctx->psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &value), -EAGAIN);
	KUNIT_EXPECT_EQ(test, atomic_read(&ctx->reads), 0);
}

static void bq27xxx_resume_restarts_updates(struct kunit *test)
{
	struct bq27xxx_pm_test *ctx = test->priv;
	int cycle;

	for (cycle = 1; cycle <= 2; cycle++) {
		KUNIT_ASSERT_EQ(test, bq27xxx_battery_suspend(ctx->di.dev), 0);
		KUNIT_ASSERT_EQ(test, bq27xxx_battery_resume(ctx->di.dev), 0);
		flush_delayed_work(&ctx->di.work);
		KUNIT_EXPECT_EQ(test, atomic_read(&ctx->reads), cycle);
	}
}

static struct kunit_case bq27xxx_pm_tests[] = {
	KUNIT_CASE(bq27xxx_notification_after_suspend),
	KUNIT_CASE(bq27xxx_update_after_suspend),
	KUNIT_CASE(bq27xxx_property_after_suspend),
	KUNIT_CASE(bq27xxx_resume_restarts_updates),
	{}
};

static struct kunit_suite bq27xxx_pm_test_suite = {
	.name = "bq27xxx-pm",
	.init = bq27xxx_pm_test_init,
	.test_cases = bq27xxx_pm_tests,
};
kunit_test_suite(bq27xxx_pm_test_suite);
