// SPDX-License-Identifier: GPL-2.0
/* Real driver callbacks with only regulator, I2C and delay boundaries stubbed. */
#include <kunit/test.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>

struct dw9768_test_event {
	int kind;
	unsigned int a;
	unsigned int b;
};

static struct {
	struct dw9768_test_event events[256];
	unsigned int count;
	unsigned int writes;
	int fail_write;
	int fail_rail;
	int enabled[3];
} fixture;

enum { ENABLE, DISABLE, WAIT, BYTE_WRITE, DAC_WRITE };

static void record(int kind, unsigned int a, unsigned int b)
{
	if (fixture.count < ARRAY_SIZE(fixture.events))
		fixture.events[fixture.count++] = (struct dw9768_test_event) {kind, a, b};
}

static int test_enable(struct regulator *regulator)
{
	unsigned long rail = (unsigned long)regulator;

	record(ENABLE, rail, 0);
	if (fixture.fail_rail == rail)
		return -EIO;
	fixture.enabled[rail]++;
	return 0;
}

static int test_disable(struct regulator *regulator)
{
	unsigned long rail = (unsigned long)regulator;

	record(DISABLE, rail, 0);
	fixture.enabled[rail]--;
	return 0;
}

static void test_wait(unsigned long min, unsigned long max)
{
	record(WAIT, min, max);
}

static s32 test_byte_write(const struct i2c_client *client, u8 reg, u8 value)
{
	record(BYTE_WRITE, reg, value);
	return ++fixture.writes == fixture.fail_write ? -EREMOTEIO : 0;
}

static s32 test_dac_write(const struct i2c_client *client, u8 reg, u16 value)
{
	record(DAC_WRITE, reg, value);
	return ++fixture.writes == fixture.fail_write ? -EREMOTEIO : 0;
}

/* Keep the tested functions byte-identical to the production translation unit. */
#define regulator_enable test_enable
#define regulator_disable test_disable
#define usleep_range test_wait
#define i2c_smbus_write_byte_data test_byte_write
#define i2c_smbus_write_word_swapped test_dac_write
#undef MODULE_DEVICE_TABLE
#define MODULE_DEVICE_TABLE(type, name)
#undef module_i2c_driver
#define module_i2c_driver(driver) \
	static void *__maybe_unused test_driver = &(driver)
#include "dw9768.c"

static int dw9768_test_init(struct kunit *test)
{
	struct dw9768 *lens;
	struct i2c_client *client;
	int ret;

	memset(&fixture, 0, sizeof(fixture));
	lens = kunit_kzalloc(test, sizeof(*lens), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lens);
	client = kunit_kzalloc(test, sizeof(*client), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, client);
	client->dev.init_name = "gt9764-kunit";
	lens->sd.dev = &client->dev;
	dev_set_drvdata(&client->dev, &lens->sd);
	v4l2_set_subdevdata(&lens->sd, client);
	lens->supplies[0].consumer = (struct regulator *)1; /* VIN */
	lens->supplies[1].consumer = (struct regulator *)2; /* VDD */
	lens->is_gt9764 = true;
	lens->move_delay_us = 6650;
	ret = dw9768_init_controls(lens);
	if (ret)
		return ret;
	test->priv = lens;
	return 0;
}

static void dw9768_test_exit(struct kunit *test)
{
	struct dw9768 *lens = test->priv;

	v4l2_ctrl_handler_free(&lens->ctrls);
}

static void gt9764_stock_sequence(struct kunit *test)
{
	/* Independent contract decoded from the stock actuator module. */
	static const struct dw9768_test_event expected[] = {
		{ ENABLE, 2, 0 }, { WAIT, 10000, 11000 },
		{ ENABLE, 1, 0 }, { WAIT, 2000, 2100 },
		{ BYTE_WRITE, 0x02, 0x00 }, { BYTE_WRITE, 0x02, 0x02 },
		{ WAIT, 100, 200 }, { BYTE_WRITE, 0x06, 0x40 },
		{ WAIT, 100, 200 }, { BYTE_WRITE, 0x07, 0x73 },
		{ WAIT, 100, 200 }, { DAC_WRITE, 0x03, 0 },
	};
	struct dw9768 *lens = test->priv;

	KUNIT_ASSERT_EQ(test, dw9768_runtime_resume(lens->sd.dev), 0);
	KUNIT_ASSERT_GE(test, fixture.count, ARRAY_SIZE(expected));
	KUNIT_EXPECT_MEMEQ(test, fixture.events, expected, sizeof(expected));
	KUNIT_EXPECT_EQ(test, fixture.enabled[1], 1);
	KUNIT_EXPECT_EQ(test, fixture.enabled[2], 1);
	KUNIT_ASSERT_EQ(test, dw9768_runtime_suspend(lens->sd.dev), 0);
	KUNIT_EXPECT_EQ(test, fixture.enabled[1], 0);
	KUNIT_EXPECT_EQ(test, fixture.enabled[2], 0);
	KUNIT_EXPECT_EQ(test, fixture.events[fixture.count - 2].kind, DISABLE);
	KUNIT_EXPECT_EQ(test, fixture.events[fixture.count - 2].a, 1);
	KUNIT_EXPECT_EQ(test, fixture.events[fixture.count - 1].a, 2);
	/* No unverified 0x02=0x01 standby write on GT9764. */
	KUNIT_EXPECT_EQ(test, fixture.writes, 6);
}

static void gt9764_focus_contract(struct kunit *test)
{
	struct dw9768 *lens = test->priv;
	int ret;

	KUNIT_EXPECT_EQ(test, lens->focus->id, V4L2_CID_FOCUS_ABSOLUTE);
	KUNIT_EXPECT_EQ(test, lens->focus->minimum, 0);
	KUNIT_EXPECT_EQ(test, lens->focus->maximum, 1023);
	KUNIT_EXPECT_EQ(test, lens->focus->step, 1);
	ret = v4l2_ctrl_s_ctrl(lens->focus, 1023);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, fixture.count, 1);
	KUNIT_EXPECT_EQ(test, fixture.events[0].kind, DAC_WRITE);
	KUNIT_EXPECT_EQ(test, fixture.events[0].a, 0x03);
	KUNIT_EXPECT_EQ(test, fixture.events[0].b, 1023);
	fixture.fail_write = 2;
	KUNIT_EXPECT_EQ(test, v4l2_ctrl_s_ctrl(lens->focus, 128), -EREMOTEIO);
	KUNIT_EXPECT_EQ(test, lens->focus->cur.val, 1023);
}

static void gt9764_failure_unwind(struct kunit *test)
{
	struct dw9768 *lens = test->priv;
	int i;

	/* Each initialization write and the initial DAC restore can fail. */
	for (i = 1; i <= 5; i++) {
		memset(&fixture, 0, sizeof(fixture));
		/* Other camera consumers already hold both shared rails. */
		fixture.enabled[1] = fixture.enabled[2] = 1;
		fixture.fail_write = i;
		KUNIT_EXPECT_EQ(test, dw9768_runtime_resume(lens->sd.dev), -EREMOTEIO);
		KUNIT_EXPECT_EQ(test, fixture.enabled[1], 1);
		KUNIT_EXPECT_EQ(test, fixture.enabled[2], 1);
		KUNIT_EXPECT_EQ(test, fixture.writes, i);
		KUNIT_EXPECT_EQ(test, fixture.events[fixture.count - 2].a, 1);
		KUNIT_EXPECT_EQ(test, fixture.events[fixture.count - 1].a, 2);
	}
	for (i = 1; i <= 2; i++) {
		memset(&fixture, 0, sizeof(fixture));
		fixture.fail_rail = i;
		KUNIT_EXPECT_EQ(test, dw9768_runtime_resume(lens->sd.dev), -EIO);
		KUNIT_EXPECT_EQ(test, fixture.enabled[1], 0);
		KUNIT_EXPECT_EQ(test, fixture.enabled[2], 0);
		KUNIT_EXPECT_EQ(test, fixture.writes, 0);
	}
}

static struct kunit_case dw9768_test_cases[] = {
	KUNIT_CASE(gt9764_stock_sequence),
	KUNIT_CASE(gt9764_focus_contract),
	KUNIT_CASE(gt9764_failure_unwind),
	{}
};

static struct kunit_suite dw9768_test_suite = {
	.name = "dw9768-gt9764",
	.init = dw9768_test_init,
	.exit = dw9768_test_exit,
	.test_cases = dw9768_test_cases,
};
kunit_test_suite(dw9768_test_suite);
