// SPDX-License-Identifier: GPL-2.0-only
/* Included by the transport: exercise real regmap/I2C, HID and input paths. */
#include <kunit/device.h>
#include <kunit/test.h>

struct wn8030_test_bus {
	struct nanosic_wn8030 nano;
	struct i2c_adapter adapter;
	int forced_result;
	u8 last[514];
	u16 last_size;
	u8 state;
};

static int wn8030_test_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct wn8030_test_bus *bus = i2c_get_adapdata(adapter);

	if (bus->forced_result != INT_MAX)
		return bus->forced_result;
	if (msgs[0].addr != 0x4c || !msgs[0].len || msgs[0].len > sizeof(bus->last))
		return -EINVAL;
	memcpy(bus->last, msgs[0].buf, msgs[0].len);
	bus->last_size = msgs[0].len;
	if (num == 2) {
		memset(msgs[1].buf, 0, msgs[1].len);
		msgs[1].buf[0] = bus->state;
	}
	return num;
}

static u32 wn8030_test_functionality(struct i2c_adapter *adapter)
{
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm wn8030_test_algorithm = {
	.xfer = wn8030_test_xfer,
	.functionality = wn8030_test_functionality,
};

static void wn8030_test_del_adapter(void *data) { i2c_del_adapter(data); }
static void wn8030_test_del_client(void *data) { i2c_unregister_device(data); }
static void wn8030_test_cleanup(void *data)
{
	struct nanosic_wn8030 *nano = data;

	cancel_work_sync(&nano->input_state_work);
	if (nano->hid_keyboard)
		hid_destroy_device(nano->hid_keyboard);
	if (nano->hid_touchpad)
		hid_destroy_device(nano->hid_touchpad);
	misc_deregister(&nano->hinge_misc);
}

static int wn8030_test_init(struct kunit *test)
{
	struct wn8030_test_bus *bus;
	struct nanosic_wn8030 *nano;
	struct device *dev;
	int ret;

	bus = kunit_kzalloc(test, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;
	test->priv = bus;
	nano = &bus->nano;
	dev = kunit_device_register(test, "wn8030-test");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	bus->forced_result = INT_MAX;
	bus->adapter.owner = THIS_MODULE;
	bus->adapter.dev.parent = dev;
	bus->adapter.algo = &wn8030_test_algorithm;
	strscpy(bus->adapter.name, "wn8030-test", sizeof(bus->adapter.name));
	i2c_set_adapdata(&bus->adapter, bus);
	ret = i2c_add_adapter(&bus->adapter);
	if (ret)
		return ret;
	ret = kunit_add_action_or_reset(test, wn8030_test_del_adapter, &bus->adapter);
	if (ret)
		return ret;
	nano->client = i2c_new_dummy_device(&bus->adapter, 0x4c);
	if (IS_ERR(nano->client))
		return PTR_ERR(nano->client);
	ret = kunit_add_action_or_reset(test, wn8030_test_del_client, nano->client);
	if (ret)
		return ret;
	nano->dev = &nano->client->dev;
	nano->variant = &nanosic_wn8030_piano;
	nano->no_touchpad = true;
	nano->input_enabled = true;
	nano->regmap = devm_regmap_init_i2c(nano->client, &nanosic_wn8030_regmap_config);
	if (IS_ERR(nano->regmap))
		return PTR_ERR(nano->regmap);
	mutex_init(&nano->conn_mutex);
	init_completion(&nano->attach_known);
	INIT_WORK(&nano->input_state_work, nanosic_wn8030_input_state_work);
	spin_lock_init(&nano->hinge_lock);
	init_waitqueue_head(&nano->hinge_read_wq);
	nano->hinge_misc.minor = MISC_DYNAMIC_MINOR;
	nano->hinge_misc.name = "wn8030-test-hinge";
	nano->hinge_misc.fops = &nanosic_hinge_fops;
	nano->hinge_misc.parent = nano->dev;
	ret = misc_register(&nano->hinge_misc);
	if (ret)
		return ret;
	ret = kunit_add_action_or_reset(test, wn8030_test_cleanup, nano);
	if (ret)
		return ret;
	return nanosic_wn8030_alloc_tablet(nano);
}

static void wn8030_transfer_errors(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;

	bus->forced_result = 0;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_set_caps_led(&bus->nano, true), -EIO);
	bus->forced_result = -EREMOTEIO;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_get_boot_state(&bus->nano), -EREMOTEIO);
}

static void wn8030_led_commands(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct hid_device *hid = kunit_kzalloc(test, sizeof(*hid), GFP_KERNEL);
	u8 caps[] = { 5, BIT(1) };

	KUNIT_ASSERT_NOT_NULL(test, hid);
	hid->driver_data = &bus->nano;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_output_report(hid, caps, sizeof(caps)), 2);
	KUNIT_EXPECT_EQ(test, bus->last[0], 0x4c);
	KUNIT_EXPECT_EQ(test, bus->last[7], 0x2e);
	KUNIT_EXPECT_EQ(test, bus->last[9], 0xfd);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_set_kbd_backlight(&bus->nano, 42), 0);
	KUNIT_EXPECT_EQ(test, bus->last[7], 0x23);
	KUNIT_EXPECT_EQ(test, bus->last[9], 42);
	KUNIT_EXPECT_EQ(test, bus->last_size, 67);
	KUNIT_EXPECT_EQ(test, bus->last[10], nanosic_wn8030_checksum8(bus->last + 3, 7));
}

static void wn8030_attach_switch_and_no_touchpad(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	u8 report[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2 };

	/* Initial detached state, then powered attach and real HID registration. */
	nanosic_wn8030_handle_vendor(nano, report, sizeof(report));
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_register_tablet(nano), 0);
	KUNIT_EXPECT_TRUE(test, test_bit(SW_TABLET_MODE, nano->tablet->sw));
	report[9] = 3;
	nanosic_wn8030_handle_vendor(nano, report, sizeof(report));
	KUNIT_EXPECT_FALSE(test, test_bit(SW_TABLET_MODE, nano->tablet->sw));
	KUNIT_ASSERT_NOT_NULL(test, nano->hid_keyboard);
	KUNIT_EXPECT_PTR_EQ(test, nano->hid_touchpad, NULL);
	KUNIT_EXPECT_TRUE(test, completion_done(&nano->attach_known));
	/* Connected but unpowered is still laptop mode. */
	report[9] = 1;
	nanosic_wn8030_handle_vendor(nano, report, sizeof(report));
	KUNIT_EXPECT_FALSE(test, test_bit(SW_TABLET_MODE, nano->tablet->sw));
	report[9] = 2;
	nanosic_wn8030_handle_vendor(nano, report, sizeof(report));
	KUNIT_EXPECT_TRUE(test, test_bit(SW_TABLET_MODE, nano->tablet->sw));
	KUNIT_EXPECT_PTR_EQ(test, nano->hid_keyboard, NULL);
	report[9] = 3;
	nanosic_wn8030_handle_vendor(nano, report, sizeof(report));
	KUNIT_ASSERT_NOT_NULL(test, nano->hid_keyboard);
	KUNIT_EXPECT_PTR_EQ(test, nano->hid_touchpad, NULL);
}

static void wn8030_initial_attached(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	u8 report[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2, 0, 0, 0, 0, 1 };

	nanosic_wn8030_handle_vendor(&bus->nano, report, sizeof(report));
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_register_tablet(&bus->nano), 0);
	KUNIT_EXPECT_FALSE(test, test_bit(SW_TABLET_MODE, bus->nano.tablet->sw));
	bus->nano.hall_handler.private = &bus->nano;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_hall_connect(&bus->nano.hall_handler,
			bus->nano.tablet, NULL), -ENODEV);
}

static void wn8030_firmware_transport(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	u8 header[64] = {}, code[508] = {};
	struct nanosic_firmware fw = { .header = header, .code = code, .size = sizeof(code) };

	bus->state = 0;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_send_header(&bus->nano, &fw), -EINVAL);
	bus->state = 1;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_send_header(&bus->nano, &fw), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_send_data(&bus->nano, &fw), 0);
	KUNIT_EXPECT_EQ(test, bus->last[0], 0x5c);
	KUNIT_EXPECT_EQ(test, bus->last[1], 7);
	KUNIT_EXPECT_EQ(test, bus->last_size, 513);
	KUNIT_EXPECT_EQ(test, get_unaligned_be32(bus->last + 2), 503);
	KUNIT_EXPECT_EQ(test, get_unaligned_be32(bus->last + 6), 5);
}

static bool wn8030_key_down(struct hid_device *hid, unsigned int key)
{
	struct hid_input *input;

	list_for_each_entry(input, &hid->inputs, list)
		if (test_bit(key, input->input->key))
			return true;
	return false;
}

static void wn8030_reports(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	u8 attach[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2, 0, 0, 0, 0, 3 };
	u8 key[9] = { 5, 0, 0, 4 }, consumer[5] = { 6, 0xe9, 0 }, touch[27] = { 0x19 };
	struct hid_report_enum *reports;

	nanosic_wn8030_handle_vendor(&bus->nano, attach, sizeof(attach));
	KUNIT_ASSERT_NOT_NULL(test, bus->nano.hid_keyboard);
	reports = &bus->nano.hid_keyboard->report_enum[HID_INPUT_REPORT];
	KUNIT_ASSERT_NOT_NULL(test, reports->report_id_hash[5]);
	KUNIT_ASSERT_NOT_NULL(test, reports->report_id_hash[6]);
	KUNIT_EXPECT_EQ(test, reports->report_id_hash[6]->size, 16);
	KUNIT_EXPECT_PTR_EQ(test, reports->report_id_hash[2], NULL);
	KUNIT_EXPECT_PTR_EQ(test, reports->report_id_hash[0x19], NULL);
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, key, sizeof(key)), 9);
	KUNIT_EXPECT_TRUE(test, wn8030_key_down(bus->nano.hid_keyboard, KEY_A));
	key[3] = 0;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, key, sizeof(key)), 9);
	KUNIT_EXPECT_FALSE(test, wn8030_key_down(bus->nano.hid_keyboard, KEY_A));
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, consumer, sizeof(consumer)), 5);
	KUNIT_EXPECT_TRUE(test, wn8030_key_down(bus->nano.hid_keyboard, KEY_VOLUMEUP));
	consumer[1] = 0;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, consumer, sizeof(consumer)), 5);
	KUNIT_EXPECT_FALSE(test, wn8030_key_down(bus->nano.hid_keyboard, KEY_VOLUMEUP));
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, touch, sizeof(touch)), 27);
	KUNIT_EXPECT_PTR_EQ(test, bus->nano.hid_touchpad, NULL);
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_dispatch(&bus->nano, key, 8), 0);
}

static struct kunit_case wn8030_cases[] = {
	KUNIT_CASE(wn8030_transfer_errors),
	KUNIT_CASE(wn8030_led_commands),
	KUNIT_CASE(wn8030_attach_switch_and_no_touchpad),
	KUNIT_CASE(wn8030_initial_attached),
	KUNIT_CASE(wn8030_firmware_transport),
	KUNIT_CASE(wn8030_reports),
	{}
};
static struct kunit_suite wn8030_suite = {
	.name = "nanosic-wn8030",
	.init = wn8030_test_init,
	.test_cases = wn8030_cases,
};
kunit_test_suite(wn8030_suite);
