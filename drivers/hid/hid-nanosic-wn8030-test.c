// SPDX-License-Identifier: GPL-2.0-only
/* Included by the transport: exercise real regmap/I2C, HID and input paths. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/gpio/driver.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/pm_wakeup.h>
#include <linux/gpio/machine.h>

enum wn8030_test_event {
	TEST_POWER_OFF = 1, TEST_SLEEP, TEST_WAKE, TEST_READ,
	TEST_KEY_DOWN, TEST_KEY_UP, TEST_POWER_ON, TEST_LED,
	TEST_WAKE_KEY_DOWN, TEST_WAKE_KEY_UP,
};

struct wn8030_test_bus {
	struct nanosic_wn8030 nano;
	struct i2c_adapter adapter;
	int forced_result;
	u8 last[514];
	u16 last_size;
	u8 state;
	struct gpio_chip gpio;
	struct drm_panel panel;
	int sleep, ready;
	bool wake_enabled;
	unsigned int transfers;
	u8 frames[8][XM_WN8030_I2C_READ];
	int frame_count, frame_read;
	spinlock_t trace_lock;
	int trace[128];
	unsigned int trace_count;
	struct input_handler input_handler;
	struct input_handle key_handle, wake_handle;
};

static void wn8030_test_record(struct wn8030_test_bus *bus, int event)
{
	unsigned long flags;

	spin_lock_irqsave(&bus->trace_lock, flags);
	if (bus->trace_count < ARRAY_SIZE(bus->trace))
		bus->trace[bus->trace_count++] = event;
	spin_unlock_irqrestore(&bus->trace_lock, flags);
}

static int wn8030_test_init_lines(struct kunit *test, struct wn8030_test_bus *bus);

static int wn8030_test_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct wn8030_test_bus *bus = i2c_get_adapdata(adapter);

	bus->transfers++;
	if (bus->forced_result != INT_MAX)
		return bus->forced_result;
	if (msgs[0].addr != 0x4c || !msgs[0].len || msgs[0].len > sizeof(bus->last))
		return -EINVAL;
	memcpy(bus->last, msgs[0].buf, msgs[0].len);
	bus->last_size = msgs[0].len;
	if (num == 2) {
		memset(msgs[1].buf, 0, msgs[1].len);
		if (msgs[0].buf[0] == 0x4c) {
			wn8030_test_record(bus, TEST_READ);
			msgs[1].buf[0] = 0x57;
			if (bus->frame_read < bus->frame_count)
				memcpy(msgs[1].buf, bus->frames[bus->frame_read++], msgs[1].len);
		} else {
			msgs[1].buf[0] = bus->state;
		}
	} else if (msgs[0].len == 67) {
		if (msgs[0].buf[7] == 0x25)
			wn8030_test_record(bus, msgs[0].buf[9] ? TEST_POWER_ON : TEST_POWER_OFF);
		if (msgs[0].buf[7] == 0x2e || msgs[0].buf[7] == 0x23)
			wn8030_test_record(bus, TEST_LED);
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

	cancel_work_sync(&nano->panel_work);
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
	i2c_set_clientdata(nano->client, nano);
	spin_lock_init(&bus->trace_lock);
	nano->variant = &nanosic_wn8030_piano;
	nano->no_touchpad = true;
	nano->input_enabled = true;
	nano->userspace_angle_enabled = true;
	nano->follower.panel = &bus->panel;
	nano->regmap = devm_regmap_init_i2c(nano->client, &nanosic_wn8030_regmap_config);
	if (IS_ERR(nano->regmap))
		return PTR_ERR(nano->regmap);
	mutex_init(&nano->conn_mutex);
	mutex_init(&nano->io_mutex);
	mutex_init(&nano->lifecycle_mutex);
	nano->panel_on = true;
	INIT_WORK(&nano->panel_work, nanosic_wn8030_panel_work);
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
	ret = nanosic_wn8030_alloc_tablet(nano);
	if (ret)
		return ret;
	ret = wn8030_test_init_lines(test, bus);
	if (ret)
		return ret;
	return kunit_add_action_or_reset(test, wn8030_test_cleanup, nano);
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

/* The fake lines and IRQ chip below exercise gpiolib and IRQ ownership too. */
static void wn8030_test_irq_noop(struct irq_data *data) { }
static int wn8030_test_irq_type(struct irq_data *data, unsigned int type) { return 0; }
static int wn8030_test_irq_wake(struct irq_data *data, unsigned int on)
{
	struct wn8030_test_bus *bus = irq_data_get_irq_chip_data(data);

	bus->wake_enabled = on;
	return 0;
}
static struct irq_chip wn8030_test_irq_chip = {
	.name = "wn8030-test",
	.irq_mask = wn8030_test_irq_noop,
	.irq_unmask = wn8030_test_irq_noop,
	.irq_set_type = wn8030_test_irq_type,
	.irq_set_wake = wn8030_test_irq_wake,
};

static int wn8030_test_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct wn8030_test_bus *bus = gpiochip_get_data(chip);

	return offset ? bus->ready : bus->sleep;
}
static int wn8030_test_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct wn8030_test_bus *bus = gpiochip_get_data(chip);

	if (!offset) {
		bus->sleep = value;
		wn8030_test_record(bus, value ? TEST_SLEEP : TEST_WAKE);
	}
	return 0;
}
static int wn8030_test_gpio_input(struct gpio_chip *chip, unsigned int offset) { return 0; }
static int wn8030_test_gpio_direction(struct gpio_chip *chip, unsigned int offset)
{
	return offset ? GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static void wn8030_test_gpio_free(void *data) { gpiochip_free_own_desc(data); }
static void wn8030_test_irq_free(void *data)
{
	struct wn8030_test_bus *bus = data;

	free_irq(bus->nano.wake_irq, &bus->nano);
	free_irq(bus->nano.client->irq, &bus->nano);
	irq_free_descs(bus->nano.client->irq, 2);
	device_init_wakeup(bus->nano.dev, false);
}

static int wn8030_test_init_lines(struct kunit *test, struct wn8030_test_bus *bus)
{
	struct nanosic_wn8030 *nano = &bus->nano;
	struct gpio_chip *chip = &bus->gpio;
	int ret, irq, i;

	chip->label = "wn8030-test";
	chip->parent = nano->dev;
	chip->owner = THIS_MODULE;
	chip->base = -1;
	chip->ngpio = 2;
	chip->get = wn8030_test_gpio_get;
	chip->set = wn8030_test_gpio_set;
	chip->direction_input = wn8030_test_gpio_input;
	chip->direction_output = wn8030_test_gpio_set;
	chip->get_direction = wn8030_test_gpio_direction;
	ret = devm_gpiochip_add_data(nano->dev, chip, bus);
	if (ret)
		return ret;
	nano->sleep_gpio = gpiochip_request_own_desc(chip, 0, "sleep", GPIO_ACTIVE_HIGH,
						   GPIOD_OUT_LOW);
	if (IS_ERR(nano->sleep_gpio))
		return PTR_ERR(nano->sleep_gpio);
	ret = kunit_add_action_or_reset(test, wn8030_test_gpio_free, nano->sleep_gpio);
	if (ret)
		return ret;
	nano->data_gpio = gpiochip_request_own_desc(chip, 1, "data", GPIO_ACTIVE_HIGH, GPIOD_IN);
	if (IS_ERR(nano->data_gpio))
		return PTR_ERR(nano->data_gpio);
	ret = kunit_add_action_or_reset(test, wn8030_test_gpio_free, nano->data_gpio);
	if (ret)
		return ret;
	bus->ready = 1;
	irq = irq_alloc_descs(-1, 0, 2, NUMA_NO_NODE);
	if (irq < 0)
		return irq;
	nano->client->irq = irq;
	nano->wake_irq = irq + 1;
	for (i = 0; i < 2; i++) {
		irq_set_chip_data(irq + i, bus);
		irq_set_chip_and_handler(irq + i, &wn8030_test_irq_chip, handle_simple_irq);
	}
	ret = request_threaded_irq(irq, NULL, nanosic_wn8030_handler, IRQF_ONESHOT,
				   "wn8030-test-data", nano);
	if (ret)
		goto free_descs;
	ret = request_threaded_irq(irq + 1, NULL, nanosic_wn8030_wakeup_handler,
				   IRQF_ONESHOT, "wn8030-test-wake", nano);
	if (ret) {
		free_irq(irq, nano);
		goto free_descs;
	}
	ret = kunit_add_action_or_reset(test, wn8030_test_irq_free, bus);
	if (ret)
		return ret;
	return device_init_wakeup(nano->dev, true);
free_descs:
	irq_free_descs(irq, 2);
	return ret;
}

static void wn8030_test_input_event(struct input_handle *handle,
				  unsigned int type, unsigned int code, int value)
{
	struct wn8030_test_bus *bus = handle->private;

	if (type == EV_KEY && code == KEY_A)
		wn8030_test_record(bus, value ? TEST_KEY_DOWN : TEST_KEY_UP);
	if (type == EV_KEY && code == KEY_WAKEUP)
		wn8030_test_record(bus, value ? TEST_WAKE_KEY_DOWN : TEST_WAKE_KEY_UP);
}
static void wn8030_test_input_close(void *data)
{
	struct input_handle *handle = data;

	input_close_device(handle);
	input_unregister_handle(handle);
}
static int wn8030_test_watch_input(struct kunit *test, struct input_dev *input,
				  struct input_handle *handle)
{
	struct wn8030_test_bus *bus = test->priv;
	int ret;

	handle->dev = input;
	handle->handler = &bus->input_handler;
	handle->name = "wn8030-test";
	handle->private = bus;
	bus->input_handler.event = wn8030_test_input_event;
	INIT_LIST_HEAD(&bus->input_handler.h_list);
	ret = input_register_handle(handle);
	if (ret)
		return ret;
	ret = input_open_device(handle);
	if (ret) {
		input_unregister_handle(handle);
		return ret;
	}
	return kunit_add_action_or_reset(test, wn8030_test_input_close, handle);
}

static void wn8030_test_attach(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	u8 attach[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2, 0, 0, 0, 0, 3 };

	nanosic_wn8030_handle_vendor(&bus->nano, attach, sizeof(attach));
	KUNIT_ASSERT_NOT_NULL(test, bus->nano.hid_keyboard);
	flush_work(&bus->nano.input_state_work);
	bus->trace_count = 0;
}
static void wn8030_test_frame(struct wn8030_test_bus *bus, const u8 *report, size_t size)
{
	u8 *frame = bus->frames[bus->frame_count++];

	frame[0] = 0x57;
	frame[1] = 0x30;
	frame[2] = 0x39;
	memcpy(frame + 3, report, size);
}
static int wn8030_test_find(struct wn8030_test_bus *bus, int event)
{
	int i;

	for (i = 0; i < bus->trace_count; i++)
		if (bus->trace[i] == event)
			return i;
	return -1;
}
static unsigned int wn8030_test_count(struct wn8030_test_bus *bus, int event)
{
	unsigned int i, count = 0;

	for (i = 0; i < bus->trace_count; i++)
		count += bus->trace[i] == event;
	return count;
}

static void wn8030_panel_order_and_first_frame(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	struct hid_device *hid;
	struct hid_input *input;
	u8 key[9] = { 5, 0, 0, 4 };
	int off, sleep, wake, read, down, up, on, led;

	wn8030_test_attach(test);
	hid = nano->hid_keyboard;
	KUNIT_ASSERT_NOT_NULL(test, hid);
	input = list_first_entry(&hid->inputs, struct hid_input, list);
	KUNIT_ASSERT_EQ(test, wn8030_test_watch_input(test, input->input, &bus->key_handle), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	wn8030_test_frame(bus, key, sizeof(key));
	key[3] = 0;
	wn8030_test_frame(bus, key, sizeof(key));
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	flush_work(&nano->input_state_work);
	off = wn8030_test_find(bus, TEST_POWER_OFF);
	sleep = wn8030_test_find(bus, TEST_SLEEP);
	wake = wn8030_test_find(bus, TEST_WAKE);
	read = wn8030_test_find(bus, TEST_READ);
	down = wn8030_test_find(bus, TEST_KEY_DOWN);
	up = wn8030_test_find(bus, TEST_KEY_UP);
	on = wn8030_test_find(bus, TEST_POWER_ON);
	led = wn8030_test_find(bus, TEST_LED);
	KUNIT_EXPECT_GE(test, off, 0);
	KUNIT_EXPECT_GT(test, sleep, off);
	KUNIT_EXPECT_GT(test, wake, sleep);
	KUNIT_EXPECT_GT(test, read, wake);
	KUNIT_EXPECT_GT(test, down, read);
	KUNIT_EXPECT_GT(test, up, down);
	KUNIT_EXPECT_GT(test, on, up);
	KUNIT_EXPECT_GT(test, led, on);
	KUNIT_EXPECT_EQ(test, bus->frame_read, 2);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_READ), 3);
	KUNIT_EXPECT_PTR_EQ(test, nano->hid_keyboard, hid);
	KUNIT_EXPECT_FALSE(test, wn8030_key_down(hid, KEY_A));
}

static void wn8030_system_sleep_no_i2c(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	u8 held[9] = { 5, 0, 0, 4 };
	unsigned int transfers;

	wn8030_test_attach(test);
	nanosic_wn8030_dispatch(nano, held, sizeof(held));
	KUNIT_ASSERT_TRUE(test, wn8030_key_down(nano->hid_keyboard, KEY_A));
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_suspend(nano->dev), 0);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_OFF), 1);
	KUNIT_EXPECT_TRUE(test, bus->wake_enabled);
	KUNIT_EXPECT_FALSE(test, wn8030_key_down(nano->hid_keyboard, KEY_A));
	transfers = bus->transfers;
	nanosic_wn8030_handler(nano->client->irq, nano);
	nanosic_wn8030_handler(nano->client->irq, nano);
	nanosic_wn8030_set_caps_led(nano, true);
	nanosic_wn8030_set_kbd_backlight(nano, 50);
	nanosic_wn8030_xm_auth_init(nano);
	nanosic_wn8030_input_state_work(&nano->input_state_work);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_EXPECT_TRUE(test, nano->suspended);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_resume(nano->dev), 0);
	flush_work(&nano->panel_work);
	KUNIT_EXPECT_FALSE(test, bus->wake_enabled);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 1);
}

static void wn8030_blank_attach_detach(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	u8 attach[32] = { 0x23, 0x31, 0x38, 0x80, 0xa2, 0, 0, 0, 0, 1 };
	unsigned int transfers;

	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	transfers = bus->transfers;
	/* A queued attach is not consumed or powered while blanked. */
	wn8030_test_frame(bus, attach, sizeof(attach));
	attach[9] = 0;
	wn8030_test_frame(bus, attach, sizeof(attach));
	nanosic_wn8030_handler(nano->client->irq, nano);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_EXPECT_EQ(test, bus->frame_read, 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	flush_work(&nano->input_state_work);
	KUNIT_EXPECT_EQ(test, bus->frame_read, 2);
	KUNIT_EXPECT_FALSE(test, nano->keyboard_attached);
	KUNIT_EXPECT_PTR_EQ(test, nano->hid_keyboard, NULL);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 0);
	/* Attach and stay attached while blanked: power once after draining. */
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	attach[9] = 1;
	wn8030_test_frame(bus, attach, sizeof(attach));
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	flush_work(&nano->input_state_work);
	KUNIT_EXPECT_TRUE(test, nano->keyboard_attached);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 1);
}

static void wn8030_pm_callback_orders(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	unsigned int transfers;

	wn8030_test_attach(test);
	/* Panel first on suspend; panel first on resume. */
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_suspend(nano->dev), 0);
	transfers = bus->transfers;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_resume(nano->dev), 0);
	flush_work(&nano->panel_work);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_OFF), 1);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 1);
	/* System first on suspend; system first on resume. */
	bus->trace_count = 0;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_suspend(nano->dev), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	transfers = bus->transfers;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_resume(nano->dev), 0);
	flush_work(&nano->panel_work);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_OFF), 1);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 1);
	/* A later blank cancels the pending enable before adapter resume. */
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_suspend(nano->dev), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	transfers = bus->transfers;
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_resume(nano->dev), 0);
	flush_work(&nano->panel_work);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
}

static void wn8030_wakeup_only_asleep(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	unsigned int transfers;

	KUNIT_ASSERT_EQ(test, nanosic_wn8030_register_tablet(nano), 0);
	KUNIT_ASSERT_EQ(test, wn8030_test_watch_input(test, nano->tablet, &bus->wake_handle), 0);
	transfers = bus->transfers;
	nanosic_wn8030_wakeup_handler(nano->wake_irq, nano);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_WAKE_KEY_DOWN), 0);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	nanosic_wn8030_wakeup_handler(nano->wake_irq, nano);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_WAKE_KEY_DOWN), 1);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_WAKE_KEY_UP), 1);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_EXPECT_FALSE(test, test_bit(KEY_WAKEUP, nano->tablet->key));
	KUNIT_EXPECT_TRUE(test, nano->dev->power.wakeup->active);
}

static void wn8030_ready_timeout_no_reset(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	unsigned int transfers;

	wn8030_test_attach(test);
	KUNIT_ASSERT_EQ(test, nanosic_wn8030_panel_disabling(&nano->follower), 0);
	bus->ready = 0;
	transfers = bus->transfers;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_panel_enabled(&nano->follower), -ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	KUNIT_EXPECT_TRUE(test, nano->bridge_asleep);
	KUNIT_EXPECT_EQ(test, bus->sleep, 1);
	KUNIT_EXPECT_EQ(test, wn8030_test_count(bus, TEST_POWER_ON), 0);
}

static void wn8030_pending_outputs_and_auth(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	struct nanosic_wn8030 *nano = &bus->nano;
	unsigned int transfers = bus->transfers;

	/* An output producer may have checked policy before wake took io_mutex. */
	nano->waking = true;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_set_indicator_led(nano, 0xfd), 0);
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_set_kbd_backlight(nano, 50), 0);
	nano->no_touchpad = false;
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_set_touchpad(nano, true), 0);
	nano->no_touchpad = true;
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
	/* Dispatch must still reply to authentication in the pending frames. */
	KUNIT_EXPECT_EQ(test, nanosic_wn8030_xm_auth_init(nano), 0);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers + 1);
	KUNIT_EXPECT_EQ(test, bus->last[7], 0x31);
	nano->waking = false;
}

static void wn8030_no_periodic_power(struct kunit *test)
{
	struct wn8030_test_bus *bus = test->priv;
	unsigned int transfers;

	wn8030_test_attach(test);
	transfers = bus->transfers;
	/* Cross the removed community worker's actual 12-second deadline. */
	msleep(13000);
	KUNIT_EXPECT_EQ(test, bus->transfers, transfers);
}

static struct kunit_case wn8030_cases[] = {
	KUNIT_CASE(wn8030_transfer_errors),
	KUNIT_CASE(wn8030_led_commands),
	KUNIT_CASE(wn8030_attach_switch_and_no_touchpad),
	KUNIT_CASE(wn8030_initial_attached),
	KUNIT_CASE(wn8030_firmware_transport),
	KUNIT_CASE(wn8030_reports),
	KUNIT_CASE(wn8030_panel_order_and_first_frame),
	KUNIT_CASE(wn8030_system_sleep_no_i2c),
	KUNIT_CASE(wn8030_blank_attach_detach),
	KUNIT_CASE(wn8030_pm_callback_orders),
	KUNIT_CASE(wn8030_wakeup_only_asleep),
	KUNIT_CASE(wn8030_ready_timeout_no_reset),
	KUNIT_CASE(wn8030_pending_outputs_and_auth),
	KUNIT_CASE_SLOW(wn8030_no_periodic_power),
	{}
};
static struct kunit_suite wn8030_suite = {
	.name = "nanosic-wn8030",
	.init = wn8030_test_init,
	.test_cases = wn8030_cases,
};
kunit_test_suite(wn8030_suite);
