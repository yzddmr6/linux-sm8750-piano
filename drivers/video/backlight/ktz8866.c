// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for the Kinetic KTZ8866/KTZ8868
 *
 * Copyright (C) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define DEFAULT_BRIGHTNESS 1500
#define MAX_BRIGHTNESS 2047
#define NUM_SINKS 6
#define REG_MAX 0x15

/* reg */
#define DEVICE_ID 0x01
#define KTZ8868_BL_CTRL 0x01	/* KTZ8868: global backlight on/off */
#define BL_CFG1 0x02
#define BL_CFG2 0x03
#define BL_BRT_LSB 0x04
#define BL_BRT_MSB 0x05
#define BL_EN 0x08
#define LCD_BIAS_CFG1 0x09
#define LCD_BIAS_CFG2 0x0A
#define LCD_BIAS_CFG3 0x0B
#define LCD_BOOST_CFG 0x0C
#define OUTP_CFG 0x0D
#define OUTN_CFG 0x0E
#define FLAG 0x0F
#define BL_OPTION1 0x10
#define BL_OPTION2 0x11
#define PWM2DIG_LSBs 0x12
#define PWM2DIG_MSBs 0x13
#define BL_DIMMING 0x14
#define PWM_RAMP_TIME 0x15

/* definition */
#define BL_EN_BIT BIT(6)
#define BL_EN_SINKS GENMASK(5, 0)
#define LCD_BIAS_EN 0x9F
#define PWM_HYST 0x5

/*
 * KTZ8868: the LED driver is not live until KTZ8868_BL_CTRL is written after
 * the sink enable + brightness (the sink bits alone leave the string dark),
 * and the LCD bias outputs are switched by the ENP/ENN pins rather than
 * LCD_BIAS_CFG1 bits. Values below follow the KTZ8868 reference sequence
 * (OVP 34V, linear dimming, 4.7uH/2.5A boost, 30mA/channel, sinks 1-3/5-6).
 */
#define KTZ8868_BL_CFG1_VAL 0xB3
#define KTZ8868_BL_CFG2_VAL 0xEB
#define KTZ8868_BL_OPTION2_VAL 0x76
#define KTZ8868_PWM_RAMP_TIME_VAL 0xF8
#define KTZ8868_BL_EN_VAL 0x77
#define KTZ8868_LCD_BOOST_CFG_VAL 0x28
#define KTZ8868_OUTP_CFG_VAL 0x1E	/* +6.0V */
#define KTZ8868_OUTN_CFG_VAL 0x1E	/* -6.0V */
#define KTZ8868_LCD_BIAS_CFG1_VAL 0x99	/* bias enable from ENP/ENN pins */

enum ktz8866_variant {
	KTZ8866,
	KTZ8868,
};

enum ktz8868_bias_output {
	KTZ8868_OUTP,
	KTZ8868_OUTN,
	KTZ8868_NUM_BIAS,
};

struct ktz8866 {
	struct i2c_client *client;
	struct regmap *regmap;
	bool led_on;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	/* Second chip driving the other LED strings of the same panel */
	struct ktz8866 *secondary;
	struct gpio_desc *bias_gpio[KTZ8868_NUM_BIAS];
	bool bias_on[KTZ8868_NUM_BIAS];
	enum ktz8866_variant variant;
	struct led_classdev led;
	unsigned int bl_en;
};

/* The datasheet register map has no 0x00, 0x06 or 0x07. */
static bool ktz8866_readable_reg(struct device *dev, unsigned int reg)
{
	return (reg >= DEVICE_ID && reg <= BL_BRT_MSB) || (reg >= BL_EN && reg <= REG_MAX);
}

/* Reading FLAG clears the fault bits it reports. */
static bool ktz8866_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == FLAG;
}

static const struct regmap_config ktz8866_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
	.readable_reg = ktz8866_readable_reg,
	.precious_reg = ktz8866_precious_reg,
};

static int ktz8866_write(struct ktz8866 *ktz, unsigned int reg,
			 unsigned int val)
{
	return regmap_write(ktz->regmap, reg, val);
}

static int ktz8866_update_bits(struct ktz8866 *ktz, unsigned int reg,
			       unsigned int mask, unsigned int val)
{
	return regmap_update_bits(ktz->regmap, reg, mask, val);
}

static void ktz8868_write_brightness(struct ktz8866 *ktz, unsigned int brightness)
{
	ktz8866_write(ktz, BL_BRT_LSB, brightness & 0x7);
	ktz8866_write(ktz, BL_BRT_MSB, (brightness >> 3) & 0xFF);
}

static int ktz8868_update_brightness(struct ktz8866 *ktz,
				     unsigned int brightness)
{
	if (brightness == 0) {
		if (ktz->led_on) {
			ktz8866_write(ktz, KTZ8868_BL_CTRL, 0);
			msleep(9);
		}
		ktz8868_write_brightness(ktz, 0);
		if (ktz->led_on) {
			ktz8866_write(ktz, BL_EN, 0);
			ktz->led_on = false;
		}
		return 0;
	}

	if (!ktz->led_on) {
		ktz8866_write(ktz, BL_CFG1, KTZ8868_BL_CFG1_VAL);
		ktz8866_write(ktz, BL_CFG2, KTZ8868_BL_CFG2_VAL);
		ktz8866_write(ktz, BL_OPTION2, KTZ8868_BL_OPTION2_VAL);
		ktz8866_write(ktz, PWM_RAMP_TIME, KTZ8868_PWM_RAMP_TIME_VAL);
		ktz8866_write(ktz, BL_EN, ktz->bl_en);
	}

	ktz8868_write_brightness(ktz, brightness);

	if (!ktz->led_on) {
		msleep(15);
		ktz8866_write(ktz, KTZ8868_BL_CTRL, 1);
		ktz->led_on = true;
	}

	return 0;
}

static int ktz8866_set_brightness(struct ktz8866 *ktz, unsigned int brightness)
{
	int ret;

	if (brightness == 0) {
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_BIT, 0);
		if (ret)
			return ret;
		ktz->led_on = false;
	}

	/* Set brightness */
	ret = ktz8866_write(ktz, BL_BRT_LSB, brightness & 0x7);
	if (ret)
		return ret;

	ret = ktz8866_write(ktz, BL_BRT_MSB, (brightness >> 3) & 0xFF);
	if (ret)
		return ret;

	if (!ktz->led_on && brightness > 0) {
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_BIT, BL_EN_BIT);
		if (ret)
			return ret;
		ktz->led_on = true;
	}

	return 0;
}

static int ktz8866_backlight_update_status(struct backlight_device *backlight_dev)
{
	struct ktz8866 *ktz = bl_get_data(backlight_dev);
	unsigned int brightness = backlight_get_brightness(backlight_dev);
	int ret;

	if (ktz->variant == KTZ8868)
		return ktz8868_update_brightness(ktz, brightness);

	ret = ktz8866_set_brightness(ktz, brightness);
	if (ret || !ktz->secondary)
		return ret;

	return ktz8866_set_brightness(ktz->secondary, brightness);
}

static const struct backlight_ops ktz8866_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ktz8866_backlight_update_status,
};

static int ktz8868_led_brightness_set(struct led_classdev *led,
				      enum led_brightness brightness)
{
	struct ktz8866 *ktz = container_of(led, struct ktz8866, led);

	return ktz8868_update_brightness(ktz, brightness);
}

/*
 * A KTZ8868 with a led child node exposes its LED string as an LED class
 * device, so that several chips lighting one panel can be combined into a
 * single backlight by led-backlight. led-sources selects the current sinks.
 */
static int ktz8868_register_led(struct ktz8866 *ktz,
				struct fwnode_handle *child)
{
	struct device *dev = &ktz->client->dev;
	struct led_init_data init_data = { .fwnode = child };
	u32 sources[NUM_SINKS];
	int count, i, ret;

	count = fwnode_property_count_u32(child, "led-sources");
	if (count < 1 || count > NUM_SINKS)
		return dev_err_probe(dev, -EINVAL, "invalid led-sources\n");

	ret = fwnode_property_read_u32_array(child, "led-sources", sources,
					     count);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read led-sources\n");

	ktz->bl_en = BL_EN_BIT;
	for (i = 0; i < count; i++) {
		if (sources[i] >= NUM_SINKS)
			return dev_err_probe(dev, -EINVAL,
					     "invalid LED sink %u\n", sources[i]);
		ktz->bl_en |= BIT(sources[i]);
	}

	ktz->led.max_brightness = MAX_BRIGHTNESS;
	ktz->led.brightness_set_blocking = ktz8868_led_brightness_set;
	ktz->led.flags = LED_CORE_SUSPENDRESUME;

	ret = devm_led_classdev_register_ext(dev, &ktz->led, &init_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register LED\n");

	return 0;
}

/*
 * Apply the configuration of node @np to @ktz. The current sinks are set
 * without touching BL_EN_BIT, so a backlight that is already on stays on.
 */
static int ktz8866_init_chip(struct ktz8866 *ktz, const struct device_node *np)
{
	unsigned int val = 0;
	int ret;

	if (!of_property_read_u32(np, "current-num-sinks", &val))
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_SINKS, BIT(val) - 1);
	else
		/* Enable all 6 current sinks if the number of current sinks isn't specified. */
		ret = ktz8866_update_bits(ktz, BL_EN, BL_EN_SINKS, BL_EN_SINKS);
	if (ret)
		return ret;

	if (!of_property_read_u32(np, "kinetic,current-ramp-delay-ms", &val)) {
		if (val <= 128)
			ret = ktz8866_write(ktz, BL_CFG2, BIT(7) | (ilog2(val) << 3) | PWM_HYST);
		else
			ret = ktz8866_write(ktz, BL_CFG2,
					    BIT(7) | ((5 + val / 64) << 3) | PWM_HYST);
		if (ret)
			return ret;
	}

	if (!of_property_read_u32(np, "kinetic,led-enable-ramp-delay-ms", &val)) {
		if (val == 0)
			ret = ktz8866_write(ktz, BL_DIMMING, 0);
		else {
			unsigned int ramp_off_time = ilog2(val) + 1;
			unsigned int ramp_on_time = ramp_off_time << 4;
			ret = ktz8866_write(ktz, BL_DIMMING, ramp_on_time | ramp_off_time);
		}
		if (ret)
			return ret;
	}

	if (of_property_read_bool(np, "kinetic,enable-lcd-bias"))
		return ktz8866_write(ktz, LCD_BIAS_CFG1, LCD_BIAS_EN);

	return 0;
}

/* The secondary chip takes the configuration of the primary's node. */
static int ktz8866_init(struct ktz8866 *ktz)
{
	const struct device_node *np = ktz->client->dev.of_node;
	int ret;

	ret = ktz8866_init_chip(ktz, np);
	if (ret || !ktz->secondary)
		return ret;

	return ktz8866_init_chip(ktz->secondary, np);
}

static int ktz8866_read_state(struct ktz8866 *ktz, unsigned int *brightness)
{
	unsigned int en, lsb, msb;
	int ret;

	ret = regmap_read(ktz->regmap, BL_EN, &en);
	if (ret)
		return ret;
	ret = regmap_read(ktz->regmap, BL_BRT_LSB, &lsb);
	if (ret)
		return ret;
	ret = regmap_read(ktz->regmap, BL_BRT_MSB, &msb);
	if (ret)
		return ret;

	ktz->led_on = en & BL_EN_BIT;
	*brightness = (msb << 3) | (lsb & 0x7);

	return 0;
}

/*
 * Keep the brightness the bootloader left on the panel instead of writing a
 * default. @sync is set when the chips have to be written to show the
 * returned brightness: the backlight is off, or the secondary disagrees.
 */
static int ktz8866_get_boot_brightness(struct ktz8866 *ktz, unsigned int *brightness,
				       bool *sync)
{
	unsigned int secondary;
	int ret;

	ret = ktz8866_read_state(ktz, brightness);
	if (ret)
		return ret;

	*sync = !ktz->led_on || !*brightness;
	if (*sync)
		*brightness = DEFAULT_BRIGHTNESS;

	if (!ktz->secondary)
		return 0;

	ret = ktz8866_read_state(ktz->secondary, &secondary);
	if (ret)
		return ret;

	if (ktz->secondary->led_on != ktz->led_on || secondary != *brightness)
		*sync = true;

	return 0;
}

static struct i2c_driver ktz8866_driver;

static int ktz8866_get_secondary(struct ktz8866 *ktz)
{
	struct device *dev = &ktz->client->dev;
	struct ktz8866 *secondary = NULL;
	struct i2c_client *client;
	struct device_node *np;
	int ret = 0;

	np = of_parse_phandle(dev->of_node, "kinetic,secondary-backlight", 0);
	if (!np)
		return 0;

	if (!of_property_read_bool(np, "kinetic,secondary")) {
		ret = dev_err_probe(dev, -EINVAL, "%pOF is not a secondary chip\n", np);
		goto out_node;
	}

	client = of_find_i2c_device_by_node(np);
	if (!client) {
		ret = dev_err_probe(dev, -EPROBE_DEFER, "secondary chip not registered\n");
		goto out_node;
	}

	device_lock(&client->dev);
	if (device_is_bound(&client->dev) && client->dev.driver == &ktz8866_driver.driver)
		secondary = i2c_get_clientdata(client);

	if (!secondary)
		ret = dev_err_probe(dev, -EPROBE_DEFER, "secondary chip not bound\n");
	else if (!device_link_add(dev, &client->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		ret = dev_err_probe(dev, -EINVAL, "failed to link the secondary chip\n");
	else
		ktz->secondary = secondary;
	device_unlock(&client->dev);

	put_device(&client->dev);
out_node:
	of_node_put(np);

	return ret;
}

/*
 * KTZ8868 power-up: HWEN, then the bias configuration over I2C. The bias
 * section is left to follow the ENP/ENN pins, which the outp/outn regulators
 * switch when the panel asks for its supplies.
 */
static int ktz8868_power_up(struct ktz8866 *ktz)
{
	int ret;

	if (!ktz->enable_gpio)
		return 0;

	ret = gpiod_direction_output(ktz->enable_gpio, 1);
	if (ret)
		return ret;
	usleep_range(1000, 1500);

	if (device_property_read_bool(&ktz->client->dev,
				      "kinetic,enable-lcd-bias")) {
		ktz8866_write(ktz, LCD_BOOST_CFG, KTZ8868_LCD_BOOST_CFG_VAL);
		ktz8866_write(ktz, OUTP_CFG, KTZ8868_OUTP_CFG_VAL);
		ktz8866_write(ktz, OUTN_CFG, KTZ8868_OUTN_CFG_VAL);
		ktz8866_write(ktz, LCD_BIAS_CFG1, KTZ8868_LCD_BIAS_CFG1_VAL);
	}

	return 0;
}

static int ktz8868_bias_enable(struct regulator_dev *rdev)
{
	struct ktz8866 *ktz = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	gpiod_set_value_cansleep(ktz->bias_gpio[id], 1);
	ktz->bias_on[id] = true;

	return 0;
}

static int ktz8868_bias_disable(struct regulator_dev *rdev)
{
	struct ktz8866 *ktz = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);

	gpiod_set_value_cansleep(ktz->bias_gpio[id], 0);
	ktz->bias_on[id] = false;

	return 0;
}

static int ktz8868_bias_is_enabled(struct regulator_dev *rdev)
{
	struct ktz8866 *ktz = rdev_get_drvdata(rdev);

	return ktz->bias_on[rdev_get_id(rdev)];
}

static const struct regulator_ops ktz8868_bias_ops = {
	.enable = ktz8868_bias_enable,
	.disable = ktz8868_bias_disable,
	.is_enabled = ktz8868_bias_is_enabled,
};

static const struct regulator_desc ktz8868_bias_desc[KTZ8868_NUM_BIAS] = {
	[KTZ8868_OUTP] = {
		.name = "outp",
		.of_match = "outp",
		.id = KTZ8868_OUTP,
		.ops = &ktz8868_bias_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[KTZ8868_OUTN] = {
		.name = "outn",
		.of_match = "outn",
		.id = KTZ8868_OUTN,
		.ops = &ktz8868_bias_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

/*
 * Register the LCD bias outputs that have an outp/outn child node. Each is
 * switched by its own enable pin; the level found at probe is kept, so a
 * panel the boot loader lit keeps its bias until the panel driver takes the
 * regulators over.
 */
static int ktz8868_register_bias(struct ktz8866 *ktz)
{
	struct device *dev = &ktz->client->dev;
	struct regulator_config config = {
		.dev = dev,
		.driver_data = ktz,
	};
	int i, ret;

	for (i = 0; i < KTZ8868_NUM_BIAS; i++) {
		const struct regulator_desc *desc = &ktz8868_bias_desc[i];
		struct fwnode_handle *child __free(fwnode_handle) =
			device_get_named_child_node(dev, desc->of_match);
		struct regulator_dev *rdev;
		struct gpio_desc *gpio;

		if (!child)
			continue;

		gpio = devm_fwnode_gpiod_get(dev, child, "enable", GPIOD_ASIS,
					     desc->name);
		if (IS_ERR(gpio))
			return dev_err_probe(dev, PTR_ERR(gpio),
					     "failed to get %s enable GPIO\n",
					     desc->name);

		ret = gpiod_get_value_cansleep(gpio);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to read %s enable GPIO\n",
					     desc->name);
		ktz->bias_on[i] = ret;

		ret = gpiod_direction_output(gpio, ktz->bias_on[i]);
		if (ret)
			return dev_err_probe(dev, ret, "failed to drive %s enable GPIO\n",
					     desc->name);
		ktz->bias_gpio[i] = gpio;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s regulator\n",
					     desc->name);
	}

	return 0;
}

static int ktz8866_probe(struct i2c_client *client)
{
	struct fwnode_handle *led_node __free(fwnode_handle) = NULL;
	struct backlight_properties props;
	unsigned int brightness;
	struct ktz8866 *ktz;
	bool sync;
	int ret = 0;

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;
	ktz->variant = (uintptr_t)i2c_get_match_data(client);
	ktz->regmap = devm_regmap_init_i2c(client, &ktz8866_regmap_config);
	if (IS_ERR(ktz->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(ktz->regmap), "failed to init regmap\n");

	/* A secondary chip is only programmed through its primary. */
	if (of_property_read_bool(client->dev.of_node, "kinetic,secondary")) {
		i2c_set_clientdata(client, ktz);
		return 0;
	}

	ret = ktz8866_get_secondary(ktz);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(&client->dev, "vddpos");
	if (ret)
		return dev_err_probe(&client->dev, ret, "get regulator vddpos failed\n");
	ret = devm_regulator_get_enable(&client->dev, "vddneg");
	if (ret)
		return dev_err_probe(&client->dev, ret, "get regulator vddneg failed\n");

	ktz->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable",
						   ktz->variant == KTZ8868 ?
						   GPIOD_ASIS : GPIOD_OUT_HIGH);
	if (IS_ERR(ktz->enable_gpio))
		return PTR_ERR(ktz->enable_gpio);

	if (ktz->variant == KTZ8868) {
		ktz->bl_en = KTZ8868_BL_EN_VAL;
		led_node = device_get_named_child_node(&client->dev, "led");

		ret = ktz8868_power_up(ktz);
		if (ret)
			return dev_err_probe(&client->dev, ret, "failed to power up\n");

		ret = ktz8868_register_bias(ktz);
		if (ret)
			return ret;

		brightness = DEFAULT_BRIGHTNESS;
		of_property_read_u32(client->dev.of_node, "default-brightness", &brightness);
		sync = true;
	} else {
		/* HWEN must be high for at least 150 us before the first I2C command. */
		usleep_range(150, 200);

		ret = ktz8866_get_boot_brightness(ktz, &brightness, &sync);
		if (ret)
			return dev_err_probe(&client->dev, ret, "failed to read backlight state\n");

		ret = ktz8866_init(ktz);
		if (ret)
			return dev_err_probe(&client->dev, ret, "failed to initialize backlight\n");
	}

	i2c_set_clientdata(client, ktz);
	if (led_node)
		return ktz8868_register_led(ktz, led_node);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	props.brightness = brightness;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	ktz->backlight = devm_backlight_device_register(&client->dev, "ktz8866-backlight",
					&client->dev, ktz, &ktz8866_backlight_ops, &props);
	if (IS_ERR(ktz->backlight))
		return dev_err_probe(&client->dev, PTR_ERR(ktz->backlight),
				"failed to register backlight device\n");

	i2c_set_clientdata(client, ktz);
	if (!sync)
		return 0;

	return backlight_update_status(ktz->backlight);
}

static void ktz8866_remove(struct i2c_client *client)
{
	struct ktz8866 *ktz = i2c_get_clientdata(client);

	/* The LED core switches a registered LED off when it is unregistered. */
	if (!ktz->backlight)
		return;

	ktz->backlight->props.brightness = 0;
	backlight_update_status(ktz->backlight);
}

static const struct i2c_device_id ktz8866_ids[] = {
	{ .name = "ktz8866", .driver_data = KTZ8866 },
	{ .name = "ktz8868", .driver_data = KTZ8868 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktz8866_ids);

static const struct of_device_id ktz8866_match_table[] = {
	{ .compatible = "kinetic,ktz8866", .data = (void *)KTZ8866 },
	{ .compatible = "kinetic,ktz8868", .data = (void *)KTZ8868 },
	{},
};
MODULE_DEVICE_TABLE(of, ktz8866_match_table);

static struct i2c_driver ktz8866_driver = {
	.driver = {
		.name = "ktz8866",
		.of_match_table = ktz8866_match_table,
	},
	.probe = ktz8866_probe,
	.remove = ktz8866_remove,
	.id_table = ktz8866_ids,
};

module_i2c_driver(ktz8866_driver);

#if IS_ENABLED(CONFIG_BACKLIGHT_KTZ8866_KUNIT_TEST)
#include "ktz8866-test.c"
#endif

MODULE_DESCRIPTION("Kinetic KTZ8866/KTZ8868 Backlight Driver");
MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_LICENSE("GPL");
