// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for the Kinetic KTZ8866
 *
 * Copyright (C) 2022, 2023 Jianhua Lu <lujianhua000@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define DEFAULT_BRIGHTNESS 1500
#define MAX_BRIGHTNESS 2047
#define REG_MAX 0x15

/* reg */
#define DEVICE_ID 0x01
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

struct ktz8866 {
	struct i2c_client *client;
	struct regmap *regmap;
	bool led_on;
	struct gpio_desc *enable_gpio;
	struct backlight_device *backlight;
	/* Second chip driving the other LED strings of the same panel */
	struct ktz8866 *secondary;
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

/*
 * Read the LED fade time the chip will use when BL_EN is cleared. The ramp
 * may have been programmed by the bootloader, so the register is the only
 * source of truth. A mirrored pair fades at its own pace, so the slower
 * chip decides how long the panel stays lit.
 */
static int ktz8866_get_off_ramp(struct ktz8866 *ktz, unsigned int *ramp)
{
	unsigned int dimming;
	int ret;

	ret = regmap_read(ktz->regmap, BL_DIMMING, &dimming);
	if (ret)
		return ret;
	*ramp = dimming & 0xf;

	if (!ktz->secondary)
		return 0;

	ret = regmap_read(ktz->secondary->regmap, BL_DIMMING, &dimming);
	if (ret)
		return ret;
	*ramp = max(*ramp, dimming & 0xf);

	return 0;
}

/* Code 0 means 512 us, then codes 1..15 mean 1..16384 ms. */
static unsigned int ktz8866_off_ramp_delay(unsigned int ramp)
{
	return ramp ? 1000U << (ramp - 1) : 512;
}

static int ktz8866_backlight_update_status(struct backlight_device *backlight_dev)
{
	struct ktz8866 *ktz = bl_get_data(backlight_dev);
	unsigned int brightness = backlight_get_brightness(backlight_dev);
	unsigned int ramp = 0;
	bool fading = false;
	int ret;

	if (!brightness) {
		fading = ktz->led_on ||
			 (ktz->secondary && ktz->secondary->led_on);
		if (fading) {
			ret = ktz8866_get_off_ramp(ktz, &ramp);
			if (ret)
				return ret;
		}
	}

	ret = ktz8866_set_brightness(ktz, brightness);
	if (ret)
		return ret;
	if (ktz->secondary) {
		ret = ktz8866_set_brightness(ktz->secondary, brightness);
		if (ret)
			return ret;
	}

	/*
	 * Clearing BL_EN starts the programmed fade; it does not make the
	 * panel dark immediately. Keep the display pipeline intact until
	 * both chips have finished, before drm_panel_disable() returns.
	 */
	if (fading)
		fsleep(ktz8866_off_ramp_delay(ramp));

	return 0;
}

static const struct backlight_ops ktz8866_backlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = ktz8866_backlight_update_status,
};

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

static int ktz8866_probe(struct i2c_client *client)
{
	struct backlight_properties props;
	unsigned int brightness;
	struct ktz8866 *ktz;
	bool sync;
	int ret = 0;

	ktz = devm_kzalloc(&client->dev, sizeof(*ktz), GFP_KERNEL);
	if (!ktz)
		return -ENOMEM;

	ktz->client = client;
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

	/*
	 * HWEN low resets every register, including the LCD bias the panel may
	 * be running from, so it is never driven low here.
	 */
	ktz->enable_gpio = devm_gpiod_get_optional(&client->dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ktz->enable_gpio))
		return PTR_ERR(ktz->enable_gpio);

	/* HWEN must be high for at least 150 us before the first I2C command. */
	usleep_range(150, 200);

	ret = ktz8866_get_boot_brightness(ktz, &brightness, &sync);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to read backlight state\n");

	ret = ktz8866_init(ktz);
	if (ret)
		return dev_err_probe(&client->dev, ret, "failed to initialize backlight\n");

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

	if (!ktz->backlight)
		return;

	ktz->backlight->props.brightness = 0;
	backlight_update_status(ktz->backlight);
}

static const struct i2c_device_id ktz8866_ids[] = {
	{ .name = "ktz8866" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktz8866_ids);

static const struct of_device_id ktz8866_match_table[] = {
	{
		.compatible = "kinetic,ktz8866",
	},
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

MODULE_DESCRIPTION("Kinetic KTZ8866 Backlight Driver");
MODULE_AUTHOR("Jianhua Lu <lujianhua000@gmail.com>");
MODULE_LICENSE("GPL");
