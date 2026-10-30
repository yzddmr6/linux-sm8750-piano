// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC8547A charge pump, without the proprietary charging PHY.
 * Register definitions and sequences derived from the Oplus GPL driver.
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */
#include <linux/bitfield.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include "sc8547a.h"

#define SC_BAT_OVP		0x00
#define SC_AC_OVP		0x02
#define SC_BUS_OVP		0x04
#define SC_BUS_OCP		0x05
#define SC_STATUS		0x06
#define SC_CONTROL		0x07
#define SC_MODE_WATCHDOG		0x09
#define SC_FAULT_STATUS		0x0e
#define SC_FAULT_FLAGS		0x0f
#define SC_ADC_CONTROL		0x11
#define SC_ADC_IBUS		0x13
#define SC_DEVICE_ID		0x36
#define SC_ID			0x67
#define SC_CHARGE_ENABLE	BIT(7)
#define SC_REG_RESET		BIT(6)
#define SC_SWITCHING		BIT(2)
#define SC_WATCHDOG_EXPIRED	BIT(3)
#define SC_MODE_2_TO_1		0x10
#define SC_WATCHDOG_1S		0x03
#define SC_WATCHDOG_MS		1000
#define SC_FAULT_MASK		GENMASK(7, 2)
#define SC_STATUS_FAULTS		(BIT(7) | BIT(6) | BIT(4) | BIT(3) | BIT(1))
#define SC_ADC_ENABLE		BIT(7)

struct sc8547a {
	struct device *dev;
	struct regmap *regmap;
	struct mutex lock;
	struct delayed_work watchdog_work;
	struct sc8547a_group *group;
	struct power_supply *psy;
	struct power_supply_desc desc;
	struct sc8547a_sample last;
	unsigned long sampled;
	unsigned long watchdog_deadline;
	u8 bat_ovp;
	u8 ac_ovp;
	u8 bus_ovp;
	u8 bus_ocp;
	u8 control_off;
	bool secondary;
	bool prepared;
	bool active;
	bool fault;
	bool shutdown;
};

/* Leave the hardware watchdog armed if stopping the pump cannot be confirmed. */
static int sc8547a_disable_locked(struct sc8547a *cp)
{
	unsigned int value;
	int ret, err;

	ret = regmap_write(cp->regmap, SC_CONTROL, cp->control_off);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_CONTROL, &value);
	if (!ret && (value & SC_CHARGE_ENABLE))
		ret = -EIO;
	cp->active = false;
	cp->prepared = false;
	cp->last.switching = false;
	if (ret)
		return ret;

	ret = regmap_write(cp->regmap, SC_MODE_WATCHDOG, SC_MODE_2_TO_1);
	err = regmap_write(cp->regmap, SC_ADC_CONTROL, 0);
	return ret ?: err;
}

static void sc8547a_fault_locked(struct sc8547a *cp, int error)
{
	cp->fault = true;
	sc8547a_disable_locked(cp);
	dev_err_ratelimited(cp->dev, "charge pump stopped: fault %d\n", error);
	if (cp->group)
		cp->group->fault(cp->group->data);
}

static void sc8547a_watchdog_work(struct work_struct *work)
{
	struct sc8547a *cp = container_of(to_delayed_work(work), struct sc8547a, watchdog_work);
	unsigned long now;

	mutex_lock(&cp->lock);
	now = jiffies;
	if (cp->active && time_after_eq(now, cp->watchdog_deadline))
		sc8547a_fault_locked(cp, -ETIMEDOUT);
	else if (cp->active)
		mod_delayed_work(system_highpri_wq, &cp->watchdog_work,
				 cp->watchdog_deadline - now);
	mutex_unlock(&cp->lock);
}

/* All flag reads belong here or to the IRQ handler, never to a register dump. */
static int sc8547a_read_locked(struct sc8547a *cp, struct sc8547a_sample *sample)
{
	unsigned int state, control, mode, status, flags, adc;
	u8 data[10];
	int ret;

	if (!cp->prepared || cp->fault || cp->shutdown)
		return -EIO;
	if (cp->active && time_after_eq(jiffies, cp->watchdog_deadline))
		return -ETIMEDOUT;
	ret = regmap_read(cp->regmap, SC_STATUS, &state);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_CONTROL, &control);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_MODE_WATCHDOG, &mode);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_FAULT_STATUS, &status);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_FAULT_FLAGS, &flags);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_ADC_CONTROL, &adc);
	if (ret)
		return ret;
	if ((state & SC_STATUS_FAULTS) || (mode & SC_WATCHDOG_EXPIRED) ||
	    (mode & BIT(7)) ||
	    (cp->active && (mode & GENMASK(2, 0)) != SC_WATCHDOG_1S) ||
	    ((status | flags) & (cp->active ? SC_FAULT_MASK : GENMASK(7, 3))) ||
	    !(adc & SC_ADC_ENABLE))
		return -EIO;
	if (cp->active && (!(control & SC_CHARGE_ENABLE) || !(state & SC_SWITCHING) ||
			   (state & BIT(5))))
		return -EIO;
	if (!cp->active && ((control & SC_CHARGE_ENABLE) || (state & SC_SWITCHING)))
		return -EIO;

	ret = regmap_bulk_read(cp->regmap, SC_ADC_IBUS, data, sizeof(data));
	if (ret)
		return ret;
	/* Reject invalid/sign bits, rather than turning an error into high current. */
	if ((data[0] | data[2] | data[8]) & 0xf0)
		return -ENODATA;
	sample->ibus_ma = ((data[0] << 8) | data[1]) * 1875 / 1000;
	sample->vbus_mv = ((data[2] << 8) | data[3]) * 375 / 100;
	sample->vbat_mv = ((data[8] << 8) | data[9]) * 125 / 100;
	sample->switching = state & SC_SWITCHING;
	if (!sample->vbus_mv || !sample->vbat_mv)
		return -ENODATA;
	cp->last = *sample;
	cp->sampled = jiffies;
	return 0;
}

int sc8547a_prepare(struct sc8547a *cp)
{
	/* sc8547_init_device(), sc8547_svooc_hw_setting(..., false). */
	const struct reg_sequence init[] = {
		{ SC_CONTROL, cp->control_off },
		{ SC_ADC_CONTROL, 0x00 },
		{ SC_BAT_OVP, cp->bat_ovp },
		{ SC_AC_OVP, cp->ac_ovp },
		{ SC_BUS_OVP, cp->bus_ovp },
		{ SC_BUS_OCP, 0x10 | cp->bus_ocp },
		{ SC_MODE_WATCHDOG, SC_MODE_2_TO_1 },
		{ 0x0d, 0x70 },
		{ 0x2b, 0x00 },
		{ 0x30, 0x7f },
		{ SC_ADC_CONTROL, SC_ADC_ENABLE },
	};
	/* The slave driver leaves BAT_OCP and the primary PHY controls at reset. */
	const struct reg_sequence primary_init[] = {
		{ 0x01, 0xbf },
		{ 0x10, 0x02 },
		{ 0x33, 0xd1 },
		{ 0x3a, 0x60 },
		{ 0x3b, 0x04 },
	};
	int ret;

	mutex_lock(&cp->lock);
	if (cp->shutdown) {
		ret = -ESHUTDOWN;
		goto out;
	}
	if (cp->active) {
		ret = -EBUSY;
		goto out;
	}
	ret = regmap_write(cp->regmap, SC_CONTROL, SC_REG_RESET);
	if (!ret)
		ret = regmap_multi_reg_write(cp->regmap, init, ARRAY_SIZE(init));
	if (!ret && cp->secondary)
		ret = regmap_write(cp->regmap, 0x3c, 0x40);
	if (!ret && !cp->secondary)
		ret = regmap_multi_reg_write(cp->regmap, primary_init, ARRAY_SIZE(primary_init));
	if (ret) {
		sc8547a_fault_locked(cp, ret);
	} else {
		cp->prepared = true;
		cp->fault = false;
	}
out:
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547a_prepare);

int sc8547a_start(struct sc8547a *cp)
{
	struct sc8547a_sample sample;
	int ret;

	mutex_lock(&cp->lock);
	if (cp->active) {
		ret = -EBUSY;
		goto fail;
	}
	ret = sc8547a_read_locked(cp, &sample);
	if (ret)
		goto fail;
	/* The policy checks initial entry; a parallel pump joins a running bus. */
	ret = regmap_write(cp->regmap, SC_MODE_WATCHDOG, SC_MODE_2_TO_1 | SC_WATCHDOG_1S);
	if (!ret)
		ret = regmap_write(cp->regmap, SC_CONTROL, cp->control_off | SC_CHARGE_ENABLE);
	if (ret)
		goto fail;
	cp->active = true;
	cp->watchdog_deadline = jiffies + msecs_to_jiffies(SC_WATCHDOG_MS);
	mod_delayed_work(system_highpri_wq, &cp->watchdog_work,
			 msecs_to_jiffies(SC_WATCHDOG_MS));
	mutex_unlock(&cp->lock);
	return 0;
fail:
	sc8547a_fault_locked(cp, ret);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547a_start);

int sc8547a_sample(struct sc8547a *cp, struct sc8547a_sample *sample)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547a_read_locked(cp, sample);
	if (!ret && cp->active && time_after_eq(jiffies, cp->watchdog_deadline))
		ret = -ETIMEDOUT;
	if (!ret && cp->active) {
		/* Never re-latch CHG_EN after a hardware shutdown or watchdog expiry. */
		ret = regmap_write(cp->regmap, SC_MODE_WATCHDOG,
				   SC_MODE_2_TO_1 | SC_WATCHDOG_1S);
		if (!ret) {
			cp->watchdog_deadline = jiffies + msecs_to_jiffies(SC_WATCHDOG_MS);
			mod_delayed_work(system_highpri_wq, &cp->watchdog_work,
					 msecs_to_jiffies(SC_WATCHDOG_MS));
		}
	}
	if (ret)
		sc8547a_fault_locked(cp, ret);
	mutex_unlock(&cp->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547a_sample);

int sc8547a_stop(struct sc8547a *cp)
{
	int ret;

	mutex_lock(&cp->lock);
	ret = sc8547a_disable_locked(cp);
	mutex_unlock(&cp->lock);
	cancel_delayed_work_sync(&cp->watchdog_work);
	/* No further bus access may keep a pump alive after an uncertain stop. */
	if (ret)
		msleep(SC_WATCHDOG_MS);
	return ret;
}
EXPORT_SYMBOL_GPL(sc8547a_stop);

static irqreturn_t sc8547a_irq(int irq, void *data)
{
	struct sc8547a *cp = data;
	unsigned int status, flags;
	int ret;

	mutex_lock(&cp->lock);
	/* An uncertain stop must be allowed to expire without any I2C traffic. */
	if (!cp->prepared)
		goto out;
	if (cp->active && time_after_eq(jiffies, cp->watchdog_deadline)) {
		sc8547a_fault_locked(cp, -ETIMEDOUT);
		goto out;
	}
	ret = regmap_read(cp->regmap, SC_FAULT_STATUS, &status);
	if (!ret)
		ret = regmap_read(cp->regmap, SC_FAULT_FLAGS, &flags);
	if (ret || ((status | flags) & (cp->active ? SC_FAULT_MASK : GENMASK(7, 3))))
		sc8547a_fault_locked(cp, ret ?: -EIO);
out:
	mutex_unlock(&cp->lock);
	return IRQ_HANDLED;
}

static int sc8547a_get_property(struct power_supply *psy, enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sc8547a *cp = power_supply_get_drvdata(psy);
	int ret = 0;

	/* Cached telemetry: sysfs must not consume flags or feed the watchdog. */
	mutex_lock(&cp->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = cp->active && cp->last.switching && !cp->fault &&
			      time_before(jiffies, cp->watchdog_deadline);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = cp->fault ? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
					 POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!cp->prepared || !cp->sampled ||
		    time_after(jiffies, cp->sampled + msecs_to_jiffies(SC_WATCHDOG_MS))) {
			ret = -ENODATA;
			break;
		}
		val->intval = (prop == POWER_SUPPLY_PROP_VOLTAGE_NOW ?
			       cp->last.vbus_mv : cp->last.ibus_ma) * 1000;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&cp->lock);
	return ret;
}

static const enum power_supply_property sc8547a_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int sc8547a_bind(struct device *dev, struct device *master, void *data)
{
	struct sc8547a_group *group = data;
	struct sc8547a *cp = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < SC8547A_PUMPS; i++) {
		if (group->nodes[i] != dev->of_node)
			continue;
		if (cp->secondary != (i == 1))
			return -EINVAL;
		mutex_lock(&cp->lock);
		cp->group = group;
		group->pumps[i] = cp;
		mutex_unlock(&cp->lock);
		return 0;
	}
	return -EINVAL;
}

static void sc8547a_unbind(struct device *dev, struct device *master, void *data)
{
	struct sc8547a_group *group = data;
	struct sc8547a *cp = dev_get_drvdata(dev);
	int i;

	sc8547a_stop(cp);
	mutex_lock(&cp->lock);
	cp->group = NULL;
	for (i = 0; i < SC8547A_PUMPS; i++)
		if (group->pumps[i] == cp)
			group->pumps[i] = NULL;
	mutex_unlock(&cp->lock);
}

static const struct component_ops sc8547a_component_ops = {
	.bind = sc8547a_bind,
	.unbind = sc8547a_unbind,
};

static int sc8547a_limit(struct device *dev, const char *name, u32 min, u32 max,
		       u32 step, u8 *reg)
{
	u32 value;
	int ret;

	ret = device_property_read_u32(dev, name, &value);
	if (ret)
		return ret;
	if (value < min || value > max || (value - min) % step)
		return -EINVAL;
	*reg = (value - min) / step;
	return 0;
}

/* Generic register dumps must neither consume flags nor service the watchdog. */
static bool sc8547a_precious_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config sc8547a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x3c,
	.cache_type = REGCACHE_NONE,
	.precious_reg = sc8547a_precious_reg,
};

static void sc8547a_cleanup(void *data)
{
	sc8547a_stop(data);
}

static int sc8547a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config config = {};
	struct sc8547a *cp;
	unsigned int id;
	int ret;

	cp = devm_kzalloc(dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	cp->dev = dev;
	mutex_init(&cp->lock);
	INIT_DELAYED_WORK(&cp->watchdog_work, sc8547a_watchdog_work);
	cp->secondary = device_property_read_bool(dev, "southchip,secondary");
	cp->control_off = cp->secondary ? 0x01 : 0x00;
	cp->regmap = devm_regmap_init_i2c(client, &sc8547a_regmap_config);
	if (IS_ERR(cp->regmap))
		return PTR_ERR(cp->regmap);
	i2c_set_clientdata(client, cp);
	ret = regmap_read(cp->regmap, SC_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read device id\n");
	if (id != SC_ID)
		return dev_err_probe(dev, -ENODEV, "unsupported device id %#x\n", id);

	ret = sc8547a_limit(dev, "southchip,battery-overvoltage-microvolt",
			   3500000, 5075000, 25000, &cp->bat_ovp);
	if (!ret)
		ret = sc8547a_limit(dev, "southchip,ac-overvoltage-microvolt",
				   11000000, 17000000, 1000000, &cp->ac_ovp);
	if (!ret)
		ret = sc8547a_limit(dev, "southchip,input-overvoltage-microvolt",
				   6000000, 12350000, 50000, &cp->bus_ovp);
	if (!ret)
		ret = sc8547a_limit(dev, "southchip,input-overcurrent-microamp",
				   1200000, 5700000, 300000, &cp->bus_ocp);
	if (ret)
		return dev_err_probe(dev, ret, "invalid protection thresholds\n");

	ret = sc8547a_stop(cp);
	if (ret)
		return dev_err_probe(dev, ret, "cannot disable charge pump\n");
	ret = devm_add_action_or_reset(dev, sc8547a_cleanup, cp);
	if (ret)
		return ret;
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL, sc8547a_irq,
						IRQF_ONESHOT, dev_name(dev), cp);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request interrupt\n");
	} else if (!cp->secondary) {
		return dev_err_probe(dev, -EINVAL, "primary pump needs an interrupt\n");
	}

	cp->desc.name = devm_kasprintf(dev, GFP_KERNEL, "sc8547a-%s", dev_name(dev));
	if (!cp->desc.name)
		return -ENOMEM;
	cp->desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	cp->desc.properties = sc8547a_properties;
	cp->desc.num_properties = ARRAY_SIZE(sc8547a_properties);
	cp->desc.get_property = sc8547a_get_property;
	config.drv_data = cp;
	config.fwnode = dev_fwnode(dev);
	cp->psy = devm_power_supply_register(dev, &cp->desc, &config);
	if (IS_ERR(cp->psy))
		return dev_err_probe(dev, PTR_ERR(cp->psy), "cannot register supply\n");
	return component_add(dev, &sc8547a_component_ops);
}

static void sc8547a_remove(struct i2c_client *client)
{
	component_del(&client->dev, &sc8547a_component_ops);
}

static void sc8547a_shutdown(struct i2c_client *client)
{
	struct sc8547a *cp = i2c_get_clientdata(client);

	mutex_lock(&cp->lock);
	cp->shutdown = true;
	mutex_unlock(&cp->lock);
	sc8547a_stop(cp);
}

static const struct of_device_id sc8547a_of_match[] = {
	{ .compatible = "southchip,sc8547a" },
	{}
};
MODULE_DEVICE_TABLE(of, sc8547a_of_match);

static struct i2c_driver sc8547a_driver = {
	.driver = {
		.name = "sc8547a",
		.of_match_table = sc8547a_of_match,
	},
	.probe = sc8547a_probe,
	.remove = sc8547a_remove,
	.shutdown = sc8547a_shutdown,
};
module_i2c_driver(sc8547a_driver);

MODULE_DESCRIPTION("Southchip SC8547A charge pump");
MODULE_LICENSE("GPL");
