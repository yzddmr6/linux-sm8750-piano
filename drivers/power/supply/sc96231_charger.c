// SPDX-License-Identifier: GPL-2.0-only
/*
 * Southchip SC96231 pen transmitter.
 *
 * Register protocol and sequencing recovered from the Piano HyperOS 4
 * sc96231_tx and mca_wireless_revchg drivers, checked against a live charge
 * cycle. Firmware remains on the chip; this driver does not program MTP.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#define SC_FW_VERSION		0x0004
#define SC_COMMAND		0x0100
#define SC_IRQ_STATUS		0x0108
#define SC_IRQ_CLEAR		0x010c
#define SC_I2C_CHECK		0x01cc
#define SC_PEN_SOC		0x01d0
#define SC_PEN_MAC		0x01dc /* six bytes starting at offset 2 */
#define SC_TX_DISABLE		BIT(18)
#define SC_TX_ENABLE		BIT(19)
#define SC_IRQ_EPT		BIT(15)
#define SC_IRQ_RX_FOUND		BIT(16)
#define SC_IRQ_FOD		BIT(17)
#define SC_IRQ_CEP_TIMEOUT	BIT(19)
#define SC_IRQ_RX_DETECTED	BIT(14)
#define SC_IRQ_CONFIG		BIT(24)
#define SC_IRQ_BLE_CONNECT	BIT(25)
#define SC_IRQ_SOC		BIT(27)
#define SC_IRQ_FAULTS		(SC_IRQ_EPT | SC_IRQ_FOD | SC_IRQ_CEP_TIMEOUT)

struct sc96231 {
	struct device *dev;
	struct regmap *map;
	struct regulator *vin;
	struct gpio_desc *tx_enable;
	struct gpio_desc *lp_enable;
	struct gpio_descs *hall;
	struct input_dev *dock;
	struct power_supply *battery;
	struct delayed_work work;
	struct mutex lock;
	int *hall_irqs;
	int irq;
	bool docked;
	bool enabled;
	bool present;
	bool full;
	bool fault;
	bool stopping;
	bool soc_valid;
	int capacity;
	unsigned long started;
	unsigned long full_since;
	/* The state that was reported to the power supply class. */
	bool reported_present;
	bool reported_full;
	bool reported_fault;
	bool reported_enabled;
	bool reported_soc_valid;
	int reported_capacity;
	/* The state that was reported to the input device. */
	bool pen_inserted;
	char address[18];
};

static int sc96231_read(struct sc96231 *sc, u16 reg, void *data, size_t size)
{
	return regmap_raw_read(sc->map, reg, data, size);
}

static int sc96231_write_u32(struct sc96231 *sc, u16 reg, u32 value)
{
	__le32 data = cpu_to_le32(value);

	return regmap_raw_write(sc->map, reg, &data, sizeof(data));
}

static void sc96231_stop(struct sc96231 *sc)
{
	int ret;

	if (!sc->enabled)
		return;
	ret = sc96231_write_u32(sc, SC_COMMAND, SC_TX_DISABLE);
	if (ret)
		dev_dbg(sc->dev, "TX disable command failed: %d\n", ret);
	/* Even an unresponsive transmitter must lose its enable and supply. */
	gpiod_set_value_cansleep(sc->tx_enable, 0);
	ret = regulator_disable(sc->vin);
	if (ret)
		dev_warn(sc->dev, "pen supply disable failed: %d\n", ret);
	else
		sc->enabled = false;
}

static int sc96231_start(struct sc96231 *sc)
{
	u8 check[4];
	__le32 version, pending;
	int ret;

	ret = regulator_enable(sc->vin);
	if (ret)
		return ret == -ENODEV ? -EPROBE_DEFER : ret;
	sc->enabled = true;
	gpiod_set_value_cansleep(sc->tx_enable, 1);
	msleep(20);

	/* Stock uses the last byte of this aligned word as an I2C handshake. */
	ret = sc96231_read(sc, SC_I2C_CHECK, check, sizeof(check));
	if (ret)
		goto fail;
	check[3] = 0x55;
	ret = regmap_raw_write(sc->map, SC_I2C_CHECK, check, sizeof(check));
	if (ret)
		goto fail;
	msleep(20);
	ret = sc96231_read(sc, SC_I2C_CHECK, check, sizeof(check));
	if (ret)
		goto fail;
	if (check[3] != 0x55) {
		ret = -EIO;
		goto fail;
	}
	ret = sc96231_read(sc, SC_FW_VERSION, &version, sizeof(version));
	if (ret)
		goto fail;
	if (!version || version == cpu_to_le32(U32_MAX)) {
		ret = -ENODATA;
		goto fail;
	}
	ret = sc96231_read(sc, SC_IRQ_STATUS, &pending, sizeof(pending));
	if (ret)
		goto fail;
	if (pending) {
		ret = sc96231_write_u32(sc, SC_IRQ_CLEAR, le32_to_cpu(pending));
		if (ret)
			goto fail;
	}
	ret = sc96231_write_u32(sc, SC_COMMAND, SC_TX_ENABLE);
	if (ret)
		goto fail;
	sc->started = jiffies;
	dev_info(sc->dev, "pen transmitter enabled, firmware %#x\n",
		 le32_to_cpu(version));
	return 0;
fail:
	sc96231_stop(sc);
	return ret;
}

static int sc96231_docked(struct sc96231 *sc)
{
	unsigned int i;
	int value, attached = 0;

	for (i = 0; i < sc->hall->ndescs; i++) {
		value = gpiod_get_value_cansleep(sc->hall->desc[i]);
		if (value < 0)
			return value;
		attached |= value;
	}
	return attached;
}

static int sc96231_update_pen(struct sc96231 *sc, u32 irq)
{
	u8 raw[8], soc[4];
	int ret, i;
	bool nonzero = false, not_ff = false;

	if (!sc->present && !(irq & (SC_IRQ_CONFIG | SC_IRQ_BLE_CONNECT | SC_IRQ_SOC)))
		return 0;
	ret = sc96231_read(sc, SC_PEN_MAC, raw, sizeof(raw));
	if (ret)
		return ret;
	for (i = 2; i < 8; i++) {
		nonzero |= raw[i] != 0;
		not_ff |= raw[i] != 0xff;
	}
	if (nonzero && not_ff) {
		/* The chip stores the Bluetooth address least-significant byte
		 * first; stock publishes the resulting u64 with "%llx".
		 */
		snprintf(sc->address, sizeof(sc->address), "%pMR", raw + 2);
		sc->present = true;
	}

	ret = sc96231_read(sc, SC_PEN_SOC, soc, sizeof(soc));
	if (ret)
		return ret;
	/* Zero before a charge-status interrupt means no report yet. */
	if ((soc[0] || (irq & SC_IRQ_SOC)) && (soc[0] & 0x7f) <= 100) {
		sc->capacity = soc[0] & 0x7f;
		sc->soc_valid = true;
		sc->full = soc[0] & BIT(7);
		/* Stock stops after 18 consecutive ten-second reports at 100%,
		 * even if the receiver never raises its explicit full flag.
		 */
		if (sc->capacity == 100) {
			if (!sc->full_since)
				sc->full_since = jiffies;
			else if (time_after_eq(jiffies, sc->full_since + 180 * HZ))
				sc->full = true;
		} else {
			sc->full_since = 0;
		}
	}
	return 0;
}

/* The power supply class is only told about a state that changed. The
 * transmitter reports every second while it runs, and the class would
 * otherwise wake up its readers once a second for nothing.
 */
static bool sc96231_state_changed(struct sc96231 *sc)
{
	return sc->present != sc->reported_present ||
	       sc->full != sc->reported_full ||
	       sc->fault != sc->reported_fault ||
	       sc->enabled != sc->reported_enabled ||
	       sc->soc_valid != sc->reported_soc_valid ||
	       sc->capacity != sc->reported_capacity;
}

static void sc96231_notify(struct sc96231 *sc)
{
	if (!sc96231_state_changed(sc))
		return;
	sc->reported_present = sc->present;
	sc->reported_full = sc->full;
	sc->reported_fault = sc->fault;
	sc->reported_enabled = sc->enabled;
	sc->reported_soc_valid = sc->soc_valid;
	sc->reported_capacity = sc->capacity;
	power_supply_changed(sc->battery);
}

/* The pen is reported as inserted once the transmitter has seen it. The
 * magnet of the keyboard can pull the Hall inputs of the dock, and that
 * must not be read as a pen in the dock.
 */
static void sc96231_report_pen(struct sc96231 *sc, bool inserted)
{
	if (sc->pen_inserted == inserted)
		return;
	sc->pen_inserted = inserted;
	input_report_switch(sc->dock, SW_PEN_INSERTED, inserted);
	input_sync(sc->dock);
}

static void sc96231_work(struct work_struct *work)
{
	struct sc96231 *sc = container_of(to_delayed_work(work),
					struct sc96231, work);
	__le32 raw_irq;
	u32 irq = 0;
	int attached, ret;

	mutex_lock(&sc->lock);
	if (sc->stopping)
		goto out;
	attached = sc96231_docked(sc);
	if (attached < 0)
		goto poll;
	if (attached != sc->docked) {
		sc->docked = attached;
		if (!attached) {
			sc96231_stop(sc);
			sc->present = false;
			sc->full = false;
			sc->fault = false;
			sc->soc_valid = false;
			sc->full_since = 0;
			sc96231_report_pen(sc, false);
			/* Retain the discovered address for reconnect after removal. */
			goto notify;
		}
	}
	if (!sc->docked || sc->fault || sc->full) {
		sc96231_stop(sc);
		goto notify;
	}
	if (!sc->enabled) {
		ret = sc96231_start(sc);
		if (ret == -EPROBE_DEFER)
			goto poll;
		if (ret) {
			dev_err(sc->dev, "pen transmitter start failed: %d\n", ret);
			sc->fault = true;
			goto notify;
		}
		if (sc->full) {
			/* A full pen is checked now and then: a pen that was
			 * used and put back has to charge again.
			 */
			ret = sc96231_update_pen(sc, 0);
			if (ret)
				goto failed;
			if (sc->full) {
				sc96231_stop(sc);
				goto notify;
			}
		}
		goto poll;
	}

	ret = sc96231_read(sc, SC_IRQ_STATUS, &raw_irq, sizeof(raw_irq));
	if (ret)
		goto failed;
	irq = le32_to_cpu(raw_irq);
	if (irq) {
		ret = sc96231_write_u32(sc, SC_IRQ_CLEAR, irq);
		if (ret)
			goto failed;
	}
	if (irq & (SC_IRQ_RX_FOUND | SC_IRQ_RX_DETECTED | SC_IRQ_CONFIG))
		sc->present = true;
	ret = sc96231_update_pen(sc, irq);
	if (ret)
		goto failed;
	if (sc->present)
		sc96231_report_pen(sc, true);
	if (sc->full) {
		sc96231_stop(sc);
		goto notify;
	}
	if (irq & SC_IRQ_FAULTS) {
		ret = -EREMOTEIO;
		goto failed;
	}
	/* Same 20-second no-receiver timeout as the original charging stack. */
	if (!sc->present && time_after(jiffies, sc->started + 20 * HZ)) {
		ret = -ETIMEDOUT;
		goto failed;
	}
	goto notify;
failed:
	dev_warn(sc->dev, "pen transmitter stopped: %d, IRQ %#x\n", ret, irq);
	sc->fault = true;
	sc96231_stop(sc);
notify:
	sc96231_notify(sc);
poll:
	if (!sc->stopping && !pm_suspend_in_progress() &&
	    (sc->enabled || (sc->docked && !sc->fault))) {
		unsigned long delay = HZ;

		/* A full pen is looked at now and then, so that a pen that
		 * was used and put back charges again.
		 */
		if (sc->full)
			delay = 10 * HZ;
		mod_delayed_work(system_freezable_wq, &sc->work, delay);
	}
out:
	mutex_unlock(&sc->lock);
}

static irqreturn_t sc96231_irq(int irq, void *data)
{
	struct sc96231 *sc = data;

	if (!READ_ONCE(sc->stopping) && !pm_suspend_in_progress())
		mod_delayed_work(system_freezable_wq, &sc->work, msecs_to_jiffies(20));
	return IRQ_HANDLED;
}

static int sc96231_get_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       union power_supply_propval *val)
{
	struct sc96231 *sc = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&sc->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sc->present;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (!sc->present)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else if (sc->full)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (sc->enabled && !sc->fault)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (!sc->soc_valid)
			ret = -ENODATA;
		else
			val->intval = sc->capacity;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "Wireless pen";
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		if (!sc->address[0])
			ret = -ENODATA;
		else
			val->strval = sc->address;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&sc->lock);
	return ret;
}

static const enum power_supply_property sc96231_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sc96231_supply = {
	.name = "sc96231-pen",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc96231_properties,
	.num_properties = ARRAY_SIZE(sc96231_properties),
	.get_property = sc96231_get_property,
};

static void sc96231_quiesce(void *data)
{
	struct sc96231 *sc = data;

	WRITE_ONCE(sc->stopping, true);
	cancel_delayed_work_sync(&sc->work);
	mutex_lock(&sc->lock);
	sc96231_stop(sc);
	mutex_unlock(&sc->lock);
}

static const struct regmap_config sc96231_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.cache_type = REGCACHE_NONE,
};

static int sc96231_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	struct sc96231 *sc;
	unsigned int i;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	sc->dev = dev;
	sc->irq = client->irq;
	i2c_set_clientdata(client, sc);
	mutex_init(&sc->lock);
	INIT_DELAYED_WORK(&sc->work, sc96231_work);
	sc->map = devm_regmap_init_i2c(client, &sc96231_regmap);
	if (IS_ERR(sc->map))
		return PTR_ERR(sc->map);
	sc->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(sc->vin))
		return dev_err_probe(dev, PTR_ERR(sc->vin), "pen power supply\n");
	sc->tx_enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(sc->tx_enable))
		return dev_err_probe(dev, PTR_ERR(sc->tx_enable), "TX enable GPIO\n");
	sc->lp_enable = devm_gpiod_get_optional(dev, "lp-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(sc->lp_enable))
		return PTR_ERR(sc->lp_enable);
	sc->hall = devm_gpiod_get_array(dev, "hall", GPIOD_IN);
	if (IS_ERR(sc->hall))
		return dev_err_probe(dev, PTR_ERR(sc->hall), "pen Hall GPIOs\n");
	sc->hall_irqs = devm_kcalloc(dev, sc->hall->ndescs,
				    sizeof(*sc->hall_irqs), GFP_KERNEL);
	if (!sc->hall_irqs)
		return -ENOMEM;
	sc->dock = devm_input_allocate_device(dev);
	if (!sc->dock)
		return -ENOMEM;
	sc->dock->name = "Xiaomi Piano Pen Dock";
	sc->dock->id.bustype = BUS_I2C;
	input_set_capability(sc->dock, EV_SW, SW_PEN_INSERTED);
	ret = input_register_device(sc->dock);
	if (ret)
		return ret;
	cfg.drv_data = sc;
	cfg.fwnode = dev_fwnode(dev);
	sc->battery = devm_power_supply_register(dev, &sc96231_supply, &cfg);
	if (IS_ERR(sc->battery))
		return PTR_ERR(sc->battery);
	ret = devm_add_action_or_reset(dev, sc96231_quiesce, sc);
	if (ret)
		return ret;
	/* IRQ resources are registered last, so devres frees them before work. */
	for (i = 0; i < sc->hall->ndescs; i++) {
		ret = gpiod_to_irq(sc->hall->desc[i]);
		if (ret < 0)
			return ret;
		sc->hall_irqs[i] = ret;
		ret = devm_request_threaded_irq(dev, ret, NULL, sc96231_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"sc96231-hall", sc);
		if (ret)
			return ret;
	}
	ret = devm_request_threaded_irq(dev, client->irq, NULL, sc96231_irq,
				       IRQF_ONESHOT, "sc96231", sc);
	if (ret)
		return ret;
	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
	}
	mod_delayed_work(system_freezable_wq, &sc->work, 0);
	return 0;
}

static void sc96231_shutdown(struct i2c_client *client)
{
	sc96231_quiesce(i2c_get_clientdata(client));
}

static int sc96231_suspend(struct device *dev)
{
	struct sc96231 *sc = dev_get_drvdata(dev);
	int ret;
	unsigned int i;

	/* The poll stops with the system: a timer that fires while the
	 * system sleeps wakes it up again. The resume re-arms the work.
	 */
	cancel_delayed_work_sync(&sc->work);

	if (!device_may_wakeup(dev))
		return 0;
	ret = enable_irq_wake(sc->irq);
	if (ret)
		return ret;
	for (i = 0; i < sc->hall->ndescs; i++) {
		ret = enable_irq_wake(sc->hall_irqs[i]);
		if (ret) {
			while (i--)
				disable_irq_wake(sc->hall_irqs[i]);
			disable_irq_wake(sc->irq);
			return ret;
		}
	}
	return 0;
}

static int sc96231_resume(struct device *dev)
{
	struct sc96231 *sc = dev_get_drvdata(dev);
	unsigned int i;

	if (device_may_wakeup(dev)) {
		for (i = 0; i < sc->hall->ndescs; i++)
			disable_irq_wake(sc->hall_irqs[i]);
		disable_irq_wake(sc->irq);
	}
	mod_delayed_work(system_freezable_wq, &sc->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(sc96231_pm_ops, sc96231_suspend, sc96231_resume);

static const struct of_device_id sc96231_of_match[] = {
	{ .compatible = "southchip,sc96231" },
	{}
};
MODULE_DEVICE_TABLE(of, sc96231_of_match);

static struct i2c_driver sc96231_driver = {
	.driver = {
		.name = "sc96231",
		.of_match_table = sc96231_of_match,
		.pm = pm_sleep_ptr(&sc96231_pm_ops),
	},
	.probe = sc96231_probe,
	.shutdown = sc96231_shutdown,
};
module_i2c_driver(sc96231_driver);

MODULE_DESCRIPTION("Southchip SC96231 wireless pen charger");
MODULE_AUTHOR("slwyts <slwyts@foxmail.com>");
MODULE_LICENSE("GPL");
