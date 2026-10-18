// SPDX-License-Identifier: GPL-2.0-only
/*
 * GPIO and power control for Piano's secure-world Goodix sensor.
 *
 * Sensor transactions use QTEE, not AP-visible MMIO. UIO only supplies the
 * interrupt notification and brackets sensor power/reset with open/close.
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/uio_driver.h>

struct piano_fp {
	struct uio_info info;
	struct regulator *vdd;
	struct gpio_desc *reset;
	struct mutex lock;
	spinlock_t irq_lock;
	unsigned long irq_flags;
	bool opened;
	bool removing;
};

#define PIANO_FP_IRQ_DISABLED 0

static int piano_fp_irqcontrol(struct uio_info *info, s32 enable)
{
	struct piano_fp *fp = info->priv;
	unsigned long flags;

	spin_lock_irqsave(&fp->irq_lock, flags);
	if (enable && (!READ_ONCE(fp->opened) || READ_ONCE(fp->removing))) {
		spin_unlock_irqrestore(&fp->irq_lock, flags);
		return -ENODEV;
	}
	if (enable) {
		if (__test_and_clear_bit(PIANO_FP_IRQ_DISABLED, &fp->irq_flags))
			enable_irq(info->irq);
	} else {
		if (!__test_and_set_bit(PIANO_FP_IRQ_DISABLED, &fp->irq_flags))
			disable_irq_nosync(info->irq);
	}
	spin_unlock_irqrestore(&fp->irq_lock, flags);
	return 0;
}

static irqreturn_t piano_fp_irq(int irq, struct uio_info *info)
{
	struct piano_fp *fp = info->priv;

	/* Userspace acknowledges the sensor through QTEE before reenabling. */
	if (!test_and_set_bit(PIANO_FP_IRQ_DISABLED, &fp->irq_flags))
		disable_irq_nosync(irq);
	return IRQ_HANDLED;
}

static int piano_fp_open(struct uio_info *info, struct inode *inode)
{
	struct piano_fp *fp = info->priv;
	int ret;

	mutex_lock(&fp->lock);
	if (fp->opened || fp->removing) {
		ret = fp->removing ? -ENODEV : -EBUSY;
		goto unlock;
	}
	ret = regulator_enable(fp->vdd);
	if (ret)
		goto unlock;
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(fp->reset, 0);
	usleep_range(10000, 11000);
	WRITE_ONCE(fp->opened, true);
	piano_fp_irqcontrol(info, 1);
unlock:
	mutex_unlock(&fp->lock);
	return ret;
}

static void piano_fp_power_off(void *data)
{
	struct piano_fp *fp = data;

	mutex_lock(&fp->lock);
	if (fp->opened) {
		piano_fp_irqcontrol(&fp->info, 0);
		gpiod_set_value_cansleep(fp->reset, 1);
		regulator_disable(fp->vdd);
		WRITE_ONCE(fp->opened, false);
	}
	mutex_unlock(&fp->lock);
}

static int piano_fp_release(struct uio_info *info, struct inode *inode)
{
	piano_fp_power_off(info->priv);
	return 0;
}

static int piano_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_desc *irq_gpio;
	struct piano_fp *fp;
	int ret;

	fp = devm_kzalloc(dev, sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;
	mutex_init(&fp->lock);
	spin_lock_init(&fp->irq_lock);
	__set_bit(PIANO_FP_IRQ_DISABLED, &fp->irq_flags);
	fp->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(fp->vdd))
		return dev_err_probe(dev, PTR_ERR(fp->vdd), "missing sensor supply\n");
	fp->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fp->reset))
		return dev_err_probe(dev, PTR_ERR(fp->reset), "missing reset GPIO\n");
	irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(irq_gpio))
		return dev_err_probe(dev, PTR_ERR(irq_gpio), "missing interrupt GPIO\n");
	fp->info.irq = gpiod_to_irq(irq_gpio);
	if (fp->info.irq < 0)
		return fp->info.irq;
	fp->info.name = "piano-goodix";
	fp->info.version = "1";
	fp->info.irq_flags = IRQF_TRIGGER_RISING | IRQF_NO_AUTOEN;
	fp->info.handler = piano_fp_irq;
	fp->info.irqcontrol = piano_fp_irqcontrol;
	fp->info.open = piano_fp_open;
	fp->info.release = piano_fp_release;
	fp->info.priv = fp;
	platform_set_drvdata(pdev, fp);
	ret = devm_add_action_or_reset(dev, piano_fp_power_off, fp);
	if (ret)
		return ret;
	return devm_uio_register_device(dev, &fp->info);
}

static void piano_fp_remove(struct platform_device *pdev)
{
	struct piano_fp *fp = platform_get_drvdata(pdev);

	mutex_lock(&fp->lock);
	WRITE_ONCE(fp->removing, true);
	mutex_unlock(&fp->lock);
	piano_fp_power_off(fp);
}

static const struct of_device_id piano_fp_match[] = {
	{ .compatible = "xiaomi,piano-goodix-control" },
	{}
};
MODULE_DEVICE_TABLE(of, piano_fp_match);

static struct platform_driver piano_fp_driver = {
	.probe = piano_fp_probe,
	.remove = piano_fp_remove,
	.driver = {
		.name = "piano-fingerprint",
		.of_match_table = piano_fp_match,
	},
};
module_platform_driver(piano_fp_driver);

MODULE_DESCRIPTION("Piano Goodix secure sensor GPIO and power control");
MODULE_AUTHOR("slwyts <slwyts@foxmail.com>");
MODULE_LICENSE("GPL");
