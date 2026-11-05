// SPDX-License-Identifier: GPL-2.0
// SPI driven IR LED device driver
//
// Copyright (c) 2016 Samsung Electronics Co., Ltd.
// Copyright (c) Andi Shyti <andi@etezian.org>

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>

#include <media/rc-core.h>

#define IR_SPI_DRIVER_NAME		"ir-spi"

#define IR_SPI_DEFAULT_FREQUENCY	38000
#define IR_SPI_BITS_PER_PULSE		16

/* The factory Piano transmitter consumes MSB-first native u32 words. */
#define IR_SPI_PIANO_WORD_BITS		32
#define IR_SPI_PIANO_PADDING_BYTES	1024
#define IR_SPI_PIANO_MAX_DURATIONS	1024
#define IR_SPI_PIANO_MAX_TX_BYTES		SZ_32K
#define IR_SPI_PIANO_MIN_CARRIER		30000
#define IR_SPI_PIANO_MAX_CARRIER		56000

struct ir_spi_variant {
	u32 sample_hz;
	u8 default_duty_cycle;
};

static const struct ir_spi_variant piano_ir_spi = {
	.sample_hz = 1920000,
	.default_duty_cycle = 40,
};

struct ir_spi_data {
	u32 freq;
	bool negated;
	u8 duty_cycle;
	const struct ir_spi_variant *variant;

	u16 pulse;
	u16 space;

	struct rc_dev *rc;
	struct spi_device *spi;
	struct regulator *regulator;
};

static int ir_spi_tx(struct rc_dev *dev, unsigned int *buffer, unsigned int count)
{
	int i;
	int ret;
	unsigned int len = 0;
	struct ir_spi_data *idata = dev->priv;
	struct spi_transfer xfer;
	u16 *tx_buf;

	/* convert the pulse/space signal to raw binary signal */
	for (i = 0; i < count; i++) {
		buffer[i] = DIV_ROUND_CLOSEST_ULL((u64)buffer[i] * idata->freq,
						  1000000);
		len += buffer[i];
	}

	tx_buf = kmalloc_objs(*tx_buf, len);
	if (!tx_buf)
		return -ENOMEM;

	len = 0;
	for (i = 0; i < count; i++) {
		int j;
		u16 val;

		/*
		 * The first value in buffer is a pulse, so that 0, 2, 4, ...
		 * contain a pulse duration. On the contrary, 1, 3, 5, ...
		 * contain a space duration.
		 */
		val = (i % 2) ? idata->space : idata->pulse;
		for (j = 0; j < buffer[i]; j++)
			tx_buf[len++] = val;
	}

	memset(&xfer, 0, sizeof(xfer));

	xfer.speed_hz = idata->freq * IR_SPI_BITS_PER_PULSE;
	xfer.len = len * sizeof(*tx_buf);
	xfer.tx_buf = tx_buf;

	ret = regulator_enable(idata->regulator);
	if (ret)
		goto err_free_tx_buf;

	ret = spi_sync_transfer(idata->spi, &xfer, 1);
	if (ret)
		dev_err(&idata->spi->dev, "unable to deliver the signal\n");

	regulator_disable(idata->regulator);

err_free_tx_buf:

	kfree(tx_buf);

	return ret ? ret : count;
}

static int ir_spi_tx_piano(struct rc_dev *dev, unsigned int *buffer,
			   unsigned int count)
{
	struct ir_spi_data *idata = dev->priv;
	struct spi_transfer xfer = {};
	u32 total_us = 0, end_us = 0, sample = 0;
	u32 samples, bytes, period, high, phase;
	u32 *tx_buf;
	unsigned int i;
	int ret;

	/* Match normal LIRC pulse/space bounds; never reinterpret negative data. */
	if (!count || count > IR_SPI_PIANO_MAX_DURATIONS || !(count & 1))
		return -EINVAL;

	for (i = 0; i < count; i++) {
		if (!buffer[i] || buffer[i] > IR_MAX_DURATION - total_us)
			return -EINVAL;
		total_us += buffer[i];
	}

	period = DIV_ROUND_CLOSEST(idata->variant->sample_hz, idata->freq);
	high = period * idata->duty_cycle / 100;
	if (!high || high >= period)
		return -EINVAL;

	samples = DIV_ROUND_CLOSEST_ULL((u64)total_us *
				      idata->variant->sample_hz, 1000000);
	/* Always finish with an all-zero word, even at an exact block boundary. */
	bytes = round_up(DIV_ROUND_UP(samples, IR_SPI_PIANO_WORD_BITS) *
			 sizeof(*tx_buf) + sizeof(*tx_buf),
			 IR_SPI_PIANO_PADDING_BYTES);
	/* GPI accepts one DMA segment. Keep diagnostic waves within 32 KiB;
	 * splitting transfers would insert an unmeasured gap in the waveform.
	 */
	if (bytes > IR_SPI_PIANO_MAX_TX_BYTES)
		return -EMSGSIZE;
	tx_buf = kzalloc(bytes, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		u32 end;

		end_us += buffer[i];
		end = DIV_ROUND_CLOSEST_ULL((u64)end_us *
					  idata->variant->sample_hz, 1000000);
		phase = 0;
		for (; sample < end; sample++) {
			if (!(i & 1) && phase < high)
				tx_buf[sample / IR_SPI_PIANO_WORD_BITS] |=
					BIT(31 - sample % IR_SPI_PIANO_WORD_BITS);
			if (++phase == period)
				phase = 0;
		}
	}

	/* No byte swap: GENI transmits each native 32-bit word MSB first. */
	xfer.tx_buf = tx_buf;
	xfer.len = bytes;
	xfer.bits_per_word = IR_SPI_PIANO_WORD_BITS;
	xfer.speed_hz = idata->variant->sample_hz;
	ret = spi_sync_transfer(idata->spi, &xfer, 1);
	if (ret)
		dev_err(&idata->spi->dev, "unable to deliver the signal: %d\n", ret);
	kfree(tx_buf);

	return ret ? ret : count;
}

static int ir_spi_set_tx_carrier(struct rc_dev *dev, u32 carrier)
{
	struct ir_spi_data *idata = dev->priv;

	if (!carrier)
		return -EINVAL;

	if (idata->variant) {
		u32 period;

		if (carrier < IR_SPI_PIANO_MIN_CARRIER ||
		    carrier > IR_SPI_PIANO_MAX_CARRIER)
			return -EINVAL;
		period = DIV_ROUND_CLOSEST(idata->variant->sample_hz, carrier);
		if (!(period * idata->duty_cycle / 100))
			return -EINVAL;
	} else if (carrier > idata->spi->max_speed_hz / IR_SPI_BITS_PER_PULSE) {
		return -EINVAL;
	}

	idata->freq = carrier;

	return 0;
}

static int ir_spi_set_duty_cycle(struct rc_dev *dev, u32 duty_cycle)
{
	struct ir_spi_data *idata = dev->priv;
	int bits = (duty_cycle * 15) / 100;

	if (idata->variant) {
		u32 period, high;

		if (!duty_cycle || duty_cycle >= 100)
			return -EINVAL;
		period = DIV_ROUND_CLOSEST(idata->variant->sample_hz, idata->freq);
		high = period * duty_cycle / 100;
		if (!high || high >= period)
			return -EINVAL;
		idata->duty_cycle = duty_cycle;
		return 0;
	}

	idata->pulse = GENMASK(bits, 0);

	if (idata->negated) {
		idata->pulse = ~idata->pulse;
		idata->space = 0xffff;
	} else {
		idata->space = 0;
	}

	return 0;
}

static int ir_spi_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	int ret;
	u8 dc;
	struct ir_spi_data *idata;

	idata = devm_kzalloc_obj(dev, *idata);
	if (!idata)
		return -ENOMEM;

	idata->variant = spi_get_device_match_data(spi);
	if (idata->variant) {
		if (spi->max_speed_hz < idata->variant->sample_hz ||
		    spi->mode != SPI_MODE_0)
			return -EINVAL;
		/* This board has no described software-controlled IR supply. */
		if (device_property_present(dev, "power-supply") ||
		    device_property_present(dev, "irda_regulator-supply") ||
		    device_property_present(dev, "led-active-low") ||
		    device_property_present(dev, "duty-cycle"))
			return -EINVAL;
	} else {
		idata->regulator = devm_regulator_get(dev, "irda_regulator");
		if (IS_ERR(idata->regulator))
			return PTR_ERR(idata->regulator);
	}

	idata->rc = devm_rc_allocate_device(&spi->dev, RC_DRIVER_IR_RAW_TX);
	if (!idata->rc)
		return -ENOMEM;

	idata->rc->tx_ir           = idata->variant ? ir_spi_tx_piano : ir_spi_tx;
	idata->rc->s_tx_carrier    = ir_spi_set_tx_carrier;
	idata->rc->s_tx_duty_cycle = ir_spi_set_duty_cycle;
	idata->rc->device_name	   = "IR SPI";
	idata->rc->driver_name     = IR_SPI_DRIVER_NAME;
	idata->rc->priv            = idata;
	idata->spi                 = spi;
	idata->freq = IR_SPI_DEFAULT_FREQUENCY;

	idata->negated = device_property_read_bool(dev, "led-active-low");
	if (idata->variant) {
		dc = idata->variant->default_duty_cycle;
	} else {
		ret = device_property_read_u8(dev, "duty-cycle", &dc);
		if (ret)
			dc = 50;
	}

	/*
	 * The Piano variant checks whether the requested duty can be sampled.
	 */
	ret = ir_spi_set_duty_cycle(idata->rc, dc);
	if (ret)
		return ret;

	return devm_rc_register_device(dev, idata->rc);
}

static const struct of_device_id ir_spi_of_match[] = {
	{ .compatible = "ir-spi-led" },
	{ .compatible = "xiaomi,piano-ir-spi", .data = &piano_ir_spi },
	{}
};
MODULE_DEVICE_TABLE(of, ir_spi_of_match);

static const struct spi_device_id ir_spi_ids[] = {
	{ "ir-spi-led" },
	{ "piano-ir-spi", (kernel_ulong_t)&piano_ir_spi },
	{}
};
MODULE_DEVICE_TABLE(spi, ir_spi_ids);

static struct spi_driver ir_spi_driver = {
	.probe = ir_spi_probe,
	.id_table = ir_spi_ids,
	.driver = {
		.name = IR_SPI_DRIVER_NAME,
		.of_match_table = ir_spi_of_match,
	},
};
module_spi_driver(ir_spi_driver);

MODULE_AUTHOR("Andi Shyti <andi@etezian.org>");
MODULE_DESCRIPTION("SPI IR LED");
MODULE_LICENSE("GPL v2");
