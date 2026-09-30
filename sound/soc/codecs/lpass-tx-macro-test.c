// SPDX-License-Identifier: GPL-2.0-only
/* Included by the driver: real ALSA controls/callbacks, synthetic register bus. */
#include <kunit/device.h>
#include <kunit/test.h>
#include <sound/control.h>

struct tx_macro_test {
	struct tx_macro tx;
	struct snd_soc_card card;
	struct snd_soc_aux_dev aux;
	struct snd_soc_component_driver component_driver;
	u32 regs[0x1000 / sizeof(u32)];
};

static int tx_test_reg_read(void *context, unsigned int reg, unsigned int *value)
{
	struct tx_macro_test *ctx = context;

	*value = ctx->regs[reg / sizeof(u32)];
	return 0;
}

static int tx_test_reg_write(void *context, unsigned int reg, unsigned int value)
{
	struct tx_macro_test *ctx = context;

	ctx->regs[reg / sizeof(u32)] = value;
	return 0;
}

static const struct regmap_config tx_test_regmap = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xffc,
	.reg_read = tx_test_reg_read,
	.reg_write = tx_test_reg_write,
};

static int tx_test_component_probe(struct snd_soc_component *component)
{
	struct tx_macro *tx = snd_soc_component_get_drvdata(component);

	tx->component = component;
	snd_soc_component_init_regmap(component, tx->regmap);
	return 0;
}

static void tx_test_unregister_card(void *data)
{
	snd_soc_unregister_card(data);
}

static int tx_test_init(struct kunit *test)
{
	struct tx_macro_test *ctx;
	struct device *dev;
	int i, ret;

	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	test->priv = ctx;
	dev = kunit_device_register(test, "lpass-tx-control-test");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	ctx->tx.dev = dev;
	dev_set_drvdata(dev, &ctx->tx);
	/* Match the driver's cold probe state for the unfixed implementation. */
	for (i = TX_MACRO_AIF1_CAP; i < TX_MACRO_MAX_DAIS; i++)
		ctx->tx.active_decimator[i] = -1;
	ctx->tx.regmap = devm_regmap_init(dev, NULL, ctx, &tx_test_regmap);
	if (IS_ERR(ctx->tx.regmap))
		return PTR_ERR(ctx->tx.regmap);

	/* Bind only the three real AIF mixer widgets; no codec power events. */
	ctx->component_driver.name = "lpass-tx-control-test";
	ctx->component_driver.probe = tx_test_component_probe;
	ctx->component_driver.dapm_widgets = &tx_macro_dapm_widgets[3];
	ctx->component_driver.num_dapm_widgets = 3;
	ret = devm_snd_soc_register_component(dev, &ctx->component_driver, NULL, 0);
	if (ret)
		return ret;
	ctx->aux.dlc.name = dev_name(dev);
	ctx->card.name = "lpass-tx-control-test";
	ctx->card.dev = dev;
	ctx->card.owner = THIS_MODULE;
	ctx->card.aux_dev = &ctx->aux;
	ctx->card.num_aux_devs = 1;
	ret = snd_soc_register_card(&ctx->card);
	if (ret)
		return ret;
	return kunit_add_action_or_reset(test, tx_test_unregister_card, &ctx->card);
}

static int tx_test_put(struct kunit *test, unsigned int dai, unsigned int dec,
		       unsigned int enabled)
{
	struct tx_macro_test *ctx = test->priv;
	struct snd_ctl_elem_value value = {};
	struct snd_kcontrol *control;
	char name[48];

	snprintf(name, sizeof(name), "TX_AIF%u_CAP Mixer DEC%u", dai, dec);
	control = snd_soc_card_get_kcontrol(&ctx->card, name);
	if (!control)
		return -ENOENT;
	value.value.integer.value[0] = enabled;
	return control->put(control, &value);
}

static void tx_test_map(struct kunit *test, unsigned int dai_id,
			unsigned int channels, unsigned int mask)
{
	struct tx_macro_test *ctx = test->priv;
	struct snd_soc_dai dai = { .component = ctx->tx.component, .id = dai_id };
	unsigned int actual_channels = UINT_MAX, actual_mask = UINT_MAX;

	KUNIT_ASSERT_EQ(test, tx_macro_dai_ops.get_channel_map(&dai, &actual_channels,
							    &actual_mask, NULL, NULL), 0);
	KUNIT_EXPECT_EQ(test, actual_channels, channels);
	KUNIT_EXPECT_EQ(test, actual_mask, mask);
}

static void tx_test_inactive_disable(struct kunit *test)
{
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 1), 1);
	KUNIT_EXPECT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 1, 0), 0);
	tx_test_map(test, TX_MACRO_AIF1_CAP, 1, BIT(0));
	KUNIT_EXPECT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 0), 1);
	tx_test_map(test, TX_MACRO_AIF1_CAP, 0, 0);
}

static void tx_test_repeat_enable(struct kunit *test)
{
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 1), 1);
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 1, 1), 1);
	KUNIT_EXPECT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 1), 0);
	tx_test_map(test, TX_MACRO_AIF1_CAP, 2, BIT(0) | BIT(1));
}

static void tx_test_close_and_reopen(struct kunit *test)
{
	int cycle;

	for (cycle = 0; cycle < 2; cycle++) {
		KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 1), 1);
		KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 1, 1), 1);
		KUNIT_EXPECT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 0), 1);
		tx_test_map(test, TX_MACRO_AIF1_CAP, 1, BIT(1));
		KUNIT_EXPECT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 1, 0), 1);
		tx_test_map(test, TX_MACRO_AIF1_CAP, 0, 0);
	}
}

static void tx_test_mute_selected_channels(struct kunit *test)
{
	struct tx_macro_test *ctx = test->priv;
	struct snd_soc_dai dai = {
		.component = ctx->tx.component,
		.id = TX_MACRO_AIF1_CAP,
	};
	unsigned int value;
	int dec;

	KUNIT_ASSERT_EQ(test, tx_test_put(test, dai.id, 0, 1), 1);
	KUNIT_ASSERT_EQ(test, tx_test_put(test, dai.id, 1, 1), 1);
	KUNIT_ASSERT_EQ(test, tx_macro_dai_ops.mute_stream(&dai, 1, SNDRV_PCM_STREAM_CAPTURE), 0);
	for (dec = 0; dec < 3; dec++) {
		KUNIT_ASSERT_EQ(test, regmap_read(ctx->tx.regmap, CDC_TXn_TX_PATH_CTL(dec), &value), 0);
		KUNIT_EXPECT_EQ(test, value & CDC_TXn_PGA_MUTE_MASK,
				dec < 2 ? CDC_TXn_PGA_MUTE_MASK : 0);
	}
	KUNIT_ASSERT_EQ(test, tx_macro_dai_ops.mute_stream(&dai, 0, SNDRV_PCM_STREAM_CAPTURE), 0);
	for (dec = 0; dec < 3; dec++) {
		KUNIT_ASSERT_EQ(test, regmap_read(ctx->tx.regmap, CDC_TXn_TX_PATH_CTL(dec), &value), 0);
		KUNIT_EXPECT_EQ(test, value & CDC_TXn_PGA_MUTE_MASK, 0);
	}
}

static void tx_test_independent_interfaces(struct kunit *test)
{
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF1_CAP, 0, 1), 1);
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF2_CAP, 2, 1), 1);
	KUNIT_ASSERT_EQ(test, tx_test_put(test, TX_MACRO_AIF2_CAP, 2, 0), 1);
	tx_test_map(test, TX_MACRO_AIF1_CAP, 1, BIT(0));
	tx_test_map(test, TX_MACRO_AIF2_CAP, 0, 0);
	tx_test_map(test, TX_MACRO_AIF3_CAP, 0, 0);
}

static struct kunit_case tx_macro_test_cases[] = {
	KUNIT_CASE(tx_test_inactive_disable),
	KUNIT_CASE(tx_test_repeat_enable),
	KUNIT_CASE(tx_test_close_and_reopen),
	KUNIT_CASE(tx_test_mute_selected_channels),
	KUNIT_CASE(tx_test_independent_interfaces),
	{}
};

static struct kunit_suite tx_macro_test_suite = {
	.name = "lpass-tx-channel-state",
	.init = tx_test_init,
	.test_cases = tx_macro_test_cases,
};

kunit_test_suite(tx_macro_test_suite);
