// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018, Linaro Limited.
// Copyright (c) 2018, The Linux Foundation. All rights reserved.

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/simple_card_utils.h>
#include <linux/input-event-codes.h>
#include "common.h"

#define NAME_SIZE	32
#define QCOM_SND_MAX_CODECS	8

static bool qcom_snd_is_codec_node(const struct device_node *np)
{
	return of_node_name_eq(np, "codec") || strstarts(np->name, "codec-");
}

static void qcom_snd_put_codecs(void *data)
{
	snd_soc_of_put_dai_link_codecs(data);
}

static int qcom_snd_parse_codecs(struct device *dev, struct device_node *link_np,
				struct snd_soc_dai_link *link)
{
	int count, total = 0, index = 0, ret;

	for_each_available_child_of_node_scoped(link_np, np) {
		if (!qcom_snd_is_codec_node(np))
			continue;

		count = of_count_phandle_with_args(np, "sound-dai", "#sound-dai-cells");
		if (count <= 0)
			return count ?: -EINVAL;
		if (count > QCOM_SND_MAX_CODECS - total)
			return -EINVAL;
		total += count;
	}
	if (!total)
		return 0;

	link->codecs = devm_kcalloc(dev, total, sizeof(*link->codecs), GFP_KERNEL);
	if (!link->codecs)
		return -ENOMEM;
	link->num_codecs = total;

	for_each_available_child_of_node_scoped(link_np, np) {
		if (!qcom_snd_is_codec_node(np))
			continue;

		count = of_count_phandle_with_args(np, "sound-dai", "#sound-dai-cells");
		for (int i = 0; i < count; i++) {
			ret = snd_soc_of_get_dlc(np, NULL, &link->codecs[index++], i);
			if (ret) {
				snd_soc_of_put_dai_link_codecs(link);
				return ret;
			}
		}
	}

	return devm_add_action_or_reset(dev, qcom_snd_put_codecs, link);
}

static const struct snd_soc_dapm_widget qcom_jack_snd_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_SPK("DP0 Jack", NULL),
	SND_SOC_DAPM_SPK("DP1 Jack", NULL),
	SND_SOC_DAPM_SPK("DP2 Jack", NULL),
	SND_SOC_DAPM_SPK("DP3 Jack", NULL),
	SND_SOC_DAPM_SPK("DP4 Jack", NULL),
	SND_SOC_DAPM_SPK("DP5 Jack", NULL),
	SND_SOC_DAPM_SPK("DP6 Jack", NULL),
	SND_SOC_DAPM_SPK("DP7 Jack", NULL),
};

static struct device_node *qcom_snd_get_link_node(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct of_phandle_args args;
	int ret;

	if (!card->dev || !card->dev->of_node)
		return NULL;

	for_each_available_child_of_node_scoped(card->dev->of_node, np) {
		struct device_node *cpu_np __free(device_node) =
			of_get_child_by_name(np, "cpu");

		if (!cpu_np)
			continue;

		ret = of_parse_phandle_with_args(cpu_np, "sound-dai", "#sound-dai-cells", 0,
						 &args);
		if (ret)
			continue;

		if (args.np == rtd->dai_link->cpus[0].of_node &&
		    args.args_count == 1 && args.args[0] == cpu_dai->id) {
			of_node_put(args.np);
			return of_node_get(np);
		}

		of_node_put(args.np);
	}

	return NULL;
}

static int qcom_snd_parse_tdm_slot(struct device_node *np,
				   struct qcom_snd_tdm_slot_cfg *cfg)
{
	memset(cfg, 0, sizeof(*cfg));

	return snd_soc_of_parse_tdm_slot(np, &cfg->tx_mask, &cfg->rx_mask,
					 &cfg->slots, &cfg->slot_width);
}

int qcom_snd_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_hw_params *params)
{
	struct device_node *np __free(device_node) = qcom_snd_get_link_node(rtd);
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *mask = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct simple_util_data data = { };
	int format = -EINVAL;

	if (!np)
		return 0;

	simple_util_parse_convert(np, NULL, &data);
	if (of_property_present(np, "convert-rate") &&
	    (data.convert_rate < 8000 || data.convert_rate > 192000))
		return -EINVAL;
	if (of_property_present(np, "convert-channels") &&
	    (!data.convert_channels || data.convert_channels > 32))
		return -EINVAL;
	if (of_property_present(np, "convert-sample-format")) {
		if (!data.convert_sample_format)
			return -EINVAL;
		format = simple_util_get_sample_fmt(&data);
		if (format < 0)
			return format;
	}

	if (data.convert_rate)
		rate->min = rate->max = data.convert_rate;
	if (data.convert_channels)
		channels->min = channels->max = data.convert_channels;
	if (format >= 0) {
		snd_mask_none(mask);
		snd_mask_set(mask, format);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_be_hw_params_fixup);

static int qcom_snd_normalize_tdm_slots(struct qcom_snd_tdm_slot_cfg *cpu_cfg,
					struct qcom_snd_tdm_slot_cfg *codec_cfg,
					unsigned int num_codecs)
{
	unsigned int slots = cpu_cfg->slots, slot_width = cpu_cfg->slot_width;
	unsigned int mask, i;

	/* All endpoints share one frame, but each codec has its own slot masks. */
	for (i = 0; i < num_codecs; i++) {
		if (slots && codec_cfg[i].slots && slots != codec_cfg[i].slots)
			return -EINVAL;
		if (slot_width && codec_cfg[i].slot_width &&
		    slot_width != codec_cfg[i].slot_width)
			return -EINVAL;
		slots = slots ?: codec_cfg[i].slots;
		slot_width = slot_width ?: codec_cfg[i].slot_width;
	}
	if (!slots)
		return 0;
	if (slots > 32 || !slot_width)
		return -EINVAL;

	mask = GENMASK(slots - 1, 0);
	if ((cpu_cfg->tx_mask | cpu_cfg->rx_mask) & ~mask)
		return -EINVAL;
	for (i = 0; i < num_codecs; i++) {
		if ((codec_cfg[i].tx_mask | codec_cfg[i].rx_mask) & ~mask)
			return -EINVAL;
	}

	cpu_cfg->slots = slots;
	cpu_cfg->slot_width = slot_width;
	for (i = 0; i < num_codecs; i++) {
		codec_cfg[i].slots = slots;
		codec_cfg[i].slot_width = slot_width;
	}

	return 0;
}

static int qcom_snd_parse_dai_tdm_slots(struct snd_soc_pcm_runtime *rtd,
					struct qcom_snd_tdm_slot_cfg *cpu_cfg,
					struct qcom_snd_tdm_slot_cfg *codec_cfg)
{
	struct device_node *link_np __free(device_node) = qcom_snd_get_link_node(rtd);
	unsigned int index = 0;
	int ret, count;

	if (!link_np)
		return -ENOENT;

	struct device_node *cpu_np __free(device_node) =
		of_get_child_by_name(link_np, "cpu");
	if (!cpu_np)
		return -ENOENT;

	ret = qcom_snd_parse_tdm_slot(cpu_np, cpu_cfg);
	if (ret)
		return ret;

	for_each_available_child_of_node_scoped(link_np, np) {
		struct qcom_snd_tdm_slot_cfg cfg;

		if (!qcom_snd_is_codec_node(np))
			continue;
		count = of_count_phandle_with_args(np, "sound-dai", "#sound-dai-cells");
		if (count <= 0 || (unsigned int)count > rtd->dai_link->num_codecs - index)
			return -EINVAL;
		ret = qcom_snd_parse_tdm_slot(np, &cfg);
		if (ret)
			return ret;
		cfg.bclk_inverted = of_property_read_bool(np, "bitclock-inversion");
		for (int i = 0; i < count; i++)
			codec_cfg[index++] = cfg;
	}

	return index == rtd->dai_link->num_codecs ? 0 : -EINVAL;
}

int qcom_snd_get_dai_tdm_slots(struct snd_soc_pcm_runtime *rtd,
			       struct qcom_snd_tdm_slot_cfg *cpu_cfg,
			       struct qcom_snd_tdm_slot_cfg *codec_cfg)
{
	unsigned int i;
	int ret;

	ret = qcom_snd_parse_dai_tdm_slots(rtd, cpu_cfg, codec_cfg);
	if (ret)
		return ret;

	ret = qcom_snd_normalize_tdm_slots(cpu_cfg, codec_cfg, rtd->dai_link->num_codecs);
	if (ret)
		return ret;

	/* Bit clock inversion needs a TDM frame and a serial format to apply to. */
	for (i = 0; i < rtd->dai_link->num_codecs; i++) {
		if (codec_cfg[i].bclk_inverted &&
		    (!cpu_cfg->slots || !(rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK)))
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_get_dai_tdm_slots);

int qcom_snd_apply_dai_tdm_slots_cfg(struct snd_soc_pcm_runtime *rtd,
				     const struct qcom_snd_tdm_slot_cfg *cpu_cfg,
				     const struct qcom_snd_tdm_slot_cfg *codec_cfg)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	unsigned int fmt = rtd->dai_link->dai_fmt;
	struct snd_soc_dai *codec_dai;
	int i;
	int ret;

	if (!cpu_cfg->slots)
		return 0;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, cpu_cfg->tx_mask, cpu_cfg->rx_mask,
				       cpu_cfg->slots, cpu_cfg->slot_width);
	if (ret)
		return ret;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_tdm_slot(codec_dai,
					       codec_cfg[i].tx_mask,
					       codec_cfg[i].rx_mask,
					       codec_cfg[i].slots,
					       codec_cfg[i].slot_width);
		if (ret)
			return ret;
	}

	/*
	 * The CPU DAI keeps its bit clock polarity. Codecs that latch data on
	 * the other edge get the link format with the bit clock inverted.
	 */
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		if (!codec_cfg[i].bclk_inverted)
			continue;
		ret = snd_soc_dai_set_fmt(codec_dai, (fmt & ~SND_SOC_DAIFMT_INV_MASK) |
					  SND_SOC_DAIFMT_IB_NF);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_apply_dai_tdm_slots_cfg);

int qcom_snd_apply_dai_tdm_slots(struct snd_soc_pcm_runtime *rtd)
{
	struct qcom_snd_tdm_slot_cfg cpu_cfg;
	struct qcom_snd_tdm_slot_cfg *codec_cfg __free(kfree) =
		kcalloc(rtd->dai_link->num_codecs, sizeof(*codec_cfg), GFP_KERNEL);
	int ret;

	if (!codec_cfg)
		return -ENOMEM;
	ret = qcom_snd_get_dai_tdm_slots(rtd, &cpu_cfg, codec_cfg);
	if (ret)
		return ret == -ENOENT ? 0 : ret;

	return qcom_snd_apply_dai_tdm_slots_cfg(rtd, &cpu_cfg, codec_cfg);
}
EXPORT_SYMBOL_GPL(qcom_snd_apply_dai_tdm_slots);

int qcom_snd_parse_of(struct snd_soc_card *card)
{
	struct device *dev = card->dev;
	struct snd_soc_dai_link *link;
	struct of_phandle_args args;
	struct snd_soc_dai_link_component *dlc;
	int ret, num_links;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret == 0 && !card->name)
		/* Deprecated, only for compatibility with old device trees */
		ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret) {
		dev_err(dev, "Error parsing card name: %d\n", ret);
		return ret;
	}

	if (of_property_present(dev->of_node, "widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(card, "widgets");
		if (ret)
			return ret;
	}

	/* DAPM routes */
	if (of_property_present(dev->of_node, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret)
			return ret;
	}
	/* Deprecated, only for compatibility with old device trees */
	if (of_property_present(dev->of_node, "qcom,audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "qcom,audio-routing");
		if (ret)
			return ret;
	}

	ret = snd_soc_of_parse_pin_switches(card, "pin-switches");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_aux_devs(card, "aux-devs");
	if (ret)
		return ret;

	/* Populate links */
	num_links = of_get_available_child_count(dev->of_node);

	/* Allocate the DAI link array */
	card->dai_link = devm_kzalloc_objs(dev, *link, num_links);
	if (!card->dai_link)
		return -ENOMEM;

	card->num_links = num_links;
	link = card->dai_link;

	for_each_available_child_of_node_scoped(dev->of_node, np) {
		dlc = devm_kzalloc_objs(dev, *dlc, 2);
		if (!dlc)
			return -ENOMEM;

		link->cpus	= &dlc[0];
		link->platforms	= &dlc[1];

		link->num_cpus		= 1;
		link->num_platforms	= 1;

		ret = of_property_read_string(np, "link-name", &link->name);
		if (ret) {
			dev_err(dev, "error getting codec dai_link name\n");
			return ret;
		}

		if (of_property_present(np, "dai-format")) {
			link->dai_fmt = snd_soc_daifmt_parse_format(np, NULL);
			if (!(link->dai_fmt & SND_SOC_DAIFMT_FORMAT_MASK))
				return -EINVAL;
			link->dai_fmt |= SND_SOC_DAIFMT_CBC_CFC;
		}

		struct device_node *cpu __free(device_node) =
			of_get_child_by_name(np, "cpu");
		struct device_node *platform __free(device_node) =
			of_get_child_by_name(np, "platform");

		if (!cpu) {
			dev_err(dev, "%s: Can't find cpu DT node\n", link->name);
			return -EINVAL;
		}

		ret = snd_soc_of_get_dlc(cpu, &args, link->cpus, 0);
		if (ret) {
			dev_err_probe(dev, ret,
				      "%s: error getting cpu dai name\n", link->name);
			return ret;
		}

		link->id = args.args[0];

		if (link->id >= LPASS_MAX_PORT) {
			dev_err(dev, "%s: Invalid cpu dai id %d\n", link->name, link->id);
			return -EINVAL;
		}

		if (platform) {
			link->platforms->of_node = of_parse_phandle(platform,
					"sound-dai",
					0);
			if (!link->platforms->of_node) {
				dev_err(dev, "%s: platform dai not found\n", link->name);
				return -EINVAL;
			}
		} else {
			link->platforms->of_node = link->cpus->of_node;
		}

		ret = qcom_snd_parse_codecs(dev, np, link);
		if (ret)
			return dev_err_probe(dev, ret, "%s: codec dai not found\n", link->name);

		if (link->num_codecs) {
			if (platform) {
				/* DPCM backend */
				link->no_pcm = 1;
				link->ignore_pmdown_time = 1;
			}
		} else {
			/* DPCM frontend */
			link->codecs	 = &snd_soc_dummy_dlc;
			link->num_codecs = 1;
			link->dynamic = 1;
		}

		if (platform || link->dynamic) {
			/* DPCM */
			link->ignore_suspend = 1;
			link->nonatomic = 1;
		}

		link->stream_name = link->name;
		link++;
	}

	if (!card->dapm_widgets) {
		card->dapm_widgets = qcom_jack_snd_widgets;
		card->num_dapm_widgets = ARRAY_SIZE(qcom_jack_snd_widgets);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_parse_of);

static struct snd_soc_jack_pin qcom_headset_jack_pins[] = {
	/* Headset */
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static int qcom_snd_headset_jack_init(struct snd_soc_card *card,
				      struct snd_soc_jack *jack,
				      bool *jack_setup)
{
	int rval;

	if (!*jack_setup) {
		rval = snd_soc_card_jack_new_pins(card, "Headset Jack",
					     SND_JACK_HEADSET | SND_JACK_LINEOUT |
					     SND_JACK_MECHANICAL |
					     SND_JACK_BTN_0 | SND_JACK_BTN_1 |
					     SND_JACK_BTN_2 | SND_JACK_BTN_3 |
					     SND_JACK_BTN_4 | SND_JACK_BTN_5,
					     jack, qcom_headset_jack_pins,
					     ARRAY_SIZE(qcom_headset_jack_pins));

		if (rval < 0) {
			dev_err(card->dev, "Unable to add Headphone Jack\n");
			return rval;
		}

		snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_MEDIA);
		snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
		snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
		snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
		*jack_setup = true;
	}

	return 0;
}

int qcom_snd_headset_jack_setup(struct snd_soc_pcm_runtime *rtd,
				struct snd_soc_jack *jack, bool *jack_setup)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_card *card = rtd->card;
	int rval, i;

	rval = qcom_snd_headset_jack_init(card, jack, jack_setup);
	if (rval)
		return rval;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		rval = snd_soc_component_set_jack(codec_dai->component, jack, NULL);
		if (rval != 0 && rval != -ENOTSUPP) {
			dev_warn(card->dev, "Failed to set jack: %d\n", rval);
			qcom_snd_headset_jack_cleanup(rtd);
			return rval;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_headset_jack_setup);

void qcom_snd_headset_jack_cleanup(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai;
	int i;

	for_each_rtd_codec_dais(rtd, i, codec_dai)
		snd_soc_component_set_jack(codec_dai->component, NULL, NULL);
}
EXPORT_SYMBOL_GPL(qcom_snd_headset_jack_cleanup);

int qcom_snd_wcd_jack_setup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_soc_jack *jack, bool *jack_setup)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	int rval, i;

	rval = qcom_snd_headset_jack_init(card, jack, jack_setup);
	if (rval)
		return rval;

	switch (cpu_dai->id) {
	case LPI_MI2S_RX_0:
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			rval = snd_soc_component_set_jack(codec_dai->component,
							  jack, NULL);
			if (rval != 0 && rval != -ENOTSUPP) {
				dev_warn(card->dev, "Failed to set jack: %d\n", rval);
				return rval;
			}
		}

		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_wcd_jack_setup);

int qcom_snd_dp_jack_setup(struct snd_soc_pcm_runtime *rtd,
			   struct snd_soc_jack *dp_jack, int dp_pcm_id)
{
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	char jack_name[NAME_SIZE];
	int rval, i;

	snprintf(jack_name, sizeof(jack_name), "DP%d Jack", dp_pcm_id);
	rval = snd_soc_card_jack_new(card, jack_name, SND_JACK_AVOUT, dp_jack);
	if (rval)
		return rval;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		rval = snd_soc_component_set_jack(codec_dai->component, dp_jack, NULL);
		if (rval != 0 && rval != -ENOTSUPP) {
			dev_warn(card->dev, "Failed to set jack: %d\n", rval);
			return rval;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_snd_dp_jack_setup);

MODULE_DESCRIPTION("ASoC Qualcomm helper functions");
MODULE_LICENSE("GPL");
