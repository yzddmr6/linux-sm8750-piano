// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (C) 2023 Linaro Ltd.
 */

#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/wcd939x-usbss.h>

#define WCD_USBSS_PMP_OUT1			0x2

#define WCD_USBSS_DP_DN_MISC1			0x20

#define WCD_USBSS_DP_DN_MISC1_DP_PCOMP_2X_DYN_BST_ON_EN			BIT(3)
#define WCD_USBSS_DP_DN_MISC1_DN_PCOMP_2X_DYN_BST_ON_EN			BIT(0)

#define WCD_USBSS_MG1_EN			0x24

#define WCD_USBSS_MG1_EN_CT_SNS_EN					BIT(1)

#define WCD_USBSS_MG1_BIAS			0x25

#define WCD_USBSS_MG1_BIAS_PCOMP_DYN_BST_EN				BIT(3)

#define WCD_USBSS_MG1_MISC			0x27

#define WCD_USBSS_MG1_MISC_PCOMP_2X_DYN_BST_ON_EN			BIT(5)

#define WCD_USBSS_MG2_EN			0x28

#define WCD_USBSS_MG2_EN_CT_SNS_EN					BIT(1)

#define WCD_USBSS_MG2_BIAS			0x29

#define WCD_USBSS_MG2_BIAS_PCOMP_DYN_BST_EN				BIT(3)

#define WCD_USBSS_MG2_MISC			0x30

#define WCD_USBSS_MG2_MISC_PCOMP_2X_DYN_BST_ON_EN			BIT(5)

#define WCD_USBSS_DISP_AUXP_THRESH		0x80

#define WCD_USBSS_DISP_AUXP_THRESH_DISP_AUXP_OVPON_CM			GENMASK(7, 5)

#define WCD_USBSS_DISP_AUXP_CTL			0x81

#define WCD_USBSS_DISP_AUXP_CTL_LK_CANCEL_TRK_COEFF			GENMASK(2, 0)

#define WCD_USBSS_CPLDO_CTL2			0xa1

#define WCD_USBSS_SWITCH_SETTINGS_ENABLE	0x403

#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_DEVICE_ENABLE			BIT(7)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXP_TO_MGX_SWITCHES	BIT(6)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXM_TO_MGX_SWITCHES	BIT(5)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_DNL_SWITCHES			BIT(4)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_DPR_SWITCHES			BIT(3)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_SENSE_SWITCHES			BIT(2)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_MIC_SWITCHES			BIT(1)
#define WCD_USBSS_SWITCH_SETTINGS_ENABLE_AGND_SWITCHES			BIT(0)

#define WCD_USBSS_SWITCH_SELECT0		0x404

#define WCD_USBSS_SWITCH_SELECT0_DP_AUXP_SWITCHES			BIT(7)	/* 1-> MG2 */
#define WCD_USBSS_SWITCH_SELECT0_DP_AUXM_SWITCHES			BIT(6)	/* 1-> MG2 */
#define WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES				GENMASK(5, 4)
#define WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES				GENMASK(3, 2)
#define WCD_USBSS_SWITCH_SELECT0_SENSE_SWITCHES				BIT(1)	/* 1-> SBU2 */
#define WCD_USBSS_SWITCH_SELECT0_MIC_SWITCHES				BIT(0)	/* 1-> MG2 */

#define WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_L		0
#define WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_DN		1
#define WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_DN2		2

#define WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_R		0
#define WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_DP		1
#define WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_DR2		2

#define WCD_USBSS_SWITCH_SELECT1		0x405

#define WCD_USBSS_SWITCH_SELECT1_AGND_SWITCHES				BIT(0)	/* 1-> MG2 */

#define WCD_USBSS_DELAY_R_SW			0x40d
#define WCD_USBSS_DELAY_MIC_SW			0x40e
#define WCD_USBSS_DELAY_SENSE_SW		0x40f
#define WCD_USBSS_DELAY_GND_SW			0x410
#define WCD_USBSS_DELAY_L_SW			0x411

#define WCD_USBSS_FUNCTION_ENABLE		0x413

#define WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT				GENMASK(1, 0)

#define WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT_MANUAL		1
#define WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT_AUDIO_FSM	2

#define WCD_USBSS_EQUALIZER1			0x415

#define WCD_USBSS_EQUALIZER1_EQ_EN					BIT(7)
#define WCD_USBSS_EQUALIZER1_BW_SETTINGS				GENMASK(6, 3)

#define WCD_USBSS_USB_SS_CNTL			0x419

#define WCD_USBSS_USB_SS_CNTL_STANDBY_STATE				BIT(4)
#define WCD_USBSS_USB_SS_CNTL_RCO_EN					BIT(3)
#define WCD_USBSS_USB_SS_CNTL_USB_SS_MODE				GENMASK(2, 0)

#define WCD_USBSS_USB_SS_CNTL_USB_SS_MODE_AATC		2
#define WCD_USBSS_USB_SS_CNTL_USB_SS_MODE_USB		5

#define WCD_USBSS_AUDIO_FSM_START		0x433

#define WCD_USBSS_AUDIO_FSM_START_AUDIO_FSM_AUDIO_TRIG			BIT(0)

#define WCD_USBSS_RATIO_SPKR_REXT_L_LSB		0x461
#define WCD_USBSS_RATIO_SPKR_REXT_L_MSB		0x462
#define WCD_USBSS_RATIO_SPKR_REXT_R_LSB		0x463
#define WCD_USBSS_RATIO_SPKR_REXT_R_MSB		0x464
#define WCD_USBSS_AUD_COEF_L_K0_0		0x475
#define WCD_USBSS_AUD_COEF_L_K0_1		0x476
#define WCD_USBSS_AUD_COEF_L_K0_2		0x477
#define WCD_USBSS_AUD_COEF_L_K1_0		0x478
#define WCD_USBSS_AUD_COEF_L_K1_1		0x479
#define WCD_USBSS_AUD_COEF_L_K2_0		0x47a
#define WCD_USBSS_AUD_COEF_L_K2_1		0x47b
#define WCD_USBSS_AUD_COEF_L_K3_0		0x47c
#define WCD_USBSS_AUD_COEF_L_K3_1		0x47d
#define WCD_USBSS_AUD_COEF_L_K4_0		0x47e
#define WCD_USBSS_AUD_COEF_L_K4_1		0x47f
#define WCD_USBSS_AUD_COEF_L_K5_0		0x480
#define WCD_USBSS_AUD_COEF_L_K5_1		0x481
#define WCD_USBSS_AUD_COEF_R_K0_0		0x482
#define WCD_USBSS_AUD_COEF_R_K0_1		0x483
#define WCD_USBSS_AUD_COEF_R_K0_2		0x484
#define WCD_USBSS_AUD_COEF_R_K1_0		0x485
#define WCD_USBSS_AUD_COEF_R_K1_1		0x486
#define WCD_USBSS_AUD_COEF_R_K2_0		0x487
#define WCD_USBSS_AUD_COEF_R_K2_1		0x488
#define WCD_USBSS_AUD_COEF_R_K3_0		0x489
#define WCD_USBSS_AUD_COEF_R_K3_1		0x48a
#define WCD_USBSS_AUD_COEF_R_K4_0		0x48b
#define WCD_USBSS_AUD_COEF_R_K4_1		0x48c
#define WCD_USBSS_AUD_COEF_R_K5_0		0x48d
#define WCD_USBSS_AUD_COEF_R_K5_1		0x48e
#define WCD_USBSS_GND_COEF_L_K0_0		0x48f
#define WCD_USBSS_GND_COEF_L_K0_1		0x490
#define WCD_USBSS_GND_COEF_L_K0_2		0x491
#define WCD_USBSS_GND_COEF_L_K1_0		0x492
#define WCD_USBSS_GND_COEF_L_K1_1		0x493
#define WCD_USBSS_GND_COEF_L_K2_0		0x494
#define WCD_USBSS_GND_COEF_L_K2_1		0x495
#define WCD_USBSS_GND_COEF_L_K3_0		0x496
#define WCD_USBSS_GND_COEF_L_K3_1		0x497
#define WCD_USBSS_GND_COEF_L_K4_0		0x498
#define WCD_USBSS_GND_COEF_L_K4_1		0x499
#define WCD_USBSS_GND_COEF_L_K5_0		0x49a
#define WCD_USBSS_GND_COEF_L_K5_1		0x49b
#define WCD_USBSS_GND_COEF_R_K0_0		0x49c
#define WCD_USBSS_GND_COEF_R_K0_1		0x49d
#define WCD_USBSS_GND_COEF_R_K0_2		0x49e
#define WCD_USBSS_GND_COEF_R_K1_0		0x49f
#define WCD_USBSS_GND_COEF_R_K1_1		0x4a0
#define WCD_USBSS_GND_COEF_R_K2_0		0x4a1
#define WCD_USBSS_GND_COEF_R_K2_1		0x4a2
#define WCD_USBSS_GND_COEF_R_K3_0		0x4a3
#define WCD_USBSS_GND_COEF_R_K3_1		0x4a4
#define WCD_USBSS_GND_COEF_R_K4_0		0x4a5
#define WCD_USBSS_GND_COEF_R_K4_1		0x4a6
#define WCD_USBSS_GND_COEF_R_K5_0		0x4a7
#define WCD_USBSS_GND_COEF_R_K5_1		0x4a8

#define WCD_USBSS_MAX_REGISTER			0x4c1

struct wcd939x_usbss {
	struct i2c_client *client;
	struct gpio_desc *reset_gpio;
	struct regulator *vdd_supply;

	/* used to serialize concurrent change requests */
	struct mutex lock;

	struct typec_switch_dev *sw;
	struct typec_mux_dev *mux;

	struct regmap *regmap;

	struct typec_mux *codec;
	struct typec_switch *codec_switch;

	enum typec_orientation orientation;
	unsigned long mode;
	unsigned int svid;

	/* DP alt-mode is routed through the SBU/MG switches right now */
	bool dp_active;
	/* D+/D- currently parked on the charge-pump crossbar (erhai VOOC) */
	bool fc_routed;
};

/*
 * Single USBSS instance reference for the charge-pump fast-charge crossbar
 * helper. The board has exactly one WCD939x USBSS; the SC8547A VOOC PHY driver
 * needs to flip its DPDM route but does not own it, so it reaches the switch
 * through wcd939x_usbss_dpdm_switch_fast_charge() below. Guarded by its own
 * lock so probe/remove can't race the exported call.
 */
static DEFINE_MUTEX(wcd939x_usbss_fc_lock);
static struct wcd939x_usbss *wcd939x_usbss_fc_dev;

static const struct regmap_range_cfg wcd939x_usbss_ranges[] = {
	{
		.range_min = 0,
		.range_max = WCD_USBSS_MAX_REGISTER,
		.selector_reg = 0x0,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config wcd939x_usbss_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WCD_USBSS_MAX_REGISTER,
	.ranges = wcd939x_usbss_ranges,
	.num_ranges = ARRAY_SIZE(wcd939x_usbss_ranges),
};

/* Linearlizer coefficients for 32ohm load */
static const struct {
	unsigned int offset;
	unsigned int mask;
	unsigned int value;
} wcd939x_usbss_coeff_init[] = {
	{ WCD_USBSS_AUD_COEF_L_K5_0, GENMASK(7, 0), 0x39 },
	{ WCD_USBSS_AUD_COEF_R_K5_0, GENMASK(7, 0), 0x39 },
	{ WCD_USBSS_GND_COEF_L_K2_0, GENMASK(7, 0), 0xe8 },
	{ WCD_USBSS_GND_COEF_L_K4_0, GENMASK(7, 0), 0x73 },
	{ WCD_USBSS_GND_COEF_R_K2_0, GENMASK(7, 0), 0xe8 },
	{ WCD_USBSS_GND_COEF_R_K4_0, GENMASK(7, 0), 0x73 },
	{ WCD_USBSS_RATIO_SPKR_REXT_L_LSB, GENMASK(7, 0), 0x00 },
	{ WCD_USBSS_RATIO_SPKR_REXT_L_MSB, GENMASK(6, 0), 0x04 },
	{ WCD_USBSS_RATIO_SPKR_REXT_R_LSB, GENMASK(7, 0), 0x00 },
	{ WCD_USBSS_RATIO_SPKR_REXT_R_MSB, GENMASK(6, 0), 0x04 },
};

static int wcd939x_usbss_set(struct wcd939x_usbss *usbss)
{
	bool reverse = (usbss->orientation == TYPEC_ORIENTATION_REVERSE);
	bool enable_audio = false;
	bool enable_usb = false;
	bool enable_dp = false;
	int ret;

	/* USB Mode */
	if (usbss->mode < TYPEC_STATE_MODAL ||
	    (!usbss->svid && (usbss->mode == TYPEC_MODE_USB2 ||
			      usbss->mode == TYPEC_MODE_USB3))) {
		enable_usb = true;
	} else if (usbss->svid) {
		switch (usbss->mode) {
		/* DP Only */
		case TYPEC_DP_STATE_C:
		case TYPEC_DP_STATE_E:
			enable_dp = true;
			break;

		/* DP + USB */
		case TYPEC_DP_STATE_D:
		case TYPEC_DP_STATE_F:
			enable_usb = true;
			enable_dp = true;
			break;

		default:
			return -EOPNOTSUPP;
		}
	} else if (usbss->mode == TYPEC_MODE_AUDIO) {
		enable_audio = true;
	} else {
		return -EOPNOTSUPP;
	}

	/*
	 * This reprograms the whole switch block, so whatever the charge-pump
	 * crossbar did to D+/D- is gone; track DP so the crossbar cannot break
	 * the AUX routing behind our back (erhai).
	 */
	usbss->dp_active = enable_dp;
	usbss->fc_routed = false;

	dev_info(&usbss->client->dev,
		 "erhai usbss: set mode=%lu svid=%04x orient=%d -> usb=%d dp=%d audio=%d dp_active=%d\n",
		 usbss->mode, usbss->svid, usbss->orientation,
		 enable_usb, enable_dp, enable_audio, usbss->dp_active);

	/* Disable all switches */
	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXP_TO_MGX_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXM_TO_MGX_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_DPR_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_DNL_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_SENSE_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_MIC_SWITCHES |
				WCD_USBSS_SWITCH_SETTINGS_ENABLE_AGND_SWITCHES);
	if (ret)
		return ret;

	/* Clear switches */
	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
				WCD_USBSS_SWITCH_SELECT0_DP_AUXP_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_DP_AUXM_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_SENSE_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_MIC_SWITCHES);
	if (ret)
		return ret;

	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT1,
				WCD_USBSS_SWITCH_SELECT1_AGND_SWITCHES);
	if (ret)
		return ret;

	/* Enable OVP_MG1_BIAS PCOMP_DYN_BST_EN */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_MG1_BIAS,
			      WCD_USBSS_MG1_BIAS_PCOMP_DYN_BST_EN);
	if (ret)
		return ret;

	/* Enable OVP_MG2_BIAS PCOMP_DYN_BST_EN */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_MG2_BIAS,
			      WCD_USBSS_MG2_BIAS_PCOMP_DYN_BST_EN);
	if (ret)
		return ret;

	/* Disable Equalizer in safe mode */
	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_EQUALIZER1,
				WCD_USBSS_EQUALIZER1_EQ_EN);
	if (ret)
		return ret;

	/* Start FSM with all disabled, force write */
	ret = regmap_write_bits(usbss->regmap, WCD_USBSS_AUDIO_FSM_START,
				WCD_USBSS_AUDIO_FSM_START_AUDIO_FSM_AUDIO_TRIG,
				WCD_USBSS_AUDIO_FSM_START_AUDIO_FSM_AUDIO_TRIG);

	/* 35us to allow the SBU switch to turn off */
	usleep_range(35, 1000);

	/* Setup Audio Accessory mux/switch */
	if (enable_audio) {
		int i;

		/*
		 * AATC switch configuration:
		 * "Normal":
		 * - R: DNR
		 * - L: DNL
		 * - Sense: GSBU2
		 * - Mic: MG1
		 * - AGND: MG2
		 * "Swapped":
		 * - R: DNR
		 * - L: DNL
		 * - Sense: GSBU1
		 * - Mic: MG2
		 * - AGND: MG1
		 * Swapped information is given by the codec MBHC logic
		 */

		/* Set AATC mode */
		ret = regmap_update_bits(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
					 WCD_USBSS_USB_SS_CNTL_USB_SS_MODE,
					 FIELD_PREP(WCD_USBSS_USB_SS_CNTL_USB_SS_MODE,
						    WCD_USBSS_USB_SS_CNTL_USB_SS_MODE_AATC));
		if (ret)
			return ret;

		/* Select L for DNL_SWITCHES and R for DPR_SWITCHES */
		ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
				WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES |
				WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
				FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
					WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_L) |
				FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES,
					WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_R));
		if (ret)
			return ret;

		if (reverse)
			/* Select MG2 for MIC, SBU1 for Sense */
			ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
						 WCD_USBSS_SWITCH_SELECT0_MIC_SWITCHES,
						 WCD_USBSS_SWITCH_SELECT0_MIC_SWITCHES);
		else
			/* Select MG1 for MIC, SBU2 for Sense */
			ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
						 WCD_USBSS_SWITCH_SELECT0_SENSE_SWITCHES,
						 WCD_USBSS_SWITCH_SELECT0_SENSE_SWITCHES);
		if (ret)
			return ret;

		if (reverse)
			/* Disable OVP_MG1_BIAS PCOMP_DYN_BST_EN */
			ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_MG1_BIAS,
						WCD_USBSS_MG1_BIAS_PCOMP_DYN_BST_EN);
		else
			/* Disable OVP_MG2_BIAS PCOMP_DYN_BST_EN */
			ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_MG2_BIAS,
						WCD_USBSS_MG2_BIAS_PCOMP_DYN_BST_EN);
		if (ret)
			return ret;

		/*  Enable SENSE, MIC switches */
		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_SENSE_SWITCHES |
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_MIC_SWITCHES);
		if (ret)
			return ret;

		if (reverse)
			/* Select MG1 for AGND_SWITCHES */
			ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT1,
						WCD_USBSS_SWITCH_SELECT1_AGND_SWITCHES);
		else
			/* Select MG2 for AGND_SWITCHES */
			ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT1,
					      WCD_USBSS_SWITCH_SELECT1_AGND_SWITCHES);
		if (ret)
			return ret;

		/* Enable AGND switches */
		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_AGND_SWITCHES);
		if (ret)
			return ret;

		/* Enable DPR, DNL switches */
		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DNL_SWITCHES |
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DPR_SWITCHES);
		if (ret)
			return ret;

		/* Setup FSM delays */
		ret = regmap_write(usbss->regmap, WCD_USBSS_DELAY_L_SW, 0x02);
		if (ret)
			return ret;

		ret = regmap_write(usbss->regmap, WCD_USBSS_DELAY_R_SW, 0x02);
		if (ret)
			return ret;

		ret = regmap_write(usbss->regmap, WCD_USBSS_DELAY_MIC_SW, 0x01);
		if (ret)
			return ret;

		/* Start FSM, force write */
		ret = regmap_write_bits(usbss->regmap, WCD_USBSS_AUDIO_FSM_START,
					WCD_USBSS_AUDIO_FSM_START_AUDIO_FSM_AUDIO_TRIG,
					WCD_USBSS_AUDIO_FSM_START_AUDIO_FSM_AUDIO_TRIG);
		if (ret)
			return ret;

		/* Default Linearlizer coefficients */
		for (i = 0; i < ARRAY_SIZE(wcd939x_usbss_coeff_init); ++i)
			regmap_update_bits(usbss->regmap,
					   wcd939x_usbss_coeff_init[i].offset,
					   wcd939x_usbss_coeff_init[i].mask,
					   wcd939x_usbss_coeff_init[i].value);

		return 0;
	}

	ret = regmap_update_bits(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
				 WCD_USBSS_USB_SS_CNTL_USB_SS_MODE,
				 FIELD_PREP(WCD_USBSS_USB_SS_CNTL_USB_SS_MODE,
					    WCD_USBSS_USB_SS_CNTL_USB_SS_MODE_USB));
	if (ret)
		return ret;

	/* Enable USB muxes */
	if (enable_usb) {
		/* Do not enable Equalizer in safe mode */
		if (usbss->mode != TYPEC_STATE_SAFE) {
			ret = regmap_set_bits(usbss->regmap, WCD_USBSS_EQUALIZER1,
					      WCD_USBSS_EQUALIZER1_EQ_EN);
			if (ret)
				return ret;
		}

		/* Select DN for DNL_SWITCHES and DP for DPR_SWITCHES */
		ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
					 WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES |
					 WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
					 FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
						    WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_DN) |
					 FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES,
						    WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_DP));
		if (ret)
			return ret;

		/* Enable DNL_SWITCHES and DPR_SWITCHES */
		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DPR_SWITCHES |
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DNL_SWITCHES);
		if (ret)
			return ret;
	}

	/* Enable DP AUX muxes */
	if (enable_dp) {
		/* Update Leakage Canceller Coefficient for AUXP pins */
		ret = regmap_update_bits(usbss->regmap, WCD_USBSS_DISP_AUXP_CTL,
					 WCD_USBSS_DISP_AUXP_CTL_LK_CANCEL_TRK_COEFF,
					 FIELD_PREP(WCD_USBSS_DISP_AUXP_CTL_LK_CANCEL_TRK_COEFF,
						    5));
		if (ret)
			return ret;

		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_DISP_AUXP_THRESH,
				      WCD_USBSS_DISP_AUXP_THRESH_DISP_AUXP_OVPON_CM);
		if (ret)
			return ret;

		if (reverse)
			/* Select MG2 for AUXP and MG1 for AUXM */
			ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXP_SWITCHES |
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXM_SWITCHES,
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXP_SWITCHES);
		else
			/* Select MG1 for AUXP and MG2 for AUXM */
			ret = regmap_update_bits(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXP_SWITCHES |
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXM_SWITCHES,
						 WCD_USBSS_SWITCH_SELECT0_DP_AUXM_SWITCHES);
		if (ret)
			return ret;

		/* Enable DP_AUXP_TO_MGX and DP_AUXM_TO_MGX switches */
		ret = regmap_set_bits(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXP_TO_MGX_SWITCHES |
				      WCD_USBSS_SWITCH_SETTINGS_ENABLE_DP_AUXM_TO_MGX_SWITCHES);

		/* 15us to allow the SBU switch to turn on again */
		usleep_range(15, 1000);
	}

	return 0;
}

static int wcd939x_usbss_switch_set(struct typec_switch_dev *sw,
				    enum typec_orientation orientation)
{
	struct wcd939x_usbss *usbss = typec_switch_get_drvdata(sw);
	int ret = 0;

	mutex_lock(&usbss->lock);

	if (usbss->orientation != orientation) {
		usbss->orientation = orientation;

		dev_info(&usbss->client->dev,
			 "erhai usbss: switch_set orient=%d\n", orientation);

		ret = wcd939x_usbss_set(usbss);
	}

	mutex_unlock(&usbss->lock);

	if (ret)
		return ret;

	/* Report orientation to codec after switch has been done */
	return typec_switch_set(usbss->codec_switch, orientation);
}

static int wcd939x_usbss_mux_set(struct typec_mux_dev *mux,
				 struct typec_mux_state *state)
{
	struct wcd939x_usbss *usbss = typec_mux_get_drvdata(mux);
	int ret = 0;

	mutex_lock(&usbss->lock);

	/*
	 * fc_routed means the charge-pump crossbar overwrote the switch block,
	 * so the cached mode no longer describes the hardware — reprogram even
	 * when the requested state is unchanged (erhai).
	 */
	/*
	 * Idempotence (erhai): the UCSI poll (erhai_ucsi_poll_ms) calls us
	 * every 4 s with an UNCHANGED state. Reprogramming the crossbar then
	 * steals D+/D- from the charge-pump path and kills SVOOC delivery
	 * (2026-08-28: CHG_EN self-clear at CP enable, margin < 150 mV).
	 * Only reprogram when the requested mode actually changed, or when
	 * the charge path owns the crossbar AND DisplayPort is really coming
	 * up: we then have to take the crossbar back for AUX routing.
	 * TYPEC_STATE_MODAL is only 2 and is shared by USB2/USB3/audio, so a
	 * mode>=MODAL test would steal the crossbar from the charge path on
	 * every plain USB re-poll; the DP SVID is the actual signal.
	 */
	if (usbss->mode != state->mode ||
	    (usbss->fc_routed && state->alt &&
	     state->alt->svid == USB_TYPEC_DP_SID)) {
		usbss->mode = state->mode;

		if (state->alt)
			usbss->svid = state->alt->svid;
		else
			usbss->svid = 0; // No SVID

		ret = wcd939x_usbss_set(usbss);
	}

	/*
	 * The block above is the only other place dp_active is written, and it
	 * runs only on a mode change. A DP sink that withdraws alt-mode while
	 * the cable stays put leaves the mode identical and the alt list empty,
	 * so dp_active would latch at 1 forever and every later fast-charge
	 * request gets -EBUSY. No alt mode means the mux state no longer claims
	 * DP, so drop the flag — clearing only, never setting, keeps this from
	 * ever granting the crossbar while DisplayPort is really up.
	 */
	if (!state->alt && usbss->dp_active) {
		usbss->dp_active = false;
		dev_info(&usbss->client->dev,
			 "erhai usbss: alt-mode gone, dp_active released\n");
	}

	mutex_unlock(&usbss->lock);

	if (ret)
		return ret;

	/* Report event to codec after switch has been done */
	return typec_mux_set(usbss->codec, state);
}

/*
 * Route connector D+/D- to the SC8547A charge-pump crossbar (enable=true) or
 * back to the normal USB path (enable=false). Register sequence captured from
 * the vendor stack on erhai; the crossbar select and USB_SS_CNTL values match
 * the live Android i2c trace byte-for-byte.
 */
int wcd939x_usbss_dpdm_switch_fast_charge(bool enable)
{
	struct wcd939x_usbss *usbss;
	int ret;

	mutex_lock(&wcd939x_usbss_fc_lock);
	usbss = wcd939x_usbss_fc_dev;
	if (!usbss) {
		mutex_unlock(&wcd939x_usbss_fc_lock);
		return -ENODEV;
	}

	mutex_lock(&usbss->lock);

	/*
	 * DP alt-mode owns the SBU/MG switches while a DisplayPort sink is
	 * attached. The register writes below rewrite SWITCH_SETTINGS_ENABLE /
	 * SELECT0 wholesale, which clears the DP_AUXP/AUXM_TO_MGX switches and
	 * kills AUX (every DPCD read then times out -110 and the connector goes
	 * disconnected). A DP sink is never a VOOC adapter, so refuse the
	 * crossbar while DP is up, and restore the mux-programmed routing when
	 * the charge path asks for the normal USB route back.
	 */
	if (usbss->dp_active) {
		if (enable) {
			ret = -EBUSY;
				dev_info(&usbss->client->dev,
					 "erhai usbss: fc DENIED while DP active\n");
		} else if (usbss->fc_routed) {
			ret = wcd939x_usbss_set(usbss);
			if (!ret)
				usbss->fc_routed = false;
		} else {
			ret = 0;
		}
		mutex_unlock(&usbss->lock);
		mutex_unlock(&wcd939x_usbss_fc_lock);
		return ret;
	}

	/* Pre-state: device enabled, DNL/DPR switches off. */
	ret = regmap_write(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
			   WCD_USBSS_SWITCH_SETTINGS_ENABLE_DEVICE_ENABLE);
	if (ret)
		goto out;

	/*
	 * Crossbar select: fast charge routes DNL->DN2, DPR->DR2 (0x28);
	 * normal USB routes DNL->DN, DPR->DP (0x14).
	 */
	ret = regmap_write(usbss->regmap, WCD_USBSS_SWITCH_SELECT0,
			   enable ?
			   (FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
				       WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_DN2) |
			    FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES,
				       WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_DR2)) :
			   (FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DNL_SWITCHES,
				       WCD_USBSS_SWITCH_SELECT0_DNL_SWITCH_DN) |
			    FIELD_PREP(WCD_USBSS_SWITCH_SELECT0_DPR_SWITCHES,
				       WCD_USBSS_SWITCH_SELECT0_DPR_SWITCH_DP)));
	if (ret)
		goto out;

	/* Enable DNL + DPR switches (0x98). */
	ret = regmap_write(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
			   WCD_USBSS_SWITCH_SETTINGS_ENABLE_DEVICE_ENABLE |
			   WCD_USBSS_SWITCH_SETTINGS_ENABLE_DNL_SWITCHES |
			   WCD_USBSS_SWITCH_SETTINGS_ENABLE_DPR_SWITCHES);
	if (ret)
		goto out;

	/*
	 * USB_SS_CNTL: RCO_EN + USB mode, never STANDBY_STATE. The old
	 * behaviour set STANDBY for the "normal USB route" (matching a
	 * misread vendor trace) which standby-ed the whole SS block at
	 * every detach (fc-restore) and deafened the HS path for all
	 * subsequent attaches (register diff A/B: 0x419 0d -> 1d).
	 */
	ret = regmap_write(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
			   WCD_USBSS_USB_SS_CNTL_RCO_EN |
			   FIELD_PREP(WCD_USBSS_USB_SS_CNTL_USB_SS_MODE,
				      WCD_USBSS_USB_SS_CNTL_USB_SS_MODE_USB));
	if (!ret)
		usbss->fc_routed = enable;
out:
	mutex_unlock(&usbss->lock);
	if (ret)
		dev_err(&usbss->client->dev,
			"fast-charge DPDM switch %s failed: %d\n",
			enable ? "on" : "off", ret);
	mutex_unlock(&wcd939x_usbss_fc_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(wcd939x_usbss_dpdm_switch_fast_charge);

static int wcd939x_usbss_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };
	struct wcd939x_usbss *usbss;
	int ret;

	usbss = devm_kzalloc(dev, sizeof(*usbss), GFP_KERNEL);
	if (!usbss)
		return -ENOMEM;

	usbss->client = client;
	mutex_init(&usbss->lock);

	usbss->regmap = devm_regmap_init_i2c(client, &wcd939x_usbss_regmap_config);
	if (IS_ERR(usbss->regmap))
		return dev_err_probe(dev, PTR_ERR(usbss->regmap), "failed to initialize regmap\n");

	usbss->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(usbss->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(usbss->reset_gpio),
				     "unable to acquire reset gpio\n");

	usbss->vdd_supply = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(usbss->vdd_supply))
		return PTR_ERR(usbss->vdd_supply);

	/* Get Codec's MUX & Switch devices */
	usbss->codec = fwnode_typec_mux_get(dev->fwnode);
	if (IS_ERR(usbss->codec))
		return dev_err_probe(dev, PTR_ERR(usbss->codec),
				     "failed to acquire codec mode-switch\n");

	usbss->codec_switch = fwnode_typec_switch_get(dev->fwnode);
	if (IS_ERR(usbss->codec_switch)) {
		ret = dev_err_probe(dev, PTR_ERR(usbss->codec_switch),
				    "failed to acquire codec orientation-switch\n");
		goto err_mux_put;
	}

	usbss->mode = TYPEC_STATE_SAFE;
	usbss->orientation = TYPEC_ORIENTATION_NONE;

	gpiod_set_value(usbss->reset_gpio, 1);

	ret = regulator_enable(usbss->vdd_supply);
	if (ret) {
		dev_err(dev, "Failed to enable vdd: %d\n", ret);
		goto err_mux_switch;
	}

	msleep(20);

	gpiod_set_value(usbss->reset_gpio, 0);

	msleep(20);

	/* Disable standby */
	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
				WCD_USBSS_USB_SS_CNTL_STANDBY_STATE);
	if (ret)
		goto err_regulator_disable;

	/* Set manual mode by default */
	ret = regmap_update_bits(usbss->regmap, WCD_USBSS_FUNCTION_ENABLE,
				 WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT,
				 FIELD_PREP(WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT,
					    WCD_USBSS_FUNCTION_ENABLE_SOURCE_SELECT_MANUAL));
	if (ret)
		goto err_regulator_disable;

	/* Enable dynamic boosting for DP and DN */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_DP_DN_MISC1,
			      WCD_USBSS_DP_DN_MISC1_DP_PCOMP_2X_DYN_BST_ON_EN |
			      WCD_USBSS_DP_DN_MISC1_DN_PCOMP_2X_DYN_BST_ON_EN);
	if (ret)
		goto err_regulator_disable;

	/* Enable dynamic boosting for MG1 OVP */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_MG1_MISC,
			      WCD_USBSS_MG1_MISC_PCOMP_2X_DYN_BST_ON_EN);
	if (ret)
		goto err_regulator_disable;

	/* Enable dynamic boosting for MG2 OVP */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_MG2_MISC,
			      WCD_USBSS_MG2_MISC_PCOMP_2X_DYN_BST_ON_EN);
	if (ret)
		goto err_regulator_disable;

	/* Write 0xFF to WCD_USBSS_CPLDO_CTL2 */
	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_CPLDO_CTL2, 0xff);
	if (ret)
		goto err_regulator_disable;

	/* Set RCO_EN: WCD_USBSS_USB_SS_CNTL Bit<3> --> 0x0 --> 0x1 */
	ret = regmap_clear_bits(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
				WCD_USBSS_USB_SS_CNTL_RCO_EN);
	if (ret)
		goto err_regulator_disable;

	ret = regmap_set_bits(usbss->regmap, WCD_USBSS_USB_SS_CNTL,
			      WCD_USBSS_USB_SS_CNTL_RCO_EN);
	if (ret)
		goto err_regulator_disable;

	/* Disable all switches but enable the mux */
	ret = regmap_write(usbss->regmap, WCD_USBSS_SWITCH_SETTINGS_ENABLE,
			   WCD_USBSS_SWITCH_SETTINGS_ENABLE_DEVICE_ENABLE);
	if (ret)
		goto err_regulator_disable;

	/* Setup in SAFE mode */
	ret = wcd939x_usbss_set(usbss);
	if (ret)
		goto err_regulator_disable;

	sw_desc.drvdata = usbss;
	sw_desc.fwnode = dev_fwnode(dev);
	sw_desc.set = wcd939x_usbss_switch_set;

	usbss->sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(usbss->sw)) {
		ret = dev_err_probe(dev, PTR_ERR(usbss->sw), "failed to register typec switch\n");
		goto err_regulator_disable;
	}

	mux_desc.drvdata = usbss;
	mux_desc.fwnode = dev_fwnode(dev);
	mux_desc.set = wcd939x_usbss_mux_set;

	usbss->mux = typec_mux_register(dev, &mux_desc);
	if (IS_ERR(usbss->mux)) {
		ret = dev_err_probe(dev, PTR_ERR(usbss->mux), "failed to register typec mux\n");
		goto err_switch_unregister;
	}

	i2c_set_clientdata(client, usbss);

	mutex_lock(&wcd939x_usbss_fc_lock);
	wcd939x_usbss_fc_dev = usbss;
	mutex_unlock(&wcd939x_usbss_fc_lock);

	return 0;

err_switch_unregister:
	typec_switch_unregister(usbss->sw);

err_regulator_disable:
	regulator_disable(usbss->vdd_supply);

err_mux_switch:
	typec_switch_put(usbss->codec_switch);

err_mux_put:
	typec_mux_put(usbss->codec);

	return ret;
}

static void wcd939x_usbss_remove(struct i2c_client *client)
{
	struct wcd939x_usbss *usbss = i2c_get_clientdata(client);

	mutex_lock(&wcd939x_usbss_fc_lock);
	if (wcd939x_usbss_fc_dev == usbss)
		wcd939x_usbss_fc_dev = NULL;
	mutex_unlock(&wcd939x_usbss_fc_lock);

	typec_mux_unregister(usbss->mux);
	typec_switch_unregister(usbss->sw);

	regulator_disable(usbss->vdd_supply);

	typec_switch_put(usbss->codec_switch);
	typec_mux_put(usbss->codec);
}

static const struct i2c_device_id wcd939x_usbss_table[] = {
	{ .name = "wcd9390-usbss" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wcd939x_usbss_table);

static const struct of_device_id wcd939x_usbss_of_table[] = {
	{ .compatible = "qcom,wcd9390-usbss" },
	{ }
};
MODULE_DEVICE_TABLE(of, wcd939x_usbss_of_table);

static struct i2c_driver wcd939x_usbss_driver = {
	.driver = {
		.name = "wcd939x-usbss",
		.of_match_table = wcd939x_usbss_of_table,
	},
	.probe		= wcd939x_usbss_probe,
	.remove		= wcd939x_usbss_remove,
	.id_table	= wcd939x_usbss_table,
};
module_i2c_driver(wcd939x_usbss_driver);

MODULE_DESCRIPTION("Qualcomm WCD939x USBSS driver");
MODULE_LICENSE("GPL");
