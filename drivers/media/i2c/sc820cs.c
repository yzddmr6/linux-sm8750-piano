// SPDX-License-Identifier: GPL-2.0
/*
 * SmartSens SC820CS camera sensor
 *
 * Based on TheMojoMan's sc820cs driver in
 * https://github.com/TheMojoMan/oneplus-erhai-linux
 * revision 74ba8ddb4dc8fd930841b6802053b4362b59ce46.
 * Register sequences originate from Android bus captures, not a datasheet.
 * The original attribution is retained:
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Exposure and gain encoding is based on the GPL-2.0 vendor sensor driver:
 * Copyright (C) 2018 MediaTek Inc.
 * https://github.com/oppo-source/android_kernel_oppo_mt6835
 * revision 9c93b939b3198a82775bbe3fb74f6e647b533825.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SC820CS_CHIP_ID		0xd154
#define SC820CS_WIDTH		3264
#define SC820CS_HEIGHT		2448
#define SC820CS_HTS		1760
/* Two output pixels per 132 MHz timing-clock tick, independent of MIPI. */
#define SC820CS_LINE_LENGTH	(SC820CS_HTS * 2)
#define SC820CS_VTS		2500
#define SC820CS_VTS_MAX		0x7fff
#define SC820CS_EXPOSURE_MARGIN	4
#define SC820CS_PIXEL_RATE	264000000
#define SC820CS_XCLK_RATE	24000000

static const s64 sc820cs_link_freqs[] = { 368000000 };
/* Power-on order: I/O, digital core, analogue. */
static const char * const sc820cs_supply_names[] = { "dovdd", "dvdd", "avdd" };

struct sc820cs {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[3];
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
};

static inline struct sc820cs *to_sc820cs(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sc820cs, sd);
}

/*
 * Native RAW10 register sequence from the Android sensor library's bus
 * traffic, as published with the sc820cs driver. Reset and stream-on
 * are issued separately. Timing and exposure controls replace the captured
 * automatic-exposure values before streaming starts.
 */
static const struct cci_reg_sequence sc820cs_mode[] = {
	{ CCI_REG8(0x36e9), 0x80 },
	{ CCI_REG8(0x37f9), 0x80 },
	{ CCI_REG8(0x36e9), 0x24 },
	{ CCI_REG8(0x37f9), 0x24 },
	{ CCI_REG8(0x301f), 0x0e },
	{ CCI_REG8(0x3205), 0xc7 },
	{ CCI_REG8(0x3211), 0x04 },
	{ CCI_REG8(0x3270), 0x00 },
	{ CCI_REG8(0x3271), 0x00 },
	{ CCI_REG8(0x3272), 0x00 },
	{ CCI_REG8(0x3273), 0x03 },
	{ CCI_REG8(0x3301), 0x08 },
	{ CCI_REG8(0x3303), 0x10 },
	{ CCI_REG8(0x3306), 0x84 },
	{ CCI_REG8(0x3307), 0x04 },
	{ CCI_REG8(0x3309), 0x88 },
	{ CCI_REG8(0x330a), 0x01 },
	{ CCI_REG8(0x330b), 0x0c },
	{ CCI_REG8(0x330c), 0x10 },
	{ CCI_REG8(0x330d), 0x18 },
	{ CCI_REG8(0x330e), 0x30 },
	{ CCI_REG8(0x330f), 0x04 },
	{ CCI_REG8(0x3310), 0x02 },
	{ CCI_REG8(0x3314), 0x15 },
	{ CCI_REG8(0x3317), 0x04 },
	{ CCI_REG8(0x331f), 0x79 },
	{ CCI_REG8(0x3326), 0x0e },
	{ CCI_REG8(0x3327), 0x0a },
	{ CCI_REG8(0x3329), 0x0b },
	{ CCI_REG8(0x3333), 0x10 },
	{ CCI_REG8(0x3334), 0x40 },
	{ CCI_REG8(0x3347), 0x0f },
	{ CCI_REG8(0x335d), 0x60 },
	{ CCI_REG8(0x3364), 0x56 },
	{ CCI_REG8(0x336c), 0xce },
	{ CCI_REG8(0x3390), 0x08 },
	{ CCI_REG8(0x3391), 0x09 },
	{ CCI_REG8(0x3392), 0x0f },
	{ CCI_REG8(0x3393), 0x10 },
	{ CCI_REG8(0x3394), 0x20 },
	{ CCI_REG8(0x3395), 0x28 },
	{ CCI_REG8(0x33ad), 0x3c },
	{ CCI_REG8(0x33af), 0x70 },
	{ CCI_REG8(0x33b2), 0x70 },
	{ CCI_REG8(0x33b3), 0x40 },
	{ CCI_REG8(0x349f), 0x1e },
	{ CCI_REG8(0x34a6), 0x09 },
	{ CCI_REG8(0x34a7), 0x0f },
	{ CCI_REG8(0x34a8), 0x30 },
	{ CCI_REG8(0x34a9), 0x20 },
	{ CCI_REG8(0x34f8), 0x1f },
	{ CCI_REG8(0x34f9), 0x08 },
	{ CCI_REG8(0x3637), 0x43 },
	{ CCI_REG8(0x363c), 0x8d },
	{ CCI_REG8(0x3650), 0x21 },
	{ CCI_REG8(0x3651), 0x9f },
	{ CCI_REG8(0x3670), 0x4a },
	{ CCI_REG8(0x3674), 0xf6 },
	{ CCI_REG8(0x3675), 0xdc },
	{ CCI_REG8(0x3676), 0xcc },
	{ CCI_REG8(0x367c), 0x09 },
	{ CCI_REG8(0x367d), 0x0f },
	{ CCI_REG8(0x3690), 0x34 },
	{ CCI_REG8(0x3691), 0x44 },
	{ CCI_REG8(0x3692), 0x55 },
	{ CCI_REG8(0x3698), 0x86 },
	{ CCI_REG8(0x3699), 0x8d },
	{ CCI_REG8(0x369a), 0x99 },
	{ CCI_REG8(0x369b), 0xb7 },
	{ CCI_REG8(0x369c), 0x0f },
	{ CCI_REG8(0x369d), 0x1f },
	{ CCI_REG8(0x36a2), 0x09 },
	{ CCI_REG8(0x36a3), 0x0b },
	{ CCI_REG8(0x36a4), 0x0f },
	{ CCI_REG8(0x36b0), 0x48 },
	{ CCI_REG8(0x36b1), 0x38 },
	{ CCI_REG8(0x36b2), 0x41 },
	{ CCI_REG8(0x370f), 0x01 },
	{ CCI_REG8(0x3724), 0xc1 },
	{ CCI_REG8(0x3771), 0x07 },
	{ CCI_REG8(0x3772), 0x03 },
	{ CCI_REG8(0x3773), 0x63 },
	{ CCI_REG8(0x377a), 0x08 },
	{ CCI_REG8(0x377b), 0x0f },
	{ CCI_REG8(0x3901), 0x04 },
	{ CCI_REG8(0x3903), 0xa0 },
	{ CCI_REG8(0x3905), 0x8d },
	{ CCI_REG8(0x391d), 0x01 },
	{ CCI_REG8(0x3926), 0x23 },
	{ CCI_REG8(0x393f), 0x80 },
	{ CCI_REG8(0x3940), 0x00 },
	{ CCI_REG8(0x3941), 0x00 },
	{ CCI_REG8(0x3942), 0x00 },
	{ CCI_REG8(0x3943), 0x63 },
	{ CCI_REG8(0x3944), 0x5f },
	{ CCI_REG8(0x3c04), 0x01 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x38 },
	{ CCI_REG8(0x3e02), 0x00 },
	{ CCI_REG8(0x4401), 0x13 },
	{ CCI_REG8(0x4402), 0x03 },
	{ CCI_REG8(0x4403), 0x0e },
	{ CCI_REG8(0x4404), 0x28 },
	{ CCI_REG8(0x4405), 0x34 },
	{ CCI_REG8(0x4407), 0x0e },
	{ CCI_REG8(0x440c), 0x42 },
	{ CCI_REG8(0x440d), 0x42 },
	{ CCI_REG8(0x440e), 0x32 },
	{ CCI_REG8(0x440f), 0x53 },
	{ CCI_REG8(0x4412), 0x01 },
	{ CCI_REG8(0x4424), 0x01 },
	{ CCI_REG8(0x442d), 0x00 },
	{ CCI_REG8(0x442e), 0x00 },
	{ CCI_REG8(0x4509), 0x28 },
	{ CCI_REG8(0x450d), 0x18 },
	{ CCI_REG8(0x451d), 0xc8 },
	{ CCI_REG8(0x4526), 0x09 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x550e), 0x00 },
	{ CCI_REG8(0x550f), 0xbc },
	{ CCI_REG8(0x5780), 0x66 },
	{ CCI_REG8(0x578d), 0x40 },
	{ CCI_REG8(0x57aa), 0xeb },
	/* No flips: the readout starts with the R pixel of the RGGB array. */
	{ CCI_REG8(0x3221), 0x00 },
	{ CCI_REG8(0x301f), 0x0e },
	{ CCI_REG8(0x3200), 0x00 },
	{ CCI_REG8(0x3201), 0x00 },
	{ CCI_REG8(0x3202), 0x00 },
	{ CCI_REG8(0x3203), 0x00 },
	{ CCI_REG8(0x3204), 0x0c },
	{ CCI_REG8(0x3206), 0x09 },
	{ CCI_REG8(0x3207), 0x9f },
	{ CCI_REG8(0x3208), 0x0c },
	{ CCI_REG8(0x3209), 0xc0 },
	{ CCI_REG8(0x320a), 0x09 },
	{ CCI_REG8(0x320b), 0x90 },
	{ CCI_REG8(0x3210), 0x00 },
	{ CCI_REG8(0x3212), 0x00 },
	{ CCI_REG8(0x3213), 0x08 },
	{ CCI_REG8(0x3215), 0x11 },
	{ CCI_REG8(0x3220), 0x00 },
	{ CCI_REG8(0x5000), 0x0e },
	{ CCI_REG8(0x5900), 0x01 },
	{ CCI_REG8(0x5901), 0x00 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x326d), 0x00 },
	{ CCI_REG8(0x320e), 0x0c },
	{ CCI_REG8(0x320f), 0x38 },
	{ CCI_REG8(0x3e20), 0x00 },
	{ CCI_REG8(0x3e00), 0x01 },
	{ CCI_REG8(0x3e01), 0x86 },
	{ CCI_REG8(0x3e02), 0x80 },
	{ CCI_REG8(0x3e08), 0x0f },
	{ CCI_REG8(0x3e07), 0x80 },
	{ CCI_REG8(0x301f), 0x00 },
	{ CCI_REG8(0x3337), 0x00 },
	{ CCI_REG8(0x391f), 0xc9 },
};

static int sc820cs_power_on(struct device *dev)
{
	struct sc820cs *sensor = to_sc820cs(dev_get_drvdata(dev));
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret)
			goto disable_supplies;
		usleep_range(1000, 1500);
	}

	/* Keep reset asserted until the input clock is running. */
	ret = clk_prepare_enable(sensor->xclk);
	if (ret)
		goto disable_supplies;
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(sensor->reset, 0);
	usleep_range(10000, 12000);

	return 0;

disable_supplies:
	while (i--) {
		regulator_disable(sensor->supplies[i].consumer);
		usleep_range(1000, 1500);
	}
	return ret;
}

static int sc820cs_power_off(struct device *dev)
{
	struct sc820cs *sensor = to_sc820cs(dev_get_drvdata(dev));
	int i;

	clk_disable_unprepare(sensor->xclk);
	usleep_range(5000, 5500);
	gpiod_set_value_cansleep(sensor->reset, 1);
	usleep_range(1000, 1500);
	for (i = ARRAY_SIZE(sensor->supplies) - 1; i >= 0; i--) {
		regulator_disable(sensor->supplies[i].consumer);
		usleep_range(1000, 1500);
	}
	return 0;
}

static int sc820cs_set_exposure(struct sc820cs *sensor, unsigned int lines)
{
	/* Exposure is encoded in half-lines in bits 19:4. */
	return cci_write(sensor->regmap, CCI_REG24(0x3e00), lines << 5, NULL);
}

static int sc820cs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc820cs *sensor = container_of(ctrl->handler, struct sc820cs, ctrls);
	static const u8 coarse_gain[] = { 0x00, 0x08, 0x09, 0x0b, 0x0f };
	unsigned int stage;
	int ret, exposure_max;

	if (ctrl->id == V4L2_CID_VBLANK) {
		exposure_max = SC820CS_HEIGHT + ctrl->val - SC820CS_EXPOSURE_MARGIN;
		ret = __v4l2_ctrl_modify_range(sensor->exposure, 2, exposure_max,
					       1, min(exposure_max, 1000));
		if (ret)
			return ret;
	}

	ret = pm_runtime_get_if_in_use(sensor->dev);
	if (ret <= 0)
		return ret;
	ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = sc820cs_set_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		/* Clamp the shutter before shortening the frame. */
		ret = sc820cs_set_exposure(sensor, sensor->exposure->val);
		cci_write(sensor->regmap, CCI_REG16(0x320e),
			  SC820CS_HEIGHT + ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		/* Vendor gain encoding: 1024 represents unity. */
		stage = fls(ctrl->val / 1024) - 1;
		cci_write(sensor->regmap, CCI_REG8(0x3e08), coarse_gain[stage], &ret);
		cci_write(sensor->regmap, CCI_REG8(0x3e07),
			  (ctrl->val >> stage) >> 3, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	pm_runtime_put(sensor->dev);
	return ret;
}

static const struct v4l2_ctrl_ops sc820cs_ctrl_ops = {
	.s_ctrl = sc820cs_set_ctrl,
};

static int sc820cs_init_controls(struct sc820cs *sensor)
{
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      sc820cs_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, SC820CS_PIXEL_RATE,
			  SC820CS_PIXEL_RATE, 1, SC820CS_PIXEL_RATE);
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK,
				 SC820CS_LINE_LENGTH - SC820CS_WIDTH,
				 SC820CS_LINE_LENGTH - SC820CS_WIDTH, 1,
				 SC820CS_LINE_LENGTH - SC820CS_WIDTH);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->vblank = v4l2_ctrl_new_std(hdl, &sc820cs_ctrl_ops, V4L2_CID_VBLANK,
					 SC820CS_VTS - SC820CS_HEIGHT,
					 SC820CS_VTS_MAX - SC820CS_HEIGHT, 1,
					 SC820CS_VTS - SC820CS_HEIGHT);
	sensor->exposure = v4l2_ctrl_new_std(hdl, &sc820cs_ctrl_ops, V4L2_CID_EXPOSURE,
					   2, SC820CS_VTS - SC820CS_EXPOSURE_MARGIN,
					   1, 1000);
	v4l2_ctrl_new_std(hdl, &sc820cs_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  1024, 16384, 8, 1024);
	if (hdl->error) {
		ret = hdl->error;
		goto free_controls;
	}
	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		goto free_controls;
	ret = v4l2_ctrl_new_fwnode_properties(hdl, &sc820cs_ctrl_ops, &props);
	if (ret)
		goto free_controls;
	sensor->sd.ctrl_handler = hdl;
	return 0;

free_controls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int sc820cs_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct sc820cs *sensor = to_sc820cs(sd);
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		return ret;
	ret = cci_write(sensor->regmap, CCI_REG8(0x0103), 1, NULL);
	if (ret)
		goto power_off;
	usleep_range(10000, 11000);
	ret = cci_multi_reg_write(sensor->regmap, sc820cs_mode,
				  ARRAY_SIZE(sc820cs_mode), NULL);
	cci_write(sensor->regmap, CCI_REG16(0x320c), SC820CS_HTS, &ret);
	if (ret)
		goto power_off;
	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto power_off;
	ret = cci_write(sensor->regmap, CCI_REG8(0x0100), 1, NULL);
	if (!ret)
		return 0;
power_off:
	pm_runtime_put_sync(sensor->dev);
	return ret;
}

static int sc820cs_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct sc820cs *sensor = to_sc820cs(sd);
	int ret;

	ret = cci_write(sensor->regmap, CCI_REG8(0x0100), 0, NULL);
	pm_runtime_put_sync(sensor->dev);
	return ret;
}

static int sc820cs_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && sd->enabled_pads)
		return -EBUSY;

	fmt->format = (struct v4l2_mbus_framefmt) {
		.width = SC820CS_WIDTH,
		.height = SC820CS_HEIGHT,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_RAW,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_FULL_RANGE,
		.xfer_func = V4L2_XFER_FUNC_NONE,
	};
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = (struct v4l2_rect) {
		.width = SC820CS_WIDTH,
		.height = SC820CS_HEIGHT,
	};
	return 0;
}

static int sc820cs_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = { .pad = 0 };

	return sc820cs_set_format(sd, state, &fmt);
}

static int sc820cs_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	return 0;
}

static int sc820cs_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;
	fse->min_width = fse->max_width = SC820CS_WIDTH;
	fse->min_height = fse->max_height = SC820CS_HEIGHT;
	return 0;
}

static int sc820cs_get_selection(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_selection *sel)
{
	if (sel->pad)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_subdev_core_ops sc820cs_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops sc820cs_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sc820cs_pad_ops = {
	.enum_mbus_code = sc820cs_enum_mbus_code,
	.enum_frame_size = sc820cs_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sc820cs_set_format,
	.get_selection = sc820cs_get_selection,
	.enable_streams = sc820cs_enable_streams,
	.disable_streams = sc820cs_disable_streams,
};

static const struct v4l2_subdev_ops sc820cs_subdev_ops = {
	.core = &sc820cs_core_ops,
	.video = &sc820cs_video_ops,
	.pad = &sc820cs_pad_ops,
};

static const struct v4l2_subdev_internal_ops sc820cs_internal_ops = {
	.init_state = sc820cs_init_state,
};

static int sc820cs_check_endpoint(struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	unsigned int i;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return -EINVAL;
	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		goto free_endpoint;
	ret = -EINVAL;
	if (ep.bus.mipi_csi2.num_data_lanes != 4 ||
	    ep.bus.mipi_csi2.clock_lane ||
	    ep.nr_of_link_frequencies != 1 ||
	    ep.link_frequencies[0] != sc820cs_link_freqs[0])
		goto free_endpoint;
	for (i = 0; i < 4; i++)
		if (ep.bus.mipi_csi2.data_lanes[i] != i + 1)
			goto free_endpoint;
	ret = 0;
free_endpoint:
	v4l2_fwnode_endpoint_free(&ep);
	return ret;
}

static int sc820cs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc820cs *sensor;
	u64 id;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	sensor->dev = dev;
	ret = sc820cs_check_endpoint(dev);
	if (ret)
		return dev_err_probe(dev, ret, "invalid CSI-2 endpoint\n");
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);
	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++)
		sensor->supplies[i].supply = sc820cs_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies), sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");
	sensor->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->xclk))
		return dev_err_probe(dev, PTR_ERR(sensor->xclk), "failed to get clock\n");
	if (clk_get_rate(sensor->xclk) != SC820CS_XCLK_RATE)
		return dev_err_probe(dev, -EINVAL, "24 MHz clock required\n");
	sensor->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(dev, PTR_ERR(sensor->reset), "failed to get reset\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &sc820cs_subdev_ops);
	sensor->sd.internal_ops = &sc820cs_internal_ops;
	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = sc820cs_power_on(dev);
	if (ret)
		return ret;
	/* The sensor may NACK while leaving reset; bound the readiness wait. */
	for (i = 0; i < 8; i++) {
		ret = cci_read(sensor->regmap, CCI_REG16(0x3107), &id, NULL);
		if (!ret)
			break;
		if (i < 7)
			msleep(20);
	}
	if (ret)
		goto power_off;
	if (id != SC820CS_CHIP_ID) {
		ret = dev_err_probe(dev, -ENODEV, "unexpected chip ID: %#llx\n", id);
		goto power_off;
	}
	dev_dbg(dev, "chip ID: %#llx\n", id);
	ret = sc820cs_init_controls(sensor);
	if (ret)
		goto power_off;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto free_controls;
	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto cleanup_entity;

	ret = pm_runtime_set_active(dev);
	if (ret)
		goto cleanup_subdev;
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto disable_pm;
	pm_runtime_put(dev);
	return 0;

disable_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);
cleanup_subdev:
	v4l2_subdev_cleanup(&sensor->sd);
cleanup_entity:
	media_entity_cleanup(&sensor->sd.entity);
free_controls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
power_off:
	sc820cs_power_off(dev);
	return ret;
}

static void sc820cs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc820cs *sensor = to_sc820cs(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		sc820cs_power_off(sensor->dev);
	pm_runtime_set_suspended(sensor->dev);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

static const struct dev_pm_ops sc820cs_pm_ops = {
	RUNTIME_PM_OPS(sc820cs_power_off, sc820cs_power_on, NULL)
};

static const struct of_device_id sc820cs_of_match[] = {
	{ .compatible = "smartsens,sc820cs" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc820cs_of_match);

static struct i2c_driver sc820cs_driver = {
	.driver = {
		.name = "sc820cs",
		.of_match_table = sc820cs_of_match,
		.pm = pm_ptr(&sc820cs_pm_ops),
	},
	.probe = sc820cs_probe,
	.remove = sc820cs_remove,
};
module_i2c_driver(sc820cs_driver);

MODULE_AUTHOR("TheMojoMan <themojoman@gmx.net>");
MODULE_DESCRIPTION("SmartSens SC820CS camera sensor driver");
MODULE_LICENSE("GPL");
