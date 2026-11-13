/* SPDX-License-Identifier: GPL-2.0-only */
/* OnePlusOSS sm8750 cc463675855a, oplus-chg-24924.dtsi, pps_charge_third_strategy. */
#include "erhai_charge_curve.h"

static const struct erhai_charge_step pps_little_cold_soc20[] = {
	{ 4150, 3000, 0 },
	{ 4480, 2650, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_cool_soc20[] = {
	{ 4150, 3000, 0 },
	{ 4200, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cool_high_soc20[] = {
	{ 4380, 3000, 0 },
	{ 4480, 2950, 0 },
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc20[] = {
	{ 4380, 3000, 0 },
	{ 4460, 2950, 0 },
	{ 4460, 2500, 0 },
	{ 4460, 2000, 0 },
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_warm_soc20[] = {
	{ 4120, 3000, 0 },
	{ 4120, 2500, 0 },
	{ 4120, 2000, 0 },
	{ 4120, 1500, 0 },
	{ 4130, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cold_soc55[] = {
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_cool_soc55[] = {
	{ 4480, 2500, 0 },
	{ 4480, 2000, 0 },
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cold_soc75[] = {
	{ 4480, 1500, 0 },
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_warm_soc75[] = {
	{ 4130, 1000, 1 },
};

static const struct erhai_charge_step pps_little_cold_soc85[] = {
	{ 4490, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc85[] = {
	{ 4460, 2500, 0 },
	{ 4460, 2000, 0 },
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_step pps_normal_high_soc90[] = {
	{ 4460, 1500, 0 },
	{ 4470, 1000, 1 },
};

static const struct erhai_charge_curve pps_curves[6][7] = {
	{
		ERHAI_CURVE(pps_little_cold_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_cool_soc55),
		ERHAI_CURVE(pps_cool_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_little_cool_high_soc20),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc20),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_cool_soc55),
		ERHAI_CURVE(pps_normal_high_soc20),
		ERHAI_CURVE(pps_warm_soc75),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc75),
		ERHAI_CURVE(pps_little_cold_soc55),
		ERHAI_CURVE(pps_normal_high_soc85),
		ERHAI_CURVE(pps_warm_soc75),
	},
	{
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc85),
		ERHAI_CURVE(pps_little_cold_soc75),
		ERHAI_CURVE(pps_normal_high_soc90),
		ERHAI_CURVE(pps_warm_soc75),
	},
};
