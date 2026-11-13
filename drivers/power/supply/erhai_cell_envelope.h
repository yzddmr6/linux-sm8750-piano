/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ERHAI_CELL_ENVELOPE_H
#define _ERHAI_CELL_ENVELOPE_H
/* OnePlusOSS sm8750 cc463675855a, oplus-chg-erhai-common.dtsi, svooc_charge_strategy. */
#include "erhai_charge_curve.h"

static const struct erhai_charge_step svooc_little_cold_soc20[] = {
	{ 4150, 3800, 0 },
	{ 4150, 3500, 0 },
	{ 4150, 3000, 0 },
	{ 4510, 2600, 0 },
	{ 4510, 2500, 0 },
	{ 4510, 2000, 0 },
	{ 4510, 1500, 0 },
	{ 4520, 1000, 1 },
};

static const struct erhai_charge_step svooc_cool_soc20[] = {
	{ 4150, 4800, 0 },
	{ 4150, 4500, 0 },
	{ 4150, 4000, 0 },
	{ 4150, 3500, 0 },
	{ 4150, 3000, 0 },
	{ 4200, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cool_high_soc20[] = {
	{ 4230, 6100, 0 },
	{ 4230, 5000, 0 },
	{ 4230, 4000, 0 },
	{ 4380, 3900, 0 },
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_normal_high_soc20[] = {
	{ 4200, 6100, 0 },
	{ 4200, 6000, 0 },
	{ 4200, 5000, 0 },
	{ 4230, 4800, 0 },
	{ 4230, 4500, 0 },
	{ 4230, 4000, 0 },
	{ 4380, 3900, 0 },
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1400, 1 },
};

static const struct erhai_charge_step svooc_warm_soc20[] = {
	{ 4120, 3200, 0 },
	{ 4120, 3000, 0 },
	{ 4120, 2500, 0 },
	{ 4120, 2000, 0 },
	{ 4120, 1500, 0 },
	{ 4130, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cold_soc55[] = {
	{ 4510, 2000, 0 },
	{ 4510, 1500, 0 },
	{ 4520, 1000, 1 },
};

static const struct erhai_charge_step svooc_cool_soc55[] = {
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cool_soc55[] = {
	{ 4150, 4000, 0 },
	{ 4150, 3500, 0 },
	{ 4150, 3000, 0 },
	{ 4200, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cool_high_soc55[] = {
	{ 4230, 4000, 0 },
	{ 4380, 3900, 0 },
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cold_soc75[] = {
	{ 4510, 1500, 0 },
	{ 4520, 1000, 1 },
};

static const struct erhai_charge_step svooc_cool_soc75[] = {
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cool_high_soc75[] = {
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_normal_high_soc75[] = {
	{ 4380, 3500, 0 },
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1400, 1 },
};

static const struct erhai_charge_step svooc_warm_soc75[] = {
	{ 4130, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cold_soc85[] = {
	{ 4520, 1000, 1 },
};

static const struct erhai_charge_step svooc_little_cool_soc85[] = {
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_normal_low_soc85[] = {
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_step svooc_normal_high_soc85[] = {
	{ 4380, 3000, 0 },
	{ 4535, 2900, 0 },
	{ 4535, 2500, 0 },
	{ 4535, 2000, 0 },
	{ 4535, 1500, 0 },
	{ 4545, 1400, 1 },
};

static const struct erhai_charge_step svooc_cool_soc90[] = {
	{ 4545, 1000, 1 },
};

static const struct erhai_charge_curve svooc_curves[6][7] = {
	{
		ERHAI_CURVE(svooc_little_cold_soc20),
		ERHAI_CURVE(svooc_cool_soc20),
		ERHAI_CURVE(svooc_cool_soc20),
		ERHAI_CURVE(svooc_little_cool_high_soc20),
		ERHAI_CURVE(svooc_little_cool_high_soc20),
		ERHAI_CURVE(svooc_normal_high_soc20),
		ERHAI_CURVE(svooc_warm_soc20),
	},
	{
		ERHAI_CURVE(svooc_little_cold_soc20),
		ERHAI_CURVE(svooc_cool_soc20),
		ERHAI_CURVE(svooc_cool_soc20),
		ERHAI_CURVE(svooc_little_cool_high_soc20),
		ERHAI_CURVE(svooc_little_cool_high_soc20),
		ERHAI_CURVE(svooc_normal_high_soc20),
		ERHAI_CURVE(svooc_warm_soc20),
	},
	{
		ERHAI_CURVE(svooc_little_cold_soc55),
		ERHAI_CURVE(svooc_cool_soc55),
		ERHAI_CURVE(svooc_little_cool_soc55),
		ERHAI_CURVE(svooc_little_cool_high_soc55),
		ERHAI_CURVE(svooc_little_cool_high_soc20),
		ERHAI_CURVE(svooc_normal_high_soc20),
		ERHAI_CURVE(svooc_warm_soc20),
	},
	{
		ERHAI_CURVE(svooc_little_cold_soc75),
		ERHAI_CURVE(svooc_cool_soc75),
		ERHAI_CURVE(svooc_cool_soc55),
		ERHAI_CURVE(svooc_little_cool_high_soc75),
		ERHAI_CURVE(svooc_little_cool_high_soc75),
		ERHAI_CURVE(svooc_normal_high_soc75),
		ERHAI_CURVE(svooc_warm_soc75),
	},
	{
		ERHAI_CURVE(svooc_little_cold_soc85),
		ERHAI_CURVE(svooc_cool_soc75),
		ERHAI_CURVE(svooc_little_cool_soc85),
		ERHAI_CURVE(svooc_cool_soc55),
		ERHAI_CURVE(svooc_normal_low_soc85),
		ERHAI_CURVE(svooc_normal_high_soc85),
		ERHAI_CURVE(svooc_warm_soc75),
	},
	{
		ERHAI_CURVE(svooc_little_cold_soc85),
		ERHAI_CURVE(svooc_cool_soc90),
		ERHAI_CURVE(svooc_cool_soc75),
		ERHAI_CURVE(svooc_little_cool_soc85),
		ERHAI_CURVE(svooc_little_cool_soc85),
		ERHAI_CURVE(svooc_cool_soc55),
		ERHAI_CURVE(svooc_warm_soc75),
	},
};

static inline int erhai_cell_cv(int temp)
{
	/* oplus_spec,full_voltage: sustained (second) threshold per region. */
	if (temp >= 440)
		return 4130;
	if (temp < 50)
		return 4520;
	if (temp >= 210 && temp < 350)
		return 4550;
	return 4545;
}

/* The board table encodes 2:1 bus mA. Other converters apply the
 * corresponding battery-side limit, regardless of negotiated voltage.
 */
static inline const struct erhai_charge_step *
erhai_cell_entry(int soc, int temp, int mv)
{
	const struct erhai_charge_curve *curve = erhai_curve_select(svooc_curves, soc, temp);

	if (!curve || mv <= 0 || mv >= erhai_cell_cv(temp))
		return NULL;
	return &curve->steps[erhai_curve_start(curve, mv, true)];
}
#endif
