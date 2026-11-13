/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ERHAI_CHARGE_CURVE_H
#define _ERHAI_CHARGE_CURVE_H

#include <linux/kernel.h>

struct erhai_charge_step {
	u16 mv;
	u16 ma;		/* bus current, before adapter and cable limits */
	bool last;
};

struct erhai_charge_curve {
	const struct erhai_charge_step *steps;
	u8 count;
};

#define ERHAI_CURVE(rows) { rows, ARRAY_SIZE(rows) }

/* Project 24926 includes oplus-chg-24924.dtsi and erhai-common.
 * soc_range upper bounds are inclusive, temp_range bounds exclusive.
 * Above the final SOC boundary keep the last curve until physical CV.
 * Use gauge temperature in deci-C regardless of display state.
 */
static inline const struct erhai_charge_curve *
erhai_curve_select(const struct erhai_charge_curve curves[][7],
		   int soc, int temp)
{
	static const int soc_end[] = { 20, 35, 55, 75, 85, 90 };
	static const int temp_end[] = { 50, 120, 160, 210, 350, 440, 510 };
	unsigned int s, t;

	if (soc < 0 || soc > 100 || temp < 0)
		return NULL;
	for (s = 0; s + 1 < ARRAY_SIZE(soc_end); s++)
		if (soc <= soc_end[s])
			break;
	for (t = 0; t < ARRAY_SIZE(temp_end); t++)
		if (temp < temp_end[t])
			break;
	if (t == ARRAY_SIZE(temp_end))
		return NULL;
	return &curves[s][t];
}

/* PPS starts below its voltage target, SUPERVOOC also accepts equality. */
static inline unsigned int erhai_curve_start(const struct erhai_charge_curve *curve,
					    int mv, bool inclusive)
{
	unsigned int i;

	for (i = 0; i + 1 < curve->count; i++)
		if (mv < curve->steps[i].mv || (inclusive && mv == curve->steps[i].mv))
			break;
	return i;
}
#endif
