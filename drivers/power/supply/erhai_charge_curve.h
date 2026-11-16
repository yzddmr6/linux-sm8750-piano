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

#define ERHAI_CURVE_TEMP_END { 50, 120, 160, 210, 350, 440, 510 }
#define ERHAI_CURVE_TEMP_WARM	6	/* 44-51 C */

/* Temperature column of erhai_curve_select(); 7 is above the table. */
static inline unsigned int erhai_curve_temp_region(int temp)
{
	static const int temp_end[] = ERHAI_CURVE_TEMP_END;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(temp_end); i++)
		if (temp < temp_end[i])
			break;
	return i;
}

/*
 * The PPS temperature ranges of oplus_chg_pps.c (cc463675855a). A range is
 * left on the PPS_TEMP_OVER_COUNTS (second) sample beyond its boundary. The
 * first such sample moves that boundary PPS_TEMP_LOW_RANGE_THD (one degree)
 * back into the range, and entering a range moves the boundary just crossed
 * one degree into the range that was left. The other boundary keeps its
 * table value. @dir is how the range was entered (+1 from below, -1 from
 * above, 0 at session entry) and @pending the direction counted so far.
 * The return value is the temperature that selects the curve; the caller
 * still applies the live temperature's tighter limits at once.
 */
static inline int erhai_curve_temp(int last, int temp, int *dir, int *pending,
				   unsigned int *samples)
{
	static const int temp_end[] = ERHAI_CURVE_TEMP_END;
	unsigned int cur = erhai_curve_temp_region(last);
	int step;

	/* Outside the table the live temperature ends the session anyway. */
	if (cur == ARRAY_SIZE(temp_end))
		return temp;

	step = 0;
	if (cur && temp < temp_end[cur - 1] + (*pending < 0 ? 10 : *dir > 0 ? -10 : 0))
		step = -1;
	else if (temp >= temp_end[cur] + (*pending > 0 ? -10 : *dir < 0 ? 10 : 0))
		step = 1;
	if (!step)
		return last;

	if (*pending != step) {
		*pending = step;
		*samples = 0;
	}
	if (++*samples < 2)
		return last;

	*pending = 0;
	*samples = 0;
	*dir = step;
	/* A sample just inside the moved boundary still enters the next range. */
	return step > 0 ? max(temp, temp_end[cur]) : min(temp, temp_end[cur - 1] - 1);
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
