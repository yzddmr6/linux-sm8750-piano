/* SPDX-License-Identifier: GPL-2.0-only */
/* Derived from Copyright (C) 2023-2024 Oplus low-current full strategy. */
#ifndef _ERHAI_SVOOC_LCF_H
#define _ERHAI_SVOOC_LCF_H

#include <linux/kernel.h>
#include <linux/jiffies.h>

/* Domestic erhai-common.dtsi svooc_lcf_strategy (24924/24926).
 * Algorithm: OnePlus cc463675855a, oplus_strategy_low_curr_full.c.
 * Positive ma charges the cell. The exit/alarm column is not consulted
 * by the vendor's low-current full decision.
 */
struct erhai_svooc_lcf {
	int region, previous;
	unsigned int temp_count, full_count;
	unsigned long next;
};

static inline int erhai_svooc_lcf_region(int temp)
{
	static const int bounds[] = { 0, 50, 120, 160, 210, 380, 440 };
	unsigned int i;

	for (i = 0; i + 1 < ARRAY_SIZE(bounds); i++)
		if (temp >= bounds[i] && temp < bounds[i + 1])
			return i;
	return 6;
}

static inline void erhai_svooc_lcf_start(struct erhai_svooc_lcf *s, int temp,
				       unsigned long now)
{
	*s = (struct erhai_svooc_lcf){
		.region = erhai_svooc_lcf_region(temp), .previous = 6,
		.next = now + msecs_to_jiffies(1000), /* VOOC_CURR_EVENT_TIME */
	};
}

static inline bool erhai_svooc_lcf_update(struct erhai_svooc_lcf *s, int temp,
					int mv, int ma, unsigned long now)
{
	static const struct { int ma, mv; } rows[6][5] = {
		{ { 600, 4520 }, { 575, 4510 }, { 550, 4500 }, { 525, 4490 }, { 500, 4480 } },
		{ { 1750, 4545 }, { 1700, 4540 }, { 1650, 4525 }, { 1550, 4515 }, { 1450, 4505 } },
		{ { 1750, 4545 }, { 1700, 4540 }, { 1600, 4525 }, { 1500, 4515 }, { 1350, 4505 } },
		{ { 1950, 4545 }, { 1900, 4540 }, { 1800, 4525 }, { 1700, 4515 }, { 1550, 4505 } },
		{ { 1950, 4545 }, { 1900, 4540 }, { 1800, 4525 }, { 1700, 4515 }, { 1550, 4505 } },
		{ { 2600, 4540 }, { 2400, 4510 }, { 2000, 4500 }, { 1500, 4490 }, { 1000, 4480 } },
	};
	int region;
	unsigned int i;
	bool low = false;

	if (time_before(now, s->next))
		return false;
	s->next = now + msecs_to_jiffies(1000);
	region = erhai_svooc_lcf_region(temp);
	/* Preserve the stock two observations of a changed region. */
	if (region != s->previous && ++s->temp_count >= 2) {
		s->previous = s->region = region;
		s->temp_count = 0;
	}
	if (s->region >= ARRAY_SIZE(rows))
		return false;
	for (i = 0; i < ARRAY_SIZE(rows[0]); i++)
		if (ma <= rows[s->region][i].ma && mv >= rows[s->region][i].mv) {
			low = true;
			break;
		}
	if (!low)
		s->full_count = 0;
	else if (++s->full_count > 6) {
		s->full_count = 0;
		return true;
	}
	return false;
}
#endif
