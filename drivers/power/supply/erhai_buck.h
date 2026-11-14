/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ERHAI_BUCK_H
#define _ERHAI_BUCK_H

#include <linux/kernel.h>
#include <linux/jiffies.h>

/* Domestic project 24926 / 24924, oplus-chg-erhai-common.dtsi.
 * Port of oplus_chg_comm.c switch_ffc, ffc_start_work, check_ffc,
 * check_sw_full, check_hw_full and check_rechg (OnePlus cc463675855a).
 * Currents here use the power_supply convention: positive charges.
 */
/* oplus_chg_comm.c VBAT_COLD_WARM_COMP, used by check_allow_set_fv. */
#define ERHAI_BUCK_WARM_FV_MARGIN_MV 10

enum erhai_buck_phase {
	ERHAI_BUCK_OFF,
	ERHAI_BUCK_NORMAL,
	ERHAI_FFC_WAIT,
	ERHAI_FFC_FAST,
	ERHAI_FFC_IDLE,
	ERHAI_BUCK_FULL,
};

struct erhai_buck_sample {
	int mv, ma, temp, soc, cycles;
	bool qualified;
};

struct erhai_buck {
	enum erhai_buck_phase phase;
	bool ffc_used, disable, recharged, over_voltage;
	unsigned int step, voltage_count, current_count;
	unsigned int sw_count, hw_count, recharge_count, over_count;
	int temp_region, ffc_region, temp_boundary, temp_offset;
	int fv_mv, normal_fv_mv, cutoff_mv, cutoff_ma, fcc_ma, recharge_mv;
	int fv_decrement;
	unsigned long next_sample, wait_until;
};

/* batt-them-thr has nine V3P7 regions; the V3P7.1 enum inserts an
 * unused little-cold-high region. ADSP FULL_CURR_SET uses that enum.
 */
static inline int erhai_buck_region(int temp)
{
	static const int bounds[] = { -100, 0, 50, 120, 160, 350, 450, 530 };
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(bounds); i++)
		if (temp < bounds[i])
			break;
	return i;
}

static inline int erhai_buck_age_mv(int cycles)
{
	/* dec-vol-cc-full-thr / dec-vol-fv-full-mv, including 1200 -> 40. */
	return cycles >= 800 ? 40 : cycles >= 400 ? 20 : 0;
}

static inline void erhai_buck_temperature(struct erhai_buck *b, int temp)
{
	static const int bounds[] = { -100, 0, 50, 120, 160, 350, 450, 530 };
	int region, boundary, previous_enum, next_enum;

	for (region = 0; region < ARRAY_SIZE(bounds); region++) {
		boundary = bounds[region];
		if (b->temp_offset && region == b->temp_boundary)
			boundary += b->temp_offset;
		if (temp < boundary)
			break;
	}
	if (region == b->temp_region)
		return;
	/* oplus_comm_temp_thr_update: only adjacent transitions get hysteresis. */
	b->temp_offset = 0;
	previous_enum = b->temp_region >= 2 ? b->temp_region + 1 : b->temp_region;
	next_enum = region >= 2 ? region + 1 : region;
	if (next_enum + 1 == previous_enum && region < 7 && b->temp_region < 8) {
		b->temp_boundary = region;
		b->temp_offset = 20;
	} else if (next_enum == previous_enum + 1 && region > 1) {
		b->temp_boundary = b->temp_region;
		b->temp_offset = -20;
	}
	b->temp_region = region;
	b->fv_decrement = 0;
}

static inline int erhai_ffc_region(int previous, int temp)
{
	static const int bounds[] = { 120, 210, 350, 440 };
	int i, bound;

	for (i = 0; i < ARRAY_SIZE(bounds); i++) {
		bound = bounds[i];
		if (i == previous - 1 && previous > 3)
			bound -= 20;
		else if (i == previous && previous < 3)
			bound += 20;
		if (temp < bound)
			break;
	}
	return i;
}

static inline void erhai_buck_normal_limits(struct erhai_buck *b,
					  const struct erhai_buck_sample *s)
{
	static const int fv[] = { 4490, 4490, 4490, 4490, 4490, 4490, 4470, 4130, 4130 };
	static const int sw[] = { 4440, 4440, 4440, 4440, 4440, 4440, 4420, 4080, 4080 };
	static const int recharge[] = { 0, 4190, 4290, 4390, 4390, 4390, 4370, 4030, 0 };
	int region = b->temp_region;
	int age = erhai_buck_age_mv(s->cycles);

	b->normal_fv_mv = s->qualified ? fv[region] - age : 3980;
	b->fv_mv = b->normal_fv_mv - b->fv_decrement;
	b->cutoff_mv = s->qualified ? sw[region] - age : 3930;
	b->cutoff_ma = 597;
	b->recharge_mv = s->qualified ? recharge[region] - age : 3680;
	b->fcc_ma = s->qualified ? 0 : 500; /* zero releases the FFC FCC vote */
	b->disable = region == 0 || region == 8 || b->over_voltage;
	/* check_allow_set_fv: warm battery above FV must not be charged
	 * against the old higher FV while the PMIC power path is retained.
	 */
	if (region >= 7 && s->mv > b->normal_fv_mv)
		b->disable = true;
}

static inline void erhai_buck_start(struct erhai_buck *b, unsigned long now)
{
	b->phase = ERHAI_BUCK_NORMAL;
	b->temp_region = -1;
	b->ffc_region = -1;
	b->recharged = false;
	b->next_sample = now;
}

static inline void erhai_buck_pump_done(struct erhai_buck *b, unsigned long now)
{
	/* FASTCHG_VOTER remains an output-disable vote until ffc_start_work. */
	b->phase = b->ffc_used ? ERHAI_BUCK_NORMAL : ERHAI_FFC_WAIT;
	b->ffc_used = true;
	b->recharged = false;
	b->disable = b->phase == ERHAI_FFC_WAIT;
	b->step = b->voltage_count = b->current_count = 0;
	b->sw_count = b->hw_count = b->recharge_count = 0;
	b->ffc_region = -1;
	b->wait_until = now + msecs_to_jiffies(15000);
	b->next_sample = now;
}

static inline void erhai_buck_update(struct erhai_buck *b,
				     const struct erhai_buck_sample *s,
				     unsigned long now)
{
	int hw_mv, over_mv;

	if (time_before(now, b->next_sample))
		return;
	b->next_sample = now + msecs_to_jiffies(4000); /* COUNT_TIMELIMIT */
	erhai_buck_temperature(b, s->temp);
	b->ffc_region = erhai_ffc_region(b->ffc_region, s->temp);
	/* common.dtsi vbatt-ov-thr-mv; check_vbatt_is_good has 200 mV recovery. */
	if (s->mv >= 4600)
		b->over_voltage = true;
	else if (s->mv < 4400)
		b->over_voltage = false;
	erhai_buck_normal_limits(b, s);
	if (b->phase == ERHAI_FFC_WAIT) {
		b->disable = true;
		if (time_before(now, b->wait_until)) {
			b->next_sample = min(b->next_sample, b->wait_until);
			return;
		}
		if (s->soc == 100 && s->mv >= 4485) {
			b->phase = ERHAI_BUCK_FULL; /* full_pre_ffc_judge */
		} else if (s->qualified && b->ffc_region >= 1 && b->ffc_region <= 3 &&
			   !b->over_voltage) {
			b->phase = ERHAI_FFC_FAST;
			b->disable = false;
		} else {
			b->phase = ERHAI_BUCK_NORMAL;
			erhai_buck_normal_limits(b, s);
		}
		goto limits;
	}
	if (b->phase == ERHAI_FFC_FAST) {
		if (!s->qualified || b->ffc_region < 1 || b->ffc_region > 3 ||
		    b->over_voltage) {
			b->phase = s->soc >= 100 ? ERHAI_BUCK_NORMAL : ERHAI_FFC_IDLE;
			if (s->soc >= 100)
				b->current_count = 0;
			goto limits;
		}
		/* The vendor accumulates voltage hits, but current hits are consecutive. */
		if (s->mv >= 4540 - erhai_buck_age_mv(s->cycles))
			b->voltage_count++;
		if (abs(s->ma) < (b->step ? 900 : 1300))
			b->current_count++;
		else
			b->current_count = 0;
		if (b->voltage_count >= 4 || b->current_count >= 2) {
			b->step++;
			b->voltage_count = b->current_count = 0;
			if (b->step >= 2)
				b->phase = s->soc >= 100 ? ERHAI_BUCK_NORMAL : ERHAI_FFC_IDLE;
		}
		goto limits;
	}
	if (b->phase == ERHAI_FFC_IDLE) {
		if (++b->current_count >= 12) {
			b->phase = ERHAI_BUCK_NORMAL;
			b->current_count = 0;
		}
		goto limits;
	}
	if (b->phase == ERHAI_BUCK_FULL) {
		if (!b->disable && s->mv <= b->recharge_mv)
			b->recharge_count++;
		else
			b->recharge_count = 0;
		if (b->recharge_count > 5) {
			b->phase = ERHAI_BUCK_NORMAL;
			b->sw_count = b->hw_count = b->recharge_count = 0;
			b->recharged = true;
		}
		goto limits;
	}
	if (b->disable)
		goto limits;
	hw_mv = b->normal_fv_mv + 18;
	b->hw_count = s->mv > hw_mv ? b->hw_count + 1 : 0;
	if (s->mv > b->cutoff_mv && s->ma <= b->cutoff_ma)
		b->sw_count++;
	else
		b->sw_count = 0;
	if (b->hw_count > 4 || b->sw_count > (s->ma > 0 ? 5 : 10)) {
		b->phase = ERHAI_BUCK_FULL;
		b->fv_decrement = 0;
	}
	/* check_fv_over: VFLOAT_OVER_NUM=2, reduce by sw-over-fv-dec-mv. */
	over_mv = s->qualified ? b->normal_fv_mv + 10 : 3990;
	b->over_count = s->mv > over_mv ? b->over_count + 1 : 0;
	if (b->over_count > 2 && b->phase != ERHAI_BUCK_FULL) {
		b->fv_decrement += 8;
		b->over_count = 0;
	}
limits:
	if (b->phase == ERHAI_BUCK_FULL)
		b->ffc_used = true;
	b->fv_mv = b->normal_fv_mv - b->fv_decrement;
	if (b->phase == ERHAI_FFC_FAST) {
		b->fv_mv = 4630;
		b->fcc_ma = b->step ? 1000 : 1500;
		b->cutoff_mv = 4540 - erhai_buck_age_mv(s->cycles);
		b->cutoff_ma = b->step ? 900 : 1300;
	} else if (b->phase == ERHAI_FFC_IDLE || b->phase == ERHAI_BUCK_FULL) {
		b->disable = true;
	}
}
#endif
