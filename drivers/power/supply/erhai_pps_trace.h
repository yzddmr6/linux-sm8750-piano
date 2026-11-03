/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * erhai_pps tracepoints: one sample per engine poll while a PPS session is
 * up, and one event per charge-pump engage / drop / session exit.
 *
 *   echo 1 > /sys/kernel/tracing/events/erhai_pps/enable
 *   cat /sys/kernel/tracing/trace
 *
 * Chip fields are -1 when not read (no sc8547a module, or an I2C error).
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM erhai_pps

#if !defined(_ERHAI_PPS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _ERHAI_PPS_TRACE_H

#include <linux/tracepoint.h>

#ifndef _ERHAI_PPS_TRACE_SAMPLE
#define _ERHAI_PPS_TRACE_SAMPLE
struct erhai_pps_trace_sample {
	const char	*state;
	int		adsp_vbus_mv;	/* ADSP USB_VOLT_NOW */
	unsigned int	set_volt_mv;	/* last SET_PPS_VOLT */
	unsigned int	set_curr_ma;	/* last SET_PPS_CURR */
	int		vbat_mv;	/* pack gauge */
	int		ibat_ma;
	int		soc;
	int		temp_c;
	bool		cp;		/* engine's view: pump engaged */
	bool		buck_off;	/* buck suspended (ICL 0) for the pump */
	int		chip_vbus_mv;	/* SC8547A master ADCs */
	int		chip_vbat_mv;
	int		chip_ibus_ma;
	int		r06;		/* master converter status */
	int		r07;		/* master CHG_EN (bit 7) */
};
#endif

TRACE_EVENT(erhai_pps_sample,

	TP_PROTO(const struct erhai_pps_trace_sample *s),

	TP_ARGS(s),

	TP_STRUCT__entry(
		__string(state, s->state)
		__field(int, adsp_vbus_mv)
		__field(unsigned int, set_volt_mv)
		__field(unsigned int, set_curr_ma)
		__field(int, vbat_mv)
		__field(int, ibat_ma)
		__field(int, soc)
		__field(int, temp_c)
		__field(bool, cp)
		__field(bool, buck_off)
		__field(int, chip_vbus_mv)
		__field(int, chip_vbat_mv)
		__field(int, chip_ibus_ma)
		__field(int, r06)
		__field(int, r07)
	),

	TP_fast_assign(
		__assign_str(state);
		__entry->adsp_vbus_mv = s->adsp_vbus_mv;
		__entry->set_volt_mv = s->set_volt_mv;
		__entry->set_curr_ma = s->set_curr_ma;
		__entry->vbat_mv = s->vbat_mv;
		__entry->ibat_ma = s->ibat_ma;
		__entry->soc = s->soc;
		__entry->temp_c = s->temp_c;
		__entry->cp = s->cp;
		__entry->buck_off = s->buck_off;
		__entry->chip_vbus_mv = s->chip_vbus_mv;
		__entry->chip_vbat_mv = s->chip_vbat_mv;
		__entry->chip_ibus_ma = s->chip_ibus_ma;
		__entry->r06 = s->r06;
		__entry->r07 = s->r07;
	),

	TP_printk("%s vbus=%d set=%u/%u vbat=%d ibat=%d soc=%d t=%d cp=%d buck_off=%d chip_vbus=%d chip_vbat=%d diff=%d ibus=%d r06=%d r07=%d",
		  __get_str(state), __entry->adsp_vbus_mv,
		  __entry->set_volt_mv, __entry->set_curr_ma,
		  __entry->vbat_mv, __entry->ibat_ma, __entry->soc,
		  __entry->temp_c, __entry->cp, __entry->buck_off,
		  __entry->chip_vbus_mv, __entry->chip_vbat_mv,
		  __entry->chip_vbus_mv - 2 * __entry->chip_vbat_mv,
		  __entry->chip_ibus_ma, __entry->r06, __entry->r07)
);

TRACE_EVENT(erhai_pps_cp_event,

	TP_PROTO(const char *what, int rc, int chip_vbus_mv, int chip_vbat_mv,
		 int chip_ibus_ma, unsigned int set_volt_mv),

	TP_ARGS(what, rc, chip_vbus_mv, chip_vbat_mv, chip_ibus_ma, set_volt_mv),

	TP_STRUCT__entry(
		__string(what, what)
		__field(int, rc)
		__field(int, chip_vbus_mv)
		__field(int, chip_vbat_mv)
		__field(int, chip_ibus_ma)
		__field(unsigned int, set_volt_mv)
	),

	TP_fast_assign(
		__assign_str(what);
		__entry->rc = rc;
		__entry->chip_vbus_mv = chip_vbus_mv;
		__entry->chip_vbat_mv = chip_vbat_mv;
		__entry->chip_ibus_ma = chip_ibus_ma;
		__entry->set_volt_mv = set_volt_mv;
	),

	TP_printk("%s rc=%d chip_vbus=%d chip_vbat=%d diff=%d ibus=%d set=%u",
		  __get_str(what), __entry->rc, __entry->chip_vbus_mv,
		  __entry->chip_vbat_mv,
		  __entry->chip_vbus_mv - 2 * __entry->chip_vbat_mv,
		  __entry->chip_ibus_ma, __entry->set_volt_mv)
);

#endif /* _ERHAI_PPS_TRACE_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE erhai_pps_trace
#include <trace/define_trace.h>
