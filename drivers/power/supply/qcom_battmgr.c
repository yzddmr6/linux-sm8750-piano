// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, Linaro Ltd
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */
#include <linux/auxiliary_bus.h>
#include <linux/cleanup.h>
#include <linux/devm-helpers.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/soc/qcom/pdr.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/spinlock.h>
#include <linux/math.h>
#include <linux/suspend.h>
#include <linux/units.h>
#include <linux/usb/pd.h>
#include <linux/workqueue.h>

#define BATTMGR_CHEMISTRY_LEN	4
#define BATTMGR_STRING_LEN	128

enum qcom_battmgr_variant {
	QCOM_BATTMGR_SC8280XP,
	QCOM_BATTMGR_SM8350,
	QCOM_BATTMGR_SM8550,
	QCOM_BATTMGR_X1E80100,
};

/* Battery current is positive while discharging */
#define QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED	BIT(0)
/* BATT_POWER_NOW does not follow the battery power */
#define QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW		BIT(1)
/* The charge control opcode and properties mean something else */
#define QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL		BIT(2)
/*
 * Battery property 24 switches charging on (1) and off (0), so the driver
 * can enforce the charge control thresholds itself.
 */
#define QCOM_BATTMGR_QUIRK_CHARGE_ENABLE		BIT(3)
/*
 * The firmware leaves the USB VBUS boost to the OS: once allowed to, it
 * reports an attached sink with a notification and waits for the OS to
 * switch the boost on.
 */
#define QCOM_BATTMGR_QUIRK_OTG_VBUS			BIT(4)
/* The OS can ask the firmware for a USB PD PPS contract and its limits */
#define QCOM_BATTMGR_QUIRK_PPS				BIT(5)
/* OPLUS adapter type, PD notifications and fixed PDO voltage requests */
#define QCOM_BATTMGR_QUIRK_OPLUS_PD			BIT(6)

#define BATTMGR_BAT_STATUS		0x1

#define BATTMGR_REQUEST_NOTIFICATION	0x4

#define BATTMGR_NOTIFICATION		0x7
#define NOTIF_BAT_PROPERTY		0x30
#define NOTIF_USB_PROPERTY		0x32
#define NOTIF_WLS_PROPERTY		0x34
#define NOTIF_BAT_STATUS		0x80
#define NOTIF_BAT_INFO			0x81
#define NOTIF_BAT_CHARGING_STATE	0x83
/* OPLUS firmware: USB port events */
#define NOTIF_OPLUS_OTG_ENABLE		0x50
#define NOTIF_OPLUS_OTG_DISABLE		0x51
#define NOTIF_OPLUS_CID_DETECT		0x53
#define NOTIF_OPLUS_TYPEC_STATE		0x55
#define NOTIF_OPLUS_PLUGIN		0x57
#define NOTIF_OPLUS_APSD_DONE		0x58
/* OPLUS firmware: UFCS and reverse charging events */
#define NOTIF_OPLUS_UFCS_DISABLE_MOS	0x73
#define NOTIF_OPLUS_SOURCECAP_DONE	0x79
#define NOTIF_OPLUS_POWER_ROLE		0x7d
#define NOTIF_OPLUS_PD_HARD_RESET	0x7f

/* OPLUS buffer opcodes, separate from BATTMGR_NOTIFICATION */
#define BATTMGR_OPLUS_READ_PDOS		0x10004
#define BATTMGR_OPLUS_PD_INFO		0x1000a
#define OPLUS_READ_BUFFER_WORDS		128
#define OPLUS_PD_INFO_WORDS		8
#define OPLUS_PD_INFO_VDM_ID		0
#define OPLUS_USB_TYPE_PD_SDP		17
#define OPLUS_USB_TYPE_APPLE_BRICK_ID	9

#define BATTMGR_BAT_INFO		0x9

#define BATTMGR_BAT_DISCHARGE_TIME	0xc

#define BATTMGR_BAT_CHARGE_TIME		0xd

#define BATTMGR_BAT_PROPERTY_GET	0x30
#define BATTMGR_BAT_PROPERTY_SET	0x31
#define BATT_STATUS			0
#define BATT_HEALTH			1
#define BATT_PRESENT			2
#define BATT_CHG_TYPE			3
#define BATT_CAPACITY			4
#define BATT_SOH			5
#define BATT_VOLT_OCV			6
#define BATT_VOLT_NOW			7
#define BATT_VOLT_MAX			8
#define BATT_CURR_NOW			9
#define BATT_CHG_CTRL_LIM		10
#define BATT_CHG_CTRL_LIM_MAX		11
#define BATT_TEMP			12
#define BATT_TECHNOLOGY			13
#define BATT_CHG_COUNTER		14
#define BATT_CYCLE_COUNT		15
#define BATT_CHG_FULL_DESIGN		16
#define BATT_CHG_FULL			17
#define BATT_MODEL_NAME			18
#define BATT_TTF_AVG			19
#define BATT_TTE_AVG			20
#define BATT_RESISTANCE			21
#define BATT_POWER_NOW			22
#define BATT_POWER_AVG			23
#define BATT_CHG_CTRL_EN		24
#define BATT_CHG_CTRL_START_THR		25
#define BATT_CHG_CTRL_END_THR		26
/* OPLUS firmware: set to 0 to stop charging, the system keeps running on input */
#define BATT_OPLUS_CHG_EN		24
/* Fixed PDO voltage in mV, written with BATTMGR_BAT_PROPERTY_SET */
#define BATT_OPLUS_SET_PDO		25
/* OPLUS firmware: allow the charge current of a raised input voltage */
#define BATT_OPLUS_FASTCHG_ALLOW	46

#define BATTMGR_USB_PROPERTY_GET	0x32
#define BATTMGR_USB_PROPERTY_SET	0x33
#define USB_ONLINE			0
#define USB_VOLT_NOW			1
#define USB_VOLT_MAX			2
#define USB_CURR_NOW			3
#define USB_CURR_MAX			4
#define USB_INPUT_CURR_LIMIT		5
#define USB_TYPE			6
#define USB_ADAP_TYPE			7
#define USB_MOISTURE_DET_EN		8
#define USB_MOISTURE_DET_STS		9
#define USB_REAL_TYPE			11
/* OPLUS firmware: VBUS boost control */
#define USB_OPLUS_OTG_AP_ENABLE		17
#define USB_OPLUS_OTG_SWITCH		18
#define USB_OPLUS_OTG_VBUS_ENABLE	24
#define USB_OPLUS_OTG_BOOST_CURRENT	64
/* OPLUS firmware: USB PD PPS, voltage in mV and current in mA */
#define USB_OPLUS_PPS_TYPE		32
#define USB_OPLUS_PPS_VOLT		34
#define USB_OPLUS_PPS_CURR		35
#define USB_OPLUS_PPS_MAX_CURR		36
#define USB_OPLUS_PPS_ENABLE		49

#define BATTMGR_WLS_PROPERTY_GET	0x34
#define BATTMGR_WLS_PROPERTY_SET	0x35
#define WLS_ONLINE			0
#define WLS_VOLT_NOW			1
#define WLS_VOLT_MAX			2
#define WLS_CURR_NOW			3
#define WLS_CURR_MAX			4
#define WLS_TYPE			5
#define WLS_BOOST_EN			6

#define BATTMGR_CHG_CTRL_LIMIT_EN	0x48
#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

/* Interval of the state of charge check while an end threshold is set */
#define CHARGE_CTRL_POLL_MS		30000

/* The vendor kernel waits this long for the answer to a property write */
#define BATTMGR_SET_TIMEOUT_MS		2500

/* VBUS boost current limit for a USB sink, from the vendor devicetree */
#define OTG_BOOST_CURRENT_MA		1250

/*
 * PPS contract: 8.4 V at up to 3 A from the charger, with an input current
 * limit of 3 A and a battery charge current limit of 4.4 A.  At most
 * 25.2 W, far below the 67 W charging limit of the device (vendor
 * devicetree oplus,vooc_power_max_w = <0x43>).
 */
#define PPS_VOLT_MV			8400
#define PPS_CURR_START_MA		1000
#define PPS_CURR_MA			3000
#define PPS_CURR_STEP_MA		250
#define PPS_INPUT_LIMIT_UA		3000000
#define PPS_CHARGE_LIMIT_UA		4400000
#define PPS_POWER_MAX_MW		67000
static_assert(PPS_VOLT_MV * PPS_CURR_MA <= PPS_POWER_MAX_MW * 1000);

/* Values written to leave the contract */
#define PPS_END_VOLT_MV			5000
#define PPS_END_CURR_MA			2000

/* Polling while a PPS contract is pending or active */
#define PPS_POLL_MS			2000
/* Time for the firmware to finish its own charger setup after plug-in */
#define PPS_SETTLE_MS			10000
/* VBUS must have left 5 V after this many polls, and never collapse */
#define PPS_RISE_POLLS			6
#define PPS_VBUS_RISEN_UV		5500000
#define PPS_VBUS_COLLAPSE_UV		3800000
/* Polls at a raised VBUS without input current before giving up */
#define PPS_VBUS_HIGH_UV		7000000
#define PPS_STALL_UA			100000
#define PPS_STALL_POLLS			15

/*
 * Battery limits for the contract, from the vendor PPS settings: it ends at
 * 90 % (oplus,pps_charge_strategy_soc), at 4.49 V
 * (oplus,pps_full_normal_sw_vbat), above 43.0 C (the highest step of
 * oplus,pps_strategy_batt_high_temp) and below 0 C.  Starting needs a
 * margin: at most 85 %, 4.4 V and 40.0 C (the vendor recovery
 * temperature), and at least 5.0 C.
 */
#define PPS_SOC_MAX			90
#define PPS_SOC_MARGIN			5
#define PPS_VBAT_MAX_UV			4490000
#define PPS_VBAT_START_UV		4400000
#define PPS_VBAT_MIN_UV			3300000
#define PPS_TEMP_MAX			430
#define PPS_TEMP_START_MAX		400
#define PPS_TEMP_START_MIN		50
#define PPS_TEMP_MIN			0

/* Fixed PD limits from oplus-chg-erhai-common.dtsi and oplus-chg-24924.dtsi. */
#define PD_FIXED_MV			9000
#define PD_DEFAULT_MV			5000
#define PD_INPUT_MAX_UA			2000000
#define PD_TRANSITION_UA			500000
#define PD_VBUS_RISEN_UV			7500000
#define PD_VBUS_MAX_UV			11000000
#define PD_VBAT_START_UV			5000000
#define PD_VBAT_STOP_UV			5100000
/* oplus_wired_pd_config_work() waits up to 15 s, checking at 1 s intervals. */
#define PD_POLL_MS			1000
#define PD_CONFIG_TIMEOUT_MS		15000
#define PD_SHUTDOWN_WAIT_MS		100

enum qcom_battmgr_pd_state {
	PD_IDLE,
	PD_WAIT,
	PD_ACTIVE,
	PD_FAILED,
};

enum qcom_battmgr_pd_event {
	PD_EVENT_CAPS,
	PD_EVENT_RESET,
};

static bool pps_enable;
module_param(pps_enable, bool, 0644);
MODULE_PARM_DESC(pps_enable,
		 "Request a USB PD PPS contract from PPS chargers where supported (default: off)");

enum qcom_battmgr_pps_state {
	PPS_IDLE,	/* no PPS charger, or PPS disabled */
	PPS_SETTLE,	/* PPS charger present, waiting for its setup to finish */
	PPS_ACTIVE,	/* contract requested */
	PPS_HOLD,	/* ended for the battery, may start again with margin */
	PPS_FAILED,	/* charger or firmware refused, not retried until unplugged */
};

struct qcom_battmgr_pps_sample {
	unsigned int soc;
	unsigned int vbat_uv;
	int ibat_ua;		/* positive while charging */
	int temp;		/* tenths of a degree Celsius */
	unsigned int health;
	unsigned int vbus_uv;
	unsigned int ibus_ua;
};

struct qcom_battmgr_enable_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 power_state;
	__le32 low_capacity;
	__le32 high_capacity;
};

struct qcom_battmgr_property_request {
	struct pmic_glink_hdr hdr;
	__le32 battery;
	__le32 property;
	__le32 value;
};

struct qcom_battmgr_read_pdos_request {
	struct pmic_glink_hdr hdr;
	__le32 data_size;
};

struct qcom_battmgr_read_pdos_response {
	struct pmic_glink_hdr hdr;
	__le32 data[OPLUS_READ_BUFFER_WORDS];
	__le32 data_size;
};

struct qcom_battmgr_pd_info {
	struct pmic_glink_hdr hdr;
	__le32 data[OPLUS_PD_INFO_WORDS];
	__le32 data_size;
	__le32 msg_id;
};

struct qcom_battmgr_update_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
};

struct qcom_battmgr_charge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 percent;
	__le32 reserved;
};

struct qcom_battmgr_discharge_time_request {
	struct pmic_glink_hdr hdr;
	__le32 battery_id;
	__le32 rate; /* 0 for current rate */
	__le32 reserved;
};

struct qcom_battmgr_charge_ctrl_request {
	struct pmic_glink_hdr hdr;
	__le32 enable;
	__le32 target_soc;
	__le32 delta_soc;
};

struct qcom_battmgr_message {
	struct pmic_glink_hdr hdr;
	union {
		struct {
			__le32 property;
			__le32 value;
			__le32 result;
		} intval;
		struct {
			__le32 property;
			char model[BATTMGR_STRING_LEN];
		} strval;
		struct {
			/*
			 * 0: mWh
			 * 1: mAh
			 */
			__le32 power_unit;
			__le32 design_capacity;
			__le32 last_full_capacity;
			/*
			 * 0 nonrechargable
			 * 1 rechargable
			 */
			__le32 battery_tech;
			__le32 design_voltage; /* mV */
			__le32 capacity_low;
			__le32 capacity_warning;
			__le32 cycle_count;
			/* thousandth of percent */
			__le32 accuracy;
			__le32 max_sample_time_ms;
			__le32 min_sample_time_ms;
			__le32 max_average_interval_ms;
			__le32 min_average_interval_ms;
			/* granularity between low and warning */
			__le32 capacity_granularity1;
			/* granularity between warning and full */
			__le32 capacity_granularity2;
			/*
			 * 0: no
			 * 1: cold
			 * 2: hot
			 */
			__le32 swappable;
			__le32 capabilities;
			char model_number[BATTMGR_STRING_LEN];
			char serial_number[BATTMGR_STRING_LEN];
			char battery_type[BATTMGR_STRING_LEN];
			char oem_info[BATTMGR_STRING_LEN];
			char battery_chemistry[BATTMGR_CHEMISTRY_LEN];
			char uid[BATTMGR_STRING_LEN];
			__le32 critical_bias;
			u8 day;
			u8 month;
			__le16 year;
			__le32 battery_id;
		} info;
		struct {
			/*
			 * BIT(0) discharging
			 * BIT(1) charging
			 * BIT(2) critical low
			 */
			__le32 battery_state;
			/* mWh or mAh, based on info->power_unit */
			__le32 capacity;
			__le32 rate;
			/* mv */
			__le32 battery_voltage;
			/*
			 * BIT(0) power online
			 * BIT(1) discharging
			 * BIT(2) charging
			 * BIT(3) battery critical
			 */
			__le32 power_state;
			/*
			 * 1: AC
			 * 2: USB
			 * 3: Wireless
			 */
			__le32 charging_source;
			__le32 temperature;
		} status;
		__le32 time;
		__le32 notification;
	};
};

#define BATTMGR_CHARGING_SOURCE_AC	1
#define BATTMGR_CHARGING_SOURCE_USB	2
#define BATTMGR_CHARGING_SOURCE_WIRELESS 3

enum qcom_battmgr_unit {
	QCOM_BATTMGR_UNIT_mWh = 0,
	QCOM_BATTMGR_UNIT_mAh = 1
};

struct qcom_battmgr_info {
	bool valid;

	bool present;
	unsigned int charge_type;
	unsigned int design_capacity;
	unsigned int last_full_capacity;
	unsigned int voltage_max_design;
	unsigned int voltage_max;
	unsigned int capacity_low;
	unsigned int capacity_warning;
	unsigned int cycle_count;
	unsigned int charge_count;
	unsigned int charge_ctrl_start;
	unsigned int charge_ctrl_end;
	char model_number[BATTMGR_STRING_LEN];
	char serial_number[BATTMGR_STRING_LEN];
	char oem_info[BATTMGR_STRING_LEN];
	unsigned char technology;
	unsigned char day;
	unsigned char month;
	unsigned short year;
};

struct qcom_battmgr_status {
	unsigned int status;
	unsigned int health;
	unsigned int capacity;
	unsigned int percent;
	int current_now;
	int power_now;
	unsigned int voltage_now;
	unsigned int voltage_ocv;
	unsigned int temperature;
	unsigned int resistance;
	unsigned int soh_percent;

	unsigned int discharge_time;
	unsigned int charge_time;
};

struct qcom_battmgr_ac {
	bool online;
};

struct qcom_battmgr_usb {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
	unsigned int current_limit;
	unsigned int usb_type;
};

struct qcom_battmgr_wireless {
	bool online;
	unsigned int voltage_now;
	unsigned int voltage_max;
	unsigned int current_now;
	unsigned int current_max;
};

struct qcom_battmgr {
	struct device *dev;
	struct pmic_glink_client *client;

	enum qcom_battmgr_variant variant;
	unsigned long quirks;

	struct power_supply *ac_psy;
	struct power_supply *bat_psy;
	struct power_supply *usb_psy;
	struct power_supply *wls_psy;

	enum qcom_battmgr_unit unit;

	int error;
	struct completion ack;
	/* Only a reply to the outstanding opcode may complete @ack. */
	u32 request_opcode;

	bool service_up;

	struct qcom_battmgr_info info;
	struct qcom_battmgr_status status;
	struct qcom_battmgr_ac ac;
	struct qcom_battmgr_usb usb;
	struct qcom_battmgr_wireless wireless;

	struct work_struct enable_work;

	/*
	 * @lock is used to prevent concurrent power supply requests to the
	 * firmware, as it then stops responding.
	 */
	struct mutex lock;

	/*
	 * @notify_lock orders @removing against the PMIC GLINK callbacks, which
	 * may still run while the power supplies and @enable_work are torn down.
	 */
	spinlock_t notify_lock;
	bool removing;

	/* Property id and value in the answer to the last accepted request */
	u32 resp_property;
	u32 resp_value;
	/* Source PDO reply, protected by @lock and the request completion. */
	u32 src_pdos[PDO_MAX_OBJECTS];
	unsigned int num_src_pdos;

	/*
	 * Charge control thresholds enforced by the driver: @charge_work runs
	 * on a freezable workqueue, so no request is sent while the system
	 * is suspended.  @charge_inhibited is set while the driver holds
	 * charging off, @charge_error while the firmware refuses to switch.
	 */
	struct delayed_work charge_work;
	bool charge_inhibited;
	bool charge_error;

	/*
	 * USB VBUS boost: @otg_request follows the firmware notifications,
	 * @otg_work (freezable) switches the boost to match and records the
	 * result in @otg_vbus.
	 */
	struct work_struct otg_work;
	bool otg_request;
	bool otg_vbus;

	/*
	 * USB PD PPS, run from @charge_work under @charge_lock; @pm_nb ends
	 * the contract before system suspend, since it is not monitored
	 * while suspended.
	 */
	struct mutex charge_lock;
	struct notifier_block pm_nb;
	enum qcom_battmgr_pps_state pps_state;
	unsigned long pps_since;
	unsigned int pps_polls;
	unsigned int pps_stalls;
	u32 pps_curr_ma;
	u32 pps_target_ma;
	/* Limits found before the contract, restored when it ends */
	u32 pps_input_limit_ua;
	u32 pps_charge_limit_ua;

	/* Fixed PD shares @charge_work and @charge_lock with PPS. */
	struct notifier_block psy_nb;
	unsigned long pd_events;
	enum qcom_battmgr_pd_state pd_state;
	unsigned long pd_since;
	bool pd_restore;
	bool pd_logged;
	u32 pd_input_limit_ua;
	u32 pd_saved_limit_ua;
};

static int __qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data,
				  size_t len, unsigned long timeout)
{
	const struct pmic_glink_hdr *hdr = data;
	unsigned long left;
	int ret;

	reinit_completion(&battmgr->ack);
	WRITE_ONCE(battmgr->request_opcode, le32_to_cpu(hdr->opcode));

	battmgr->error = 0;

	ret = pmic_glink_send(battmgr->client, data, len);
	if (!ret) {
		left = wait_for_completion_timeout(&battmgr->ack, timeout);
		ret = left ? battmgr->error : -ETIMEDOUT;
	}
	WRITE_ONCE(battmgr->request_opcode, U32_MAX);

	return ret;
}

static int qcom_battmgr_request(struct qcom_battmgr *battmgr, void *data, size_t len)
{
	return __qcom_battmgr_request(battmgr, data, len, HZ);
}

static int qcom_battmgr_request_property(struct qcom_battmgr *battmgr, int opcode,
					 int property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

/*
 * Write a battery (BATTMGR_BAT_PROPERTY_SET) or USB (BATTMGR_USB_PROPERTY_SET)
 * property and wait until the firmware has accepted it.  The answer must
 * name the written property, so a late answer to an earlier request that
 * timed out is not taken for this one.
 */
static int qcom_battmgr_set_property(struct qcom_battmgr *battmgr, int opcode,
				     u32 property, u32 value)
{
	struct qcom_battmgr_property_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(opcode),
		.battery = cpu_to_le32(0),
		.property = cpu_to_le32(property),
		.value = cpu_to_le32(value),
	};
	int ret;

	mutex_lock(&battmgr->lock);
	battmgr->resp_property = U32_MAX;
	ret = __qcom_battmgr_request(battmgr, &request, sizeof(request),
				     msecs_to_jiffies(BATTMGR_SET_TIMEOUT_MS));
	if (!ret && battmgr->resp_property != property)
		ret = -EIO;
	mutex_unlock(&battmgr->lock);

	return ret;
}

/* Read the raw value of a battery or USB property */
static int qcom_battmgr_get_property(struct qcom_battmgr *battmgr, int opcode,
				     u32 property, u32 *value)
{
	int ret;

	mutex_lock(&battmgr->lock);
	battmgr->resp_property = U32_MAX;
	ret = qcom_battmgr_request_property(battmgr, opcode, property, 0);
	if (!ret && battmgr->resp_property != property)
		ret = -EIO;
	if (!ret)
		*value = battmgr->resp_value;
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_update_status(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_STATUS),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_info(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_update_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_INFO),
		.battery_id = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_charge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_charge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_CHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.percent = cpu_to_le32(100),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_update_discharge_time(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_discharge_time_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_BAT_DISCHARGE_TIME),
		.battery_id = cpu_to_le32(0),
		.rate = cpu_to_le32(0),
	};

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static const u8 sm8350_bat_prop_map[] = {
	[POWER_SUPPLY_PROP_STATUS] = BATT_STATUS,
	[POWER_SUPPLY_PROP_HEALTH] = BATT_HEALTH,
	[POWER_SUPPLY_PROP_PRESENT] = BATT_PRESENT,
	[POWER_SUPPLY_PROP_CHARGE_TYPE] = BATT_CHG_TYPE,
	[POWER_SUPPLY_PROP_CAPACITY] = BATT_CAPACITY,
	[POWER_SUPPLY_PROP_VOLTAGE_OCV] = BATT_VOLT_OCV,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = BATT_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = BATT_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = BATT_CURR_NOW,
	[POWER_SUPPLY_PROP_TEMP] = BATT_TEMP,
	[POWER_SUPPLY_PROP_TECHNOLOGY] = BATT_TECHNOLOGY,
	[POWER_SUPPLY_PROP_CHARGE_COUNTER] =  BATT_CHG_COUNTER,
	[POWER_SUPPLY_PROP_CYCLE_COUNT] = BATT_CYCLE_COUNT,
	[POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN] =  BATT_CHG_FULL_DESIGN,
	[POWER_SUPPLY_PROP_CHARGE_FULL] = BATT_CHG_FULL,
	[POWER_SUPPLY_PROP_MODEL_NAME] = BATT_MODEL_NAME,
	[POWER_SUPPLY_PROP_TIME_TO_FULL_AVG] = BATT_TTF_AVG,
	[POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG] = BATT_TTE_AVG,
	[POWER_SUPPLY_PROP_INTERNAL_RESISTANCE] = BATT_RESISTANCE,
	[POWER_SUPPLY_PROP_STATE_OF_HEALTH] = BATT_SOH,
	[POWER_SUPPLY_PROP_POWER_NOW] = BATT_POWER_NOW,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD] = BATT_CHG_CTRL_START_THR,
	[POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD] = BATT_CHG_CTRL_END_THR,
};

static int qcom_battmgr_bat_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_bat_prop_map))
		return -EINVAL;

	prop = sm8350_bat_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_BAT_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

/*
 * With QCOM_BATTMGR_QUIRK_CHARGE_ENABLE the thresholds live in the driver;
 * reading them must not send the firmware a request for their property ids.
 */
static bool qcom_battmgr_charge_ctrl_local(struct qcom_battmgr *battmgr,
					   enum power_supply_property psp)
{
	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE))
		return false;

	return psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD ||
	       psp == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD;
}

static int qcom_battmgr_bat_sc8280xp_update(struct qcom_battmgr *battmgr,
					    enum power_supply_property psp)
{
	int ret;

	mutex_lock(&battmgr->lock);

	if (!battmgr->info.valid) {
		ret = qcom_battmgr_update_info(battmgr);
		if (ret < 0)
			goto out_unlock;
		battmgr->info.valid = true;
	}

	ret = qcom_battmgr_update_status(battmgr);
	if (ret < 0)
		goto out_unlock;

	if (psp == POWER_SUPPLY_PROP_TIME_TO_FULL_AVG) {
		ret = qcom_battmgr_update_charge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

	if (psp == POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG) {
		ret = qcom_battmgr_update_discharge_time(battmgr);
		if (ret < 0) {
			ret = -ENODATA;
			goto out_unlock;
		}
	}

out_unlock:
	mutex_unlock(&battmgr->lock);
	return ret;
}

static int qcom_battmgr_bat_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	enum qcom_battmgr_unit unit = battmgr->unit;
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (qcom_battmgr_charge_ctrl_local(battmgr, psp))
		ret = 0;
	else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		 battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_bat_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battmgr->status.status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = battmgr->info.charge_type;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battmgr->status.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battmgr->info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battmgr->info.technology;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = battmgr->info.cycle_count;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = battmgr->info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->info.voltage_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->status.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = battmgr->status.voltage_ocv;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->status.current_now;
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED)
			val->intval = -val->intval;
		break;
	case POWER_SUPPLY_PROP_POWER_NOW:
		val->intval = battmgr->status.power_now;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mAh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = battmgr->info.charge_count;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.design_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.last_full_capacity;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->info.capacity_low;
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
		if (unit != QCOM_BATTMGR_UNIT_mWh)
			return -ENODATA;
		val->intval = battmgr->status.capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (battmgr->status.percent == (unsigned int)-1)
			return -ENODATA;
		val->intval = battmgr->status.percent;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battmgr->status.temperature;
		break;
	case POWER_SUPPLY_PROP_INTERNAL_RESISTANCE:
		val->intval = battmgr->status.resistance;
		break;
	case POWER_SUPPLY_PROP_STATE_OF_HEALTH:
		val->intval = battmgr->status.soh_percent;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		val->intval = battmgr->status.discharge_time;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		val->intval = battmgr->status.charge_time;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_start;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = battmgr->info.charge_ctrl_end;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_YEAR:
		val->intval = battmgr->info.year;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_MONTH:
		val->intval = battmgr->info.month;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURE_DAY:
		val->intval = battmgr->info.day;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = battmgr->info.model_number;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = battmgr->info.oem_info;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = battmgr->info.serial_number;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int qcom_battmgr_set_charge_control(struct qcom_battmgr *battmgr,
					   u32 target_soc, u32 delta_soc)
{
	struct qcom_battmgr_charge_ctrl_request request = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_CHG_CTRL_LIMIT_EN),
		.enable = cpu_to_le32(1),
		.target_soc = cpu_to_le32(target_soc),
		.delta_soc = cpu_to_le32(delta_soc),
	};

	/* The driver enforces the thresholds, see qcom_battmgr_charge_worker() */
	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE)
		return 0;

	return qcom_battmgr_request(battmgr, &request, sizeof(request));
}

static int qcom_battmgr_set_charge_start_threshold(struct qcom_battmgr *battmgr, int start_soc)
{
	u32 target_soc, delta_soc;
	int ret;

	start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

	/*
	 * If the new start threshold is larger than the old end threshold,
	 * move the end threshold one step (DELTA_SOC) after the new start
	 * threshold.
	 */
	if (start_soc > battmgr->info.charge_ctrl_end) {
		target_soc = start_soc + CHARGE_CTRL_DELTA_SOC;
		target_soc = min_t(u32, target_soc, CHARGE_CTRL_END_THR_MAX);
		delta_soc = target_soc - start_soc;
		delta_soc = min_t(u32, delta_soc, CHARGE_CTRL_DELTA_SOC);
	} else {
		target_soc =  battmgr->info.charge_ctrl_end;
		delta_soc = battmgr->info.charge_ctrl_end - start_soc;
	}

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, target_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = target_soc;
	}

	return 0;
}

static int qcom_battmgr_set_charge_end_threshold(struct qcom_battmgr *battmgr, int end_soc)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int ret;

	end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);

	if (battmgr->info.charge_ctrl_start && end_soc > battmgr->info.charge_ctrl_start)
		delta_soc = end_soc - battmgr->info.charge_ctrl_start;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_set_charge_control(battmgr, end_soc, delta_soc);
	mutex_unlock(&battmgr->lock);
	if (!ret) {
		battmgr->info.charge_ctrl_start = end_soc - delta_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

static int qcom_battmgr_charge_control_thresholds_init(struct qcom_battmgr *battmgr)
{
	int ret;
	u8 en, end_soc, start_soc, delta_soc;

	ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_en", &en);
	if (!ret && en != 0) {
		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_end", &end_soc);
		if (ret < 0)
			return ret;

		ret = nvmem_cell_read_u8(battmgr->dev->parent, "charge_limit_delta", &delta_soc);
		if (ret < 0)
			return ret;

		if (delta_soc >= end_soc)
			return -EINVAL;

		start_soc = end_soc - delta_soc;
		end_soc = clamp(end_soc, CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		start_soc = clamp(start_soc, CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);

		battmgr->info.charge_ctrl_start = start_soc;
		battmgr->info.charge_ctrl_end = end_soc;
	}

	return 0;
}

/* Charge control is unavailable if its request means something else */
static bool qcom_battmgr_no_charge_ctrl(struct qcom_battmgr *battmgr)
{
	return (battmgr->quirks & QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL) &&
	       !(battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE);
}

static bool qcom_battmgr_charge_limited(struct qcom_battmgr *battmgr)
{
	unsigned int end = READ_ONCE(battmgr->info.charge_ctrl_end);

	return end && end < CHARGE_CTRL_END_THR_MAX;
}

static int qcom_battmgr_charge_enable(struct qcom_battmgr *battmgr, bool enable)
{
	int ret;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_OPLUS_CHG_EN, enable);

	/* Report a refusal once, not on every check */
	if (ret && !battmgr->charge_error)
		dev_warn(battmgr->dev, "failed to turn charging %s: %d\n",
			 enable ? "on" : "off", ret);
	battmgr->charge_error = !!ret;

	return ret;
}

/*
 * Stop charging once the state of charge reaches the end threshold and
 * resume once it has fallen to the start threshold.  Nothing is sent while
 * no end threshold below 100 % is set, so charging is then left entirely
 * to the firmware.
 */
static void qcom_battmgr_charge_limit_update(struct qcom_battmgr *battmgr)
{
	unsigned int start = READ_ONCE(battmgr->info.charge_ctrl_start);
	unsigned int end = READ_ONCE(battmgr->info.charge_ctrl_end);
	unsigned int soc;
	int ret;

	if (!qcom_battmgr_charge_limited(battmgr)) {
		if (battmgr->charge_inhibited &&
		    !qcom_battmgr_charge_enable(battmgr, true)) {
			battmgr->charge_inhibited = false;
			dev_info(battmgr->dev, "charge limit removed, charging resumed\n");
		}
		return;
	}

	ret = qcom_battmgr_bat_sm8350_update(battmgr, POWER_SUPPLY_PROP_CAPACITY);
	if (ret)
		return;

	soc = battmgr->status.percent;
	if (soc >= end) {
		/*
		 * Repeat the request on every check: the firmware may turn
		 * charging back on by itself, for example on a new charger.
		 */
		if (!qcom_battmgr_charge_enable(battmgr, false) &&
		    !battmgr->charge_inhibited) {
			battmgr->charge_inhibited = true;
			dev_info(battmgr->dev, "charging stopped at %u%% (end threshold %u%%)\n",
				 soc, end);
		}
	} else if (soc <= start && battmgr->charge_inhibited) {
		if (!qcom_battmgr_charge_enable(battmgr, true)) {
			battmgr->charge_inhibited = false;
			dev_info(battmgr->dev, "charging resumed at %u%% (start threshold %u%%)\n",
				 soc, start);
		}
	}
}

static bool qcom_battmgr_pps_enabled(struct qcom_battmgr *battmgr)
{
	return (battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS) && READ_ONCE(pps_enable);
}

/* A charger that offers PPS, as detected by the firmware */
static bool qcom_battmgr_pps_charger(struct qcom_battmgr *battmgr)
{
	u32 type, pps;

	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_ADAP_TYPE, &type) ||
	    type != POWER_SUPPLY_USB_TYPE_PD_PPS)
		return false;

	return !qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					  USB_OPLUS_PPS_TYPE, &pps) && pps == 1;
}

static int qcom_battmgr_pps_sample(struct qcom_battmgr *battmgr,
				   struct qcom_battmgr_pps_sample *s)
{
	u32 soc, curr, temp;
	int ret;

	ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
					BATT_CAPACITY, &soc);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_VOLT_NOW, &s->vbat_uv);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_CURR_NOW, &curr);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_TEMP, &temp);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_HEALTH, &s->health);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_VOLT_NOW, &s->vbus_uv);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_CURR_NOW, &s->ibus_ua);
	if (ret)
		return ret;

	s->soc = soc / 100;
	s->ibat_ua = (s32)curr;
	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED)
		s->ibat_ua = -s->ibat_ua;
	s->temp = DIV_ROUND_CLOSEST((s32)temp, 10);

	return 0;
}

/* The contract ends at the end threshold if that is below the PPS limit */
static unsigned int qcom_battmgr_pps_soc_max(struct qcom_battmgr *battmgr)
{
	unsigned int end = READ_ONCE(battmgr->info.charge_ctrl_end);

	if (qcom_battmgr_charge_limited(battmgr) && end < PPS_SOC_MAX)
		return end;

	return PPS_SOC_MAX;
}

static const char *qcom_battmgr_pps_stop_reason(struct qcom_battmgr *battmgr,
						const struct qcom_battmgr_pps_sample *s)
{
	if (battmgr->charge_inhibited)
		return "charging stopped for the end threshold";
	if (s->health != POWER_SUPPLY_HEALTH_GOOD)
		return "battery health";
	if (s->temp < PPS_TEMP_MIN || s->temp >= PPS_TEMP_MAX)
		return "battery temperature";
	if (s->vbat_uv < PPS_VBAT_MIN_UV || s->vbat_uv >= PPS_VBAT_MAX_UV)
		return "battery voltage";
	if (s->soc >= qcom_battmgr_pps_soc_max(battmgr))
		return "battery charge level";

	return NULL;
}

static bool qcom_battmgr_pps_may_start(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_pps_sample s;

	if (qcom_battmgr_pps_sample(battmgr, &s) ||
	    qcom_battmgr_pps_stop_reason(battmgr, &s))
		return false;

	return s.temp >= PPS_TEMP_START_MIN && s.temp <= PPS_TEMP_START_MAX &&
	       s.vbat_uv <= PPS_VBAT_START_UV &&
	       s.soc + PPS_SOC_MARGIN <= qcom_battmgr_pps_soc_max(battmgr);
}

/*
 * Leave the contract: back to 5 V first, then release the limits and the
 * PPS mode.  The limits found before the contract are restored, and
 * charging stays off if the end threshold holds it off.
 */
static void qcom_battmgr_pps_stop(struct qcom_battmgr *battmgr,
				  enum qcom_battmgr_pps_state next,
				  const char *reason)
{
	int ret, err = 0;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_PPS_CURR, PPS_END_CURR_MA);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_PPS_VOLT, PPS_END_VOLT_MV);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_OPLUS_FASTCHG_ALLOW, 0);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_CHG_CTRL_LIM, battmgr->pps_charge_limit_ua);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_OPLUS_CHG_EN, !battmgr->charge_inhibited);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_INPUT_CURR_LIMIT, battmgr->pps_input_limit_ua);
	err = err ?: ret;
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_PPS_ENABLE, 0);
	err = err ?: ret;

	/* Writes to a port without a charger may be refused */
	if (err && next != PPS_IDLE)
		dev_warn(battmgr->dev, "PPS: failed to restore 5 V charging: %d\n", err);

	dev_info(battmgr->dev, "PPS contract ended: %s\n", reason);
	battmgr->pps_state = next;
}

/*
 * Request the contract, in the order the firmware needs: PPS mode, the
 * voltage before the current, the input limit, and only then the charge
 * current permission and limit.  Other orders leave VBUS at about 6.6 V
 * or the input current at about 1.3 A.
 */
static int qcom_battmgr_pps_start(struct qcom_battmgr *battmgr)
{
	u32 max_ma;
	int ret;

	/*
	 * Both limits are put back when the contract ends.  Do not start
	 * without them, and not while either is 0: an input current limit
	 * of 0 suspends the input.
	 */
	ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
					USB_INPUT_CURR_LIMIT,
					&battmgr->pps_input_limit_ua);
	if (!ret)
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET,
						BATT_CHG_CTRL_LIM,
						&battmgr->pps_charge_limit_ua);
	if (!ret && (!battmgr->pps_input_limit_ua || !battmgr->pps_charge_limit_ua))
		ret = -ENODATA;
	if (ret) {
		dev_warn(battmgr->dev, "PPS: current limits unavailable: %d\n", ret);
		battmgr->pps_state = PPS_FAILED;
		return ret;
	}

	/* Stay within the current the charger offers */
	battmgr->pps_target_ma = PPS_CURR_MA;
	if (!qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				       USB_OPLUS_PPS_MAX_CURR, &max_ma) &&
	    max_ma >= PPS_CURR_START_MA && max_ma < PPS_CURR_MA)
		battmgr->pps_target_ma = max_ma;
	battmgr->pps_curr_ma = PPS_CURR_START_MA;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_PPS_ENABLE, 1);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_PPS_VOLT, PPS_VOLT_MV);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_PPS_CURR, battmgr->pps_curr_ma);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT, PPS_INPUT_LIMIT_UA);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
						BATT_OPLUS_FASTCHG_ALLOW, 1);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
						BATT_OPLUS_CHG_EN, 1);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
						BATT_CHG_CTRL_LIM, PPS_CHARGE_LIMIT_UA);
	if (ret) {
		qcom_battmgr_pps_stop(battmgr, PPS_FAILED, "request refused");
		return ret;
	}

	battmgr->pps_state = PPS_ACTIVE;
	battmgr->pd_logged = false;
	battmgr->pps_polls = 0;
	battmgr->pps_stalls = 0;
	dev_info(battmgr->dev, "PPS contract requested: %u mV, up to %u mA\n",
		 PPS_VOLT_MV, battmgr->pps_target_ma);

	return 0;
}

static void qcom_battmgr_pps_monitor(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_pps_sample s;
	const char *reason;
	u32 curr;

	if (qcom_battmgr_pps_sample(battmgr, &s)) {
		qcom_battmgr_pps_stop(battmgr, PPS_HOLD, "no battery data");
		return;
	}

	reason = qcom_battmgr_pps_stop_reason(battmgr, &s);
	if (reason) {
		qcom_battmgr_pps_stop(battmgr, PPS_HOLD, reason);
		return;
	}

	battmgr->pps_polls++;
	if (s.vbus_uv < PPS_VBUS_COLLAPSE_UV) {
		qcom_battmgr_pps_stop(battmgr, PPS_FAILED, "VBUS collapsed");
		return;
	}
	if (battmgr->pps_polls >= PPS_RISE_POLLS && s.vbus_uv < PPS_VBUS_RISEN_UV) {
		qcom_battmgr_pps_stop(battmgr, PPS_FAILED, "VBUS did not rise");
		return;
	}

	if (s.vbus_uv > PPS_VBUS_HIGH_UV && s.ibus_ua < PPS_STALL_UA) {
		if (++battmgr->pps_stalls >= PPS_STALL_POLLS) {
			qcom_battmgr_pps_stop(battmgr, PPS_FAILED, "no input current");
			return;
		}
	} else {
		battmgr->pps_stalls = 0;
	}

	if (battmgr->pps_curr_ma >= battmgr->pps_target_ma)
		return;

	curr = min(battmgr->pps_curr_ma + PPS_CURR_STEP_MA, battmgr->pps_target_ma);
	/* The firmware expects a voltage/current pair for each PDO update. */
	if (qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
				      USB_OPLUS_PPS_VOLT, PPS_VOLT_MV) ||
	    qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
				      USB_OPLUS_PPS_CURR, curr)) {
		qcom_battmgr_pps_stop(battmgr, PPS_FAILED, "current request refused");
		return;
	}

	battmgr->pps_curr_ma = curr;
	if (curr == battmgr->pps_target_ma)
		dev_info(battmgr->dev, "PPS: VBUS %u mV %u mA, battery %d mA\n",
			 s.vbus_uv / 1000, s.ibus_ua / 1000, s.ibat_ua / 1000);
}

/*
 * Run the PPS state machine once; returns the delay to the next run in
 * milliseconds, or 0 if nothing has to be watched.
 */
static unsigned int qcom_battmgr_pps_update(struct qcom_battmgr *battmgr)
{
	u32 online;

	if (!qcom_battmgr_pps_enabled(battmgr)) {
		if (battmgr->pps_state == PPS_ACTIVE)
			qcom_battmgr_pps_stop(battmgr, PPS_IDLE, "disabled");
		battmgr->pps_state = PPS_IDLE;
		return 0;
	}

	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_ONLINE, &online))
		return CHARGE_CTRL_POLL_MS;

	if (!online) {
		if (battmgr->pps_state == PPS_ACTIVE)
			qcom_battmgr_pps_stop(battmgr, PPS_IDLE, "charger removed");
		battmgr->pps_state = PPS_IDLE;
		return CHARGE_CTRL_POLL_MS;
	}

	switch (battmgr->pps_state) {
	case PPS_IDLE:
	case PPS_HOLD:
		if (!qcom_battmgr_pps_charger(battmgr) ||
		    !qcom_battmgr_pps_may_start(battmgr))
			return CHARGE_CTRL_POLL_MS;

		battmgr->pps_state = PPS_SETTLE;
		battmgr->pps_since = jiffies;
		return PPS_POLL_MS;
	case PPS_SETTLE:
		if (!qcom_battmgr_pps_charger(battmgr) ||
		    !qcom_battmgr_pps_may_start(battmgr)) {
			battmgr->pps_state = PPS_IDLE;
			return CHARGE_CTRL_POLL_MS;
		}

		if (time_before(jiffies, battmgr->pps_since +
				msecs_to_jiffies(PPS_SETTLE_MS)))
			return PPS_POLL_MS;

		if (qcom_battmgr_pps_start(battmgr))
			return CHARGE_CTRL_POLL_MS;

		return PPS_POLL_MS;
	case PPS_ACTIVE:
		qcom_battmgr_pps_monitor(battmgr);
		return battmgr->pps_state == PPS_ACTIVE ? PPS_POLL_MS : CHARGE_CTRL_POLL_MS;
	case PPS_FAILED:
	default:
		return CHARGE_CTRL_POLL_MS;
	}
}

/* Match only the Type-C supply belonging to this PMIC GLINK instance. */
static bool qcom_battmgr_is_ucsi_supply(struct qcom_battmgr *battmgr,
				      struct power_supply *psy)
{
	struct device *parent = psy->dev.parent;

	return parent && parent != battmgr->dev &&
	       parent->parent == battmgr->dev->parent &&
	       psy->desc->type == POWER_SUPPLY_TYPE_USB;
}

struct qcom_battmgr_pd_source {
	struct qcom_battmgr *battmgr;
	bool sink;
};

static int qcom_battmgr_pd_source_match(struct power_supply *psy, void *data)
{
	struct qcom_battmgr_pd_source *src = data;
	union power_supply_propval val;

	if (!qcom_battmgr_is_ucsi_supply(src->battmgr, psy))
		return 0;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &val) ||
	    !val.intval)
		return 0;
	if (power_supply_get_property(psy, POWER_SUPPLY_PROP_USB_TYPE, &val))
		return 0;

	src->sink = val.intval == POWER_SUPPLY_USB_TYPE_PD ||
		    val.intval == POWER_SUPPLY_USB_TYPE_PD_PPS;
	return src->sink;
}

/* Read fresh capabilities; never infer a 9 V fixed PDO from an APDO range. */
static int qcom_battmgr_pd_current(struct qcom_battmgr *battmgr, u32 *ua)
{
	struct qcom_battmgr_read_pdos_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_REQ_RESP),
		.hdr.opcode = cpu_to_le32(BATTMGR_OPLUS_READ_PDOS),
		.data_size = cpu_to_le32(OPLUS_READ_BUFFER_WORDS * sizeof(__le32)),
	};
	unsigned int i;
	u32 pdo;
	int ret;

	*ua = 0;
	mutex_lock(&battmgr->lock);
	battmgr->num_src_pdos = 0;
	ret = __qcom_battmgr_request(battmgr, &req, sizeof(req),
				     msecs_to_jiffies(BATTMGR_SET_TIMEOUT_MS));
	if (ret)
		goto out;

	/* A valid source list starts with a nonzero 5 V fixed PDO. */
	pdo = battmgr->src_pdos[0];
	if (!battmgr->num_src_pdos || pdo_type(pdo) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(pdo) != PD_DEFAULT_MV || !pdo_max_current(pdo)) {
		ret = -ENODATA;
		goto out;
	}

	for (i = 0; i < battmgr->num_src_pdos; i++) {
		pdo = battmgr->src_pdos[i];
		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == PD_FIXED_MV) {
			*ua = min_t(u32, pdo_max_current(pdo) * 1000, PD_INPUT_MAX_UA);
			break;
		}
	}
out:
	mutex_unlock(&battmgr->lock);
	return ret;
}

/* Record measured input, not the value merely accepted by a SET response. */
static void qcom_battmgr_pd_log(struct qcom_battmgr *battmgr, u32 type)
{
	const char *name;
	u32 uv, ua, icl;

	if (battmgr->pd_logged ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, &uv) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET, USB_CURR_NOW, &ua) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_INPUT_CURR_LIMIT, &icl))
		return;

	switch (type) {
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case OPLUS_USB_TYPE_PD_SDP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		name = battmgr->pps_state == PPS_ACTIVE ? "PPS" : "PD fixed";
		break;
	case POWER_SUPPLY_USB_TYPE_SDP:
		name = "SDP";
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		name = "CDP";
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
		name = "DCP";
		break;
	default:
		name = "USB";
		break;
	}

	if (battmgr->pps_state == PPS_ACTIVE &&
	    (!battmgr->pps_polls || uv < PPS_VBUS_RISEN_UV))
		return;

	dev_info(battmgr->dev, "USB contract: %s, VBUS %u mV, IBUS %u mA, ICL %u mA\n",
		 name, uv / 1000, ua / 1000, icl / 1000);
	battmgr->pd_logged = true;
}

/* Restore the host's previous ICL only after asking the firmware for 5 V. */
static int qcom_battmgr_pd_stop(struct qcom_battmgr *battmgr, bool offline,
			      const char *reason)
{
	u32 vbus;
	int ret, err;

	if (!battmgr->pd_restore)
		return 0;

	err = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_INPUT_CURR_LIMIT,
					min(battmgr->pd_input_limit_ua, PD_TRANSITION_UA));
	ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
					BATT_OPLUS_SET_PDO, PD_DEFAULT_MV);
	if (!ret && !offline) {
		msleep(PD_SHUTDOWN_WAIT_MS);
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_VOLT_NOW, &vbus);
		if (!ret && vbus >= PD_VBUS_RISEN_UV)
			ret = -EAGAIN;
	}
	if (!ret || offline) {
		err = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT, battmgr->pd_saved_limit_ua);
		if (!err || offline)
			battmgr->pd_restore = false;
	}
	err = ret ?: err;
	battmgr->pd_state = battmgr->pd_restore ? PD_FAILED : PD_IDLE;
	battmgr->pd_logged = false;
	if (err)
		dev_warn(battmgr->dev, "PD: restore 5 V (%s) failed: %d\n", reason, err);
	else
		dev_info(battmgr->dev, "PD: restored 5 V (%s)\n", reason);
	return err;
}

/* Called with @charge_lock held, before the PPS state machine. */
static unsigned int qcom_battmgr_pd_update(struct qcom_battmgr *battmgr)
{
	struct qcom_battmgr_pd_source src = { .battmgr = battmgr };
	bool prefer_pps, caps_changed, current_changed = false;
	u32 online, type, health, vbat, vbus, ua;
	int ret;

	if (test_and_clear_bit(PD_EVENT_RESET, &battmgr->pd_events)) {
		battmgr->pd_state = PD_IDLE;
		battmgr->pd_restore = false;
		battmgr->pd_logged = false;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	}

	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_ONLINE, &online))
		return CHARGE_CTRL_POLL_MS;
	if (!online) {
		qcom_battmgr_pd_stop(battmgr, true, "charger removed");
		battmgr->pd_state = PD_IDLE;
		battmgr->pd_logged = false;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		return qcom_battmgr_pps_update(battmgr);
	}

	if (qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
				      USB_ADAP_TYPE, &type))
		return CHARGE_CTRL_POLL_MS;

	power_supply_for_each_psy(&src, qcom_battmgr_pd_source_match);
	if (!qcom_battmgr_pps_enabled(battmgr) && battmgr->pps_state != PPS_IDLE)
		qcom_battmgr_pps_update(battmgr);

	prefer_pps = src.sink && !READ_ONCE(battmgr->otg_request) &&
		     !(battmgr->pd_state == PD_FAILED && battmgr->pd_restore) &&
		     qcom_battmgr_pps_enabled(battmgr) &&
		     battmgr->pps_state != PPS_FAILED &&
		     qcom_battmgr_pps_charger(battmgr) &&
		     (battmgr->pps_state == PPS_ACTIVE || qcom_battmgr_pps_may_start(battmgr));
	if (prefer_pps) {
		if (qcom_battmgr_pd_stop(battmgr, false, "PPS handover"))
			return CHARGE_CTRL_POLL_MS;
		battmgr->pd_state = PD_IDLE;
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		ret = qcom_battmgr_pps_update(battmgr);
		if (battmgr->pps_state == PPS_ACTIVE)
			qcom_battmgr_pd_log(battmgr, type);
		return ret;
	}
	/* Complete a PPS exit before allowing a fixed PDO request. */
	if (battmgr->pps_state == PPS_ACTIVE)
		qcom_battmgr_pps_stop(battmgr, PPS_HOLD, "fixed PD handover");
	else if (battmgr->pps_state == PPS_SETTLE)
		battmgr->pps_state = PPS_IDLE;

	if (!src.sink || READ_ONCE(battmgr->otg_request) ||
	    (type != POWER_SUPPLY_USB_TYPE_PD && type != POWER_SUPPLY_USB_TYPE_PD_DRP &&
	     type != POWER_SUPPLY_USB_TYPE_PD_PPS && type != OPLUS_USB_TYPE_PD_SDP)) {
		qcom_battmgr_pd_stop(battmgr, false, "no PD sink");
		qcom_battmgr_pd_log(battmgr, type);
		return 0;
	}

	if (battmgr->pd_state == PD_FAILED) {
		qcom_battmgr_pd_log(battmgr, type);
		return 0;
	}

	if (qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET, BATT_HEALTH, &health) ||
	    qcom_battmgr_get_property(battmgr, BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, &vbat))
		return CHARGE_CTRL_POLL_MS;
	if (battmgr->charge_inhibited || health != POWER_SUPPLY_HEALTH_GOOD ||
	    vbat >= (battmgr->pd_restore ? PD_VBAT_STOP_UV : PD_VBAT_START_UV)) {
		qcom_battmgr_pd_stop(battmgr, false, "battery limit");
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		qcom_battmgr_pd_log(battmgr, type);
		return CHARGE_CTRL_POLL_MS;
	}

	caps_changed = test_and_clear_bit(PD_EVENT_CAPS, &battmgr->pd_events);
	if (caps_changed) {
		ret = qcom_battmgr_pd_current(battmgr, &ua);
		if (ret || !ua) {
			dev_info(battmgr->dev, "PD: no verified 9 V fixed PDO (error %d)\n", ret);
			qcom_battmgr_pd_stop(battmgr, false, "source capabilities");
			qcom_battmgr_pd_log(battmgr, type);
			return 0;
		}
		current_changed = ua != battmgr->pd_input_limit_ua;
		battmgr->pd_input_limit_ua = ua;
	}

	switch (battmgr->pd_state) {
	case PD_IDLE:
		if (!caps_changed)
			return 0;
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_INPUT_CURR_LIMIT, &battmgr->pd_saved_limit_ua);
		if (ret || !battmgr->pd_saved_limit_ua) {
			qcom_battmgr_pd_log(battmgr, type);
			return 0;
		}
		/* Also undo a timed-out write, since the firmware may have applied it. */
		battmgr->pd_restore = true;
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_INPUT_CURR_LIMIT,
						min(battmgr->pd_input_limit_ua, PD_TRANSITION_UA));
		if (!ret)
			ret = qcom_battmgr_set_property(battmgr, BATTMGR_BAT_PROPERTY_SET,
							BATT_OPLUS_SET_PDO, PD_FIXED_MV);
		if (ret)
			goto failed;
		battmgr->pd_state = PD_WAIT;
		battmgr->pd_since = jiffies;
		battmgr->pd_logged = false;
		dev_info(battmgr->dev, "PD: requested 9000 mV, input limit up to %u mA\n",
			 battmgr->pd_input_limit_ua / 1000);
		return PD_POLL_MS;
	case PD_WAIT:
	case PD_ACTIVE:
		ret = qcom_battmgr_get_property(battmgr, BATTMGR_USB_PROPERTY_GET,
						USB_VOLT_NOW, &vbus);
		if (ret)
			goto failed;
		if (vbus < PD_VBUS_RISEN_UV) {
			if (battmgr->pd_state == PD_WAIT &&
			    time_before(jiffies, battmgr->pd_since +
					msecs_to_jiffies(PD_CONFIG_TIMEOUT_MS)))
				return PD_POLL_MS;
			ret = -ETIMEDOUT;
			goto failed;
		}
		if (vbus >= PD_VBUS_MAX_UV) {
			ret = -ERANGE;
			goto failed;
		}
		if (battmgr->pd_state == PD_WAIT || current_changed) {
			ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
							USB_INPUT_CURR_LIMIT,
							battmgr->pd_input_limit_ua);
			if (ret)
				goto failed;
		}
		battmgr->pd_state = PD_ACTIVE;
		qcom_battmgr_pd_log(battmgr, type);
		return CHARGE_CTRL_POLL_MS;
	default:
		return 0;
	}

failed:
	dev_warn(battmgr->dev, "PD: 9 V request failed: %d\n", ret);
	qcom_battmgr_pd_stop(battmgr, false, "request failed");
	battmgr->pd_state = PD_FAILED;
	qcom_battmgr_pd_log(battmgr, type);
	return 0;
}

static void qcom_battmgr_charge_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(to_delayed_work(work),
						    struct qcom_battmgr, charge_work);
	unsigned int delay = 0;

	/* Kicked again by qcom_battmgr_enable_worker() once the service is up */
	if (!battmgr->service_up || READ_ONCE(battmgr->removing))
		return;

	mutex_lock(&battmgr->charge_lock);

	qcom_battmgr_charge_limit_update(battmgr);

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)
		delay = qcom_battmgr_pd_update(battmgr);
	else if (battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS)
		delay = qcom_battmgr_pps_update(battmgr);

	/* Also retry turning charging back on after the limit was removed */
	if (!delay && (qcom_battmgr_charge_limited(battmgr) || battmgr->charge_inhibited ||
		       qcom_battmgr_pps_enabled(battmgr)))
		delay = CHARGE_CTRL_POLL_MS;

	mutex_unlock(&battmgr->charge_lock);

	if (delay && !READ_ONCE(battmgr->removing))
		queue_delayed_work(system_freezable_wq, &battmgr->charge_work,
				   msecs_to_jiffies(delay));
}

/* Check the charge state now, may be called in atomic context */
static void qcom_battmgr_charge_kick(struct qcom_battmgr *battmgr)
{
	if ((battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) &&
	    !READ_ONCE(battmgr->removing))
		mod_delayed_work(system_freezable_wq, &battmgr->charge_work, 0);
}

/*
 * A charger came or went: the firmware may have turned charging back on, so
 * check right away instead of at the next poll.  Called in atomic context.
 */
static void qcom_battmgr_usb_event(struct qcom_battmgr *battmgr)
{
	if (qcom_battmgr_charge_limited(battmgr) || battmgr->charge_inhibited ||
	    qcom_battmgr_pps_enabled(battmgr) ||
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		qcom_battmgr_charge_kick(battmgr);
}

static int qcom_battmgr_psy_notify(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct qcom_battmgr *battmgr = container_of(nb, struct qcom_battmgr, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && qcom_battmgr_is_ucsi_supply(battmgr, psy)) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing) {
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			qcom_battmgr_usb_event(battmgr);
		}
	}
	return NOTIFY_OK;
}

/*
 * A PPS contract is not monitored while the system is suspended, so end it
 * before and let the work start it again after resume.
 */
static int qcom_battmgr_pm_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct qcom_battmgr *battmgr = container_of(nb, struct qcom_battmgr, pm_nb);

	switch (action) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		cancel_delayed_work_sync(&battmgr->charge_work);
		mutex_lock(&battmgr->charge_lock);
		if (battmgr->pps_state == PPS_ACTIVE)
			qcom_battmgr_pps_stop(battmgr, PPS_HOLD, "system suspend");
		mutex_unlock(&battmgr->charge_lock);
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		qcom_battmgr_charge_kick(battmgr);
		break;
	}

	return NOTIFY_DONE;
}

static int qcom_battmgr_bat_is_writeable(struct power_supply *psy,
					 enum power_supply_property psp)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);

	if (qcom_battmgr_no_charge_ctrl(battmgr))
		return 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		return 0;
	}

	return 0;
}

static int qcom_battmgr_bat_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *pval)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (qcom_battmgr_no_charge_ctrl(battmgr))
		return -EOPNOTSUPP;

	if (!battmgr->service_up)
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		ret = qcom_battmgr_set_charge_start_threshold(battmgr, pval->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		ret = qcom_battmgr_set_charge_end_threshold(battmgr, pval->intval);
		break;
	default:
		return -EINVAL;
	}

	qcom_battmgr_charge_kick(battmgr);

	return ret;
}

static const enum power_supply_property sc8280xp_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc sc8280xp_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sc8280xp_bat_props,
	.num_properties = ARRAY_SIZE(sc8280xp_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property x1e80100_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_ENERGY_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MANUFACTURE_YEAR,
	POWER_SUPPLY_PROP_MANUFACTURE_MONTH,
	POWER_SUPPLY_PROP_MANUFACTURE_DAY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc x1e80100_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = x1e80100_bat_props,
	.num_properties = ARRAY_SIZE(x1e80100_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static const enum power_supply_property sm8350_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
};

static const struct power_supply_desc sm8350_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8350_bat_props,
	.num_properties = ARRAY_SIZE(sm8350_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
};

static const enum power_supply_property sm8550_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_INTERNAL_RESISTANCE,
	POWER_SUPPLY_PROP_STATE_OF_HEALTH,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
};

static const struct power_supply_desc sm8550_bat_psy_desc = {
	.name = "qcom-battmgr-bat",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = sm8550_bat_props,
	.num_properties = ARRAY_SIZE(sm8550_bat_props),
	.get_property = qcom_battmgr_bat_get_property,
	.set_property = qcom_battmgr_bat_set_property,
	.property_is_writeable = qcom_battmgr_bat_is_writeable,
};

static int qcom_battmgr_ac_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->ac.online;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_ac_psy_desc = {
	.name = "qcom-battmgr-ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = sc8280xp_ac_props,
	.num_properties = ARRAY_SIZE(sc8280xp_ac_props),
	.get_property = qcom_battmgr_ac_get_property,
};

static const u8 sm8350_usb_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = USB_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = USB_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = USB_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = USB_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = USB_CURR_MAX,
	[POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT] = USB_INPUT_CURR_LIMIT,
	[POWER_SUPPLY_PROP_USB_TYPE] = USB_ADAP_TYPE,
};

static int qcom_battmgr_usb_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_usb_prop_map))
		return -EINVAL;

	prop = sm8350_usb_prop_map[psp];
	if (psp == POWER_SUPPLY_PROP_USB_TYPE &&
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		prop = USB_ADAP_TYPE;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_USB_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_usb_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_usb_sm8350_update(battmgr, psp);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->usb.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->usb.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->usb.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->usb.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->usb.current_max;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = battmgr->usb.current_limit;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = battmgr->usb.usb_type;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sc8280xp_usb_props,
	.num_properties = ARRAY_SIZE(sc8280xp_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const enum power_supply_property sm8350_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc sm8350_usb_psy_desc = {
	.name = "qcom-battmgr-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = sm8350_usb_props,
	.num_properties = ARRAY_SIZE(sm8350_usb_props),
	.get_property = qcom_battmgr_usb_get_property,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
		     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
		     BIT(POWER_SUPPLY_USB_TYPE_C)       |
		     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
		     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
		     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
};

static const u8 sm8350_wls_prop_map[] = {
	[POWER_SUPPLY_PROP_ONLINE] = WLS_ONLINE,
	[POWER_SUPPLY_PROP_VOLTAGE_NOW] = WLS_VOLT_NOW,
	[POWER_SUPPLY_PROP_VOLTAGE_MAX] = WLS_VOLT_MAX,
	[POWER_SUPPLY_PROP_CURRENT_NOW] = WLS_CURR_NOW,
	[POWER_SUPPLY_PROP_CURRENT_MAX] = WLS_CURR_MAX,
};

static int qcom_battmgr_wls_sm8350_update(struct qcom_battmgr *battmgr,
					  enum power_supply_property psp)
{
	unsigned int prop;
	int ret;

	if (psp >= ARRAY_SIZE(sm8350_wls_prop_map))
		return -EINVAL;

	prop = sm8350_wls_prop_map[psp];

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request_property(battmgr, BATTMGR_WLS_PROPERTY_GET, prop, 0);
	mutex_unlock(&battmgr->lock);

	return ret;
}

static int qcom_battmgr_wls_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!battmgr->service_up)
		return -EAGAIN;

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100)
		ret = qcom_battmgr_bat_sc8280xp_update(battmgr, psp);
	else
		ret = qcom_battmgr_wls_sm8350_update(battmgr, psp);
	if (ret < 0)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = battmgr->wireless.online;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battmgr->wireless.voltage_now;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = battmgr->wireless.voltage_max;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battmgr->wireless.current_now;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = battmgr->wireless.current_max;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property sc8280xp_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sc8280xp_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sc8280xp_wls_props,
	.num_properties = ARRAY_SIZE(sc8280xp_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static const enum power_supply_property sm8350_wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc sm8350_wls_psy_desc = {
	.name = "qcom-battmgr-wls",
	.type = POWER_SUPPLY_TYPE_WIRELESS,
	.properties = sm8350_wls_props,
	.num_properties = ARRAY_SIZE(sm8350_wls_props),
	.get_property = qcom_battmgr_wls_get_property,
};

static void qcom_battmgr_notification(struct qcom_battmgr *battmgr,
				      const struct qcom_battmgr_message *msg,
				      int len)
{
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int notification;

	if (payload_len != sizeof(msg->notification)) {
		dev_warn(battmgr->dev, "ignoring notification with invalid length\n");
		return;
	}

	notification = le32_to_cpu(msg->notification);
	if (!(battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD))
		notification &= 0xff;
	switch (notification) {
	case NOTIF_BAT_INFO:
		battmgr->info.valid = false;
		fallthrough;
	case NOTIF_BAT_STATUS:
	case NOTIF_BAT_PROPERTY:
	case NOTIF_BAT_CHARGING_STATE:
		power_supply_changed(battmgr->bat_psy);
		break;
	case NOTIF_USB_PROPERTY:
		power_supply_changed(battmgr->usb_psy);
		qcom_battmgr_usb_event(battmgr);
		break;
	case NOTIF_WLS_PROPERTY:
		power_supply_changed(battmgr->wls_psy);
		break;
	case NOTIF_OPLUS_OTG_ENABLE:
	case NOTIF_OPLUS_OTG_DISABLE:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OTG_VBUS) {
			WRITE_ONCE(battmgr->otg_request,
				   notification == NOTIF_OPLUS_OTG_ENABLE);
			queue_work(system_freezable_wq, &battmgr->otg_work);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		fallthrough;
	case NOTIF_OPLUS_CID_DETECT:
	case NOTIF_OPLUS_SOURCECAP_DONE:
	case NOTIF_OPLUS_TYPEC_STATE:
	case NOTIF_OPLUS_PLUGIN:
	case NOTIF_OPLUS_APSD_DONE:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) {
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			power_supply_changed(battmgr->usb_psy);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		fallthrough;
	case NOTIF_OPLUS_UFCS_DISABLE_MOS:
	case NOTIF_OPLUS_POWER_ROLE:
	case NOTIF_OPLUS_PD_HARD_RESET:
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
			set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
			qcom_battmgr_usb_event(battmgr);
			break;
		}
		/*
		 * UFCS charge pump and reverse charging events, for features
		 * this driver does not have; the power role reaches the OS
		 * through UCSI.
		 */
		if (battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE)
			break;
		fallthrough;
	default:
		dev_err(battmgr->dev, "unknown notification: %#x\n", notification);
		break;
	}
}

static void qcom_battmgr_sc8280xp_strcpy(char *dest, const char *src)
{
	size_t len = src[0];

	/* Some firmware versions return Pascal-style strings */
	if (len < BATTMGR_STRING_LEN && len == strnlen(src + 1, BATTMGR_STRING_LEN - 1)) {
		memcpy(dest, src + 1, len);
		dest[len] = '\0';
	} else {
		strscpy(dest, src, BATTMGR_STRING_LEN);
	}
}

static unsigned int qcom_battmgr_sc8280xp_parse_technology(const char *chemistry)
{
	if ((!strncmp(chemistry, "LIO", 3)) ||
	    (!strncmp(chemistry, "OOI", 3)))
		return POWER_SUPPLY_TECHNOLOGY_LION;
	if (!strncmp(chemistry, "LIP", 3) ||
	    !strncmp(chemistry, "LiP", 3))
		return POWER_SUPPLY_TECHNOLOGY_LIPO;

	pr_err("Unknown battery technology '%s'\n", chemistry);
	return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static unsigned int qcom_battmgr_sc8280xp_convert_temp(unsigned int temperature)
{
	return DIV_ROUND_CLOSEST(temperature, 10);
}

/*
 * The charger supplies' online state comes from the battery status response,
 * and the firmware sends no notification of its own when it changes, so
 * announce the change here. A supply that is not registered yet announces its
 * state when it is.
 */
static void qcom_battmgr_sc8280xp_set_online(struct power_supply *psy,
					     bool *online, bool value)
{
	if (*online == value)
		return;

	*online = value;
	if (psy)
		power_supply_changed(psy);
}

static void qcom_battmgr_sc8280xp_callback(struct qcom_battmgr *battmgr,
					   const struct qcom_battmgr_message *resp,
					   size_t len)
{
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	unsigned int power_state;
	unsigned int source;
	unsigned int state;
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_REQUEST_NOTIFICATION:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_INFO:
		/* some firmware versions report an extra __le32 at the end of the payload */
		if (payload_len != sizeof(resp->info) &&
		    payload_len != (sizeof(resp->info) + sizeof(__le32))) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery information request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->unit = le32_to_cpu(resp->info.power_unit);

		battmgr->info.present = true;
		battmgr->info.design_capacity = le32_to_cpu(resp->info.design_capacity) * 1000;
		battmgr->info.last_full_capacity = le32_to_cpu(resp->info.last_full_capacity) * 1000;
		battmgr->info.voltage_max_design = le32_to_cpu(resp->info.design_voltage) * 1000;
		battmgr->info.capacity_low = le32_to_cpu(resp->info.capacity_low) * 1000;
		battmgr->info.cycle_count = le32_to_cpu(resp->info.cycle_count);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.model_number, resp->info.model_number);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.serial_number, resp->info.serial_number);
		battmgr->info.technology = qcom_battmgr_sc8280xp_parse_technology(resp->info.battery_chemistry);
		qcom_battmgr_sc8280xp_strcpy(battmgr->info.oem_info, resp->info.oem_info);
		battmgr->info.day = resp->info.day;
		battmgr->info.month = resp->info.month;
		battmgr->info.year = le16_to_cpu(resp->info.year);
		break;
	case BATTMGR_BAT_STATUS:
		if (payload_len != sizeof(resp->status)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for battery status request: %zd\n",
				 payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->status.capacity = le32_to_cpu(resp->status.capacity) * 1000;
		battmgr->status.power_now = le32_to_cpu(resp->status.rate) * 1000;
		battmgr->status.voltage_now = le32_to_cpu(resp->status.battery_voltage) * 1000;
		battmgr->status.temperature = qcom_battmgr_sc8280xp_convert_temp(le32_to_cpu(resp->status.temperature));

		state = le32_to_cpu(resp->status.battery_state);
		power_state = le32_to_cpu(resp->status.power_state);
		if (state & BIT(0))
			battmgr->status.status = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (state & BIT(1))
			battmgr->status.status = POWER_SUPPLY_STATUS_CHARGING;
		else if ((power_state & BIT(0)) &&
			 battmgr->info.last_full_capacity &&
			 battmgr->status.capacity >= battmgr->info.last_full_capacity)
			battmgr->status.status = POWER_SUPPLY_STATUS_FULL;
		else
			battmgr->status.status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		source = le32_to_cpu(resp->status.charging_source);
		qcom_battmgr_sc8280xp_set_online(battmgr->ac_psy, &battmgr->ac.online,
						 source == BATTMGR_CHARGING_SOURCE_AC);
		qcom_battmgr_sc8280xp_set_online(battmgr->usb_psy, &battmgr->usb.online,
						 source == BATTMGR_CHARGING_SOURCE_USB);
		qcom_battmgr_sc8280xp_set_online(battmgr->wls_psy, &battmgr->wireless.online,
						 source == BATTMGR_CHARGING_SOURCE_WIRELESS);
		if (battmgr->info.last_full_capacity != 0) {
			/*
			 * 100 * battmgr->status.capacity can overflow a 32bit
			 * unsigned integer. FW readings are in m{W/A}h, which
			 * are multiplied by 1000 converting them to u{W/A}h,
			 * the format the power_supply API expects.
			 * To avoid overflow use the original value for dividend
			 * and convert the divider back to m{W/A}h, which can be
			 * done without any loss of precision.
			 */
			battmgr->status.percent =
				(100 * le32_to_cpu(resp->status.capacity)) /
				(battmgr->info.last_full_capacity / 1000);
		} else {
			/*
			 * Let the sysfs handler know no data is available at
			 * this time.
			 */
			battmgr->status.percent = (unsigned int)-1;
		}
		break;
	case BATTMGR_BAT_DISCHARGE_TIME:
		battmgr->status.discharge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_BAT_CHARGE_TIME:
		battmgr->status.charge_time = le32_to_cpu(resp->time);
		break;
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

	complete(&battmgr->ack);
}

static void qcom_battmgr_sm8350_callback(struct qcom_battmgr *battmgr,
					 const struct qcom_battmgr_message *resp,
					 size_t len)
{
	unsigned int property;
	unsigned int opcode = le32_to_cpu(resp->hdr.opcode);
	size_t payload_len = len - sizeof(struct pmic_glink_hdr);
	unsigned int val;

	if (payload_len < sizeof(__le32)) {
		dev_warn(battmgr->dev, "invalid payload length for %#x: %zd\n",
			 opcode, len);
		return;
	}

	switch (opcode) {
	case BATTMGR_BAT_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (property == BATT_MODEL_NAME) {
			if (payload_len != sizeof(resp->strval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for BATT_MODEL_NAME request: %zd\n",
					 payload_len);
				battmgr->error = -ENODATA;
				return;
			}
		} else {
			if (payload_len != sizeof(resp->intval)) {
				dev_warn(battmgr->dev,
					 "invalid payload length for %#x request: %zd\n",
					 property, payload_len);
				battmgr->error = -ENODATA;
				return;
			}

			battmgr->error = le32_to_cpu(resp->intval.result);
			if (battmgr->error)
				goto out_complete;

			battmgr->resp_property = property;
			battmgr->resp_value = le32_to_cpu(resp->intval.value);
		}

		switch (property) {
		case BATT_STATUS:
			battmgr->status.status = le32_to_cpu(resp->intval.value);
			break;
		case BATT_HEALTH:
			battmgr->status.health = le32_to_cpu(resp->intval.value);
			break;
		case BATT_PRESENT:
			battmgr->info.present = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_TYPE:
			battmgr->info.charge_type = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CAPACITY:
			battmgr->status.percent = le32_to_cpu(resp->intval.value) / 100;
			break;
		case BATT_SOH:
			battmgr->status.soh_percent = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_OCV:
			battmgr->status.voltage_ocv = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_NOW:
			battmgr->status.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_VOLT_MAX:
			battmgr->info.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CURR_NOW:
			battmgr->status.current_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TEMP:
			val = le32_to_cpu(resp->intval.value);
			battmgr->status.temperature = DIV_ROUND_CLOSEST(val, 10);
			break;
		case BATT_TECHNOLOGY:
			battmgr->info.technology = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_COUNTER:
			battmgr->info.charge_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CYCLE_COUNT:
			battmgr->info.cycle_count = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL_DESIGN:
			battmgr->info.design_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_FULL:
			battmgr->info.last_full_capacity = le32_to_cpu(resp->intval.value);
			break;
		case BATT_MODEL_NAME:
			strscpy(battmgr->info.model_number, resp->strval.model, BATTMGR_STRING_LEN);
			break;
		case BATT_TTF_AVG:
			battmgr->status.charge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_TTE_AVG:
			battmgr->status.discharge_time = le32_to_cpu(resp->intval.value);
			break;
		case BATT_RESISTANCE:
			battmgr->status.resistance = le32_to_cpu(resp->intval.value);
			break;
		case BATT_POWER_NOW:
			battmgr->status.power_now = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_START_THR:
			battmgr->info.charge_ctrl_start = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_END_THR:
			battmgr->info.charge_ctrl_end = le32_to_cpu(resp->intval.value);
			break;
		case BATT_CHG_CTRL_LIM:
			/* Only read through qcom_battmgr_get_property() */
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_USB_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		battmgr->resp_property = property;
		battmgr->resp_value = le32_to_cpu(resp->intval.value);

		switch (property) {
		case USB_ONLINE:
			battmgr->usb.online = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_NOW:
			battmgr->usb.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_VOLT_MAX:
			battmgr->usb.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_NOW:
			battmgr->usb.current_now = le32_to_cpu(resp->intval.value);
			break;
		case USB_CURR_MAX:
			battmgr->usb.current_max = le32_to_cpu(resp->intval.value);
			break;
		case USB_INPUT_CURR_LIMIT:
			battmgr->usb.current_limit = le32_to_cpu(resp->intval.value);
			break;
		case USB_ADAP_TYPE:
			battmgr->usb.usb_type = le32_to_cpu(resp->intval.value);
			break;
		case USB_ADAP_TYPE:
			val = le32_to_cpu(resp->intval.value);
			if (val == OPLUS_USB_TYPE_PD_SDP)
				val = POWER_SUPPLY_USB_TYPE_PD;
			else if (val >= 0x80 && val <= 0x82)
				val = POWER_SUPPLY_USB_TYPE_DCP; /* Qualcomm HVDCP */
			else if (val == OPLUS_USB_TYPE_APPLE_BRICK_ID)
				val = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID;
			else if (val > POWER_SUPPLY_USB_TYPE_PD_PPS)
				val = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			battmgr->usb.usb_type = val;
			break;
		case USB_REAL_TYPE:
		case USB_OPLUS_PPS_TYPE:
		case USB_OPLUS_PPS_MAX_CURR:
			/* Only read through qcom_battmgr_get_property() */
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_WLS_PROPERTY_GET:
		property = le32_to_cpu(resp->intval.property);
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x request: %zd\n",
				 property, payload_len);
			battmgr->error = -ENODATA;
			return;
		}

		battmgr->error = le32_to_cpu(resp->intval.result);
		if (battmgr->error)
			goto out_complete;

		switch (property) {
		case WLS_ONLINE:
			battmgr->wireless.online = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_NOW:
			battmgr->wireless.voltage_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_VOLT_MAX:
			battmgr->wireless.voltage_max = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_NOW:
			battmgr->wireless.current_now = le32_to_cpu(resp->intval.value);
			break;
		case WLS_CURR_MAX:
			battmgr->wireless.current_max = le32_to_cpu(resp->intval.value);
			break;
		default:
			dev_warn(battmgr->dev, "unknown property %#x\n", property);
			break;
		}
		break;
	case BATTMGR_REQUEST_NOTIFICATION:
	case BATTMGR_CHG_CTRL_LIMIT_EN:
		battmgr->error = 0;
		break;
	case BATTMGR_BAT_PROPERTY_SET:
	case BATTMGR_USB_PROPERTY_SET:
		if (payload_len != sizeof(resp->intval)) {
			dev_warn(battmgr->dev,
				 "invalid payload length for %#x response: %zd\n",
				 opcode, payload_len);
			battmgr->error = -ENODATA;
			break;
		}

		/* The firmware answers a write with a nonzero code if it refused it */
		if (le32_to_cpu(resp->intval.result)) {
			battmgr->error = -EREMOTEIO;
			break;
		}

		battmgr->resp_property = le32_to_cpu(resp->intval.property);
		break;
	default:
		dev_warn(battmgr->dev, "unknown message %#x\n", opcode);
		break;
	}

out_complete:
	complete(&battmgr->ack);
}

/* AP_OPCODE_PD_INFO_BUFFER carries partner identity VDOs, not a PDO reply. */
static void qcom_battmgr_pd_info(struct qcom_battmgr *battmgr,
			       const struct qcom_battmgr_pd_info *msg, size_t len)
{
	u32 size, id;
	unsigned int i;

	if (len != sizeof(*msg))
		goto invalid;

	size = le32_to_cpu(msg->data_size);
	id = le32_to_cpu(msg->msg_id);
	if (!size || size > sizeof(msg->data) || size % sizeof(__le32))
		goto invalid;
	if (id != OPLUS_PD_INFO_VDM_ID) {
		dev_dbg(battmgr->dev, "unhandled PD info id %u\n", id);
		return;
	}

	for (i = 0; i < size / sizeof(__le32); i++)
		dev_dbg(battmgr->dev, "PD identity VDO %u: %#08x\n",
			i, le32_to_cpu(msg->data[i]));
	return;

invalid:
	dev_warn_ratelimited(battmgr->dev, "invalid PD identity buffer\n");
}

static void qcom_battmgr_pdos_response(struct qcom_battmgr *battmgr,
				     const struct qcom_battmgr_read_pdos_response *msg,
				     size_t len)
{
	unsigned int i, words;
	u32 size;

	battmgr->num_src_pdos = 0;
	battmgr->error = -ENODATA;
	if (len != sizeof(*msg))
		goto out;

	size = le32_to_cpu(msg->data_size);
	if (size < 3 * sizeof(__le32) || size > sizeof(msg->data) ||
	    size % sizeof(__le32))
		goto out;

	/* Words 0/1 are PPS imax/vmax, followed by up to seven raw PDOs. */
	words = min_t(unsigned int, size / sizeof(__le32) - 2, PDO_MAX_OBJECTS);
	for (i = 0; i < words; i++)
		battmgr->src_pdos[i] = le32_to_cpu(msg->data[i + 2]);
	battmgr->num_src_pdos = words;
	battmgr->error = 0;
out:
	complete(&battmgr->ack);
}

static void qcom_battmgr_callback(const void *data, size_t len, void *priv)
{
	const struct pmic_glink_hdr *hdr = data;
	struct qcom_battmgr *battmgr = priv;
	unsigned int opcode = le32_to_cpu(hdr->opcode);

	if (opcode == BATTMGR_OPLUS_PD_INFO &&
	    (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing)
			qcom_battmgr_pd_info(battmgr, data, len);
	} else if (opcode == BATTMGR_NOTIFICATION) {
		guard(spinlock_irqsave)(&battmgr->notify_lock);
		if (!battmgr->removing)
			qcom_battmgr_notification(battmgr, data, len);
	} else if (opcode != READ_ONCE(battmgr->request_opcode)) {
		dev_dbg(battmgr->dev, "unsolicited message %#x\n", opcode);
	} else if (opcode == BATTMGR_OPLUS_READ_PDOS &&
		   (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD)) {
		qcom_battmgr_pdos_response(battmgr, data, len);
	} else if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
		   battmgr->variant == QCOM_BATTMGR_X1E80100) {
		qcom_battmgr_sc8280xp_callback(battmgr, data, len);
	} else {
		qcom_battmgr_sm8350_callback(battmgr, data, len);
	}
}

/*
 * Let the firmware use the port as a USB source.  It then detects a sink on
 * the port and asks for VBUS with NOTIF_OPLUS_OTG_ENABLE.  The firmware does
 * not detect a sink while this is cleared, so it is done once the service
 * is up, before anything is plugged in; it does not turn VBUS on by itself.
 */
static void qcom_battmgr_otg_init(struct qcom_battmgr *battmgr)
{
	int ret;

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_OTG_AP_ENABLE, 1);
	if (!ret)
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_OTG_SWITCH, 1);
	if (ret)
		dev_err(battmgr->dev, "failed to enable USB source mode: %d\n", ret);
}

/*
 * Switch the VBUS boost to what the firmware asked for last: on while it
 * reports a sink, that is while the port is the USB source and host, and
 * off once the sink is gone.  The data role stays with the Type-C driver.
 */
static void qcom_battmgr_otg_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, otg_work);
	bool on = READ_ONCE(battmgr->otg_request);
	int ret;

	if (!battmgr->service_up || on == battmgr->otg_vbus)
		return;

	/* The vendor kernel sets the boost current limit before the boost */
	if (on) {
		ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
						USB_OPLUS_OTG_BOOST_CURRENT,
						OTG_BOOST_CURRENT_MA);
		if (ret)
			dev_warn(battmgr->dev, "failed to set the VBUS current limit: %d\n",
				 ret);
	}

	ret = qcom_battmgr_set_property(battmgr, BATTMGR_USB_PROPERTY_SET,
					USB_OPLUS_OTG_VBUS_ENABLE, on);
	if (ret) {
		dev_err(battmgr->dev, "failed to turn VBUS %s: %d\n",
			on ? "on" : "off", ret);
		return;
	}

	battmgr->otg_vbus = on;
	dev_info(battmgr->dev, "VBUS %s for a USB sink\n", on ? "on" : "off");
}

static void qcom_battmgr_enable_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(work, struct qcom_battmgr, enable_work);
	struct qcom_battmgr_enable_request req = {
		.hdr.owner = cpu_to_le32(PMIC_GLINK_OWNER_BATTMGR),
		.hdr.type = cpu_to_le32(PMIC_GLINK_NOTIFY),
		.hdr.opcode = cpu_to_le32(BATTMGR_REQUEST_NOTIFICATION),
	};
	int ret;

	mutex_lock(&battmgr->lock);
	ret = qcom_battmgr_request(battmgr, &req, sizeof(req));
	mutex_unlock(&battmgr->lock);
	if (ret)
		dev_err(battmgr->dev, "failed to request power notifications\n");

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OTG_VBUS)
		qcom_battmgr_otg_init(battmgr);

	qcom_battmgr_charge_kick(battmgr);
}

static void qcom_battmgr_pdr_notify(void *priv, int state)
{
	struct qcom_battmgr *battmgr = priv;

	guard(spinlock_irqsave)(&battmgr->notify_lock);
	if (battmgr->removing)
		return;

	if (state == SERVREG_SERVICE_STATE_UP) {
		battmgr->service_up = true;
		schedule_work(&battmgr->enable_work);
	} else {
		battmgr->service_up = false;
		/* A restarted firmware charges again by default */
		battmgr->charge_inhibited = false;
		/* ... and starts with the boost off and without a PPS contract */
		battmgr->otg_request = false;
		battmgr->otg_vbus = false;
		battmgr->pps_state = PPS_IDLE;
		set_bit(PD_EVENT_RESET, &battmgr->pd_events);
	}
}

static const struct of_device_id qcom_battmgr_of_variants[] = {
	{ .compatible = "qcom,glymur-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	{ .compatible = "qcom,kaanapali-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,sc8180x-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sc8280xp-pmic-glink", .data = (void *)QCOM_BATTMGR_SC8280XP },
	{ .compatible = "qcom,sm8550-pmic-glink", .data = (void *)QCOM_BATTMGR_SM8550 },
	{ .compatible = "qcom,x1e80100-pmic-glink", .data = (void *)QCOM_BATTMGR_X1E80100 },
	/* Unmatched devices falls back to QCOM_BATTMGR_SM8350 */
	{}
};

/*
 * Devices whose firmware follows the protocol of their SoC but reports some
 * values with a different meaning.
 */
static const struct of_device_id qcom_battmgr_of_quirks[] = {
	{
		.compatible = "oneplus,pad2pro-pmic-glink",
		.data = (void *)(QCOM_BATTMGR_QUIRK_BATT_CURRENT_INVERTED |
				 QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW |
				 QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL |
				 QCOM_BATTMGR_QUIRK_CHARGE_ENABLE |
				 QCOM_BATTMGR_QUIRK_OTG_VBUS |
				 QCOM_BATTMGR_QUIRK_PPS |
				 QCOM_BATTMGR_QUIRK_OPLUS_PD),
	},
	{}
};

static bool qcom_battmgr_bat_prop_hidden(struct qcom_battmgr *battmgr,
					 enum power_supply_property psp)
{
	switch (psp) {
	/* Leave POWER_NOW out rather than report a value that is not one */
	case POWER_SUPPLY_PROP_POWER_NOW:
		return battmgr->quirks & QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW;
	/* Never send a request that the firmware reads as another command */
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return qcom_battmgr_no_charge_ctrl(battmgr);
	default:
		return false;
	}
}

static const struct power_supply_desc *
qcom_battmgr_sm8350_bat_desc(struct qcom_battmgr *battmgr,
			     const struct power_supply_desc *desc)
{
	enum power_supply_property *props;
	struct power_supply_desc *copy;
	size_t i, n = 0;

	if (!(battmgr->quirks & (QCOM_BATTMGR_QUIRK_NO_BATT_POWER_NOW |
				 QCOM_BATTMGR_QUIRK_NO_CHARGE_CONTROL)))
		return desc;

	copy = devm_kmemdup(battmgr->dev, desc, sizeof(*desc), GFP_KERNEL);
	props = devm_kcalloc(battmgr->dev, desc->num_properties, sizeof(*props),
			     GFP_KERNEL);
	if (!copy || !props)
		return NULL;

	for (i = 0; i < desc->num_properties; i++)
		if (!qcom_battmgr_bat_prop_hidden(battmgr, desc->properties[i]))
			props[n++] = desc->properties[i];

	copy->properties = props;
	copy->num_properties = n;

	return copy;
}

static char *qcom_battmgr_battery[] = { "battery" };

static void qcom_battmgr_stop_notifications(void *data)
{
	struct qcom_battmgr *battmgr = data;

	guard(spinlock_irqsave)(&battmgr->notify_lock);
	battmgr->removing = true;
}

static void qcom_battmgr_cancel_work(void *data)
{
	struct qcom_battmgr *battmgr = data;

	cancel_delayed_work_sync(&battmgr->charge_work);
	cancel_work_sync(&battmgr->otg_work);
	mutex_lock(&battmgr->charge_lock);
	if (battmgr->service_up) {
		if (battmgr->pps_state == PPS_ACTIVE)
			qcom_battmgr_pps_stop(battmgr, PPS_IDLE, "driver stopped");
		qcom_battmgr_pd_stop(battmgr, false, "driver stopped");
	}
	mutex_unlock(&battmgr->charge_lock);
}

static void qcom_battmgr_unregister_psy_notifier(void *data)
{
	struct qcom_battmgr *battmgr = data;

	power_supply_unreg_notifier(&battmgr->psy_nb);
}

static void qcom_battmgr_unregister_pm_notifier(void *data)
{
	struct qcom_battmgr *battmgr = data;

	unregister_pm_notifier(&battmgr->pm_nb);
}

static int qcom_battmgr_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	const struct power_supply_desc *psy_desc;
	struct power_supply_config psy_cfg_supply = {};
	struct power_supply_config psy_cfg = {};
	const struct of_device_id *match;
	struct qcom_battmgr *battmgr;
	struct device *dev = &adev->dev;
	int ret;

	battmgr = devm_kzalloc(dev, sizeof(*battmgr), GFP_KERNEL);
	if (!battmgr)
		return -ENOMEM;

	battmgr->dev = dev;
	auxiliary_set_drvdata(adev, battmgr);

	psy_cfg.drv_data = battmgr;
	psy_cfg.fwnode = dev_fwnode(&adev->dev);

	psy_cfg_supply.drv_data = battmgr;
	psy_cfg_supply.fwnode = dev_fwnode(&adev->dev);
	psy_cfg_supply.supplied_to = qcom_battmgr_battery;
	psy_cfg_supply.num_supplicants = 1;

	INIT_DELAYED_WORK(&battmgr->charge_work, qcom_battmgr_charge_worker);
	INIT_WORK(&battmgr->otg_work, qcom_battmgr_otg_worker);
	mutex_init(&battmgr->lock);
	spin_lock_init(&battmgr->notify_lock);
	mutex_init(&battmgr->charge_lock);
	init_completion(&battmgr->ack);
	battmgr->request_opcode = U32_MAX;

	match = of_match_device(qcom_battmgr_of_variants, dev->parent);
	if (match)
		battmgr->variant = (unsigned long)match->data;
	else
		battmgr->variant = QCOM_BATTMGR_SM8350;

	match = of_match_device(qcom_battmgr_of_quirks, dev->parent);
	if (match)
		battmgr->quirks = (unsigned long)match->data;

	ret = qcom_battmgr_charge_control_thresholds_init(battmgr);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "failed to init battery charge control thresholds\n");

	/*
	 * The client must outlive the power supplies and enable_work, which
	 * both send requests through it: devres releases it after them.
	 */
	battmgr->client = devm_pmic_glink_client_alloc(dev, PMIC_GLINK_OWNER_BATTMGR,
						       qcom_battmgr_callback,
						       qcom_battmgr_pdr_notify,
						       battmgr);
	if (IS_ERR(battmgr->client))
		return PTR_ERR(battmgr->client);

	/* Without a configured end threshold, charge to full */
	if ((battmgr->quirks & QCOM_BATTMGR_QUIRK_CHARGE_ENABLE) &&
	    !battmgr->info.charge_ctrl_end)
		battmgr->info.charge_ctrl_end = CHARGE_CTRL_END_THR_MAX;

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_OPLUS_PD) {
		set_bit(PD_EVENT_CAPS, &battmgr->pd_events);
		battmgr->psy_nb.notifier_call = qcom_battmgr_psy_notify;
		ret = power_supply_reg_notifier(&battmgr->psy_nb);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev, qcom_battmgr_unregister_psy_notifier,
					       battmgr);
		if (ret)
			return ret;
	}

	if (battmgr->quirks & QCOM_BATTMGR_QUIRK_PPS) {
		battmgr->pm_nb.notifier_call = qcom_battmgr_pm_notify;
		ret = register_pm_notifier(&battmgr->pm_nb);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(dev, qcom_battmgr_unregister_pm_notifier,
					       battmgr);
		if (ret)
			return ret;
	}

	if (battmgr->variant == QCOM_BATTMGR_SC8280XP ||
	    battmgr->variant == QCOM_BATTMGR_X1E80100) {
		if (battmgr->variant == QCOM_BATTMGR_X1E80100)
			psy_desc = &x1e80100_bat_psy_desc;
		else
			psy_desc = &sc8280xp_bat_psy_desc;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->ac_psy = devm_power_supply_register(dev, &sc8280xp_ac_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->ac_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->ac_psy),
					     "failed to register AC power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sc8280xp_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sc8280xp_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	} else {
		/*
		 * The SM8350-class firmware has no BATTMGR_BAT_INFO request, so
		 * @unit is never populated from the firmware. It reports charge
		 * domain values, so set it statically here - otherwise the
		 * CHARGE_* properties are rejected with -ENODATA.
		 */
		battmgr->unit = QCOM_BATTMGR_UNIT_mAh;

		if (battmgr->variant == QCOM_BATTMGR_SM8550)
			psy_desc = &sm8550_bat_psy_desc;
		else
			psy_desc = &sm8350_bat_psy_desc;

		psy_desc = qcom_battmgr_sm8350_bat_desc(battmgr, psy_desc);
		if (!psy_desc)
			return -ENOMEM;

		battmgr->bat_psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
		if (IS_ERR(battmgr->bat_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->bat_psy),
					     "failed to register battery power supply\n");

		battmgr->usb_psy = devm_power_supply_register(dev, &sm8350_usb_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->usb_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->usb_psy),
					     "failed to register USB power supply\n");

		battmgr->wls_psy = devm_power_supply_register(dev, &sm8350_wls_psy_desc, &psy_cfg_supply);
		if (IS_ERR(battmgr->wls_psy))
			return dev_err_probe(dev, PTR_ERR(battmgr->wls_psy),
					     "failed to register wireless charing power supply\n");
	}

	ret = devm_work_autocancel(dev, &battmgr->enable_work,
				   qcom_battmgr_enable_worker);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, qcom_battmgr_cancel_work, battmgr);
	if (ret)
		return ret;

	/* Released first: no notification may touch the supplies or the work. */
	ret = devm_add_action_or_reset(dev, qcom_battmgr_stop_notifications, battmgr);
	if (ret)
		return ret;

	pmic_glink_client_register(battmgr->client);

	return 0;
}

static void qcom_battmgr_shutdown(struct auxiliary_device *adev)
{
	struct qcom_battmgr *battmgr = auxiliary_get_drvdata(adev);

	qcom_battmgr_stop_notifications(battmgr);
	cancel_work_sync(&battmgr->enable_work);
	qcom_battmgr_cancel_work(battmgr);
}

static const struct auxiliary_device_id qcom_battmgr_id_table[] = {
	{ .name = "pmic_glink.power-supply", },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, qcom_battmgr_id_table);

static struct auxiliary_driver qcom_battmgr_driver = {
	.name = "pmic_glink_power_supply",
	.probe = qcom_battmgr_probe,
	.shutdown = qcom_battmgr_shutdown,
	.id_table = qcom_battmgr_id_table,
};

module_auxiliary_driver(qcom_battmgr_driver);

MODULE_DESCRIPTION("Qualcomm PMIC GLINK battery manager driver");
MODULE_LICENSE("GPL");
