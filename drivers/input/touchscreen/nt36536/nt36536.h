/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * $Revision: 69262 $
 * $Date: 2020-09-23 15:07:14 +0800 (週三, 23 九月 2020) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#ifndef 	_LINUX_NVT36536_H
#define		_LINUX_NVT36536_H

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>

#include <drm/drm_panel.h>

#include "nt36536_mem_map.h"

#define NVT_DEBUG 1

//---INT trigger mode, used when the DT irq specifier carries no type---
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_FALLING


//---SPI driver info.---
#define NVT_SPI_NAME "nt36536"

#if NVT_DEBUG
#define NVT_LOG(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#else
#define NVT_LOG(fmt, args...)    pr_info("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#endif
#define NVT_ERR(fmt, args...)    pr_err("[%s] %s %d: " fmt, NVT_SPI_NAME, __func__, __LINE__, ##args)
#define NVT_ERR_ONCE(fmt, args...) \
	pr_err_once("[%s] %s %d: " fmt, NVT_SPI_NAME, \
		    __func__, __LINE__, ##args)

//---Input device info.---
#define NVT_TS_NAME "NVTCapacitiveTouchScreen"
#define NVT_PEN_NAME "NVTCapacitivePen"

//---Touch info.---
#define TOUCH_DEFAULT_MAX_WIDTH 2400
#define TOUCH_DEFAULT_MAX_HEIGHT 3392
/*
 * The high-resolution report format carries finger coordinates in tenths of a
 * panel pixel, so the range it can produce is abs_x_max * NVT_COORD_SCALE by
 * abs_y_max * NVT_COORD_SCALE. The packed 12-bit format reports whole panel
 * pixels instead. The input device always reports whole panel pixels.
 */
#define NVT_COORD_SCALE 10
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
extern const uint16_t touch_key_array[TOUCH_KEY_NUM];
#endif
#define TOUCH_FORCE_NUM 1000
//---for Pen---
#define PEN_DISTANCE_MAX (1)
#define PEN_TILT_MIN (-60)
#define PEN_TILT_MAX (60)

/* This module has no host-wired tp reset pin; reset is shared with the panel. */

//---Customerized func.---
#define NVT_TOUCH_MP 0
#define MT_PROTOCOL_B 1

#define BOOT_UPDATE_FIRMWARE 1
#define DEFAULT_BOOT_UPDATE_FIRMWARE_NAME "novatek/nt36536-xn242.bin"

//---ESD Protect.---
#define NVT_TOUCH_ESD_PROTECT 1
#define NVT_TOUCH_ESD_CHECK_PERIOD 1500	/* ms */

/*
 * Report formats, selected by fw info byte 13 the same way the vendor module
 * picks its point parser: 0xF1 means 16-bit big-endian coordinates in tenths
 * of a pixel, anything else means packed 12-bit panel-pixel coordinates.
 */
enum nvt_report_format {
	NVT_REPORT_16BIT = 0,
	NVT_REPORT_12BIT,
};

/*
 * Pen report parameters that come with the controller firmware rather than
 * with the board geometry.
 */
struct nvt_ts_variant {
	/* pen report units per touchscreen-size unit (panel pixel) */
	unsigned int pen_coord_scale;
	unsigned int pen_pressure_max;
};

enum nvt_ic_state {
	NVT_IC_SUSPEND_IN,
	NVT_IC_SUSPEND_OUT,
	NVT_IC_RESUME_IN,
	NVT_IC_RESUME_OUT,
	NVT_IC_INIT,
};

struct nvt_ts_data {
	struct spi_device *client;
	struct input_dev *input_dev;
	struct touchscreen_properties prop;
	/* pen frame: prop scaled by variant->pen_coord_scale */
	struct touchscreen_properties pen_prop;
	const struct nvt_ts_variant *variant;
	struct delayed_work nvt_fwu_work;
	bool pen_is_charge;
	uint16_t addr;
	int8_t phys[32];
	struct drm_panel_follower panel_follower;
	/* Serializes panel ownership, resource readiness and wake commit. */
	spinlock_t lifecycle_lock;
	bool is_panel_follower;
	bool panel_on;
	bool panel_follower_registered;
	bool resources_ready;
	bool stopping;
	bool touch_awake;
	/* chip id and memory map read; a follower does it on first prepare */
	bool chip_identified;
	const char *fw_name;
	uint8_t fw_ver;
	uint8_t x_num;
	uint8_t y_num;
	int ic_state;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint8_t auto_copy;
	uint32_t int_trigger_type;
	struct mutex lock;
	const struct nvt_ts_mem_map *mmap;
	uint8_t hw_crc;
	uint8_t bld_multi_header;
	uint32_t crc_err_flag_addr;
	uint16_t nvt_pid;
	uint8_t *rbuf;
	uint8_t *xbuf;
	struct mutex xbuf_lock;
	bool irq_enabled;
	bool irq_requested;
	u32 eng_reset_count;
	u32 resume_count;
	u32 firmware_update_count;
	uint8_t report_format;
	bool pen_support;
	uint8_t x_gang_num;
	uint8_t y_gang_num;
	struct input_dev *pen_input_dev;
	bool pen_input_dev_enable;
	int8_t pen_phys[32];
	struct workqueue_struct *event_wq;
	struct work_struct resume_work;
	int db_wakeup;
	bool dev_pm_suspend;
	struct completion dev_pm_suspend_completion;
	int gesture_command_delayed;
};

typedef enum {
	RESET_STATE_INIT = 0xA0,// IC reset
	RESET_STATE_REK,		// ReK baseline
	RESET_STATE_REK_FINISH,	// baseline is ready
	RESET_STATE_NORMAL_RUN,	// normal run
	RESET_STATE_MAX  = 0xAF
} RST_COMPLETE_STATE;

typedef enum {
	EVENT_MAP_HOST_CMD                      = 0x50,
	EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE   = 0x51,
	EVENT_MAP_RESET_COMPLETE                = 0x60,
	EVENT_MAP_FWINFO                        = 0x78,
	EVENT_MAP_PROJECTID                     = 0x9A,
} SPI_EVENT_MAP;

//---SPI READ/WRITE---
#define SPI_WRITE_MASK(a)	(a | 0x80)
#define SPI_READ_MASK(a)	(a & 0x7F)

#define DUMMY_BYTES (1)
/* Vendor module writes firmware to SRAM in 1 KiB chunks. */
#define NVT_TRANSFER_LEN	(1*1024)
#define NVT_READ_LEN		(2*1024)
/* Vendor module always reads the full 121-byte report frame. */
#define NVT_REPORT_LEN		121

typedef enum {
	NVTWRITE = 0,
	NVTREAD  = 1
} NVT_SPI_RW;

//---extern structures---
extern struct nvt_ts_data *ts;
extern uint32_t SWRST_N8_ADDR;

//---extern functions---
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len);
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len);
void nvt_irq_enable(bool enable);
void nvt_bootloader_reset(void);
void nvt_eng_reset(void);
void nvt_sw_reset(void);
void nvt_sw_reset_idle(void);
void nvt_boot_ready(void);
void nvt_fw_crc_enable(void);
void nvt_tx_auto_copy_mode(void);
int32_t nvt_update_firmware(const char *firmware_name);
int32_t nvt_update_firmware_no_eng_reset(const char *firmware_name);
void Boot_Update_Firmware(struct work_struct *work);
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state);
int32_t nvt_get_fw_info(void);
int32_t nvt_clear_fw_status(void);
int32_t nvt_check_fw_status(void);
int32_t nvt_check_spi_dma_tx_info(void);
int32_t nvt_check_tx_auto_copy(void);
int32_t nvt_read_pid(void);
int32_t nvt_set_page(uint32_t addr);
int32_t nvt_write_addr(uint32_t addr, uint8_t data);
int32_t nvt_wait_auto_copy(void);
int32_t disable_pen_input_device(bool disable);
#if NVT_TOUCH_ESD_PROTECT
extern void nvt_esd_check_enable(uint8_t enable);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#endif /* _LINUX_NVT36536_H */
