/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * $Revision: 73033 $
 * $Date: 2020-11-26 10:09:14 +0800 (週四, 26 十一月 2020) $
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input/mt.h>
#include <linux/of.h>

#include "nt36536.h"

#if NVT_TOUCH_ESD_PROTECT
#include <linux/jiffies.h>
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#if NVT_TOUCH_ESD_PROTECT
static struct delayed_work nvt_esd_check_work;
static struct workqueue_struct *nvt_esd_check_wq;
static unsigned long irq_timer = 0;
uint8_t esd_check = false;
uint8_t esd_retry = 0;
#endif /* #if NVT_TOUCH_ESD_PROTECT */

struct nvt_ts_data *ts;
static struct drm_panel_follower_funcs nt36536_panel_follower_funcs;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
#endif

static int32_t nvt_ts_suspend(struct device *dev);
static int32_t nvt_ts_resume(struct device *dev);
static void nvt_set_touch_awake_state(struct nvt_ts_data *ts_core,
				      bool awake);

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR = 0; //read from dtsi

#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM] = {
	KEY_BACK,
	KEY_HOME,
	KEY_MENU
};
#endif

/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
void nvt_irq_enable(bool enable)
{
	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}
}

static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len , NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memset(ts->xbuf, 0, len + DUMMY_BYTES);
	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 0---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTREAD);
		if (ret == 0) break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 0---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;
	int32_t retries = 0;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_WRITE_MASK(buf[0]);

	while (retries < 5) {
		ret = spi_read_write(client, buf, len, NVTWRITE);
		if (ret == 0)	break;
		retries++;
	}

	if (unlikely(retries == 5)) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	//---set xdata index---
	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	//---write data to index---
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

	The frame lengths below are byte-for-byte what the NT36536 vendor
	module issues; trimming or padding them leaves the firmware stuck
	in RESET_STATE_INIT with the interrupt line silent.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	u8 clear_cmd[2] = {EVENT_MAP_RESET_COMPLETE, 0x00};
	CTP_SPI_WRITE(ts->client, clear_cmd, sizeof(clear_cmd));

	//---enable fw crc---
	u8 crc_cmd[2] = {EVENT_MAP_HOST_CMD, 0xAE};
	CTP_SPI_WRITE(ts->client, crc_cmd, sizeof(crc_cmd));
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen enable auto copy mode function.

return:
	N/A.
*******************************************************/
void nvt_tx_auto_copy_mode(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		//---write TX_AUTO_COPY_EN cmds---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x69);
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		//---write SPI_MST_AUTO_COPY cmds---
		nvt_write_addr(ts->mmap->TX_AUTO_COPY_EN, 0x56);
	}

	NVT_LOG("tx auto copy mode %d enable\n", ts->auto_copy);
}
/*******************************************************
Description:
	Novatek touchscreen check spi dma tx info function.

return:
	N/A.
*******************************************************/
int32_t nvt_check_spi_dma_tx_info(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->SPI_DMA_TX_INFO);

		//---read fw status---
		buf[0] = ts->mmap->SPI_DMA_TX_INFO & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	if (ts)
		ts->eng_reset_count++;
	NVT_LOG("engineering reset #%u\n", ts ? ts->eng_reset_count : 0);

	//---eng reset cmds to ENG_RST_ADDR---
	nvt_write_addr(ENG_RST_ADDR, 0x5A);

	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	/* vendor module waits 20ms before the first poll */
	msleep(20);

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	/* vendor module polls 11 times for RESET_STATE_INIT, 101 otherwise */
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 100;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		/* vendor module sleeps 10ms before every poll */
		usleep_range(10000, 10000);

		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);
		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[4] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen declare the coordinate axes of the
	touch and pen input devices, in panel pixels.

	Both devices are registered from probe(), which runs
	before the firmware has been downloaded and therefore
	only has the DT coordinates to declare them from.

return:
	n.a.
*******************************************************/
static void nvt_ts_set_touch_abs_params(void)
{
	if (!ts->input_dev)
		return;

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max - 1, 0, 0);
}

static void nvt_ts_set_pen_abs_params(void)
{
	int32_t x_max = ts->abs_x_max - 1;
	int32_t y_max = ts->abs_y_max - 1;

	if (!ts->pen_input_dev)
		return;

	input_set_abs_params(ts->pen_input_dev, ABS_X, 0, x_max, 0, 0);
	input_set_abs_params(ts->pen_input_dev, ABS_Y, 0, y_max, 0, 0);
	input_abs_set_res(ts->pen_input_dev, ABS_X, x_max / PANEL_DEFAULT_WIDTH_MM);
	input_abs_set_res(ts->pen_input_dev, ABS_Y, y_max / PANEL_DEFAULT_HEIGHT_MM);
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

	The touch resolution comes from the DT, not from this frame: the
	vendor firmware answers in different units per report format, and
	the vendor module likewise only consumes fw_ver and the report
	format selector (byte 13) here.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	//---read fw info---
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 39);
	ts->fw_ver = buf[1];
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	if (ts->pen_support) {
		ts->x_gang_num = buf[37];
		ts->y_gang_num = buf[38];
	}

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->fw_ver = 0;
		ts->x_num = 0;
		ts->y_num = 0;

		if(retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d!\n",
				ts->fw_ver, ts->x_num, ts->y_num);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	/*
	 * Byte 13 selects the report format, exactly like the vendor module
	 * selects its point parser: 0xF1 means 16-bit coordinates in tenths
	 * of a panel pixel, anything else means packed 12-bit panel pixels.
	 */
	if (ret == 0)
		ts->report_format = (buf[13] == 0xF1) ?
			NVT_REPORT_16BIT : NVT_REPORT_12BIT;

	NVT_LOG("fw_ver = 0x%02X, fw_type = 0x%02X, x_num=%d, y_num=%d, report_format=%d\n",
			ts->fw_ver, buf[14], ts->x_num, ts->y_num,
			ts->report_format);

	//---Get Novatek PID---
	nvt_read_pid();

	return ret;
}

static void release_pen_event(void) {
	if (ts && ts->pen_input_dev) {
		input_report_abs(ts->pen_input_dev, ABS_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_PRESSURE, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_X, 0);
		input_report_abs(ts->pen_input_dev, ABS_TILT_Y, 0);
		input_report_abs(ts->pen_input_dev, ABS_DISTANCE, 0);
		input_report_key(ts->pen_input_dev, BTN_TOUCH, 0);
		input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS, 0);
		input_report_key(ts->pen_input_dev, BTN_STYLUS2, 0);
		input_sync(ts->pen_input_dev);
	}
}

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct of_phandle_args args;
	uint32_t coords[2] = {0};
	int32_t ret = 0;

	/*
	 * The interrupt itself comes from the node's standard interrupts
	 * property (the SPI core maps it into client->irq).  The vendor-style
	 * novatek,irq-gpio specifier is honored as the trigger-type fallback.
	 */
	ts->int_trigger_type = 0;
	ret = of_parse_phandle_with_args(np, "novatek,irq-gpio", "#gpio-cells",
					 0, &args);
	if (!ret) {
		ts->int_trigger_type = args.args[1] & IRQD_TRIGGER_MASK;
		of_node_put(args.np);
	}
	if (!ts->int_trigger_type) {
		NVT_LOG("no trigger type in DT, use default 0x%08X\n",
			INT_TRIGGER_TYPE);
		ts->int_trigger_type = INT_TRIGGER_TYPE;
	}

	ts->pen_support = of_property_read_bool(np, "novatek,pen-support");
	NVT_LOG("novatek,pen-support=%d\n", ts->pen_support);

	ts->wgp_stylus = of_property_read_bool(np, "novatek,wgp-stylus");
	NVT_LOG("novatek,wgp-stylus=%d\n", ts->wgp_stylus);

	/*
	 * The pen half of the report is sampled on a finer grid than the panel,
	 * and the vendor trees name two different factors for it in the two
	 * boards that carry this controller family: one sets novatek,wgp-stylus,
	 * which this driver has always treated as ten firmware units per pixel,
	 * and another sets novatek,stylus-resol-double, which is two. Neither
	 * changes the finger scale. With no property the pen reports panel
	 * pixels like the touch half does.
	 */
	if (ts->wgp_stylus)
		ts->pen_coord_scale = 10;
	else if (of_property_read_bool(np, "novatek,stylus-resol-double"))
		ts->pen_coord_scale = 2;
	else
		ts->pen_coord_scale = 1;
	NVT_LOG("pen_coord_scale=%d\n", ts->pen_coord_scale);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	ret = of_property_read_string(np, "firmware-name", &ts->fw_name);
	if (ret) {
		NVT_LOG("Unable to get touchscreen firmware name\n");
		ts->fw_name = DEFAULT_BOOT_UPDATE_FIRMWARE_NAME;
	}

	ret = of_property_read_u32_array(np, "novatek,display-coords", coords, 2);
	if (ret) {
		NVT_LOG("novatek,display-coords not set, use defaults %dx%d\n",
			TOUCH_DEFAULT_MAX_WIDTH, TOUCH_DEFAULT_MAX_HEIGHT);
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	} else {
		ts->abs_x_max = coords[0];
		ts->abs_y_max = coords[1];
	}
	NVT_LOG("display coords: %dx%d\n", ts->abs_x_max, ts->abs_y_max);

	ret = of_property_read_u32(np, "spi-max-frequency", &ts->spi_max_freq);
	if (ret) {
		NVT_LOG("Unable to get spi freq\n");
		return ret;
	} else {
		NVT_LOG("spi-max-frequency: %u\n", ts->spi_max_freq);
	}

	return 0;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
	ts->pen_coord_scale = 1;
	ts->int_trigger_type = INT_TRIGGER_TYPE;
	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	return 0;
}
#endif

static uint8_t nvt_fw_recovery(uint8_t *point_data)
{
	uint8_t i = 0;
	uint8_t detected = true;

	/* check pattern */
	for (i=1 ; i<7 ; i++) {
		if (point_data[i] != 0x77) {
			detected = false;
			break;
		}
	}

	return detected;
}

void nvt_set_dbgfw_status(bool enable)
{
	ts->fw_debug = enable;
}

bool nvt_get_dbgfw_status(void)
{
	return ts->fw_debug;
}

#if NVT_TOUCH_ESD_PROTECT
void nvt_esd_check_enable(uint8_t enable)
{
	/* update interrupt timer */
	irq_timer = jiffies;
	/* clear esd_retry counter, if protect function is enabled */
	esd_retry = enable ? 0 : esd_retry;
	/* enable/disable esd check flag */
	esd_check = enable;
}

static void nvt_esd_check_func(struct work_struct *work)
{
	unsigned int timer = jiffies_to_msecs(jiffies - irq_timer);

	//NVT_LOG("esd_check = %d (retry %d)\n", esd_check, esd_retry);	//DEBUG

	if ((timer > NVT_TOUCH_ESD_CHECK_PERIOD) && esd_check) {
		/*
		 * A follower may only recover after the panel has re-established
		 * ownership of the shared TDDI.  Stop input and wait for the next
		 * unprepare/prepare cycle instead of downloading behind the panel.
		 */
		if (ts->is_panel_follower) {
			nvt_irq_enable(false);
			nvt_set_touch_awake_state(ts, false);
			nvt_esd_check_enable(false);
			NVT_ERR("ESD signature: follower stopped until panel cycle\n");
			return;
		}
		mutex_lock(&ts->lock);
		NVT_ERR("do ESD recovery, timer = %d, retry = %d\n", timer, esd_retry);
		/* do esd recovery, reload fw */
		nvt_update_firmware(ts->fw_name);
		mutex_unlock(&ts->lock);
		/* update interrupt timer */
		irq_timer = jiffies;
		/* update esd_retry counter */
		esd_retry++;
	}

	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

#define PEN_DATA_LEN 14
#define FW_HISTORY_SIZE 128

/*
 * Touch report: [1..64] point data, [65] checksum, [66..79] pen block,
 * followed by vendor extension bytes up to [120].  The vendor module always
 * reads the full 121-byte frame on every interrupt.
 */
#define POINT_DATA_LEN 65
#define NVT_REPORT_LEN 121

static uint8_t nvt_report_checksum(const uint8_t *data, size_t len)
{
	uint8_t checksum = 0;
	size_t i;

	for (i = 0; i < len; i++)
		checksum += data[i];

	return (uint8_t)(-checksum);
}

/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret = -1;
	uint8_t point_data[NVT_REPORT_LEN + DUMMY_BYTES] = {0};
	uint32_t position = 0;
	uint32_t input_x = 0;
	uint32_t input_y = 0;
	uint32_t input_w = 0;
	uint32_t input_p = 0;
	uint8_t input_id = 0;
#if MT_PROTOCOL_B
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
#endif /* MT_PROTOCOL_B */
	int32_t i = 0;
	int32_t finger_cnt = 0;
	uint8_t pen_format_id = 0;
	uint32_t pen_x = 0;
	uint32_t pen_y = 0;
	uint32_t pen_pressure = 0;
	uint32_t pen_distance = 0;
	int8_t pen_tilt_x = 0;
	int8_t pen_tilt_y = 0;
	uint32_t pen_btn1 = 0;
	uint32_t pen_btn2 = 0;
	uint32_t pen_battery = 0;

	mutex_lock(&ts->lock);

	if (ts->dev_pm_suspend) {
		ret = wait_for_completion_timeout(&ts->dev_pm_suspend_completion, msecs_to_jiffies(500));
		if (!ret) {
			NVT_ERR("system(spi) can't finished resuming procedure, skip it\n");
			goto XFER_ERROR;
		}
	}

	ret = CTP_SPI_READ(ts->client, point_data, NVT_REPORT_LEN);
	if (ret < 0) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto XFER_ERROR;
	}

	/* ESD protect by FW handshake */
	if (nvt_fw_recovery(point_data)) {
#if NVT_TOUCH_ESD_PROTECT
		nvt_esd_check_enable(true);
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		goto XFER_ERROR;
	}

	/*
	 * The touch half of the report is protected by a checksum byte the
	 * same way the vendor module validates it: point_data[65] must equal
	 * the two's complement of the sum of point_data[1..64].
	 */
	if (point_data[POINT_DATA_LEN] !=
	    nvt_report_checksum(&point_data[1], POINT_DATA_LEN - 1)) {
		NVT_ERR("touch report checksum failed: got 0x%02X, want 0x%02X\n",
			point_data[POINT_DATA_LEN],
			nvt_report_checksum(&point_data[1], POINT_DATA_LEN - 1));
		goto XFER_ERROR;
	}

	finger_cnt = 0;

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position + 0] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02)) {	//finger down (enter & moving)
#if NVT_TOUCH_ESD_PROTECT
			/* update interrupt timer */
			irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
			if (ts->report_format == NVT_REPORT_16BIT) {
				/* 16-bit big-endian coordinates in tenths of a panel pixel */
				input_x = (uint32_t)(point_data[position + 1] << 8) + (uint32_t) (point_data[position + 2]);
				input_y = (uint32_t)(point_data[position + 3] << 8) + (uint32_t) (point_data[position + 4]);
				if ((input_x > ts->abs_x_max * NVT_COORD_SCALE - 1) ||
						(input_y > ts->abs_y_max * NVT_COORD_SCALE - 1))
					continue;
				input_x /= NVT_COORD_SCALE;
				input_y /= NVT_COORD_SCALE;
				input_w = (uint32_t)(point_data[position + 5]);
				input_p = 1;
			} else {
				/* packed 12-bit panel-pixel coordinates */
				input_x = (uint32_t)(point_data[position + 1] << 4) +
						(uint32_t)(point_data[position + 3] >> 4);
				input_y = (uint32_t)(point_data[position + 2] << 4) +
						(uint32_t)(point_data[position + 3] & 0x0F);
				if ((input_x > ts->abs_x_max - 1) || (input_y > ts->abs_y_max - 1))
					continue;
				input_w = (uint32_t)(point_data[position + 4]);
				/*
				 * Pressure low byte per contact; the first two
				 * contacts carry the high byte at [63] and [64],
				 * clamped to the firmware's 1000 maximum.
				 */
				input_p = (uint32_t)(point_data[position + 5]);
				if (i < 2)
					input_p |= (uint32_t)(point_data[63 + i] << 8);
				if (input_p > TOUCH_FORCE_NUM)
					input_p = TOUCH_FORCE_NUM;
			}
			if (input_w == 0)
				input_w = 1;
			if (input_p == 0)
				input_p = 1;

#if MT_PROTOCOL_B
			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#else /* MT_PROTOCOL_B */
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id - 1);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
#endif /* MT_PROTOCOL_B */

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, input_p);

#if MT_PROTOCOL_B
#else /* MT_PROTOCOL_B */
			input_mt_sync(ts->input_dev);
#endif /* MT_PROTOCOL_B */

			finger_cnt++;
		}
	}

#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		if (press_id[i] != 1) {
			input_mt_slot(ts->input_dev, i);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}
	input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt > 0));
#else /* MT_PROTOCOL_B */
	if (finger_cnt == 0) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
#endif /* MT_PROTOCOL_B */

#if TOUCH_KEY_NUM > 0
	if (point_data[61] == 0xF8) {
#if NVT_TOUCH_ESD_PROTECT
		/* update interrupt timer */
		irq_timer = jiffies;
#endif /* #if NVT_TOUCH_ESD_PROTECT */
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], ((point_data[62] >> i) & 0x01));
		}
	} else {
		for (i = 0; i < ts->max_button_num; i++) {
			input_report_key(ts->input_dev, touch_key_array[i], 0);
		}
	}
#endif

	input_sync(ts->input_dev);

	if (ts->pen_support && ts->pen_input_dev_enable && !(ts->pen_is_charge)) {
		// parse and handle pen report
		pen_format_id = point_data[66];
		if (pen_format_id != 0xFF) {
			if (pen_format_id == 0x01) {
				// report pen data
				pen_x = (uint32_t)(point_data[67] << 8) + (uint32_t)(point_data[68]);
				pen_y = (uint32_t)(point_data[69] << 8) + (uint32_t)(point_data[70]);
				pen_x /= ts->pen_coord_scale;
				pen_y /= ts->pen_coord_scale;
				if (pen_x > ts->abs_x_max - 1) {
					pen_x = ts->abs_x_max - 1;
				}
				if (pen_y > ts->abs_y_max - 1) {
					pen_y = ts->abs_y_max - 1;
				}
				pen_pressure = (uint32_t)(point_data[71] << 8) + (uint32_t)(point_data[72]);
				pen_tilt_x = (int32_t)point_data[73];
				pen_tilt_y = (int32_t)point_data[74];
				pen_distance = (uint32_t)(point_data[75] << 8) + (uint32_t)(point_data[76]);
				pen_btn1 = (uint32_t)(point_data[77] & 0x01);
				pen_btn2 = (uint32_t)((point_data[77] >> 1) & 0x01);
				pen_battery = (uint32_t)point_data[78];

				input_report_abs(ts->pen_input_dev, ABS_X, pen_x);
				input_report_abs(ts->pen_input_dev, ABS_Y, pen_y);
				input_report_abs(ts->pen_input_dev, ABS_PRESSURE, pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_TOUCH, !!pen_pressure);
				input_report_abs(ts->pen_input_dev, ABS_TILT_X, pen_tilt_x);
				input_report_abs(ts->pen_input_dev, ABS_TILT_Y, pen_tilt_y);
				input_report_abs(ts->pen_input_dev, ABS_DISTANCE, pen_distance);
				input_report_key(ts->pen_input_dev, BTN_TOOL_PEN, !!pen_distance || !!pen_pressure);
				input_report_key(ts->pen_input_dev, BTN_STYLUS, pen_btn1);
				input_report_key(ts->pen_input_dev, BTN_STYLUS2, pen_btn2);
				input_sync(ts->pen_input_dev);
				// TBD: pen battery event report
				// NVT_LOG("pen_battery=%d\n", pen_battery);
			} else if (pen_format_id == 0xF0) {
				// report Pen ID
			} else {
				NVT_ERR_ONCE("Unknown pen format id 0x%02X!\n",
					     pen_format_id);
				goto XFER_ERROR;
			}
		} else { // pen_format_id = 0xFF, i.e. no pen present
			release_pen_event();
		}
	} /* if (ts->pen_support) */

XFER_ERROR:

	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

	The transaction sequence matches the vendor module attempt for
	attempt: bootloader reset, set page to the trim address, a 7-byte
	dummy write, then a 7-byte read carrying the six trim id bytes.

	An unmatched id is not fatal here: the id bytes are logged and the
	NT36536 maps are used anyway, so a new trim revision cannot block
	bring-up.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	uint8_t i = 0;
	int32_t list = 0;
	int32_t retry = 0;
	const int32_t retry_max = 6;
	const struct nvt_ts_trim_id_table *entry = NULL;
	uint32_t chip_ver_trim_addr = trim_id_table[0].hwinfo->hw_regs->chip_ver_trim_addr;

	for (retry = 0; retry < retry_max; retry++) {
		//---reset MCU into bootloader---
		nvt_bootloader_reset();

		//---set xdata index to chip ver trim address---
		nvt_set_page(chip_ver_trim_addr);

		//---dummy write before trim read, same as the vendor module---
		buf[0] = chip_ver_trim_addr & 0x7F;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 7);

		//---read chip id---
		buf[0] = chip_ver_trim_addr & 0x7F;
		CTP_SPI_READ(ts->client, buf, 7);

		NVT_LOG("chip id:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		//---compare read chip id with trim_id_table---
		for (list = 0; list < ARRAY_SIZE(trim_id_table); list++) {
			entry = &trim_id_table[list];
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (entry->mask[i] && (buf[i + 1] != entry->id[i]))
					break;
			}
			if (i == NVT_ID_BYTE_MAX)
				goto matched;
		}
		entry = NULL;

		msleep(10);
	}

matched:
	if (!entry) {
		NVT_ERR("chip id not in trim table, continue with NT36536 maps\n");
		entry = &trim_id_table[0];
	}

	ts->hw_crc = entry->hwinfo->hw_crc;
	ts->auto_copy = entry->hwinfo->auto_copy;
	ts->crc_err_flag_addr = entry->hwinfo->hw_regs->crc_err_flag_addr;

	//---detect single or cascade via ENB_CASC_REG, like the vendor module---
	{
		const struct nvt_ts_mem_map *mmap_single = entry->mmap;
		uint8_t bit = 0;
		uint8_t mask = mmap_single->ENB_CASC_REG.mask;
		int32_t ret;

		while (bit < 8 && !(mask & (1 << bit)))
			bit++;

		ret = nvt_set_page(mmap_single->ENB_CASC_REG.addr);
		if (ret == 0) {
			buf[0] = mmap_single->ENB_CASC_REG.addr & 0x7F;
			buf[1] = 0xFF;
			ret = CTP_SPI_READ(ts->client, buf, 2);
		}
		if (ret == 0 && (buf[1] & mask)) {
			NVT_LOG("Single Chip\n");
			ts->mmap = mmap_single;
		} else {
			NVT_LOG("Cascade Chip\n");
			ts->mmap = entry->mmap_casc;
		}
	}

	NVT_LOG("hw_crc is %d, auto_copy is %d\n", ts->hw_crc, ts->auto_copy);

	return 0;
}

int32_t nvt_check_tx_auto_copy(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 200;

	if (ts->mmap->TX_AUTO_COPY_EN == 0) {
		NVT_ERR("error, TX_AUTO_COPY_EN = 0\n");
		return -1;
	}

	for (i = 0; i < retry; i++) {
		//---set xdata index to SPI_MST_AUTO_COPY---
		nvt_set_page(ts->mmap->TX_AUTO_COPY_EN);

		//---read auto copy status---
		buf[0] = ts->mmap->TX_AUTO_COPY_EN & 0x7F;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(1000, 1000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

int32_t nvt_wait_auto_copy(void)
{
	if (ts->auto_copy == CHECK_SPI_DMA_TX_INFO) {
		return nvt_check_spi_dma_tx_info();
	} else if (ts->auto_copy == CHECK_TX_AUTO_COPY_EN) {
		return nvt_check_tx_auto_copy();
	} else {
		NVT_ERR("failed, not support mode %d!\n", ts->auto_copy);
		return -1;
	}
}


static int32_t __nvt_set_pen_state(bool disable)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;

	NVT_LOG("++\n");
	msleep(35);
	disable = (!(ts->pen_input_dev_enable) || ts->pen_is_charge) ? true : disable;
	//---set xdata index to EVENT BUF ADDR---
	ret = nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	if (ret < 0) {
		NVT_ERR("Set event buffer index fail!\n");
		goto nvt_set_pen_enable_out;
	}
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x7B;
	buf[2] = !disable;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret < 0) {
		NVT_ERR("set pen %s failed!\n", disable ? "DISABLE" : "ENABLE");
		goto nvt_set_pen_enable_out;
	}
	NVT_LOG("pen charge state is %s, %s pen input device\n",
	ts->pen_is_charge ? "ENABLE" : "DISABLE",
	disable ? "DISABLE" : "ENABLE");
nvt_set_pen_enable_out:
	NVT_LOG("--\n");
	return ret;
}

int32_t disable_pen_input_device(bool disable)
{
	if (!ts || !READ_ONCE(ts->touch_awake)) {
		NVT_LOG("touch suspend, stop set pen state %s\n",
			disable ? "DISABLE" : "ENABLE");
		return 0;
	}

	return __nvt_set_pen_state(disable);
}

static void nvt_resume_work(struct work_struct *work)
{
	struct nvt_ts_data *ts_core = container_of(work, struct nvt_ts_data, resume_work);
	nvt_ts_resume(&ts_core->client->dev);
}

static bool nvt_diagnostic_access_allowed(struct nvt_ts_data *ts_core)
{
	if (!ts_core->is_panel_follower)
		return true;

	return READ_ONCE(ts_core->panel_on) &&
	       READ_ONCE(ts_core->touch_awake);
}

static ssize_t point_data_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct nvt_ts_data *ts_core = dev_get_drvdata(dev);
	u8 point_data[NVT_REPORT_LEN] = { 0 };
	const size_t len = sizeof(point_data);
	ssize_t count;
	int i, ret;

	if (!ts_core)
		return -ENODEV;
	if (!nvt_diagnostic_access_allowed(ts_core))
		return -EHOSTDOWN;

	mutex_lock(&ts_core->lock);
	if (!nvt_diagnostic_access_allowed(ts_core)) {
		mutex_unlock(&ts_core->lock);
		return -EHOSTDOWN;
	}
	ret = CTP_SPI_READ(ts_core->client, point_data, len);
	mutex_unlock(&ts_core->lock);

	count = sysfs_emit(buf, "ret=%d len=%zu data=", ret, len);
	for (i = 0; i < len; i++)
		count += sysfs_emit_at(buf, count, "%02x", point_data[i]);
	count += sysfs_emit_at(buf, count, "\n");

	return count;
}
static DEVICE_ATTR_RO(point_data);

static int nvt_read_event_bytes(struct nvt_ts_data *ts_core, u8 offset,
				u8 *data, size_t len)
{
	int ret;

	memset(data, 0, len);
	ret = nvt_set_page(ts_core->mmap->EVENT_BUF_ADDR | offset);
	if (ret)
		return ret;

	data[0] = offset;
	return CTP_SPI_READ(ts_core->client, data, len);
}

static ssize_t touch_state_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct nvt_ts_data *ts_core = dev_get_drvdata(dev);
	u8 host_cmd[2] = { 0 };
	u8 handshake[2] = { 0 };
	u8 reset_state[6] = { 0 };
	u8 fw_info[39] = { 0 };
	ssize_t count;
	int host_ret, handshake_ret, reset_ret, fw_ret, restore_ret;
	int i;

	if (!ts_core)
		return -ENODEV;
	if (!nvt_diagnostic_access_allowed(ts_core))
		return -EHOSTDOWN;

	mutex_lock(&ts_core->lock);
	if (!nvt_diagnostic_access_allowed(ts_core)) {
		mutex_unlock(&ts_core->lock);
		return -EHOSTDOWN;
	}
	host_ret = nvt_read_event_bytes(ts_core, EVENT_MAP_HOST_CMD,
					host_cmd, sizeof(host_cmd));
	handshake_ret = nvt_read_event_bytes(ts_core,
					     EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE,
					     handshake, sizeof(handshake));
	reset_ret = nvt_read_event_bytes(ts_core, EVENT_MAP_RESET_COMPLETE,
					 reset_state, sizeof(reset_state));
	fw_ret = nvt_read_event_bytes(ts_core, EVENT_MAP_FWINFO,
				      fw_info, sizeof(fw_info));
	restore_ret = nvt_set_page(ts_core->mmap->EVENT_BUF_ADDR);
	mutex_unlock(&ts_core->lock);

	count = sysfs_emit(buf,
			   "host_ret=%d host=%02x handshake_ret=%d handshake=%02x ",
			   host_ret, host_cmd[1], handshake_ret, handshake[1]);
	count += sysfs_emit_at(buf, count, "reset_ret=%d reset=", reset_ret);
	for (i = 1; i < sizeof(reset_state); i++)
		count += sysfs_emit_at(buf, count, "%02x", reset_state[i]);
	count += sysfs_emit_at(buf, count, " fw_ret=%d fwinfo=", fw_ret);
	for (i = 1; i < sizeof(fw_info); i++)
		count += sysfs_emit_at(buf, count, "%02x", fw_info[i]);
	count += sysfs_emit_at(buf, count,
			       " report_format=%u restore_ret=%d",
			       ts_core->report_format, restore_ret);
	count += sysfs_emit_at(buf, count,
			       " eng_reset_count=%u resume_count=%u firmware_update_count=%u\n",
			       ts_core->eng_reset_count, ts_core->resume_count,
			       ts_core->firmware_update_count);

	return count;
}
static DEVICE_ATTR_RO(touch_state);

static bool nvt_panel_resume_allowed(struct nvt_ts_data *ts_core)
{
	unsigned long flags;
	bool allowed;

	spin_lock_irqsave(&ts_core->lifecycle_lock, flags);
	allowed = ts_core->is_panel_follower && ts_core->resources_ready &&
		  ts_core->panel_on && !ts_core->stopping &&
		  !ts_core->touch_awake;
	spin_unlock_irqrestore(&ts_core->lifecycle_lock, flags);

	return allowed;
}

static bool nvt_panel_is_on(struct nvt_ts_data *ts_core)
{
	unsigned long flags;
	bool panel_on;

	spin_lock_irqsave(&ts_core->lifecycle_lock, flags);
	panel_on = ts_core->panel_on && !ts_core->stopping;
	spin_unlock_irqrestore(&ts_core->lifecycle_lock, flags);

	return panel_on;
}

static void nvt_set_touch_awake_state(struct nvt_ts_data *ts_core, bool awake)
{
	unsigned long flags;

	spin_lock_irqsave(&ts_core->lifecycle_lock, flags);
	ts_core->touch_awake = awake;
	spin_unlock_irqrestore(&ts_core->lifecycle_lock, flags);
}

static bool nvt_commit_follower_awake(struct nvt_ts_data *ts_core)
{
	unsigned long flags;
	bool committed = false;

	spin_lock_irqsave(&ts_core->lifecycle_lock, flags);
	if (ts_core->resources_ready && ts_core->panel_on &&
	    !ts_core->stopping) {
		ts_core->touch_awake = true;
		committed = true;
	}
	spin_unlock_irqrestore(&ts_core->lifecycle_lock, flags);

	return committed;
}

static void nvt_unregister_panel_follower(struct nvt_ts_data *ts_core)
{
	unsigned long flags;

	spin_lock_irqsave(&ts_core->lifecycle_lock, flags);
	ts_core->stopping = true;
	ts_core->resources_ready = false;
	spin_unlock_irqrestore(&ts_core->lifecycle_lock, flags);

	if (!ts_core->panel_follower_registered)
		return;

	drm_panel_remove_follower(&ts_core->panel_follower);
	ts_core->panel_follower_registered = false;
}

static void nvt_destroy_workqueues(struct nvt_ts_data *ts_core)
{
	/* Suspend/resume work can touch both firmware and ESD workqueues. */
	if (ts_core->event_wq) {
		cancel_work_sync(&ts_core->resume_work);
		destroy_workqueue(ts_core->event_wq);
		ts_core->event_wq = NULL;
	}

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts_core->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
		destroy_workqueue(nvt_esd_check_wq);
		nvt_esd_check_wq = NULL;
	}
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct spi_device *client)
{
	int32_t ret = 0;
#if (TOUCH_KEY_NUM > 0)
	int32_t retry = 0;
#endif

	NVT_LOG("probe start\n");

	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}

	ts->xbuf = (uint8_t *)kzalloc((NVT_TRANSFER_LEN+1+DUMMY_BYTES), GFP_KERNEL);
	if(ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_xbuf;
	}

	ts->rbuf = (uint8_t *)kzalloc(NVT_READ_LEN, GFP_KERNEL);
	if(ts->rbuf == NULL) {
		NVT_ERR("kzalloc for rbuf failed!\n");
		ret = -ENOMEM;
		goto err_malloc_rbuf;
	}

	ts->client = client;
	spi_set_drvdata(client, ts);

	//---prepare for spi parameter---
	if (ts->client->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	//---parse dts---
	ret = nvt_parse_dt(&client->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);
	spin_lock_init(&ts->lifecycle_lock);
	ts->is_panel_follower = drm_is_panel_follower(&client->dev);
	WRITE_ONCE(ts->touch_awake, false);

	/*
	 * A panel follower shares the TDDI die with the display.  It must not
	 * issue an engineering reset before the panel owns and prepares that die.
	 * The non-follower path is kept byte-for-byte equivalent to the known-good
	 * cold boot sequence.  There is no host-owned reset line on this module:
	 * the touch die is reset together with the panel.
	 */
	if (!ts->is_panel_follower) {
		WRITE_ONCE(ts->panel_on, true);
		nvt_eng_reset();
		/* need 10ms delay after POR (power-on reset) */
		usleep_range(10000, 11000);
	}

	//---check chip version trim---
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}

	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

#if TOUCH_KEY_NUM > 0
	ts->max_button_num = TOUCH_KEY_NUM;
#endif

	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	ts->db_wakeup = 0;
	ts->fw_ver = 0;
	ts->x_num = 0;
	ts->y_num = 0;
	ts->x_gang_num = 0;
	ts->y_gang_num = 0;
	ts->max_button_num = TOUCH_KEY_NUM;
	ts->report_format = NVT_REPORT_16BIT;

#if MT_PROTOCOL_B
	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, TOUCH_FORCE_NUM, 0, 0);    //pressure = TOUCH_FORCE_NUM

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	nvt_ts_set_touch_abs_params();
#if MT_PROTOCOL_B
	// no need to set ABS_MT_TRACKING_ID, input_mt_init_slots() already set it
#else
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif //MT_PROTOCOL_B
#endif //TOUCH_MAX_FINGER_NUM > 1

#if TOUCH_KEY_NUM > 0
	for (retry = 0; retry < ts->max_button_num; retry++) {
		input_set_capability(ts->input_dev, EV_KEY, touch_key_array[retry]);
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;

	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	if (ts->pen_support) {
		//---allocate pen input device---
		ts->pen_input_dev = input_allocate_device();
		if (ts->pen_input_dev == NULL) {
			NVT_ERR("allocate pen input device failed\n");
			ret = -ENOMEM;
			goto err_pen_input_dev_alloc_failed;
		}

		//---set pen input device info.---
		ts->pen_input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_TOOL_PEN)] |= BIT_MASK(BTN_TOOL_PEN);
		//ts->pen_input_dev->keybit[BIT_WORD(BTN_TOOL_RUBBER)] |= BIT_MASK(BTN_TOOL_RUBBER);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_STYLUS)] |= BIT_MASK(BTN_STYLUS);
		ts->pen_input_dev->keybit[BIT_WORD(BTN_STYLUS2)] |= BIT_MASK(BTN_STYLUS2);
		ts->pen_input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

		nvt_ts_set_pen_abs_params();

		input_set_abs_params(ts->pen_input_dev, ABS_PRESSURE, 0, PEN_PRESSURE_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_DISTANCE, 0, PEN_DISTANCE_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_TILT_X, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);
		input_set_abs_params(ts->pen_input_dev, ABS_TILT_Y, PEN_TILT_MIN, PEN_TILT_MAX, 0, 0);

		sprintf(ts->pen_phys, "input/pen");
		ts->pen_input_dev->name = NVT_PEN_NAME;
		ts->pen_input_dev->phys = ts->pen_phys;
		ts->pen_input_dev->id.bustype = BUS_SPI;

		//---register pen input device---
		ret = input_register_device(ts->pen_input_dev);
		if (ret) {
			NVT_ERR("register pen input device (%s) failed. ret=%d\n", ts->pen_input_dev->name, ret);
			goto err_pen_input_register_device_failed;
		}
	} /* if (ts->pen_support) */

	//---set int-pin & request irq---
	/*
	 * The SPI core has already mapped client->irq from the node's
	 * interrupts property.  The trigger type comes from that mapping;
	 * if the domain left it unspecified, fall back to the type carried
	 * by novatek,irq-gpio, then to the driver default.
	 */
	if (client->irq <= 0) {
		NVT_ERR("no irq mapped from DT\n");
		ret = -EINVAL;
		goto err_int_request_failed;
	}
	{
		u32 irq_type = irqd_get_trigger_type(irq_get_irq_data(client->irq));

		if (irq_type != IRQ_TYPE_NONE)
			ts->int_trigger_type = irq_type;
	}

	NVT_LOG("int_trigger_type=0x%08X\n", ts->int_trigger_type);
	ts->irq_enabled = !ts->is_panel_follower;
	ret = request_threaded_irq(client->irq, NULL, nvt_ts_work_func,
				   ts->int_trigger_type | IRQF_ONESHOT |
				   (ts->is_panel_follower ? IRQF_NO_AUTOEN : 0),
				   NVT_SPI_NAME, ts);
	if (ret != 0) {
		NVT_ERR("request irq failed. ret=%d\n", ret);
		goto err_int_request_failed;
	}
	ts->irq_requested = true;
	if (!ts->is_panel_follower)
		nvt_irq_enable(false);
	NVT_LOG("request irq %d %s\n", client->irq,
		ts->is_panel_follower ? "disabled until firmware is ready" :
		"using legacy cold-boot sequencing");

	ts->pen_is_charge = false;

	if (!ts->is_panel_follower)
		pm_stay_awake(&client->dev);

	ts->ic_state = NVT_IC_INIT;
	ts->dev_pm_suspend = false;
	ts->gesture_command_delayed = -1;
	init_completion(&ts->dev_pm_suspend_completion);
	ts->fw_debug = false;

	ts->pen_input_dev_enable = 1;

#if BOOT_UPDATE_FIRMWARE
	if (!ts->is_panel_follower) {
		nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq",
					     WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
		if (!nvt_fwu_wq) {
			NVT_ERR("nvt_fwu_wq create workqueue failed\n");
			ret = -ENOMEM;
			goto err_create_nvt_fwu_wq_failed;
		}
		INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	}
#endif

	NVT_LOG("NVT_TOUCH_ESD_PROTECT is %d\n", NVT_TOUCH_ESD_PROTECT);
#if NVT_TOUCH_ESD_PROTECT
	INIT_DELAYED_WORK(&nvt_esd_check_work, nvt_esd_check_func);
	nvt_esd_check_wq = alloc_workqueue("nvt_esd_check_wq", WQ_MEM_RECLAIM, 1);
	if (!nvt_esd_check_wq) {
		NVT_ERR("nvt_esd_check_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_esd_check_wq_failed;
	}
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	ts->event_wq = alloc_workqueue("nvt-event-queue",
		WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ts->event_wq) {
		NVT_ERR("Can not create work thread for suspend/resume!!");
		ret = -ENOMEM;
		goto err_alloc_work_thread_failed;
	}
	INIT_WORK(&ts->resume_work, nvt_resume_work);

	/*
	 * Register only after every callback-visible resource exists.  If the
	 * panel object is not registered yet, this is the sole allowed probe
	 * defer and it occurs before any follower SPI write or reset.
	 */
	if (ts->is_panel_follower) {
		ts->panel_follower.funcs = &nt36536_panel_follower_funcs;
		ret = drm_panel_add_follower(&client->dev,
					     &ts->panel_follower);
		if (ret) {
			NVT_ERR("failed to add panel follower: %d\n", ret);
			goto err_register_panel_follower_failed;
		}
		ts->panel_follower_registered = true;
	}

	{
		unsigned long flags;
		bool queue_resume;

		spin_lock_irqsave(&ts->lifecycle_lock, flags);
		ts->resources_ready = true;
		queue_resume = ts->is_panel_follower && ts->panel_on &&
			       !ts->stopping;
		spin_unlock_irqrestore(&ts->lifecycle_lock, flags);

		if (queue_resume)
			queue_work(ts->event_wq, &ts->resume_work);
	}

	if (!ts->is_panel_follower) {
		WRITE_ONCE(ts->touch_awake, true);
#if BOOT_UPDATE_FIRMWARE
		queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, 0);
#endif
#if NVT_TOUCH_ESD_PROTECT
		queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
				   msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif
		if (ts->irq_requested)
			nvt_irq_enable(true);
	}

	NVT_LOG("probe complete: follower=%u panel_on=%u\n",
		ts->is_panel_follower, READ_ONCE(ts->panel_on));

	ret = device_create_file(&client->dev, &dev_attr_point_data);
	if (ret)
		dev_warn(&client->dev,
			 "failed to create point_data diagnostic: %d\n", ret);
	else
		ts->point_data_attr_created = true;

	ret = device_create_file(&client->dev, &dev_attr_touch_state);
	if (ret)
		dev_warn(&client->dev,
			 "failed to create touch_state diagnostic: %d\n", ret);
	else
		ts->touch_state_attr_created = true;

	return 0;

err_register_panel_follower_failed:
err_alloc_work_thread_failed:
#if NVT_TOUCH_ESD_PROTECT
err_create_nvt_esd_check_wq_failed:
#endif
#if BOOT_UPDATE_FIRMWARE
err_create_nvt_fwu_wq_failed:
#endif
	nvt_unregister_panel_follower(ts);
	nvt_destroy_workqueues(ts);
	if (ts->irq_requested) {
		free_irq(client->irq, ts);
		ts->irq_requested = false;
	}
err_int_request_failed:
	nvt_unregister_panel_follower(ts);
	if (ts->pen_support) {
		input_unregister_device(ts->pen_input_dev);
		ts->pen_input_dev = NULL;
	}
err_pen_input_register_device_failed:
	nvt_unregister_panel_follower(ts);
	if (ts->pen_support) {
		if (ts->pen_input_dev) {
			input_free_device(ts->pen_input_dev);
			ts->pen_input_dev = NULL;
		}
	}
err_pen_input_dev_alloc_failed:
	nvt_unregister_panel_follower(ts);
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	nvt_unregister_panel_follower(ts);
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
err_chipvertrim_failed:
	nvt_unregister_panel_follower(ts);
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(client, NULL);
	if (ts->rbuf) {
		kfree(ts->rbuf);
		ts->rbuf = NULL;
	}
err_malloc_rbuf:
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
err_malloc_xbuf:
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static void nvt_ts_remove(struct spi_device *client)
{
	NVT_LOG("Removing driver...\n");
	if (ts->touch_state_attr_created) {
		device_remove_file(&client->dev, &dev_attr_touch_state);
		ts->touch_state_attr_created = false;
	}
	if (ts->point_data_attr_created) {
		device_remove_file(&client->dev, &dev_attr_point_data);
		ts->point_data_attr_created = false;
	}

	nvt_unregister_panel_follower(ts);
	nvt_irq_enable(false);
	nvt_destroy_workqueues(ts);
	if (ts->irq_requested) {
		free_irq(client->irq, ts);
		ts->irq_requested = false;
	}

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	if (ts->pen_support) {
		if (ts->pen_input_dev) {
			input_unregister_device(ts->pen_input_dev);
			ts->pen_input_dev = NULL;
		}
	}

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	spi_set_drvdata(client, NULL);

	if (ts) {
		kfree(ts);
		ts = NULL;
	}
}

static void nvt_ts_shutdown(struct spi_device *client)
{
	NVT_LOG("Shutdown driver...\n");
	if (ts->touch_state_attr_created) {
		device_remove_file(&client->dev, &dev_attr_touch_state);
		ts->touch_state_attr_created = false;
	}
	if (ts->point_data_attr_created) {
		device_remove_file(&client->dev, &dev_attr_point_data);
		ts->point_data_attr_created = false;
	}

	nvt_unregister_panel_follower(ts);
	nvt_irq_enable(false);
	nvt_destroy_workqueues(ts);
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
	bool was_awake;
#if MT_PROTOCOL_B
	uint32_t i = 0;
#endif

	was_awake = READ_ONCE(ts->touch_awake);
	if (!was_awake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}

	pm_stay_awake(dev);
	ts->ic_state = NVT_IC_SUSPEND_IN;

#if NVT_TOUCH_ESD_PROTECT
	NVT_LOG("cancel delayed work sync\n");
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (!ts->db_wakeup) {
		if (!ts->irq_enabled)
			NVT_LOG("IRQ already disabled\n");
		else
			nvt_irq_enable(false);
	}

	mutex_lock(&ts->lock);
	if (!READ_ONCE(ts->touch_awake)) {
		mutex_unlock(&ts->lock);
		pm_relax(dev);
		return 0;
	}

	NVT_LOG("suspend start\n");

	if (ts->pen_input_dev_enable) {
		NVT_LOG("if enable pen, will close it\n");
		__nvt_set_pen_state(true);
	}

	if (ts->db_wakeup) {
		/*---write command to enter "wakeup gesture mode"---*/
		/*DoubleClick wakeup CMD was sent by display to meet timing*/
		/*
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);
		*/
		enable_irq_wake(ts->client->irq);

		NVT_LOG("Enabled touch wakeup gesture\n");
	} else {
		/*---write command to enter "deep sleep mode"---*/
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
	}
	nvt_set_touch_awake_state(ts, false);

	mutex_unlock(&ts->lock);

	/* release all touches */
#if MT_PROTOCOL_B
	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#endif
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
#if !MT_PROTOCOL_B
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	msleep(50);
	/* release pen event */
	release_pen_event();
	if (likely(ts->ic_state == NVT_IC_SUSPEND_IN))
		ts->ic_state = NVT_IC_SUSPEND_OUT;
	else
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	pm_relax(dev);
	NVT_LOG("end\n");

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	bool hold_wake = ts->dev_pm_suspend;
	int ret;

	if (READ_ONCE(ts->touch_awake)) {
		NVT_LOG("Touch is already resume\n");
		return 0;
	}
	if (ts->is_panel_follower && !nvt_panel_resume_allowed(ts)) {
		NVT_LOG("panel is not ready for touch resume\n");
		return -EHOSTDOWN;
	}

	/* Keep every failure path quiescent before taking the SPI mutex. */
	if (ts->irq_requested)
		nvt_irq_enable(false);
#if NVT_TOUCH_ESD_PROTECT
	if (nvt_esd_check_wq) {
		cancel_delayed_work_sync(&nvt_esd_check_work);
		nvt_esd_check_enable(false);
	}
#endif

	if (hold_wake)
		pm_stay_awake(dev);

	mutex_lock(&ts->lock);
	if (ts->is_panel_follower && !nvt_panel_is_on(ts)) {
		ret = -EHOSTDOWN;
		goto resume_fail;
	}

	ts->resume_count++;
	NVT_LOG("follower resume #%u start\n", ts->resume_count);
	ts->ic_state = NVT_IC_RESUME_IN;

	/*
	 * The panel has already reset and initialized the shared TDDI.  A
	 * follower may only use the HW-CRC download path, which is explicitly
	 * guaranteed not to issue a standalone engineering reset.
	 */
	if (ts->is_panel_follower)
		ret = nvt_update_firmware_no_eng_reset(ts->fw_name);
	else
		ret = nvt_update_firmware(ts->fw_name);
	if (ret) {
		NVT_ERR("download firmware failed: %d\n", ret);
		goto resume_fail;
	}

	ret = nvt_check_fw_reset_state(RESET_STATE_REK);
	if (ret) {
		NVT_ERR("firmware did not reach ReK: %d\n", ret);
		goto resume_fail;
	}

	if (ts->is_panel_follower && !nvt_panel_is_on(ts)) {
		ret = -EHOSTDOWN;
		goto resume_fail;
	}

	ret = __nvt_set_pen_state(false);
	if (ret) {
		NVT_ERR("failed to enable pen mailbox: %d\n", ret);
		goto resume_fail;
	}

	if (ts->is_panel_follower && !nvt_commit_follower_awake(ts)) {
		ret = -EHOSTDOWN;
		goto resume_fail;
	}
	if (!ts->is_panel_follower)
		nvt_set_touch_awake_state(ts, true);

	if (likely(ts->ic_state == NVT_IC_RESUME_IN)) {
		ts->ic_state = NVT_IC_RESUME_OUT;
	} else {
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	}
	if (ts->gesture_command_delayed >= 0){
		ts->db_wakeup = ts->gesture_command_delayed;
		ts->gesture_command_delayed = -1;
		NVT_LOG("execute delayed command, set double click wakeup %d\n", ts->db_wakeup);
	}

	mutex_unlock(&ts->lock);

	if (ts->irq_requested)
		nvt_irq_enable(true);
#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
	queue_delayed_work(nvt_esd_check_wq, &nvt_esd_check_work,
			   msecs_to_jiffies(NVT_TOUCH_ESD_CHECK_PERIOD));
#endif

	if (hold_wake)
		pm_relax(dev);
	NVT_LOG("follower resume #%u complete\n", ts->resume_count);

	return 0;

resume_fail:
	nvt_set_touch_awake_state(ts, false);
	ts->ic_state = NVT_IC_SUSPEND_OUT;
	mutex_unlock(&ts->lock);
	if (hold_wake)
		pm_relax(dev);
	NVT_ERR("resume failed closed: %d (IRQ off, ESD off)\n", ret);
	return ret;
}

static int nvt_pm_suspend(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("enable touch irq wake\n");
		enable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = true;
	reinit_completion(&ts->dev_pm_suspend_completion);

	return 0;
}

static int nvt_pm_resume(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("disable touch irq wake\n");
		disable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = false;
	complete(&ts->dev_pm_suspend_completion);

	return 0;
}

static const struct dev_pm_ops nvt_dev_pm_ops = {
	.suspend = nvt_pm_suspend,
	.resume = nvt_pm_resume,
};

static int panel_prepared(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *ts = container_of(follower, struct nvt_ts_data,
					       panel_follower);
	unsigned long flags;
	bool queue_resume;

	spin_lock_irqsave(&ts->lifecycle_lock, flags);
	ts->panel_on = true;
	queue_resume = ts->resources_ready && !ts->stopping;
	spin_unlock_irqrestore(&ts->lifecycle_lock, flags);

	NVT_LOG("panel prepared\n");

	if (queue_resume)
		queue_work(ts->event_wq, &ts->resume_work);

	return 0;
}

static int panel_unpreparing(struct drm_panel_follower *follower)
{
	struct nvt_ts_data *ts = container_of(follower, struct nvt_ts_data,
					       panel_follower);
	unsigned long flags;
	bool resources_ready;

	spin_lock_irqsave(&ts->lifecycle_lock, flags);
	ts->panel_on = false;
	resources_ready = ts->resources_ready;
	spin_unlock_irqrestore(&ts->lifecycle_lock, flags);

	NVT_LOG("panel unpreparing\n");

	/* The callback is the barrier before the panel's physical reset. */
	if (ts->event_wq)
		cancel_work_sync(&ts->resume_work);

	if (!resources_ready)
		return 0;
	return nvt_ts_suspend(&ts->client->dev);
}

static struct drm_panel_follower_funcs nt36536_panel_follower_funcs = {
	.panel_prepared = panel_prepared,
	.panel_unpreparing = panel_unpreparing,
};

static const struct spi_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, nvt_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id nt36536_of_match[] = {
	{ .compatible = "novatek,nt36536",},
	{ },
};
MODULE_DEVICE_TABLE(of, nt36536_of_match);
#endif

static struct spi_driver nvt_spi_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &nvt_dev_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = nt36536_of_match
#endif
	},
};

module_spi_driver(nvt_spi_driver);

MODULE_DESCRIPTION("Novatek NT36536 noflash SPI Touchscreen Driver");
MODULE_LICENSE("GPL");
