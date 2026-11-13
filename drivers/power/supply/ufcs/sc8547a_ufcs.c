// SPDX-License-Identifier: GPL-2.0-only
/*
 * SC8547A UFCS PHY, adapted from OnePlusOSS cc463675855af,
 * vendor/oplus/kernel/charger/v2/ufcs_ic/oplus_hal_sc8547a.c.
 * Copyright (C) 2022-2023 Oplus. All rights reserved.
 */
#include <linux/delay.h>
#include <linux/regmap.h>
#include "sc8547a_ufcs.h"

#define UFCS_CTRL 0x40
#define UFCS_FLAGS 0x41
#define TX_LEN 0x47
#define TX_BUF 0x48
#define RX_LEN 0x6c
#define RX_BUF 0x6d
#define WATCHDOG 0x09

#define SC8547A_FLAG_NUM (3)
#define SC8547A_FLAG_ACK_RECEIVE_TIMEOUT BIT(0)
#define SC8547A_FLAG_HARD_RESET BIT(1)
#define SC8547A_FLAG_DATA_READY BIT(2)
#define SC8547A_FLAG_SENT_PACKET_COMPLETE BIT(3)
#define SC8547A_FLAG_CRC_ERROR BIT(4)
#define SC8547A_FLAG_BAUD_RATE_ERROR BIT(5)
#define SC8547A_FLAG_HANDSHAKE_SUCCESS BIT(6)
#define SC8547A_FLAG_HANDSHAKE_FAIL BIT(7)
#define SC8547A_FLAG_TRAINING_BYTE_ERROR BIT(0)
#define SC8547A_FLAG_MSG_TRANS_FAIL BIT(1)
#define SC8547A_FLAG_DATA_BYTE_TIMEOUT BIT(3)
#define SC8547A_FLAG_BAUD_RATE_CHANGE BIT(4)
#define SC8547A_FLAG_LENGTH_ERROR BIT(5)
#define SC8547A_FLAG_RX_OVERFLOW BIT(6)
#define SC8547A_FLAG_BUS_CONFLICT BIT(7)

static struct regmap *phy_map(struct ufcs_dev *ufcs)
{
	struct sc8547a_ufcs_phy *phy = ufcs->drv_data;

	return phy->map;
}

int sc8547a_ufcs_irq(struct ufcs_dev *ufcs)
{
	u8 flag_buf[SC8547A_FLAG_NUM];
	unsigned int err_flag = 0;
	int ret;

	ret = regmap_bulk_read(phy_map(ufcs), UFCS_FLAGS, flag_buf, sizeof(flag_buf));
	if (ret)
		return ret;
	if (flag_buf[1] & SC8547A_FLAG_ACK_RECEIVE_TIMEOUT)
		err_flag |= BIT(UFCS_RECV_ERR_ACK_TIMEOUT);
	if (flag_buf[1] & SC8547A_FLAG_HARD_RESET)
		err_flag |= BIT(UFCS_HW_ERR_HARD_RESET);
	if (flag_buf[1] & SC8547A_FLAG_DATA_READY)
		err_flag |= BIT(UFCS_RECV_ERR_DATA_READY);
	if (flag_buf[1] & SC8547A_FLAG_SENT_PACKET_COMPLETE)
		err_flag |= BIT(UFCS_RECV_ERR_SENT_CMP);
	if (flag_buf[1] & SC8547A_FLAG_CRC_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_CRC_ERR);
	if (flag_buf[1] & SC8547A_FLAG_BAUD_RATE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_ERR);

	if (flag_buf[2] & SC8547A_FLAG_TRAINING_BYTE_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_TRAINING_ERR);
	if (flag_buf[2] & SC8547A_FLAG_MSG_TRANS_FAIL)
		err_flag |= BIT(UFCS_RECV_ERR_TRANS_FAIL);
	if (flag_buf[2] & SC8547A_FLAG_DATA_BYTE_TIMEOUT)
		err_flag |= BIT(UFCS_COMM_ERR_BYTE_TIMEOUT);
	if (flag_buf[2] & SC8547A_FLAG_BAUD_RATE_CHANGE)
		err_flag |= BIT(UFCS_COMM_ERR_BAUD_RATE_CHANGE);
	if (flag_buf[2] & SC8547A_FLAG_LENGTH_ERROR)
		err_flag |= BIT(UFCS_COMM_ERR_RX_LEN_ERR);
	if (flag_buf[2] & SC8547A_FLAG_RX_OVERFLOW)
		err_flag |= BIT(UFCS_COMM_ERR_RX_OVERFLOW);
	if (flag_buf[2] & SC8547A_FLAG_BUS_CONFLICT)
		err_flag |= BIT(UFCS_COMM_ERR_BUS_CONFLICT);
	ufcs->err_flag_save = err_flag;

	if (ufcs->handshake_state == UFCS_HS_WAIT) {
		if ((flag_buf[1] & SC8547A_FLAG_HANDSHAKE_SUCCESS) &&
		    !(flag_buf[1] & SC8547A_FLAG_HANDSHAKE_FAIL)) {
			ufcs->handshake_state = UFCS_HS_SUCCESS;
		} else if (flag_buf[1] & SC8547A_FLAG_HANDSHAKE_FAIL) {
			ufcs->handshake_state = UFCS_HS_FAIL;
		}
	}

	ret = ufcs_set_error_flag(ufcs, err_flag);
	if (ret < 0)
		return ret;
	return ufcs_msg_handler(ufcs);
}
EXPORT_SYMBOL_GPL(sc8547a_ufcs_irq);

static int phy_write(struct ufcs_dev *ufcs, u8 *buf, int len)
{
	struct regmap *map = phy_map(ufcs);
	int rc;

	if (len < 1 || len > 36)
		return -EMSGSIZE;
	rc = regmap_write(map, TX_LEN, len);
	if (rc)
		return rc;
	rc = regmap_bulk_write(map, TX_BUF, buf, len);
	if (rc)
		return rc;
	return regmap_write_bits(map, UFCS_CTRL, BIT(2), BIT(2));
}

static int phy_read(struct ufcs_dev *ufcs, u8 *buf, int len)
{
	struct regmap *map = phy_map(ufcs);
	unsigned int size;
	int rc;

	rc = regmap_read(map, RX_LEN, &size);
	if (rc)
		return rc;
	if (!size || size > len || size > 125)
		return -EMSGSIZE;
	rc = regmap_bulk_read(map, RX_BUF, buf, size);
	return rc ? rc : (int)size;
}

static int phy_disable(struct ufcs_dev *ufcs)
{
	int rc = regmap_write(phy_map(ufcs), UFCS_CTRL, 0);
	int wd = regmap_update_bits(phy_map(ufcs), WATCHDOG, 7, 0);

	return rc ? rc : wd;
}

static int phy_enable(struct ufcs_dev *ufcs)
{
	struct sc8547a_ufcs_phy *phy = ufcs->drv_data;
	static const struct reg_sequence enable[] = {
		{ 0x21, 0x01 }, { UFCS_CTRL, 0x80 }, { 0xea, 0xc0 },
		{ UFCS_FLAGS, 0x80 }, { 0x44, 0x01 }, { 0x45, 0x01 },
	};
	int rc;

	rc = phy->prepare(phy->ctx);
	if (rc)
		return rc;
	rc = regmap_multi_reg_write(phy->map, enable, ARRAY_SIZE(enable));
	if (rc)
		return rc;
	rc = regmap_update_bits(phy->map, 0xeb, 0xc0, 0x80);
	if (rc)
		return rc;
	return regmap_update_bits(phy->map, WATCHDOG, 7, 3);
}

static int phy_handshake(struct ufcs_dev *ufcs)
{
	return regmap_write_bits(phy_map(ufcs), UFCS_CTRL, BIT(5), BIT(5));
}

static int phy_source_reset(struct ufcs_dev *ufcs)
{
	int rc = regmap_write_bits(phy_map(ufcs), UFCS_CTRL, BIT(0), BIT(0));

	if (!rc)
		msleep(100);
	return rc;
}

static int phy_cable_reset(struct ufcs_dev *ufcs)
{
	return -EOPNOTSUPP;
}

static int phy_baud(struct ufcs_dev *ufcs, enum ufcs_baud_rate baud)
{
	return regmap_update_bits(phy_map(ufcs), UFCS_CTRL, GENMASK(4, 3), baud << 3);
}

static int phy_watchdog(struct ufcs_dev *ufcs, unsigned int ms)
{
	static const unsigned int limits[] = { 0, 200, 500, 1000, 5000, 30000 };
	int i;

	for (i = 0; i < ARRAY_SIZE(limits); i++)
		if (ms <= limits[i])
			return regmap_update_bits(phy_map(ufcs), WATCHDOG, 7, i);
	return -EINVAL;
}

static int phy_init(struct ufcs_dev *ufcs)
{
	return 0;
}

static struct ufcs_dev_ops phy_ops = {
	.init = phy_init,
	.write_msg = phy_write,
	.read_msg = phy_read,
	.enable = phy_enable,
	.disable = phy_disable,
	.handshake = phy_handshake,
	.source_hard_reset = phy_source_reset,
	.cable_hard_reset = phy_cable_reset,
	.set_baud_rate = phy_baud,
	.watchdog_config = phy_watchdog,
};

struct ufcs_dev *sc8547a_ufcs_register(struct device *dev,
				       struct sc8547a_ufcs_phy *phy)
{
	struct ufcs_config config = {
		.handshake_hard_retry = true,
		.ic_vendor_id = 6,
	};
	unsigned int chip_id;
	int rc;

	if (!phy || !phy->map || !phy->prepare)
		return ERR_PTR(-EINVAL);
	rc = regmap_read(phy->map, 0x36, &chip_id);
	if (rc)
		return ERR_PTR(rc);
	if (chip_id != 0x67)
		return ERR_PTR(-ENODEV);
	return ufcs_device_register(dev, &phy_ops, phy, &config);
}
EXPORT_SYMBOL_GPL(sc8547a_ufcs_register);
