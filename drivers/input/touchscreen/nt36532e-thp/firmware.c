// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * Adapted from MiCode piano p81/nt36532/nt36xxx_fw_update.c and nt36xxx.c.
 * See README.rst for pinned sources and the supported firmware layout.
 */
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#include "firmware.h"

#define NVT_ADDRESS_MAX		0x7fffff
#define NVT_EVENT_BUF		0x11c400
#define NVT_AUTO_COPY		0x1fc925
#define NVT_RESET		0x1fb43e
#define NVT_BOOT_READY		0x1fb50d

static int nvt_partition(const u8 *data, size_t size, size_t position,
			 bool primary, struct nvt_thp_partition *part)
{
	u32 encoded_size;

	if (position > size || size - position < (primary ? 12 : 16))
		return -EINVAL;
	if (primary) {
		part->offset = get_unaligned_le32(data + position);
		part->address = get_unaligned_le32(data + position + 4);
		encoded_size = get_unaligned_le32(data + position + 8);
		part->crc = get_unaligned_le32(data + 0x18 + position / 12 * 4);
	} else {
		part->address = get_unaligned_le32(data + position);
		encoded_size = get_unaligned_le32(data + position + 4);
		part->offset = get_unaligned_le32(data + position + 8);
		part->crc = get_unaligned_le32(data + position + 12);
	}
	/* Zero denotes a reserved entry; all other lengths are inclusive. */
	if (!encoded_size) {
		part->length = 0;
		return primary ? -EINVAL : 0;
	}
	if (part->offset >= size || encoded_size >= size - part->offset ||
	    part->address > NVT_ADDRESS_MAX || encoded_size > NVT_ADDRESS_MAX - part->address)
		return -EINVAL;
	part->length = encoded_size + 1;
	return 0;
}

int nvt_thp_parse_firmware(const u8 *data, size_t size,
			   struct nvt_thp_firmware *image)
{
	size_t end, first_end, position, write_end;
	unsigned int info_count, overlays, index;
	bool found_header = false;
	int ret;

	memset(image, 0, sizeof(*image));
	if (size < SZ_4K)
		return -EINVAL;
	for (write_end = round_down(size, SZ_4K); write_end; write_end -= SZ_4K)
		if (!memcmp(data + write_end - 3, "NVT", 3) ||
		    !memcmp(data + write_end - 3, "MOD", 3))
			break;
	if (!write_end || data[write_end - SZ_4K] + data[write_end - SZ_4K + 1] != 0xff)
		return -ENOEXEC;

	end = get_unaligned_le32(data);
	if (end < 0x40 || end > size || !IS_ALIGNED(end, 16))
		return -EINVAL;
	/* This transport supports the observed cascade, non-DMA-divider layout. */
	if (!(data[0x20] & BIT(1)) || (data[0x29] & BIT(0)))
		return -EOPNOTSUPP;
	first_end = end;
	for (position = 0x30; position < end; position += 16) {
		if (get_unaligned_le32(data + position + 4) &&
		    !get_unaligned_le32(data + position + 8)) {
			first_end = position + 16;
			found_header = true;
			break;
		}
	}
	if (!found_header || first_end > end / 2)
		return -EINVAL;
	end = first_end * 2;
	info_count = (first_end - 0x30) / 16 + 1;
	overlays = data[0x28] & BIT(4) ? data[0x28] & 0x0f : 0;
	if (info_count > U8_MAX)
		return -E2BIG;
	image->count = 2 + info_count + overlays;
	image->partitions = kcalloc(image->count, sizeof(*image->partitions), GFP_KERNEL);
	if (!image->partitions)
		return -ENOMEM;
	found_header = false;
	for (index = 0; index < image->count; ++index) {
		if (index < 2)
			position = index * 12;
		else if (index < 2 + info_count)
			position = found_header ? end - 16 : 0x30 + (index - 2) * 16;
		else
			position = (size_t)image->partitions[1].offset +
				   (index - 2 - info_count) * 16;
		ret = nvt_partition(data, size, position, index < 2, &image->partitions[index]);
		if (ret)
			goto fail;
		if (index >= 2 && index < 2 + info_count &&
		    image->partitions[index].length && image->partitions[index].offset < end)
			found_header = true;
	}
	return 0;
fail:
	nvt_thp_free_firmware(image);
	return ret;
}

void nvt_thp_free_firmware(struct nvt_thp_firmware *image)
{
	kfree(image->partitions);
	memset(image, 0, sizeof(*image));
}

static int nvt_boot_reset(struct regmap *map)
{
	int ret = regmap_write(map, NVT_RESET, 0x69);

	if (!ret)
		usleep_range(5000, 6000); /* Vendor tBRST2FR. */
	return ret;
}

static int nvt_crc_bank(struct regmap *map, const struct nvt_thp_partition *part,
			u32 destination, u32 length, u32 golden)
{
	u8 bytes[4];
	int ret;

	put_unaligned_le32(part->address, bytes);
	ret = regmap_bulk_write(map, destination, bytes, 3);
	if (ret)
		return ret;
	put_unaligned_le32(part->length - 1, bytes);
	ret = regmap_bulk_write(map, length, bytes, 3);
	if (ret)
		return ret;
	put_unaligned_le32(part->crc, bytes);
	return regmap_bulk_write(map, golden, bytes, sizeof(bytes));
}

int nvt_thp_boot_firmware(struct regmap *map, const struct firmware *firmware,
			  const struct nvt_thp_firmware *image, u16 *payload_length)
{
	u8 identity[6], info[39], zeros[6] = {}, crc_command[] = { 0xae, 0 };
	unsigned int value, index;
	int ret;

	ret = regmap_write(map, 0x7fff80, 0x5a);
	if (ret)
		return ret;
	usleep_range(1000, 2000); /* Vendor tMCU_Idle2TP_REX_Hi. */
	usleep_range(10000, 11000); /* Vendor post-reset POR interval before identification. */
	ret = nvt_boot_reset(map);
	if (ret)
		return ret;
	ret = regmap_bulk_write(map, 0x1fb104, zeros, sizeof(zeros));
	if (ret)
		return ret;
	ret = regmap_bulk_read(map, 0x1fb104, identity, sizeof(identity));
	if (ret)
		return ret;
	if (identity[3] != 0x32 || identity[4] != 0x65 || identity[5] != 0x03)
		return -ENODEV;
	ret = regmap_read(map, 0x1fb12c, &value);
	if (ret)
		return ret;
	if (value & BIT(0))
		return -EOPNOTSUPP;
	ret = nvt_boot_reset(map);
	if (ret)
		return ret;
	ret = nvt_crc_bank(map, &image->partitions[0], 0x1fb528, 0x1fb518, 0x1fb500);
	if (ret)
		return ret;
	ret = nvt_crc_bank(map, &image->partitions[1], 0x1fb52c, 0x1fb530, 0x1fb504);
	if (ret)
		return ret;
	ret = regmap_write(map, NVT_AUTO_COPY, 0x56);
	if (ret)
		return ret;
	for (index = 0; index < image->count; ++index) {
		const struct nvt_thp_partition *part = &image->partitions[index];

		if (!part->length)
			continue;
		ret = regmap_bulk_write(map, part->address,
					firmware->data + part->offset, part->length);
		if (ret)
			return ret;
	}
	ret = regmap_read_poll_timeout(map, NVT_AUTO_COPY, value, !value, 1000, 200000);
	if (ret)
		return ret;
	ret = regmap_bulk_write(map, NVT_EVENT_BUF + 0x60, zeros, sizeof(zeros));
	if (ret)
		return ret;
	ret = regmap_bulk_write(map, NVT_EVENT_BUF + 0x50, crc_command, sizeof(crc_command));
	if (ret)
		return ret;
	ret = regmap_write(map, NVT_BOOT_READY, 1);
	if (ret)
		return ret;
	usleep_range(5000, 6000); /* Vendor boot-ready settling time. */
	ret = regmap_read_poll_timeout(map, NVT_EVENT_BUF + 0x60, value,
				       value >= 0xa0 && value <= 0xaf, 10000, 100000);
	if (ret)
		return ret;
	ret = regmap_bulk_read(map, NVT_EVENT_BUF + 0x78, info, sizeof(info) - 1);
	if (ret)
		return ret;
	if (info[0] + info[1] != 0xff)
		return -EBADMSG;
	ret = regmap_bulk_read(map, 0x1093d8, info, sizeof(info));
	if (ret)
		return ret;
	*payload_length = get_unaligned_le16(info + 2);
	return 0;
}
