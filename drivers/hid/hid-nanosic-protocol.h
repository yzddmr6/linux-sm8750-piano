/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WN8030 protocol helpers, without device I/O or power policy.
 * Firmware framing is based on hid-nanosic-wn8030.c by map220v:
 * Copyright (c) 2025 map220v <map220v300@gmail.com>
 * Input framing follows the liuqin hid-nanosic.c transport, with piano's
 * 27-byte pressure-bearing touch report. Neither board's timings are here.
 */
#ifndef _HID_NANOSIC_PROTOCOL_H
#define _HID_NANOSIC_PROTOCOL_H

#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define NANOSIC_BOOT_REGISTER		0x5c
#define NANOSIC_REPORT_REGISTER		0x4c
#define NANOSIC_FW_HEADER_OFFSET		0x7fc0
#define NANOSIC_FW_CODE_OFFSET		0x8000
#define NANOSIC_FW_HEADER_CMD_SIZE	65
#define NANOSIC_FW_DATA_CMD_SIZE		512
#define NANOSIC_FW_CHUNK_SIZE		503
#define NANOSIC_FRAME_SIZE		68

struct nanosic_firmware {
	const u8 *header;
	const u8 *code;
	u32 size;
	u32 word_sum;
};

static inline int nanosic_firmware_parse(const u8 *data, size_t size,
					struct nanosic_firmware *fw)
{
	const u8 *header;
	u32 code_size, byte_sum = 0, word_sum = 0;
	u8 header_sum = 0;
	size_t i;

	if (size < NANOSIC_FW_CODE_OFFSET)
		return -EINVAL;
	header = data + NANOSIC_FW_HEADER_OFFSET;
	if (memcmp(header, "NANO IC", 7) || get_unaligned_le16(header + 7) != 0x8030)
		return -EINVAL;

	code_size = get_unaligned_le32(header + 48);
	if (!code_size || code_size % 4 || code_size > size - NANOSIC_FW_CODE_OFFSET)
		return -EINVAL;
	for (i = 0; i < 63; i++)
		header_sum += header[i];
	if (header_sum != header[63])
		return -EBADMSG;

	for (i = 0; i < code_size; i++)
		byte_sum += data[NANOSIC_FW_CODE_OFFSET + i];
	if (byte_sum != get_unaligned_le32(header + 52))
		return -EBADMSG;
	for (i = 0; i < code_size; i += 4)
		word_sum += get_unaligned_le32(data + NANOSIC_FW_CODE_OFFSET + i);

	*fw = (struct nanosic_firmware) {
		.header = header,
		.code = data + NANOSIC_FW_CODE_OFFSET,
		.size = code_size,
		.word_sum = word_sum,
	};
	return 0;
}

/* The caller retains the validated firmware bytes while building commands. */
static inline void nanosic_firmware_header(const struct nanosic_firmware *fw,
					   u8 cmd[NANOSIC_FW_HEADER_CMD_SIZE])
{
	u8 checksum = 0;
	unsigned int i;

	memset(cmd, 0, NANOSIC_FW_HEADER_CMD_SIZE);
	cmd[0] = 0x06;
	memcpy(cmd + 1, fw->header, 7);
	put_unaligned_le16(0x8140, cmd + 8);
	put_unaligned_le32(0x2345, cmd + 10);
	put_unaligned_le32(0x40, cmd + 14);
	put_unaligned_le32(fw->size, cmd + 18);
	/* Entry, product/vendor IDs and two version bytes retain their wire order. */
	memcpy(cmd + 22, fw->header + 21, 10);
	put_unaligned_le32(fw->word_sum, cmd + 53);
	for (i = 1; i < NANOSIC_FW_HEADER_CMD_SIZE - 1; i++)
		checksum += cmd[i];
	cmd[NANOSIC_FW_HEADER_CMD_SIZE - 1] = checksum;
}

/* Returns the number of code bytes consumed; the wire payload is always 512. */
static inline int nanosic_firmware_chunk(const struct nanosic_firmware *fw,
					u32 offset, u8 cmd[NANOSIC_FW_DATA_CMD_SIZE])
{
	u32 count;

	if (offset >= fw->size)
		return -EINVAL;
	count = min_t(u32, NANOSIC_FW_CHUNK_SIZE, fw->size - offset);
	memset(cmd, 0, NANOSIC_FW_DATA_CMD_SIZE);
	cmd[0] = 0x07;
	put_unaligned_be32(offset, cmd + 1);
	put_unaligned_be32(count, cmd + 5);
	memcpy(cmd + 9, fw->code + offset, count);
	return count;
}

struct nanosic_report {
	u8 *data;
	u8 wire_size;
	u8 hid_size;
};

/* Start with cursor=0. Stop on 0 or error; unknown reports cannot be skipped. */
static inline int nanosic_frame_next(u8 *frame, size_t size, size_t *cursor,
				     struct nanosic_report *report)
{
	u8 wire_size, hid_size;
	size_t offset = *cursor;

	if (size < 3 || size > NANOSIC_FRAME_SIZE)
		return -EMSGSIZE;
	if (!offset) {
		if ((frame[0] & 0x7f) != 0x57)
			return -EBADMSG;
		switch (frame[2]) {
		case 0x00:
			return 0;
		case 0x39:
		case 0x4a:
		case 0x5b:
		case 0x6c:
			break;
		default:
			return -EBADMSG;
		}
		offset = 3;
	}
	if (offset < 3 || offset > size)
		return -EINVAL;
	if (offset == size || !frame[offset])
		return 0;

	switch (frame[offset]) {
	case 0x02:
		wire_size = hid_size = 8;
		break;
	case 0x05:
		wire_size = hid_size = 9;
		break;
	case 0x06:
		wire_size = 5;
		hid_size = 3;
		break;
	case 0x19:
		wire_size = hid_size = 27;
		break;
	case 0x22:
		wire_size = 16;
		hid_size = 0;
		break;
	case 0x23:
		wire_size = 32;
		hid_size = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (wire_size > size - offset)
		return -EMSGSIZE;
	*report = (struct nanosic_report) {
		.data = frame + offset,
		.wire_size = wire_size,
		.hid_size = hid_size,
	};
	*cursor = offset + wire_size;
	return 1;
}

#endif
