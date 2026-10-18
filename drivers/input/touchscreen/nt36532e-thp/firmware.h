/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef _NT36532E_THP_FIRMWARE_H
#define _NT36532E_THP_FIRMWARE_H

#include <linux/ktime.h>
#include <linux/types.h>

struct firmware;
struct regmap;

enum nvt_thp_reset_state {
	NVT_RESET_STATE_INIT = 0xa0,
	NVT_RESET_STATE_REK = 0xa1,
	NVT_RESET_STATE_REK_FINISH = 0xa2,
	NVT_RESET_STATE_NORMAL_RUN = 0xa3,
	NVT_RESET_STATE_MAX = 0xaf,
};

struct nvt_thp_boot_status {
	ktime_t reset_time;
	s64 rek_ms;
	u8 reset_state;
};

int nvt_thp_read_reset_state(struct regmap *map, u8 *state);
int nvt_thp_wait_rek(struct regmap *map, u8 *state);

struct nvt_thp_partition {
	u32 offset;
	u32 address;
	u32 length;
	u32 crc;
};

struct nvt_thp_firmware {
	struct nvt_thp_partition *partitions;
	unsigned int count;
};

int nvt_thp_parse_firmware(const u8 *data, size_t size,
			   struct nvt_thp_firmware *image);
void nvt_thp_free_firmware(struct nvt_thp_firmware *image);
int nvt_thp_boot_firmware(struct regmap *map, const struct firmware *firmware,
			  const struct nvt_thp_firmware *image, u16 *payload_length,
			  struct nvt_thp_boot_status *status);
/* Last host command written and the command byte last read back. */
struct nvt_thp_command_status {
	u8 sub_command;
	u8 status;
};

int nvt_thp_get_extend_command(struct regmap *map, u8 sub_command, u16 *value,
			       struct nvt_thp_command_status *last);

int nvt_thp_apply_host_commands(struct regmap *map, struct nvt_thp_command_status *last);

#endif
