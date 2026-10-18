// SPDX-License-Identifier: GPL-2.0-or-later
#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "firmware.h"

struct nvt_mock {
	u8 *memory;
	unsigned int calls, fail_call, uploaded, resets;
	unsigned int host_commands, host_polls, busy_reads;
	u8 host_command[3][5];
	u8 reset_states[4];
	unsigned int reset_count, reset_polls;
	u16 readback_value;
	bool bad_id, bad_info, boot_ready, stuck_command;
};

static void nvt_test_release(void *data)
{
	release_firmware(data);
}

static const struct firmware *nvt_fixture(struct kunit *test)
{
	const struct firmware *firmware;
	struct device *dev = kunit_device_register(test, "nvt-firmware-test");
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	ret = request_firmware_direct(&firmware, "nt36532e/piano-test.bin", dev);
	if (ret == -ENOENT) {
		kunit_skip(test, "optional private piano firmware fixture not supplied");
		return NULL;
	}
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = kunit_add_action_or_reset(test, nvt_test_release, (void *)firmware);
	KUNIT_ASSERT_EQ(test, ret, 0);
	return firmware;
}

static void nvt_test_free_image(void *data)
{
	nvt_thp_free_firmware(data);
}

static struct nvt_thp_firmware *nvt_image(struct kunit *test, const struct firmware *fw)
{
	struct nvt_thp_firmware *image = kunit_kzalloc(test, sizeof(*image), GFP_KERNEL);

	KUNIT_ASSERT_NOT_NULL(test, image);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_free_image, image), 0);
	KUNIT_ASSERT_EQ(test, nvt_thp_parse_firmware(fw->data, fw->size, image), 0);
	return image;
}

static int nvt_mock_write(void *context, const void *data, size_t count)
{
	struct nvt_mock *mock = context;
	u32 address = get_unaligned((const u32 *)data);
	const u8 *value = (const u8 *)data + sizeof(u32);
	size_t size = count - sizeof(u32);

	if (++mock->calls == mock->fail_call)
		return -EREMOTEIO;
	if (size > 63 * 1024)
		return -EMSGSIZE;
	if (address == 0x7fff80)
		return size == 1 && value[0] == 0x5a ? 0 : -EINVAL;
	if (address >= SZ_2M || size > SZ_2M - address)
		return -EINVAL;
	memcpy(mock->memory + address, value, size);
	if (address == 0x11c450 && (size == 3 || size == 5) && value[0] == 0xbf) {
		if (mock->host_commands < ARRAY_SIZE(mock->host_command))
			memcpy(mock->host_command[mock->host_commands], value, size);
		mock->host_commands++;
		/* Firmware reports the command busy once, then clears it. */
		mock->busy_reads = 1;
	}
	if (address < 0x120000 && address != 0x11c460 && address != 0x11c450)
		mock->uploaded += size;
	if (address == 0x1fb43e)
		mock->resets++;
	if (address == 0x1fb50d)
		mock->boot_ready = true;
	return 0;
}

static int nvt_mock_read(void *context, const void *reg, size_t reg_size,
			 void *data, size_t size)
{
	struct nvt_mock *mock = context;
	u32 address = get_unaligned((const u32 *)reg);
	u8 *value = data;

	if (++mock->calls == mock->fail_call)
		return -EREMOTEIO;
	memset(value, 0, size);
	switch (address) {
	case 0x1fb104:
		if (size != 6)
			return -EINVAL;
		value[3] = mock->bad_id ? 0 : 0x32;
		value[4] = 0x65;
		value[5] = 0x03;
		break;
	case 0x1fb12c:
	case 0x1fc925:
		break;
	case 0x11c460:
		if (mock->boot_ready)
			value[0] = mock->reset_count ?
				mock->reset_states[min(mock->reset_polls, mock->reset_count - 1)] :
				NVT_RESET_STATE_REK;
		mock->reset_polls++;
		break;
	case 0x11c450:
		if (size != 1 && size != 5)
			return -EINVAL;
		if (size == 5)
			put_unaligned_le16(mock->readback_value, value + 3);
		mock->host_polls++;
		if (mock->stuck_command || mock->busy_reads) {
			if (mock->busy_reads)
				mock->busy_reads--;
			value[0] = mock->memory[address];
		} else {
			mock->memory[address] = 0;
		}
		break;
	case 0x11c478:
		if (size != 38)
			return -EINVAL;
		value[0] = 0x12;
		value[1] = mock->bad_info ? 0 : 0xed;
		break;
	case 0x1093d8:
		if (size != 39)
			return -EINVAL;
		put_unaligned_le16(5192, value + 2);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void nvt_test_vfree(void *data)
{
	vfree(data);
}

static void nvt_test_regmap_exit(void *data)
{
	regmap_exit(data);
}

static struct regmap *nvt_mock_map(struct kunit *test, struct nvt_mock *mock)
{
	const struct regmap_config config = {
		.reg_bits = 32,
		.val_bits = 8,
		.read = nvt_mock_read,
		.write = nvt_mock_write,
		.max_register = 0x7fffff,
		.max_raw_read = 8190,
		.max_raw_write = 63 * 1024,
		.cache_type = REGCACHE_NONE,
	};
	struct regmap *map;

	mock->memory = vzalloc(SZ_2M);
	KUNIT_ASSERT_NOT_NULL(test, mock->memory);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_vfree, mock->memory), 0);
	map = regmap_init(NULL, NULL, mock, &config);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, map);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nvt_test_regmap_exit, map), 0);
	return map;
}

static void nvt_test_layout(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_thp_firmware *image;
	unsigned int index, count = 0, total = 0;

	if (!fw)
		return;
	image = nvt_image(test, fw);
	KUNIT_ASSERT_EQ(test, image->count, 16U);
	KUNIT_EXPECT_EQ(test, image->partitions[0].offset, 0x200U);
	KUNIT_EXPECT_EQ(test, image->partitions[0].length, 104532U);
	KUNIT_EXPECT_EQ(test, image->partitions[1].length, 82088U);
	KUNIT_EXPECT_EQ(test, image->partitions[14].offset, 0U);
	KUNIT_EXPECT_EQ(test, image->partitions[15].offset, 0x100U);
	for (index = 0; index < image->count; ++index) {
		count += !!image->partitions[index].length;
		total += image->partitions[index].length;
	}
	KUNIT_EXPECT_EQ(test, count, 4U);
	KUNIT_EXPECT_EQ(test, total, 187132U);
}

static void nvt_test_bad_layout(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_thp_firmware image;
	u8 *data;
	static const struct {
		u32 offset, value;
		int error;
	} changes[] = {
		{ 0, 0x31, -EINVAL },
		{ 0, U32_MAX, -EINVAL },
		{ 8, U32_MAX, -EINVAL },
		{ 4, 0x800000, -EINVAL },
		{ 0xf4, 0, -EINVAL },
		{ 0xf8, U32_MAX, -EINVAL },
		{ 0x1f8, U32_MAX, -EINVAL },
		{ 0x20, 0x101, -EOPNOTSUPP },
		{ 0x28, 0x100, -EOPNOTSUPP },
	};
	unsigned int index;
	int ret;

	if (!fw)
		return;
	data = kunit_kmalloc(test, fw->size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, data);
	KUNIT_EXPECT_EQ(test, nvt_thp_parse_firmware(fw->data, SZ_4K - 1, &image), -EINVAL);
	for (index = 0; index < ARRAY_SIZE(changes); ++index) {
		memcpy(data, fw->data, fw->size);
		put_unaligned_le32(changes[index].value, data + changes[index].offset);
		ret = nvt_thp_parse_firmware(data, fw->size, &image);
		KUNIT_EXPECT_EQ_MSG(test, ret, changes[index].error, "case %u", index);
		nvt_thp_free_firmware(&image);
	}
	memset(data, 0, fw->size);
	KUNIT_EXPECT_EQ(test, nvt_thp_parse_firmware(data, fw->size, &image), -ENOEXEC);
}

static void nvt_test_upload(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_firmware *image;
	struct nvt_thp_boot_status boot;
	struct regmap *map;
	u16 length = 0;
	unsigned int index;

	if (!fw)
		return;
	KUNIT_ASSERT_NOT_NULL(test, mock);
	image = nvt_image(test, fw);
	map = nvt_mock_map(test, mock);
	/* INIT alone must not let boot finish before REK. */
	mock->reset_states[0] = NVT_RESET_STATE_INIT;
	mock->reset_states[1] = NVT_RESET_STATE_INIT;
	mock->reset_states[2] = NVT_RESET_STATE_REK;
	mock->reset_count = 3;
	KUNIT_ASSERT_EQ(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot), 0);
	KUNIT_EXPECT_EQ(test, mock->reset_polls, 3U);
	KUNIT_EXPECT_EQ(test, boot.reset_state, NVT_RESET_STATE_REK);
	KUNIT_EXPECT_GT(test, boot.reset_time, (ktime_t)0);
	KUNIT_EXPECT_GE(test, boot.rek_ms, (s64)10);
	KUNIT_EXPECT_EQ(test, length, 5192);
	KUNIT_EXPECT_EQ(test, mock->uploaded, 187132U);
	KUNIT_EXPECT_EQ(test, mock->resets, 2U);
	KUNIT_EXPECT_TRUE(test, mock->boot_ready);
	for (index = 0; index < image->count; ++index) {
		const struct nvt_thp_partition *part = &image->partitions[index];

		if (part->length)
			KUNIT_EXPECT_MEMEQ(test, mock->memory + part->address,
					   fw->data + part->offset, part->length);
	}
	for (index = 0; index < 2; ++index) {
		const struct nvt_thp_partition *part = &image->partitions[index];
		u32 length_address = index ? 0x1fb530 : 0x1fb518;

		KUNIT_EXPECT_EQ(test, get_unaligned_le24(mock->memory + 0x1fb528 + index * 4),
				part->address);
		KUNIT_EXPECT_EQ(test, get_unaligned_le24(mock->memory + length_address),
				part->length - 1);
		KUNIT_EXPECT_EQ(test, get_unaligned_le32(mock->memory + 0x1fb500 + index * 4),
				part->crc);
	}
}

static void nvt_test_io_errors(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_firmware *image;
	struct nvt_thp_boot_status boot;
	struct regmap *map;
	unsigned int calls, index;
	u16 length;

	if (!fw)
		return;
	KUNIT_ASSERT_NOT_NULL(test, mock);
	image = nvt_image(test, fw);
	map = nvt_mock_map(test, mock);
	KUNIT_ASSERT_EQ(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot), 0);
	calls = mock->calls;
	kunit_info(test, "injecting failure at each of %u register-bus calls", calls);
	for (index = 1; index <= calls; ++index) {
		mock->calls = 0;
		mock->fail_call = index;
		length = 0;
		KUNIT_EXPECT_EQ_MSG(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot),
				    -EREMOTEIO, "bus call %u", index);
		KUNIT_EXPECT_EQ(test, mock->calls, index);
		KUNIT_EXPECT_EQ(test, length, 0);
	}
}

static void nvt_test_identity(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_firmware *image;
	struct nvt_thp_boot_status boot;
	struct regmap *map;
	u16 length;

	if (!fw)
		return;
	KUNIT_ASSERT_NOT_NULL(test, mock);
	image = nvt_image(test, fw);
	map = nvt_mock_map(test, mock);
	mock->bad_id = true;
	KUNIT_EXPECT_EQ(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot), -ENODEV);
	KUNIT_EXPECT_EQ(test, mock->uploaded, 0U);
	KUNIT_EXPECT_FALSE(test, mock->boot_ready);
	mock->bad_id = false;
	mock->bad_info = true;
	KUNIT_EXPECT_EQ(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot), -EBADMSG);
}

static void nvt_test_rek_success(struct kunit *test)
{
	static const u8 ready[] = {
		NVT_RESET_STATE_REK, NVT_RESET_STATE_REK_FINISH,
		NVT_RESET_STATE_NORMAL_RUN, NVT_RESET_STATE_MAX,
	};
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct regmap *map;
	unsigned int index;
	ktime_t start;
	u8 state = 0;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	mock->boot_ready = true;
	mock->reset_count = 3;
	mock->reset_states[0] = 0;
	mock->reset_states[1] = NVT_RESET_STATE_INIT;
	for (index = 0; index < ARRAY_SIZE(ready); index++) {
		mock->reset_states[2] = ready[index];
		mock->reset_polls = 0;
		start = ktime_get();
		KUNIT_EXPECT_EQ(test, nvt_thp_wait_rek(map, &state), 0);
		KUNIT_EXPECT_EQ(test, state, ready[index]);
		KUNIT_EXPECT_EQ(test, mock->reset_polls, 3U);
		KUNIT_EXPECT_GE(test, ktime_ms_delta(ktime_get(), start), (s64)20);
	}
	/* Already ready: accept the first read without another poll. */
	mock->reset_polls = 0;
	mock->reset_count = 1;
	mock->reset_states[0] = NVT_RESET_STATE_REK;
	KUNIT_EXPECT_EQ(test, nvt_thp_wait_rek(map, &state), 0);
	KUNIT_EXPECT_EQ(test, mock->reset_polls, 1U);
}

static void nvt_test_rek_timeout(struct kunit *test)
{
	static const u8 not_ready[] = { 0, 0x9f, NVT_RESET_STATE_INIT, 0xb0 };
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct regmap *map;
	unsigned int index;
	ktime_t start;
	u8 state = 0;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	mock->boot_ready = true;
	mock->reset_count = 1;
	for (index = 0; index < ARRAY_SIZE(not_ready); index++) {
		mock->reset_states[0] = not_ready[index];
		mock->reset_polls = 0;
		start = ktime_get();
		KUNIT_EXPECT_EQ(test, nvt_thp_wait_rek(map, &state), -ETIMEDOUT);
		KUNIT_EXPECT_EQ(test, state, not_ready[index]);
		KUNIT_EXPECT_GE(test, ktime_ms_delta(ktime_get(), start), (s64)500);
		KUNIT_EXPECT_GE(test, mock->reset_polls, 2U);
		KUNIT_EXPECT_LE(test, mock->reset_polls, 51U);
	}
	KUNIT_EXPECT_EQ(test, mock->host_commands, 0U);
}

static void nvt_test_rek_io_error(struct kunit *test)
{
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct regmap *map;
	u8 state = 0;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	mock->boot_ready = true;
	mock->reset_count = 1;
	mock->reset_states[0] = NVT_RESET_STATE_INIT;
	mock->fail_call = 2;
	KUNIT_EXPECT_EQ(test, nvt_thp_wait_rek(map, &state), -EREMOTEIO);
	KUNIT_EXPECT_EQ(test, state, NVT_RESET_STATE_INIT);
	KUNIT_EXPECT_EQ(test, mock->calls, 2U);
	KUNIT_EXPECT_EQ(test, mock->reset_polls, 1U);
	KUNIT_EXPECT_EQ(test, mock->host_commands, 0U);
}

static void nvt_test_boot_rek_timeout(struct kunit *test)
{
	const struct firmware *fw = nvt_fixture(test);
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_firmware *image;
	struct nvt_thp_boot_status boot;
	struct regmap *map;
	u16 length = 0;

	if (!fw)
		return;
	KUNIT_ASSERT_NOT_NULL(test, mock);
	image = nvt_image(test, fw);
	map = nvt_mock_map(test, mock);
	mock->reset_count = 1;
	mock->reset_states[0] = NVT_RESET_STATE_INIT;
	KUNIT_EXPECT_EQ(test, nvt_thp_boot_firmware(map, fw, image, &length, &boot),
			-ETIMEDOUT);
	KUNIT_EXPECT_TRUE(test, mock->boot_ready);
	KUNIT_EXPECT_EQ(test, boot.reset_state, NVT_RESET_STATE_INIT);
	KUNIT_EXPECT_GE(test, boot.rek_ms, (s64)500);
	KUNIT_EXPECT_EQ(test, length, 0);
	KUNIT_EXPECT_EQ(test, mock->host_commands, 0U);
}

static void nvt_test_host_commands(struct kunit *test)
{
	static const u8 expected[3][5] = {
		{ 0xbf, 0x04, 0x00, 0x00, 0x00 },	/* no pen consumer: stock stylus disable */
		{ 0xbf, 0x19, 0x00, 0x01, 0x00 },	/* idle-baseline update */
		{ 0xbf, 0x01, 0x00, 0x02, 0x00 },	/* op mode 2 */
	};
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_command_status last;
	struct regmap *map;
	unsigned int calls, index;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	KUNIT_ASSERT_EQ(test, nvt_thp_apply_host_commands(map, &last), 0);
	KUNIT_EXPECT_EQ(test, last.sub_command, 0x01);
	KUNIT_EXPECT_EQ(test, last.status, 0);
	KUNIT_ASSERT_EQ(test, mock->host_commands, 3U);
	KUNIT_EXPECT_MEMEQ(test, mock->host_command, expected, sizeof(expected));
	/* Each command is polled while busy and once more after clearing. */
	KUNIT_EXPECT_EQ(test, mock->host_polls, 6U);
	KUNIT_EXPECT_EQ(test, mock->memory[0x11c450], 0);

	calls = mock->calls;
	for (index = 1; index <= calls; ++index) {
		mock->calls = 0;
		mock->fail_call = index;
		KUNIT_EXPECT_EQ_MSG(test, nvt_thp_apply_host_commands(map, &last), -EREMOTEIO,
				    "bus call %u", index);
		KUNIT_EXPECT_EQ(test, mock->calls, index);
	}
}

static void nvt_test_host_command_timeout(struct kunit *test)
{
	static const u8 expected[5] = { 0xbf, 0x04, 0x00, 0x00, 0x00 };
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_command_status last;
	struct regmap *map;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	mock->stuck_command = true;
	KUNIT_EXPECT_EQ(test, nvt_thp_apply_host_commands(map, &last), -ETIMEDOUT);
	/* The failing command and the byte it left behind are reported. */
	KUNIT_EXPECT_EQ(test, last.sub_command, 0x04);
	KUNIT_EXPECT_EQ(test, last.status, 0xbf);
	/* No later command is sent after the first one fails. */
	KUNIT_EXPECT_EQ(test, mock->host_commands, 1U);
	KUNIT_EXPECT_MEMEQ(test, mock->host_command[0], expected, sizeof(expected));
	KUNIT_EXPECT_GE(test, mock->host_polls, 2U);
	KUNIT_EXPECT_EQ(test, mock->memory[0x11c450], 0xbf);
}

static void nvt_test_get_extend_command(struct kunit *test)
{
	static const u8 expected[] = { 0xbf, 0x01, 0x01 };
	struct nvt_mock *mock = kunit_kzalloc(test, sizeof(*mock), GFP_KERNEL);
	struct nvt_thp_command_status last;
	struct regmap *map;
	u16 value = 0xffff;
	unsigned int calls, i;

	KUNIT_ASSERT_NOT_NULL(test, mock);
	map = nvt_mock_map(test, mock);
	mock->readback_value = 0x1234;
	KUNIT_ASSERT_EQ(test, nvt_thp_get_extend_command(map, 0x01, &value, &last), 0);
	KUNIT_EXPECT_EQ(test, value, 0x1234);
	KUNIT_EXPECT_EQ(test, last.sub_command, 0x01);
	KUNIT_EXPECT_EQ(test, last.status, 0);
	KUNIT_EXPECT_EQ(test, mock->host_commands, 1U);
	KUNIT_EXPECT_EQ(test, mock->host_polls, 2U);
	KUNIT_EXPECT_MEMEQ(test, mock->host_command[0], expected, sizeof(expected));

	calls = mock->calls;
	for (i = 1; i <= calls; i++) {
		mock->calls = 0;
		mock->fail_call = i;
		value = 0xffff;
		KUNIT_EXPECT_EQ(test, nvt_thp_get_extend_command(map, 0x19, &value, &last),
				-EREMOTEIO);
		KUNIT_EXPECT_EQ(test, mock->calls, i);
		KUNIT_EXPECT_EQ(test, value, 0xffff);
	}

	mock->fail_call = 0;
	mock->host_commands = 0;
	mock->host_polls = 0;
	mock->stuck_command = true;
	KUNIT_EXPECT_EQ(test, nvt_thp_get_extend_command(map, 0x04, &value, &last),
			-ETIMEDOUT);
	KUNIT_EXPECT_EQ(test, value, 0xffff);
	KUNIT_EXPECT_EQ(test, last.sub_command, 0x04);
	KUNIT_EXPECT_EQ(test, last.status, 0xbf);
	KUNIT_EXPECT_EQ(test, mock->host_commands, 1U);
	KUNIT_EXPECT_EQ(test, mock->host_polls, 11U);
}

static struct kunit_case nvt_cases[] = {
	KUNIT_CASE(nvt_test_layout),
	KUNIT_CASE(nvt_test_bad_layout),
	KUNIT_CASE(nvt_test_upload),
	KUNIT_CASE(nvt_test_io_errors),
	KUNIT_CASE(nvt_test_identity),
	KUNIT_CASE(nvt_test_rek_success),
	KUNIT_CASE(nvt_test_rek_timeout),
	KUNIT_CASE(nvt_test_rek_io_error),
	KUNIT_CASE(nvt_test_boot_rek_timeout),
	KUNIT_CASE(nvt_test_get_extend_command),
	KUNIT_CASE(nvt_test_host_commands),
	KUNIT_CASE(nvt_test_host_command_timeout),
	{}
};

static struct kunit_suite nvt_suite = {
	.name = "nt36532e-firmware",
	.test_cases = nvt_cases,
};

kunit_test_suite(nvt_suite);
