// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/device.h>
#include <kunit/test.h>
#include <linux/firmware.h>
#include <linux/module.h>

#include "hid-nanosic-protocol.h"

static void nanosic_test_release_firmware(void *data)
{
	release_firmware(data);
}

static const struct firmware *nanosic_test_firmware(struct kunit *test)
{
	const struct firmware *firmware;
	struct device *dev;
	int ret;

	dev = kunit_device_register(test, "nanosic-protocol-test");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, dev);
	ret = request_firmware_direct(&firmware, "nanosic/piano-protocol-test.bin", dev);
	if (ret == -ENOENT) {
		kunit_skip(test, "optional private piano stock firmware fixture not supplied");
		return NULL;
	}
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, nanosic_test_release_firmware,
						     (void *)firmware), 0);
	return firmware;
}

static void nanosic_test_stock_firmware(struct kunit *test)
{
	static const u8 header_prefix[] = {
		0x06, 0x4e, 0x41, 0x4e, 0x4f, 0x20, 0x49, 0x43,
		0x40, 0x81, 0x45, 0x23, 0x00, 0x00, 0x40, 0x00,
		0x00, 0x00, 0xec, 0xc5, 0x00, 0x00, 0x44, 0x00,
		0xff, 0x1f, 0x94, 0x12, 0x06, 0x32, 0x06, 0x02,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	const struct firmware *fixture = nanosic_test_firmware(test);
	struct nanosic_firmware fw;
	u8 header[NANOSIC_FW_HEADER_CMD_SIZE];
	u8 chunk[NANOSIC_FW_DATA_CMD_SIZE];
	u32 offset = 0, packets = 0;
	int count = 0;

	if (!fixture)
		return;
	KUNIT_ASSERT_EQ(test, fixture->size, (size_t)83436);
	KUNIT_ASSERT_EQ(test, nanosic_firmware_parse(fixture->data, fixture->size, &fw), 0);
	KUNIT_EXPECT_EQ(test, fw.size, 50668U);
	KUNIT_EXPECT_EQ(test, fw.word_sum, 0x197d2249U);
	nanosic_firmware_header(&fw, header);
	KUNIT_EXPECT_MEMEQ(test, header, header_prefix, sizeof(header_prefix));
	KUNIT_EXPECT_EQ(test, header[64], 0x3b);
	KUNIT_EXPECT_EQ(test, get_unaligned_le32(header + 53), fw.word_sum);

	while (offset < fw.size) {
		count = nanosic_firmware_chunk(&fw, offset, chunk);
		KUNIT_ASSERT_GT(test, count, 0);
		KUNIT_EXPECT_EQ(test, chunk[0], 0x07);
		KUNIT_EXPECT_EQ(test, get_unaligned_be32(chunk + 1), offset);
		KUNIT_EXPECT_EQ(test, get_unaligned_be32(chunk + 5), count);
		KUNIT_EXPECT_MEMEQ(test, chunk + 9, fw.code + offset, count);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(chunk + 9 + count, 0,
						   sizeof(chunk) - 9 - count), NULL);
		offset += count;
		packets++;
	}
	KUNIT_EXPECT_EQ(test, packets, 101U);
	KUNIT_EXPECT_EQ(test, count, 368);
	KUNIT_EXPECT_EQ(test, nanosic_firmware_chunk(&fw, fw.size, chunk), -EINVAL);
}

static void nanosic_test_bad_firmware(struct kunit *test)
{
	const struct firmware *fixture = nanosic_test_firmware(test);
	struct nanosic_firmware fw;
	u8 *data;

	if (!fixture)
		return;
	data = kunit_kmalloc(test, fixture->size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, data);
	memcpy(data, fixture->data, fixture->size);
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, 0x7fff, &fw), -EINVAL);
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size - 1, &fw), -EINVAL);

	data[0x7fc0] ^= 1;
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size, &fw), -EINVAL);
	data[0x7fc0] ^= 1;
	data[0x7fff] ^= 1;
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size, &fw), -EBADMSG);
	data[0x7fff] ^= 1;
	data[0x8000] ^= 1;
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size, &fw), -EBADMSG);
	data[0x8000] ^= 1;
	put_unaligned_le32(0xfffffffc, data + 0x7fc0 + 48);
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size, &fw), -EINVAL);
	put_unaligned_le32(50667, data + 0x7fc0 + 48);
	KUNIT_EXPECT_EQ(test, nanosic_firmware_parse(data, fixture->size, &fw), -EINVAL);
}

static void nanosic_test_reports(struct kunit *test)
{
	u8 frame[NANOSIC_FRAME_SIZE] = { 0x57, 0, 0x39 };
	struct nanosic_report report;
	size_t cursor = 0;

	frame[3] = 0x19;
	frame[30] = 0x06;
	frame[35] = 0x05;
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_PTR_EQ(test, report.data, frame + 3);
	KUNIT_EXPECT_EQ(test, report.wire_size, 27);
	KUNIT_EXPECT_EQ(test, report.hid_size, 27);
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_EQ(test, report.wire_size, 5);
	KUNIT_EXPECT_EQ(test, report.hid_size, 3);
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_PTR_EQ(test, report.data, frame + 35);
	KUNIT_EXPECT_EQ(test, report.hid_size, 9);
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 0);
}

static void nanosic_test_vendor_and_mouse(struct kunit *test)
{
	u8 frame[NANOSIC_FRAME_SIZE] = { 0xd7, 0, 0x4a };
	struct nanosic_report report;
	size_t cursor = 0;

	frame[3] = 0x23;
	frame[35] = 0x22;
	frame[51] = 0x02;
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_EQ(test, report.wire_size, 32);
	KUNIT_EXPECT_EQ(test, report.hid_size, 0);
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_EQ(test, report.wire_size, 16);
	KUNIT_EXPECT_EQ(test, report.hid_size, 0);
	KUNIT_ASSERT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 1);
	KUNIT_EXPECT_EQ(test, report.hid_size, 8);
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 0);
}

static void nanosic_test_bad_frames(struct kunit *test)
{
	u8 frame[NANOSIC_FRAME_SIZE] = { 0x57, 0, 0x39, 0x19 };
	struct nanosic_report report;
	size_t cursor = 0;

	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, 29, &cursor, &report), -EMSGSIZE);
	KUNIT_EXPECT_EQ(test, cursor, (size_t)0);
	frame[3] = 0x27;
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report),
			-EOPNOTSUPP);
	frame[0] = 0;
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report),
			-EBADMSG);
	frame[0] = 0x57;
	frame[2] = 0x01;
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report),
			-EBADMSG);
	frame[2] = 0;
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, sizeof(frame), &cursor, &report), 0);
	KUNIT_EXPECT_EQ(test, nanosic_frame_next(frame, 2, &cursor, &report), -EMSGSIZE);
}

static struct kunit_case nanosic_protocol_cases[] = {
	KUNIT_CASE(nanosic_test_stock_firmware),
	KUNIT_CASE(nanosic_test_bad_firmware),
	KUNIT_CASE(nanosic_test_reports),
	KUNIT_CASE(nanosic_test_vendor_and_mouse),
	KUNIT_CASE(nanosic_test_bad_frames),
	{}
};

static struct kunit_suite nanosic_protocol_suite = {
	.name = "nanosic-protocol",
	.test_cases = nanosic_protocol_cases,
};
kunit_test_suite(nanosic_protocol_suite);

MODULE_LICENSE("GPL");
