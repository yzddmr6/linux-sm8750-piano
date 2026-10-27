// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests for the IGCv5 hardware LUT packing and for the REGDMA
 * descriptor offset the shared command stream needs.
 *
 * Included by dpu_hw_igcv5.c to reach the packing helpers.
 */

#include <kunit/test.h>

#include "dpu_hw_gcv2.h"
#include "dpu_hw_igcv5.h"

#define IGCV5_TEST_WORDS 1024

/* The same opcodes the packer emits, spelled out for the expectations. */
#define TEST_REGDMA_V3_DECODE_SELECT 0x18180114u
#define TEST_REGDMA_SINGLE_WRITE BIT(28)
#define TEST_REGDMA_LUTBUS_WRITE (BIT(30) | BIT(29))
#define TEST_IGC_CONTROL_OFFSET 0x4
#define TEST_IGC_ENABLE BIT(8)
#define TEST_IGC_HIGH_PRECISION BIT(16)
#define TEST_IGC_TRANSFER_WORDS 3
#define TEST_IGC_TRANSFER_COUNT (DPU_IGCV5_DATA_WORDS / TEST_IGC_TRANSFER_WORDS)

/* A ramp LUT: every entry carries its own index in all three channels. */
static void igcv5_test_build_lut(struct drm_color_lut *lut)
{
	unsigned int i;

	for (i = 0; i < DPU_IGCV5_LUT_SIZE; i++) {
		lut[i].red = i;
		lut[i].green = i;
		lut[i].blue = i;
	}
}

/* The DRM entry a hardware point is taken from. */
static unsigned int igcv5_test_entry(unsigned int hw_entry)
{
	if (hw_entry >= DPU_IGCV5_HW_ENTRIES)
		return 0;
	if (hw_entry < DPU_IGCV5_MAIN_ENTRIES)
		return hw_entry * 4;
	return hw_entry - DPU_IGCV5_MAIN_ENTRIES;
}

/* The word holding the two components at component index @first. */
static u32 igcv5_test_word(const struct drm_color_lut *lut, unsigned int first)
{
	unsigned int e0 = igcv5_test_entry(first / 3);
	unsigned int e1 = igcv5_test_entry((first + 1) / 3);
	u32 lo, hi;

	switch (first % 3) {
	case 0:
		lo = lut[e0].red;
		break;
	case 1:
		lo = lut[e0].green;
		break;
	default:
		lo = lut[e0].blue;
		break;
	}

	switch ((first + 1) % 3) {
	case 0:
		hi = lut[e1].red;
		break;
	case 1:
		hi = lut[e1].green;
		break;
	default:
		hi = lut[e1].blue;
		break;
	}

	return lo | (hi << 16);
}

/* The packing must lay out the opcodes the REGDMA decoder expects. */
static void igcv5_test_opcodes(struct kunit *test)
{
	struct drm_color_lut *lut;
	__le32 words[DPU_IGCV5_LUT_WORDS];
	int n;

	lut = kunit_kzalloc(test, sizeof(*lut) * DPU_IGCV5_LUT_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	igcv5_test_build_lut(lut);

	n = dpu_hw_igcv5_pack_regdma(words, ARRAY_SIZE(words), BIT(17), lut);
	KUNIT_ASSERT_EQ(test, n, DPU_IGCV5_LUT_WORDS);

	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[0]), TEST_REGDMA_V3_DECODE_SELECT);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[1]), BIT(17));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[2]), TEST_REGDMA_LUTBUS_WRITE);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[3]),
			(TEST_IGC_TRANSFER_WORDS << 16) | TEST_IGC_TRANSFER_COUNT);

	/* The published LUTBUS workaround: three real NO_OP opcodes. */
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[4 + DPU_IGCV5_DATA_WORDS]), 0u);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[5 + DPU_IGCV5_DATA_WORDS]), 0u);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[6 + DPU_IGCV5_DATA_WORDS]), 0u);

	/* The block is enabled in the high precision mode. */
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[n - 2]),
			TEST_REGDMA_SINGLE_WRITE |
			(DPU_IGCV5_REG_BASE + TEST_IGC_CONTROL_OFFSET));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[n - 1]),
			TEST_IGC_ENABLE | TEST_IGC_HIGH_PRECISION);
}

/*
 * The hardware table has 257 main points in 1/256 steps and 128 dark
 * points in 1/1024 steps. The uniform 1025 point LUT is sampled from
 * both, so the main points come from every fourth entry and the dark
 * points from the first 128.
 */
static void igcv5_test_grid(struct kunit *test)
{
	struct drm_color_lut *lut;
	__le32 words[DPU_IGCV5_LUT_WORDS];
	unsigned int hw, word, comp;

	lut = kunit_kzalloc(test, sizeof(*lut) * DPU_IGCV5_LUT_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, lut);
	igcv5_test_build_lut(lut);

	KUNIT_ASSERT_EQ(test, dpu_hw_igcv5_pack_regdma(words, ARRAY_SIZE(words),
						       BIT(17), lut),
			DPU_IGCV5_LUT_WORDS);

	/* Walk the hardware points and check the word each one lands in. */
	for (hw = 0; hw < DPU_IGCV5_HW_ENTRIES; hw++) {
		unsigned int entry = igcv5_test_entry(hw);

		for (comp = 0; comp < 3; comp++) {
			unsigned int index = hw * 3 + comp;

			word = 4 + index / 2;
			KUNIT_EXPECT_EQ(test, le32_to_cpu(words[word]) & 0xffff,
					entry & 0xffff);
			if (index % 2 == 0)
				continue;
			KUNIT_EXPECT_EQ(test, le32_to_cpu(words[word]) >> 16,
					entry & 0xffff);
		}
	}

	/* The first word holds the first entry in red and green. */
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[4]) & 0xffff, 0u);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[4]) >> 16, 0u);

	/* The second word crosses into the fourth entry. */
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[5]) & 0xffff, 0u);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[5]) >> 16, 4u);
}

/* Without a LUT the block is only switched off again. */
static void igcv5_test_disable(struct kunit *test)
{
	__le32 words[8];
	int n;

	n = dpu_hw_igcv5_pack_regdma(words, ARRAY_SIZE(words), BIT(17), NULL);
	KUNIT_ASSERT_EQ(test, n, 4);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[0]), TEST_REGDMA_V3_DECODE_SELECT);
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[1]), BIT(17));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[2]),
			TEST_REGDMA_SINGLE_WRITE |
			(DPU_IGCV5_REG_BASE + TEST_IGC_CONTROL_OFFSET));
	KUNIT_EXPECT_EQ(test, le32_to_cpu(words[3]), 0u);
}

/* A short buffer or an impossible DSPP mask must be refused. */
static void igcv5_test_arguments(struct kunit *test)
{
	__le32 words[DPU_IGCV5_LUT_WORDS];

	KUNIT_EXPECT_EQ(test, dpu_hw_igcv5_pack_regdma(NULL, ARRAY_SIZE(words),
						       BIT(17), NULL),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dpu_hw_igcv5_pack_regdma(words, ARRAY_SIZE(words),
						       0, NULL),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dpu_hw_igcv5_pack_regdma(words, ARRAY_SIZE(words),
						       ~DPU_GCV2_DSPP_MASK, NULL),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dpu_hw_igcv5_pack_regdma(words, 3, BIT(17), NULL),
			-ENOSPC);
}

/*
 * The trailing descriptor must follow the command stream. Both the GC
 * and the IGC table travel in one submission, so the offset has to be
 * derived from the command length.
 */
static void igcv5_test_last_offset(struct kunit *test)
{
	size_t offset = 0;

	/* The GC table alone keeps the offset the driver used before. */
	KUNIT_ASSERT_EQ(test, dpu_hw_color_regdma_last_offset(DPU_GCV2_LUT_WORDS,
							      SZ_16K, &offset),
			0);
	KUNIT_EXPECT_EQ(test, offset, 6400);

	/* GC and IGC together need the larger buffer. */
	KUNIT_ASSERT_EQ(test, dpu_hw_color_regdma_last_offset(DPU_GCV2_LUT_WORDS +
							      DPU_IGCV5_LUT_WORDS,
							      SZ_16K, &offset),
			0);
	KUNIT_EXPECT_EQ(test, offset, 8704);

	/* The 8K buffer cannot hold both tables. */
	KUNIT_EXPECT_EQ(test, dpu_hw_color_regdma_last_offset(DPU_GCV2_LUT_WORDS +
							      DPU_IGCV5_LUT_WORDS,
							      SZ_8K, &offset),
			-ENOSPC);

	/* Nonsensical arguments are refused. */
	KUNIT_EXPECT_EQ(test, dpu_hw_color_regdma_last_offset(0, SZ_16K, &offset),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dpu_hw_color_regdma_last_offset(16, SZ_16K, NULL),
			-EINVAL);
	KUNIT_EXPECT_EQ(test, dpu_hw_color_regdma_last_offset(0x4000, SZ_16K,
							      &offset),
			-EINVAL);
}

static struct kunit_case igcv5_test_cases[] = {
	KUNIT_CASE(igcv5_test_opcodes),
	KUNIT_CASE(igcv5_test_grid),
	KUNIT_CASE(igcv5_test_disable),
	KUNIT_CASE(igcv5_test_arguments),
	KUNIT_CASE(igcv5_test_last_offset),
	{}
};

static struct kunit_suite igcv5_test_suite = {
	.name = "dpu-igcv5",
	.test_cases = igcv5_test_cases,
};

kunit_test_suite(igcv5_test_suite);
