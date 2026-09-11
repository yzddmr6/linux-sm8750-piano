/*
 * Copyright (C) 2010 - 2022 Novatek, Inc.
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 * $Revision: 97627 $
 * $Date: 2022-04-06 15:02:09 +0800 (週三, 06 四月 2022) $
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


typedef struct nvt_ts_reg {
	uint32_t addr; /* byte in which address */
	uint8_t mask; /* in which bits of that byte */
} nvt_ts_reg_t;

struct nvt_ts_mem_map {
	uint32_t EVENT_BUF_ADDR;
	uint32_t RAW_PIPE0_ADDR;
	uint32_t RAW_PIPE1_ADDR;
	uint32_t BASELINE_ADDR;
	uint32_t BASELINE_BTN_ADDR;
	uint32_t DIFF_PIPE0_ADDR;
	uint32_t DIFF_PIPE1_ADDR;
	uint32_t RAW_BTN_PIPE0_ADDR;
	uint32_t RAW_BTN_PIPE1_ADDR;
	uint32_t DIFF_BTN_PIPE0_ADDR;
	uint32_t DIFF_BTN_PIPE1_ADDR;
	uint32_t PEN_2D_BL_TIP_X_ADDR;
	uint32_t PEN_2D_BL_TIP_Y_ADDR;
	uint32_t PEN_2D_BL_RING_X_ADDR;
	uint32_t PEN_2D_BL_RING_Y_ADDR;
	uint32_t PEN_2D_DIFF_TIP_X_ADDR;
	uint32_t PEN_2D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_2D_DIFF_RING_X_ADDR;
	uint32_t PEN_2D_DIFF_RING_Y_ADDR;
	uint32_t PEN_2D_RAW_TIP_X_ADDR;
	uint32_t PEN_2D_RAW_TIP_Y_ADDR;
	uint32_t PEN_2D_RAW_RING_X_ADDR;
	uint32_t PEN_2D_RAW_RING_Y_ADDR;
	uint32_t PEN_1D_DIFF_TIP_X_ADDR;
	uint32_t PEN_1D_DIFF_TIP_Y_ADDR;
	uint32_t PEN_1D_DIFF_RING_X_ADDR;
	uint32_t PEN_1D_DIFF_RING_Y_ADDR;
	nvt_ts_reg_t ENB_CASC_REG;
	/* FW History */
	uint32_t MMAP_HISTORY_EVENT0;
	uint32_t MMAP_HISTORY_EVENT1;
	/* START for flash FW update */
	uint32_t READ_FLASH_CHECKSUM_ADDR;
	uint32_t RW_FLASH_DATA_ADDR;
	/* END for FW Update Use */
	/* Phase 2 Host Download */
	uint32_t BOOT_RDY_ADDR;
	uint32_t POR_CD_ADDR;
	uint32_t TX_AUTO_COPY_EN;
	uint32_t SPI_DMA_TX_INFO;
	/* BLD CRC */
	uint32_t BLD_LENGTH_ADDR;
	uint32_t ILM_LENGTH_ADDR;
	uint32_t DLM_LENGTH_ADDR;
	uint32_t BLD_DES_ADDR;
	uint32_t ILM_DES_ADDR;
	uint32_t DLM_DES_ADDR;
	uint32_t G_ILM_CHECKSUM_ADDR;
	uint32_t G_DLM_CHECKSUM_ADDR;
	uint32_t R_ILM_CHECKSUM_ADDR;
	uint32_t R_DLM_CHECKSUM_ADDR;
	uint32_t DMA_CRC_EN_ADDR;
	uint32_t BLD_ILM_DLM_CRC_ADDR;
	uint32_t DMA_CRC_FLAG_ADDR;
};

struct nvt_ts_hw_info {
	uint8_t hw_crc;
	uint8_t auto_copy;
	bool use_gcm;
	const struct nvt_ts_hw_reg_addr_info *hw_regs;
};

typedef enum {
	HWCRC_NOSUPPORT  = 0x00,
	HWCRC_LEN_2Bytes = 0x01,
	HWCRC_LEN_3Bytes = 0x02,
} HWCRCBankConfig;

typedef enum {
	AUTOCOPY_NOSUPPORT    = 0x00,
	CHECK_SPI_DMA_TX_INFO = 0x01,
	CHECK_TX_AUTO_COPY_EN = 0x02,
} AUTOCOPYCheck;

/* hw info reg*/
struct nvt_ts_hw_reg_addr_info {
	uint32_t chip_ver_trim_addr;
	uint32_t swrst_sif_addr;
	uint32_t crc_err_flag_addr;
};

static const struct nvt_ts_hw_reg_addr_info hw_reg_addr_info = {
	.chip_ver_trim_addr = 0x1FB104,
	.swrst_sif_addr = 0x1FB43E,
	.crc_err_flag_addr = 0x1FB535,
};

/*
 * NT36536 memory maps, extracted from the vendor module
 * (oplus_bsp_tp_nt36536_noflash.ko .rodata NT36536_single/cascade_memory_map)
 * and verified field by field against its disassembly.
 */
static const struct nvt_ts_mem_map NT36536_cascade_memory_map = {
	.EVENT_BUF_ADDR           = 0x130900,
	.RAW_PIPE0_ADDR           = 0x10BAA0,
	.RAW_PIPE1_ADDR           = 0x10BAA0,
	.BASELINE_ADDR            = 0x109FA0,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x127E70,
	.DIFF_PIPE1_ADDR          = 0x129970,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x110BA0,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x111020,
	.PEN_2D_BL_RING_X_ADDR    = 0x1114A0,
	.PEN_2D_BL_RING_Y_ADDR    = 0x111920,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x1126A0,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x112B20,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x112FA0,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x113420,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x130B00,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x130F90,
	.PEN_2D_RAW_RING_X_ADDR   = 0x131420,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x1318B0,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x131D40,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x131DD0,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x131E60,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x131EF0,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* FW History */
	.MMAP_HISTORY_EVENT0      = 0x124D7C,
	.MMAP_HISTORY_EVENT1      = 0x124DBC,
	/* START for flash FW update */
	.READ_FLASH_CHECKSUM_ADDR = 0x124E00,
	.RW_FLASH_DATA_ADDR       = 0x124E40,
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.POR_CD_ADDR              = 0x1FB605,
	.TX_AUTO_COPY_EN          = 0x1FC825,
	.SPI_DMA_TX_INFO          = 0x1FC814,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB518,
	.ILM_LENGTH_ADDR          = 0x1FCC34,
	.DLM_LENGTH_ADDR          = 0x1FCC44,
	.BLD_DES_ADDR             = 0x1FB514,
	.ILM_DES_ADDR             = 0x1FCC30,
	.DLM_DES_ADDR             = 0x1FCC40,
	.G_ILM_CHECKSUM_ADDR      = 0x1FCC38,
	.G_DLM_CHECKSUM_ADDR      = 0x1FCC48,
	.R_ILM_CHECKSUM_ADDR      = 0x1FCC7C,
	.R_DLM_CHECKSUM_ADDR      = 0x1FCC80,
	.DMA_CRC_EN_ADDR          = 0x1FB51B,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB532,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

static const struct nvt_ts_mem_map NT36536_single_memory_map = {
	.EVENT_BUF_ADDR           = 0x130900,
	.RAW_PIPE0_ADDR           = 0x10D748,
	.RAW_PIPE1_ADDR           = 0x10D748,
	.BASELINE_ADDR            = 0x10C348,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x11DB50,
	.DIFF_PIPE1_ADDR          = 0x11EF50,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x121750,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x121990,
	.PEN_2D_BL_RING_X_ADDR    = 0x121E10,
	.PEN_2D_BL_RING_Y_ADDR    = 0x122050,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x110B14,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x110D54,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x1111D4,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x111414,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x111AD4,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x111D14,
	.PEN_2D_RAW_RING_X_ADDR   = 0x112194,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x1123D4,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x130B00,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x130B90,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x130C20,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x130CB0,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* FW History */
	.MMAP_HISTORY_EVENT0      = 0x12CB2C,
	.MMAP_HISTORY_EVENT1      = 0x12CB6C,
	/* START for flash FW update */
	.READ_FLASH_CHECKSUM_ADDR = 0,
	.RW_FLASH_DATA_ADDR       = 0,
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.POR_CD_ADDR              = 0x1FB605,
	.TX_AUTO_COPY_EN          = 0x1FC825,
	.SPI_DMA_TX_INFO          = 0x1FC814,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB518,
	.ILM_LENGTH_ADDR          = 0x1FCC34,
	.DLM_LENGTH_ADDR          = 0x1FCC44,
	.BLD_DES_ADDR             = 0x1FB514,
	.ILM_DES_ADDR             = 0x1FCC30,
	.DLM_DES_ADDR             = 0x1FCC40,
	.G_ILM_CHECKSUM_ADDR      = 0x1FCC38,
	.G_DLM_CHECKSUM_ADDR      = 0x1FCC48,
	.R_ILM_CHECKSUM_ADDR      = 0x1FCC7C,
	.R_DLM_CHECKSUM_ADDR      = 0x1FCC80,
	.DMA_CRC_EN_ADDR          = 0x1FB51B,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB532,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

/* tddi */
static const struct nvt_ts_mem_map NT36532_cascade_memory_map = {
	.EVENT_BUF_ADDR           = 0x125800,
	.RAW_PIPE0_ADDR           = 0x10B200,
	.RAW_PIPE1_ADDR           = 0x10B200,
	.BASELINE_ADDR            = 0x109E00,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x128140,
	.DIFF_PIPE1_ADDR          = 0x129540,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x10F940,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x10FD40,
	.PEN_2D_BL_RING_X_ADDR    = 0x110140,
	.PEN_2D_BL_RING_Y_ADDR    = 0x110540,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x111140,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x111540,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x111940,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x111D40,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x126E00,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x127210,
	.PEN_2D_RAW_RING_X_ADDR   = 0x127620,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x127A30,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x127E40,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x127EC0,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x127F40,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x127FC0,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* FW History */
	.MMAP_HISTORY_EVENT0      = 0x121AFC,
	.MMAP_HISTORY_EVENT1      = 0x121B3C,
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.TX_AUTO_COPY_EN          = 0x1FC925,
	.SPI_DMA_TX_INFO          = 0x1FC914,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB538,	//0x1FB538 ~ 0x1FB53A (3 bytes)
	.ILM_LENGTH_ADDR          = 0x1FB518,	//0x1FB518 ~ 0x1FB51A (3 bytes)
	.DLM_LENGTH_ADDR          = 0x1FB530,	//0x1FB530 ~ 0x1FB532 (3 bytes)
	.BLD_DES_ADDR             = 0x1FB514,	//0x1FB514 ~ 0x1FB516 (3 bytes)
	.ILM_DES_ADDR             = 0x1FB528,	//0x1FB528 ~ 0x1FB52A (3 bytes)
	.DLM_DES_ADDR             = 0x1FB52C,	//0x1FB52C ~ 0x1FB52E (3 bytes)
	.G_ILM_CHECKSUM_ADDR      = 0x1FB500,   //0x1FB500 ~ 0x1FB503 (4 bytes)
	.G_DLM_CHECKSUM_ADDR      = 0x1FB504,	//0x1FB504 ~ 0x1FB507 (4 bytes)
	.R_ILM_CHECKSUM_ADDR      = 0x1FB520,	//0x1FB520 ~ 0x1FB523 (4 bytes)
	.R_DLM_CHECKSUM_ADDR      = 0x1FB524,	//0x1FB524 ~ 0x1FB527 (4 bytes)
	.DMA_CRC_EN_ADDR          = 0x1FB536,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB533,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

static const struct nvt_ts_mem_map NT36532_single_memory_map = {
	.EVENT_BUF_ADDR           = 0x125800,
	.RAW_PIPE0_ADDR           = 0x10C1EC,
	.RAW_PIPE1_ADDR           = 0x10C1EC,
	.BASELINE_ADDR            = 0x10B1EC,
	.BASELINE_BTN_ADDR        = 0,
	.DIFF_PIPE0_ADDR          = 0x11BFC4,
	.DIFF_PIPE1_ADDR          = 0x11CFC4,
	.RAW_BTN_PIPE0_ADDR       = 0,
	.RAW_BTN_PIPE1_ADDR       = 0,
	.DIFF_BTN_PIPE0_ADDR      = 0,
	.DIFF_BTN_PIPE1_ADDR      = 0,
	.PEN_2D_BL_TIP_X_ADDR     = 0x11E5EC,
	.PEN_2D_BL_TIP_Y_ADDR     = 0x126C00,
	.PEN_2D_BL_RING_X_ADDR    = 0x11E7EC,
	.PEN_2D_BL_RING_Y_ADDR    = 0x126E00,
	.PEN_2D_DIFF_TIP_X_ADDR   = 0x11E1EC,
	.PEN_2D_DIFF_TIP_Y_ADDR   = 0x11E9EC,
	.PEN_2D_DIFF_RING_X_ADDR  = 0x11E3EC,
	.PEN_2D_DIFF_RING_Y_ADDR  = 0x11EBEC,
	.PEN_2D_RAW_TIP_X_ADDR    = 0x120CEC,
	.PEN_2D_RAW_TIP_Y_ADDR    = 0x120EFC,
	.PEN_2D_RAW_RING_X_ADDR   = 0x12110C,
	.PEN_2D_RAW_RING_Y_ADDR   = 0x12131C,
	.PEN_1D_DIFF_TIP_X_ADDR   = 0x126A00,
	.PEN_1D_DIFF_TIP_Y_ADDR   = 0x126A80,
	.PEN_1D_DIFF_RING_X_ADDR  = 0x126B00,
	.PEN_1D_DIFF_RING_Y_ADDR  = 0x126B80,
	.ENB_CASC_REG             = {.addr = 0x1FB12C, .mask = 0x01},
	/* FW History */
	.MMAP_HISTORY_EVENT0      = 0x122930,
	.MMAP_HISTORY_EVENT1      = 0x122970,
	/* Phase 2 Host Download */
	.BOOT_RDY_ADDR            = 0x1FB50D,
	.TX_AUTO_COPY_EN          = 0x1FC925,
	.SPI_DMA_TX_INFO          = 0x1FC914,
	/* BLD CRC */
	.BLD_LENGTH_ADDR          = 0x1FB538,	//0x1FB538 ~ 0x1FB53A (3 bytes)
	.ILM_LENGTH_ADDR          = 0x1FB518,	//0x1FB518 ~ 0x1FB51A (3 bytes)
	.DLM_LENGTH_ADDR          = 0x1FB530,	//0x1FB530 ~ 0x1FB532 (3 bytes)
	.BLD_DES_ADDR             = 0x1FB514,	//0x1FB514 ~ 0x1FB516 (3 bytes)
	.ILM_DES_ADDR             = 0x1FB528,	//0x1FB528 ~ 0x1FB52A (3 bytes)
	.DLM_DES_ADDR             = 0x1FB52C,	//0x1FB52C ~ 0x1FB52E (3 bytes)
	.G_ILM_CHECKSUM_ADDR      = 0x1FB500,   //0x1FB500 ~ 0x1FB503 (4 bytes)
	.G_DLM_CHECKSUM_ADDR      = 0x1FB504,	//0x1FB504 ~ 0x1FB507 (4 bytes)
	.R_ILM_CHECKSUM_ADDR      = 0x1FB520,	//0x1FB520 ~ 0x1FB523 (4 bytes)
	.R_DLM_CHECKSUM_ADDR      = 0x1FB524,	//0x1FB524 ~ 0x1FB527 (4 bytes)
	.DMA_CRC_EN_ADDR          = 0x1FB536,
	.BLD_ILM_DLM_CRC_ADDR     = 0x1FB533,
	.DMA_CRC_FLAG_ADDR        = 0x1FB534,
};

static struct nvt_ts_hw_info NT36536_hw_info = {
	.hw_crc    = HWCRC_LEN_3Bytes,
	.auto_copy = CHECK_TX_AUTO_COPY_EN,
	.use_gcm   = 1,
	.hw_regs   = &hw_reg_addr_info,
};

static struct nvt_ts_hw_info NT36532_hw_info = {
	.hw_crc    = HWCRC_LEN_3Bytes,
	.auto_copy = CHECK_TX_AUTO_COPY_EN,
	.use_gcm   = 0,
	.hw_regs   = &hw_reg_addr_info,
};

#define NVT_ID_BYTE_MAX 6
struct nvt_ts_trim_id_table {
	uint8_t id[NVT_ID_BYTE_MAX];
	uint8_t mask[NVT_ID_BYTE_MAX];
	const struct nvt_ts_mem_map *mmap;
	const struct nvt_ts_mem_map *mmap_casc;
	const struct nvt_ts_hw_info *hwinfo;
};

/*
 * Match order and id/mask values follow the vendor module trim_id_table.
 * NT36536 entries come first so an NT36532 id never shadows them.
 */
static const struct nvt_ts_trim_id_table trim_id_table[] = {
	{.id = {0x14, 0xFF, 0xFF, 0x23, 0x65, 0x03}, .mask = {1, 0, 0, 1, 1, 1},
		.mmap = &NT36536_single_memory_map, .mmap_casc = &NT36536_cascade_memory_map, .hwinfo = &NT36536_hw_info},
	{.id = {0x15, 0x0E, 0xFF, 0x23, 0x65, 0x03}, .mask = {1, 1, 0, 1, 1, 1},
		.mmap = &NT36536_single_memory_map, .mmap_casc = &NT36536_cascade_memory_map, .hwinfo = &NT36536_hw_info},
	{.id = {0xFF, 0xFF, 0xFF, 0x32, 0x65, 0x03}, .mask = {0, 0, 0, 1, 1, 1},
		.mmap = &NT36532_single_memory_map, .mmap_casc = &NT36532_cascade_memory_map, .hwinfo = &NT36532_hw_info},
};
