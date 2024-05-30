/*
 * Copyright (c) 2024 Alif Semiconductor.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/slist.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>

#define ALIF_ENSEMBLE_OSPI_REG  0x83000000
#define ALIF_ENSEMBLE_OSPI_SIZE KB(16)

#define MPU_MAIR_INDEX_DEVICE 3

#define REGION_DEVICE_ATTR(limit)                                                                  \
	{                                                                                          \
		/* AP, XN, SH */                                                                   \
		.rbar = NOT_EXEC | P_RW_U_NA_Msk | NON_SHAREABLE_Msk, /* Cache-ability */          \
			.mair_idx = MPU_MAIR_INDEX_DEVICE,            /* Region Limit */           \
			.r_limit = limit - 1,                                                      \
	}

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0", CONFIG_FLASH_BASE_ADDRESS,
			 REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, CONFIG_FLASH_SIZE * 1024)),
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0", CONFIG_SRAM_BASE_ADDRESS,
			 REGION_RAM_ATTR(CONFIG_SRAM_BASE_ADDRESS, CONFIG_SRAM_SIZE * 1024)),
	/* Region 2 */
	MPU_REGION_ENTRY("OSPI_CTRL", ALIF_ENSEMBLE_OSPI_REG,
			 REGION_DEVICE_ATTR(ALIF_ENSEMBLE_OSPI_REG + ALIF_ENSEMBLE_OSPI_SIZE - 1)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
