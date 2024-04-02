/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio/gpio_mmio32.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int ensemble_e3_dk_rtss_hp_init(void)
{
	/* Enable UART clock and clock selection bits in CFGMST0 */
	sys_write32(0xFFFF, 0x4902F008);
	/* Set PLL clock of 160 MHz */
	sys_write32(0x10000, 0x4903F00C);

	return 0;
}

SYS_INIT(ensemble_e3_dk_rtss_hp_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
