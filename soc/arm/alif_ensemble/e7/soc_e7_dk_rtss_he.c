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
static int ensemble_e7_dk_rtss_he_init(void)
{
	unsigned int data;

	/* Might need to move later.. Just putting this here for now..*/
	/* Enable UART clock and clock selection bits in CFGMST0 */
	sys_write32(0xFFFF, 0x4902F008);
	/* Set PLL clock of 160 MHz */
	sys_write32(0x10000, 0x4903F00C);

	/* CGU_UART_CLK source to PLL */
	data =  sys_read32(0x1A602008);
	data |= 1U << 8;
	sys_write32(data, 0x1A602008);

	/* Enable CGU_UART_CLK */
	data =  sys_read32(0x1A602014);
	data |= 1U << 17;
	sys_write32(data, 0x1A602014);

	/* Switch HOSTUARTCLK Source to CGU_UART_CLK */
	sys_write32(2, 0x1A010850);

	/*
	 * Setting expansion master0 control register value for enabling clock
	 */
	data |= 0xc0000000;
	sys_write32(data, 0x4902F000);

	data = sys_read32(0x43007010);
	data |= (1 << 16);
	sys_write32(data, 0x43007010);

	return 0;
}

SYS_INIT(ensemble_e7_dk_rtss_he_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
