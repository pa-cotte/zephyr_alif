/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/gpio/gpio_mmio32.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/linker/linker-defs.h>
#ifdef CONFIG_REBOOT
#include <zephyr/sys/reboot.h>
#include <se_service.h>
#endif

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * @return 0
 */
static int ensemble_e1c_dk_rtss_he_init(void)
{
	/* enable all UART[5-0] modules */
	/* select UART[5-0]_SCLK as SYST_PCLK clock. */
	sys_write32(0xFFFF, UART_CLK_EN);

	/* LPUART settings */
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPUART_CKEN);
	}

	/* RTC Clk Enable */
	sys_write32(0x1, LPRTC0_CLK_EN);
	sys_write32(0x1, LPRTC1_CLK_EN);

	return 0;
}

#ifdef CONFIG_REBOOT
void sys_arch_reboot(int type)
{
	switch (type) {
	case SYS_REBOOT_WARM:
		/* Use Cold boot until NVIC reset is fully working */
		/* se_service_boot_reset_cpu(EXTSYS_1); */
		se_service_boot_reset_soc();
		break;
	case SYS_REBOOT_COLD:
		se_service_boot_reset_soc();
		break;

	default:
		/* Do nothing */
		break;
	}
}
#endif

SYS_INIT(ensemble_e1c_dk_rtss_he_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
