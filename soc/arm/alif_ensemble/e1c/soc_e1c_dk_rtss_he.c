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
	uint32_t reg_val;

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

	/*
	 * Setting expansion master0 SPI control register values
	 * 0xf at 8-11 bit is setting all 4 SPI instances as master
	 * bit 0-3; ss_in_sel; 0 - from io pad; 1 - from ssi_in_val
	 * bit 8-11; ss_in_val; when ss_in_sel=1, feed ss_in_val to SSI,
	 * each bit controls one SSI instance.
	 * For setting an spi instance as slave, put 0 in the corresponding
	 * bit position of both 8-11 and 0-3 bit fields.
	 * For example if we want to set SPI1 as master and
	 * remaining instances as slave, set the 1st bit for ss_in_sel, which will
	 * make ss_in_val to feed to SSI, and set the corresponding ss_in_val bit.
	 * here for SPI1 as master set the 9th bit. So the value to feed SPI1 as
	 * master and remaining as slave is 0x0202.
	 */
	sys_write32(0x0202, SSI_CTRL_EN);

	/*Clock : LP-SPI*/
	reg_val = sys_read32(HE_PER_CLK_EN);
	reg_val |= 1U << 16;
	sys_write32(reg_val, HE_PER_CLK_EN);

	/* LP-SPI0 Mode Selection */
	/* To Slave Set Bit : 15  */
	/* To Master Clear Bit : 15 */
	reg_val = sys_read32(HE_PER_CLK_EN);
	reg_val &= ~(1U << 15);
	sys_write32(reg_val, HE_PER_CLK_EN);

	/*LP-SPI0 Flex GPIO */
	sys_write32(0x1, VBAT_GPIO_CTRL_EN);

	if (IS_ENABLED(CONFIG_VIDEO)) {
		/*
		 * TODO: Check from the DTS property if LP-CAM is enabled and
		 * set clocks only for LP-CAM controller.
		 */
		/* Enable LPCAM Controller Peripheral clock. */
		sys_set_bits(HE_PER_CLK_EN, BIT(12));

		/* Enable LPCAM controller Pixel Clock (XVCLK). */
		/*
		 * Not needed for the time being as LP-CAM supports only
		 * parallel data-mode of cature and only MT9M114 sensor is
		 * tested with parallel data capture which generates clock
		 * internally. But can be used to generate XVCLK from LP CAM
		 * controller.
		 * sys_write32(0x140001, HE_CAMERA_PIXCLK);
		 */
	}

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
