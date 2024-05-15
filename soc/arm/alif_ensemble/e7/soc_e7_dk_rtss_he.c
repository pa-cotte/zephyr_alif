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

	/* enable pdm in expansion master */
	data = sys_read32(0x4902F000);
	data |= 0x100;

	/* lptimer settings */
#if DT_HAS_COMPAT_STATUS_OKAY(snps_dw_timers)
	uint32_t hw_en = 0, dirn_op_enable = 0, clk_src = 0;

	data = 0;

	/* LPTIMER 0 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer0), okay)
	if (IS_ENABLED(CONFIG_LPTIMER0_OUTPUT_TOGGLE)) {
		hw_en |= (1 << 0);  /* config lpgpio pin 0 as Hardware control data */
		dirn_op_enable = (1 << 0);  /* config pin as output */
	}
	if (IS_ENABLED(CONFIG_LPTIMER0_CLOCK_SOURCE_32K)) {
		clk_src = 0;
	} else if (IS_ENABLED(CONFIG_LPTIMER0_CLOCK_SOURCE_128K)) {
		clk_src = 1;
	} else if (IS_ENABLED(CONFIG_LPTIMER0_CLOCK_SOURCE_EXT)) {
		clk_src = 2;
		hw_en |= (1 << 0);  /* config lpgpio pin 0 as Hardware control data */
		dirn_op_enable &= ~(1 << 0);  /* config pin as input */
		sys_write32(1, 0x42007000);
	} else if (IS_ENABLED(CONFIG_LPTIMER0_CLOCK_SOURCE_CASCADE)) {
		clk_src = 3;
	}
	data |= (clk_src & 0x3) << 0;
#endif

	/* LPTIMER 1 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer1), okay)
	if (IS_ENABLED(CONFIG_LPTIMER1_OUTPUT_TOGGLE)) {
		hw_en |= (1 << 1);  /* config lpgpio pin 1 as Hardware control data */
		dirn_op_enable |= (1 << 1);  /* config pin as output */
	}
	if (IS_ENABLED(CONFIG_LPTIMER1_CLOCK_SOURCE_32K)) {
		clk_src = 0;
	} else if (IS_ENABLED(CONFIG_LPTIMER1_CLOCK_SOURCE_128K)) {
		clk_src = 1;
	} else if (IS_ENABLED(CONFIG_LPTIMER1_CLOCK_SOURCE_EXT)) {
		clk_src = 2;
		hw_en |= (1 << 1);  /* config lpgpio pin 1 as Hardware control data */
		dirn_op_enable &= ~(1 << 1);  /* config pin as input */
		sys_write32(1, 0x42007004);
	} else if (IS_ENABLED(CONFIG_LPTIMER1_CLOCK_SOURCE_CASCADE)) {
		clk_src = 3;
	}
	data |= (clk_src & 0x3) << 4;
#endif

	/* LPTIMER 2 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer2), okay)
	if (IS_ENABLED(CONFIG_LPTIMER2_OUTPUT_TOGGLE)) {
		hw_en |= (1 << 2);  /* config lpgpio pin 2 as Hardware control data */
		dirn_op_enable |= (1 << 2);  /* config pin as output */
	}
	if (IS_ENABLED(CONFIG_LPTIMER2_CLOCK_SOURCE_32K)) {
		clk_src = 0;
	} else if (IS_ENABLED(CONFIG_LPTIMER2_CLOCK_SOURCE_128K)) {
		clk_src = 1;
	} else if (IS_ENABLED(CONFIG_LPTIMER2_CLOCK_SOURCE_EXT)) {
		clk_src = 2;
		hw_en |= (1 << 2);  /* config lpgpio pin 2 as Hardware control data */
		dirn_op_enable &= ~(1 << 2);  /* config pin as input */
		sys_write32(1, 0x42007008);
	} else if (IS_ENABLED(CONFIG_LPTIMER2_CLOCK_SOURCE_CASCADE)) {
		clk_src = 3;
	}
	data |= (clk_src & 0x3) << 8;
#endif

	/* LPTIMER 3 settings */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(timer3), okay)
	if (IS_ENABLED(CONFIG_LPTIMER3_OUTPUT_TOGGLE)) {
		hw_en |= (1 << 3);  /* config lpgpio pin 3 as Hardware control data */
		dirn_op_enable |= (1 << 3);  /* config pin as output */
	}
	if (IS_ENABLED(CONFIG_LPTIMER3_CLOCK_SOURCE_32K)) {
		clk_src = 0;
	} else if (IS_ENABLED(CONFIG_LPTIMER3_CLOCK_SOURCE_128K)) {
		clk_src = 1;
	} else if (IS_ENABLED(CONFIG_LPTIMER3_CLOCK_SOURCE_EXT)) {
		clk_src = 2;
		hw_en |= (1 << 3);  /* config lpgpio pin 3 as Hardware control data */
		dirn_op_enable &= ~(1 << 3);  /* config pin as input */
		sys_write32(1, 0x4200700C);
	} else if (IS_ENABLED(CONFIG_LPTIMER3_CLOCK_SOURCE_CASCADE)) {
		clk_src = 3;
	}
	data |= (clk_src & 0x3) << 12;
#endif

	sys_write32(data, 0x1A609004);
	sys_write32(hw_en, 0x42002008);
	sys_write32(dirn_op_enable, 0x42002004);
#endif

	if (IS_ENABLED(CONFIG_DISPLAY)) {
		/* Enable CDC200 peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(1));

		if (IS_ENABLED(CONFIG_MIPI_DSI)) {
			/*
			 * CDC200 clock enablement for serial display.
			 *  Pixclk control register:
			 *	clk_divisor[24:16] - 0x10 (16)
			 * Pixel clock observed = (400 / 16) MHz = 25 MHz
			 * Serial display has recommended FPS of 54-66 FPS,
			 * and tested at 60 FPS.
			 */
			sys_write32(0x100001, EXPMST_CDC200_PIXCLK_CTRL);
		} else {
			/*
			 * CDC200 clock Pixel clock for parallel display.
			 *  Pixclk control register:
			 *	clk_divisor[24:16] - 0x90 (144)
			 * Pixel clock observed = (400 / 144) MHz = 2.77 MHz
			 * Parallel display tested at 5 FPS.
			 */
			sys_write32(0x900001, EXPMST_CDC200_PIXCLK_CTRL);
		}
	}
	if (IS_ENABLED(CONFIG_MIPI_DSI)) {
		/* Enable DSI controller peripheral clock. */
		sys_set_bits(EXPMST_PERIPH_CLK_EN, BIT(28));

		/* Enable TX-DPHY and PLL ref clock.*/
		sys_set_bits(EXPMST_MIPI_CKEN, BIT(0) | BIT(8));

		/* Enable TX-DPHY and D-PLL Power and Disable Isolation.*/
		sys_clear_bits(VBAT_PWR_CTRL, BIT(0) | BIT(1) | BIT(8) |
				BIT(9) | BIT(12));

		/* Enable HFOSC (38.4 MHz) and CFG (100 MHz) clock.*/
		sys_set_bits(CGU_CLK_ENA, BIT(21) | BIT(23));
	}

	/* LPUART settings */
	if (IS_ENABLED(CONFIG_SERIAL)) {
		/* Enable clock supply for LPUART */
		sys_write32(0x1, AON_RTSS_HE_LPUART_CKEN);
	}

	return 0;
}

SYS_INIT(ensemble_e7_dk_rtss_he_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
