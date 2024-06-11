/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ALIF_CLOCK_CONTROL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ALIF_CLOCK_CONTROL_H_

/**
 * @brief ALIF clock configuration bit field.
 *
 * - module       [ 0 : 2 ]
 * - reg          [ 3 : 10 ]
 * - en_bit       [ 11 : 15 ]
 * - en_mask      [ 16 ]
 * - src          [ 17 : 18 ]
 * - src_mask     [ 19 : 20 ]
 * - src_bit      [ 21 : 25 ]
 * - reserved     [ 26 : 31 ]
 *
 * @param module   clock module.
 * @param reg      register offset
 * @param en_bit   clock enable bit.
 * @param en_mask  clock enable mask (used to check clock ctrl is required)
 * @param src      clock source value.
 * @param src_mask clock source value mask (check clk src select is required)
 * @param src_bit  clock source bit position
 */
#define ALIF_CLK_CFG(module, reg, en_bit, en_mask, src, src_mask, src_bit)    \
	((((ALIF_##module##_CLKCTL_MODULE) & ALIF_CLOCK_MODULE_MASK) <<       \
			ALIF_CLOCK_MODULE_SHIFT) |                            \
	(((ALIF_##reg##_REG) & ALIF_CLOCK_REG_MASK) << ALIF_CLOCK_REG_SHIFT) | \
	(((en_bit) & ALIF_CLOCK_EN_BIT_POS_MASK) <<                           \
			ALIF_CLOCK_EN_BIT_POS_SHIFT) |                        \
	((en_mask) << ALIF_CLOCK_EN_MASK_SHIFT) |                             \
	(((src) & ALIF_CLOCK_SRC_MASK) << ALIF_CLOCK_SRC_SHIFT) |             \
	(((src_mask) & ALIF_CLOCK_SRC_MASK_MASK) <<                           \
			ALIF_CLOCK_SRC_MASK_SHIFT) |                          \
	(((src_bit) & ALIF_CLOCK_SRC_BIT_POS_MASK) <<                         \
			ALIF_CLOCK_SRC_BIT_POS_SHIFT))

#define ALIF_CLOCK_MODULE_MASK         0x7U
#define ALIF_CLOCK_MODULE_SHIFT        0U
#define ALIF_CLOCK_REG_MASK            0xFFU
#define ALIF_CLOCK_REG_SHIFT           3U
#define ALIF_CLOCK_EN_BIT_POS_MASK     0x1FU
#define ALIF_CLOCK_EN_BIT_POS_SHIFT    11U
#define ALIF_CLOCK_EN_MASK_SHIFT       16U
#define ALIF_CLOCK_SRC_MASK            0x3U
#define ALIF_CLOCK_SRC_SHIFT           17U
#define ALIF_CLOCK_SRC_MASK_MASK       0x3U
#define ALIF_CLOCK_SRC_MASK_SHIFT      19U
#define ALIF_CLOCK_SRC_BIT_POS_MASK    0x1FU
#define ALIF_CLOCK_SRC_BIT_POS_SHIFT   21U

/* Clock modules */
#define ALIF_PER_MST_CLKCTL_MODULE     0x1U
#define ALIF_PER_SLV_CLKCTL_MODULE     0x2U
#define ALIF_AON_CLKCTL_MODULE         0x3U
#define ALIF_VBAT_CLKCTL_MODULE        0x4U
#define ALIF_M55HE_CLKCTL_MODULE       0x5U
#define ALIF_M55HP_CLKCTL_MODULE       0x6U

/* register offset and for PER_MST_CLKCTL module */
#define ALIF_CAMERA_PIXCLK_CTRL_REG    0x0U
#define ALIF_CDC200_PIXCLK_CTRL_REG    0x4U
#define ALIF_CSI_PIXCLK_CTRL_REG       0x8U
#define ALIF_PERIPH_CLK_ENA_REG        0xCU
#define ALIF_DPHY_PLL_CTRL0_REG        0x10U
#define ALIF_DPHY_PLL_CTRL1_REG        0x14U
#define ALIF_DPHY_PLL_CTRL2_REG        0x18U
#define ALIF_MIPI_CKEN_REG             0x40U
#define ALIF_ETH_CTRL_REG              0x80U

/* register offset and for PER_SLV_CLKCTL module */
#define ALIF_EXPMST0_CTRL_REG          0x0U
#define ALIF_UART_CTRL_REG             0x8U
#define ALIF_CANFD_CTRL_REG            0xCU
#define ALIF_I2S0_CTRL_REG             0x10U
#define ALIF_I2S1_CTRL_REG             0x14U
#define ALIF_I2S2_CTRL_REG             0x18U
#define ALIF_I2S3_CTRL_REG             0x1CU
#define ALIF_I3C_CTRL_REG              0x24U
#define ALIF_ADC_CTRL_REG              0x30U
#define ALIF_DAC_CTRL_REG              0x34U
#define ALIF_CMP_CTRL_REG              0x38U
#define ALIF_GPIO0_CTRL_REG            0x80U
#define ALIF_GPIO1_CTRL_REG            0x84U
#define ALIF_GPIO2_CTRL_REG            0x88U
#define ALIF_GPIO3_CTRL_REG            0x8CU
#define ALIF_GPIO4_CTRL_REG            0x90U
#define ALIF_GPIO5_CTRL_REG            0x94U
#define ALIF_GPIO6_CTRL_REG            0x98U
#define ALIF_GPIO7_CTRL_REG            0x9CU
#define ALIF_GPIO8_CTRL_REG            0xA0U
#define ALIF_GPIO9_CTRL_REG            0xA4U
#define ALIF_GPIO10_CTRL_REG           0xA8U
#define ALIF_GPIO11_CTRL_REG           0xACU
#define ALIF_GPIO12_CTRL_REG           0xB0U
#define ALIF_GPIO13_CTRL_REG           0xB4U
#define ALIF_GPIO14_CTRL_REG           0xB8U

/* register offset and for AON_CLKCTL module */
#define ALIF_RTSS_HE_LPUART_CKEN_REG   0x1CU
#define ALIF_SYSTOP_CLK_DIV_REG        0x20U
#define ALIF_MISC_REG1_REG             0x30U

/* register offset and for VBAT_CLKCTL module */
#define ALIF_TIMER_CLKSEL_REG          0x4U
#define ALIF_RTC_CLK_EN_REG            0x10U

/* register offset and for M55HE_CFG_CLKCTL module */
#define ALIF_HE_CLK_ENA_REG            0x10U
#define ALIF_HE_I2S_CTRL_REG           0x14U
#define ALIF_HE_CAMERA_PIXCLK_REG      0x20U

/* register offset and for M55HP_CFG_LKCTL module */
#define ALIF_HP_CLK_ENA_REG            0x10U

/* Macro definitions for clkid cell */
#define ALIF_CAMERA_PIX_SYST_ACLK  \
	ALIF_CLK_CFG(PER_MST, CAMERA_PIXCLK_CTRL, 0U, 1U, 0U, 1U, 4U)
#define ALIF_CAMERA_PIX_PLL_CLK3   \
	ALIF_CLK_CFG(PER_MST, CAMERA_PIXCLK_CTRL, 0U, 1U, 1U, 1U, 4U)
#define ALIF_CDC200_PIX_SYST_ACLK  \
	ALIF_CLK_CFG(PER_MST, CDC200_PIXCLK_CTRL, 0U, 1U, 0U, 1U, 4U)
#define ALIF_CDC200_PIX_PLL_CLK3   \
	ALIF_CLK_CFG(PER_MST, CDC200_PIXCLK_CTRL, 0U, 1U, 1U, 1U, 4U)
#define ALIF_CSI_PIX_SYST_ACLK      \
	ALIF_CLK_CFG(PER_MST, CSI_PIXCLK_CTRL, 0U, 1U, 0U, 1U, 4U)
#define ALIF_CSI_PIX_PLL_CLK3       \
	ALIF_CLK_CFG(PER_MST, CSI_PIXCLK_CTRL, 0U, 1U, 1U, 1U, 4U)
#define ALIF_CPI_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 0U, 1U, 0U, 0U, 0U)
#define ALIF_DPI_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 1U, 1U, 0U, 0U, 0U)
#define ALIF_DMA0_CLK               \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 4U, 1U, 0U, 0U, 0U)
#define ALIF_GPU_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 8U, 1U, 0U, 0U, 0U)
#define ALIF_ETHERNET_CLK           \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 12U, 1U, 0U, 0U, 0U)
#define ALIF_ETH_RMII_REFCLK_PIN    \
	ALIF_CLK_CFG(PER_MST, ETH_CTRL, 0U, 0U, 0U, 1U, 4U)
#define ALIF_ETH_RMII_PLL_CLK_50M    \
	ALIF_CLK_CFG(PER_MST, ETH_CTRL, 0U, 0U, 1U, 1U, 4U)
#define ALIF_SDC_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 16U, 1U, 0U, 0U, 0U)
#define ALIF_USB_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 20U, 1U, 0U, 0U, 0U)
#define ALIF_CSI_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 24U, 1U, 0U, 0U, 0U)
#define ALIF_DSI_CLK                \
	ALIF_CLK_CFG(PER_MST, PERIPH_CLK_ENA, 28U, 1U, 0U, 0U, 0U)
#define ALIF_MIPI_TXDPHY_CLK        \
	ALIF_CLK_CFG(PER_MST, MIPI_CKEN, 0U, 1U, 0U, 0U, 0U)
#define ALIF_MIPI_RXDPHY_CLK        \
	ALIF_CLK_CFG(PER_MST, MIPI_CKEN, 4U, 1U, 0U, 0U, 0U)
#define ALIF_MIPI_PLLREF_CLK        \
	ALIF_CLK_CFG(PER_MST, MIPI_CKEN, 8U, 1U, 0U, 0U, 0U)
#define ALIF_MIPI_BYPASS_CLK        \
	ALIF_CLK_CFG(PER_MST, MIPI_CKEN, 12U, 1U, 0U, 0U, 0U)
#define ALIF_BACKUP_RAM_CLK         \
	ALIF_CLK_CFG(PER_SLV, EXPMST0_CTRL, 4U, 1U, 0U, 0U, 0U)
#define ALIF_PDM_76M8_CLK           \
	ALIF_CLK_CFG(PER_SLV, EXPMST0_CTRL, 8U, 1U, 0U, 1U, 9U)
#define ALIF_PDM_AUDIO_CLK          \
	ALIF_CLK_CFG(PER_SLV, EXPMST0_CTRL, 8U, 1U, 1U, 1U, 9U)
#define ALIF_UARTn_38M4_CLK(n)      \
	ALIF_CLK_CFG(PER_SLV, UART_CTRL, n, 1U, 0U, 1U, (n + 8))
#define ALIF_UARTn_SYST_PCLK(n)     \
	ALIF_CLK_CFG(PER_SLV, UART_CTRL, n, 1U, 1U, 1U, (n + 8))
#define ALIF_CANFD_HFOSC_CLK        \
	ALIF_CLK_CFG(PER_SLV, CANFD_CTRL, 12U, 1U, 0U, 1U, 16U)
#define ALIF_CANFD_160M_CLK         \
	ALIF_CLK_CFG(PER_SLV, CANFD_CTRL, 12U, 1U, 1U, 1U, 16U)
#define ALIF_I2Sn_76M8_CLK(n)       \
	ALIF_CLK_CFG(PER_SLV, I2S##n##_CTRL, 12U, 1U, 0U, 1U, 20U)
#define ALIF_I2Sn_AUDIO_CLK(n)      \
	ALIF_CLK_CFG(PER_SLV, I2S##n##_CTRL, 12U, 1U, 1U, 1U, 20U)
#define ALIF_I3C_CLK                \
	ALIF_CLK_CFG(PER_SLV, I3C_CTRL, 0U, 1U, 0U, 0U, 0U)
#define ALIF_ADCn_CLK(n)            \
	ALIF_CLK_CFG(PER_SLV, ADC_CTRL, (n * 4), 1U, 0U, 0U, 0U)
#define ALIF_ADC24_CLK              \
	ALIF_CLK_CFG(PER_SLV, ADC_CTRL, 12U, 1U, 0, 0U, 0U)
#define ALIF_DACn_CLK(n)            \
	ALIF_CLK_CFG(PER_SLV, DAC_CTRL, (n * 4), 1U, 0U, 0U, 0U)
#define ALIF_CMPn_CLK(n)            \
	ALIF_CLK_CFG(PER_SLV, CMP_CTRL, (n * 4), 1U, 0U, 0U, 0U)
#define ALIF_GPIOn_DB_CLK(n)        \
	ALIF_CLK_CFG(PER_SLV, GPIO##n##_CTRL, 12U, 1U, 0U, 0U, 0U)
#define ALIF_LPUART_CLK             \
	ALIF_CLK_CFG(AON, RTSS_HE_LPUART_CKEN, 0U, 1U, 0U, 0U, 0U)
#define ALIF_LPTIMERn_S32K_CLK(n)   \
	ALIF_CLK_CFG(VBAT, TIMER_CLKSEL, 0U, 0U, 0U, 3U, (n * 4))
#define ALIF_LPTIMERn_128K_CLK(n)   \
	ALIF_CLK_CFG(VBAT, TIMER_CLKSEL, 0U, 0U, 1U, 3U, (n * 4))
#define ALIF_LPTIMERn_LPTMRn_IO_PIN(n)  \
	ALIF_CLK_CFG(VBAT, TIMER_CLKSEL, 0U, 0U, 2U, 3U, (n * 4))
#define ALIF_LPTIMERn_CASCADE_CLK(n)  \
	ALIF_CLK_CFG(VBAT, TIMER_CLKSEL, 0U, 0U, 3U, 3U, (n * 4))
#define ALIF_LPRTC_CLK              \
	ALIF_CLK_CFG(VBAT, RTC_CLK_EN, 0U, 1U, 0U, 0U, 0U)
#define ALIF_NPU_HE_CLK             \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 0U, 1U, 0U, 0U, 0U)
#define ALIF_DMA2_CLK               \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 4U, 1U, 0U, 0U, 0U)
#define ALIF_LPPDM_76M8_CLK         \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 8U, 1U, 0U, 1U, 9U)
#define ALIF_LPPDM_AUDIO_CLK        \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 8U, 1U, 1U, 1U, 9U)
#define ALIF_LPCPI_CLK              \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 12U, 1U, 0U, 0U, 0U)
#define ALIF_LPSPI_CLK              \
	ALIF_CLK_CFG(M55HE, HE_CLK_ENA, 16U, 1U, 0U, 0U, 0U)
#define ALIF_LPI2S_76M8_CLK         \
	ALIF_CLK_CFG(M55HE, HE_I2S_CTRL, 0U, 1U, 0U, 1U, 16U)
#define ALIF_LPI2S_AUDIO_CLK        \
	ALIF_CLK_CFG(M55HE, HE_I2S_CTRL, 0U, 1U, 1U, 1U, 16U)
#define ALIF_LPCPI_PXL_CLK          \
	ALIF_CLK_CFG(M55HE, HE_CAMERA_PIXCLK, 0U, 1U, 0U, 0U, 0U)
#define ALIF_NPU_HP_CLK             \
	ALIF_CLK_CFG(M55HP, HP_CLK_ENA, 0U, 1U, 0U, 0U, 0U)
#define ALIF_DMA1_CLK               \
	ALIF_CLK_CFG(M55HP, HP_CLK_ENA, 4U, 1U, 0U, 0U, 0U)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_ALIF_CLOCK_CONTROL_H_ */
