/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief SDIO driver for Alif MCU family.
 *
 * This driver implements SDIO protocol of the SD interface for general I/O functions.
 *
 * Features:
 * - Supports 4-bit interface
 * - Supports SDIO card interrupts in 1-bit and 4-bit SD modes
 * - Supports SDSC, SDHC and SDXC memory
 */

#define DT_DRV_COMPAT alif_sdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <sd.h>

// #include "alif_sdio_adapter.h"

LOG_MODULE_REGISTER(alif_sdio, CONFIG_SDHC_LOG_LEVEL);

struct alif_sdio_config {
	const struct pinctrl_dev_config *pincfg;
	SDMMC_Type *reg_addr;
	uint8_t irq_priority;
	// const struct device *clk_dev;
	// clock_control_subsys_t clk_id;
};

struct alif_sdio_data {
	// bool card_present;
	// bool card_busy;

	enum sdhc_clock_speed clock_speed;
	enum sdhc_bus_width bus_width;

	sd_handle_t sd_handle;
	sd_param_t sd_param;

	sdhc_interrupt_cb_t sdio_cb;
	void *sdio_cb_user_data;
	// struct alif_sd_config host_cfg;
};

static int alif_sdio_reset(const struct device *dev)
{
	struct alif_sdio_data *data = dev->data;

	hc_reset(&data->sd_handle, (uint8_t)(SDMMC_SW_RST_ALL_Msk));

	/* Needed ????
	    if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_3V3_Msk) != 0U) {
		powerlevel = SDMMC_PC_BUS_VSEL_3V3_Msk;
	    } else if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_3V0_Msk) != 0U) {
		powerlevel = SDMMC_PC_BUS_VSEL_3V0_Msk;
	    } else if ((pHsd->hc_caps & SDMMC_HOST_SD_CAP_VOLT_1V8_Msk) != 0U) {
		powerlevel = SDMMC_PC_BUS_VSEL_1V8_Msk;
	    } else {
		powerlevel = 0U;
	    }

	    hc_set_bus_power(pHsd, (uint8_t)(powerlevel | SDMMC_PC_BUS_PWR_VDD1_Msk));
	*/

	hc_set_tout(&data->sd_handle, 0xE);

	hc_config_interrupt(&data->sd_handle);

	return 0;
}

static int alif_sdio_set_io(const struct device *dev, struct sdhc_io *ios)
{
	// struct alif_sdio_data *data = dev->data;
	// data->host_cfg.clock_hz = ios->clock;
	// data->host_cfg.bus_width = ios->bus_width;
	// return alif_sdio_adapter_set_io(dev, ios);
	return 0;
}

static int alif_sdio_card_busy(const struct device *dev)
{
	// struct alif_sdio_data *data = dev->data;
	// return data->card_busy ? 1 : 0;
	return 0;
}

static int alif_sdio_get_card_present(const struct device *dev)
{
	// struct alif_sdio_data *data = dev->data;
	// return data->card_present ? 1 : 0;
	return 0;
}

static int alif_sdio_request(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *data)
{
	// return alif_sdio_adapter_request(dev, cmd, data);
	return 0;
}

static int alif_sdio_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	// /* Fill in host capabilities */
	// memset(props, 0, sizeof(*props));
	// props->f_max = 50000000;
	// props->f_min = 400000;
	// props->host_caps.bus_4_bit_support = true;
	// props->host_caps.high_spd_support = true;
	// props->host_caps.sdr50_support = true;
	// props->host_caps.sdio_async_interrupt_support = true;
	// props->host_caps.vol_330_support = true;
	return 0;
}

static int alif_sdio_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t cb, int sources,
				      void *user_data)
{
	struct alif_sdio_data *data = dev->data;

	if (sources != SDHC_INT_SDIO) {
		return -ENOTSUP;
	}

	if (cb == NULL) {
		return -EINVAL;
	}

	data->sdio_cb = cb;
	data->sdio_cb_user_data = user_data;
	// return alif_sdio_adapter_enable_irq(dev);
	return 0;
}

static int alif_sdio_disable_interrupt(const struct device *dev, int sources)
{
	// struct alif_sdio_data *data = dev->data;
	// data->sdio_cb = NULL;
	// data->sdio_cb_user_data = NULL;
	// return alif_sdio_adapter_disable_irq(dev);
	return 0;
}

static const struct sdhc_driver_api alif_sdio_api = {
	.reset = alif_sdio_reset,
	.request = alif_sdio_request,
	.set_io = alif_sdio_set_io,
	.get_card_present = alif_sdio_get_card_present,
	.card_busy = alif_sdio_card_busy,
	.get_host_props = alif_sdio_get_host_props,
	.enable_interrupt = alif_sdio_enable_interrupt,
	.disable_interrupt = alif_sdio_disable_interrupt,
};

static int alif_sdio_init(const struct device *dev)
{
	struct alif_sdio_data *data = dev->data;
	const struct alif_sdio_config *config = dev->config;
	// struct alif_sdio_data *data = dev->data;
	int ret;
	SD_DRV_STATUS status;

	// if (!device_is_ready(config->clk_dev)) {
	//     LOG_ERR("Clock control device not ready");
	//     return -ENODEV;
	// }

	/* Configure pins */
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	// /* Initialize configuration */
	// data->host_cfg.pcfg = config->pcfg;
	// data->host_cfg.clock_hz = 400000;  /* Initial clock frequency */
	// data->host_cfg.bus_width = SDHC_BUS_WIDTH1BIT;
	// data->host_cfg.use_dma = false;

	// config->sd_param.dev_id = SDMMC_DEV_ID;
	// config->sd_param.clock_id = RTE_SDC_CLOCK_SELECT;
	// config->sd_param.bus_width = RTE_SDC_BUS_WIDTH;
	// config->sd_param.dma_mode = RTE_SDC_DMA_SELECT;
	// config->sd_param.app_callback = sd_cb;

	// /* Initialize adapter */
	status = sd_init(&data->sd_param);
	// ret = alif_sdio_adapter_init(dev, &data->host_cfg);
	// if (ret < 0) {
	//     return ret;
	// }

	return 0;
}

#define ALIF_SDIO_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct alif_sdio_config alif_sdio_config_##n = {                              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.reg_addr = (SDMMC_Type *)DT_INST_REG_ADDR(n),                                     \
		.irq_priority = DT_INST_IRQ(n, priority)};                                         \
                                                                                                   \
	static struct alif_sdio_data alif_sdio_data_##n;                                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &alif_sdio_init, NULL, &alif_sdio_data_##n,                       \
			      &alif_sdio_config_##n, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,       \
			      &alif_sdio_api);

DT_INST_FOREACH_STATUS_OKAY(ALIF_SDIO_INIT)

// .clk_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),
// .clk_id = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),