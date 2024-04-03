/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <zephyr/device.h>
#include "ipm_arm_mhuv2.h"

#define DT_DRV_COMPAT arm_mhuv2
#define BIT0_1 0x1
#define NR2R 0x1
#define R2NR 0x2
#define COMB 0x4
#define TIMEOUT 0xFFFF00
#define DRV_MHUV2_CFG(d) ((const struct mhuv2_device_config *)(d)->config)
#define DRV_MHUV2_DATA(d) ((struct mhuv2_device_data *)(d)->data)

static void mhuv2_recv_isr(const struct device *);
static void mhuv2_send_isr(const struct device *);
static int mhuv2_send(const struct device *, uint8_t, const uint32_t *);
static int mhuv2_init(const struct device *);

static int mhuv2_send(const struct device *d, uint8_t ch_id,
		      const uint32_t *pdata)
{
	if ((d == NULL) || (pdata == NULL) || ch_id >= NUM_OF_CHAN) {
		return -EINVAL;
	}
	const struct mhuv2_device_config *drv_cfg = DRV_MHUV2_CFG(d);
	struct MHUV2_SND *SND = (struct MHUV2_SND *)(drv_cfg->base);
	uint32_t snd_timeout = TIMEOUT;

	/* Clear Interrupt Status */
	SND->INT_CLR = (NR2R | R2NR | COMB);
	/* Clear Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
	SND->INT_EN = SND->INT_EN & ~(NR2R | R2NR | COMB);
	/* Enable Interrupt generation (NR2R Int, R2NR Int, Combined Int) */
	SND->INT_EN = (NR2R | R2NR | COMB);

	/* Send Access Request */
	SND->ACCESS_REQUEST = 0x00000001;

	if (!(SND->ACCESS_READY & BIT0_1)) {
		while (snd_timeout) {
			if ((SND->ACCESS_READY & BIT0_1)) {
				/* Receiver is ready */
				break;
			}
			--snd_timeout;
		}
		if (!snd_timeout) {
			/* Receiver is busy */
			SND->ACCESS_REQUEST = 0x00000000;
			return -EBUSY;
		}
	}

	/* Clear any pending channel interrupts */
	SND->CHANNEL[ch_id].CH_INT_CLR = BIT0_1;

	SND->CHANNEL[ch_id].CH_INT_EN = BIT0_1;
	SND->CHANNEL[ch_id].CH_SET = *pdata;

	return 0;
}

static int mhuv2_init(const struct device *d)
{
	const struct mhuv2_device_config *config = DRV_MHUV2_CFG(d);
	struct mhuv2_device_data *data = DRV_MHUV2_DATA(d);

	config->init_func(d);
	data->callback = NULL;
	data->user_data = NULL;
	return 0;
}

static void mhuv2_isr(const struct device *d)
{
	const struct mhuv2_device_config *config = DRV_MHUV2_CFG(d);

	if (config->irq_type) {
		mhuv2_send_isr(d);
	} else {
		mhuv2_recv_isr(d);
	}
}

static void mhuv2_recv_isr(const struct device *d)
{
	uint8_t ch_id;
	uint8_t offset = 0;
	volatile uint32_t recv_ch_status_reg;
	const struct mhuv2_device_config *drv_cfg = DRV_MHUV2_CFG(d);
	struct mhuv2_device_data *drv_data = DRV_MHUV2_DATA(d);

	struct MHUV2_REC *RECV = (struct MHUV2_REC *)(drv_cfg->base);

	for (ch_id = 0; ch_id < NUM_OF_CHAN; ch_id++) {
		if (ch_id >= CHCOMB_INT_ST0_BEGIN &&
		    ch_id <= CHCOMB_INT_ST0_END) {
			offset = CHCOMB_INT_ST0_BEGIN;
			recv_ch_status_reg = RECV->CH_INT_ST0;
		} else if (ch_id >= CHCOMB_INT_ST1_BEGIN &&
			ch_id <= CHCOMB_INT_ST1_END) {
			offset = CHCOMB_INT_ST0_END;
			recv_ch_status_reg = RECV->CH_INT_ST1;
		} else if (ch_id >= CHCOMB_INT_ST2_BEGIN &&
			ch_id <= CHCOMB_INT_ST2_END) {
			offset = CHCOMB_INT_ST1_END;
			recv_ch_status_reg = RECV->CH_INT_ST2;
		} else if (ch_id >= CHCOMB_INT_ST3_BEGIN &&
			ch_id <= CHCOMB_INT_ST3_END) {
			offset = CHCOMB_INT_ST2_END;
			recv_ch_status_reg = RECV->CH_INT_ST3;
		}
		if ((recv_ch_status_reg) & (BIT0_1 << (ch_id - offset))) {
			*(drv_data->user_data) = RECV->CHANNEL[ch_id].CH_ST;
			if (drv_data->callback)
				drv_data->callback(d, drv_data->user_data);
			RECV->CHANNEL[ch_id].CH_CLR = 0xFFFFFFFF;
		}
	}
}

static void mhuv2_register_cb(const struct device *d,
			     callback_t cb, uint32_t *data)
{
	struct mhuv2_device_data *mhuv2_cb_data = DRV_MHUV2_DATA(d);

	mhuv2_cb_data->callback = cb;
	mhuv2_cb_data->user_data = data;
}

static const struct mhuv2_driver_api_t mhuv2_driver_api = {
	.send = mhuv2_send,
	.register_callback = mhuv2_register_cb,
};

static void mhuv2_send_isr(const struct device *d)
{
	uint8_t ch_id;
	const struct mhuv2_device_config *drv_cfg = DRV_MHUV2_CFG(d);
	struct MHUV2_SND *SND = (struct MHUV2_SND *)(drv_cfg->base);
	uint32_t snd_int_st = SND->INT_ST;
	struct mhuv2_device_data *drv_data = DRV_MHUV2_DATA(d);

	/* Check NR2R interrupt Status */
	if ((snd_int_st & NR2R) == NR2R) {
		SND->INT_CLR = NR2R;
	}
	/* Check R2NR interrupt status */
	if ((snd_int_st & R2NR) == R2NR) {
		SND->INT_CLR = R2NR;
	}
	/* Check Combined interrupt status */
	if (((snd_int_st & COMB) == COMB) && (SND->CH_INT_ST0 != 0x0)) {
		for (ch_id = 0 ; ch_id < NUM_OF_CHAN ; ++ch_id) {
			if ((SND->CHANNEL[ch_id].CH_INT_ST & BIT0_1)
				== BIT0_1) {
				/* Ack the CHCOMB interrupt */
				SND->CHANNEL[ch_id].CH_INT_CLR = BIT0_1;
				/* Channel's CH_INT_EN makes sure */
				/* the interrupt is generated on */
				/* setting CH_SET. */
				/* CH_ST is checked to make sure */
				/* receiver got the data and */
				/* set ACC_REQ to 0. */
				if ((SND->CHANNEL[ch_id].CH_ST == 0) &&
				    (SND->CHANNEL[ch_id].CH_INT_EN & BIT0_1)) {
					SND->CHANNEL[ch_id].CH_INT_EN
							&= ~(BIT0_1);
					SND->ACCESS_REQUEST = 0x00000000;
					if (drv_data->callback)
						drv_data->callback(d,
						drv_data->user_data);
				}
			}
		}
	}
}

#if defined(CONFIG_ARM_MHUV2)
#define ARM_MHUV2_DATA_INST(n)						\
	static void mhuv2_##n##_init(const struct device *dev);		\
	static const struct mhuv2_device_config mhuv2_cfg_##n = {	\
		.base = (uint8_t *)DT_INST_REG_ADDR(n),			\
		.irq_num = DT_INST_IRQN(n),				\
		.init_func = mhuv2_##n##_init,				\
		.irq_type = DT_INST_IRQ_HAS_NAME(n, tx),		\
	};								\
	static struct mhuv2_device_data mhuv2_data_##n;			\
									\
	DEVICE_DT_INST_DEFINE(n,					\
			      mhuv2_init,				\
			      NULL,					\
			      &mhuv2_data_##n,				\
			      &mhuv2_cfg_##n, POST_KERNEL,		\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			      &mhuv2_driver_api);			\
	static void mhuv2_##n##_init(const struct device *dev)		\
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    0,						\
			    mhuv2_isr,					\
			    DEVICE_DT_INST_GET(n), 0);			\
		irq_enable(DT_INST_IRQN(n));				\
	}
#endif

DT_INST_FOREACH_STATUS_OKAY(ARM_MHUV2_DATA_INST)
