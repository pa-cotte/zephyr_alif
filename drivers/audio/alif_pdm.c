/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/init.h>
#if defined(CONFIG_PINCTRL)
#include <zephyr/drivers/pinctrl.h>
#endif

LOG_MODULE_REGISTER(alif_pdm, 3);

#define PDM_CONFIG_REGISTER           (0x0)
#define PDM_CTL_REGISTER              (0x4)
#define PDM_THRESHOLD_REGISTER        (0x8)
#define PDM_FIFO_STATUS_REGISTER      (0xC)
#define PDM_INT_STATUS_REGISTER       (0x14)
#define PDM_INTERRUPT_REGISTER        (0x1C)
#define PDM_DATA_REGISTER             (0x20)
#define BYPASS_IIR_FILTER             (0x2)
#define PDM_MODE_SELECT               (16)
#define PDM_CHANNEL_ENABLE            (0)
#define CHANNEL_IRQ_ENABLE            (8)
#define FIFO_FULL_INTERRUPT           (0)

#define MAX_DATA_ITEMS                (8)
#define MAX_QUEUE_LEN                 (100)

#define PDM(idx) DT_NODELABEL(pdm##idx)
#define MAX_NUM_CHANNELS              (8)

unsigned short data[MAX_NUM_CHANNELS * MAX_DATA_ITEMS];
void *queue_data[MAX_QUEUE_LEN];
const struct device *dt_device;

struct pdm_data {
	struct k_mem_slab *mem_slab;
	unsigned int block_size;
	struct k_msgq buf_queue;
	uint8_t  channel_map;
	uint32_t num_channels;
	unsigned char *data_buffer;
	unsigned int buf_index;
	unsigned int slab_missed;
	unsigned int record_data;
	unsigned int bytes_got;
};

struct pdm_config {
	const unsigned long meminit;
	const unsigned int irqnum;
#if defined(CONFIG_PINCTRL)
	const struct pinctrl_dev_config *pcfg;
#endif
};

static int dmic_alif_pdm_configure(const struct device *dev,
				   struct dmic_cfg *config)
{
	struct pdm_data *pdata;

	if (config->channel.req_num_chan == 0 ||
	   config->channel.req_num_chan > MAX_NUM_CHANNELS) {
		LOG_DBG("config invalid: number of channels not valid\n");
		return -EINVAL;
	}

	pdata = dev->data;

	if (pdata) {
		pdata->mem_slab = config->streams[0].mem_slab;
		pdata->block_size = config->streams[0].block_size;
		pdata->channel_map = config->channel.req_chan_map_lo & 0xFF;
		pdata->num_channels = config->channel.req_num_chan;

		LOG_DBG("slab: %p\n", pdata->mem_slab);
		LOG_DBG("block size: %d\n", pdata->block_size);
	}

	LOG_DBG("dmic configure okay\n");

	return 0;
}

static void pcm_setup(const struct device *dev)
{
	const struct pdm_config *cfg;

	cfg = dev->config;

	/* Set the coefficient values for each channels. The values are */
	/* based on simiulation testing. */

	/* Channel 0 */
	sys_write32(0x00000000, cfg->meminit+0x040);
	sys_write32(0x000007FF, cfg->meminit+0x044);
	sys_write32(0x00000000, cfg->meminit+0x048);
	sys_write32(0x00000004, cfg->meminit+0x04c);
	sys_write32(0x00000004, cfg->meminit+0x050);
	sys_write32(0x000007FC, cfg->meminit+0x054);
	sys_write32(0x00000000, cfg->meminit+0x058);
	sys_write32(0x000007FB, cfg->meminit+0x05c);
	sys_write32(0x000007E4, cfg->meminit+0x060);
	sys_write32(0x00000000, cfg->meminit+0x064);
	sys_write32(0x0000002B, cfg->meminit+0x068);
	sys_write32(0x00000009, cfg->meminit+0x06c);
	sys_write32(0x00000016, cfg->meminit+0x070);
	sys_write32(0x00000049, cfg->meminit+0x074);
	sys_write32(0x00000793, cfg->meminit+0x078);
	sys_write32(0x000006F8, cfg->meminit+0x07c);
	sys_write32(0x00000045, cfg->meminit+0x080);
	sys_write32(0x00000178, cfg->meminit+0x084);
	sys_write32(0x00000004, cfg->meminit+0x0c0);
	sys_write32(0x00000003, cfg->meminit+0x0c4);
	sys_write32(0x00000013, cfg->meminit+0x0c8);
	sys_write32(0x00060002, cfg->meminit+0x0cc);
	sys_write32(0x00020027, cfg->meminit+0x0d0);

	/* Channel 1*/
	sys_write32(0x00000001, (cfg->meminit+0x140));
	sys_write32(0x00000003, (cfg->meminit+0x144));
	sys_write32(0x00000003, (cfg->meminit+0x148));
	sys_write32(0x000007F4, (cfg->meminit+0x14c));
	sys_write32(0x00000004, (cfg->meminit+0x150));
	sys_write32(0x000007ED, (cfg->meminit+0x154));
	sys_write32(0x000007F5, (cfg->meminit+0x158));
	sys_write32(0x000007F4, (cfg->meminit+0x15c));
	sys_write32(0x000007D3, (cfg->meminit+0x160));
	sys_write32(0x000007FE, (cfg->meminit+0x164));
	sys_write32(0x000007BC, (cfg->meminit+0x168));
	sys_write32(0x000007E5, (cfg->meminit+0x16c));
	sys_write32(0x000007D9, (cfg->meminit+0x170));
	sys_write32(0x00000793, (cfg->meminit+0x174));
	sys_write32(0x00000029, (cfg->meminit+0x178));
	sys_write32(0x0000072C, (cfg->meminit+0x17c));
	sys_write32(0x00000072, (cfg->meminit+0x180));
	sys_write32(0x000002FD, (cfg->meminit+0x184));
	sys_write32(0x00000004, (cfg->meminit+0x1c0));
	sys_write32(0x0000001F, (cfg->meminit+0x1c4));
	sys_write32(0x0000000D, (cfg->meminit+0x1c8));
	sys_write32(0x00060002, (cfg->meminit+0x1cc));
	sys_write32(0x0004002D, (cfg->meminit+0x1d0));

	/* Channel 2 */
	sys_write32(0x00000000, (cfg->meminit+0x240));
	sys_write32(0x000007FF, (cfg->meminit+0x244));
	sys_write32(0x00000000, (cfg->meminit+0x248));
	sys_write32(0x00000004, (cfg->meminit+0x24c));
	sys_write32(0x00000004, (cfg->meminit+0x250));
	sys_write32(0x000007FC, (cfg->meminit+0x254));
	sys_write32(0x00000000, (cfg->meminit+0x258));
	sys_write32(0x000007FB, (cfg->meminit+0x25c));
	sys_write32(0x000007E4, (cfg->meminit+0x260));
	sys_write32(0x00000000, (cfg->meminit+0x264));
	sys_write32(0x0000002B, (cfg->meminit+0x268));
	sys_write32(0x00000009, (cfg->meminit+0x26c));
	sys_write32(0x00000016, (cfg->meminit+0x270));
	sys_write32(0x00000049, (cfg->meminit+0x274));
	sys_write32(0x00000793, (cfg->meminit+0x278));
	sys_write32(0x000006F8, (cfg->meminit+0x27c));
	sys_write32(0x00000045, (cfg->meminit+0x280));
	sys_write32(0x00000178, (cfg->meminit+0x284));
	sys_write32(0x00000004, (cfg->meminit+0x2c0));
	sys_write32(0x00000003, (cfg->meminit+0x2c4));
	sys_write32(0x00000013, (cfg->meminit+0x2c8));
	sys_write32(0x00060002, (cfg->meminit+0x2cc));
	sys_write32(0x00020027, (cfg->meminit+0x2d0));

	/* Channel 3 */
	sys_write32(0x00000001, (cfg->meminit+0x340));
	sys_write32(0x00000003, (cfg->meminit+0x344));
	sys_write32(0x00000003, (cfg->meminit+0x348));
	sys_write32(0x000007F4, (cfg->meminit+0x34c));
	sys_write32(0x00000004, (cfg->meminit+0x350));
	sys_write32(0x000007ED, (cfg->meminit+0x354));
	sys_write32(0x000007F5, (cfg->meminit+0x358));
	sys_write32(0x000007F4, (cfg->meminit+0x35c));
	sys_write32(0x000007D3, (cfg->meminit+0x360));
	sys_write32(0x000007FE, (cfg->meminit+0x364));
	sys_write32(0x000007BC, (cfg->meminit+0x368));
	sys_write32(0x000007E5, (cfg->meminit+0x36c));
	sys_write32(0x000007D9, (cfg->meminit+0x370));
	sys_write32(0x00000793, (cfg->meminit+0x374));
	sys_write32(0x00000029, (cfg->meminit+0x378));
	sys_write32(0x0000072C, (cfg->meminit+0x37c));
	sys_write32(0x00000072, (cfg->meminit+0x380));
	sys_write32(0x000002FD, (cfg->meminit+0x384));
	sys_write32(0x00000004, (cfg->meminit+0x3c0));
	sys_write32(0x0000001F, (cfg->meminit+0x3c4));
	sys_write32(0x0000000D, (cfg->meminit+0x3c8));
	sys_write32(0x00060002, (cfg->meminit+0x3cc));
	sys_write32(0x0004002D, (cfg->meminit+0x3d0));

	/* Channel 4 */
	sys_write32(0x00000001, (cfg->meminit+0x440));
	sys_write32(0x00000003, (cfg->meminit+0x444));
	sys_write32(0x00000003, (cfg->meminit+0x448));
	sys_write32(0x000007F4, (cfg->meminit+0x44c));
	sys_write32(0x00000004, (cfg->meminit+0x450));
	sys_write32(0x000007ED, (cfg->meminit+0x454));
	sys_write32(0x000007F5, (cfg->meminit+0x458));
	sys_write32(0x000007F4, (cfg->meminit+0x45c));
	sys_write32(0x000007D3, (cfg->meminit+0x460));
	sys_write32(0x000007FE, (cfg->meminit+0x464));
	sys_write32(0x000007BC, (cfg->meminit+0x468));
	sys_write32(0x000007E5, (cfg->meminit+0x46c));
	sys_write32(0x000007D9, (cfg->meminit+0x470));
	sys_write32(0x00000793, (cfg->meminit+0x474));
	sys_write32(0x00000029, (cfg->meminit+0x478));
	sys_write32(0x0000072C, (cfg->meminit+0x47c));
	sys_write32(0x00000072, (cfg->meminit+0x480));
	sys_write32(0x000002FD, (cfg->meminit+0x484));
	sys_write32(0x00000004, (cfg->meminit+0x4c0));
	sys_write32(0x0000001F, (cfg->meminit+0x4c4));
	sys_write32(0x0000000D, (cfg->meminit+0x4c8));
	sys_write32(0x00060002, (cfg->meminit+0x4cc));
	sys_write32(0x0004002D, (cfg->meminit+0x4d0));

	/* Channel 5 */
	sys_write32(0x00000000, (cfg->meminit+0x540));
	sys_write32(0x000007FF, (cfg->meminit+0x544));
	sys_write32(0x00000000, (cfg->meminit+0x548));
	sys_write32(0x00000004, (cfg->meminit+0x54c));
	sys_write32(0x00000004, (cfg->meminit+0x550));
	sys_write32(0x000007FC, (cfg->meminit+0x554));
	sys_write32(0x00000000, (cfg->meminit+0x558));
	sys_write32(0x000007FB, (cfg->meminit+0x55c));
	sys_write32(0x000007E4, (cfg->meminit+0x560));
	sys_write32(0x00000000, (cfg->meminit+0x564));
	sys_write32(0x0000002B, (cfg->meminit+0x568));
	sys_write32(0x00000009, (cfg->meminit+0x56c));
	sys_write32(0x00000016, (cfg->meminit+0x570));
	sys_write32(0x00000049, (cfg->meminit+0x574));
	sys_write32(0x00000793, (cfg->meminit+0x578));
	sys_write32(0x000006F8, (cfg->meminit+0x57c));
	sys_write32(0x00000045, (cfg->meminit+0x580));
	sys_write32(0x00000178, (cfg->meminit+0x584));
	sys_write32(0x00000004, (cfg->meminit+0x5c0));
	sys_write32(0x00000003, (cfg->meminit+0x5c4));
	sys_write32(0x00000013, (cfg->meminit+0x5c8));
	sys_write32(0x00060002, (cfg->meminit+0x5cc));
	sys_write32(0x00020027, (cfg->meminit+0x5d0));

	/* Channel 6 */
	sys_write32(0x00000001, (cfg->meminit+0x640));
	sys_write32(0x00000003, (cfg->meminit+0x644));
	sys_write32(0x00000003, (cfg->meminit+0x648));
	sys_write32(0x000007F4, (cfg->meminit+0x64c));
	sys_write32(0x00000004, (cfg->meminit+0x650));
	sys_write32(0x000007ED, (cfg->meminit+0x654));
	sys_write32(0x000007F5, (cfg->meminit+0x658));
	sys_write32(0x000007F4, (cfg->meminit+0x65c));
	sys_write32(0x000007D3, (cfg->meminit+0x660));
	sys_write32(0x000007FE, (cfg->meminit+0x664));
	sys_write32(0x000007BC, (cfg->meminit+0x668));
	sys_write32(0x000007E5, (cfg->meminit+0x66c));
	sys_write32(0x000007D9, (cfg->meminit+0x670));
	sys_write32(0x00000793, (cfg->meminit+0x674));
	sys_write32(0x00000029, (cfg->meminit+0x678));
	sys_write32(0x0000072C, (cfg->meminit+0x67c));
	sys_write32(0x00000072, (cfg->meminit+0x680));
	sys_write32(0x000002FD, (cfg->meminit+0x684));
	sys_write32(0x00000004, (cfg->meminit+0x6c0));
	sys_write32(0x0000001F, (cfg->meminit+0x6c4));
	sys_write32(0x0000000D, (cfg->meminit+0x6c8));
	sys_write32(0x00060002, (cfg->meminit+0x6cc));
	sys_write32(0x0004002D, (cfg->meminit+0x6d0));

	/* Channel 7 */
	sys_write32(0x00000000, (cfg->meminit+0x740));
	sys_write32(0x000007FF, (cfg->meminit+0x744));
	sys_write32(0x00000000, (cfg->meminit+0x748));
	sys_write32(0x00000004, (cfg->meminit+0x74c));
	sys_write32(0x00000004, (cfg->meminit+0x750));
	sys_write32(0x000007FC, (cfg->meminit+0x754));
	sys_write32(0x00000000, (cfg->meminit+0x758));
	sys_write32(0x000007FB, (cfg->meminit+0x75c));
	sys_write32(0x000007E4, (cfg->meminit+0x760));
	sys_write32(0x00000000, (cfg->meminit+0x764));
	sys_write32(0x0000002B, (cfg->meminit+0x768));
	sys_write32(0x00000009, (cfg->meminit+0x76c));
	sys_write32(0x00000016, (cfg->meminit+0x770));
	sys_write32(0x00000049, (cfg->meminit+0x774));
	sys_write32(0x00000793, (cfg->meminit+0x778));
	sys_write32(0x000006F8, (cfg->meminit+0x77c));
	sys_write32(0x00000045, (cfg->meminit+0x780));
	sys_write32(0x00000178, (cfg->meminit+0x784));
	sys_write32(0x00000004, (cfg->meminit+0x7c0));
	sys_write32(0x00000003, (cfg->meminit+0x7c4));
	sys_write32(0x00000013, (cfg->meminit+0x7c8));
	sys_write32(0x00060002, (cfg->meminit+0x7cc));
	sys_write32(0x00020027, (cfg->meminit+0x7d0));

}

static void enable_interrupt(const struct device *dev)
{
	const struct pdm_config *cfg;

	cfg = dev->config;

	/* Enable interrupt for all channels */
	sys_write32(0xFF << CHANNEL_IRQ_ENABLE |
		    0x3 << FIFO_FULL_INTERRUPT,
		    cfg->meminit + PDM_INTERRUPT_REGISTER);
}

static void disable_interrupt(const struct device *dev)
{
	const struct pdm_config *cfg;

	cfg = dev->config;
	sys_write32(0x0, cfg->meminit + PDM_INTERRUPT_REGISTER);
}

static int dmic_alif_pdm_trigger(const struct device *dev,
				 enum dmic_trigger cmd)
{
	struct pdm_data *pdata = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_STOP:
		disable_interrupt(dev);
		pdata->record_data = 0;
		LOG_DBG("slab missed %d\n", pdata->slab_missed);
		break;

	case DMIC_TRIGGER_START:
		LOG_DBG("trigger start\n");
		pdata->record_data = 1;
		pdata->bytes_got = 0;
		pdata->buf_index = 0;
		pdata->data_buffer = NULL;
		pdata->slab_missed = 0;
		enable_interrupt(dev);
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int dmic_alif_pdm_read(const struct device *dev,
			      uint8_t stream,
			      void **buffer, size_t *size, int32_t timeout)
{
	struct pdm_data *pdata = dev->data;
	int rc;

	rc = k_msgq_get(&pdata->buf_queue, buffer, SYS_TIMEOUT_MS(timeout));

	if (rc != 0) {
		LOG_DBG("No audio data to be read\n");
		LOG_DBG("bytes_got: %d\n", pdata->bytes_got);
	} else {

		*size = pdata->block_size;
	}

	return rc;
}

/*
 * Check if this channel is enabled
 * in the channel map
 */
int channel_data(int channel, struct pdm_data *pdm_data)
{
	unsigned int mask;

	mask = 0x1 << channel;

	/* check if the channel bit is set in channel_map */

	if (pdm_data->channel_map & mask)
		return 1;
	else
		return 0;
}

void *get_slab(struct pdm_data *pdm_data)
{
	int rc;
	void *buffer;

	rc = k_mem_slab_alloc(pdm_data->mem_slab, &buffer, K_NO_WAIT);

	if (rc) {
		pdm_data->slab_missed++;
		return NULL;
	}

	return buffer;
}

static void alif_pdm_irq(void *value)
{
	const struct pdm_config *cfg;
	struct pdm_data *pdmdata;
	uint32_t meminit;
	uint32_t num_items;
	uint32_t data_bytes;
	uint32_t intstatus;
	uint32_t block_size;
	uint32_t whole;
	uint32_t bytes_available;
	uint32_t temp;
	uint32_t i;
	uint32_t j;
	uint32_t k = 0;

	pdmdata = dt_device->data;
	block_size = pdmdata->block_size;

	cfg = dt_device->config;
	meminit = cfg->meminit;

	intstatus = sys_read32(meminit + PDM_INT_STATUS_REGISTER);
	num_items = sys_read32(meminit + PDM_FIFO_STATUS_REGISTER);

	for (i = 0; i < num_items; i++) {
		/* read data for all channels */
		for (j = 0; j < 4; j++) {
			temp = sys_read32(meminit +
				PDM_DATA_REGISTER + (j * 4));

			if (channel_data(j * 2, pdmdata)) {
				data[k++] = temp & 0x0000FFFF;
			}

			if (channel_data((j * 2) + 1, pdmdata)) {
				data[k++] = (temp & 0xFFFF0000) >> 16;
			}
		}
	}

	if (pdmdata->record_data == 0) {
		return;
	}

	data_bytes = num_items *
		     pdmdata->num_channels *
		     sizeof(unsigned short);

	pdmdata->bytes_got += data_bytes;

	if (pdmdata->data_buffer == NULL) {

		pdmdata->data_buffer = get_slab(pdmdata);

		if (pdmdata->data_buffer == NULL) {
			return;
		}

		pdmdata->buf_index = 0;
	}

	bytes_available = block_size - pdmdata->buf_index;

	if (bytes_available >= data_bytes) {
		memcpy((pdmdata->data_buffer + pdmdata->buf_index),
			data, data_bytes);
		pdmdata->buf_index += data_bytes;
	} else {
		if (bytes_available > 0)
			memcpy((pdmdata->data_buffer + pdmdata->buf_index),
				data, bytes_available);

		whole = data_bytes - bytes_available;

		/* queue this buffer */
		k_msgq_put(&pdmdata->buf_queue,
			   &pdmdata->data_buffer, K_NO_WAIT);

		pdmdata->data_buffer = NULL;
		pdmdata->buf_index = 0;

		/* get the next buffer */
		/* The buffer size will be at least equal to the */
		/* max size of data that is available at one instant. */
		/* which is max_fifo_size * max_channels * 2 */
		/* 8 * 8 * 2 = 128 bytes */
		pdmdata->data_buffer = get_slab(pdmdata);

		if (pdmdata->data_buffer) {
			memcpy(pdmdata->data_buffer,
			       data + bytes_available, whole);
			pdmdata->buf_index = whole;
		}

	}
}

static void init_registers(const struct device *dev)
{
	uint32_t meminit;
	const struct pdm_config *cfg;

	cfg = dev->config;
	meminit = cfg->meminit;

	/* Enable all the channels and set the PDM mode as 4 */
	sys_write32(4 << PDM_MODE_SELECT | 0xFF << PDM_CHANNEL_ENABLE,
		    meminit + PDM_CONFIG_REGISTER);

	/* Enable the Bypass IIR Filter setting */
	sys_write32(0x1 << BYPASS_IIR_FILTER, meminit + PDM_CTL_REGISTER);

	/*
	 * Set the FIFO threshold as 7
	 * Interrupt will be generated when this threshold is reached
	 * in the FIFO
	 */
	sys_write32(0x7, meminit + PDM_THRESHOLD_REGISTER);

	pcm_setup(dev);
}

static int pdm_alif_init(const struct device *dev)
{
	const struct pdm_config *cfg;
	struct pdm_data *pdata;

	dt_device = dev;

	cfg = dev->config;
	pdata = dev->data;

	IRQ_CONNECT(DT_IRQN(PDM(0)), 1, alif_pdm_irq, NULL, 0);

	irq_enable(DT_IRQN(PDM(0)));

	k_msgq_init(&pdata->buf_queue,
		    (char *)queue_data,
		    sizeof(void *),
		    MAX_QUEUE_LEN);

	if (cfg->pcfg != NULL) {
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	}

	init_registers(dev);

	LOG_INF("alif pdm driver init okay\n");

	return 0;
}

static const struct _dmic_ops dmic_alif_pdm_ops = {
	.configure = dmic_alif_pdm_configure,
	.trigger = dmic_alif_pdm_trigger,
	.read = dmic_alif_pdm_read,
};

static struct pdm_data dmic_alif_pdm_data;

#if defined(CONFIG_PINCTRL)
PINCTRL_DT_DEFINE(PDM(0));
#endif

static const struct pdm_config dmic_alif_pdm_cfg = {
	.meminit = DT_REG_ADDR(PDM(0)),

#if defined(CONFIG_PINCTRL)
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(PDM(0)),
#endif
};

DEVICE_DT_DEFINE(PDM(0), pdm_alif_init, NULL,
		 &dmic_alif_pdm_data, &dmic_alif_pdm_cfg,
		 POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY,
		 &dmic_alif_pdm_ops);
