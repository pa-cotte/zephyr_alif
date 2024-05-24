/*
 * Copyright (c) 2019, Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>

#include <zephyr/drivers/video.h>

#ifdef CONFIG_SOC_FAMILY_ENSEMBLE
#define SRAM1_BASE 0x08000000  /* CONFIG_SOC_FAMILY_ENSEMBLE */
#else
K_HEAP_DEFINE(video_buffer_pool, CONFIG_VIDEO_BUFFER_POOL_SZ_MAX *
		CONFIG_VIDEO_BUFFER_POOL_NUM_MAX);
#endif

static struct video_buffer video_buf[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];

struct mem_block {
	void *data;
};

static struct mem_block video_block[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];

struct video_buffer *video_buffer_alloc(size_t size)
{
	struct video_buffer *vbuf = NULL;
	struct mem_block *block;
	int i;

#ifdef CONFIG_SOC_FAMILY_ENSEMBLE
	/*
	 * TODO: HACK - Allocate everything from the SRAM1_BASE address for
	 * testing.
	 */
	static uint32_t base_addr = SRAM1_BASE;
#endif /* CONFIG_SOC_FAMILY_ENSEMBLE */

	/* find available video buffer */
	for (i = 0; i < ARRAY_SIZE(video_buf); i++) {
		if (video_buf[i].buffer == NULL) {
			vbuf = &video_buf[i];
			block = &video_block[i];
			break;
		}
	}

	if (vbuf == NULL) {
		return NULL;
	}

	/* Alloc buffer memory */
#if CONFIG_SOC_FAMILY_ENSEMBLE
	/* TODO: HACK - Increment base_address as new allocations come. */
	block->data = (void *)base_addr;
	base_addr = base_addr + size;
#else
	block->data = k_heap_alloc(&video_buffer_pool, size, K_FOREVER);
#endif /* CONFIG_SOC_FAMILY_ENSEMBLE */

	if (block->data == NULL) {
		return NULL;
	}

	vbuf->buffer = block->data;
	vbuf->size = size;
	vbuf->bytesused = 0;

	return vbuf;
}

void video_buffer_release(struct video_buffer *vbuf)
{
	struct mem_block *block = NULL;
	int i;

	/* vbuf to block */
	for (i = 0; i < ARRAY_SIZE(video_block); i++) {
		if (video_block[i].data == vbuf->buffer) {
			block = &video_block[i];
			break;
		}
	}

	vbuf->buffer = NULL;
	if (block) {
#ifndef CONFIG_SOC_FAMILY_ENSEMBLE
		k_heap_free(&video_buffer_pool, block->data);
#endif /* CONFIG_SOC_FAMILY_ENSEMBLE */
	}
}
