/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ZEPHYR_INCLUDE_DRIVERS_ENSEMBLE_DSI_H__
#define __ZEPHYR_INCLUDE_DRIVERS_ENSEMBLE_DSI_H__

#include <zephyr/device.h>
#include <stddef.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ensemble_dsi_mode {
	ENSEMBLE_DSI_COMMAND_MODE,
	ENSEMBLE_DSI_VIDEO_MODE,
};

/*
 * Set command/video mode for Ensemble DSI.
 */
int ensemble_dsi_set_mode(const struct device *dev,
		enum ensemble_dsi_mode mode);

#ifdef __cplusplus
}
#endif
#endif /* __ZEPHYR_INCLUDE_DRIVERS_ENSEMBLE_DSI_H__ */
