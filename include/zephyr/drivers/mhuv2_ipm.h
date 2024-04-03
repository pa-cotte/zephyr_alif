/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_MHUV2_IPM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MHUV2_IPM_H_

/**
 * @brief MHUv2 IPM Interface
 * @defgroup mhuv2_ipm_interface IPM Interface
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef callback_t
 * @brief Callback API for incoming MHUv2 messages
 *
 * These callbacks execute in interrupt context. Therefore, use only
 * interrupt-safe APIS. Registration of callbacks is done via
 * @a mhuv2_ipm_register_callback
 *
 * @param mhuv2_ipmdev - Driver instance
 * @param user_data - User data pointer to contain received data.
 */
typedef void (*callback_t)(const struct device *mhuv2_ipmdev,
			   uint32_t *user_data);

/**
 * @typedef mhuv2_ipm_send_t
 * @brief Callback API to send MHUv2 IPM messages
 *
 * See @a mhuv2_ipm_send() for argument definitions.
 *
 * @param mhuv2_ipmdev - Driver instance
 * @param data - user data pointer, pointing to message to be send.
 *
 * @return See return values for @a mhuv2_ipm_send()
 */
typedef int (*mhuv2_ipm_send_t)(const struct device *mhuv2_ipmdev,
				uint8_t ch_id, const uint32_t *data);

/**
 * @typedef mhuv2_ipm_register_callback_t
 *
 * @brief Callback API should call mhuv2_ipm_register_callback_t
 * to register 'cb' in order to receive MHUv2 messages.
 *
 * See @a mhuv2_ipm_register_callback() for function description
 *
 * @param mhuv2_ipm - Driver instance
 * @param cb - Callback function to execute on incoming message interrupts.
 * @param user_data - Application-specific data pointer which will be passed
 *                    to the callback function to receive MHUv2 message.
 *
 */

typedef void (*mhuv2_ipm_register_callback_t)(const struct device *mhuv2_ipm,
					      callback_t cb,
					      void *user_data);
__subsystem struct mhuv2_ipm_driver_api {
	mhuv2_ipm_send_t send;
	mhuv2_ipm_register_callback_t register_callback;
};

__syscall void mhuv2_ipm_rb(const struct device *mhuv2_ipmdev,
			   callback_t cb, void *data);

static inline void mhuv2_ipm_rb(const struct device *mhuv2_ipmdev,
			       callback_t cb, void *data)
{
	const struct mhuv2_ipm_driver_api *api =
			(const struct mhuv2_ipm_driver_api *)mhuv2_ipmdev->api;

	api->register_callback(mhuv2_ipmdev, cb, data);
}

/**
 * @brief Try to send a message over the MHUv2 IPM device.
 *
 * Send message pointed by 'data' through 'mhuv2_ipmdev' MHUv2 driver instance.
 *
 * If the *data parameter is not NULL, this *data is expected to be delivered
 * on the receiving side using the data parameter by a registered callback.
 *
 * @param mhuv2_ipmdev - Driver instance
 * @param data - Pointer to the data sent in the message.
 *
 * @retval 0 On success, negative value on error.
 */
__syscall int mhuv2_ipm_send(const struct device *mhuv2_ipmdev,
			     uint8_t ch_id, uint32_t *data);

static inline int mhuv2_ipm_send(const struct device *mhuv2_ipmdev,
				 uint8_t ch_id, uint32_t *data)
{
	const struct mhuv2_ipm_driver_api *api =
			(const struct mhuv2_ipm_driver_api *)mhuv2_ipmdev->api;

	if (api->send == NULL)
		return -ENOSYS;

	return api->send(mhuv2_ipmdev, ch_id, data);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MHUV2_IPM_H_ */
