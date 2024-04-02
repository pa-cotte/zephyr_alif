/* Copyright (c) 2024 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT alif_pinctrl

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/sys/util.h>

/* Pinmux settings:
 * syntax : U  U  U  U  U  U  P  P  P  P  P  P  P  F  F  F
 * bit pos: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
 * syntax : U  U  U  U  U  U  U  U  DR E2 E1 P2 P1 SR ST REN
 * bit pos: 31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16
 * bits 0:2 [FFF] denote pinmux functions
 * bits 3:9 [PPPPPPP] denote port values
 * bits 11:15 and 24:31 are unused
 * bit 16 denotes read enable
 * bit 17 denotes schmitt trigger enable
 * bit 18 denotes slew rate
 * bits 19:20 denote driver disabled state control
 * bits 21:22 denote output drive strength
 * bit 23 denotes driver
 */
#define PIN_FUNC_SHIFT 3
#define PIN_NUM_MASK 0x7F
#define PIN_NUM_CLR_MASK 0x3F8

#if DT_NODE_HAS_PROP(DT_NODELABEL(pinctrl), reg)
#define PINCTRL_BASE      DT_REG_ADDR(DT_NODELABEL(pinctrl))
#endif

#define PINMUX_ADDR(value) (uint32_t *)(PINCTRL_BASE + \
		(((value >> PIN_FUNC_SHIFT) & PIN_NUM_MASK) * 4))

int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t pinmux_value = *(uint32_t *)pin;
	/* get the address of the pin using port value */
	uint32_t *pinctrl_addr = PINMUX_ADDR(pinmux_value);
	/* clear only the port value, other fields are set already */
	*pinctrl_addr = (pinmux_value & (~PIN_NUM_CLR_MASK));
	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins,
			   uint8_t pin_cnt, uintptr_t reg)
{
	int ret = 0;

	ARG_UNUSED(reg);
	while (pin_cnt-- > 0) {
		ret = pinctrl_configure_pin(pins++);
		if (ret < 0) {
			break;
		}
	}
	return ret;
}
