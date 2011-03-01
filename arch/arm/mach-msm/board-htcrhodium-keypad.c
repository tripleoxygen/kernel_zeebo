/* arch/arm/mach-msm/board-htcrhodium-keypad.c
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>
#include "gpio_chip.h"
#include "board-htcrhodium.h"
static char *keycaps = "--qwerty";
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "board_rhodium." 
module_param_named(keycaps, keycaps, charp, 0);


static unsigned int rhodium_col_gpios[] = { RHODIUM_KPD_COL0, RHODIUM_KPD_COL1 };

static unsigned int rhodium_row_gpios[] = { RHODIUM_KPD_ROW0, RHODIUM_KPD_ROW1, RHODIUM_KPD_ROW2, RHODIUM_KPD_ROW3 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(rhodium_row_gpios) + (row))

/* HOME(up) + MENU (down)*/
static const unsigned short rhodium_keymap[ARRAY_SIZE(rhodium_col_gpios) *
					ARRAY_SIZE(rhodium_row_gpios)] = {

	//DEVICE IS RHOD400 !!!!!

	//GPIO 27 == Keyboard IRQ
	//39 == Volume UP (GPIO 39)
	//40 == Volume DOWN (GPIO 40)
	//18 == END key

	[KEYMAP_INDEX(0, 2)] = KEY_SEND,
	[KEYMAP_INDEX(1, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(0, 2)] = KEY_HOME,
	[KEYMAP_INDEX(0, 2)] = KEY_MENU,
	[KEYMAP_INDEX(1, 3)] = KEY_END,
};


static struct gpio_event_matrix_info rhodium_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = rhodium_keymap,
	.output_gpios = rhodium_col_gpios,
	.input_gpios = rhodium_row_gpios,
	.noutputs = ARRAY_SIZE(rhodium_col_gpios),
	.ninputs = ARRAY_SIZE(rhodium_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 50 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ |
		 GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_info *rhodium_keypad_info[] = {
	&rhodium_keypad_matrix_info.info,
};

static struct gpio_event_platform_data rhodium_keypad_data = {
	.name = "rhodium-keypad",
	.info = rhodium_keypad_info,
	.info_count = ARRAY_SIZE(rhodium_keypad_info)
};

static struct platform_device rhodium_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &rhodium_keypad_data,
	},
};

static int __init rhodium_init_keypad(void)
{
	if (!machine_is_htcrhodium())
		return 0;

	rhodium_keypad_matrix_info.keymap = rhodium_keymap;

	return platform_device_register(&rhodium_keypad_device);
}

device_initcall(rhodium_init_keypad);

