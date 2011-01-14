/* arch/arm/mach-msm/board-topaz-keypad.c
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
#include "board-htctopaz.h"

static unsigned int htctopaz_col_gpios[] = { 35, 34 };

static unsigned int htctopaz_row_gpios[] = { 42, 41, 40 };

#define KEYMAP_INDEX(col, row) ((col)*ARRAY_SIZE(htctopaz_row_gpios) + (row))

/* HOME(up) + MENU (down)*/
static const unsigned short htctopaz_keymap[ARRAY_SIZE(htctopaz_col_gpios) *
					ARRAY_SIZE(htctopaz_row_gpios)] = {
	[KEYMAP_INDEX(0, 0)] = KEY_SEND,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 2)] = KEY_VOLUMEDOWN,

	[KEYMAP_INDEX(1, 0)] = KEY_END,
	[KEYMAP_INDEX(1, 1)] = KEY_MENU,
	[KEYMAP_INDEX(1, 2)] = KEY_BACK,
};

static struct gpio_event_matrix_info htctopaz_keypad_matrix_info = {
	.info.func = gpio_event_matrix_func,
	.keymap = htctopaz_keymap,
	.output_gpios = htctopaz_col_gpios,
	.input_gpios = htctopaz_row_gpios,
	.noutputs = ARRAY_SIZE(htctopaz_col_gpios),
	.ninputs = ARRAY_SIZE(htctopaz_row_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ |
		 GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS*/
};

static struct gpio_event_direct_entry htctopaz_keypad_key_map[] = {
	{
		.gpio	= TOPA100_POWER_KEY,
		.code	= KEY_POWER,
	},
};

static struct gpio_event_input_info htctopaz_keypad_key_info = {
	.info.func = gpio_event_input_func,
	.info.no_suspend = true,
	.debounce_time.tv.nsec = 5 * NSEC_PER_MSEC,
	.flags = 0,
	.type = EV_KEY,
	.keymap = htctopaz_keypad_key_map,
	.keymap_size = ARRAY_SIZE(htctopaz_keypad_key_map)
};

static struct gpio_event_info* htctopaz_keypad_info[] = {
	&htctopaz_keypad_matrix_info.info,
	&htctopaz_keypad_key_info.info,
};

static struct gpio_event_platform_data htctopaz_keypad_data = {
	.name = "htctopaz-keypad",
	.info = htctopaz_keypad_info,
	.info_count = ARRAY_SIZE(htctopaz_keypad_info)
};

static struct platform_device htctopaz_keypad_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev = {
		.platform_data	= &htctopaz_keypad_data,
	},
};

static int __init htctopaz_init_keypad(void)
{
	if (!machine_is_htctopaz())
		return -1;

	return platform_device_register(&htctopaz_keypad_device);
}

device_initcall(htctopaz_init_keypad);
