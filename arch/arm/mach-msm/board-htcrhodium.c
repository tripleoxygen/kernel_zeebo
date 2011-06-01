/* linux/arch/arm/mach-msm/board-htcrhodium.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/android_pmem.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/gpio_keys.h>
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <asm/setup.h>
#include <mach/msm_serial_hs.h>

#include <mach/board.h>
#ifdef CONFIG_MSM_SMEM_BATTCHG
#include <mach/htc_battery.h>
#endif
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/msm_fb.h>
#include <mach/msm_hsusb.h>
#include <mach/vreg.h>

#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#include <linux/microp-keypad.h>
#include <linux/input/msm_ts.h>
#include <linux/mfd/microp-ng.h>

#ifdef CONFIG_HTC_HEADSET
#include <mach/htc_headset.h>
#endif

#include <mach/board_htc.h>

#include "proc_comm_wince.h"
#include "devices.h"
#include "gpio_chip.h"
#include "board-htcrhodium.h"


extern int init_mmc(void);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	//.wakeup_irq = MSM_GPIO_TO_INT(21),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

/******************************************************************************
 * MicroP Keypad
 ******************************************************************************/
static int htcrhodium_microp_keymap[] = {
	KEY_B, //KEY_RESERVED, // invalid
	KEY_BACK, //Back key
	KEY_Q,
	KEY_HOME, //Mute button on back of device
	KEY_CAMERA,
	KEY_A,
	KEY_F,
	KEY_S,
	KEY_D,
	KEY_SEND, //Send Key
	// 10
	KEY_MENU, //Windows Key
	KEY_S, //KEY_RESERVED, // 0x0b   //Unknown
	KEY_I,
	KEY_K,
	KEY_J,
	KEY_H,
	KEY_G,
	KEY_A,
	KEY_4,
	KEY_1, //KEY_RESERVED, // 0x13  //Unknown
	// 20
	KEY_2, //KEY_RESERVED, // 0x14	//Unknown
	KEY_L,
	KEY_I,
	KEY_P,
	KEY_O,
	KEY_B,
	KEY_9,
	KEY_8,
	KEY_N,
	KEY_ENTER,
	// 30
	KEY_M,
	KEY_C,
	KEY_V,
	KEY_0,
	KEY_U,
	KEY_E,
	KEY_R,
	KEY_Q,
	KEY_T,
	KEY_Y,
	// 40
	KEY_W,
	KEY_UP,  //ARROW KEY
	KEY_1,
	KEY_2,
	KEY_DOWN, //KEY_LEFT,	//KEY_DOWN,   //ITS KEY DOWN FOR SURE!!!! STILL DOESN't GO DOWN 
	KEY_3,
	KEY_4,
	KEY_5,
	KEY_LEFT, 	//KEY_LEFT,	//ARROW KEY
	KEY_6,
	// 50
	KEY_2,			//KEY_RESERVED, // 0x32 //Unknown
	KEY_SPACE,
	KEY_BACKSPACE,
	KEY_7,
	KEY_RIGHT,		//KEY_UNKNOWN,  // ARROW KEY
	KEY_SPACE, //UNKNOWN
	KEY_X,	//KEY_COMMA, //UNKNOWN
	KEY_EMAIL,
	KEY_DOT,
	KEY_FN,  //Doesn't do anything?
	// 60
	KEY_LEFTSHIFT,
	KEY_Z,
	KEY_X,
	KEY_COMMA,
	KEY_COMPOSE, //Brings up search box?
	KEY_C, //KEY_SLASH,  //Unknown
	KEY_COMMA,
	KEY_6, 			//Unknown
	KEY_8,			//Unknown
	KEY_1, //KEY_RESERVED, // 0x45	//Unknown
	// 70
	KEY_2, //KEY_RESERVED, // 0x46	//Unknown
	KEY_P, //KEY_EMAIL,	//Unknown
};

static int htcrhodium_init_microp_keypad(struct device *dev)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	ret = gpio_request(RHODIUM_KPD_IRQ, "Keyboard");
	if (ret)
		return ret;
	ret = gpio_direction_input(RHODIUM_KPD_IRQ);
	if (ret)
		gpio_free(RHODIUM_KPD_IRQ);

	return ret;
}

static void htcrhodium_exit_microp_keypad(struct device *dev) {
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_free(RHODIUM_KPD_IRQ);
}

static struct microp_keypad_platform_data htcrhodium_keypad_data = {
	.read_modifiers = true,
	.gpio_clamshell = -1,
	.init = htcrhodium_init_microp_keypad,
	.exit = htcrhodium_exit_microp_keypad,
	.irq_keypress = MSM_GPIO_TO_INT(RHODIUM_KPD_IRQ),
	.keypad_scancodes = htcrhodium_microp_keymap,
	.keypad_scancodes_size = ARRAY_SIZE(htcrhodium_microp_keymap),
};

static struct platform_device htcrhodium_keypad = {
	.name = "microp-keypad",
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_keypad_data,
	},
};

static struct platform_device* htcrhodium_microp_keypad_clients[] = {
	&htcrhodium_keypad,
};

static uint16_t micropksc_compatible_versions[] = {
	RHOD_MICROP_KSC_VERSION,
};

static struct microp_platform_data htcrhodium_microp_keypad_pdata = {
	.version_reg = RHOD_MICROP_KSC_VERSION_REG,
	.clients = htcrhodium_microp_keypad_clients,
	.nclients = ARRAY_SIZE(htcrhodium_microp_keypad_clients),
	.comp_versions = micropksc_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropksc_compatible_versions),
};

/******************************************************************************
 * MicroP LED
 ******************************************************************************/
#if 0
static struct platform_device htcrhodium_microp_leds = {
	.id = -1,
	.name = "htcrhodium-microp-leds",
};

static struct platform_device* htcrhodium_microp_clients[] = {
	&htcrhodium_microp_leds,
};

static uint16_t micropklt_compatible_versions[] = {
	RHOD_MICROP_KLT_VERSION
};

static struct microp_platform_data htcrhodium_microp_pdata = {
	.version_reg = RHOD_MICROP_KLT_VERSION_REG,
	.clients = htcrhodium_microp_clients,
	.nclients = ARRAY_SIZE(htcrhodium_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};
#endif

/******************************************************************************
 * USB
 ******************************************************************************/
static void htcrhodium_usb_disable(void)
{
	gpio_set_value(RHODIUM_USBPHY_RST, 0); 
	mdelay(3);
}

static void htcrhodium_usb_enable(void)
{
	gpio_set_value(RHODIUM_USBPHY_RST, 1);
	mdelay(3);
}

static void htcrhodium_usb_hw_reset(bool off)
{
	printk(KERN_WARNING "%s(%d)\n", __func__, off ? 1 : 0);

	if (off) {
		htcrhodium_usb_disable();
	} else {
		htcrhodium_usb_enable();
	}
}

static void htcrhodium_usb_phy_reset(void)
{
	printk(KERN_DEBUG "%s\n", __func__);

	htcrhodium_usb_disable();
	htcrhodium_usb_enable();
}

static struct msm_hsusb_platform_data htcrhodium_hsusb_board_pdata = {
	.hw_reset = htcrhodium_usb_hw_reset,
	.phy_reset = htcrhodium_usb_phy_reset,
	.usb_connected = notify_usb_connected,
};

#if 0
static struct platform_device raphael_rfkill = {
	.name = "htcraphael_rfkill",
	.id = -1,
};
#endif

/******************************************************************************
 * I2C
 ******************************************************************************/
static struct i2c_board_info i2c_devices[] = {
	{
		//gsensor 0x38
		I2C_BOARD_INFO("bma150", 0x70>>1),
	},
	{
		// LED & Backlight controller
		I2C_BOARD_INFO("microp-klt", 0x66),
	},
	{
		// Keyboard controller
		.type = "microp-ng",
		.addr = 0x67,
		.platform_data = &htcrhodium_microp_keypad_pdata,
	},
	{		
		I2C_BOARD_INFO("mt9t013", 0x6c>>1),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
	{		
		I2C_BOARD_INFO("tpa2016", 0xb0>>1),
	},
	{		
		I2C_BOARD_INFO("a1010", 0xf4>>1),
	},
	{		
		I2C_BOARD_INFO("adc3001", 0x30>>1),
	},
};

/******************************************************************************
 * Sound
 ******************************************************************************/
#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(2, "NO_MIC_HEADSET"),
	SND(3, "BT"),
	SND(3, "BT_EC_OFF"),

	SND(13, "SPEAKER_MIC"),

	SND(0x11, "IDLE"),
	SND(0x11, "CURRENT"),
};
#undef SND

static struct msm_snd_endpoints htcrhodium_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device htcrhodium_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_snd_endpoints,
	},
};

#if 0
static struct platform_device rhod_prox = {
    .name       = "rhodium_proximity",
};
#endif

/******************************************************************************
 * H2W
 ******************************************************************************/
#ifdef CONFIG_HTC_HEADSET
static void h2w_config_cpld(int route);
static void h2w_init_cpld(void);
static struct h2w_platform_data rhodium_h2w_data = {
	.cable_in1		= RHODIUM_CABLE_IN1,
	.cable_in2		= RHODIUM_CABLE_IN2,
	.h2w_clk		= RHODIUM_H2W_CLK,
	.h2w_data		= RHODIUM_H2W_DATA,
	.debug_uart 		= H2W_UART3,
	.config_cpld 		= h2w_config_cpld,
	.init_cpld 		= h2w_init_cpld,
	.headset_mic_35mm	= 17,
};

static void h2w_config_cpld(int route)
{
	switch (route) {
	case H2W_UART3:
		gpio_set_value(RHODIUM_H2W_UART_MUX, 1);
		break;
	case H2W_GPIO:
		gpio_set_value(RHODIUM_H2W_UART_MUX, 0);
		break;
	}
}

static void h2w_init_cpld(void)
{
	h2w_config_cpld(H2W_UART3);
	gpio_set_value(rhodium_h2w_data.h2w_clk, 0);
	gpio_set_value(rhodium_h2w_data.h2w_data, 0);
}

static struct platform_device rhodium_h2w = {
	.name		= "h2w",
	.id		= -1,
	.dev		= {
		.platform_data	= &rhodium_h2w_data,
	},
};
#endif

/******************************************************************************
 * Touchscreen
 ******************************************************************************/
#if 0
static struct ts_virt_key htcrhodium_ts_keys_y[] = {
	// key      min   max
	{KEY_UP,    105, 267},
	{KEY_DOWN,  268, 429},
	{KEY_HOME,  430, 591},
	{KEY_LEFT,  592, 753},
	{KEY_RIGHT, 754, 915},
};

static struct msm_ts_virtual_keys htcrhodium_ts_virtual_keys_y = {
	.keys = &htcrhodium_ts_keys_y[0],
	.num_keys = 5,
};
#endif

static struct msm_ts_platform_data htcrhodium_ts_pdata = {
	.min_x		= 185,
	.max_x		= 825,
	.inv_x		= 1010,
	.min_y		= 10,
	.max_y		= 835,
	.inv_y		= 950,
	.min_press	= 100,
	.max_press	= 256,
#if 0
	.virt_y_start = 862,
	.vkeys_y	= &htcrhodium_ts_virtual_keys_y,
#endif
};

/******************************************************************************
 * GPIO Keys
 ******************************************************************************/
static struct gpio_keys_button htcrhodium_button_table[] = {
	/*KEY		GPIO			ACTIVE_LOW	DESCRIPTION		type	wakeup	debounce*/
	{KEY_END,	RHODIUM_END_KEY,	1,		"End",			EV_KEY, 1,	0},
	{KEY_VOLUMEUP,	RHODIUM_VOLUMEUP_KEY,	1,		"Volume Up",		EV_KEY,	0,	0},
	{KEY_VOLUMEDOWN,RHODIUM_VOLUMEDOWN_KEY,	1,		"Volume Down",		EV_KEY,	0,	0},
	{KEY_POWER,	RHODIUM_POWER_KEY,	1,		"Power button",		EV_KEY,	1,	0},
};

static struct gpio_keys_platform_data htcrhodium_gpio_keys_pdata = {
	.buttons = htcrhodium_button_table,
	.nbuttons = ARRAY_SIZE(htcrhodium_button_table), 
};

static struct platform_device htcrhodium_gpio_keys = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &htcrhodium_gpio_keys_pdata,
	},
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_rtc,
//	&msm_device_htc_hw,
#ifdef CONFIG_MSM_SMEM_BATTCHG
	&msm_device_htc_battery_smem,
#endif
	&htcrhodium_snd,
	&htcrhodium_gpio_keys,
//	&raphael_rfkill,
	&msm_device_touchscreen,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
	// &rhod_prox,	
#ifdef CONFIG_HTC_HEADSET
	&rhodium_h2w,
#endif
};

extern struct sys_timer msm_timer;

static void __init htcrhodium_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 122880,
//	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

/*
//TODO: use platform data
int wifi_set_power(int on, unsigned long msec) {
	if(machine_is_htcrhodium()) {
		gpio_configure(25, GPIOF_OWNER_ARM11);
		gpio_direction_output(25, 0);
		msleep(0x64);
		gpio_direction_output(25, 1);
	}
	return 0;
}

int wifi_get_irq_number(int on, unsigned long msec) {
	return gpio_to_irq(29);
}

static htc_hw_pdata_t msm_htc_hw_pdata = {
	.set_vibrate = blac_set_vibrate,
	.battery_smem_offset = 0xfc110,
	.battery_smem_field_size = 2,
};
*/

#ifdef CONFIG_MSM_SMEM_BATTCHG
static smem_batt_t htcrhodium_htc_battery_smem_pdata = {
	.gpio_battery_detect = RHODIUM_BAT_IRQ,
	.gpio_charger_enable = RHODIUM_CHARGE_EN_N,
	.gpio_charger_current_select = RHODIUM_USB_AC_PWR,
	//.gpio_ac_detect = RHODIUM_AC_DETECT,
	.smem_offset = 0xfc110,
	.smem_field_size = 2,
};
#endif

static void __init htcrhodium_init(void)
{
	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

//	msm_device_htc_hw.dev.platform_data = &msm_htc_hw_pdata;
#ifdef CONFIG_MSM_SMEM_BATTCHG
	msm_device_htc_battery_smem.dev.platform_data = &htcrhodium_htc_battery_smem_pdata;
#endif
	msm_device_touchscreen.dev.platform_data = &htcrhodium_ts_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

	msm_add_usb_devices(&htcrhodium_hsusb_board_pdata);

	init_mmc();

	msm_init_pmic_vibrator();

	msm_proc_comm_wince_vibrate_welcome();
}

static void __init htcrhodium_map_io(void)
{
	msm_map_common_io();
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init htcrhodium_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 2;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
#ifdef CONFIG_HOLES_IN_ZONE
	mi->bank[0].size = (99 * 1024 * 1024);
#else
	mi->bank[0].size = (96 * 1024 * 1024);
#endif

	mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x10000000);
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
#ifndef RHODIUM_USE_SMI2
#ifdef CONFIG_HOLES_IN_ZONE
	mi->bank[1].size = (128 - 50) * 1024 * 1024; // see pmem.c for the value
#else
	mi->bank[1].size = (128 - 52) * 1024 * 1024; // see pmem.c for the value
#endif
#else
	mi->bank[1].size = (128 * 1024 * 1024)-42*1024*1024; // See pmem.c

	//mi->nr_banks++;   // Don't make it available for allocation
	mi->bank[2].start = PAGE_ALIGN(0x02000000+(8)*1024*1024);
	mi->bank[2].node = PHYS_TO_NID(mi->bank[2].start);
	mi->bank[2].size = (32-8/*camera*/)*1024*1024;
#endif

	printk(KERN_INFO "%s: nr_banks = %d\n", __func__, mi->nr_banks);
	printk(KERN_INFO "%s: bank0 start=%08lx, node=%08x, size=%08lx\n", __func__,
		mi->bank[0].start, mi->bank[0].node, mi->bank[0].size);
	printk(KERN_INFO "%s: bank1 start=%08lx, node=%08x, size=%08lx\n", __func__,
		mi->bank[1].start, mi->bank[1].node, mi->bank[1].size);
#ifdef RHODIUM_USE_SMI2
	printk(KERN_INFO "%s: bank2 start=%08lx, node=%08x, size=%08lx\n",__func__,
		mi->bank[2].start, mi->bank[2].node, mi->bank[2].size);
#endif
}

MACHINE_START(HTCRHODIUM, "HTC Rhodium cellphone")
	.fixup = htcrhodium_fixup,
	.boot_params = 0x10000100,
	.map_io = htcrhodium_map_io,
	.init_irq = htcrhodium_init_irq,
	.init_machine = htcrhodium_init,
	.timer = &msm_timer,
MACHINE_END
