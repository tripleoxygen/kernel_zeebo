/* linux/arch/arm/mach-msm/board-htctopaz.c
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
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
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
#include <linux/input/msm_ts.h>
#include <linux/mfd/microp-ng.h>

#include <mach/board_htc.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/htc_headset.h>

#include "proc_comm_wince.h"
#include "devices.h"
#include "board-htctopaz.h"
#include "gpio_chip.h"

extern int init_mmc(void);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(TOPA100_UART2DM_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

/******************************************************************************
 * MicroP
 ******************************************************************************/
static struct platform_device htctopaz_microp_leds = {
	.id = -1,
	.name = "htctopaz-microp-leds",
};

static struct platform_device* htctopaz_microp_clients[] = {
	&htctopaz_microp_leds,
};

static uint16_t micropklt_compatible_versions[] = {
	TOPA_MICROP_VERSION
};

static struct microp_platform_data htctopaz_microp_pdata = {
	.version_reg = TOPA_MICROP_VERSION_REG,
	.clients = htctopaz_microp_clients,
	.nclients = ARRAY_SIZE(htctopaz_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};

/******************************************************************************
 * USB
 ******************************************************************************/
static void htctopaz_usb_disable(void)
{
	gpio_set_value(TOPA100_USBPHY_RST, 0); 
	mdelay(3);
}

static void htctopaz_usb_enable(void)
{
	gpio_set_value(TOPA100_USBPHY_RST, 1);
	mdelay(3);
}

static void htctopaz_usb_hw_reset(bool off)
{
	printk(KERN_WARNING "%s(%d)\n", __func__, off ? 1 : 0);

	if (off) {
		htctopaz_usb_disable();
	} else {
		htctopaz_usb_enable();
	}
}

static struct msm_hsusb_platform_data htctopaz_hsusb_board_pdata = {
	.hw_reset = htctopaz_usb_hw_reset,
	.usb_connected = notify_usb_connected,
};

static struct i2c_board_info i2c_devices[] = {
	{
		// LED & Backlight controller
		.type = "microp-ng",
		.addr = 0x66,
		.platform_data = &htctopaz_microp_pdata,
	},
#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9P012)
	{		
		I2C_BOARD_INFO("mt9p012", 0x6c >> 1),
	},
#endif
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

	SND(0x11, "IDLE"),
	SND(256, "CURRENT"),
};
#undef SND

static struct msm_snd_endpoints htctopaz_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device htctopaz_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &htctopaz_snd_endpoints,
	},
};

/******************************************************************************
 * H2W
 ******************************************************************************/
#ifdef CONFIG_HTC_HEADSET
static void htctopaz_h2w_config_cpld(int route)
{
	printk(KERN_DEBUG "%s: route=%d\n", __func__, route);
	switch (route) {
		case H2W_UART3:
			//~ gpio_set_value(103, 1); // TODO wrong GPIO?
			break;
		case H2W_GPIO:
			//~ gpio_set_value(103, 0); // TODO wrong GPIO?
			break;
		default:
			printk(KERN_ERR "%s: unknown route=%d\n", __func__, route);
	}
}

static void htctopaz_h2w_init_cpld(void)
{
	htctopaz_h2w_config_cpld(H2W_UART3);
	gpio_set_value(TOPA100_H2W_CLK, 0);
	gpio_set_value(TOPA100_H2W_DATA, 0);
}

static struct h2w_platform_data htctopaz_h2w_pdata = {
	.cable_in1              = TOPA100_CABLE_IN1,
	.cable_in2              = TOPA100_CABLE_IN2,
	.h2w_clk                = TOPA100_H2W_CLK,
	.h2w_data               = TOPA100_H2W_DATA,
	.debug_uart             = H2W_UART3,
	.config_cpld            = htctopaz_h2w_config_cpld,
	.init_cpld              = htctopaz_h2w_init_cpld,
	.headset_mic_35mm       = TOPA100_AUD_HSMIC_DET_N,
};

static struct platform_device htctopaz_h2w = {
	.name = "h2w",
	.id = -1,
	.dev = {
		.platform_data  = &htctopaz_h2w_pdata,
	},
};
#endif

#if 0
static struct platform_device topaz_bt_rfkill = {
	.name = "htcraphael_rfkill",
	.id = -1,
};
#endif

static struct ts_virt_key htctopaz_ts_keys_y[] = {
	// key      min   max
	{KEY_UP,    105, 267}, //  420, 1068},
	{KEY_DOWN,  268, 429}, // 1069, 1716},
	{KEY_HOME,  430, 591}, // 1717, 2364},
	{KEY_LEFT,  592, 753}, // 2365, 3012},
	{KEY_RIGHT, 754, 915}, // 3013, 3660},
};

static struct msm_ts_virtual_keys htctopaz_ts_virtual_keys_y = {
	.keys = &htctopaz_ts_keys_y[0],
	.num_keys = 5,
};

static struct msm_ts_platform_data htctopaz_ts_pdata = {
	.min_x		= 105, // 420,
	.max_x		= 915, // 3660,
	.min_y		= 0, // 0,
	.max_y		= 837, // 3350,
	.min_press	= 0,
	.max_press	= 256,
	.inv_x		= 0,
	.inv_y		= 970, // 3880,
	.virt_y_start = 862, // 3450, // 3350 + space
	.vkeys_y	= &htctopaz_ts_virtual_keys_y,
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
	&htctopaz_snd,
#ifdef CONFIG_HTC_HEADSET
	&htctopaz_h2w,
#endif
//	&topaz_bt_rfkill,
	&msm_device_touchscreen,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
	&acoustic_device,
};

extern struct sys_timer msm_timer;

static void __init htctopaz_init_irq(void)
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

#if 0
static htc_hw_pdata_t msm_htc_hw_pdata = {
	.set_vibrate = topaz_set_vibrate,
	.battery_smem_offset = 0xfc110,
	.battery_smem_field_size = 2,
};
#endif

#ifdef CONFIG_MSM_SMEM_BATTCHG
static smem_batt_t htctopaz_htc_battery_smem_pdata = {
	.gpio_battery_detect = TOPA100_BAT_IN,
	.gpio_charger_enable = TOPA100_CHARGE_EN_N,
	.gpio_charger_current_select = TOPA100_USB_AC_PWR,
//	.gpio_ac_detect = -1,
	.smem_offset = 0xfc110,
	.smem_field_size = 2,
};
#endif

static void __init htctopaz_init(void)
{
	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

//	msm_device_htc_hw.dev.platform_data = &msm_htc_hw_pdata;
#ifdef CONFIG_MSM_SMEM_BATTCHG
	msm_device_htc_battery_smem.dev.platform_data = &htctopaz_htc_battery_smem_pdata;
#endif
	msm_device_touchscreen.dev.platform_data = &htctopaz_ts_pdata;

	// Set acoustic device specific parameters
	acoustic_device.dev.platform_data = &amss_6120_acoustic_data;

	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

	msm_add_usb_devices(&htctopaz_hsusb_board_pdata);

	init_mmc();

	msm_init_pmic_vibrator();

	msm_proc_comm_wince_vibrate_welcome();
}

static void __init htctopaz_map_io(void)
{
	msm_map_common_io();
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init htctopaz_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
//	parse_tag_monodie((const struct tag *)tags);

	mi->nr_banks = 2;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
	mi->bank[0].size = (104 * 1024 * 1024);

//	if (board_mcp_monodie()) {
//		mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x08000000);
//	} else {
		mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x10000000);
//	}
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
#ifdef CONFIG_HOLES_IN_ZONE
	mi->bank[1].size = (128 - 50) * 1024 * 1024; // see pmem.c for the value
#else
	mi->bank[1].size = (128 - 52) * 1024 * 1024; // see pmem.c for the value
#endif

	printk(KERN_INFO "%s: nr_banks = %d\n", __func__, mi->nr_banks);
	printk(KERN_INFO "%s: bank0 start=%08lx, node=%08x, size=%08lx\n", __func__,
		mi->bank[0].start, mi->bank[0].node, mi->bank[0].size);
	printk(KERN_INFO "%s: bank1 start=%08lx, node=%08x, size=%08lx\n", __func__,
		mi->bank[1].start, mi->bank[1].node, mi->bank[1].size);
}

MACHINE_START(HTCTOPAZ, "HTC Topaz cellphone (Topaz is a silicate mineral of aluminium and fluorine)")
	.fixup = htctopaz_fixup,
	.boot_params = 0x10000100,
	.map_io = htctopaz_map_io,
	.init_irq = htctopaz_init_irq,
	.init_machine = htctopaz_init,
	.timer = &msm_timer,
MACHINE_END
