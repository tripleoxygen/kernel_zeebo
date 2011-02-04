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
//#include <mach/htc_battery.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/msm_fb.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_ts.h>
#include <mach/vreg.h>

#include <mach/gpio.h>
#include <mach/io.h>
#include <linux/delay.h>
#include <linux/gpio_keys.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

//#include <linux/microp-keypad.h>
#include <mach/board_htc.h>
#include <mach/htc_headset.h>

#include "proc_comm_wince.h"
#include "devices.h"
//#include "htc_hw.h"
#include "board-htctopaz.h"
//#include "htc-usb.h"
#include "gpio_chip.h"

//static void htctopaz_device_specific_fixes(void);

extern int init_mmc(void);

#if 0
static struct resource msm_serial0_resources[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

#if 0
static struct platform_device msm_serial0_device = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_serial0_resources),
	.resource	= msm_serial0_resources,
};
#endif

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(21),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

#if 0
static int usb_phy_init_seq_raph100[] = {
	0x40, 0x31, /* Leave this pair out for USB Host Mode */
	0x1D, 0x0D,
	0x1D, 0x10,
	-1
};
#endif

static unsigned ulpi_on_gpio_table[] = {
	GPIO_CFG(0x6f, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static unsigned ulpi_off_gpio_table[] = {
	GPIO_CFG(0x6f, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x70, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x71, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x72, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x73, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x74, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x75, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x76, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x77, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x78, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(0x79, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
};

static void usb_gpio_init(void)
{
	if (gpio_request(0x6f, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(0x70, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(0x71, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(0x72, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(0x73, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(0x74, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(0x75, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(0x76, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(0x77, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(0x78, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(0x79, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_on_gpio_table); pin++) {
			rc = gpio_tlmm_config(ulpi_on_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_off_gpio_table[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(ulpi_off_gpio_table); pin++) {
			rc = gpio_tlmm_config(ulpi_off_gpio_table[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, ulpi_on_gpio_table[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}

static int usb_phy_init_seq_topa[] = {
	0x40, 0x31, /* Leave this pair out for USB Host Mode */
	0x1D, 0x0D,
	0x1D, 0x10,
	0x5, 0xA,
	-1
};

static void usb_phy_shutdown(void)
{
	printk("%s: %s\n", __FILE__, __func__);
	gpio_set_value(TOPA100_USBPHY_RST, 1); 
	gpio_set_value(TOPA100_USBPHY_RST, 0);
}
static void usb_phy_reset(void)
{
	printk("%s: %s\n", __FILE__, __func__);
	usb_phy_shutdown();
	gpio_set_value(TOPA100_USBPHY_RST, 0); 
	mdelay(3);
	gpio_set_value(TOPA100_USBPHY_RST, 1);
	mdelay(3);
	usb_config_gpio(1);
}

static void usb_hw_reset(bool enable) {
	printk("%s: %s what to do?. FIXME !!!\n", __FILE__, __func__);
}

static void usb_connected(int on) {
	printk(KERN_DEBUG "%s(%d)\n", __func__, on);

	switch (on) {
	case 2: /* ac power? */

	break;
	case 1:	/* usb plugged in */
		//notify_usb_connected(1);
	break;
	case 0:
		notify_usb_connected(0);
		usb_config_gpio(0);
	break;
	default:
		printk(KERN_WARNING "%s: FIXME! value for ON? %u ?\n", __func__, on);
	}
	
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq	= usb_phy_init_seq_topa,
	.phy_reset     	= usb_phy_reset,
	.hw_reset	= usb_hw_reset,
	.usb_connected	= usb_connected,
};
/***************************************************************
 * Android usb defines should be removed as google did for .35
 * All usb stuff should be moved to new usb platform that has
 * yet to be made. For debugging keep in board file for now.
 ***************************************************************/
#ifdef CONFIG_USB_ANDROID
static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},	
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0fff,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "XDA",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif


static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c01,
	.version	= 0x0100,
	.serial_number		= "000000000000",
	.product_name		= "XDA",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

/***************************************************************
 * End of android stuff
 ***************************************************************/

static struct i2c_board_info i2c_devices[] = {
	{
		// LED & Backlight controller
		I2C_BOARD_INFO("microp-klt", 0x66),
	},
	{		
		I2C_BOARD_INFO("mt9t013", 0x6c>>1),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
};

#if 0
#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

};
#undef SND

static struct msm_snd_endpoints topaz_snd_endpoints = {
        .endpoints = snd_endpoints_list,
        .num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device topaz_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev	= {
		.platform_data = &topaz_snd_endpoints,
	},
};
#endif

#if 0
#ifdef CONFIG_HTC_HEADSET
static void h2w_config_cpld(int route);
static void h2w_init_cpld(void);
static struct h2w_platform_data topaz_h2w_data = {
	.cable_in1              = TOPA100_CABLE_IN1,
	.cable_in2              = TOPA100_CABLE_IN2,
	.h2w_clk                = TOPA100_H2W_CLK,
	.h2w_data               = TOPA100_H2W_DATA,
	.debug_uart             = H2W_UART3,
	.config_cpld            = h2w_config_cpld,
	.init_cpld              = h2w_init_cpld,
	.headset_mic_35mm       = TOPA100_AUD_HSMIC_DET_N,
};

static void h2w_config_cpld(int route)
{
	printk(KERN_INFO "htctopaz %s: route=%d TODO\n",
		__func__, route);
	switch (route) {
		case H2W_UART3:
			//gpio_set_value(0, 1); 	/*TODO wrong GPIO*/
			break;
		case H2W_GPIO:
			//gpio_set_value(0, 0); 	/*TODO wrong GPIO*/
			break;
	}
}

static void h2w_init_cpld(void)
{
	h2w_config_cpld(H2W_UART3);
	gpio_set_value(topaz_h2w_data.h2w_clk, 0);
	gpio_set_value(topaz_h2w_data.h2w_data, 0);
}

static struct platform_device topaz_h2w = {
	.name           = "h2w",
	.id             = -1,
	.dev            = {
		.platform_data  = &topaz_h2w_data,
	},
};
#endif
#endif

#if 0
static struct platform_device topaz_bt_rfkill = {
	.name = "htcraphael_rfkill",
	.id = -1,
};
#endif

struct ts_virt_key ts_keys_y[] = {
	// 3240 x-range / 5 = 648
	// key      min   max
	{KEY_UP,     420, 1068},
	{KEY_DOWN,  1069, 1716},
	{KEY_HOME,  1717, 2364},
	{KEY_LEFT,  2365, 3012},
	{KEY_RIGHT, 3013, 3660},
};

static struct msm_ts_virtual_keys htctopaz_ts_virtual_keys_y = {
	.keys = &ts_keys_y[0],
	.num_keys = 5,
};

static struct msm_ts_platform_data htctopaz_ts_pdata = {
	.min_x		= 420,
	.max_x		= 3660,
	.min_y		= 0,
	.max_y		= 3350,
	.min_press	= 0,
	.max_press	= 256,
	.inv_x		= 0,
	.inv_y		= 3880,
	.virt_y_start = 3450, // 3350 + space
	.vkeys_y	= &htctopaz_ts_virtual_keys_y,
};

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_rtc,
	&msm_device_hsusb,
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&rndis_device,
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	&usb_mass_storage_device,
#endif
//	&msm_device_htc_hw,
//	&msm_device_htc_battery,
//	&topaz_snd,
#ifdef CONFIG_HTC_HEADSET
//	&topaz_h2w,
#endif
//	&topaz_bt_rfkill,
	&msm_device_touchscreen,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
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
	.wait_for_irq_khz = 19200,
//	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

static void htctopaz_reset(void)
{
	struct msm_dex_command dex = { .cmd = PCOM_NOTIFY_ARM9_REBOOT };
	msm_proc_comm_wince(&dex, 0);
	msleep(0x15e);
	gpio_request(25, "MSM Reset");
	msm_gpio_set_flags(25, GPIOF_OWNER_ARM11);
	gpio_direction_output(25, 0);
	printk(KERN_INFO "%s: Soft reset done.\n", __func__);
}

static void htctopaz_set_vibrate(uint32_t val)
{
	struct msm_dex_command vibra;

	if (val == 0) {
		vibra.cmd = PCOM_VIBRA_OFF;
		msm_proc_comm_wince(&vibra, 0);
	} else if (val > 0) {
		if (val == 1 || val > 0xb22)
			val = 0xb22;
		writel(val, MSM_SHARED_RAM_BASE + 0xfc130);
		vibra.cmd = PCOM_VIBRA_ON;
		msm_proc_comm_wince(&vibra, 0);
	}
}

#if 0
static htc_hw_pdata_t msm_htc_hw_pdata = {
	.set_vibrate = topaz_set_vibrate,
	.battery_smem_offset = 0xfc140,
	.battery_smem_field_size = 4,
};

static smem_batt_t msm_battery_pdata = {
	.gpio_battery_detect = TOPA100_BAT_IN,
	.gpio_charger_enable = TOPA100_CHARGE_EN_N,
	.gpio_charger_current_select = TOPA100_USB_AC_PWR,
	.smem_offset = 0xfc140,
	.smem_field_size = 4,
};
#endif

static void __init htctopaz_init(void)
{
	int i;

	// Fix data in arrays depending on GSM/CDMA version
//	htctopaz_device_specific_fixes();

	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

	msm_hw_reset_hook = htctopaz_reset;

//	msm_device_htc_hw.dev.platform_data = &msm_htc_hw_pdata;
//	msm_device_htc_battery.dev.platform_data = &msm_battery_pdata;
	
	msm_device_touchscreen.dev.platform_data = &htctopaz_ts_pdata;
	
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

	msm_device_hsusb.dev.platform_data = &msm_hsusb_pdata;
	usb_gpio_init();
	/* TODO: detect vbus and correctly notify USB about its presence 
	 * For now we just declare that VBUS is present at boot and USB
	 * copes, but this is not ideal.
	 */
	msm_hsusb_set_vbus_state(1);

	init_mmc();

	msm_init_pmic_vibrator();

	/* A little vibrating welcome */
	for (i = 0; i < 2; i++) {
		htctopaz_set_vibrate(1);
		mdelay(150);
		htctopaz_set_vibrate(0);
		mdelay(75);
	}
}

static void __init htctopaz_map_io(void)
{
	msm_map_common_io();
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init htctopaz_fixup(struct machine_desc *desc, struct tag *tags,
                                    char **cmdline, struct meminfo *mi)
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
	mi->bank[1].size = (128 * 1024 * 1024)-34*1024*1024;//See pmem.c for the value

	printk(KERN_INFO "fixup: nr_banks = %d\n", mi->nr_banks);
	printk(KERN_INFO "fixup: bank0 start=%08lx, node=%08x, size=%08lx\n", mi->bank[0].start, mi->bank[0].node, mi->bank[0].size);
	printk(KERN_INFO "fixup: bank1 start=%08lx, node=%08x, size=%08lx\n", mi->bank[1].start, mi->bank[1].node, mi->bank[1].size);
}

#if 0
static void htctopaz_device_specific_fixes(void)
{
	msm_htc_hw_pdata.battery_smem_offset = 0xfc110;
	msm_htc_hw_pdata.battery_smem_field_size = 2;
	msm_battery_pdata.smem_offset = 0xfc110;
	msm_battery_pdata.smem_field_size = 2;
}
#endif

MACHINE_START(HTCTOPAZ, "HTC Topaz cellphone (Topaz is a silicate mineral of aluminium and fluorine)")
	.fixup = htctopaz_fixup,
	.boot_params = 0x10000100,
	.map_io = htctopaz_map_io,
	.init_irq = htctopaz_init_irq,
	.init_machine = htctopaz_init,
	.timer = &msm_timer,
MACHINE_END
