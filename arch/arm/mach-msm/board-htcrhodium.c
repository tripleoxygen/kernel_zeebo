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
#include <linux/leds.h>
#include <linux/mm.h>
#include <linux/capella_cm3602.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <linux/mmc/sdio_ids.h>
#include <asm/setup.h>
#include <mach/msm7200a_rfkill.h>
#include <mach/msm_serial_hs.h>
#if defined(CONFIG_SERIAL_BCM_BT_LPM)
#include <mach/bcm_bt_lpm.h>
#endif

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
#include <mach/htc_headset_microp.h>
#endif

#include <mach/board_htc.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/tpa2016d2.h>

#include "proc_comm_wince.h"
#include "devices.h"
#include "gpio_chip.h"
#include "board-htcrhodium.h"


extern int init_mmc(void);

/******************************************************************************
 * MicroP Keypad LEDs
 ******************************************************************************/
static struct i2c_client *kp_client = NULL;

static void htcrhodium_set_keypad_led(struct led_classdev *, enum led_brightness);
static void htcrhodium_update_keypad_led(struct work_struct* work);
static DECLARE_WORK(htcrhodium_keypad_led_wq, htcrhodium_update_keypad_led);

static struct led_classdev htcrhodium_qwerty_led = {
	 .name = "keyboard-backlight",
	 .brightness_set = htcrhodium_set_keypad_led,
	 .default_trigger = "microp-keypad",
};

static void htcrhodium_update_keypad_led(struct work_struct* work) {
	uint8_t buffer[4] = {};
	uint8_t brightness = htcrhodium_qwerty_led.brightness;
	static uint8_t last_brightness = 0;

	printk(KERN_DEBUG "%s: brightness=%d\n", __func__, brightness);

	buffer[0] = RHOD_MICROP_KSC_LED_BRIGHTNESS;
	buffer[1] = last_brightness;    /* initial brightness */
	buffer[2] = 255;                /* transition duration */
	buffer[3] = brightness;         /* target brightness */
	microp_ng_write(kp_client, buffer, 4);

	buffer[0] = RHOD_MICROP_KSC_LED_STATE;
	buffer[1] = 0;
	buffer[2] = !!brightness;
	microp_ng_write(kp_client, buffer, 3);

	last_brightness = brightness;
}

static void htcrhodium_set_keypad_led(struct led_classdev *led_cdev,
	enum led_brightness brightness) {
	schedule_work(&htcrhodium_keypad_led_wq);
}

static int htcrhodium_keypad_led_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	kp_client = dev_get_drvdata(&pdev->dev);
	led_classdev_register(&pdev->dev, &htcrhodium_qwerty_led);
	return 0;
}

static int htcrhodium_keypad_led_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	cancel_work_sync(&htcrhodium_keypad_led_wq);
	led_classdev_unregister(&htcrhodium_qwerty_led);
	kp_client = NULL;
	return 0;
}

#if CONFIG_PM
static int htcrhodium_keypad_led_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	cancel_work_sync(&htcrhodium_keypad_led_wq);
	return 0;
}
#else
#define htcrhodium_keypad_led_suspend NULL
#endif

static struct platform_driver htcrhodium_keypad_led_driver = {
	.probe		= htcrhodium_keypad_led_probe,
	.remove		= htcrhodium_keypad_led_remove,
	.suspend	= htcrhodium_keypad_led_suspend,
	.driver		= {
		.name		= "htcrhodium-microp-keypad-led",
		.owner		= THIS_MODULE,
	},
};

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

static struct platform_device htcrhodium_keypad_led = {
	.name = "htcrhodium-microp-keypad-led",
	.id = -1,
};

static struct platform_device* htcrhodium_microp_keypad_clients[] = {
	&htcrhodium_keypad,
	&htcrhodium_keypad_led,
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
 * MicroP LED, Audio, Light Sensor (Capella CM3602) & 3.5mm Headset
 ******************************************************************************/
static struct platform_device htcrhodium_microp_leds = {
	.id = -1,
	// use the topaz led setup until an LED framework is invented
	.name = "htctopaz-microp-leds",
};

static struct platform_device htcrhodium_microp_audio = {
	.id = -1,
	.name = "htcrhodium-microp-audio",
};

static struct platform_device htcrhodium_microp_ls = {
	.id = -1,
	.name = "lightsensor_microp",
};

static struct htc_headset_microp_platform_data htc_headset_microp_data = {
	.hpin_mask		= {0x00, 0x01, 0x00},	/* READ_GPI_STATE_HPIN */
	.hpin_int		= BIT_35MM_HEADSET,		/* IRQ_HEADSETIN */
	.hpin_irq		= MSM_GPIO_TO_INT(RHODIUM_GSENROR_MOT),
};

static struct platform_device htcrhodium_microp_35mm = {
	.id = -1,
	.name = "HTC_HEADSET_MICROP",
	.dev = {
		.platform_data = &htc_headset_microp_data,
	},
};

static struct platform_device* htcrhodium_microp_clients[] = {
	&htcrhodium_microp_leds,
	&htcrhodium_microp_audio,
	&htcrhodium_microp_ls,
	// this must be last; dropped when variant is not RHODW(RHOD400/RHOD500)
	&htcrhodium_microp_35mm,
};

static uint16_t micropklt_compatible_versions[] = {
	RHOD_MICROP_KLT_VERSION
};

static struct microp_platform_data htcrhodium_microp_led_audio_pdata = {
	.version_reg = RHOD_MICROP_KLT_VERSION_REG,
	.clients = htcrhodium_microp_clients,
	.nclients = ARRAY_SIZE(htcrhodium_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};

/******************************************************************************
 * USB
 ******************************************************************************/
static void htcrhodium_usb_disable(void)
{
	gpio_direction_output(RHODIUM_USBPHY_RST, 0); 
	mdelay(3);
}

static void htcrhodium_usb_enable(void)
{
	gpio_direction_output(RHODIUM_USBPHY_RST, 1);
	mdelay(3);
}

static void htcrhodium_usb_hw_reset(bool off)
{
	static bool gpio_requested = false;

	printk(KERN_WARNING "%s(%d)\n", __func__, off ? 1 : 0);

	if (!gpio_requested) {
		if (gpio_request(RHODIUM_USBPHY_RST, "RHODIUM_USBPHY_RST")) {
			printk(KERN_ERR "%s: gpio %d request failed\n", __func__,
				RHODIUM_USBPHY_RST);
		}
		gpio_requested = true;
	}

	if (off) {
		htcrhodium_usb_disable();
	} else {
		htcrhodium_usb_disable();
		htcrhodium_usb_enable();
	}
}

static struct msm_hsusb_platform_data htcrhodium_hsusb_board_pdata = {
	.hw_reset = htcrhodium_usb_hw_reset,
	.usb_connected = notify_usb_connected,
};

static struct tpa2016d2_platform_data tpa2016d2_data = {
	.gpio_tpa2016_spk_en = RHODIUM_SPKR_PWR,
};

/******************************************************************************
 * I2C
 ******************************************************************************/
static struct i2c_board_info i2c_devices[] = {
	{
		//gsensor 0x38
		I2C_BOARD_INFO("bma150", 0x70>>1),
	},
	{
		// LED & Audio controller
		I2C_BOARD_INFO("microp-ng", 0x66),
		.platform_data = &htcrhodium_microp_led_audio_pdata,
	},
	{
		// Keyboard controller
		I2C_BOARD_INFO("microp-ng", 0x67),
		.platform_data = &htcrhodium_microp_keypad_pdata,
	},
	{		
		I2C_BOARD_INFO("mt9t013", 0x6c>>1),
		/* .irq = TROUT_GPIO_TO_INT(TROUT_GPIO_CAM_BTN_STEP1_N), */
	},
	{		
		I2C_BOARD_INFO("tpa2016d2", 0xb0>>1),
		.platform_data = &tpa2016d2_data,
	},
	{		
//		I2C_BOARD_INFO("audience_A1010", 0xf4>>1),
	},
	{		
		I2C_BOARD_INFO("adc3001", 0x30>>1),
	},
};

/******************************************************************************
 * AMSS-specific stuff
 ******************************************************************************/
static struct platform_device htcrhodium_amss_device = {
	.name = "amss_6125",
	.id = -1,
};

/******************************************************************************
 * Sound
 ******************************************************************************/
extern void htcrhodium_set_headset_amp(bool enable);

static struct htc_acoustic_wce_board_data htcrhodium_acoustic_data = {
	.set_headset_amp = htcrhodium_set_headset_amp,
	.set_speaker_amp = tpa2016d2_set_power,
	.dual_mic_supported = true,
};

/******************************************************************************
 * Proximity Sensor (Capella CM3602)
 ******************************************************************************/
static int htcrhodium_capella_cm3602_power(int enable)
{
	static uint8_t gpio_requested = 0;
	int rc = 0;

	printk(KERN_DEBUG "%s(enable=%d)\n", __func__, enable);

	if (!gpio_requested) {
		rc = gpio_request(RHODIUM_GPIO_PROXIMITY_EN, "gpio_proximity_en");
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio %d request failed (%d)\n", __func__,
				RHODIUM_GPIO_PROXIMITY_EN, rc);
			return rc;
		}
		gpio_requested = 1;
	}

	if (enable) {
		rc = gpio_direction_output(RHODIUM_GPIO_PROXIMITY_EN, 1);
	} else {
		rc = gpio_direction_output(RHODIUM_GPIO_PROXIMITY_EN, 0);
	}
	return rc;
}

static struct capella_cm3602_platform_data htcrhodium_capella_cm3602_pdata = {
	.power = htcrhodium_capella_cm3602_power,
	.p_out = RHODIUM_GPIO_PROXIMITY_INT_N
};

static struct platform_device htcrhodium_capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_capella_cm3602_pdata
	}
};

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
	printk(KERN_DEBUG "%s: route=%d\n", __func__, route);
	switch (route) {
		case H2W_UART3:
			gpio_set_value(RHODIUM_H2W_UART_MUX, 1);
			break;
		case H2W_GPIO:
			gpio_set_value(RHODIUM_H2W_UART_MUX, 0);
			break;
		default:
			printk(KERN_ERR "%s: unknown route=%d\n", __func__, route);
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
	.min_press	= 0,
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

/******************************************************************************
 * Bluetooth
 ******************************************************************************/
#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(MSM7200A_UART2DM_RX),
	.inject_rx_on_wakeup = 0,
# if defined(CONFIG_SERIAL_BCM_BT_LPM)
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
# endif
};

# if defined(CONFIG_SERIAL_BCM_BT_LPM)
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = RHODIUM_BT_WAKE,
	.gpio_host_wake = RHODIUM_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off_locked,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};
# endif
#endif

static struct msm_gpio bt_init_gpio_table_rhodium[] = {
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_nRST, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_nRST" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_SHUTDOWN_N, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_SHUTDOWN_N" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_HOST_WAKE, 0,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_HOST_WAKE" },
	{	.gpio_cfg = GPIO_CFG(RHODIUM_BT_WAKE, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		.label = "RHODIUM_BT_WAKE" },
};

static int htcrhodium_bt_power(void *data, bool blocked)
{
	printk(KERN_DEBUG "%s(%s)\n", __func__, blocked ? "off" : "on");

	if (!blocked) {
		gpio_direction_output(RHODIUM_BT_nRST, 1);
		msleep(150);
		gpio_direction_output(RHODIUM_BT_SHUTDOWN_N, 1);
	} else {
		gpio_direction_output(RHODIUM_BT_SHUTDOWN_N, 0);
		msleep(150);
		gpio_direction_output(RHODIUM_BT_nRST, 0);
	}
	return 0;
}

static int htcrhodium_bt_init(struct platform_device *pdev)
{
	int rc;

	printk(KERN_DEBUG "%s\n", __func__);

	rc = msm_gpios_request(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));
	if (rc)
		goto fail_setup_gpio;

	msm_gpios_disable(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));

	return 0;

fail_setup_gpio:
	printk(KERN_ERR "%s: failed with %d\n", __func__, rc);
	return rc;
}

static void htcrhodium_bt_exit(struct platform_device *pdev) {
	printk(KERN_DEBUG "%s\n", __func__);

	msm_gpios_disable_free(bt_init_gpio_table_rhodium,
		ARRAY_SIZE(bt_init_gpio_table_rhodium));
}

#define BDADDR_STR_SIZE 18
static char bdaddr[BDADDR_STR_SIZE];

static const char* htcrhodium_get_btaddr(void) {
	unsigned char *b = (unsigned char *)(MSM_SPL_BASE + 0x81C34);
	memset(bdaddr, 0, sizeof(bdaddr));

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
		b[5], b[4], b[3], b[2], b[1], b[0]);

	return bdaddr;
}

static struct msm7200a_rfkill_pdata htcrhodium_rfkill_data = {
	.init = htcrhodium_bt_init,
	.exit = htcrhodium_bt_exit,
	.set_power = htcrhodium_bt_power,
	.get_bdaddr = htcrhodium_get_btaddr,
	.uart_number = 2,
	.rfkill_name = "bcm4325",
	.configure_bt_pcm = 1,
};

static struct platform_device htcrhodium_rfkill = {
	.name = "msm7200a_rfkill",
	.id = -1,
	.dev = {
		.platform_data = &htcrhodium_rfkill_data,
	},
};

/******************************************************************************
 * Platform devices
 ******************************************************************************/
static struct platform_device *devices[] __initdata = {
	&htcrhodium_amss_device,
	&msm_device_nand,
	&msm_device_i2c,
	&msm_device_rtc,
	&msm_device_htc_hw,
#ifdef CONFIG_MSM_SMEM_BATTCHG
	&msm_device_htc_battery_smem,
#endif
	&htcrhodium_gpio_keys,
	&htcrhodium_rfkill,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm2,
#endif
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	&bcm_bt_lpm_device,
#endif
	&msm_device_touchscreen,
	&htcrhodium_capella_cm3602,
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

#ifdef CONFIG_MSM_SMEM_BATTCHG
	msm_device_htc_battery_smem.dev.platform_data = &htcrhodium_htc_battery_smem_pdata;
#endif
	msm_device_touchscreen.dev.platform_data = &htcrhodium_ts_pdata;

	// Set acoustic device specific parameters
	htc_acoustic_wce_board_data = &htcrhodium_acoustic_data;

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
	platform_driver_register(&htcrhodium_keypad_led_driver);

	/* Don't register htc_headset_microp for non 35mm variants. */
	if (get_machine_variant_type() != MACHINE_VARIANT_RHOD_4XX
			&& get_machine_variant_type() != MACHINE_VARIANT_RHOD_5XX) {
		htcrhodium_microp_led_audio_pdata.nclients =
			ARRAY_SIZE(htcrhodium_microp_clients) - 1;
	}

	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

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
