/* linux/arch/arm/mach-msm/board-htctopaz-led.c
 *
 * Copyright (2011) Michael Weirauch <mweirauch@xdandroid.com>
 * 
 * Based on leds-microp-htckovsky.c:
 * Copyright (2010) Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * Some code was written by ultrashot at xda-developers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/mfd/microp-ng.h>

#include <asm/mach-types.h>

#include "board-htctopaz.h"

#define DEBUG 0

#if DEBUG
 #define D(fmt, arg...) printk(KERN_DEBUG "%s: " fmt, __FUNCTION__, ## arg);
#else
 #define D(fmt, arg...) do {} while(0)
#endif
#define E(fmt, arg...) printk(KERN_ERR "%s: " fmt, __FUNCTION__, ## arg);

static void htctopaz_update_color_led(struct work_struct* work);
static void htctopaz_update_lcd_backlight(struct work_struct* work);
static void htctopaz_update_button_backlight(struct work_struct* work);
static void htctopaz_update_keyboard_backlight(struct work_struct* work);
static void htctopaz_update_meta_key_brightness(struct work_struct* work);

static void htctopaz_set_color_led(struct led_classdev*, enum led_brightness);
static void htctopaz_set_lcd_backlight(struct led_classdev*, enum led_brightness);
static void htctopaz_set_button_backlight(struct led_classdev*, enum led_brightness);
static void htctopaz_set_keyboard_backlight(struct led_classdev*, enum led_brightness);
static void htctopaz_set_meta_key_brightness(struct led_classdev*, enum led_brightness);

static int htctopaz_set_color_led_blink(struct led_classdev*, unsigned long*, unsigned long*);

static DECLARE_WORK(colorled_wq, htctopaz_update_color_led);
static DECLARE_WORK(backlight_wq, htctopaz_update_lcd_backlight);
static DECLARE_WORK(buttonlight_wq, htctopaz_update_button_backlight);
static DECLARE_WORK(keyboardlight_wq, htctopaz_update_keyboard_backlight);
static DECLARE_WORK(meta_key_wq, htctopaz_update_meta_key_brightness);
static struct i2c_client *klt_client = NULL;
static struct i2c_client *ksc_client = NULL;
static uint8_t g_auto_backlight = 0;
static uint8_t g_blink_status = 0;

static unsigned int led_regime=0;
module_param_named(led_regime, led_regime, int, S_IRUGO | S_IWUSR | S_IWGRP);

enum led_color {
	COLOR_OFF = 0,
	COLOR_GREEN = 1,
	COLOR_AMBER = 2,
	BLINK_GREEN = 4,
	BLINK_AMBER = 5,
};

#if DEBUG
static char* led_color_name(enum led_color color) {
	switch(color) {
		case COLOR_OFF:
			return "OFF";
		case COLOR_GREEN:
			return "GREEN";
		case COLOR_AMBER:
			return "AMBER";
		case BLINK_GREEN:
			return "BLINK GREEN";
		case BLINK_AMBER:
			return "BLINK AMBER";
		default:
			return "UNKNOWN";
	}
}
#endif

enum supported_led {AMBER, GREEN, LCD, BUTTONS, KEYBOARD, CAPS, FUNC};

static struct led_classdev htctopaz_leds[] = {
	[AMBER] = {
		.name = "amber",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_color_led,
		.blink_set = htctopaz_set_color_led_blink,
	},
	[GREEN] = {
		.name = "green",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_color_led,
		.blink_set = htctopaz_set_color_led_blink,
	},
	[LCD] = {
		.name = "lcd-backlight",
		.brightness = LED_FULL,
		.brightness_set = htctopaz_set_lcd_backlight,
	},
	[BUTTONS] = {
		.name = "button-backlight",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_button_backlight,
	},
	[KEYBOARD] = {
		.name = "keyboard-backlight",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_keyboard_backlight,
		.default_trigger = "microp-keypad",
	},
	[CAPS] = {
		.name = "caps",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_meta_key_brightness,
	},
	[FUNC] = {
		.name = "func",
		.brightness = LED_OFF,
		.brightness_set = htctopaz_set_meta_key_brightness,
	},
};

/*
 * MicropP functions
 */

static int microp_led_set_color_led(enum led_color front_led_color,
	enum led_color back_led_color)
{
	int ret;
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	if (!klt_client) {
		return -EAGAIN;
	}

	D("front=%s back=%s\n", led_color_name(front_led_color),
		led_color_name(back_led_color));

	if (machine_is_htctopaz()) {
		buf[0] = TOPA_MICROP_COLOR_LED_ADDRESS; // 0x51
	} else {
		buf[0] = 0x50; // RHOD
	}
	buf[1] = back_led_color;
	buf[2] = front_led_color;
	buf[3] = 0xff;
	buf[4] = 0xff;

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret) {
		E("Failed setting color led value (%d)\n", ret);
	}
	return ret;
}

static int microp_led_set_lcd_backlight(enum led_brightness brightness)
{
	int ret;
	uint8_t buf[2] = { 0, 0 };

	if (!klt_client) {
		return -EAGAIN;
	}

	D("brightness=%d\n", brightness);

	if (machine_is_htctopaz()) {
		buf[0] = MICROP_I2C_WCMD_LCD_BRIGHTNESS; // 0x22
	} else {
		buf[0] = 0x24; // RHOD
	}
	buf[1] = brightness / 2 & 0xf0; // should be revised

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret) {
		E("Failed setting lcd backlight value (%d)\n", ret);
	}
	return ret;
}

static int microp_led_set_button_backlight(enum led_brightness brightness)
{
	int ret;
	unsigned state;
	uint8_t buf[3] = { 0, 0, 0 };
	struct i2c_client *client_to_use = NULL;

	D("brightness=%d\n", brightness);

	// TODO this needs to be tracked
	state = brightness ? 0x01 : 0x00;

	if (machine_is_htctopaz()) {
		client_to_use = klt_client;
		buf[0] = 0x40; // MICROP_KLT_ID_LED_STATE;
		buf[1] = 0xff & state;
		buf[2] = 0xff & (state >> 8);
	} else {
		client_to_use = ksc_client;
		buf[0] = 0x14; // MICROP_KSC_ID_KEYPAD_LIGHT_RHOD;
		buf[1] = 0x80; // brightness? variable?
		buf[2] = state; // just on/off?
	}

	if (!client_to_use) {
		return -EAGAIN;
	}

	ret = microp_ng_write(client_to_use, buf, ARRAY_SIZE(buf));
	if (ret) {
		E("Failed setting button backlight value (%d)\n", ret);
	}
	return ret;
}

static int microp_led_set_keyboard_backlight(enum led_brightness brightness)
{
	int ret;
	uint8_t buffer[4] = {};
	static uint8_t last_brightness = 0;

	if (machine_is_htctopaz()) {
		return -ENODEV;
	}

	if (!ksc_client) {
		return -EAGAIN;
	}

	D("brightness=%d\n", brightness);

	buffer[0] = 0x32; // RHOD_MICROP_KSC_LED_BRIGHTNESS;
	buffer[1] = last_brightness;    /* initial brightness */
	buffer[2] = 255;                /* transition duration */
	buffer[3] = brightness;         /* target brightness */
	ret = microp_ng_write(ksc_client, buffer, 4);
	if (ret) {
		printk(KERN_ERR "%s: Failed setting button backlight value (%d)\n",
			__func__, ret);
		return ret;
	}

	buffer[0] = 0x30; // RHOD_MICROP_KSC_LED_STATE;
	buffer[1] = 0;
	buffer[2] = !!brightness;
	microp_ng_write(ksc_client, buffer, 3);
	if (ret) {
		E("Failed setting button backlight value (%d)\n", ret);
		return ret;
	}
	last_brightness = brightness;
	return ret;
}

static int microp_led_set_meta_key_backlight(enum led_brightness caps_brightness,
	enum led_brightness func_brightness)
{
	static struct meta_led {
		uint8_t state:2;
	} meta_led;
	int ret;
	uint8_t buf[2] = { 0, 0 };
	unsigned int bit;

	if (machine_is_htctopaz()) {
		return -ENODEV;
	}

	if (!klt_client) {
		return -EAGAIN;
	}

	D("caps=%d func=%d\n", caps_brightness, func_brightness);

	bit = (1<<0); //RHOD_MICROP_KSC_LED_CAPS;
	if (htctopaz_leds[CAPS].brightness)
		meta_led.state |= bit;
	else
		meta_led.state &=~bit;

	bit = (1<<1); //RHOD_MICROP_KSC_LED_FN;
	if (htctopaz_leds[FUNC].brightness)
		meta_led.state |= bit;
	else
		meta_led.state &=~bit;

	buf[0] = 0x25; // MICROP_KLT_RHOD_LED_STATE
	buf[1] = meta_led.state;

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret) {
		E("Failed setting meta key backlight value (%d)\n", ret);
	}
	return ret;
}

static int microp_led_set_auto_backlight(int val)
{
	int ret;
	uint8_t buf[3] = { 0, 0, 0 };

	if (!klt_client) {
		return -EAGAIN;
	}

	D("%s (%d)\n", val ? "on" : "off", val);

	if (machine_is_htctopaz()) {
		buf[0] = MICROP_I2C_WCMD_AUTO_BL_CTL; // 0x23
		buf[1] = val ? 0x01 : 0x00;
		buf[2] = val ? 0x01 : 0x00;
	} else {
		buf[0] = 0x22; // RHOD
		buf[1] = val ? 0xf3 : 0xf8;
		buf[2] = val ? 0xf3 : 0xf8;
	}

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret) {
		E("Failed writing auto backlight status (%d)\n", ret);
	}
	return ret;
}

static int microp_led_get_spi_auto_backlight_status(
	uint8_t* auto_backlight_status, uint8_t* spi_status)
{
	int ret;
	uint8_t cmd;
	uint8_t buf[2] = { 0, 0 };

	if (!klt_client) {
		return -EAGAIN;
	}

	cmd = machine_is_htctopaz() ? MICROP_I2C_RCMD_SPI_BL_STATUS /*0x24*/ : 0x21; // RHOD testing

	ret = microp_ng_read(klt_client, cmd, buf, 2);
	if (ret) {
		printk(KERN_ERR "%s: Failed reading SPI backlight status with ret=%d.\n",
			__func__, ret);
		return ret;
	}

	printk(KERN_INFO "%s: auto_backlight=0x%02x SPI=0x%02x\n", __func__,
		buf[0], buf[1]);

	*auto_backlight_status = buf[0];
	*spi_status = buf[1];

	return ret;
}

#if 0
static int microp_spi_enable(uint8_t on)
{
	int ret;
	uint8_t buf[2] = { 0, 0 };

	if (!klt_client) {
		return -EAGAIN;
	}

	printk(KERN_INFO "%s(%u)\n", __func__, on);

	buf[0] = MICROP_I2C_WCMD_SPI_EN;
	buf[1] = on;

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed %s SPI bus. (%d)\n", __func__,
			on ? "enabling" : "disabling", ret);
		return ret;
	}

	return ret;
}
#endif

/*
 * Worker functions
 */

static void htctopaz_update_color_led(struct work_struct* work)
{
	enum led_brightness brightness_amber = htctopaz_leds[AMBER].brightness;
	enum led_brightness brightness_green = htctopaz_leds[GREEN].brightness;

	// give amber precedence over green
	if (brightness_amber) {
		microp_led_set_color_led(g_blink_status ? BLINK_AMBER : COLOR_AMBER,
			COLOR_OFF);
	} else if (brightness_green) {
		microp_led_set_color_led(g_blink_status ? BLINK_GREEN : COLOR_GREEN,
			COLOR_OFF);
	} else {
		microp_led_set_color_led(COLOR_OFF, COLOR_OFF);
	}
}

static void htctopaz_update_lcd_backlight(struct work_struct* work)
{
	enum led_brightness brightness = htctopaz_leds[LCD].brightness;

	microp_led_set_lcd_backlight(brightness);
}

static void htctopaz_update_button_backlight(struct work_struct* work)
{
	enum led_brightness brightness = htctopaz_leds[BUTTONS].brightness;

	microp_led_set_button_backlight(brightness);
}

static void htctopaz_update_keyboard_backlight(struct work_struct* work)
{
	enum led_brightness brightness = htctopaz_leds[KEYBOARD].brightness;

	microp_led_set_keyboard_backlight(brightness);
}

static void htctopaz_update_meta_key_brightness(struct work_struct* work)
{
	enum led_brightness caps_brightness = htctopaz_leds[CAPS].brightness;
	enum led_brightness func_brightness = htctopaz_leds[FUNC].brightness;

	microp_led_set_meta_key_backlight(caps_brightness, func_brightness);
}

/*
 * dev_attr_auto_backlight for lcd-backlight led device
 */

static ssize_t htctopaz_auto_backlight_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	int ret;
	uint8_t auto_backlight_status;
	uint8_t spi_status;

	ret = microp_led_get_spi_auto_backlight_status(&auto_backlight_status, &spi_status);
	if (ret) {
		return ret;
	}

	g_auto_backlight = auto_backlight_status ? 1 : 0;

	return sprintf(ret_buf, "%u\n", g_auto_backlight);
}

static ssize_t htctopaz_auto_backlight_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);

	if (microp_led_set_auto_backlight(val)) {
		return count;
	}

	g_auto_backlight = val ? 1 : 0;
	return count;
}

static DEVICE_ATTR(auto_backlight, 0644, htctopaz_auto_backlight_get,
	htctopaz_auto_backlight_set);

/*
 * dev_attr_blink for amber and green led device
 */

static ssize_t htctopaz_color_led_blink_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	return sprintf(ret_buf, "%u\n", g_blink_status);
}

static ssize_t htctopaz_color_led_blink_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (!led_cdev->blink_set) {
		printk(KERN_ERR "%s: led device %s has no blink_set assigned!\n",
			__func__, led_cdev->name);
		return -ENODATA;
	}

	g_blink_status = val ? 1 : 0;

	led_cdev->blink_set(led_cdev, 0, 0);
	return count;
}

static DEVICE_ATTR(blink, 0644, htctopaz_color_led_blink_get,
	htctopaz_color_led_blink_set);

/*
 * led_cdev brightness_set callbacks
 */

static void htctopaz_set_color_led(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&colorled_wq);
}

static int htctopaz_set_color_led_blink(struct led_classdev* led_cdev,
	unsigned long *delay_on, unsigned long* delay_off)
{
	return schedule_work(&colorled_wq);
}

static void htctopaz_set_lcd_backlight(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&backlight_wq);
}

static void htctopaz_set_button_backlight(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&buttonlight_wq);
}

static void htctopaz_set_keyboard_backlight(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&keyboardlight_wq);
}

static void htctopaz_set_meta_key_brightness(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&meta_key_wq);
}

/*
 * Init
 */

static int htctopaz_microp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	uint8_t auto_backlight_status;
	uint8_t spi_status;

	printk(KERN_INFO "%s\n", __func__);
	klt_client = dev_get_drvdata(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(htctopaz_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &htctopaz_leds[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed registering led %s", __func__,
				htctopaz_leds[i].name);
			goto led_fail;
		}
		if (htctopaz_leds[i].blink_set) {
			ret = device_create_file(htctopaz_leds[i].dev, &dev_attr_blink);
			if (ret < 0) {
				printk(KERN_ERR "%s: Failed registering blink attribute %s",
					__func__, htctopaz_leds[i].name);
				goto led_fail;
			}
		}
	}

	ret = device_create_file(htctopaz_leds[LCD].dev, &dev_attr_auto_backlight);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed registering auto_backlight attribute (%d)",
			__func__, ret);
		goto led_fail;
	}

	// init auto backlight global
	ret = microp_led_get_spi_auto_backlight_status(&auto_backlight_status,
		&spi_status);
	if (ret) {
		printk(KERN_ERR "%s: Failed reading auto_backlight status (%d)",
			__func__, ret);
		goto attr_fail;
	}
	g_auto_backlight = auto_backlight_status;

	// some defaults at boot
	microp_led_set_color_led(led_regime ? COLOR_AMBER : COLOR_OFF, COLOR_OFF);
	microp_led_set_button_backlight(LED_OFF);
	microp_led_set_lcd_backlight(LED_FULL);
	microp_led_set_keyboard_backlight(LED_OFF);
	microp_led_set_meta_key_backlight(LED_OFF, LED_OFF);

	return 0;

attr_fail:
	device_remove_file(htctopaz_leds[LCD].dev, &dev_attr_auto_backlight);
led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&htctopaz_leds[i]);
		if (htctopaz_leds[i].blink_set) {
			device_remove_file(htctopaz_leds[i].dev, &dev_attr_blink);
		}
	}
	return ret;
}

static int htctopaz_microp_remove(struct platform_device *pdev)
{
	int i;

	device_remove_file(htctopaz_leds[LCD].dev, &dev_attr_auto_backlight);

	for (i = 0; i < ARRAY_SIZE(htctopaz_leds); i++) {
		led_classdev_unregister(&htctopaz_leds[i]);
	}
	klt_client = NULL;
	return 0;
}

#if CONFIG_PM
static int htctopaz_microp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	// set color led to OFF when sleeping in led_regime mode 1
	if (led_regime == 1) {
		microp_led_set_color_led(COLOR_OFF, COLOR_OFF);
	}
	// set color led to GREEN when sleeping in led_regime mode 2
	if (led_regime == 2) {
		microp_led_set_color_led(COLOR_GREEN, COLOR_OFF);
	}

	//microp_spi_enable(0);

	return 0;
}

static int htctopaz_microp_resume(struct platform_device *pdev)
{
	//microp_spi_enable(1);

	// set color led to AMBER when awake in led_regime mode 1 or 2
	if (led_regime) {
		microp_led_set_color_led(COLOR_AMBER, COLOR_OFF);
	}

	return 0;
}
#else
#define htctopaz_microp_suspend NULL
#define htctopaz_microp_resume NULL
#endif

static struct platform_driver microp_led_klt_driver = {
	.probe		= htctopaz_microp_probe,
	.remove		= htctopaz_microp_remove,
	.suspend	= htctopaz_microp_suspend,
	.resume		= htctopaz_microp_resume,
	.driver		= {
		.name		= "microp-led-klt",
		.owner		= THIS_MODULE,
	},
};

static int microp_led_ksc_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	ksc_client = dev_get_drvdata(&pdev->dev);
	return 0;
}

static int microp_led_ksc_remove(struct platform_device *pdev)
{
	ksc_client = NULL;
	return 0;
}

static struct platform_driver microp_led_ksc_driver = {
	.probe		= microp_led_ksc_probe,
	.remove		= microp_led_ksc_remove,
	.driver		= {
		.name		= "microp-led-ksc",
		.owner		= THIS_MODULE,
	},
};

static int __init microp_led_init(void)
{
	int rc;

	rc = platform_driver_register(&microp_led_klt_driver);
	if (rc) {
		return rc;
	}
	return platform_driver_register(&microp_led_ksc_driver);
}

static void __exit microp_led_exit(void)
{
	platform_driver_unregister(&microp_led_klt_driver);
	platform_driver_unregister(&microp_led_ksc_driver);
}

module_init(microp_led_init);
module_exit(microp_led_exit);

/*
 * DebugFS
 */

#if defined(CONFIG_DEBUG_FS)
static int microp_dbg_led_set_color_led(void *dat, u64 val)
{
	int ret;
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	unsigned back = 0xff & (val >> 24);
	unsigned front = 0xff & (val >> 16);
	unsigned three = 0xff & (val >> 8);
	unsigned four = 0xff & val;

	if (!klt_client) {
		return -EAGAIN;
	}

	printk(KERN_DEBUG "%s: val=0x%08x (%llu) back=%d front=%d three=%d four=%d\n",
		__func__, (unsigned)val, val, back, front, three, four);

	if (machine_is_htctopaz()) {
		buf[0] = TOPA_MICROP_COLOR_LED_ADDRESS; // 0x51
	} else {
		buf[0] = 0x50; // RHOD
	}
	buf[1] = back;
	buf[2] = front;
	buf[3] = three;
	buf[4] = four;

	ret = microp_ng_write(klt_client, buf, ARRAY_SIZE(buf));
	if (ret) {
		printk(KERN_ERR "%s: Failed setting color led value (%d)\n",
			__func__, ret);
	}
	return ret;
}

static int microp_dbg_led_get_color_led(void *data, u64 *val) {
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(microp_dbg_led_set_color_led_fops,
	microp_dbg_led_get_color_led,
	microp_dbg_led_set_color_led, "%llu\n");

static int __init microp_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("microp_dbg", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("color_led", 0666, dent, NULL,
			&microp_dbg_led_set_color_led_fops);

	return 0;
}

device_initcall(microp_dbg_init);
#endif /* CONFIG_DEBUG_FS */
