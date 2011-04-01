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
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/mfd/microp-ng.h>

#include "board-htctopaz.h"

static void htctopaz_update_color_leds(struct work_struct* work);
static void htctopaz_microp_update_backlight(struct work_struct* work);
static void htctopaz_update_button_light(struct work_struct* work);

static void htctopaz_set_backlight(struct led_classdev*, enum led_brightness);
static void htctopaz_set_button_light(struct led_classdev*, enum led_brightness);
static void htctopaz_set_brightness_color(struct led_classdev*, enum led_brightness);

static DECLARE_WORK(colorled_wq, htctopaz_update_color_leds);
static DECLARE_WORK(backlight_wq, htctopaz_microp_update_backlight);
static DECLARE_WORK(buttonlight_wq, htctopaz_update_button_light);
static struct i2c_client *client = NULL;
static uint8_t g_auto_backlight = 0;

enum led_color {
	COLOR_OFF = 0,
	COLOR_GREEN = 1,
	COLOR_AMBER = 2,
};

enum supported_led {AMBER, GREEN, LCD, BUTTONS};

static struct led_classdev htctopaz_leds[] = {
	[AMBER] = {
		.name = "amber",
		.brightness_set = htctopaz_set_brightness_color,
	},
	[GREEN] = {
		.name = "green",
		.brightness_set = htctopaz_set_brightness_color,
	},
	[LCD] = {
		.name = "lcd-backlight",
		.brightness_set = htctopaz_set_backlight,
	},
	[BUTTONS] = {
		.name = "button-backlight",
		.brightness_set = htctopaz_set_button_light,
	}
};

/*
 * Helper functions
 */

static int htctopaz_microp_set_color_led(enum led_color led_color_value)
{
	int ret;
	uint8_t buf[5] = { 0, 0, 0, 0, 0 };

	if (!client) {
		return -EAGAIN;
	}

	buf[0] = TOPA_MICROP_COLOR_LED_ADDRESS;
	buf[1] = 0;
	buf[2] = led_color_value;
	buf[3] = 0xff;
	buf[4] = 0xff;

	ret = microp_ng_write(client, buf, ARRAY_SIZE(buf));
	if (ret) {
		printk(KERN_ERR "%s: Failed setting color led value (%d)\n",
			__func__, ret);
	}
	return ret;
}

static int htctopaz_microp_set_auto_backlight(int on)
{
	int ret;
	uint8_t buf[3] = { 0, 0, 0 };

	if (!client) {
		return -EAGAIN;
	}

	buf[0] = MICROP_I2C_WCMD_AUTO_BL_CTL;
	buf[1] = on ? 0x01 : 0x00;
	buf[2] = on ? 0x01 : 0x00;

	ret = microp_ng_write(client, buf, ARRAY_SIZE(buf));
	if (ret) {
		printk(KERN_ERR "%s: Failed writing auto backlight status (%d)\n",
			__func__, ret);
	}
	return ret;
}

static int htctopaz_microp_get_spi_auto_backlight_status(
	uint8_t* auto_backlight_status, uint8_t* spi_status)
{
	int ret;
	uint8_t buf[2] = { 0, 0 };

	if (!client) {
		return -EAGAIN;
	}

	ret = microp_ng_read(client, MICROP_I2C_RCMD_SPI_BL_STATUS, buf, 2);
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

	if (!client) {
		return -EAGAIN;
	}

	printk(KERN_INFO "%s(%u)\n", __func__, on);

	buf[0] = MICROP_I2C_WCMD_SPI_EN;
	buf[1] = on;

	ret = microp_ng_write(client, buf, ARRAY_SIZE(buf));
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

static void htctopaz_update_color_leds(struct work_struct* work)
{
	//printk(KERN_DEBUG "%s\n", __func__);
}

static void htctopaz_microp_update_backlight(struct work_struct* work)
{
	uint8_t buf[2] = { 0, 0 };
	enum led_brightness brightness = htctopaz_leds[LCD].brightness;

	if (!client) {
		return;
	}

	//printk(KERN_DEBUG "%s: brightness=%d\n", __func__, brightness);

	buf[0] = MICROP_I2C_WCMD_LCD_BRIGHTNESS;
	buf[1] = brightness/2 & 0xf0;

	microp_ng_write(client, buf, ARRAY_SIZE(buf));
}

static ssize_t htctopaz_microp_auto_backlight_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	int ret;
	uint8_t auto_backlight_status;
	uint8_t spi_status;

	ret = htctopaz_microp_get_spi_auto_backlight_status(&auto_backlight_status, &spi_status);
	if (ret) {
		return ret;
	}

	g_auto_backlight = auto_backlight_status ? 1 : 0;

	return sprintf(ret_buf, "%u\n", g_auto_backlight);
}

static ssize_t htctopaz_microp_auto_backlight_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned long val = simple_strtoul(in_buf, NULL, 10);

	printk(KERN_DEBUG "%s: %lu\n", __func__, val);

	if (htctopaz_microp_set_auto_backlight(val ? 1 : 0)) {
		return count;
	}

	g_auto_backlight = val ? 1 : 0;
	return count;
}

static DEVICE_ATTR(auto_backlight, 0644, htctopaz_microp_auto_backlight_get,
	htctopaz_microp_auto_backlight_set);

static void htctopaz_update_button_light(struct work_struct* work)
{
	//printk(KERN_DEBUG "%s\n", __func__);
}

static void htctopaz_set_brightness_color(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&colorled_wq);
}

static void htctopaz_set_button_light(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&buttonlight_wq);
}

static void htctopaz_set_backlight(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	schedule_work(&backlight_wq);
}

static int htctopaz_microp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	uint8_t auto_backlight_status;
	uint8_t spi_status;

	printk(KERN_INFO "%s\n", __func__);
	client = dev_get_drvdata(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(htctopaz_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &htctopaz_leds[i]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed registering led %s", __func__,
				htctopaz_leds[i].name);
			goto led_fail;
		}
	}

	ret = device_create_file(htctopaz_leds[LCD].dev, &dev_attr_auto_backlight);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed registering auto_backlight attribute (%d)",
			__func__, ret);
		goto led_fail;
	}

	// init auto backlight global
	ret = htctopaz_microp_get_spi_auto_backlight_status(&auto_backlight_status,
		&spi_status);
	if (ret) {
		printk(KERN_ERR "%s: Failed reading auto_backlight status (%d)",
			__func__, ret);
		goto attr_fail;
	}
	g_auto_backlight = auto_backlight_status;

	// reset the color led on boot
	htctopaz_microp_set_color_led(COLOR_OFF);

	return 0;

attr_fail:
	device_remove_file(htctopaz_leds[LCD].dev, &dev_attr_auto_backlight);
led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&htctopaz_leds[i]);
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
	client = NULL;
	return 0;
}

#if CONFIG_PM
static int htctopaz_microp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	htctopaz_microp_set_color_led(COLOR_GREEN);

	//microp_spi_enable(0);

	return 0;
}

static int htctopaz_microp_resume(struct platform_device *pdev)
{
	//microp_spi_enable(1);

	htctopaz_microp_set_color_led(COLOR_OFF);

	return 0;
}
#else
#define htctopaz_microp_suspend NULL
#define htctopaz_microp_resume NULL
#endif

static struct platform_driver htctopaz_microp_driver = {
	.probe		= htctopaz_microp_probe,
	.remove		= htctopaz_microp_remove,
	.suspend	= htctopaz_microp_suspend,
	.resume		= htctopaz_microp_resume,
	.driver		= {
		.name		= "htctopaz-microp-leds",
		.owner		= THIS_MODULE,
	},
};

static int __init htctopaz_microp_init(void)
{
	return platform_driver_register(&htctopaz_microp_driver);
}

static void __exit htctopaz_microp_exit(void)
{
	platform_driver_unregister(&htctopaz_microp_driver);
}

module_init(htctopaz_microp_init);
module_exit(htctopaz_microp_exit)
