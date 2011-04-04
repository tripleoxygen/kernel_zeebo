/* arch/arm/mach-msm/pwrsink.c
 *
 * Copyright (C) 2008 HTC Corporation.
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
 * Adopted from desirec board file for common use.
 * Michael Weirauch <mweirauch@xdandroid.com>
 * 
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <mach/htc_pwrsink.h>

static struct pwr_sink common_pwrsink_table[] = {
	{
		.id	= PWRSINK_AUDIO,
		.ua_max	= 100000,
	},
	{
		.id	= PWRSINK_BACKLIGHT,
		.ua_max	= 125000,
	},
	{
		.id	= PWRSINK_LED_BUTTON,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_LED_KEYBOARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_GP_CLK,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_BLUETOOTH,
		.ua_max	= 15000,
	},
	{
		.id	= PWRSINK_CAMERA,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_SDCARD,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_VIDEO,
		.ua_max	= 0,
	},
	{
		.id	= PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id	= PWRSINK_SYSTEM_LOAD,
		.ua_max	= 100000,
		.percent_util = 38,
	},
};

static int common_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void common_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void common_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int common_pwrsink_suspend_late(struct platform_device *pdev,
	pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data common_pwrsink_data = {
	.num_sinks	= ARRAY_SIZE(common_pwrsink_table),
	.sinks		= common_pwrsink_table,
	.suspend_late	= common_pwrsink_suspend_late,
	.resume_early	= common_pwrsink_resume_early,
	.suspend_early	= common_pwrsink_suspend_early,
	.resume_late	= common_pwrsink_resume_late,
};

static struct platform_device common_pwrsink_device = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev	= {
		.platform_data = &common_pwrsink_data,
	},
};

static struct platform_device *common_pwrsink_devices[] __initdata = {
	&common_pwrsink_device,
};

static int __init common_pwrsink_init(void)
{
	int rc = -1;
	rc = platform_add_devices(common_pwrsink_devices,
		ARRAY_SIZE(common_pwrsink_devices));
	printk(KERN_INFO "Common pwrsink setup %s (%d)\n",
		(rc == 0 ? "succeeded"	: "failed"), rc);
	return rc;
}

module_init(common_pwrsink_init);
 
MODULE_DESCRIPTION("Common pwrsink setup");
MODULE_LICENSE("GPL");
