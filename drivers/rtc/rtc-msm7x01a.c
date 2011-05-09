/* drivers/rtc/rtc-msm7x00.c
 *
 * Author: Martin Johnson <M.J.Johnson@massey.ac.nz>
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

#include <linux/module.h>
#include <linux/version.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/rtc.h>

#ifdef CONFIG_QUICK_WAKEUP
#include <linux/quickwakeup.h>
#endif

#include "../../arch/arm/mach-msm/proc_comm_wince.h"

#define PDEV_NAME "msm_rtc"

#define SECSFROM_1970_TO_1980 315532800

static int msmrtc_resume(struct platform_device *dev);
static unsigned long msmrtc_get_seconds(void);

static struct rtc_device *rtc;

static unsigned long rtcalarm_time;
static unsigned int max_diff;

static int
msmrtc_pmlib_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long unix_time;
	struct msm_dex_command dex;

	if (rtc_valid_tm(tm))
		return -EINVAL;

	rtc_tm_to_time(tm, &unix_time);
	unix_time=unix_time-SECSFROM_1970_TO_1980; // MSM RTC starts 10 years after unix time

	dex.cmd = PCOM_WRITE_RTC;
	dex.has_data = 1;
	dex.data = unix_time;
	msm_proc_comm_wince(&dex, 0);

	return 0;
}

static int
msmrtc_pmlib_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned secs;
	struct msm_dex_command dex;

	dex.cmd = PCOM_READ_RTC;
	msm_proc_comm_wince(&dex, &secs);
	secs = secs + SECSFROM_1970_TO_1980;	// MSM RTC starts 10 years after unix time
	rtc_time_to_tm(secs, tm);
	return 0;
}

static int
msmrtc_virtual_alarm_set(struct device *dev, struct rtc_wkalrm *a)
{
	unsigned long now = get_seconds();

	if (!a->enabled) {
		rtcalarm_time = 0;
		return 0;
	} else
		rtc_tm_to_time(&a->time, &rtcalarm_time);

	if (now > rtcalarm_time) {
		printk(KERN_ERR "%s: Attempt to set alarm in the past\n",
		       __func__);
		rtcalarm_time = 0;
		return -EINVAL;
	}

	return 0;
}

static struct rtc_class_ops msm_rtc_ops = {
	.read_time	= msmrtc_pmlib_read_time,
	.set_time	= msmrtc_pmlib_set_time,
	.set_alarm	= msmrtc_virtual_alarm_set,
};

static void
msmrtc_alarmtimer_expired(unsigned long _data)
{
	printk(KERN_DEBUG "%s: Generating alarm event (src %lu)\n",
	       rtc->name, _data);

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);
	rtcalarm_time = 0;
}

#ifdef CONFIG_QUICK_WAKEUP
int msmrtc_check(void)
{
	//Force complete resume is rtc alarm time has passed
	unsigned long int secs = msmrtc_get_seconds();
	if(rtcalarm_time && rtcalarm_time <= secs)
	{
//		printk("msmrtc: alarm went off: %d, %d\n", rtcalarm_time, secs);
		return 1;
	}
//	printk("msmrtc: alarm not yet: %d, %d\n", rtcalarm_time, secs);
	return 0;
}

int msmrtc_callback(void)
{
	//Abort the quick sleep
//	printk("msmrtc callback\n");
	return 1;
}

static struct quickwakeup_ops quick_ops = {
	.qw_callback = msmrtc_callback,
	.qw_check = msmrtc_check,
};
#endif

static int
msmrtc_probe(struct platform_device *pdev)
{
	printk("RTC probe\n");
	rtc = rtc_device_register("msm_rtc",
				  &pdev->dev,
				  &msm_rtc_ops,
				  THIS_MODULE);
	if (IS_ERR(rtc)) {
		printk(KERN_ERR "%s: Can't register RTC device (%ld)\n",
		       pdev->name, PTR_ERR(rtc));
		return PTR_ERR(rtc);
	}

#ifdef CONFIG_QUICK_WAKEUP
	if(quickwakeup_register(&quick_ops))
	{
		printk(KERN_ERR "%s: Can't register quickwakeup events\n",
			pdev->name);
		return 1;
	}
#endif
 
	printk("MSM RTC started\n");
	return 0;
}

static unsigned long msmrtc_get_seconds(void)
{
	struct rtc_time tm;
	unsigned long now;

	msmrtc_pmlib_read_time(NULL, &tm);
	rtc_tm_to_time(&tm, &now);
	return now;
}

void msmrtc_maxdiff(unsigned int diff)
{
	max_diff = diff;
}
EXPORT_SYMBOL(max_diff);

int msmrtc_rewake(int diff)
{
	unsigned secs;
	struct msm_dex_command dex;

	
	dex.cmd = PCOM_READ_RTC;
	msm_proc_comm_wince(&dex, &secs);

	if(diff)
	{
//		printk("Wake up in %d seconds\n",diff);

		dex.cmd = PCOM_SET_ALARM_RTC;
		dex.has_data = 1;
		dex.data = secs + diff;
		msm_proc_comm_wince(&dex, 0);
	}

	return 0;
}
EXPORT_SYMBOL(msmrtc_rewake);

static int
msmrtc_suspend(struct platform_device *dev, pm_message_t state)
{
	unsigned secs;
	struct msm_dex_command dex;
	if (rtcalarm_time) {
		unsigned long now = msmrtc_get_seconds();
		int diff = rtcalarm_time - now;
		if (diff <= 0) {
			msmrtc_alarmtimer_expired(1);
			return 0;
		}

		if(max_diff)
			diff = max_diff;

		dex.cmd = PCOM_READ_RTC;
		msm_proc_comm_wince(&dex, &secs);

		printk("Wake up in %d seconds\n",diff);

		dex.cmd = PCOM_SET_ALARM_RTC;
		dex.has_data = 1;
		dex.data = secs + diff;
		msm_proc_comm_wince(&dex, 0);
	}

	return 0;
}

static int
msmrtc_resume(struct platform_device *dev)
{
	unsigned long now;
	int diff;
	if (rtcalarm_time) {
		now = msmrtc_get_seconds();
		diff = rtcalarm_time - now;
		if (diff <= 0)
			msmrtc_alarmtimer_expired(2);
	}
	return 0;
}

static struct platform_driver msmrtc_driver = {
	.probe	= msmrtc_probe,
	.suspend	= msmrtc_suspend,
	.resume		= msmrtc_resume,
	.driver	= {
		.name	= "msm_rtc",
	},
};

static int __init msmrtc_init(void)
{
	return platform_driver_register(&msmrtc_driver);
}

module_init(msmrtc_init);

MODULE_DESCRIPTION("RTC driver for Qualcomm MSM7x00 chipsets");
MODULE_AUTHOR("Martin Johnson <M.J.Johnson@massey.ac.nz>");
MODULE_LICENSE("GPL");
