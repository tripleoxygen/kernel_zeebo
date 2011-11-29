/*
 * Author: Abel C. Laura <abel.laura@gmail.com>
 *
 * Based on mahimahi-microp.c
 * Copyright (C) 2009 HTC, Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <mach/htc_ls_microp.h>

#define MICROP_I2C_WCMD_AUTO_BL_CTL			0x22
#define MICROP_I2C_RCMD_ADC_VALUE			0x31

/* not sure about this. traces show its called but not sure what it does.*/
#define MICROP_I2C_WCMD_UNKNOWN		0x23

struct microp_ls_info {
	//struct microp_function_config *ls_config; //<-- will be used with microp manager
	struct input_dev *ls_input_dev;
	struct early_suspend early_suspend;
	struct i2c_client *client;
	struct delayed_work work;

	uint32_t als_func;
	//uint32_t als_kadc;
	//uint32_t als_gadc;
	uint8_t als_calibrating;
	int als_intr_enabled;
	int is_suspend;
};

#define POLL_DELAY   	2000
#define INITIAL_DELAY   350

struct microp_ls_info *ls_info;

//static int ls_enable_num;
static int polling_enabled = 0;
static uint16_t levels[10] = { 0, 2, 7, 17, 33, 172, 299, 326, 344, 1023 };

/* ------------------------------------------------------------------*/
/* The following needs to be moved to a common location.             */
/* Move to microp_ng(.35)? use atmega_micro(.39) since its all there */

static struct i2c_client *private_microp_client;

static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10

static int i2c_read_block(uint8_t addr, uint8_t *data, int length)
{
	int retry;
	int ret;
	struct i2c_msg msgs[] = {
	{
		.addr = private_microp_client->addr,
		.flags = 0,
		.len = 1,
		.buf = &addr,
	},
	{
		.addr = private_microp_client->addr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = data,
	}
	};

	mdelay(1);
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		ret = i2c_transfer(private_microp_client->adapter, msgs, 2);
		if (ret == 2) {
			//DBG_MSG("R [%02X] = %s", addr,hex2string(data, length));
			return 0;
		}
		msleep(10);
	}

	//SYS_ERR("i2c_read_block retry over %d",I2C_READ_RETRY_TIMES);
	return -EIO;
}

#define MICROP_I2C_WRITE_BLOCK_SIZE 21
static int i2c_write_block(uint8_t addr, uint8_t *data, int length)
{
	int retry;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = private_microp_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	//DBG_MSG("W [%02X] = %s", addr,hex2string(data, length));

	if (length + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		//SYS_ERR("i2c_write_block length too long");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, length);

	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		ret = i2c_transfer(private_microp_client->adapter, msg, 1);
		if (ret == 1)
			return 0;
		msleep(10);
	}
	//SYS_ERR("i2c_write_block retry over %d",
	//		I2C_WRITE_RETRY_TIMES);
	return -EIO;
}

static int microp_read_adc(uint8_t *data)
{
	int ret;

	if (!data)
		return -EIO;
	/* TODO: this addr is device specific. In future we should */
	/* be passed down via platform data */
	ret = i2c_read_block(MICROP_I2C_RCMD_ADC_VALUE, data, 2);
	if (ret < 0) {
		pr_err("%s: request adc fail\n", __func__);
		return -EIO;
	}
	return 0;
}


/* End common code. */

static int get_ls_adc_level(uint8_t *data)
{
	uint8_t i, adc_level = 0;
	uint16_t adc_value = 0;

	if (microp_read_adc(data) == -EIO) return -1;

	adc_value = data[0]<<8 | data[1];
	if (adc_value > 0x3FF) {
		printk(KERN_WARNING "%s: get wrong value: 0x%X\n",
			__func__, adc_value);
		return -1;
	} else {
		for (i = 0; i < 10; i++) {
			if (adc_value <= levels[i]) {
				adc_level = i;
				break;
			}
		}
		//printk(KERN_DEBUG "ALS value: 0x%X, level: %d #\n",
		//		adc_value, adc_level);
		data[2] = adc_level;
	}
	return 0;
}

static int ls_microp_intr_enable(uint8_t enable)
{
	int ret;
	uint8_t data[2];
	/* TODO: this is device specific so the enable mask and address */
	/* needs to be passed down via platform data on the board file*/
	data[1]=0;
	data[0] = enable ? 0x81 : 0x80;	// rhod specific need to pass others via platform
	ret = i2c_write_block(MICROP_I2C_WCMD_AUTO_BL_CTL, data, 2);
	return ret;
}

static void lightsensor_poll_func(struct work_struct *work)
{
	uint8_t data[3] = {0, 0, 0};
	int ret;
	struct microp_ls_info *li = ls_info;

	if (!li->als_intr_enabled) {
		pr_err("%s: we should not be polling\n", __func__);
		cancel_delayed_work(&li->work);
		ls_microp_intr_enable(0);
		return;
	}
	/* Wait for Framework event polling ready */
	//if (ls_enable_num == 0) {
	//	ls_enable_num = 1;
	//	msleep(300);
	//}

	ret = get_ls_adc_level(data);

	if (!ret) {
		input_report_abs(li->ls_input_dev,
				ABS_MISC, (int)data[2]);
		input_sync(li->ls_input_dev);
	}
	if (polling_enabled) schedule_delayed_work(&li->work, msecs_to_jiffies(POLL_DELAY));
}


static int lightsensor_enable(void)
{
	int ret;
	struct microp_ls_info *li = ls_info;

	pr_info("%s\n", __func__);

	if (li->is_suspend) {
		li->als_intr_enabled = 1;
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}
	if (!li->als_intr_enabled) {
		ret = ls_microp_intr_enable(1);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			li->als_intr_enabled = 1;
			/* report an invalid value first to ensure we trigger an event
			* when adc_level is zero.
			*/
			input_report_abs(li->ls_input_dev, ABS_MISC, -1);
			input_sync(li->ls_input_dev);
		}
		if (polling_enabled) schedule_delayed_work(&li->work, msecs_to_jiffies(INITIAL_DELAY));
	}
	return 0;
}

static int lightsensor_disable(void)
{
	/* update trigger data when done */
	struct microp_ls_info *li = ls_info;
	int ret;

	pr_info("%s\n", __func__);

	if (li->is_suspend) {
		li->als_intr_enabled = 0;
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}

	if (li->als_intr_enabled) {
		cancel_delayed_work(&li->work);
		ret = ls_microp_intr_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);
		else
			li->als_intr_enabled = 0;
	}
	return 0;
}

DEFINE_MUTEX(ls_i2c_api_lock);
static int lightsensor_opened;

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	if (lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lightsensor_opened = 1;
	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	lightsensor_opened = 0;
	mutex_unlock(&ls_i2c_api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct microp_ls_info *li = ls_info;
	mutex_lock(&ls_i2c_api_lock);
	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		printk(KERN_INFO "%s value = %d\n", __func__, val);
		polling_enabled = 1; /* if GB wants to use ictl then polling is on */
		rc = val ? lightsensor_enable() : lightsensor_disable();
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = li->als_intr_enabled;
		pr_debug("%s enabled %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

int panel_switch(int on, int bl)
{
	/* This is called by Panel unblank. */
 	/* We ignore all of this in GB      */
	struct microp_ls_info *li = ls_info;
	uint8_t buf[2] = { 0, 0};
	int ret=0;
	if (!polling_enabled) {
		buf[0]=on ? 0x01 : 0x00;
		ret = i2c_write_block(MICROP_I2C_WCMD_UNKNOWN, buf, 2);
		msleep(10);
		li->als_intr_enabled = bl ? 1 : 0;
	}
	return ret;
}
EXPORT_SYMBOL(panel_switch);

int autobacklight_control(int enable)
{
	if (!polling_enabled) {
		printk("%s: auto_bl set to %d\n", __func__, enable);
		if (enable)
			lightsensor_enable();
		else
			lightsensor_disable();
	}
	return 0;
}
EXPORT_SYMBOL(autobacklight_control);

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t data[3] = {0, 0, 0};
	int ret;

	ret = get_ls_adc_level(data);
	ret = sprintf(buf,
				"ADC[0x%03X] => level %d\n",
				(data[0] << 8 | data[1]), data[2]);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0666, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct microp_ls_info *li = ls_info;
	int ret;

	ret = sprintf(buf, "ls_en = %d\n",li->als_intr_enabled);
	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct microp_ls_info *li = ls_info;
	int ls_auto;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		li->als_calibrating = (ls_auto == 147) ? 1 : 0;
		lightsensor_enable();
	} else {
		li->als_calibrating = 0;
		li->als_intr_enabled = 0;
		lightsensor_disable();
	}
	return count;
}

static DEVICE_ATTR(ls_auto, 0666, ls_enable_show, ls_enable_store);

static ssize_t ls_polling_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "polling_en = %d\n",polling_enabled);
	return ret;
}

static ssize_t ls_polling_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int poll; 
	sscanf(buf, "%d", &poll);
	if (poll != 0 && poll != 1)
		return -EINVAL;
	polling_enabled = poll;
	return count;
}
static DEVICE_ATTR(ls_polling, 0666, ls_polling_show, ls_polling_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void light_sensor_suspend(struct early_suspend *h)
{
	struct microp_ls_info *li = ls_info;
	int ret;

	li->is_suspend = 1;
	if (li->als_intr_enabled) {
		if (polling_enabled) cancel_delayed_work(&li->work);
		ret = ls_microp_intr_enable(0);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
				__func__);
		else
			li->als_intr_enabled = 0;
	}
}

static void light_sensor_resume(struct early_suspend *h)
{
	struct microp_ls_info *li = ls_info;
	int ret;
	if (li->als_intr_enabled) {
		ret = ls_microp_intr_enable(1);
		if (ret < 0) {
			pr_err("%s: enable auto light sensor fail\n",
				__func__);
		}
		else {
			if (polling_enabled) schedule_delayed_work(&li->work, msecs_to_jiffies(POLL_DELAY));
			input_report_abs(li->ls_input_dev, ABS_MISC, -1);
			input_sync(li->ls_input_dev);
		}
	}
	li->is_suspend = 0;
}
#endif

static int lightsensor_probe(struct platform_device *pdev)
{
	int ret;
	struct microp_ls_info *li;
	li = kzalloc(sizeof(struct microp_ls_info), GFP_KERNEL);
	if (!li)
		return -ENOMEM;
	ls_info = li;
	//li->client = dev_get_drvdata(&pdev->dev);
	private_microp_client = dev_get_drvdata(&pdev->dev);
	//if (!li->client) {
	if (!private_microp_client) {
		pr_err("%s: can't get microp i2c client\n", __func__);
		return -1;
	}
	li->ls_input_dev = input_allocate_device();
	if (!li->ls_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		return -ENOMEM;
	}
	li->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, li->ls_input_dev->evbit);
	input_set_abs_params(li->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(li->ls_input_dev);
	if (ret < 0) {
		pr_err("%s: can not register input device\n",
				__func__);
		return ret;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register misc device\n",
				__func__);
		return ret;
	}

	INIT_DELAYED_WORK(&li->work, lightsensor_poll_func);

#ifdef CONFIG_HAS_EARLYSUSPEND
	li->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	li->early_suspend.suspend = light_sensor_suspend;
	li->early_suspend.resume = light_sensor_resume;
	register_early_suspend(&li->early_suspend);
#endif
	//ret = device_create_file(&li->client->dev, &dev_attr_ls_adc);
	//ret = device_create_file(&li->client->dev, &dev_attr_ls_auto);

	ret = device_create_file(&private_microp_client->dev, &dev_attr_ls_adc);
	ret = device_create_file(&private_microp_client->dev, &dev_attr_ls_auto);
	ret = device_create_file(&private_microp_client->dev, &dev_attr_ls_polling);

	return 0;

}

static struct platform_driver lightsensor_driver = {
	.probe = lightsensor_probe,
	.driver = { .name = "lightsensor_microp", },
};

static int __init light_sensor_init(void)
{
	return platform_driver_register(&lightsensor_driver);
}

static void __exit light_sensor_exit(void)
{
	platform_driver_unregister(&lightsensor_driver);
}

module_init(light_sensor_init);
module_exit(light_sensor_exit);

MODULE_DESCRIPTION("HTC LIGHT SENSOR");
MODULE_LICENSE("GPL");
