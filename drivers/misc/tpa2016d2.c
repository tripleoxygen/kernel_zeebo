/* driver/i2c/chip/tap2016d2.c
 *
 * TI tpa2016d2 Speaker Amp
 *
 * Copyright (C) 2011 Xdandroid
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <mach/tpa2016d2.h>
#include <linux/mutex.h>
#include <asm/gpio.h>

#define DEBUG (1)
#define DEBUG_I2C (0)

#define RETRY_CNT 5

/* Userspace interraction error printout */
#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

#if (DEBUG==1)
#define D(fmt, args...) printk(KERN_INFO "tpa2016d2: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "tpa2016d2: "fmt, ##args)
#else
#define D(fmt, args...) do{} while(0);
#define E(fmt, args...) do{} while(0);
#endif

#define I2C_WRITE_RETRY_TIMES 2

static struct i2c_client *this_client;
static struct tpa2016d2_platform_data *pdata;
struct mutex spk_amp_lock;
static int tpa2016d2_opened;

static int tpa2016_ready = false;

#define MAX_I2C_WRITE 20
static int tpa2016_i2c_write(unsigned char addr, char *data, int len)
{
	int retry;
	uint8_t buf[MAX_I2C_WRITE];
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = len + 1,
		 .buf = buf,
		}
	};

	if (len + 1 > MAX_I2C_WRITE) {
		dev_err(&this_client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, len);

	mdelay(1);
	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		ret = i2c_transfer(this_client->adapter, msg, 1);
		if (ret == 1)
			return 0;
		msleep(10);
	}
	dev_err(&this_client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
	return -EIO;
}

static int tpa2016_i2c_read(unsigned char addr,
            char *data, int length)
{
	int rc;
	struct i2c_msg msgs[] = {
	    {
		    .addr = this_client->addr,
		    .flags = 0,
		    .len = 1,
		    .buf = &addr,
	    },
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	rc = i2c_transfer(this_client->adapter, msgs, 2);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG_I2C
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: rx[%d] = %2x\n", __func__, i, data[i]);
	}
#endif

	return 0;
}

/* raw register access */
static void tpa_set_register(uint8_t reg, uint8_t arg)
{
    if ( tpa2016_ready ) {
        D("%s: reg %d, value 0x%x\n", __func__, reg, arg);
       	if(tpa2016_i2c_write(reg, &arg, 1))
		    pr_err("Can't set tpa's reg\n"); 
    }
}

void tpa2016d2_set_register(uint8_t reg, uint8_t arg)
{
    tpa_set_register(reg, arg);
}

static void tpa_read_register(uint8_t reg, uint8_t* arg)
{
    if ( tpa2016_ready ) {
	    if(tpa2016_i2c_read(reg, arg, 1))
		    pr_err("Can't get tpa's agc reg\n");
        D("%s: reg %d, value 0x%x\n", __func__, reg, arg[0]);
    }
}

void tpa2016d2_read_register(uint8_t reg, uint8_t* arg)
{
    tpa_read_register(reg, arg);
}

/* Enable / Disable power to amplifier */
static void tpa_set_amp_power(bool bOn)
{
    D(" set power %d\n", bOn);
    if ( bOn ) {
        gpio_direction_output(pdata->gpio_tpa2016_spk_en, 1);
        mdelay(1);
        tpa2016_ready = true;
    } else {
        if ( tpa2016_ready ) {
            /* TPA2016 Quick shudown sequence */
            tpa_set_register(1, 0xC2);
            tpa_set_register(2, 0x01);
            tpa_set_register(3, 0x01);
            tpa_set_register(4, 0x00);
            tpa_set_register(5, 0x00);
            tpa_set_register(6, 0x19);
            tpa_set_register(7, 0xC0);            
            gpio_direction_output(pdata->gpio_tpa2016_spk_en, 0);
        }
        tpa2016_ready = false;
    }
}


void tpa2016d2_set_power(bool bOn)
{
    tpa_set_amp_power(bOn);
}

static int tpa2016d2_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&spk_amp_lock);

	if (tpa2016d2_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}
	tpa2016d2_opened = 1;
done:
	mutex_unlock(&spk_amp_lock);
	return rc;
}

static int tpa2016d2_release(struct inode *inode, struct file *file)
{
	mutex_lock(&spk_amp_lock);
	tpa2016d2_opened = 0;
	mutex_unlock(&spk_amp_lock);

	return 0;
}

static int
tpa2016d2_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
    int rc = -EIO, i;
    bool bOn;
    char I2C_regs[7];

	switch (cmd) {
        case TPA2016_SET_POWER:
            if (copy_from_user(&bOn, argp, sizeof(bool))) {
		        ERR_COPY_FROM_USER();
		        rc = -EFAULT;
	        } else {
                tpa_set_amp_power(bOn);
                rc = 0;
            }
            D("ioctl: TPA2016_SET_POWER called %d rc %d.\n", current->pid, rc);
        break;

        case TPA2016_SET_CONFIG:
            if (copy_from_user(&I2C_regs, argp, sizeof(I2C_regs))) {
		        ERR_COPY_FROM_USER();
		        rc = -EFAULT;
	        } else {
                for(i=0; i<7; i++) {
                    tpa_set_register(i+1, I2C_regs[i]);
                }
                rc = 0;
            }                   
            D("ioctl: TPA2016_SET_CONFIG called %d rc %d.\n", current->pid, rc);    
        break;

        case TPA2016_READ_CONFIG:
            for(i=0; i<7; i++) {
               tpa_read_register(i+1, &I2C_regs[i]);
            }
            if ( copy_to_user(argp, &I2C_regs, sizeof(I2C_regs)) ) {
                ERR_COPY_TO_USER();
                rc = -EFAULT;
            } else {
                rc = 0;
            }              
            D("ioctl: TPA2016_READ_CONFIG called %d rc %d.\n", current->pid, rc);
        break;

	    default:
		    pr_err("%s: Invalid command\n", __func__);
		    rc = -EINVAL;
		    break;
	}
	return rc;
}

static struct file_operations tpa2016d2_fops = {
	.owner = THIS_MODULE,
	.open = tpa2016d2_open,
	.release = tpa2016d2_release,
	.ioctl = tpa2016d2_ioctl,
};

static struct miscdevice tpa2016d2_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tpa2016d2",
	.fops = &tpa2016d2_fops,
};

int tpa2016d2_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	D("%s\n", __func__);

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			ret = -ENOMEM;
			pr_err("%s: platform data is NULL\n", __func__);
			goto err_alloc_data_failed;
		}
	}

	this_client = client;

	ret = gpio_request(pdata->gpio_tpa2016_spk_en, "tpa2016d2_amp");
	if (ret < 0) {
		pr_err("%s: gpio request aud_spk_en pin failed\n", __func__);
		goto err_free_gpio_all;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_free_gpio_all;
	}

	ret = misc_register(&tpa2016d2_device);
	if (ret) {
		pr_err("%s: tpa2016d2_device register failed\n", __func__);
		goto err_free_gpio_all;
	}

	return 0;

err_free_gpio_all:
	return ret;
err_alloc_data_failed:
	return ret;
}

static int tpa2016d2_remove(struct i2c_client *client)
{
	struct tpa2016d2_platform_data *p2016data = i2c_get_clientdata(client);
	kfree(p2016data);

	return 0;
}

static int tpa2016d2_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int tpa2016d2_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tpa2016d2_id[] = {
	{ TPA2016D2_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver tpa2016d2_driver = {
	.probe = tpa2016d2_probe,
	.remove = tpa2016d2_remove,
	.suspend = tpa2016d2_suspend,
	.resume = tpa2016d2_resume,
	.id_table = tpa2016d2_id,
	.driver = {
		.name = TPA2016D2_I2C_NAME,
	},
};

static int __init tpa2016d2_init(void)
{
	D("%s\n", __func__);
	mutex_init(&spk_amp_lock);
	return i2c_add_driver(&tpa2016d2_driver);
}

static void __exit tpa2016d2_exit(void)
{
	i2c_del_driver(&tpa2016d2_driver);
}

module_init(tpa2016d2_init);
module_exit(tpa2016d2_exit);

MODULE_DESCRIPTION("tpa2016d2 Speaker Amp driver");
MODULE_LICENSE("GPL");
