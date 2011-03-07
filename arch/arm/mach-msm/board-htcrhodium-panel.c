/* linux/arch/arm/mach-msm/board-htcrhod-panel.c
 * Based on board-trout-panel.c by: Brian Swetland <swetland@google.com>
 * Remodelled based on board-supersonic-panel.c by: Jay Tu <jay_tu@htc.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
//#include <linux/microp-klt.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
//#include <linux/microp-klt.h>

#include "board-htcrhodium.h"
#include "proc_comm_wince.h"
#include "devices.h"

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

int panel_id;

#define REG_WAIT (0xffff)

/* Panel IDs for reference
12-13 : Sharp
1 : Auo
5 : Hitachi
7 : EID
19 : AUO
20: EID 
*/
struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	/* EID es3 */
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x00},
	{0x180, 0x02},
	{0x880, 0x00},
	{0x2080, 0x33},
	{0x2680, 0x78},
	{0x2880, 0x3e},
	{0x2980, 0x04},
	{0x2c80, 0x22},
	{0x2e80, 0x00},
	{0xde80, 0x02},
	{0x3a00, 0x55},
	{0x3500, 0x00},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5303, 0x01},
};

struct nov_regs nov_deinit_seq[] = {

	{0x2800, 0x01},
	{0x5300, 0x28},
	{0x5500, 0x00},
	{0x5300, 0x08},
	{0x5e03, 0x00},
	{0x5300, 0x00},
	{0x1000, 0x01},
	{REG_WAIT, 0x5},
};

struct nov_regs nov_init_seq1[] = {
	/* Auo es1 */
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{REG_WAIT, 0x1},
	{0x3500, 0x02},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5303, 0x01},
};

struct nov_regs nov_init_seq2[] = {
	/* Sharp */
	{0x5100, 0x00},
	{0x2e80, 0x01},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x01},
	{0x3a00, 0x05},
	{0x3500, 0x00},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5e03, 0x01},
};

struct nov_regs nov_init_seq3[] = {
	/* EID */
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{0x3500, 0x00},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5e03, 0x01},
};

static void suc_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 5;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 104 * (val - 30) / 113 + 5;
	else
		shrink_br = 145 * (val - 144) / 111 + 110;
	mutex_lock(&cabc.lock);
	client->remote_write(client, 0, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);	
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
suc_get_brightness(struct led_classdev *led_cdev)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
}

#define DEFAULT_BRIGHTNESS 100
static void suc_backlight_switch(int on)
{
#if 0
	enum led_brightness val;
	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
		 * If the physical layer is not ready, we should
		 * not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		suc_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		suc_set_brightness(&cabc.lcd_backlight, 0);
	}
#endif
}

static int suc_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;

	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = suc_set_brightness;
	cabc.lcd_backlight.brightness_get = suc_get_brightness;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	printk(KERN_DEBUG "%s successful\n", __func__);	
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

static int htcrhod_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;
	printk(KERN_DEBUG "%s\n", __func__);	
	switch (panel_id)
	{
	case	0x14:	/*EID*/
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq); i++) {
			reg = cpu_to_le32(nov_init_seq[i].reg);
			val = cpu_to_le32(nov_init_seq[i].val);
			if (reg == REG_WAIT)
			msleep(val);
			else
			client_data->remote_write(client_data, val, reg);
		}
		client_data->auto_hibernate(client_data, 1);
		break;
	case	0x1:
	case	0x13:	/* AUO */
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq1); i++) {
			reg = cpu_to_le32(nov_init_seq1[i].reg);
			val = cpu_to_le32(nov_init_seq1[i].val);
			if (reg == REG_WAIT)
			msleep(val);
			else
			client_data->remote_write(client_data, val, reg);
		}
		client_data->auto_hibernate(client_data, 1);
		break;
	default:
		printk(KERN_WARNING "%s: FIXME! Don't know panel_id %d?\n", __func__, panel_id);
		return 0;
	}

	return 0;
}

static int htcrhod_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;
	for (i = 0; i < ARRAY_SIZE(nov_deinit_seq); i++) {
		reg = cpu_to_le32(nov_deinit_seq[i].reg);
		val = cpu_to_le32(nov_deinit_seq[i].val);
		client_data->remote_write(client_data, val, reg);
	}

	return 0;
}

/* FIXME: remove after XA03 */
static int backlight_control(int on)
{

	/* Portion needs to be written not to use microp */
	//autobacklight_control(on);
	return 0;
}

static int htcrhod_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
//	suc_backlight_switch(LED_OFF);
//	backlight_control(0);
	return 0;
}

static int htcrhod_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	//printk(KERN_DEBUG "%s dbg_auto = %d cabc = %d\n", __func__, is_auto(), test_bit(GATE_ON, &cabc.status));

	//if (is_auto())
	if(0)
	{
//		client_data->remote_write(client_data, 0x00, 0x6a17);
//		backlight_control(1);
		msleep(100);
	}
	else 
	{
//		client_data->remote_write(client_data, 0x01, 0x6a17);
//		backlight_control(0);
	}
//	suc_backlight_switch(LED_FULL);
	//msleep(10);
	//backlight_control(1);
	return 0;
}


static void htcrhod_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	printk(KERN_DEBUG "%s (%s)\n", __func__, on ? "on" : "off");
	if (on) {
//		gpio_set_value(RHOD_LCD_PWR2,1);
//		gpio_set_value(RHOD_LCD_PWR1,1);
/*		msleep(20);
		gpio_set_value(0x52,1);
		msleep(10);
		gpio_set_value(0x52,0);
		msleep(1);
		gpio_set_value(0x52,1);
		msleep(25);
*/
	} else {
/*		msleep(104);
		gpio_set_value(0x52,0);
		msleep(25);*/
//		gpio_set_value(RHOD_LCD_PWR1,0);
//		gpio_set_value(RHOD_LCD_PWR2,0);
//		msleep(3);

	}
}

extern struct resource resources_msm_fb[];

static struct msm_mddi_bridge_platform_data novatec_client_data = {
//	.init = htcrhod_mddi_client_init,
//	.uninit = htcrhod_mddi_client_uninit,
//	.blank = htcrhod_mddi_panel_blank,
//	.unblank = htcrhod_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.vsync_irq = MSM_GPIO_TO_INT(RHOD_LCD_VSYNC),
	.power_client = htcrhod_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			.product_id = (0xb9f6 << 16 | 0x5580),
			//.name = "mddi_c_b9f6_5582",
			.name = "mddi_c_simple",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			.product_id = (0xb9f6 << 16 | 0x5582),
			//.name = "mddi_c_b9f6_5582",
			.name = "mddi_c_simple",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		}
	},
};

static struct platform_driver suc_backlight_driver = {
	.probe = suc_backlight_probe,
	.driver = {
		.owner = THIS_MODULE,
	},
};

int __init htcrhod_init_panel(void)
{
	int rc;

	if(!machine_is_htcrhodium()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return 0;
	}
	//panel_id = readl(MSM_SPL_BASE+0x81034);
	panel_id = 0x100;
	printk(KERN_INFO "%s: Initializing panel id=%d\n", __func__, panel_id);
	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	suc_backlight_driver.driver.name = "nov_cabc";

	rc = platform_driver_register(&suc_backlight_driver);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);

}

device_initcall(htcrhod_init_panel);
