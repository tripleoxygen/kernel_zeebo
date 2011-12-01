/* linux/arch/arm/mach-msm/board-htcrhodium-panel.c
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

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/board_htc.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/vreg.h>

#include "board-htcrhodium.h"
#include "devices.h"

#define REG_WAIT (0xffff)

static struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	/* Auo Rhod100 */
	{0x1100, 0x01},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{REG_WAIT, 0x1},
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x00},		// vbp 1 clk
	{0x3b03, 0x00},		// vfp 1 clk
	{0x3b04, 0x00},		// hbp 1 clk
	{0x3b05, 0x00},		// hfp 1 clk
	{0x3500, 0x02},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5100, 0x40},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0x40},
	//~ {0x2900, 0x01},
	//~ {0x5300, 0x2c},
	{0x5303, 0x01},
};

struct nov_regs nov_init_seq1[] = {
	/* EID Rhod100,210,	// Table modified by WisTilt2
	       400,500 */
	//~ {0x2900, 0x01},		// display on
	{0x1200, 0x01},		// partial mode
	{0x1100, 0x01},		// sleep-out
	{0x1300, 0x01},		// normal mode
	{0x3500, 0x00},		// tearing effect
	{0x3a00, 0x55},		// bits per pixel
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x01},		// vbp 1 clk
	{0x3b03, 0x01},		// vfp 1 clk
	{0x3b04, 0x01},		// hbp 1 clk
	{0x3b05, 0x01},		// hfp 1 clk
	{0x4405, 0x00},		// set tear line 1st param
	{0x4401, 0x00},		// set tear line 2nd param
	{0x4e00, 0x00},		// set spi/i2c mode
	{0x5100, 0x40},		// display brightness
	{0x5301, 0x10},		// led control pins
	{0x5302, 0x01},		// labc dimming on/off
	{0x5303, 0x01},		// labc rising dimming style
	{0x5304, 0x01},		// labc falling dimming style
	{0x5500, 0x02},		// cabc enable & image type
	{0x5e00, 0x04},		// cabc minimum brightness
	{0x5e03, 0x05},		// labc adc & hysteresis enable
	{0x5e04, 0x01},		// labc reference voltage 1.7v
	{0x6a01, 0x00},		// pwm duty/freq control
	{0x6a02, 0x01},		// pwm freq
	{0x6a17, 0x00},		// cabc pwm force off
	{0x6a18, 0x40},		// cabc pwm duty
	{0x6f00, 0x00},		// clear ls lsb
	{0x6f01, 0x00},		// clear ls msb
	{0x6f02, 0x00},		// force cabc pwm off
	{0x6f03, 0x00},		// force pwm duty
	//~ {0x5300, 0x3c},		// set autobl bit during init
};

struct nov_regs nov_init_seq2[] = {
	/* EID Rhod300 */	// Table modified by WisTilt2
	//~ {0x2900, 0x01},		// display on
	{0x1200, 0x01},		// partial mode
	{0x1100, 0x01},		// sleep-out
	{0x1300, 0x01},		// normal mode
	{0x3500, 0x00},		// tearing effect
	{0x3a00, 0x55},		// bits per pixel
	{0x3b00, 0x00},		// hsp/vsp high trigger
	{0x3b02, 0x01},		// vbp 1 clk
	{0x3b03, 0x01},		// vfp 1 clk
	{0x3b04, 0x01},		// hbp 1 clk
	{0x3b05, 0x01},		// hfp 1 clk
	{0x4405, 0x00},		// set tear line 1st param
	{0x4401, 0x00},		// set tear line 2nd param
	{0x4e00, 0x00},		// set spi/i2c mode
	{0x5100, 0x20},		// display brightness
	{0x5301, 0x10},		// led control pins
	{0x5302, 0x01},		// labc dimming on/off
	{0x5303, 0x01},		// labc rising dimming style
	{0x5304, 0x01},		// labc falling dimming style
	{0x5500, 0x02},		// cabc enable & image type
	{0x5e00, 0x04},		// cabc minimum brightness
	{0x5e03, 0x05},		// labc adc & hysteresis enable
	{0x5e04, 0x01},		// labc reference voltage 1.7v
	{0x6a01, 0x00},		// pwm duty/freq control
	{0x6a02, 0x01},		// pwm freq
	{0x6a17, 0x00},		// cabc pwm force off
	{0x6a18, 0xff},		// cabc pwm duty
	{0x6f00, 0x00},		// clear ls lsb
	{0x6f01, 0x00},		// clear ls msb
	{0x6f02, 0x00},		// force cabc pwm off
	{0x6f03, 0x00},		// force pwm duty
	//~ {0x5300, 0x3c},		// set autobl bit during init
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
	//~ {0x2900, 0x01},
	//~ {0x5300, 0x2c},
	{0x5e03, 0x01},
};

struct nov_regs nov_deinit_seq[] = {
	{0x2800, 0x01},		// display off
   	{0x5300, 0x28},
	{0x5500, 0x00},
	{0x5300, 0x08},
	{0x5E03, 0x00},
	{0x5300, 0x00},
	{0x1000, 0x01},		// sleep-in
};

int panel_id;
static struct vreg *vreg_lcd_1;	/* LCD1 */
static struct vreg *vreg_lcd_2;	/* LCD2 */

static int htcrhod_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

	switch (panel_id)	//Added by WisTilt2 - Panel detect
	{
	case	0x14:	/* EID - Rhod100,210,400,500 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq1); i++)
		{
			reg = cpu_to_le32(nov_init_seq1[i].reg);
			val = cpu_to_le32(nov_init_seq1[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	case	0x15:	/* EID - Rhod300 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq2); i++)
		{
			reg = cpu_to_le32(nov_init_seq2[i].reg);
			val = cpu_to_le32(nov_init_seq2[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	case	0x01:	/* AUO - Rhod100 */
	case	0x07:	/* AUO - Rhod100 */
	case	0x13:	/* AUO - Rhod100 */

		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq); i++)
		{
			reg = cpu_to_le32(nov_init_seq[i].reg);
			val = cpu_to_le32(nov_init_seq[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
		break;

	default:
		printk(KERN_WARNING "%s: Unknown panel_id %x detected!\n", __func__, panel_id);
		client_data->auto_hibernate(client_data, 0);
		for (i = 0; i < ARRAY_SIZE(nov_init_seq1); i++)	//Default to EID on unknown
		{
			reg = cpu_to_le32(nov_init_seq1[i].reg);
			val = cpu_to_le32(nov_init_seq1[i].val);
			if (reg == REG_WAIT)
				mdelay(val);
			else
				client_data->remote_write(client_data, val, reg);
		}
	}
	client_data->auto_hibernate(client_data, 1);

	return 0;
}

static int htcrhod_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(nov_deinit_seq); i++) {
		reg = cpu_to_le32(nov_deinit_seq[i].reg);
		val = cpu_to_le32(nov_deinit_seq[i].val);
		client_data->remote_write(client_data, val, reg);
	}

	return 0;
}

static int htcrhod_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	return 0;
}

static int htcrhod_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	client_data->remote_write(client_data, 0x01, 0x2900);	// display on
	client_data->remote_write(client_data, 0x2c, 0x5300);	// toggle autobl bit

	return 0;
}

static void htcrhod_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	printk(KERN_DEBUG "%s(%s)\n", __func__, on ? "on" : "off");

	if (on) {
		if (get_machine_variant_type() < MACHINE_VARIANT_RHOD_4XX) {
			vreg_enable(vreg_lcd_1);
			vreg_enable(vreg_lcd_2);
		} else {
			gpio_tlmm_config(
				GPIO_CFG(RHOD_LCD_PWR1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
			gpio_tlmm_config(
				GPIO_CFG(RHOD_LCD_PWR2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
		}
		mdelay(20);

		gpio_tlmm_config(
			GPIO_CFG(RHOD_LCD_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		mdelay(10);
		gpio_tlmm_config(
			GPIO_CFG(RHOD_LCD_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
		mdelay(1);
		gpio_tlmm_config(
			GPIO_CFG(RHOD_LCD_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		mdelay(25);
	} else {
		//~ mdelay(104);
		gpio_tlmm_config(
			GPIO_CFG(RHOD_LCD_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
		mdelay(25);

		if (get_machine_variant_type() < MACHINE_VARIANT_RHOD_4XX) {
			vreg_disable(vreg_lcd_1);
			vreg_disable(vreg_lcd_2);
		} else {
			gpio_tlmm_config(
				GPIO_CFG(RHOD_LCD_PWR2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA),
				GPIO_CFG_DISABLE);
			gpio_tlmm_config(
				GPIO_CFG(RHOD_LCD_PWR1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA),
				GPIO_CFG_DISABLE);
		}
		mdelay(3);
	}
}

extern struct resource resources_msm_fb[];

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = htcrhod_mddi_client_init,
	.uninit = htcrhod_mddi_client_uninit,
	.blank = htcrhod_mddi_panel_blank,
	.unblank = htcrhod_mddi_panel_unblank,
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
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5580),
			.name = "mddi_c_simple",//"mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_simple",//"mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		}
	},
};

int __init htcrhod_init_panel(void)
{
	int rc;

	if(!machine_is_htcrhodium()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return -1;
	}

	panel_id = readl(MSM_SPL_BASE + 0x81034);

	printk(KERN_INFO "%s: Initializing panel type 0x%x\n", __func__, panel_id);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	vreg_lcd_1 = vreg_get(0, "rftx");
	if (IS_ERR(vreg_lcd_1))
		return PTR_ERR(vreg_lcd_1);

	vreg_lcd_2 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcd_2))
		return PTR_ERR(vreg_lcd_2);

	rc = gpio_request(RHOD_LCD_VSYNC, "lcd vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(RHOD_LCD_VSYNC);
	if (rc)
		return rc;

	rc = gpio_request(RHOD_LCD_RST, "lcd reset");
	if (rc)
		return rc;
	rc = gpio_request(RHOD_LCD_PWR1, "lcd pwr1");
	if (rc)
		return rc;
	rc = gpio_request(RHOD_LCD_PWR2, "lcd pwr2");
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}

device_initcall(htcrhod_init_panel);
