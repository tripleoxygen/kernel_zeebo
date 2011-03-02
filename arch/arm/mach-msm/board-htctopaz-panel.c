/* linux/arch/arm/mach-msm/board-htctopaz-panel.c
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

#include <mach/msm_fb.h>
#include <mach/vreg.h>
//#include <linux/microp-klt.h>

#include "board-htctopaz.h"
#include "devices.h"

#define REG_WAIT	(0xffff)

static struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_seq[] = {
	{0xc000, 0x86}, // PWRCTR1
	{0xc001, 0x00},
	{0xc002, 0x86},
	{0xc003, 0x00},

	{0xc100, 0x40}, // PWRCTR2
	{0xc200, 0x21},
	{0xc202, 0x02},

	{0xc700, 0x91}, // VMFCTR

	{0x3500, 0x02}, // SET_TEAR_ON
	{0x4400, 0x00}, // SET_TEAR_SCANLINE
	{0x4401, 0x4f}, // SET_TEAR_SCANLINE

	{0xe000, 0x01}, // GMACTRL1
	{0xe001, 0x05},
	{0xe002, 0x1c},
	{0xe003, 0x33},
	{0xe004, 0x21},
	{0xe005, 0x35},
	{0xe006, 0x60},
	{0xe007, 0x33},
	{0xe008, 0x24},
	{0xe009, 0x26},
	{0xe00a, 0x84},
	{0xe00b, 0x15},
	{0xe00c, 0x3a},
	{0xe00d, 0x4f},
	{0xe00e, 0x8c},
	{0xe00f, 0xaf},
	{0xe010, 0x4b},
	{0xe011, 0x4d},

	{0xe100, 0x01}, // GMACTRL2
	{0xe101, 0x05},
	{0xe102, 0x1c},
	{0xe103, 0x33},
	{0xe104, 0x21},
	{0xe105, 0x35},
	{0xe106, 0x62},
	{0xe107, 0x33},
	{0xe108, 0x26},
	{0xe109, 0x26},
	{0xe10a, 0x84},
	{0xe10b, 0x11},
	{0xe10c, 0x38},
	{0xe10d, 0x4d},
	{0xe10e, 0x8a},
	{0xe10f, 0xad},
	{0xe110, 0x49},
	{0xe111, 0x4d},

	{0xe200, 0x08}, // GMACTRL3
	{0xe201, 0x19},
	{0xe202, 0x28},
	{0xe203, 0x3e},
	{0xe204, 0x1f},
	{0xe205, 0x34},
	{0xe206, 0x68},
	{0xe207, 0x43},
	{0xe208, 0x2b},
	{0xe209, 0x2a},
	{0xe20a, 0x88},
	{0xe20b, 0x19},
	{0xe20c, 0x3c},
	{0xe20d, 0x52},
	{0xe20e, 0x8b},
	{0xe20f, 0xad},
	{0xe210, 0x4d},
	{0xe211, 0x4d},

	{0xe300, 0x08}, // GMACTRL4
	{0xe301, 0x19},
	{0xe302, 0x28},
	{0xe303, 0x3e},
	{0xe304, 0x1f},
	{0xe305, 0x34},
	{0xe306, 0x68},
	{0xe307, 0x43},
	{0xe308, 0x2d},
	{0xe309, 0x2a},
	{0xe30a, 0x88},
	{0xe30b, 0x15},
	{0xe30c, 0x3a},
	{0xe30d, 0x50},
	{0xe30e, 0x89},
	{0xe30f, 0xab},
	{0xe310, 0x4b},
	{0xe311, 0x4d},

	{0xe400, 0x7a}, // GMACTRL5
	{0xe401, 0x76},
	{0xe402, 0x7d},
	{0xe403, 0x90},
	{0xe404, 0x3b},
	{0xe405, 0x40},
	{0xe406, 0x6a},
	{0xe407, 0x61},
	{0xe408, 0x29},
	{0xe409, 0x29},
	{0xe40a, 0x94},
	{0xe40b, 0x15},
	{0xe40c, 0x35},
	{0xe40d, 0x4f},
	{0xe40e, 0x9b},
	{0xe40f, 0xcc},
	{0xe410, 0x4d},
	{0xe411, 0x4d},

	{0xe500, 0x7a}, // GMACTRL6
	{0xe501, 0x76},
	{0xe502, 0x7d},
	{0xe503, 0x90},
	{0xe504, 0x3b},
	{0xe505, 0x40},
	{0xe506, 0x6c},
	{0xe507, 0x61},
	{0xe508, 0x2b},
	{0xe509, 0x29},
	{0xe50a, 0x94},
	{0xe50b, 0x11},
	{0xe50c, 0x33},
	{0xe50d, 0x4d},
	{0xe50e, 0x99},
	{0xe50f, 0xca},
	{0xe510, 0x4b},
	{0xe511, 0x4d},

	{0xf402, 0x14}, // ?
	{0xf100, 0x0c}, // ?
	{0xb600, 0x10}, // SD_OP_SET

#if 0
	{0xb100, 0x06}, // VPA[7:0] register (default: 6=0x06)
	{0xb101, 0x01}, // T2[9:8] register (default: 340=0x0154)
	{0xb102, 0x54}, // T2[7:0] register
#endif

	{0x1100, 0x00}, // EXIT_SLEEP_MODE ("Sleep Out")
	{REG_WAIT, 120}, // REG_WAIT (must wait 120ms after "Sleep Out")
};

static struct nov_regs nov_deinit_seq[] = {
	{0x5300, 0x00}, // WRCTRLD
	{0x2800, 0x00}, // SET_DISPLAY_OFF
	{0x1000, 0x00}, // ENTER_SLEEP_MODE ("Sleep In")
};

//static struct clk *gp_clk;
static struct vreg *vreg_aux;
static struct vreg *vreg_rfrx2;
static struct vreg *vreg_mddi;


static int htctopaz_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

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

	return 0;
}

static int htctopaz_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;

	printk(KERN_DEBUG "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(nov_deinit_seq); i++) {
		reg = cpu_to_le32(nov_deinit_seq[i].reg);
		val = cpu_to_le32(nov_deinit_seq[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
	}

	return 0;
}

static int htctopaz_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

//	micropklt_panel_suspend();

	return 0;
}

static int htctopaz_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);

	// SET_DISPLAY_ON
	client_data->remote_write(client_data, 0x00, 0x2900);
	// WRCTRLD
	client_data->remote_write(client_data, 0x2c, 0x5300);

//	micropklt_panel_resume();

	return 0;
}

static void htctopaz_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	printk(KERN_DEBUG "%s(%s)\n", __func__, on ? "on" : "off");

	if (on) {
		//gpio_configure(87, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		gpio_tlmm_config(
			GPIO_CFG(87, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
		msleep(10);

		vreg_enable(vreg_aux);
		vreg_enable(vreg_rfrx2);
		vreg_enable(vreg_mddi);
		mdelay(50);

#if 0
		gpio_configure(57, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_set_value(57, 0);
		gpio_configure(58, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_set_value(58, 0);
		msleep(5);
#endif
	} else {
#if 0
		gpio_configure(57, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		gpio_set_value(57, 1);
		gpio_configure(58, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		gpio_set_value(58, 1);
		msleep(5);
#endif

		vreg_disable(vreg_mddi);
		vreg_disable(vreg_rfrx2);
		vreg_disable(vreg_aux);
		mdelay(50);

		//gpio_configure(87, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_tlmm_config(
			GPIO_CFG(87, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
		msleep(10);
	}
}

extern struct resource resources_msm_fb[];

struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = htctopaz_mddi_client_init,
	.uninit = htctopaz_mddi_client_uninit,
	.blank = htctopaz_mddi_panel_blank,
	.unblank = htctopaz_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.output_format = 0,
	},
#if 0
	.panel_conf = {
		.caps = MSMFB_CAP_CABC,
	},
#endif
};

struct msm_mddi_platform_data mddi_pdata = {
	.vsync_irq = MSM_GPIO_TO_INT(TOPA100_LCD_VSYNC),
	.power_client = htctopaz_mddi_power_client,
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

int __init htctopaz_init_panel(void)
{
	int rc;

	if(!machine_is_htctopaz()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return -1;
	}

	printk(KERN_INFO "%s: Initializing panel\n", __func__);

#if 0
	gp_clk = clk_get(NULL, "gp_clk");
	if (IS_ERR(gp_clk)) {
		printk(KERN_ERR "%s: could not get gp clock\n", __func__);
		gp_clk = NULL;
	}
	rc = clk_set_rate(gp_clk, 19200000);
	if (rc)
	{
		printk(KERN_ERR "%s: set clock rate failed\n", __func__);
	}
#endif
	vreg_aux = vreg_get(0, "gp4");
	if (IS_ERR(vreg_aux))
		return PTR_ERR(vreg_aux);

	vreg_rfrx2 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_rfrx2))
		return PTR_ERR(vreg_rfrx2);

	vreg_mddi = vreg_get(0, "gp2");
	if (IS_ERR(vreg_mddi))
		return PTR_ERR(vreg_mddi);

	rc = gpio_request(TOPA100_LCD_VSYNC, "vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(TOPA100_LCD_VSYNC);
	if (rc)
		return rc;

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;
	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}

device_initcall(htctopaz_init_panel);
