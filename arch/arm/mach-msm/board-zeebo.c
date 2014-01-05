/* linux/arch/arm/mach-msm/board-zeebo.c
 * 
 * Copyright (C) 2012 Triple Oxygen
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
#include <linux/mm.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <asm/setup.h>
#include <mach/msm7200a_rfkill.h>
#include <mach/msm_serial_hs.h>

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
#include <linux/input/msm_ts.h>
#include <linux/mfd/microp-ng.h>
#include <mach/msm_rpcrouter.h>

#include <mach/board_htc.h>
#include <mach/htc_headset.h>

#include "proc_comm_wince.h"
#include <mach/devices.h>
#include "board-zeebo.h"
#include "gpio_chip.h"
#include "pmic.h"
#include "smd_private.h"

extern int zeebo_init_mmc(void);

/******************************************************************************
 * MicroP
 ******************************************************************************/
static struct platform_device zeebo_microp_leds = {
	.id = -1,
	.name = "microp-led-klt",
};

static struct platform_device* zeebo_microp_clients[] = {
	&zeebo_microp_leds,
};

static uint16_t micropklt_compatible_versions[] = {
	TOPA_MICROP_VERSION
};

static struct microp_platform_data zeebo_microp_pdata = {
	.version_reg = TOPA_MICROP_VERSION_REG,
	.clients = zeebo_microp_clients,
	.nclients = ARRAY_SIZE(zeebo_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};

/******************************************************************************
 * USB
 ******************************************************************************/
static void zeebo_usb_disable(void)
{
	gpio_set_value(TOPA100_USBPHY_RST, 0); 
	mdelay(3);
}

static void zeebo_usb_enable(void)
{
	gpio_set_value(TOPA100_USBPHY_RST, 1);
	mdelay(3);
}

static void zeebo_usb_hw_reset(bool off)
{
	printk(KERN_WARNING "%s(%d)\n", __func__, off ? 1 : 0);

	if (off) {
		zeebo_usb_disable();
	} else {
		zeebo_usb_enable();
	}
}

static struct msm_hsusb_platform_data zeebo_hsusb_board_pdata = {
	.hw_reset = zeebo_usb_hw_reset,
	//.usb_connected = notify_usb_connected,
};

static struct i2c_board_info i2c_devices[] = {
	{
		// LED & Backlight controller
		.type = "microp-ng",
		.addr = 0x66,
		.platform_data = &zeebo_microp_pdata,
	},
#if defined(CONFIG_MSM_CAMERA) && defined(CONFIG_MT9P012)
	{		
		I2C_BOARD_INFO("mt9p012", 0x6c >> 1),
	},
#endif
};

/******************************************************************************
 * AMSS-specific stuff
 ******************************************************************************/
static struct platform_device zeebo_amss_device = {
	.name = "amss_brew",
	.id = -1,
};

/******************************************************************************
 * H2W
 ******************************************************************************/
#ifdef CONFIG_HTC_HEADSET
static void zeebo_h2w_config_cpld(int route)
{
	printk(KERN_DEBUG "%s: route=%d\n", __func__, route);
	switch (route) {
		case H2W_UART3:
			//~ gpio_set_value(103, 1); // TODO wrong GPIO?
			break;
		case H2W_GPIO:
			//~ gpio_set_value(103, 0); // TODO wrong GPIO?
			break;
		default:
			printk(KERN_ERR "%s: unknown route=%d\n", __func__, route);
	}
}

static void zeebo_h2w_init_cpld(void)
{
	zeebo_h2w_config_cpld(H2W_UART3);
	gpio_set_value(TOPA100_H2W_CLK, 0);
	gpio_set_value(TOPA100_H2W_DATA, 0);
}

static struct h2w_platform_data zeebo_h2w_pdata = {
	.cable_in1              = TOPA100_CABLE_IN1,
	.cable_in2              = TOPA100_CABLE_IN2,
	.h2w_clk                = TOPA100_H2W_CLK,
	.h2w_data               = TOPA100_H2W_DATA,
	.debug_uart             = H2W_UART3,
	.config_cpld            = zeebo_h2w_config_cpld,
	.init_cpld              = zeebo_h2w_init_cpld,
	.headset_mic_35mm       = TOPA100_AUD_HSMIC_DET_N,
};

static struct platform_device zeebo_h2w = {
	.name = "h2w",
	.id = -1,
	.dev = {
		.platform_data  = &zeebo_h2w_pdata,
	},
};
#endif

static struct ts_virt_key zeebo_ts_keys_y[] = {
	// key      min   max
	{KEY_UP,    105, 267}, //  420, 1068},
	{KEY_DOWN,  268, 429}, // 1069, 1716},
	{KEY_HOME,  430, 591}, // 1717, 2364},
	{KEY_LEFT,  592, 753}, // 2365, 3012},
	{KEY_RIGHT, 754, 915}, // 3013, 3660},
};

static struct msm_ts_virtual_keys zeebo_ts_virtual_keys_y = {
	.keys = &zeebo_ts_keys_y[0],
	.num_keys = 5,
};

static struct msm_ts_platform_data zeebo_ts_pdata = {
	.min_x		= 105, // 420,
	.max_x		= 915, // 3660,
	.min_y		= 0, // 0,
	.max_y		= 837, // 3350,
	.min_press	= 0,
	.max_press	= 256,
	.inv_x		= 0,
	.inv_y		= 970, // 3880,
	.virt_y_start = 862, // 3450, // 3350 + space
	.vkeys_y	= &zeebo_ts_virtual_keys_y,
};

/******************************************************************************
 * Bluetooth
 ******************************************************************************/
#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(MSM7200A_UART2DM_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static struct vreg *vreg_bt;

static int zeebo_bt_power(void *data, bool blocked)
{
	printk(KERN_DEBUG "%s(%s)\n", __func__, blocked ? "off" : "on");

	if (!blocked) {
		vreg_enable(vreg_bt);
		gpio_direction_output(TOPA100_BT_RST, 0);
		mdelay(50);
		gpio_direction_output(TOPA100_BT_RST, 1);
	} else {
		gpio_direction_output(TOPA100_BT_RST, 0);
		vreg_disable(vreg_bt);
	}
	return 0;
}

static int zeebo_bt_init(struct platform_device *pdev)
{
	int rc;

	printk(KERN_DEBUG "%s\n", __func__);

	vreg_bt = vreg_get_by_id(0, 11); // rftx
	if (IS_ERR(vreg_bt)) {
		rc = PTR_ERR(vreg_bt);
		goto fail_vreg_bt;
	}
	rc = gpio_request(TOPA100_BT_RST, "BT Power");
	if (rc)
		goto fail_power_gpio;

	return 0;

fail_power_gpio:
	vreg_put(vreg_bt);
fail_vreg_bt:
	printk(KERN_ERR "%s: failed with %d\n", __func__, rc);
	return rc;
}

static void zeebo_bt_exit(struct platform_device *pdev) {
	printk(KERN_DEBUG "%s\n", __func__);

	gpio_free(TOPA100_BT_RST);
	vreg_put(vreg_bt);
}

static struct msm7200a_rfkill_pdata zeebo_rfkill_data = {
	.init = zeebo_bt_init,
	.exit = zeebo_bt_exit,
	.set_power = zeebo_bt_power,
	.uart_number = 2,
	.rfkill_name = "brf6300",
};

static struct platform_device zeebo_rfkill = {
	.name = "msm7200a_rfkill",
	.id = -1,
	.dev = {
		.platform_data = &zeebo_rfkill_data,
	},
};

/******************************************************************************
 * Platform devices
 ******************************************************************************/

static struct platform_device *devices[] __initdata = {
	&msm_device_uart1,
	&zeebo_amss_device,	
	&msm_device_nand,
	&msm_device_rtc,	
	//&msm_device_htc_hw,
};

extern struct sys_timer msm_timer;

static void __init zeebo_init_irq(void)
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

void msm_proc_comm_wince_enter_sleep(void)
{
    printk("[%s]\n", __func__);
}

void msm_proc_comm_wince_exit_sleep(void)
{
    printk("[%s]\n", __func__);
}

static void __init zeebo_init(void)
{	
	msm_acpu_clock_init(&halibut_clock_data);
	msm_proc_comm_wince_init();

#ifdef CONFIG_MSM_SMEM_BATTCHG
	msm_device_htc_battery_smem.dev.platform_data = &zeebo_htc_battery_smem_pdata;
#endif
	//msm_device_touchscreen.dev.platform_data = &zeebo_ts_pdata;

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
	//i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	//msm_add_usb_devices(&zeebo_hsusb_board_pdata);

	

	//msm_rpc_create_server(&ffff_server);
	//msm_rpc_create_server(&fffe_server);
	//msm_init_pmic_vibrator();
	//msm_proc_comm_wince_vibrate_welcome();
}

static void __init zeebo_map_io(void)
{
	msm_map_common_io();
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init zeebo_fixup(struct machine_desc *desc,
		struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = (80 * 1024 * 1024);

	printk(KERN_INFO "%s: nr_banks = %d\n", __func__, mi->nr_banks);
	printk(KERN_INFO "%s: bank0 start=%08lx, node=%08x, size=%08lx\n", __func__,
		mi->bank[0].start, mi->bank[0].node, mi->bank[0].size);
}

MACHINE_START(ZEEBO, "Zeebo")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io	= MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.fixup = zeebo_fixup,
	.boot_params = 0, //0x10000100,
	.map_io = zeebo_map_io,
	.init_irq = zeebo_init_irq,
	.init_machine = zeebo_init,
	.timer = &msm_timer,
MACHINE_END

static DEFINE_MUTEX(mmoc_mutex);

int __init mmoc_poke(uint32_t d1, uint32_t d2)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[2];
	} req;

	printk(KERN_INFO "%s: trying to poke MMoC...\n", __func__);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);

	mmoc_ep = msm_rpc_connect(0x3000004b, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/4,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_adsprtosatom(uint32_t d1, uint32_t d2, uint32_t d3, uint32_t d4)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[4];
	} req;

	printk(KERN_INFO "%s\n", __func__);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);

	mmoc_ep = msm_rpc_connect(0x3000000a, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/1,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_audmgr(uint32_t proc, uint32_t d)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[1];
	} req;

	printk(KERN_INFO "%s: proc: %02x data: %02x\n", __func__, proc, d);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d);

	mmoc_ep = msm_rpc_connect(0x30000013, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_snd(uint32_t proc, uint32_t d1, uint32_t d2)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[2];
	} req;

	printk(KERN_INFO "%s: proc: %02x data: %08x %08x\n", __func__, proc, d1, d2);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);

	mmoc_ep = msm_rpc_connect(0x30000002, 0xaa2b1a44, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_clkrgm_sec(uint32_t proc, uint32_t d)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[1];
	} req;

	printk(KERN_INFO "%s: proc: %02x data: %02x\n", __func__, proc, d);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d);

	mmoc_ep = msm_rpc_connect(0x3000000f, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_nv(uint32_t proc, uint32_t d1, uint32_t d2, uint32_t d3,
			uint32_t d4, uint32_t d5, uint32_t d6)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[6];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);

	mmoc_ep = msm_rpc_connect(0x3000000e, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_tlmm_remote_atom_0(uint32_t proc)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		//uint32_t data[6];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	/*req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);*/

	mmoc_ep = msm_rpc_connect(0x30000066, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}


int __init rpc_tlmm_remote_atom_1(uint32_t proc, uint32_t d)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[1];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d);
	/*req.data[1] = cpu_to_be32(d2);
	req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);*/

	mmoc_ep = msm_rpc_connect(0x30000066, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_tlmm_remote_atom_2(uint32_t proc, uint32_t d1, uint32_t d2)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[2];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	/*req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);*/

	mmoc_ep = msm_rpc_connect(0x30000066, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}


int __init rpc_pmic(uint32_t proc, uint32_t d1, uint32_t d2)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[2];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	/*req.data[2] = cpu_to_be32(d3);
	req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);*/

	mmoc_ep = msm_rpc_connect(0x30000061, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init rpc_pmapp(uint32_t proc, uint32_t d1, uint32_t d2, uint32_t d3)
{
	struct msm_rpc_endpoint *mmoc_ep;
	int rc;
	struct mmoc_req {
		struct rpc_request_hdr hdr;
		uint32_t data[3];
	} req;

	printk(KERN_INFO "%s: proc: %02x\n", __func__, proc);
	mutex_lock(&mmoc_mutex);

	req.data[0] = cpu_to_be32(d1);
	req.data[1] = cpu_to_be32(d2);
	req.data[2] = cpu_to_be32(d3);
	/*req.data[3] = cpu_to_be32(d4);
	req.data[4] = cpu_to_be32(d5);
	req.data[5] = cpu_to_be32(d6);*/

	mmoc_ep = msm_rpc_connect(0x30000060, 0, 0);
	if (IS_ERR(mmoc_ep)) {
		printk(KERN_ERR "%s: init rpc failed! error: %ld\n",
				__func__, PTR_ERR(mmoc_ep));
		goto close;
	}
	rc = msm_rpc_call(mmoc_ep, /*proc*/ proc,
			&req, sizeof(req), 5 * HZ);
	if (rc < 0)
		printk(KERN_ERR "%s: rpc call failed! (%d)\n", __func__, rc);

close:
	msm_rpc_close(mmoc_ep);
	mutex_unlock(&mmoc_mutex);
	printk(KERN_INFO "%s: exiting\n", __func__);
	return rc;
}

int __init mmoc_init(void)
{
	int rc = 0;

	//msleep(5000);
	
	printk("pmic_vid_en ret=%d\n", pmic_vid_en(1));
	//printk("pmic_set_led_intensity ret=%d\n", pmic_set_led_intensity(3, 0));

	printk("rpc_adsprtosatom1 ret=%d\n", rpc_adsprtosatom(1, 0, 2 ,4));
	printk("rpc_adsprtosatom2 ret=%d\n", rpc_adsprtosatom(1, 0, 2 ,0xd));
	printk("rpc_adsprtosatom3 ret=%d\n", rpc_adsprtosatom(1, 0, 2 ,0xe));

	printk("rpc_audmgr ret=%d\n", rpc_audmgr(4, 0));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x32));

	printk("rpc_audmgr ret=%d\n", rpc_audmgr(6, 1));

	printk("rpc_snd ret=%d\n", rpc_snd(0x1f, 0xd, 0x2));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x2f));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_nv ret=%d\n", rpc_nv(0xa, 0, 1, 0x15dc, 0, 0, 0));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(9));
	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(0xe));
	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_2(0x17, 0xd, 0));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_1(3, 0x19f));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x18));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x2f));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x28));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x15, 0x5));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x33));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x43));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(9));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x29));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_pmic ret=%d\n", rpc_pmic(4, 0x12, 0xbea));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x25, 0));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_pmapp ret=%d\n", rpc_pmapp(4, 1, 0x12, 1));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(9));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_audmgr ret=%d\n", rpc_audmgr(6, 3));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x25, 0x1));

	printk("rpc_pmic ret=%d\n", rpc_pmic(4, 0x5, 0x708));

	printk("rpc_pmapp ret=%d\n", rpc_pmapp(4, 1, 5, 1));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x26, 0x247c550));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x15, 0x0));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x25, 0));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x43));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(8, 0x60));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(8, 0x36));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(8, 0x37));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x25, 0x1));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(0xa));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x26, 0x16e3600));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(6, 0x2f));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(9));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));

	printk("rpc_tlmm_remote_atom ret=%d\n", rpc_tlmm_remote_atom_0(9));

	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(0x15, 0x8));
	printk("clkrgm5 ret=%d\n", rpc_clkrgm_sec(2, 0x3c));
	

	printk("------SMSM------\n");
	printk("BASE = %08x\n", MSM_SHARED_RAM_BASE);
	printk("smsm_get_state APPS = %x\n", readl(MSM_SHARED_RAM_BASE + 0x1e28 + 1 * 4));
	printk("smsm_get_state MDM  = %x\n", readl(MSM_SHARED_RAM_BASE + 0x1e28 + 3 * 4));
	
//	rc = platform_device_register(&msm_device_mdp);
//	if (rc)
//		return rc;

	msleep(5000);
	
	printk("mmoc_poke(0x00000002, 0x00000000) ret=%d\n", mmoc_poke(0x00000002, 0x00000008));
	printk("mmoc_poke(0x00000004, 0x00000000) ret=%d\n", mmoc_poke(0x00000004, 0x00000008));
	printk("mmoc_poke(0x00000007, 0x00000000) ret=%d\n", mmoc_poke(0x00000007, 0x00000008));
	
	
	zeebo_init_mmc();

	/*
	printk("mmoc_poke(0x00000002, 0x00000008) ret=%d\n", mmoc_poke(0x00000002, 0x00000008));
	printk("mmoc_poke(0x00000004, 0x00000008) ret=%d\n", mmoc_poke(0x00000004, 0x00000008));
	printk("mmoc_poke(0x00000007, 0x00000008) ret=%d\n", mmoc_poke(0x00000007, 0x00000008));
	*/

	return 0;
}

late_initcall(mmoc_init);

int __init zeebo_init_panel(void)
{
	int rc;

	if(!machine_is_zeebo()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return -1;
	}

	printk(KERN_INFO "%s: Initializing Zeebo panel/TV Out\n", __func__);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	/*vreg_lcd_1 = vreg_get(0, "rftx");
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

	msm_device_mddi0.dev.platform_data = &mddi_pdata;*/
	return rc;
}

//device_initcall(zeebo_init_panel);
