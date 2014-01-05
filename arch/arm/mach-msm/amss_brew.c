/* linux/arch/arm/mach-msm/amss_brew.c
 *
 * Copyright (C) 2011 htc-linux.org
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

#include <linux/platform_device.h>
#include <linux/msm_audio.h>
#include <mach/amss/amss_brew.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>

struct amss_value amss_brew_para[] = {
	{AMSS_AUDMGR_PROG, AMSS_VAL_UINT, {.value = 0x30000013}},
	{AMSS_AUDMGR_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_AUDMGR_CB_PROG, AMSS_VAL_UINT, {.value = 0x31000013}},
	{AMSS_AUDMGR_CB_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_TIME_REMOTE_MTOA_VERS, AMSS_VAL_UINT, {.value = 0}},
	{AMSS_TIME_TOD_SET_APPS_BASES, AMSS_VAL_UINT, {.value = 1}},
	{AMSS_RPC_DOG_KEEPALIVE_PROG, AMSS_VAL_UINT, {.value = 0x30000015}},
	{AMSS_RPC_DOG_KEEPALIVE_VERS, AMSS_VAL_UINT, {.value = 0x0}},
	{AMSS_RPC_DOG_KEEPALIVE_BEACON, AMSS_VAL_UINT, {.value = 1}},
};

static struct msm_early_server smd_brew_early_servers[] = {
	/*{
		.pid = 1,
		.cid = 0x1,
		.prog = 0x3000fffe,
		.vers = 1,
	},
	{
		.pid = 1,
		.cid = 0x1,
		.prog = 0x3000ffff,
		.vers = 0,
	},*/
	{ .pid = 1, .cid = 0x1, .prog = 0x3000ffff, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000fffe, .vers = 0x00000001, },
	/*{ .pid = 1, .cid = 0x1, .prog = 0x3000000b, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000000, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000004c, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000004d, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000016, .vers = 0x6639dad6, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000019, .vers = 0xd3f684e3, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3100003c, .vers = 0x6e5e6aab, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000002, .vers = 0x71e691ca, },
	*/
	{ .pid = 1, .cid = 0x1, .prog = 0x31000014, .vers = 0x00000000, },/*
	{ .pid = 1, .cid = 0x1, .prog = 0x31000013, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000003, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3100005b, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000001d, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000059, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000005d, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000006, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000053, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000010, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000011, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000012, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000015, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3100003b, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000038, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000039, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x31000045, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000046, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000052, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000058, .vers = 0x00000001, },
	
	{ .pid = 1, .cid = 0x1, .prog = 0x30000062, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000063, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3100006b, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3100006c, .vers = 0x00000000, },
	{ .pid = 1, .cid = 0x1, .prog = 0x3000006d, .vers = 0x00000001, },
	{ .pid = 1, .cid = 0x1, .prog = 0x30000074, .vers = 0x00000000, },
	*/
};

static struct platform_device adsp_device = {
	.name = "msm_adsp_brew",
	.id = -1,
};

static struct msm_smd_platform_data smd_pdata_brew = {
	.amss_values = amss_brew_para,
	.n_amss_values = ARRAY_SIZE(amss_brew_para),
	.use_v2_alloc_elm = false,
	.early_servers = smd_brew_early_servers,
	.n_early_servers = ARRAY_SIZE(smd_brew_early_servers),
};

static const struct smd_tty_channel_desc smd_brew_tty_channels[] = {
	{.id = 1,.name = "SMD_DIAG"},
};

static struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.dev = {
		.platform_data = &smd_pdata_brew,
	},
	.id	= -1,
};

/******************************************************************************
 * Sound driver settings
 ******************************************************************************/
#define SND(num, desc) { .name = desc, .id = num }
static struct msm_snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(2, "NO_MIC_HEADSET"),
	SND(3, "BT"),
	SND(3, "BT_EC_OFF"),

	SND(13, "SPEAKER_MIC"), // RHOD only?

	SND(0x11, "IDLE"),
	SND(256, "CURRENT"),
};
#undef SND

static struct msm_snd_platform_data amss_brew_snd_pdata = {
	.endpoints = snd_endpoints_list,
	.num_endpoints = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device amss_brew_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &amss_brew_snd_pdata,
	},
};

/******************************************************************************
 * Acoustic driver settings
 ******************************************************************************/
static struct msm_rpc_endpoint *mic_endpoint = NULL;
 
static void amss_brew_mic_bias_callback(bool on, bool enable_dualmic) {
	struct {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;

	if (!mic_endpoint)
		mic_endpoint = msm_rpc_connect(0x30000061, 0x0, 0);
	if (!mic_endpoint) {
		printk(KERN_ERR "%s: couldn't open rpc endpoint\n", __func__);
		return;
	}
	req.data=cpu_to_be32(on);
	msm_rpc_call(mic_endpoint, 0x1c, &req, sizeof(req), 5 * HZ);
}
 
static struct htc_acoustic_wce_amss_data amss_brew_acoustic_data = {
	.volume_table = (MSM_SHARED_RAM_BASE+0xfc300),
	.ce_table = (MSM_SHARED_RAM_BASE+0xfc600),
	.adie_table = (MSM_SHARED_RAM_BASE+0xf8000),
	.codec_table = (MSM_SHARED_RAM_BASE+0xf9000),
	.mic_offset = (MSM_SHARED_RAM_BASE+0xfb9c0),
	.voc_cal_field_size = 0xb,
	.mic_bias_callback = amss_brew_mic_bias_callback,
};

static struct platform_device acoustic_device = {
	.name = "htc_acoustic_wince",
	.id = -1,
	.dev = {
		.platform_data = &amss_brew_acoustic_data,
	},
};

static int amss_brew_probe(struct platform_device *pdev)
{
	int ret;

	//do it before anything rpc kicks in
	printk(KERN_INFO "%s: registering smd tty channels\n", __func__);
	smd_set_channel_list(smd_brew_tty_channels, ARRAY_SIZE(smd_brew_tty_channels));

	//We're in a serious trouble
	printk(KERN_INFO "%s: registering smd device\n", __func__);
	ret = platform_device_register(&msm_device_smd);
	if (ret) {
		printk(KERN_CRIT "%s: failed to register SMD driver\n", __func__);
		return ret;
	}

/*
	//We can live without adsp, though
	printk(KERN_INFO "%s: registering adsp device\n", __func__);
	ret = platform_device_register(&adsp_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register ADSP driver\n", __func__);
		return 0;
	}

	printk(KERN_INFO "%s: registering snd device\n", __func__);
	ret = platform_device_register(&amss_brew_snd);
	if (ret) {
		printk(KERN_ERR "%s: failed to register SND driver\n", __func__);
		return 0;
	}

	printk(KERN_INFO "%s: registering acoustic device\n", __func__);
	ret = platform_device_register(&acoustic_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register acoustic driver\n", __func__);
	}
*/
	return 0;
}

static struct platform_driver amss_brew_driver = {
	.probe		= amss_brew_probe,
	.driver		= {
		.name		= "amss_brew",
		.owner		= THIS_MODULE,
	},
};

static int __init amss_brew_init(void)
{
	return platform_driver_register(&amss_brew_driver);
}

module_init(amss_brew_init);
