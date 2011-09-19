/* linux/arch/arm/mach-msm/amss_5225.c
 *
 * Copyright (C) 2011 htc-linux.org
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
#include <mach/amss/amss_5225.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>

struct amss_value amss_5225_para[] = {
	{AMSS_AUDMGR_PROG, AMSS_VAL_UINT, {.value = 0x30000013}},
	{AMSS_AUDMGR_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_AUDMGR_CB_PROG, AMSS_VAL_UINT, {.value = 0x31000013}},
	{AMSS_AUDMGR_CB_VERS, AMSS_VAL_UINT, { .value = 0x5fa922a9}},
	{AMSS_TIME_REMOTE_MTOA_VERS, AMSS_VAL_UINT, {.value = 0}},
	{AMSS_TIME_TOD_SET_APPS_BASES, AMSS_VAL_UINT, {.value = 1}},
};

static struct platform_device adsp_device = {
	.name = "msm_adsp_5225",
	.id = -1,
};

/******************************************************************************
 * SMD driver settings
 ******************************************************************************/
static struct msm_early_server smd_5225_early_servers[] = {
	{
		.pid = 1,
		.cid = 0xfadefade,
		.prog = 0x3000fffe,
		.vers = 1,
	}
};

static struct msm_smd_platform_data smd_pdata_5225 = {
	.amss_values = amss_5225_para,
	.n_amss_values = ARRAY_SIZE(amss_5225_para),
	.early_servers = smd_5225_early_servers,
	.n_early_servers = ARRAY_SIZE(smd_5225_early_servers),
};

static const struct smd_tty_channel_desc smd_5225_tty_channels[] = {
	{.id = 0,.name = "SMD_DS"},
	{.id = 1,.name = "SMD_DIAG"},
	{.id = 7,.name = "SMD_DATA1"},
};

static struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.dev = {
		.platform_data = &smd_pdata_5225,
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

	SND(0xd, "IDLE"),
	SND(0xd, "CURRENT"),
};
#undef SND

static struct msm_snd_platform_data amss_5225_snd_pdata = {
	.endpoints = snd_endpoints_list,
	.num_endpoints = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device amss_5225_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &amss_5225_snd_pdata,
	},
};

/******************************************************************************
 * Acoustic driver settings
 ******************************************************************************/
static struct msm_rpc_endpoint *mic_endpoint = NULL;

static void amss_5225_mic_bias_callback(bool on, bool enable_dualmic) {
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

static struct htc_acoustic_wce_amss_data amss_5225_acoustic_data = {
	.volume_table = (MSM_SHARED_RAM_BASE+0xfc300),
	.ce_table = (MSM_SHARED_RAM_BASE+0xfc600),
	.adie_table = (MSM_SHARED_RAM_BASE+0xfd000),
	.codec_table = (MSM_SHARED_RAM_BASE+0xfdc00),
	.mic_offset = (MSM_SHARED_RAM_BASE+0xfed00),
	.voc_cal_field_size = 0xa,
	.mic_bias_callback = amss_5225_mic_bias_callback,
};

static struct platform_device acoustic_device = {
	.name = "htc_acoustic_wince",
	.id = -1,
	.dev = {
		.platform_data = &amss_5225_acoustic_data,
	},
};

static int amss_5225_probe(struct platform_device *pdev)
{
	int ret;
	//do it before anything rpc kicks in
	smd_set_channel_list(smd_5225_tty_channels, ARRAY_SIZE(smd_5225_tty_channels));

	//We're in a serious trouble
	ret = platform_device_register(&msm_device_smd);
	if (ret) {
		printk(KERN_CRIT "%s: failed to register SMD driver\n", __func__);
		return ret;
	}

	//We can live without adsp, though
	ret = platform_device_register(&adsp_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register ADSP driver\n", __func__);
		return 0;
	}

	ret = platform_device_register(&amss_5225_snd);
	if (ret) {
		printk(KERN_ERR "%s: failed to register SND driver\n", __func__);
		return 0;
	}

	ret = platform_device_register(&acoustic_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register acoustic driver\n", __func__);
	}

	return 0;
}

static struct platform_driver amss_5225_driver = {
	.probe		= amss_5225_probe,
	.driver		= {
		.name		= "amss_5225",
		.owner		= THIS_MODULE,
	},
};

static int __init amss_5225_init(void)
{
	return platform_driver_register(&amss_5225_driver);
}

module_init(amss_5225_init);
