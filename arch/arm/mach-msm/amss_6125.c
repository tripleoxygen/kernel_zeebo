/* linux/arch/arm/mach-msm/amss_6125.c
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
#include <mach/amss/amss_6125.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>

struct amss_value amss_6125_para[] = {
	{AMSS_AUDMGR_PROG, AMSS_VAL_UINT, {.value = 0x30000013}},
	{AMSS_AUDMGR_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_AUDMGR_CB_PROG, AMSS_VAL_UINT, {.value = 0x31000013}},
	{AMSS_AUDMGR_CB_VERS, AMSS_VAL_UINT, { .value = 0x0}},
	{AMSS_TIME_REMOTE_MTOA_VERS, AMSS_VAL_UINT, {.value = 0}},
	{AMSS_TIME_TOD_SET_APPS_BASES, AMSS_VAL_UINT, {.value = 1}},
};

static struct platform_device adsp_device = {
	.name = "msm_adsp_6125",
	.id = -1,
};

static struct msm_smd_platform_data smd_pdata_6125 = {
	.amss_values = amss_6125_para,
	.n_amss_values = ARRAY_SIZE(amss_6125_para),
	.use_v2_alloc_elm = true,
};

static const struct smd_tty_channel_desc smd_6125_tty_channels[] = {
	{.id = 0,.name = "SMD_DS"},
	{.id = 1,.name = "SMD_DIAG"},
	{.id = 7,.name = "SMD_DATA1"},
};

static struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.dev = {
		.platform_data = &smd_pdata_6125,
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

static struct msm_snd_platform_data amss_6125_snd_pdata = {
	.endpoints = snd_endpoints_list,
	.num_endpoints = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device amss_6125_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &amss_6125_snd_pdata,
	},
};

/******************************************************************************
 * Acoustic driver settings
 ******************************************************************************/
static struct htc_acoustic_wce_amss_data amss_6125_acoustic_data = {
	.volume_table = (MSM_SHARED_RAM_BASE+0xfc300),
	.ce_table = (MSM_SHARED_RAM_BASE+0xfc600),
	.adie_table = (MSM_SHARED_RAM_BASE+0xf8000),
	.codec_table = (MSM_SHARED_RAM_BASE+0xf9000),
	.mic_offset = (MSM_SHARED_RAM_BASE+0xfb9c0),
	.voc_cal_field_size = 0xb,
};

static struct platform_device acoustic_device = {
	.name = "htc_acoustic",
	.id = -1,
	.dev = {
		.platform_data = &amss_6125_acoustic_data,
	},
};

static int amss_6125_probe(struct platform_device *pdev)
{
	int ret;

	//do it before anything rpc kicks in
	printk(KERN_INFO "%s: registering smd tty channels\n", __func__);
	smd_set_channel_list(smd_6125_tty_channels, ARRAY_SIZE(smd_6125_tty_channels));

	//We're in a serious trouble
	printk(KERN_INFO "%s: registering smd device\n", __func__);
	ret = platform_device_register(&msm_device_smd);
	if (ret) {
		printk(KERN_CRIT "%s: failed to register SMD driver\n", __func__);
		return ret;
	}

	//We can live without adsp, though
	printk(KERN_INFO "%s: registering adsp device\n", __func__);
	ret = platform_device_register(&adsp_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register ADSP driver\n", __func__);
		return 0;
	}

	printk(KERN_INFO "%s: registering snd device\n", __func__);
	ret = platform_device_register(&amss_6125_snd);
	if (ret) {
		printk(KERN_ERR "%s: failed to register SND driver\n", __func__);
		return 0;
	}

	printk(KERN_INFO "%s: registering acoustic device\n", __func__);
	ret = platform_device_register(&acoustic_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register acoustic driver\n", __func__);
	}

	return 0;
}

static struct platform_driver amss_6125_driver = {
	.probe		= amss_6125_probe,
	.driver		= {
		.name		= "amss_6125",
		.owner		= THIS_MODULE,
	},
};

static int __init amss_6125_init(void)
{
	return platform_driver_register(&amss_6125_driver);
}

module_init(amss_6125_init);
