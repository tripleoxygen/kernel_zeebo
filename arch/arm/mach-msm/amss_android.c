/* linux/arch/arm/mach-msm/amss_android.c
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
#include <mach/htc_acoustic_wince.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>

#if defined(CONFIG_MSM_AMSS_VERSION_6225)
#include <mach/amss/amss_6225.h>
static struct platform_device adsp_device_6225 = {
	.name = "msm_adsp_6225",
	.id = -1,
};

static struct msm_smd_platform_data smd_pdata_6225 = {
	.amss_values = amss_6225_para,
	.n_amss_values = ARRAY_SIZE(amss_6225_para),
};
#endif

#if defined(CONFIG_MSM_AMSS_VERSION_6220)
#include <mach/amss/amss_6220.h>
static struct platform_device adsp_device_6220 = {
	.name = "msm_adsp_6220",
	.id = -1,
};

static struct msm_smd_platform_data smd_pdata_6220 = {
	.amss_values = amss_6220_para,
	.n_amss_values = ARRAY_SIZE(amss_6220_para),
};
#endif

#if defined(CONFIG_MSM_AMSS_VERSION_6210)
#include <mach/amss/amss_6210.h>
static struct platform_device adsp_device_6210 = {
	.name = "msm_adsp_6210",
	.id = -1,
};

static struct msm_smd_platform_data smd_pdata_6210 = {
	.amss_values = amss_6210_para,
	.n_amss_values = ARRAY_SIZE(amss_6210_para),
};
#endif

static struct platform_device msm_device_smd = {
	.name	= "msm_smd",
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
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

	/* Bluetooth accessories. */

	SND(12, "HTC BH S100"),
	SND(13, "HTC BH M100"),
	SND(14, "Motorola H500"),
	SND(15, "Nokia HS-36W"),
	SND(16, "PLT 510v.D"),
	SND(17, "M2500 by Plantronics"),
	SND(18, "Nokia HDW-3"),
	SND(19, "HBH-608"),
	SND(20, "HBH-DS970"),
	SND(21, "i.Tech BlueBAND"),
	SND(22, "Nokia BH-800"),
	SND(23, "Motorola H700"),
	SND(24, "HTC BH M200"),
	SND(25, "Jabra JX10"),
	SND(26, "320Plantronics"),
	SND(27, "640Plantronics"),
	SND(28, "Jabra BT500"),
	SND(29, "Motorola HT820"),
	SND(30, "HBH-IV840"),
	SND(31, "6XXPlantronics"),
	SND(32, "3XXPlantronics"),
	SND(33, "HBH-PV710"),
	SND(34, "Motorola H670"),
	SND(35, "HBM-300"),
	SND(36, "Nokia BH-208"),
	SND(37, "Samsung WEP410"),
	SND(38, "Jabra BT8010"),
	SND(39, "Motorola S9"),
	SND(40, "Jabra BT620s"),
	SND(41, "Nokia BH-902"),
	SND(42, "HBH-DS220"),
	SND(43, "HBH-DS980"),
};
#undef SND

static struct msm_snd_platform_data amss_android_snd_pdata = {
	.endpoints = snd_endpoints_list,
	.num_endpoints = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device amss_android_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &amss_android_snd_pdata,
		},
};

static int amss_android_probe(struct platform_device *pdev)
{
	int ret;

	//We're in a serious trouble
	ret = platform_device_register(&msm_device_smd);
	if (ret) {
		printk(KERN_CRIT "%s: failed to register SMD driver\n", __func__);
		return ret;
	}
#if defined(CONFIG_MSM_AMSS_VERSION_6225)
	msm_device_smd.dev.platform_data = &smd_pdata_6225;
	ret = platform_device_register(&adsp_device_6225);
#elif defined(CONFIG_MSM_AMSS_VERSION_6220)
	msm_device_smd.dev.platform_data = &smd_pdata_6220;
	ret = platform_device_register(&adsp_device_6220);
#elif defined(CONFIG_MSM_AMSS_VERSION_6210)
	msm_device_smd.dev.platform_data = &smd_pdata_6210;
	ret = platform_device_register(&adsp_device_6210);
#else
	return 0;
#endif
	if (ret) {
		printk(KERN_ERR "%s: failed to register ADSP driver\n", __func__);
		return 0;
	}

	ret = platform_device_register(&amss_android_snd);
	if (ret) {
		printk(KERN_ERR "%s: failed to register SND driver\n", __func__);
		return 0;
	}

	return 0;
}

static struct platform_driver amss_android_driver = {
	.probe		= amss_android_probe,
	.driver		= {
		.name		= "amss_android",
		.owner		= THIS_MODULE,
	},
};

static int __init amss_android_init(void)
{
	return platform_driver_register(&amss_android_driver);
}

module_init(amss_android_init);
