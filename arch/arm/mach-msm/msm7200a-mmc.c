/* linux/arch/arm/mach-msm/board-msm7200a-mmc.c
*
* Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
* based on previous work which is
* Copyright (C) 2008-2009 Octavian Voicu, Martijn Stolk
* Copyright (C) 2007-2008 Brian Swetland <swetland@google.com>
* Copyright (C) 2007 Google, Inc.
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
#include <linux/err.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/spi/wl12xx.h>

#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/board.h>
#include <mach/mmc.h>
#include <mach/vreg.h>
#include <mach/msm7200a_mmc.h>

#include "devices.h"

/******************************************************************************
 * GPIO Tables
 ******************************************************************************/
static struct msm_gpio sdc1_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(51, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_4MA),.label = "MMC1_DAT3"},
	{.gpio_cfg = GPIO_CFG(52, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_4MA),.label = "MMC1_DAT2"},
	{.gpio_cfg = GPIO_CFG(53, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_4MA),.label = "MMC1_DAT1"},
	{.gpio_cfg = GPIO_CFG(54, 1, GPIO_CFG_OUTPUT,	
		GPIO_CFG_PULL_UP, GPIO_CFG_4MA),.label = "MMC1_DAT0"},
	{.gpio_cfg = GPIO_CFG(55, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC1_CMD"},
	{.gpio_cfg = GPIO_CFG(56, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC1_CLK"},
};

static struct msm_gpio sdc1_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(51, 0, GPIO_CFG_OUTPUT,	
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT3"},
	{.gpio_cfg = GPIO_CFG(52, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT2"},
	{.gpio_cfg = GPIO_CFG(53, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT1"},
	{.gpio_cfg = GPIO_CFG(54, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT0"},
	{.gpio_cfg = GPIO_CFG(55, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_CMD"},
	{.gpio_cfg = GPIO_CFG(56, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_CLK"},
};

static struct msm_gpio sdc2_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(62, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC2_CLK"},
	{.gpio_cfg = GPIO_CFG(63, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC2_CMD"},
	{.gpio_cfg = GPIO_CFG(64, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC2_DAT3"},
	{.gpio_cfg = GPIO_CFG(65, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC2_DAT2"},
	{.gpio_cfg = GPIO_CFG(66, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC2_DAT1"},
	{.gpio_cfg = GPIO_CFG(67, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC2_DAT0"},
};

static struct msm_gpio sdc2_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(62, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_CLK"},
	{.gpio_cfg = GPIO_CFG(63, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_CMD"},
	{.gpio_cfg = GPIO_CFG(64, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_DAT3"},
	{.gpio_cfg = GPIO_CFG(65, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_DAT2"},
	{.gpio_cfg = GPIO_CFG(66, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_DAT1"},
	{.gpio_cfg = GPIO_CFG(67, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC2_DAT0"},
};

static struct msm_gpio sdc3_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(88, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_CLK"},
	{.gpio_cfg = GPIO_CFG(89, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC3_CMD"},
	{.gpio_cfg = GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC3_DAT3"},
	{.gpio_cfg = GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC3_DAT2"},
	{.gpio_cfg = GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC3_DAT1"},
	{.gpio_cfg = GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC3_DAT0"},
};

static struct msm_gpio sdc3_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(88, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_CLK"},
	{.gpio_cfg = GPIO_CFG(89, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_CMD"},
	{.gpio_cfg = GPIO_CFG(90, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT3"},
	{.gpio_cfg = GPIO_CFG(91, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT2"},
	{.gpio_cfg = GPIO_CFG(92, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT1"},
	{.gpio_cfg = GPIO_CFG(93, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT0"},
};

static struct msm_gpio sdc4_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(142, 3, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC4_CLK"},
	{.gpio_cfg = GPIO_CFG(143, 3, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC4_CMD"},
	{.gpio_cfg = GPIO_CFG(144, 2, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC4_DAT3"},
	{.gpio_cfg = GPIO_CFG(145, 2, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC4_DAT2"},
	{.gpio_cfg = GPIO_CFG(146, 3, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC4_DAT1"},
	{.gpio_cfg = GPIO_CFG(147, 3, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC4_DAT0"},
};

static struct msm_gpio sdc4_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(142, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_CLK"},
	{.gpio_cfg = GPIO_CFG(143, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_CMD"},
	{.gpio_cfg = GPIO_CFG(144, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_DAT3"},
	{.gpio_cfg = GPIO_CFG(145, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_DAT2"},
	{.gpio_cfg = GPIO_CFG(146, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_DAT1"},
	{.gpio_cfg = GPIO_CFG(146, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC4_DAT0"},
};
/******************************************************************************
 * Common code and data structures
 ******************************************************************************/
#define MSM_MMC_VDD	MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 \
			| MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 \
			| MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 \
			| MMC_VDD_28_29 | MMC_VDD_29_30

static struct mmc_vdd_xlat {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{MMC_VDD_165_195, 1800},
	{MMC_VDD_20_21, 2050},
	{MMC_VDD_21_22, 2150},
	{MMC_VDD_22_23, 2250},
	{MMC_VDD_23_24, 2350},
	{MMC_VDD_24_25, 2450},
	{MMC_VDD_25_26, 2550},
	{MMC_VDD_26_27, 2650},
	{MMC_VDD_27_28, 2750},
	{MMC_VDD_28_29, 2850},
	{MMC_VDD_29_30, 2950},
};

struct msm7200a_sdcc_gpios {
	struct msm_gpio *on;
	struct msm_gpio *off;

	unsigned on_length;
	unsigned off_length;
};

static void msm7200a_setup_gpios(unsigned sdcc_id,
	struct msm7200a_sdcc_gpios* gpios) {
	
	if (!gpios) {
		printk(KERN_ERR "%s: gpios structure is NULL\n", __func__);
		return;
	}

	switch (sdcc_id) {
		case 1:
			gpios->on = sdc1_on_gpio_table;
			gpios->off = sdc1_off_gpio_table;
			gpios->on_length = ARRAY_SIZE(sdc1_on_gpio_table);
			gpios->off_length = ARRAY_SIZE(sdc1_off_gpio_table);
			break;
		case 2:
			gpios->on = sdc2_on_gpio_table;
			gpios->off = sdc2_off_gpio_table;
			gpios->on_length = ARRAY_SIZE(sdc2_on_gpio_table);
			gpios->off_length = ARRAY_SIZE(sdc2_off_gpio_table);
			break;
		case 3:
			gpios->on = sdc3_on_gpio_table;
			gpios->off = sdc3_off_gpio_table;
			gpios->on_length = ARRAY_SIZE(sdc3_on_gpio_table);
			gpios->off_length = ARRAY_SIZE(sdc3_off_gpio_table);
			break;
		case 4:
			gpios->on = sdc4_on_gpio_table;
			gpios->off = sdc4_off_gpio_table;
			gpios->on_length = ARRAY_SIZE(sdc4_on_gpio_table);
			gpios->off_length = ARRAY_SIZE(sdc4_off_gpio_table);
			break;
		default:
			printk(KERN_ERR "%s: invalid sdcc id %d\n", __func__, sdcc_id);
	}
}

/******************************************************************************
 * SD Slot
 ******************************************************************************/
static struct msm7200a_sdslot_priv {
	struct vreg *vreg;	
	struct msm7200a_mmc_pdata* pdata;
	struct msm7200a_sdcc_gpios gpios;
} sdslot_priv;

static uint32_t sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc, i;
	if (vdd) {
		msm_gpios_enable(sdslot_priv.gpios.on, sdslot_priv.gpios.on_length);
		if (!sdslot_priv.vreg)
			return 0;
		
		rc = vreg_enable(sdslot_priv.vreg);
		if (rc)
			printk(KERN_ERR
			       "%s: Error enabling sd slot error code %d\n",
			       __func__, rc);

		for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
			if (mmc_vdd_table[i].mask == (1 << vdd)) {
				rc = vreg_set_level(sdslot_priv.vreg, mmc_vdd_table[i].level);
				if (rc) {
					printk(KERN_ERR
						   "%s: Error setting vreg level (%d)\n",
						   __func__, rc);
				}
				return rc;
			}
		}
		printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	} else {
		if (sdslot_priv.vreg) {
			rc = vreg_disable(sdslot_priv.vreg);
			if (rc) {
				printk(KERN_ERR
			    	   "%s: Error disabling sd slot error code %d\n",
				       __func__, rc);
				return rc;
			}
		}
		msm_gpios_disable(sdslot_priv.gpios.off, sdslot_priv.gpios.off_length);
	}

	return 0;
}

static unsigned int msm7200a_sdslot_get_status(struct device *dev)
{
	if (!sdslot_priv.pdata || sdslot_priv.pdata->gpio_detection < 0)
		return 1;

	return !gpio_get_value(sdslot_priv.pdata->gpio_detection);
}

static struct mmc_platform_data msm7200a_sdslot_data = {
	.ocr_mask = MSM_MMC_VDD,
	.status = msm7200a_sdslot_get_status,
	.translate_vdd = sdslot_switchvdd,
};

static int msm7200a_mmc_probe(struct platform_device *pdev)
{
	int rc;
	struct vreg* vreg;
	struct msm7200a_mmc_pdata *pdata = pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_ERR "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	if (pdata->slot_number < 1 || pdata->slot_number > 4) {
		printk(KERN_ERR "%s: invalid slot number %d\n", __func__,
			pdata->slot_number);
		return -EINVAL;
	}

	if (pdata->gpio_detection >= 0) {
		rc = gpio_request(pdata->gpio_detection, "SD Status");
		if (rc) {
			goto err_gpio;
		}
	}

	if (pdata->vreg_id >= 0) {
		vreg = vreg_get_by_id(&pdev->dev, pdata->vreg_id);
		if (IS_ERR(vreg)) {
			rc = PTR_ERR(vreg);
			goto err_vreg;
		}
		sdslot_priv.vreg = vreg;
	}

	msm7200a_setup_gpios(pdata->slot_number, &sdslot_priv.gpios);
	rc = msm_gpios_request(sdslot_priv.gpios.on, sdslot_priv.gpios.on_length);
	if (rc) {
		goto err_sdcc_gpios;
	}
	
	sdslot_priv.pdata = pdata;

	if (pdata->gpio_detection >= 0) {
		set_irq_wake(gpio_to_irq(pdata->gpio_detection), 1);
		rc = msm_add_sdcc(pdata->slot_number, &msm7200a_sdslot_data,
			gpio_to_irq(pdata->gpio_detection),
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	}
	else {
		rc = msm_add_sdcc(pdata->slot_number, &msm7200a_sdslot_data, 0, 0);
	}
	
	if (rc) {
		goto err_sdcc_dev;
	}

	return 0;

err_sdcc_dev:
	sdslot_priv.pdata = NULL;
	if (pdata->gpio_detection >= 0) {
		set_irq_wake(gpio_to_irq(pdata->gpio_detection), 0);
	}
err_sdcc_gpios:
	if (sdslot_priv.vreg) {
		vreg_put(sdslot_priv.vreg);
		sdslot_priv.vreg = NULL;
	}
err_vreg:
	if (pdata->gpio_detection >= 0) {
		gpio_free(pdata->gpio_detection);
	}
err_gpio:
	return rc;
}

static int msm7200a_mmc_remove(struct platform_device *pdev) {
	msm_delete_sdcc(sdslot_priv.pdata->slot_number);
	msm_gpios_disable_free(sdslot_priv.gpios.off, sdslot_priv.gpios.off_length);
	if (sdslot_priv.vreg) {
		vreg_put(sdslot_priv.vreg);
		sdslot_priv.vreg = NULL;
	}
	if (sdslot_priv.pdata->gpio_detection >= 0) {
		set_irq_wake(gpio_to_irq(sdslot_priv.pdata->gpio_detection), 0);
		gpio_free(sdslot_priv.pdata->gpio_detection);
	}
	sdslot_priv.pdata = NULL;
	return 0;	
}

static struct platform_driver msm7200a_mmc_driver = {
	.probe = msm7200a_mmc_probe,
	.remove = msm7200a_mmc_remove,
	.driver = {
		   .name = MSM7200A_MMC_DRV_NAME,
		   .owner = THIS_MODULE,
	},
};
/******************************************************************************
 * WiFi SDIO
 ******************************************************************************/
// use a platform device to pass irq etc until we upgrade to the latest driver
#define MSM7200A_WL1251_HACK 0

static struct msm7200a_wl1251_priv {
	struct vreg *vreg;	
	struct msm7200a_wl1251_pdata* pdata;
	struct msm7200a_sdcc_gpios gpios;
	bool state;
} wl1251_priv;

#if MSM7200A_WL1251_HACK
static void fake_wifi_enable(bool enable) {
}

static struct wl12xx_platform_data wl1251_pdata = {
	.set_power = fake_wifi_enable,
};

static struct platform_device wl1251_device = {
	.id = -1,
	.name = "wl1251_data",
	.dev = {
		.platform_data = &wl1251_pdata,
	}
};
#endif

static struct sdio_embedded_func wifi_func = {
	.f_class = SDIO_CLASS_WLAN,
	.f_maxblksize = 512,
};

static struct embedded_sdio_data ti_wifi_emb_data = {
	.cis = {
		.vendor = 0x104c,
		.device = 0x9066,
		.blksize = 512,
		.max_dtr = 11000000,
		},
	.cccr = {
		 .multi_block = 0,
		 .low_speed = 0,
		 .wide_bus = 1,
		 .high_power = 0,
		 .high_speed = 0,
		 },
	.funcs = &wifi_func,
	.num_funcs = 1,
};

static uint32_t wifi_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc;
	printk("%s: vdd=%d\n", __func__, vdd);

	if (vdd) {
		if (!wl1251_priv.state) {
			if (wl1251_priv.vreg) {
				rc = vreg_enable(wl1251_priv.vreg);
				if (rc) {
					printk(KERN_ERR "%s: failed to enable vreg %d\n",
						   __func__, rc);
					goto pwroff;
				}
			}
			wl1251_priv.state = true;
		}
		mdelay(10);
		msm_gpios_enable(wl1251_priv.gpios.on, wl1251_priv.gpios.on_length);
		
		//We need to set power here to make the card identify
		if (wl1251_priv.pdata->gpio_32k_osc >= 0) {
			gpio_direction_output(wl1251_priv.pdata->gpio_32k_osc, 1);
		}
		mdelay(100);
		if (wl1251_priv.pdata->gpio_reset >= 0) {
			gpio_set_value(wl1251_priv.pdata->gpio_reset, 0);
			mdelay(50);
		}
		if (wl1251_priv.pdata->gpio_enable >= 0) {
			gpio_direction_output(wl1251_priv.pdata->gpio_enable, 1);
		}
		mdelay(100);
		return 0;
	}

pwroff:
	wl1251_priv.state = false;
	if (wl1251_priv.vreg) {
		vreg_disable(wl1251_priv.vreg);
	}
	if (wl1251_priv.pdata->gpio_enable >= 0) {
		gpio_direction_output(wl1251_priv.pdata->gpio_enable, 0);
	}
	if (wl1251_priv.pdata->gpio_reset >= 0) {
		gpio_set_value(wl1251_priv.pdata->gpio_reset, 1);
	}
	if (wl1251_priv.pdata->gpio_32k_osc >= 0) {
		gpio_direction_output(wl1251_priv.pdata->gpio_32k_osc, 0);
	}
	mdelay(200);
	msm_gpios_disable(wl1251_priv.gpios.off, wl1251_priv.gpios.off_length);
	
	return 0;
}

static struct mmc_platform_data msm7200a_wl1251_data = {
	.built_in = 1,
	.ocr_mask = MMC_VDD_28_29,
	.embedded_sdio = &ti_wifi_emb_data,
	.translate_vdd = wifi_switchvdd,
};

static int msm7200a_wl1251_probe(struct platform_device *pdev)
{
	int rc;
	struct vreg* vreg;
	struct msm7200a_wl1251_pdata *pdata = pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_ERR "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	if (pdata->slot_number < 1 || pdata->slot_number > 4) {
		printk(KERN_ERR "%s: invalid slot number %d\n", __func__,
			pdata->slot_number);
		return -EINVAL;
	}

	if (pdata->gpio_irq >= 0) {
		rc = gpio_request(pdata->gpio_irq, "WiFi IRQ");
		if (rc) {
			goto err_gpio_irq;
		}
#if MSM7200A_WL1251_HACK
		wl1251_pdata.irq = gpio_to_irq(pdata->gpio_irq);
		set_irq_flags(wl1251_pdata.irq, IRQF_VALID | IRQF_NOAUTOEN);
#endif
	}
	
	if (pdata->gpio_32k_osc >= 0) {
		rc = gpio_request(pdata->gpio_32k_osc, "WiFi IRQ");
		if (rc) {
			goto err_gpio_32k_osc;
		}
	}
	if (pdata->gpio_enable >= 0) {
		rc = gpio_request(pdata->gpio_enable, "WiFi Power");
		if (rc) {
			goto err_gpio_enable;
		}
	}
	if (pdata->gpio_reset >= 0) {
		rc = gpio_request(pdata->gpio_reset, "WiFi Reset");
		if (rc) {
			goto err_gpio_reset;
		}
	}

	if (pdata->vreg_id >= 0) {
		vreg = vreg_get_by_id(&pdev->dev, pdata->vreg_id);
		if (IS_ERR(vreg)) {
			rc = PTR_ERR(vreg);
			goto err_vreg;
		}
		wl1251_priv.vreg = vreg;
	}

	msm7200a_setup_gpios(pdata->slot_number, &wl1251_priv.gpios);
	rc = msm_gpios_request(wl1251_priv.gpios.on, wl1251_priv.gpios.on_length);
	if (rc) {
		goto err_sdcc_gpios;
	}
	
	wl1251_priv.pdata = pdata;

	rc = msm_add_sdcc(pdata->slot_number, &msm7200a_wl1251_data, 0, 0);
	if (rc) {
		goto err_sdcc_dev;
	}

#if MSM7200A_WL1251_HACK
	rc = platform_device_register(&wl1251_device);
	if (rc) {
		goto err_pdev;
	}
#endif
	return 0;

#if MSM7200A_WL1251_HACK
err_pdev:
	msm_delete_sdcc(pdata->slot_number);
#endif
err_sdcc_dev:
	wl1251_priv.pdata = NULL;
err_sdcc_gpios:
	if (wl1251_priv.vreg) {
		vreg_put(wl1251_priv.vreg);
		wl1251_priv.vreg = NULL;
	}
err_vreg:
	if (pdata->gpio_reset >= 0) {
		gpio_free(pdata->gpio_reset);
	}
err_gpio_reset:
	if (pdata->gpio_enable >= 0) {
		gpio_free(pdata->gpio_enable);
	}
err_gpio_enable:
	if (pdata->gpio_32k_osc >= 0) {
		gpio_free(pdata->gpio_32k_osc);
	}
err_gpio_32k_osc:
	if (pdata->gpio_irq >= 0) {
		gpio_free(pdata->gpio_irq);
	}
err_gpio_irq:
	return rc;
}

static int msm7200a_wl1251_remove(struct platform_device *pdev) {
#if MSM7200A_WL1251_HACK
	platform_device_unregister(&wl1251_device);
#endif
	msm_delete_sdcc(wl1251_priv.pdata->slot_number);
	msm_gpios_disable_free(wl1251_priv.gpios.off, wl1251_priv.gpios.off_length);
	if (wl1251_priv.vreg) {
		vreg_put(wl1251_priv.vreg);
		wl1251_priv.vreg = NULL;
	}
	if (wl1251_priv.pdata->gpio_irq >= 0) {
		gpio_free(wl1251_priv.pdata->gpio_irq);
	}
	if (wl1251_priv.pdata->gpio_enable) {
		gpio_free(wl1251_priv.pdata->gpio_enable);
	}
	if (wl1251_priv.pdata->gpio_reset) {
		gpio_free(wl1251_priv.pdata->gpio_reset);
	}
	if (wl1251_priv.pdata->gpio_32k_osc) {
		gpio_free(wl1251_priv.pdata->gpio_32k_osc);
	}
	wl1251_priv.pdata = NULL;
#if MSM7200A_WL1251_HACK
	wl1251_pdata.irq = 0;
#endif
	return 0;	
}

static struct platform_driver msm7200a_wl1251_driver = {
	.probe = msm7200a_wl1251_probe,
	.remove = msm7200a_wl1251_remove,
	.driver = {
		   .name = MSM7200A_WL1251_DRV_NAME,
		   .owner = THIS_MODULE,
	},
};

static int __init msm7200a_mmc_init(void) {
	platform_driver_register(&msm7200a_mmc_driver);
	platform_driver_register(&msm7200a_wl1251_driver);
	return 0;
}

static void __exit msm7200a_mmc_exit(void) {
	platform_driver_unregister(&msm7200a_mmc_driver);
	platform_driver_unregister(&msm7200a_wl1251_driver);
}

module_init(msm7200a_mmc_init);
module_exit(msm7200a_mmc_exit);
