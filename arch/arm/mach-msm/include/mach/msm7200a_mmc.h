/* include/asm/mach-msm/msm7200a_mmc.h
 *
 * Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
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
#ifndef __ARCH_ARM_MACH_MSM_MSM7200A_MMC_H__
#define __ARCH_ARM_MACH_MSM_MSM7200A_MMC_H__

#define MSM7200A_MMC_DRV_NAME	"msm7200a-mmc"
#define MSM7200A_WL1251_DRV_NAME	"msm7200a-wl1251"

struct msm7200a_mmc_pdata {
	unsigned slot_number;
	int vreg_id;
	int gpio_detection;
};

struct msm7200a_wl1251_pdata {
	unsigned slot_number;
	int vreg_id;
	int gpio_irq;
	int gpio_32k_osc;
	int gpio_enable;
	int gpio_reset;
};

#endif //__ARCH_ARM_MACH_MSM_MSM7200A_MMC_H__

