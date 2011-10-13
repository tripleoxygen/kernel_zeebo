/* include/asm/mach-msm/msm7200a_rfkill.h
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
#ifndef __ARCH_ARM_MACH_MSM_MSM720XA_RFKILL_H__
#define __ARCH_ARM_MACH_MSM_MSM720XA_RFKILL_H__

#define MSM7200A_UART1DM_RTS	43
#define MSM7200A_UART1DM_CTS	44
#define MSM7200A_UART1DM_RX		45
#define MSM7200A_UART1DM_TX		46

#define MSM7200A_UART2DM_RTS	19
#define MSM7200A_UART2DM_CTS	20
#define MSM7200A_UART2DM_RX		21
#define MSM7200A_UART2DM_TX		108

#define MSM7200A_PCM_DOUT		68
#define MSM7200A_PCM_DIN		69
#define MSM7200A_PCM_SYNC		70
#define MSM7200A_PCM_CLK		71

struct msm7200a_rfkill_pdata {
	unsigned uart_number;
	const char* rfkill_name;
	int (*set_power)(void* data, bool blocked);
	int (*init)(struct platform_device *pdev);
	void (*exit)(struct platform_device *pdev);
	bool configure_bt_pcm;
};

#endif //__ARCH_ARM_MACH_MSM_MSM720XA_RFKILL_H__

