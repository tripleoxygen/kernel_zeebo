/* arch/arm/mach-msm/proc_comm.h
 *
 * Copyright (c) 2007 QUALCOMM Incorporated
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

#ifndef _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_CE_H_
#define _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_CE_H_

// PCOM_* research taken from http://wiki.xda-developers.com/index.php?pagename=RaphaelDEX
// Research by cr2

#define DEX_HAS_DATA    0x100
#define DEX_STATUS_FAIL 0x200

enum {
	PCOM_PMIC_WLAN_ON = 0x2,
	PCOM_PMIC_WLAN_OFF = 0x3,
	PCOM_PMIC_VIBRA_ON = 0x4,
	PCOM_PMIC_VIBRA_OFF = 0x5,
	PCOM_PMIC_IR_ON = 0x6,
	PCOM_PMIC_IR_OFF = 0x7,
	PCOM_PMIC_CAM_ON = 0x8,
	PCOM_PMIC_CAM_OFF = 0x9,
	PCOM_PMIC_VGACAM_ON = 0xa,
	PCOM_PMIC_VGACAM_OFF = 0xb,
	PCOM_PMIC_SD_ON = 0xc,
	PCOM_PMIC_SD_OFF = 0xd,
	PCOM_PMIC_LCD_ON = 0xe,
	PCOM_PMIC_LCD_OFF = 0xf,
	PCOM_PMIC_MDDI_ON = 0x10,
	PCOM_PMIC_MDDI_OFF = 0x11,
	PCOM_PMIC_BT_ON = 0x12,
	PCOM_PMIC_BT_OFF = 0x13,
	PCOM_POWER_OFF = 0x14,
	PCOM_PMIC_REG_ON = 0x15,
	PCOM_PMIC_REG_OFF = 0x16,
	PCOM_VIBRA_ON = 0x17,
	PCOM_VIBRA_OFF = 0x18,
	PCOM_SET_AUDIO_PATH = 0x19,
	PCOM_PMIC_REG_VOLTAGE = 0x1a,
	PCOM_SETUSB_DPLUS = 0x1b,
	PCOM_UPDATE_AUDIO = 0x1c,

	//???
	PCOM_SET_L2_LOCK_BUS_CLK = 0x1d,
	PCOM_ARM9_LOW_SPEED = 0x1d,
	PCOM_REGISTER_VOCODER_PCM = 0x1e,
	PCOM_UNREGISTER_VOCODER_PCM = 0x1f,
	PCOM_SET_CLOCK_ON = 0x20,
	PCOM_SET_CLOCK_OFF = 0x21,
	PCOM_RESET_ARM9 = 0x22,

	PCOM_PMIC_TVOUT_AUTO_ON = 0x25,
	PCOM_PMIC_TVOUT_AUTO_FF = 0x26,

	PCOM_LCD_STATUS = 0x30,
	PCOM_CONFIG_MPP_PINS = 0x31,
	PCOM_SET_CHARGER_STATUS = 0x32,
	PCOM_TASK_REGISTER = 0x33,
	PCOM_TASK_UNREGISTER = 0x34,

	PCOM_READ_RTC = 0x81,
	PCOM_WRITE_RTC = 0x82,
	PCOM_SET_ALARM_RTC = 0x84,

	PCOM_GET_BATTERY_DATA = 0x8a,
	PCOM_GET_BATTERY_ID = 0x8b,

	PCOM_NOTIFY_ARM9_REBOOT = 0x8e,

	PCOM_GET_TX_POWER = 0x90,
	PCOM_GET_NETWORK_BAND = 0x91,
	PCOM_GET_GSM_TX_BAND = 0x92,

	PCOM_GET_SLEEP_CLOCK = 0xa2,

	PCOM_FOTA_READ = 0xa4,
	PCOM_FOTA_WRITE = 0xa5,
};

/* Constants for PCOM_GPIO_CFG */

// ??
#define GPIO_ENABLE     0
#define GPIO_DISABLE    1

// .dir (1b)
#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

// .pull (2b)
#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_KEEPER     2
#define GPIO_PULL_UP    3

// .drvstr (4b)
#define GPIO_2MA        0
#define GPIO_4MA        1
#define GPIO_6MA        2
#define GPIO_8MA        3
#define GPIO_10MA       4
#define GPIO_12MA       5
#define GPIO_14MA       6
#define GPIO_16MA       7

struct msm_dex_command {
	char cmd;
	char has_data;
	unsigned data;
};

int msm_proc_comm_wince(struct msm_dex_command *in, unsigned *out);
int msm_proc_comm_wince_init(void);

#endif
