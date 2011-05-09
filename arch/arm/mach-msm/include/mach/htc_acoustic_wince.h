/* include/asm/mach-msm/htc_acoustic_wince.h
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
#ifndef __ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_WINCE_H__
#define __ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_WINCE_H__

#include <linux/platform_device.h>

#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_ARM11_DONE	                    _IOW(ACOUSTIC_IOCTL_MAGIC, 22, unsigned int)

#define ACOUSTIC_UPDATE_ADIE_TABLE              _IOW(ACOUSTIC_IOCTL_MAGIC,  1, struct adie_table* )
#define ACOUSTIC_UPDATE_VOLUME_TABLE            _IOW(ACOUSTIC_IOCTL_MAGIC,  2, uint16_t* )
#define ACOUSTIC_UPDATE_CE_TABLE                _IOW(ACOUSTIC_IOCTL_MAGIC,  3, uint16_t* )
#define ACOUSTIC_UPDATE_AUDIO_PATH_TABLE        _IOW(ACOUSTIC_IOCTL_MAGIC,  4, uint16_t* )
#define ACOUSTIC_UPDATE_AUDIO_SETTINGS          _IOW(ACOUSTIC_IOCTL_MAGIC,  5, struct audio_update_req* )
#define ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE _IOW(ACOUSTIC_IOCTL_MAGIC,  6, struct htc_voc_cal_table* )
#define ACOUSTIC_GET_CAPABILITIES               _IOW(ACOUSTIC_IOCTL_MAGIC,  8, struct msm_acoustic_capabilities* )
#define ACOUSTIC_SET_HW_AUDIO_PATH              _IOW(ACOUSTIC_IOCTL_MAGIC,  10, struct msm_audio_path* )


struct htc_acoustic_wce_amss_data {
	void *volume_table;
	void *ce_table;
	void *adie_table;
	void *codec_table;
	void *mic_offset;
	int voc_cal_field_size;
	void (*mic_bias_callback)(bool enable);
};

struct htc_acoustic_wce_board_data {
	void (*set_speaker_amp)(bool enabled);
	void (*set_headset_amp)(bool enabled);
	bool dual_mic_supported;
	bool use_tpa_2016;
};

extern struct htc_acoustic_wce_board_data *htc_acoustic_wce_board_data;

#endif //__ARCH_ARM_MACH_MSM_HTC_ACOUSTIC_WINCE_H__

