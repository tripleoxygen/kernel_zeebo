/* arch/arm/mach-msm/qdsp5/adsp_5225.h
 *
 * Copyright (c) 2008 QUALCOMM Incorporated.
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

#include "adsp.h"

/* Firmware modules */
typedef enum { 
	QDSP_MODULE_KERNEL,		// 0x0
	QDSP_MODULE_AFETASK,		// 0x1
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2+
	QDSP_MODULE_AUDPLAY1TASK,	// 0x3
	QDSP_MODULE_AUDPPTASK,		// 0x4+
	QDSP_MODULE_VIDEOTASK,		// 0x5
	QDSP_MODULE_VIDEO_AAC_VOC,	// 0x6
	QDSP_MODULE_PCM_DEC,		// 0x7
	QDSP_MODULE_AUDIO_DEC_MP3,	// 0x8
	QDSP_MODULE_AUDIO_DEC_AAC,	// 0x9
	QDSP_MODULE_AUDIO_DEC_WMA,	// 0xa
	QDSP_MODULE_HOSTPCM,		// 0xb
	QDSP_MODULE_DTMF,		// 0xc
	QDSP_MODULE_AUDRECTASK,		// 0xd+
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe+
	QDSP_MODULE_SBC_ENC,		// 0xf
	QDSP_MODULE_VOC,		// 0x10
	QDSP_MODULE_VOC_PCM,		// 0x11
	QDSP_MODULE_VOCENCTASK,		// 0x12
	QDSP_MODULE_VOCDECTASK,		// 0x13
	QDSP_MODULE_VOICEPROCTASK,	// 0x14
	QDSP_MODULE_VIDEOENCTASK,	// 0x15
	QDSP_MODULE_VFETASK,		// 0x16+
	QDSP_MODULE_WAV_ENC,		// 0x17	
	QDSP_MODULE_AACLC_ENC,		// 0x18
	QDSP_MODULE_VIDEO_AMR,		// 0x19
	QDSP_MODULE_VOC_AMR,		// 0x1a
	QDSP_MODULE_VOC_EVRC,		// 0x1b
	QDSP_MODULE_VOC_13K,		// 0x1c
	QDSP_MODULE_VOC_FGV,		// 0x1d
	QDSP_MODULE_DIAGTASK,		// 0x1e
	QDSP_MODULE_JPEGTASK,		// 0x1f+
	QDSP_MODULE_LPMTASK,		// 0x20+
	QDSP_MODULE_QCAMTASK,		// 0x21+
	QDSP_MODULE_MODMATHTASK,	// 0x22
	QDSP_MODULE_AUDPLAY2TASK,	// 0x23
	QDSP_MODULE_AUDPLAY3TASK,	// 0x24
	QDSP_MODULE_AUDPLAY4TASK,	// 0x25
	QDSP_MODULE_GRAPHICSTASK,	// 0x26
	QDSP_MODULE_MIDI,		// 0x27
	QDSP_MODULE_GAUDIO,		// 0x28
	QDSP_MODULE_VDEC_LP_MODE,	// 0x29
	QDSP_MODULE_MAX,		// 0x2a
} qdsp_module_type;

#define QDSP_RTOS_MAX_TASK_ID  19U

/* Table of modules indexed by task ID for the GAUDIO image */
static qdsp_module_type qdsp_gaudio_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0
	QDSP_MODULE_AFETASK,		// 0x1
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDPPTASK,		// 0x4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2
	QDSP_MODULE_AUDPLAY1TASK,	// 0x3
	QDSP_MODULE_AUDPLAY2TASK,	// 0x23
	QDSP_MODULE_AUDPLAY3TASK,	// 0x24
	QDSP_MODULE_AUDPLAY4TASK,	// 0x25
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDRECTASK,		// 0xd
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_GRAPHICSTASK,	// 0x26
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the GAUDIO image */
static uint32_t qdsp_gaudio_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue              */
	0x3c8,               /* QDSP_mpuAfeQueue                  */
	0x3f8,               /* QDSP_mpuGraphicsCmdQueue          */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuModmathCmdQueue           */ 
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecPktQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_txMpuEncQueue                */
	0x3cc,               /* QDSP_uPAudPPCmd1Queue             */
	0x3d0,               /* QDSP_uPAudPPCmd2Queue             */
	0x3d4,               /* QDSP_uPAudPPCmd3Queue             */
	0x3e4,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	0x3e8,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	0x3ec,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	0x3f0,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	0x3f4,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x3d8,               /* QDSP_uPAudPreProcCmdQueue         */
	0x3e0,               /* QDSP_uPAudRecBitStreamQueue       */
	0x3dc,               /* QDSP_uPAudRecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue               */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue         */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue         */
};

/* Table of modules indexed by task ID for the COMBO image */
static qdsp_module_type qdsp_combo_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0
	QDSP_MODULE_AFETASK,		// 0x1
	QDSP_MODULE_VOCDECTASK,		// 0x13
	QDSP_MODULE_VOCENCTASK,		// 0x12
	QDSP_MODULE_VIDEOTASK,		// 0x5
	QDSP_MODULE_VIDEOENCTASK,	// 0x15
	QDSP_MODULE_VOICEPROCTASK,	// 0x14
	QDSP_MODULE_VFETASK,		// 0x16
	QDSP_MODULE_JPEGTASK,		// 0x1f
	QDSP_MODULE_AUDPPTASK,		// 0x4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2
	QDSP_MODULE_AUDPLAY1TASK,	// 0x3
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_LPMTASK,		// 0x20
	QDSP_MODULE_AUDRECTASK,		// 0xd
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe
	QDSP_MODULE_MODMATHTASK,	// 0x22
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the COMBO image */
static uint32_t qdsp_combo_queue_offset_table[] = {
	0x6a4,               /* QDSP_lpmCommandQueue              */
	0x650,               /* QDSP_mpuAfeQueue                  */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue          */ 
	0x664,               /* QDSP_mpuModmathCmdQueue           */
	0x678,               /* QDSP_mpuVDecCmdQueue              */
	0x67c,               /* QDSP_mpuVDecPktQueue              */
	0x674,               /* QDSP_mpuVEncCmdQueue              */
	0x658,               /* QDSP_rxMpuDecCmdQueue             */
	0x65c,               /* QDSP_rxMpuDecPktQueue             */
	0x660,               /* QDSP_txMpuEncQueue                */
	0x680,               /* QDSP_uPAudPPCmd1Queue             */
	0x684,               /* QDSP_uPAudPPCmd2Queue             */
	0x688,               /* QDSP_uPAudPPCmd3Queue             */
	0x698,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x68c,               /* QDSP_uPAudPreProcCmdQueue         */
	0x694,               /* QDSP_uPAudRecBitStreamQueue       */
	0x690,               /* QDSP_uPAudRecCmdQueue             */
	0x6a0,               /* QDSP_uPJpegActionCmdQueue         */
	0x69c,               /* QDSP_uPJpegCfgCmdQueue            */
	0x654,               /* QDSP_uPVocProcQueue               */
	0x668,               /* QDSP_vfeCommandQueue              */
	0x670,               /* QDSP_vfeCommandScaleQueue         */
	0x66c                /* QDSP_vfeCommandTableQueue         */
};

/* Table of modules indexed by task ID for the QTV_LP image */
static qdsp_module_type qdsp_qtv_lp_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0
	QDSP_MODULE_AFETASK,		// 0x1
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_VIDEOTASK,		// 0x5
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDPPTASK,		// 0x4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDRECTASK,		// 0xd
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the QTV_LP image */
static uint32_t qdsp_qtv_lp_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue              */
	0x408,               /* QDSP_mpuAfeQueue                  */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue          */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuModmathCmdQueue           */
	0x40c,               /* QDSP_mpuVDecCmdQueue              */
	0x410,               /* QDSP_mpuVDecPktQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_txMpuEncQueue                */
	0x418,               /* QDSP_uPAudPPCmd1Queue             */
	0x41c,               /* QDSP_uPAudPPCmd2Queue             */
	0x420,               /* QDSP_uPAudPPCmd3Queue             */
	0x42c,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x414,               /* QDSP_uPAudPreProcCmdQueue         */
	0x428,               /* QDSP_uPAudRecBitStreamQueue       */
	0x42c,               /* QDSP_uPAudRecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue               */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue         */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue         */
};

/* Tables to convert tasks to modules */
static uint32_t *qdsp_task_to_module[] = {
	qdsp_combo_task_to_module_table,
	qdsp_gaudio_task_to_module_table,
	qdsp_qtv_lp_task_to_module_table,
};

/* Tables to retrieve queue offsets */
static uint32_t *qdsp_queue_offset_table[] = {
	qdsp_combo_queue_offset_table,
	qdsp_gaudio_queue_offset_table,
	qdsp_qtv_lp_queue_offset_table,
};

#define QDSP_MODULE(n, clkname, clkrate, verify_cmd_func, patch_event_func) \
	{ .name = #n, .pdev_name = "adsp_" #n, .id = QDSP_MODULE_##n, \
	  .clk_name = clkname, .clk_rate = clkrate, \
	  .verify_cmd = verify_cmd_func, .patch_event = patch_event_func }

static struct adsp_module_info module_info[] = {
	QDSP_MODULE(AUDPLAY0TASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDPPTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDRECTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(AUDPREPROCTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(VFETASK, "vfe_clk", 0, adsp_vfe_verify_cmd,
		adsp_vfe_patch_event),
	QDSP_MODULE(QCAMTASK, NULL, 0, NULL, NULL),
	QDSP_MODULE(LPMTASK, NULL, 0, adsp_lpm_verify_cmd, NULL),
	QDSP_MODULE(JPEGTASK, "vdc_clk", 0, adsp_jpeg_verify_cmd,
    	adsp_jpeg_patch_event),
	QDSP_MODULE(VIDEOTASK, "vdc_clk", 96000000,
		adsp_video_verify_cmd, NULL),
	QDSP_MODULE(VDEC_LP_MODE, NULL, 0, NULL, NULL),
	QDSP_MODULE(VIDEOENCTASK, "vdc_clk", 96000000,
		adsp_videoenc_verify_cmd, NULL),
};

int adsp_init_info_5225(struct adsp_info *info)
{
	info->send_irq =   0x00c00200;
	info->read_ctrl =  0x00400038;
	info->write_ctrl = 0x00400034;

	info->max_msg16_size = 193;
	info->max_msg32_size = 8;

	info->max_task_id = QDSP_RTOS_MAX_TASK_ID;
	info->max_module_id = QDSP_MODULE_MAX - 1;
	info->max_queue_id = QDSP_QUEUE_MAX;
	info->max_image_id = 2;
	info->queue_offset = qdsp_queue_offset_table;
	info->task_to_module = qdsp_task_to_module;

	info->module_count = ARRAY_SIZE(module_info);
	info->module = module_info;
	return 0;
}
