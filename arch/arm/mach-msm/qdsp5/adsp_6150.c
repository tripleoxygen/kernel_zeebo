/* arch/arm/mach-msm/qdsp5/adsp_6150.h
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
	QDSP_MODULE_KERNEL=0x0,		// 0x0
	QDSP_MODULE_AFETASK=0x1,		// 0x1
	QDSP_MODULE_AUDPLAY0TASK=0x2,	// 0x2+
	QDSP_MODULE_AUDPLAY1TASK=0x3,	// 0x3
	QDSP_MODULE_AUDPPTASK=0x4,		// 0x4+
	QDSP_MODULE_VIDEOTASK=0x5,		// 0x5
	QDSP_MODULE_AUDRECTASK=0xc,		// 0xc+
	QDSP_MODULE_AUDPREPROCTASK=0xd,	// 0xd+
	QDSP_MODULE_VOCENCTASK=0x14,		// 0x12
	QDSP_MODULE_VOCDECTASK=0x15,		// 0x13
	QDSP_MODULE_VOICEPROCTASK=0x16,	// 0x14
	QDSP_MODULE_VIDEOENCTASK=0x17,	// 0x15
	QDSP_MODULE_VFETASK=0x18,		// 0x16+
	QDSP_MODULE_JPEGTASK=0x20,		// 0x1f+
	QDSP_MODULE_LPMTASK=0x21,		// 0x20+
	QDSP_MODULE_MODMATHTASK=0x22,		// 0x22+ (0x22 23 from below, tmzt)
	QDSP_MODULE_AUDPLAY2TASK=0x27,	// 0x23
	QDSP_MODULE_AUDPLAY3TASK=0x28,	// 0x24
	QDSP_MODULE_AUDPLAY4TASK=0x29,	// 0x25
	QDSP_MODULE_GRAPHICSTASK=0x2a,	// 0x26
	QDSP_MODULE_MAX=0x2e,		// 0x2e
} qdsp_module_type;

#define QDSP_RTOS_MAX_TASK_ID  19U

/* Table of modules indexed by task ID for the GAUDIO image */
static qdsp_module_type qdsp_gaudio_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0  0
	QDSP_MODULE_AFETASK,		// 0x1  1
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDPPTASK,		// 0x4 4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2 2
	QDSP_MODULE_AUDPLAY1TASK,	// 0x3 3
	QDSP_MODULE_AUDPLAY2TASK,	// 0x23 27
	QDSP_MODULE_AUDPLAY3TASK,	// 0x24 28
	QDSP_MODULE_AUDPLAY4TASK,	// 0x25 29
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDRECTASK,		// 0xd c
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe d
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_GRAPHICSTASK,	// 0x26 2a
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the GAUDIO image */
static uint32_t qdsp_gaudio_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue              */
	0x3f0,               /* QDSP_mpuAfeQueue                  */
	0x420,               /* QDSP_mpuGraphicsCmdQueue          */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuModmathCmdQueue           */ 
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecPktQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_txMpuEncQueue                */
	0x3f4,               /* QDSP_uPAudPPCmd1Queue             */
	0x3f8,               /* QDSP_uPAudPPCmd2Queue             */
	0x3fc,               /* QDSP_uPAudPPCmd3Queue             */
	0x40c,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	0x410,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	0x414,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	0x418,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	0x41c,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x400,               /* QDSP_uPAudPreProcCmdQueue         */
	0x408,               /* QDSP_uPAudRecBitStreamQueue       */
	0x404,               /* QDSP_uPAudRecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue               */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue         */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue         */
};

/* Table of modules indexed by task ID for the COMBO image */
static qdsp_module_type qdsp_combo_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0  0
	QDSP_MODULE_AFETASK,		// 0x1  1
	QDSP_MODULE_VOCDECTASK,		// 0x13 15
	QDSP_MODULE_VOCENCTASK,		// 0x12 14
	QDSP_MODULE_VIDEOTASK,		// 0x5 5
	QDSP_MODULE_VIDEOENCTASK,	// 0x15 17
	QDSP_MODULE_VOICEPROCTASK,	// 0x14 16
	QDSP_MODULE_VFETASK,		// 0x16 18
	QDSP_MODULE_JPEGTASK,		// 0x1f 20
	QDSP_MODULE_AUDPPTASK,		// 0x4 4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2 2
	QDSP_MODULE_AUDPLAY1TASK,	// 0x3 3
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_LPMTASK,		// 0x20 21
	QDSP_MODULE_AUDRECTASK,		// 0xd c
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe d
	QDSP_MODULE_MODMATHTASK,	// 0x22 23
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the COMBO image */
static uint32_t qdsp_combo_queue_offset_table[] = {
	0x6ec,               /* QDSP_lpmCommandQueue              */
	0x698,               /* QDSP_mpuAfeQueue                  */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue          */ 
	0x6ac,               /* QDSP_mpuModmathCmdQueue           */
	0x6c0,               /* QDSP_mpuVDecCmdQueue              */
	0x6c4,               /* QDSP_mpuVDecPktQueue              */
	0x6bc,               /* QDSP_mpuVEncCmdQueue              */
	0x6a0,               /* QDSP_rxMpuDecCmdQueue             */
	0x6a4,               /* QDSP_rxMpuDecPktQueue             */
	0x6a8,               /* QDSP_txMpuEncQueue                */
	0x6c8,               /* QDSP_uPAudPPCmd1Queue             */
	0x6cc,               /* QDSP_uPAudPPCmd2Queue             */
	0x6d0,               /* QDSP_uPAudPPCmd3Queue             */
	0x6e0,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x6d4,               /* QDSP_uPAudPreProcCmdQueue         */
	0x6dc,               /* QDSP_uPAudRecBitStreamQueue       */
	0x6d8,               /* QDSP_uPAudRecCmdQueue             */
	0x6e8,               /* QDSP_uPJpegActionCmdQueue         */
	0x6e4,               /* QDSP_uPJpegCfgCmdQueue            */
	0x69c,               /* QDSP_uPVocProcQueue               */
	0x6b0,               /* QDSP_vfeCommandQueue              */
	0x6b8,               /* QDSP_vfeCommandScaleQueue         */
	0x6b4                /* QDSP_vfeCommandTableQueue         */
};

/* Table of modules indexed by task ID for the QTV_LP image */
static qdsp_module_type qdsp_qtv_lp_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0x0 0
	QDSP_MODULE_AFETASK,		// 0x1 1
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_VIDEOTASK,		// 0x5 5
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDPPTASK,		// 0x4 4
	QDSP_MODULE_AUDPLAY0TASK,	// 0x2 2
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_AUDRECTASK,		// 0xd c
	QDSP_MODULE_AUDPREPROCTASK,	// 0xe d
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX,		// 0x2a
	QDSP_MODULE_MAX			// 0x2a
};

/* Queue offset table indexed by queue ID for the QTV_LP image */
static uint32_t qdsp_qtv_lp_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue              */
	0x43a,               /* QDSP_mpuAfeQueue                  */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue          */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuModmathCmdQueue           */
	0x43e,               /* QDSP_mpuVDecCmdQueue              */
	0x442,               /* QDSP_mpuVDecPktQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_txMpuEncQueue                */
	0x44a,               /* QDSP_uPAudPPCmd1Queue             */
	0x44e,               /* QDSP_uPAudPPCmd2Queue             */
	0x452,               /* QDSP_uPAudPPCmd3Queue             */
	0x45e,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x446,               /* QDSP_uPAudPreProcCmdQueue         */
	0x45a,               /* QDSP_uPAudRecBitStreamQueue       */
	0x456,               /* QDSP_uPAudRecCmdQueue             */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue               */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue              */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue         */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue         */
};

/* Table of modules indexed by task ID for the image4 */
static qdsp_module_type qdsp_image4_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,		// 0
	QDSP_MODULE_AFETASK,		// 1
	QDSP_MODULE_VOCDECTASK,		// 15
	QDSP_MODULE_VOCENCTASK,		// 14
	QDSP_MODULE_MAX,		// 2e
	QDSP_MODULE_MAX,		// 2e
	QDSP_MODULE_VOICEPROCTASK,	// 16
	QDSP_MODULE_MAX,		// 2e
	QDSP_MODULE_MAX,		// 2e
	QDSP_MODULE_AUDPPTASK,		// 4
	QDSP_MODULE_MAX,	// 
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,	//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX,		//
	QDSP_MODULE_MAX			//
};

/* Queue offset table indexed by queue ID for the image 4*/
static uint32_t qdsp_image4_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,
	0x55a,             
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	0x562,             
	0x566,             
	0x56a,             
	0x56e,             
	0x572,             
	0x576,             
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	0x55e, 
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
};

/* Tables to convert tasks to modules */
static uint32_t *qdsp_task_to_module[] = {
	qdsp_combo_task_to_module_table,
	qdsp_combo_task_to_module_table,
	qdsp_gaudio_task_to_module_table,
	qdsp_qtv_lp_task_to_module_table,
	qdsp_image4_task_to_module_table,
};

/* Tables to retrieve queue offsets */
static uint32_t *qdsp_queue_offset_table[] = {
	qdsp_combo_queue_offset_table,
	qdsp_combo_queue_offset_table,
	qdsp_gaudio_queue_offset_table,
	qdsp_qtv_lp_queue_offset_table,
	qdsp_image4_queue_offset_table,
};

#define QDSP_MODULE(n) \
	{ .name = #n, .pdev_name = "adsp_" #n, .id = QDSP_MODULE_##n }

static struct adsp_module_info module_info[] = {
	QDSP_MODULE(AUDPPTASK),
	QDSP_MODULE(AUDRECTASK),
	QDSP_MODULE(AUDPREPROCTASK),
	QDSP_MODULE(VFETASK),
/*	QDSP_MODULE(QCAMTASK),		*/
	QDSP_MODULE(LPMTASK),
	QDSP_MODULE(JPEGTASK),
	QDSP_MODULE(VIDEOTASK),
/*	QDSP_MODULE(VDEC_LP_MODE),	*/
};

int adsp_init_info_6150(struct adsp_info *info)
{
	info->send_irq =   0x00c00200;
	info->read_ctrl =  0x00400038;
	info->write_ctrl = 0x00400034;

	info->max_msg16_size = 193;
	info->max_msg32_size = 8;

	info->max_task_id = QDSP_RTOS_MAX_TASK_ID;
	info->max_module_id = QDSP_MODULE_MAX - 1;
	info->max_queue_id = QDSP_QUEUE_MAX;
	info->max_image_id = 4;
	info->queue_offset = qdsp_queue_offset_table;
	info->task_to_module = qdsp_task_to_module;

	info->module_count = ARRAY_SIZE(module_info);
	info->module = module_info;
	return 0;
}
