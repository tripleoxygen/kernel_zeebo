/* arch/arm/mach-msm/qdsp5/adsp_6125.c
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
#include <mach/amss/amss_6125.h>
#include <mach/irqs.h>

/* Firmware modules */
typedef enum {
	QDSP_MODULE_KERNEL=0x0,
	QDSP_MODULE_AFETASK=0x1,
	QDSP_MODULE_AUDPLAY0TASK=0x2,
	QDSP_MODULE_AUDPPTASK=0x3,
	QDSP_MODULE_VIDEOTASK=0x4,
	QDSP_MODULE_AUDRECTASK=0x1e,
	QDSP_MODULE_AUDPREPROCTASK=0x1f,
	QDSP_MODULE_VOCENCTASK=0x9,
	QDSP_MODULE_VOCDECTASK=0xa,
	QDSP_MODULE_VOICEPROCTASK=0xb,
	QDSP_MODULE_LPMTASK=0x2c,
	QDSP_MODULE_QCAMTASK=0x2d,
	QDSP_MODULE_VIDEOENCTASK=0x26,
	QDSP_MODULE_VFETASK=0x27,
	QDSP_MODULE_JPEGTASK=0x2b,
	QDSP_MODULE_MAX=0x33,
} qdsp_module_type;

#define QDSP_RTOS_MAX_TASK_ID  19U

/* Table of modules indexed by task ID for the image0 */
static qdsp_module_type qdsp_image0_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_VOCDECTASK,
	QDSP_MODULE_VOCENCTASK,
	QDSP_MODULE_VIDEOTASK,
	QDSP_MODULE_VIDEOENCTASK,
	QDSP_MODULE_VOICEPROCTASK,
	QDSP_MODULE_VFETASK,
	QDSP_MODULE_JPEGTASK,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_AUDPLAY0TASK,
	0x14,
	0x15,
	0x16,
	0x17,
	0x2c,
	0x1e,
	0x1f,
	0x2e,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	0x12,
	QDSP_MODULE_MAX
};

/* Queue offset table indexed by queue ID for the image0 */
static uint32_t qdsp_image0_queue_offset_table[] = {
	0x65a,               /* QDSP_lpmCommandQueue        */
	0x5f2,               /* QDSP_mpuAfeQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue    */
	0x606,               /* QDSP_mpuModmathCmdQueue     */
	0x61e,               /* QDSP_mpuVDecCmdQueue        */
	0x622,               /* QDSP_mpuVDecPktQueue        */
	0x61a,               /* QDSP_mpuVEncCmdQueue        */
	0x5fa,               /* QDSP_rxMpuDecCmdQueue       */
	0x5fe,               /* QDSP_rxMpuDecPktQueue       */
	0x602,               /* QDSP_txMpuEncQueue          */
	0x626,               /* QDSP_uPAudPPCmd1Queue       */
	0x62a,               /* QDSP_uPAudPPCmd2Queue       */
	0x62e,               /* QDSP_uPAudPPCmd3Queue       */
	0x63e,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	0x642,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	0x646,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	0x64a,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	0x64e,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x632,               /* QDSP_uPAudPreProcCmdQueue   */
	0x63a,               /* QDSP_uPAudRecBitStreamQueue */
	0x636,               /* QDSP_uPAudRecCmdQueue */
	0x656,               /* QDSP_uPJpegActionCmdQueue */
	0x652,               /* QDSP_uPJpegCfgCmdQueue */
	0x5f6,               /* QDSP_uPVocProcQueue */
	0x60e,               /* QDSP_vfeCommandQueue         */
	0x616,               /* QDSP_vfeCommandScaleQueue        */
	0x612,               /* QDSP_vfeCommandTableQueue   */
	0x60a                /* QDSP_uPDiagQueue   */
};

/* Table of modules indexed by task ID for the COMBO image */
static qdsp_module_type qdsp_combo_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_VOCDECTASK,
	QDSP_MODULE_VOCENCTASK,
	QDSP_MODULE_VIDEOTASK,
	QDSP_MODULE_VIDEOENCTASK,
	QDSP_MODULE_VOICEPROCTASK,
	QDSP_MODULE_VFETASK,
	QDSP_MODULE_JPEGTASK,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_AUDPLAY0TASK,
	0x14,
	0x15,
	0x16,
	0x17,
	0x2c,
	0x1e,
	0x1f,
	0x2e,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	0x12,
	QDSP_MODULE_MAX
};

/* Queue offset table indexed by queue ID for the COMBO image */
static uint32_t qdsp_combo_queue_offset_table[] = {
	0x65a,               /* QDSP_lpmCommandQueue        */
	0x5f2,               /* QDSP_mpuAfeQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue    */ //Not used on Rhod
	0x606,               /* QDSP_mpuModmathCmdQueue     */
	0x61e,               /* QDSP_mpuVDecCmdQueue        */
	0x622,               /* QDSP_mpuVDecPktQueue        */
	0x61a,               /* QDSP_mpuVEncCmdQueue        */
	0x5fa,               /* QDSP_rxMpuDecCmdQueue       */
	0x5fe,               /* QDSP_rxMpuDecPktQueue       */
	0x602,               /* QDSP_txMpuEncQueue          */
	0x626,               /* QDSP_uPAudPPCmd1Queue       */
	0x62a,               /* QDSP_uPAudPPCmd2Queue       */
	0x62e,               /* QDSP_uPAudPPCmd3Queue       */
	0x63e,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	0x642,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	0x646,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	0x64a,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	0x64e,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	0x632,               /* QDSP_uPAudPreProcCmdQueue   */
	0x63a,               /* QDSP_uPAudRecBitStreamQueue */
	0x636,               /* QDSP_uPAudRecCmdQueue   */
	0x656,               /* QDSP_uPJpegActionCmdQueue */
	0x652,               /* QDSP_uPJpegCfgCmdQueue      */
	0x5f6,               /* QDSP_uPVocProcQueue       */
	0x60e,               /* QDSP_vfeCommandQueue         */
	0x616,               /* QDSP_vfeCommandScaleQueue        */
	0x612,               /* QDSP_vfeCommandTableQueue   */
	0x60a                /* QDSP_uPDiagQueue   */
};

/* Table of modules indexed by task ID for the GAUDIO image */
static qdsp_module_type qdsp_gaudio_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_VIDEOTASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_AUDPLAY0TASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX
};

/* Queue offset table indexed by queue ID for the GAUDIO image */
static uint32_t qdsp_gaudio_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue        */
	0x391,               /* QDSP_mpuAfeQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue    */
	0x395,  /* QDSP_mpuModmathCmdQueue     */
	0x399,  /* QDSP_mpuVDecCmdQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecPktQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue       */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue       */
	0x39d,  /* QDSP_txMpuEncQueue          */
	0x3a1,               /* QDSP_uPAudPPCmd1Queue       */
	0x3a5,               /* QDSP_uPAudPPCmd2Queue       */
	0x3a9,               /* QDSP_uPAudPPCmd3Queue       */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudPreProcCmdQueue   */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudRecBitStreamQueue */
	QDSP_RTOS_NO_QUEUE,               /* QDSP_uPAudRecCmdQueue       */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue   */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue      */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue   */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue   */
};

/* Table of modules indexed by task ID for the QTV_LP image */
static qdsp_module_type qdsp_qtv_lp_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_VIDEOTASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_AUDPLAY0TASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
};

/* Queue offset table indexed by queue ID for the QTV_LP image */
static uint32_t qdsp_qtv_lp_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,  /* QDSP_lpmCommandQueue        */
	0x3ce,               /* QDSP_mpuAfeQueue            */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuGraphicsCmdQueue    */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuModmathCmdQueue     */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecCmdQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVDecPktQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_mpuVEncCmdQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecCmdQueue       */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_rxMpuDecPktQueue       */
	0x3d2,               /* QDSP_txMpuEncQueue          */
	0x3d6,               /* QDSP_uPAudPPCmd1Queue       */
	0x3da,               /* QDSP_uPAudPPCmd2Queue       */
	0x3de,               /* QDSP_uPAudPPCmd3Queue       */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay0BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay1BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay2BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay3BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPlay4BitStreamCtrlQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudPreProcCmdQueue   */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudRecBitStreamQueue */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPAudRecCmdQueue       */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegActionCmdQueue   */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPJpegCfgCmdQueue      */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_uPVocProcQueue         */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandQueue        */
	QDSP_RTOS_NO_QUEUE,  /* QDSP_vfeCommandScaleQueue   */
	QDSP_RTOS_NO_QUEUE   /* QDSP_vfeCommandTableQueue   */
};

/* Table of modules indexed by task ID for the image4 */
static qdsp_module_type qdsp_image4_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_AUDPLAY0TASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
};

/* Queue offset table indexed by queue ID for the image 4*/
static uint32_t qdsp_image4_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,
	0x3ce,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	0x3d2,
	0x3d6,
	0x3da,
	0x3de,
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
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
};

/* Table of modules indexed by task ID for the image5 */
static qdsp_module_type qdsp_image5_task_to_module_table[] = {
	QDSP_MODULE_KERNEL,
	QDSP_MODULE_AFETASK,
	QDSP_MODULE_VOCDECTASK,
	QDSP_MODULE_VOCENCTASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_VOICEPROCTASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_AUDPPTASK,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
	QDSP_MODULE_MAX,
};

/* Queue offset table indexed by queue ID for the image 5*/
static uint32_t qdsp_image5_queue_offset_table[] = {
	QDSP_RTOS_NO_QUEUE,
	0x57a,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	0x582,
	0x586,
	0x58a,
	0x58e,
	0x592,
	0x596,
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
	QDSP_RTOS_NO_QUEUE,
	0x57e,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
	QDSP_RTOS_NO_QUEUE,
};

/* Tables to convert tasks to modules */
static uint32_t *qdsp_task_to_module[] = {
	qdsp_image0_task_to_module_table,
	qdsp_combo_task_to_module_table,
	qdsp_gaudio_task_to_module_table,
	qdsp_qtv_lp_task_to_module_table,
	qdsp_image4_task_to_module_table,
	qdsp_image5_task_to_module_table,
};

/* Tables to retrieve queue offsets */
static uint32_t *qdsp_queue_offset_table[] = {
	qdsp_image0_queue_offset_table,
	qdsp_combo_queue_offset_table,
	qdsp_gaudio_queue_offset_table,
	qdsp_qtv_lp_queue_offset_table,
	qdsp_image4_queue_offset_table,
	qdsp_image5_queue_offset_table,
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
        QDSP_MODULE(VFETASK, "vfe_clk", 0, adsp_vfe_verify_cmd, adsp_vfe_patch_event),
        QDSP_MODULE(QCAMTASK, NULL, 0, NULL, NULL),
        QDSP_MODULE(LPMTASK, NULL, 0, adsp_lpm_verify_cmd, NULL),
        QDSP_MODULE(JPEGTASK, "vdc_clk", 0, adsp_jpeg_verify_cmd, adsp_jpeg_patch_event),
        QDSP_MODULE(VIDEOTASK, "vdc_clk", 96000000, adsp_video_verify_cmd, NULL),
        QDSP_MODULE(VIDEOENCTASK, "vdc_clk", 96000000, adsp_videoenc_verify_cmd, NULL),
};

static struct adsp_info info = {
	.send_irq =   0x00c00200,
	.read_ctrl =  0x00400038,
	.write_ctrl = 0x00400034,

	.max_msg16_size = 193,
	.max_msg32_size = 8,

	.max_task_id = QDSP_RTOS_MAX_TASK_ID,
	.max_module_id = QDSP_MODULE_MAX - 1,
	.max_queue_id = QDSP_QUEUE_MAX,
	.max_image_id = 6,
	.queue_offset = qdsp_queue_offset_table,
	.task_to_module = qdsp_task_to_module,

	.module_count = ARRAY_SIZE(module_info),
	.module = module_info,

	.mtoa_vers = ADSP_RTOS_MTOA_VERS_6125,
	.atom_vers = ADSP_RTOS_ATOM_VERS_6125,
	.atom_proc = ADSP_RTOS_ATOM_PROC_6125,
	.mtoa_proc = ADSP_RTOS_MTOA_PROC_6125,
	.atom_null_proc = ADSP_RTOS_ATOM_NULL_PROC_6125,
	.mtoa_null_proc = ADSP_RTOS_MTOA_NULL_PROC_6125,
	.mtoa_prog = ADSP_RTOS_MTOA_PROG_6125,
	.atom_prog = ADSP_RTOS_ATOM_PROG_6125,
	.snd_prog = ADSP_RTOS_SND_PROG_6125,
	.snd_vers = ADSP_RTOS_SND_VERS_6125,
	.snd_device_proc = ADSP_RTOS_SND_DEV_PROC_6125,
	.snd_volume_proc = ADSP_RTOS_SND_VOL_PROC_6125,

	.irq_adsp = INT_ADSP_A11,
};

static int adsp_probe_6125(struct platform_device *pdev)
{
  	int rc;
	rc = msm_adsp_probe(&info);
	return rc;
}

static struct platform_driver msm_adsp_driver = {
	.probe = adsp_probe_6125,
	.driver = {
		.name = "msm_adsp_6125",
		.owner = THIS_MODULE,
	},
};

static int __init adsp_6125_init(void)
{
	return platform_driver_register(&msm_adsp_driver);
}

device_initcall(adsp_6125_init);
