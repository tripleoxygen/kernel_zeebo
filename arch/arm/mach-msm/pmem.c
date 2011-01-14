/* arch/arm/mach-msm/board-trout.c
 *
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <mach/board_htc.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>

#define MODULE_NAME "pmem"
#define MSM_SMI_BASE	0x00000000
#define MSM_SMI2_BASE	0x02000000
#define MSM_EBI_BASE	0x10000000
#define MSM_EBIN_BASE	0x20000000


static struct msm_pmem_setting pmem_setting = {};

struct resource resources_msm_fb[] = {
	{
	},
};

#define CALC_PMEM(name, prev, size) \
	pmem_setting.name## _start = pmem_setting.prev## _start+\
		pmem_setting.prev## _size;\
	pmem_setting.name## _size = size;


static int __init msm_pmem_init(void) {
	long pmem_shift = 0;
	switch(__machine_arch_type) {
		case MACH_TYPE_HTCDIAMOND:
		case MACH_TYPE_HTCRAPHAEL_CDMA500:
			//SMI 64 + EBI 128
			pmem_setting.pmem_start=MSM_SMI2_BASE;
			pmem_setting.pmem_size=12*1024*1024;
			CALC_PMEM(pmem_adsp, pmem, 0);//adsp is useless currently.
			CALC_PMEM(fb, pmem_adsp, 2*1025*1023);//640*480*2 (2byte/pixel)(Or 3?) *2 (front+back buffer)=1.2MiB
			CALC_PMEM(pmem_camera, fb, 0);//To be calculated more precisely

			//GPU1 must be in EBI bank 1
			pmem_setting.pmem_gpu1_start=MSM_EBI_BASE+107*1024*1024;
			pmem_setting.pmem_gpu1_size=0x800000;

			//Put ramconsole somewhere ...
			//End of SMI2
			pmem_setting.ram_console_start=MSM_SMI2_BASE+31*1024*1024;
			pmem_setting.ram_console_size=0x00100000;
			break;
		case MACH_TYPE_HTCRAPHAEL:
		case MACH_TYPE_HTCRAPHAEL_CDMA:
		case MACH_TYPE_HTCDIAMOND_CDMA:
		case MACH_TYPE_HTCBLACKSTONE:
		case MACH_TYPE_HTCTOPAZ:
		case MACH_TYPE_HTCRHODIUM:
		case MACH_TYPE_HTCKOVSKY:
			//SMI 32 + EBI 2*128 or 1*256 (newer htctopaz)

#if 0
			// only htctopaz for now
			if (board_mcp_monodie()) {
				// we can start right after the first 128MB
				pmem_shift = 0x8000000;
			}
#endif

			pmem_setting.pmem_start=MSM_EBIN_BASE+128*1024*1024-34*1024*1024-pmem_shift;
			pmem_setting.pmem_size=32*1024*1024;//32MB
			CALC_PMEM(pmem_adsp, pmem, 0);//16MB
			CALC_PMEM(fb, pmem_adsp, 2*1024*1024);//2MB
			CALC_PMEM(pmem_camera, fb, 0);//1MB
			//Total 34MB

			if (machine_is_htcrhodium()) {
				// wince oemaddresstable gives us 0-107mb usable range,
				// so set up 99mb for bank0 and 8mb for gpu1 (=107mb)
				pmem_setting.pmem_gpu1_start=MSM_EBI_BASE+99*1024*1024;
			} else if (machine_is_htctopaz()) {
				// wince oemaddresstable gives us 0-112mb usable range,
				// so set up 104mb for bank0 and 8mb for gpu1 (=112mb)
				// (confirmed to not work on topa when gpu1 mem exceeds
				// 112mb)
				pmem_setting.pmem_gpu1_start=MSM_EBI_BASE+104*1024*1024;
			} else {
				pmem_setting.pmem_gpu1_start=MSM_EBI_BASE+107*1024*1024;
			}
			pmem_setting.pmem_gpu1_size=8*1024*1024;

			pmem_setting.ram_console_start=0x8e0000;
			pmem_setting.ram_console_size=0x20000;
			break;
		default:
			//SMI 32 + EBI 128
			//So much things for so few memory

			pmem_setting.pmem_start=MSM_EBI_BASE+89*1024*1024;
			pmem_setting.pmem_size=0x800000;//8MB
			CALC_PMEM(pmem_adsp, pmem, 0x800000);//8MB
			CALC_PMEM(fb, pmem_adsp, 0x200000);//2MB
			CALC_PMEM(pmem_camera, fb, 0x100000);//1MB
	}
	//GPU0 must be in SMI1
	pmem_setting.pmem_gpu0_start=MSM_SMI_BASE+1024*1024;
	pmem_setting.pmem_gpu0_size=0x700000;
	resources_msm_fb[0].start=pmem_setting.fb_start;
	resources_msm_fb[0].end=pmem_setting.fb_start+pmem_setting.fb_size;
	resources_msm_fb[0].flags=IORESOURCE_MEM;
	msm_add_mem_devices(&pmem_setting);

	return 0;
}
module_init(msm_pmem_init);

MODULE_DESCRIPTION("PMEM settings");
MODULE_AUTHOR("HUSSON Pierre-Hugues <phhusson@free.fr>");
MODULE_LICENSE("GPL");
