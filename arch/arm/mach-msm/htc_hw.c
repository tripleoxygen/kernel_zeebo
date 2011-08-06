/* arch/arm/mach-msm/htc_hw.c
 * Author: Joe Hansche <madcoder@gmail.com>
 * Based on vogue-hw.c by Martin Johnson <M.J.Jonson@massey.ac.nz>
 * 
 * Stripped down from .27 version.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <asm/mach-types.h>
#include <mach/msm_iomap.h>
#include <mach/board_htc.h>

#if 1
 #define DHTC(fmt, arg...) printk(KERN_DEBUG "[HTC] %s: " fmt "\n", __FUNCTION__, ## arg)
#else
 #define DHTC(fmt, arg...) do {} while (0)
#endif

static ssize_t machine_variant_show(struct class *class,
	struct class_attribute *attr, char *buf)
{
	char machine_variant[8];
	int i;

	if(!machine_is_htcrhodium())
		return 0;

	/* RHOD model stored at MSM_SPL_BASE + 0x81068 as wchar */
	/* Expected format 'RHODn00' */
	for(i = 0; i < 7; i++) {
		machine_variant[i] = (char)*(unsigned short*)(MSM_SPL_BASE + 0x81068 + i*2);
	}
	machine_variant[7] = 0;

	return sprintf(buf, "%s\n", machine_variant);
}

int get_machine_variant_type(void)
{
	int machine_variant_type = MACHINE_VARIANT_UNDEFINED;
	char machine_variant[10];

	if(machine_is_htcrhodium()
		&& machine_variant_show(NULL, NULL, machine_variant) >= 5)
	{
		switch (machine_variant[4]) {
			case '1':
				machine_variant_type = MACHINE_VARIANT_RHOD_1XX;
				break;
			case '2':
				machine_variant_type = MACHINE_VARIANT_RHOD_2XX;
				break;
			case '3':
				machine_variant_type = MACHINE_VARIANT_RHOD_3XX;
				break;
			case '4':
				machine_variant_type = MACHINE_VARIANT_RHOD_4XX;
				break;
			case '5':
				machine_variant_type = MACHINE_VARIANT_RHOD_5XX;
				break;
		}
	} 

	return machine_variant_type;
}
EXPORT_SYMBOL(get_machine_variant_type);

static ssize_t radio_show(struct class *class, struct class_attribute *attr,
	char *buf)
{
	char *radio_type = ((machine_is_htcraphael_cdma()
		|| machine_is_htcraphael_cdma500()) || machine_is_htcdiamond_cdma())
			? "CDMA" : "GSM";
	return sprintf(buf, "%s\n", radio_type);
}

static ssize_t machtype_show(struct class *class, struct class_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", machine_arch_type);
}

extern unsigned int __amss_version; // amss_para.c
static ssize_t amss_show(struct class *class, struct class_attribute *attr,
	char *buf)
{
	return sprintf(buf, "%d\n", __amss_version);
}

static struct class_attribute htc_hw_class_attrs[] = {
	__ATTR_RO(radio),
	__ATTR_RO(machtype),
	__ATTR_RO(amss),
	__ATTR_RO(machine_variant),
	__ATTR_NULL,
};

static struct class htc_hw_class = {
	.name = "htc_hw",
	.class_attrs = htc_hw_class_attrs,
};

static int __init htc_hw_probe(struct platform_device *pdev)
{
	int ret;

	ret = class_register(&htc_hw_class);

	if (ret)
		printk(KERN_ERR "%s: class init failed: %d\n", __func__, ret);
	DHTC("done");
	return ret;
}

static struct platform_driver htc_hw_driver = {
	.probe = htc_hw_probe,
	.driver = {
		.name = "htc_hw",
		.owner = THIS_MODULE,
	},
};

static int __init htc_hw_init(void)
{
	DHTC("Initializing HTC hardware platform driver");
	return platform_driver_register(&htc_hw_driver);
}

module_init(htc_hw_init);
MODULE_DESCRIPTION("HTC hardware platform driver");
MODULE_AUTHOR("Joe Hansche <madcoder@gmail.com>");
MODULE_LICENSE("GPL");
