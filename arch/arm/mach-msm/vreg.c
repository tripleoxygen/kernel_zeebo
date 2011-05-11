/* arch/arm/mach-msm/vreg.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <mach/vreg.h>
#include <mach/msm_iomap.h>
#include <asm/io.h>

#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
#include "proc_comm_wince.h"
#else
#include "proc_comm.h"
#endif

struct vreg {
	const char *name;
	unsigned id;
	int status;
	unsigned refcnt;
};

#define VREG(_name, _id, _status, _refcnt) \
	{ .name = _name, .id = _id, .status = _status, .refcnt = _refcnt }

static struct vreg vregs[] = {
	VREG("msma",	0, 0, 0),
	VREG("msmp",	1, 0, 0),
	VREG("msme1",	2, 0, 0),
	VREG("msmc1",	3, 0, 0),
	VREG("msmc2",	4, 0, 0),
	VREG("gp3",	5, 0, 0),
	VREG("msme2",	6, 0, 0),
	VREG("gp4",	7, 0, 0),
	VREG("gp1",	8, 0, 0),
	VREG("tcxo",	9, 0, 0),
	VREG("pa",	10, 0, 0),
	VREG("rftx",	11, 0, 0),
	VREG("rfrx1",	12, 0, 0),
	VREG("rfrx2",	13, 0, 0),
	VREG("synt",	14, 0, 0),
	VREG("wlan",	15, 0, 0),
	VREG("usb",	16, 0, 0),
	VREG("boost",	17, 0, 0),
	VREG("mmc",	18, 0, 0),
	VREG("ruim",	19, 0, 0),
	VREG("msmc0",	20, 0, 0),
	VREG("gp2",	21, 0, 0),
	VREG("gp5",	22, 0, 0),
	VREG("gp6",	23, 0, 0),
	VREG("rf",	24, 0, 0),
	VREG("rf_vco",	26, 0, 0),
	VREG("mpll",	27, 0, 0),
	VREG("s2",	28, 0, 0),
	VREG("s3",	29, 0, 0),
	VREG("rfubm",	30, 0, 0),
	VREG("ncp",	31, 0, 0),
	VREG("gp7",	32, 0, 0),
	VREG("gp8",	33, 0, 0),
	VREG("gp9",	34, 0, 0),
	VREG("gp10",	35, 0, 0),
	VREG("gp11",	36, 0, 0),
	VREG("gp12",	37, 0, 0),
	VREG("gp13",	38, 0, 0),
	VREG("gp14",	39, 0, 0),
	VREG("gp15",	40, 0, 0),
	VREG("gp16",	41, 0, 0),
	VREG("gp17",	42, 0, 0),
	VREG("s4",	43, 0, 0),
	VREG("usb2",	44, 0, 0),
	VREG("wlan2",	45, 0, 0),
	VREG("xo_out",	46, 0, 0),
	VREG("lvsw0",	47, 0, 0),
	VREG("lvsw1",	48, 0, 0),
};

struct vreg *vreg_get(struct device *dev, const char *id)
{
	int n;
	for (n = 0; n < ARRAY_SIZE(vregs); n++) {
		if (!strcmp(vregs[n].name, id))
			return vregs + n;
	}
	return ERR_PTR(-ENOENT);
}

struct vreg *vreg_get_by_id(struct device *dev, int id)
{
	if (id < ARRAY_SIZE(vregs))
		return vregs + id;
	return ERR_PTR(-ENOENT);
}

void vreg_put(struct vreg *vreg)
{
}

int vreg_enable(struct vreg *vreg)
{
	unsigned id = vreg->id;

#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
	struct msm_dex_command dex;
	id = 1U << id;
	dex.cmd = PCOM_PMIC_REG_ON;
	dex.has_data = 1;
	dex.data = id;
	if (vreg->refcnt == 0)
	vreg->status = msm_proc_comm_wince(&dex, 0);
#else
	unsigned enable = 1;
	if (vreg->refcnt == 0)
	vreg->status = msm_proc_comm(PCOM_VREG_SWITCH, &id, &enable);
#endif

	if ((vreg->refcnt < UINT_MAX) && (!vreg->status))
		vreg->refcnt++;

	return vreg->status;
}

int vreg_disable(struct vreg *vreg)
{
	unsigned id = vreg->id;
#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
	struct msm_dex_command dex;
#endif

	if (!vreg->refcnt)
		return 0;

#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
	id = 1U << id;
	dex.cmd = PCOM_PMIC_REG_OFF;
	dex.has_data = 1;
	dex.data = id;
	if (vreg->refcnt == 1)
	vreg->status = msm_proc_comm_wince(&dex, 0);
#else
	unsigned enable = 0;
	if (vreg->refcnt == 1)
	vreg->status = msm_proc_comm(PCOM_VREG_SWITCH, &id, &enable);
#endif

	if (!vreg->status)
		vreg->refcnt--;

	return vreg->status;
}

int vreg_set_level(struct vreg *vreg, unsigned mv)
{
	unsigned id = vreg->id;


#if defined(CONFIG_MSM_AMSS_VERSION_WINCE)
	struct msm_dex_command dex = { 
		.cmd = PCOM_PMIC_REG_VOLTAGE,
		.has_data = 1, 
		.data = (1U << id) };
	// This reg appears to only be used by vreg_set_level()
	writel(mv, MSM_SHARED_RAM_BASE + 0xfc130);
	printk(KERN_DEBUG "vreg_set_level %d -> %u\n", id, mv);
	vreg->status = msm_proc_comm_wince(&dex, 0);
#else
	vreg->status = msm_proc_comm(PCOM_VREG_SET_LEVEL, &id, &mv);
#endif
	return vreg->status;
}

#if defined(CONFIG_DEBUG_FS)

static int vreg_debug_set(void *data, u64 val)
{
	struct vreg *vreg = data;
	switch (val) {
	case 0:
		vreg_disable(vreg);
		break;
	case 1:
		vreg_enable(vreg);
		break;
	default:
		vreg_set_level(vreg, val);
		break;
	}
	return 0;
}

static int vreg_debug_get(void *data, u64 *val)
{
	struct vreg *vreg = data;

	if (!vreg->status)
		*val = 0;
	else
		*val = 1;

	return 0;
}

static int vreg_debug_count_set(void *data, u64 val)
{
	struct vreg *vreg = data;
	if (val > UINT_MAX)
		val = UINT_MAX;
	vreg->refcnt = val;
	return 0;
}

static int vreg_debug_count_get(void *data, u64 *val)
{
	struct vreg *vreg = data;

	*val = vreg->refcnt;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vreg_fops, vreg_debug_get, vreg_debug_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(vreg_count_fops, vreg_debug_count_get,
			vreg_debug_count_set, "%llu\n");

static int __init vreg_debug_init(void)
{
	struct dentry *dent;
	int n;
	char name[32];
	const char *refcnt_name = "_refcnt";

	dent = debugfs_create_dir("vreg", 0);
	if (IS_ERR(dent))
		return 0;

	for (n = 0; n < ARRAY_SIZE(vregs); n++) {
		(void) debugfs_create_file(vregs[n].name, 0644,
					   dent, vregs + n, &vreg_fops);

		strlcpy(name, vregs[n].name, sizeof(name));
		strlcat(name, refcnt_name, sizeof(name));
		(void) debugfs_create_file(name, 0644,
					   dent, vregs + n, &vreg_count_fops);
	}

	return 0;
}

device_initcall(vreg_debug_init);
#endif
