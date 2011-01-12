/* arch/arm/mach-msm/clock-msm-a11.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2008-2010 htc-linux.org
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <mach/msm_rpcrouter.h>

#include <mach/msm_iomap.h>
#include <asm/io.h>

#include "clock.h"

static DEFINE_MUTEX(clocks_mutex);
static DEFINE_SPINLOCK(clocks_lock);
static HLIST_HEAD(clocks);

enum {
	DEBUG_UNKNOWN_ID = 1 << 0,
	DEBUG_UNKNOWN_FREQ = 1 << 1,
	DEBUG_MDNS = 1 << 2,
	DEBUG_UNKNOWN_CMD = 1 << 3,
};
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

int nand_boot = 0;
module_param_named(nand_boot, nand_boot, int, 0);

static int a11_clk_is_enabled(unsigned id);
static int a11_clk_enable(unsigned id);
static void a11_clk_disable(unsigned id);

#if 1
#define D(x...) printk(KERN_WARNING "clock-msm-a11: " x)
#else
#define D(x...) do {} while (0)
#endif

struct mdns_clock_params {
	unsigned long freq;
	unsigned calc_freq;
	unsigned md;
	unsigned ns;
	unsigned pll_freq;
	unsigned clk_id;
};

struct msm_clock_params {
//      unsigned clk_id;
	unsigned idx;
	unsigned offset;	// Offset points to .ns register
	bool setup_mdns;
	char *name;
};

static int max_clk_rate[P_NR_CLKS], min_clk_rate[P_NR_CLKS];

#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#define TCX0			19200000	// Hz
#define PLL_FREQ(l, m, n)	(TCX0 * (l) + TCX0 * (m) / (n))

static unsigned int pll_get_rate(int n)
{
	unsigned int mode, L, M, N, freq;

	if (n == -1)
		return TCX0;
	if (n > 3)
		return 0;
	else {

		do {
			mode = readl(PLLn_BASE(n)) & 0x3f;
			cpu_relax();
			udelay(50);
		} while (mode == 0);

		mode = readl(PLLn_BASE(n) + 0x0);
		L = readl(PLLn_BASE(n) + 0x4);
		M = readl(PLLn_BASE(n) + 0x8);
		N = readl(PLLn_BASE(n) + 0xc);
		freq = PLL_FREQ(L, M, N);
		D("PLL%d: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n",
		  n, mode, L, M, N, freq, freq / 1000000);
	}

	return freq;
}

static unsigned int idx2pll(unsigned idx)
{
	int ret;

	switch (idx) {
	case 0:		/* TCX0 */
		ret = -1;
		break;
	case 1:		/* PLL1 */
		ret = 1;
		break;
	case 4:		/* PLL0 */
		ret = 0;
		break;
	default:
		ret = 4;	/* invalid */
	}

	return ret;
}

/* Note that some are not used
 * like, we use MicroP instead of GP CLK
 */
static struct msm_clock_params msm_clock_parameters[NR_CLKS] = {
	[ADSP_CLK] = {.offset = 0x34,.name = "ADSP_CLK",},
	[GP_CLK] = {.offset = 0x5c,.name = "GP_CLK",},
	[GRP_3D_CLK] = {.idx = 3,.name = "GRP_3D_CLK",},
	[IMEM_CLK] = {.idx = 3,.name = "IMEM_CLK",},
	[I2C_CLK] = {.offset = 0x68,.setup_mdns = 1,.name = "I2C_CLK",},
	[MDC_CLK] = {.offset = 0x7c,.name = "MDC_CLK",},
	[MDP_CLK] = {.idx = 9,.name = "MDP_CLK",},
	[PMDH_CLK] = {.offset = 0x8c,.setup_mdns = 0,.name = "PMDH_CLK",},
	[SDAC_CLK] = {.offset = 0x9c,.name = "SDAC_CLK",},
	[SDC1_CLK] = {.offset = 0xa4,.setup_mdns = 1,.name = "SDC1_CLK",},
	[SDC2_CLK] = {.offset = 0xac,.setup_mdns = 1,.name = "SDC2_CLK",},
	[SDC3_CLK] = {.offset = 0xb4,.setup_mdns = 1,.name = "SDC3_CLK",},
	[SDC4_CLK] = {.offset = 0xbc,.setup_mdns = 1,.name = "SDC4_CLK",},
	[SDC1_P_CLK] = {.idx = 7,.name = "SDC1_P_CLK",},
	[SDC2_P_CLK] = {.idx = 8,.name = "SDC2_P_CLK",},
	[SDC3_P_CLK] = {.idx = 27,.name = "SDC3_P_CLK",},
	[SDC4_P_CLK] = {.idx = 28,.name = "SDC4_P_CLK",},
	[UART1_CLK] = {.offset = 0xe0,.name = "UART1_CLK",},
	[UART2_CLK] = {.offset = 0xe0,.name = "UART2_CLK",},
	[UART3_CLK] = {.offset = 0xe0,.name = "UART3_CLK",},
	[UART1DM_CLK] = {.idx = 17,.offset = 0xd4,.setup_mdns = 1,.name =
			 "UART1DM_CLK",},
	[UART2DM_CLK] = {.idx = 26,.offset = 0xdc,.setup_mdns = 1,.name =
			 "UART2DM_CLK",},
	[USB_HS_CLK] = {.offset = 0x2c0,.name = "USB_HS_CLK",},
	[USB_HS_P_CLK] = {.idx = 25,.name = "USB_HS_P_CLK",},
	[VFE_CLK] = {.offset = 0x44,.setup_mdns = 1,.name = "VFE_CLK",},
	[VFE_MDC_CLK] = {.offset = 0x44,.name = "VFE_MDC_CLK",},
	[VDC_CLK] = {.offset = 0xf0,.name = "VDC_CLK",},
};

// This formula is used to generate md and ns reg values
#define MSM_CLOCK_REG(frequency,M,N,D,PRE,a5,SRC,MNE,pll_frequency) { \
	.freq = (frequency), \
	.md = ((0xffff & (M)) << 16) | (0xffff & ~((D) << 1)), \
	.ns = ((0xffff & ~((N) - (M))) << 16) \
	    | ((0xff & (0xa | (MNE))) << 8) \
	    | ((0x7 & (a5)) << 5) \
	    | ((0x3 & (PRE)) << 3) \
	    | (0x7 & (SRC)), \
	.pll_freq = (pll_frequency), \
	.calc_freq = (pll_frequency*M/((PRE+1)*N)), \
}

static struct mdns_clock_params *msm_clock_freq_parameters;

// GSM phones typically use a 245 MHz PLL0
struct mdns_clock_params msm_clock_freq_parameters_pll0_245[] = {

	MSM_CLOCK_REG(144000, 3, 0x64, 0x32, 3, 3, 0, 1, 19200000),	/* SD, 144kHz */
#if 0				/* wince uses this clock setting for UART2DM */
	MSM_CLOCK_REG(1843200, 3, 0x64, 0x32, 3, 2, 4, 1, 245760000),	/*  115200*16=1843200 */
//      MSM_CLOCK_REG(     , 2, 0xc8, 0x64, 3, 2, 1, 1, 768888888), /* 1.92MHz for 120000 bps */
#else
	MSM_CLOCK_REG(7372800, 3, 0x64, 0x32, 0, 2, 4, 1, 245760000),	/*  460800*16, will be divided by 4 for 115200 */
#endif
	MSM_CLOCK_REG(12000000, 1, 0x20, 0x10, 1, 3, 1, 1, 768000000),	/* SD, 12MHz */
	MSM_CLOCK_REG(14745600, 3, 0x32, 0x19, 0, 2, 4, 1, 245760000),	/* BT, 921600 (*16) */
	MSM_CLOCK_REG(19200000, 1, 0x0a, 0x05, 3, 3, 1, 1, 768000000),	/* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000, 1, 0x10, 0x08, 1, 3, 1, 1, 768000000),	/* SD, 24MHz */
	MSM_CLOCK_REG(32000000, 1, 0x0c, 0x06, 1, 3, 1, 1, 768000000),	/* SD, 32MHz */
	MSM_CLOCK_REG(58982400, 6, 0x19, 0x0c, 0, 2, 4, 1, 245760000),	/* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000, 0x19, 0x60, 0x30, 0, 2, 4, 1, 245760000),	/* BT, 4000000 (*16) */
	{0, 0, 0, 0, 0, 0},
};

// CDMA phones typically use a 196 MHz PLL0
struct mdns_clock_params msm_clock_freq_parameters_pll0_196[] = {

	MSM_CLOCK_REG(144000, 3, 0x64, 0x32, 3, 3, 0, 1, 19200000),	/* SD, 144kHz */
	MSM_CLOCK_REG(7372800, 3, 0x50, 0x28, 0, 2, 4, 1, 196608000),	/*  460800*16, will be divided by 4 for 115200 */
	MSM_CLOCK_REG(12000000, 1, 0x20, 0x10, 1, 3, 1, 1, 768000000),	/* SD, 12MHz */
	MSM_CLOCK_REG(14745600, 3, 0x28, 0x14, 0, 2, 4, 1, 196608000),	/* BT, 921600 (*16) */
	MSM_CLOCK_REG(19200000, 1, 0x0a, 0x05, 3, 3, 1, 1, 768000000),	/* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000, 1, 0x10, 0x08, 1, 3, 1, 1, 768000000),	/* SD, 24MHz */
	MSM_CLOCK_REG(32000000, 1, 0x0c, 0x06, 1, 3, 1, 1, 768000000),	/* SD, 32MHz */
	MSM_CLOCK_REG(58982400, 3, 0x0a, 0x05, 0, 2, 4, 1, 196608000),	/* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000, 0x7d, 0x180, 0xC0, 0, 2, 4, 1, 196608000),	/* BT, 4000000 (*16) */
	{0, 0, 0, 0, 0, 0},
};

// defines from MSM7500_Core.h

// often used defines
#define MSM_PRPH_WEB_NS_REG	(MSM_CLK_CTL_BASE+0x80)
#define MSM_GRP_3D_NS_REG	(MSM_CLK_CTL_BASE+0x84)
#define MSM_AXI_RESET 		(MSM_CLK_CTL_BASE+0x208)
#define MSM_ROW_RESET 		(MSM_CLK_CTL_BASE+0x214)
#define MSM_VDD_GRP_3D_GFS_CTL	(MSM_CLK_CTL_BASE+0x284)
#define MSM_VDD_VDC_GFS_CTL	(MSM_CLK_CTL_BASE+0x288)
#define MSM_RAIL_CLAMP_IO	(MSM_CLK_CTL_BASE+0x290)

#define REG_OR(reg, value) do { u32 i = readl(reg); writel(i | (value), reg); } while(0)
#define REG_AND(reg, value) do { u32 i = readl(reg); writel(i & ~value, reg); } while(0)
#define REG_SET(reg, value) do { writel(value, reg); } while(0)

static void set_grp_3d_clk(int on)
{
	static int last_status = -1;
	int i = 0;
	int status = 0;
	int control;

	if (on == last_status)
		return;

	if (on != 0) {
		REG_OR(MSM_AXI_RESET, 0x20);
		REG_OR(MSM_ROW_RESET, 0x20000);
		REG_SET(MSM_VDD_GRP_3D_GFS_CTL, 0x11f);
		// very rough delay
		mdelay(20);

		REG_OR(MSM_GRP_3D_NS_REG, 0x800);
		REG_OR(MSM_GRP_3D_NS_REG, 0x80);
		REG_OR(MSM_GRP_3D_NS_REG, 0x200);

		// grp idx
		REG_OR(MSM_CLK_CTL_BASE, 0x8);

		REG_AND(MSM_RAIL_CLAMP_IO, 0x4);
		REG_AND(MSM_PRPH_WEB_NS_REG, 0x1);
		REG_AND(MSM_AXI_RESET, 0x20);
		REG_AND(MSM_ROW_RESET, 0x20000);
	} else {
		REG_OR(MSM_GRP_3D_NS_REG, 0x800);
		REG_OR(MSM_GRP_3D_NS_REG, 0x80);
		REG_OR(MSM_GRP_3D_NS_REG, 0x200);

		//grp idx
		REG_OR(MSM_CLK_CTL_BASE, 0x8);

		//grp MD
		REG_OR(MSM_PRPH_WEB_NS_REG, 0x1);

		while (status == 0 && i < 100) {
			i++;
			status = readl(MSM_GRP_3D_NS_REG) & 0x1;
		}

		REG_OR(MSM_AXI_RESET, 0x20);
		REG_OR(MSM_ROW_RESET, 0x20000);

		REG_AND(MSM_GRP_3D_NS_REG, 0x800);
		REG_AND(MSM_GRP_3D_NS_REG, 0x80);
		REG_AND(MSM_GRP_3D_NS_REG, 0x200);

		//grp clk ramp
		REG_OR(MSM_RAIL_CLAMP_IO, 0x4);

		REG_SET(MSM_VDD_GRP_3D_GFS_CTL, 0x11f);

		control = readl(MSM_VDD_VDC_GFS_CTL);

		//grp idx
		if (control & 0x100) {
			REG_AND(MSM_CLK_CTL_BASE, 0x8);
		}
	}
	last_status = on;
}

static inline struct msm_clock_params msm_clk_get_params(unsigned id)
{
	struct msm_clock_params empty = { };
	if (id < NR_CLKS)
		return msm_clock_parameters[id];
	return empty;
}

static inline unsigned msm_clk_enable_bit(unsigned id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	if (!params.idx)
		return 0;
	return 1U << params.idx;
}

static inline unsigned msm_clk_reg_offset(unsigned id)
{
	struct msm_clock_params params;
	params = msm_clk_get_params(id);
	return params.offset;
}

static int set_mdns_host_clock(unsigned id, unsigned long freq)
{
	int n = 0;
	unsigned offset;
	bool found, enabled, needs_setup;
	struct msm_clock_params params;
	found = false;

	params = msm_clk_get_params(id);
	offset = params.offset;
	needs_setup = params.setup_mdns;

	if (id == EBI1_CLK)
		return 0;

	if (debug_mask & DEBUG_MDNS)
		D("%s(%u, %lu); bitidx=%u, offset=%x\n", __func__, id, freq,
		  params.idx, params.offset);

	if (!params.offset) {
		D("%s: FIXME! Don't know how to set clock %u - no known Md/Ns reg\n", __func__, id);
		return 0;
		//return -ENOTSUPP;
	}

	enabled = a11_clk_is_enabled(id);
	if (enabled)
		a11_clk_disable(id);

	if (!needs_setup)
		goto skip_clock;

	while (msm_clock_freq_parameters[n].freq) {
		n++;
	}

	for (n--; n >= 0; n--) {
		if (freq < msm_clock_freq_parameters[n].freq)
			continue;

		// This clock requires MD and NS regs to set frequency:
		writel(msm_clock_freq_parameters[n].md,
		       MSM_CLK_CTL_BASE + offset - 4);

		if (id == VFE_CLK) {
			writel((msm_clock_freq_parameters[n].ns & ~0x7e00) |
			       (readl(MSM_CLK_CTL_BASE + offset)
				& 0x7e00)
			       , MSM_CLK_CTL_BASE + offset);
		} else {
			writel(msm_clock_freq_parameters[n].ns,
			       MSM_CLK_CTL_BASE + offset);
		}
		if (debug_mask & DEBUG_MDNS)
			D("%s: %u, freq=%lu calc_freq=%u pll%d=%u expected pll =%u\n", __func__, id, msm_clock_freq_parameters[n].freq, msm_clock_freq_parameters[n].calc_freq, msm_clock_freq_parameters[n].ns & 7, pll_get_rate(idx2pll(msm_clock_freq_parameters[n].ns & 7)), msm_clock_freq_parameters[n].pll_freq);

		found = true;
		break;
	}

 skip_clock:
	if (enabled)
		a11_clk_enable(id);

	if ((!found && needs_setup) && (debug_mask & DEBUG_UNKNOWN_FREQ)) {
		D("FIXME! set_sdcc_host_clock could not "
		  "find suitable parameter for freq %lu\n", freq);
	}

	return 0;
}

static unsigned long get_mdns_host_clock(unsigned id)
{
	int n;
	unsigned offset;
	unsigned mdreg;
	unsigned nsreg;
	unsigned long freq = 0;

	offset = msm_clk_reg_offset(id);
	if (offset == 0)
		return -EINVAL;

	mdreg = readl(MSM_CLK_CTL_BASE + offset - 4);
	nsreg = readl(MSM_CLK_CTL_BASE + offset);

	n = 0;
	while (msm_clock_freq_parameters[n].freq) {
		if (msm_clock_freq_parameters[n].md == mdreg &&
		    msm_clock_freq_parameters[n].ns == nsreg) {
			freq = msm_clock_freq_parameters[n].freq;
			break;
		}
		n++;
	}

	return freq;
}

static int a11_clk_enable(unsigned id)
{
	struct msm_clock_params params;
	int done = 0;

	params = msm_clk_get_params(id);
	switch (id) {
	case MDP_CLK:
		//Check this mask
		writel((readl(MSM_CLK_CTL_BASE) & 0x3ffffdff) | 0x200,
		       MSM_CLK_CTL_BASE);
		done = 1;
		break;

	case PMDH_CLK:
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x67f) |
		       0x800, MSM_CLK_CTL_BASE + params.offset);
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0xc7f) |
		       0x200, MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case I2C_CLK:
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x600) |
		       0x800, MSM_CLK_CTL_BASE + params.offset);
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0xc00) |
		       0x200, MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case ADSP_CLK:
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x7ff7ff) |
		       0x800, MSM_CLK_CTL_BASE + params.offset);
		writel((readl(MSM_CLK_CTL_BASE + params.offset) & 0x6fffff) |
		       0x100000, MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case UART1_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x10,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x20,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case UART2_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x400,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case UART3_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x10000,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x20000,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case SDAC_CLK:
	case SDC1_CLK:
	case SDC2_CLK:
	case SDC3_CLK:
	case SDC4_CLK:
	case USB_HS_CLK:
	case VDC_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x100,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x200,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case IMEM_CLK:
	case GRP_3D_CLK:
		set_grp_3d_clk(1);
		done = 1;
		break;

	case MDC_CLK:
		D("Enabling MDC clock\n");
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0xa01,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;
	case VFE_CLK:
	case VFE_MDC_CLK:
		writel((readl(MSM_CLK_CTL_BASE + params.offset) | (1 << 13)),
		       MSM_CLK_CTL_BASE + params.offset);
		writel((readl(MSM_CLK_CTL_BASE + params.offset) | (1 << 9)),
		       MSM_CLK_CTL_BASE + params.offset);
		writel((readl(MSM_CLK_CTL_BASE + params.offset) | (1 << 11)),
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	default:
		break;
	}

	if (params.idx) {
		writel(readl(MSM_CLK_CTL_BASE) | (1U << params.idx),
		       MSM_CLK_CTL_BASE);
		done = 1;
	}

	if (done)
		return 0;

	//XXX: too spammy, extreme debugging only: D(KERN_DEBUG "%s: %d\n", __func__, id);
	if (debug_mask & DEBUG_UNKNOWN_ID)
		D("%s: FIXME! enabling a clock that doesn't have an ena bit "
		  "or ns-only offset: %u\n", __func__, id);

	return 0;
}

static void a11_clk_disable(unsigned id)
{
	int done = 0;
	struct msm_clock_params params;
	params = msm_clk_get_params(id);

	//GRP and IMEM use special order.But do they really need it?
	if (params.idx && (id != GRP_3D_CLK && id != IMEM_CLK)) {
		writel(readl(MSM_CLK_CTL_BASE) & ~(1U << params.idx),
		       MSM_CLK_CTL_BASE);
		done = 1;
	}

	switch (id) {
	case MDP_CLK:
		writel(readl(MSM_CLK_CTL_BASE) & 0x3ffffdff, MSM_CLK_CTL_BASE);
		done = 1;
		break;

	case UART1_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x20,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x10,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case UART2_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x400,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case UART3_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x20000,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x10000,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case SDAC_CLK:
	case SDC1_CLK:
	case SDC2_CLK:
	case SDC3_CLK:
	case SDC4_CLK:
	case USB_HS_CLK:
	case VDC_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x200,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x100,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case I2C_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0xc00,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x600,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case ADSP_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x6fffff,
		       MSM_CLK_CTL_BASE + params.offset);
		if (readl(MSM_CLK_CTL_BASE + params.offset) & 0x280) {
			done = 1;
			break;
		}
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x7ff7ff,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case PMDH_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0xc7f,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & 0x67f,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case IMEM_CLK:
	case GRP_3D_CLK:
		set_grp_3d_clk(0);
		writel(readl(MSM_CLK_CTL_BASE) & ~(1U << params.idx),
		       MSM_CLK_CTL_BASE);
		done = 1;
		break;

	case MDC_CLK:
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0xa00,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case VFE_CLK:
	case VFE_MDC_CLK:
		D("Disabling VFE clock\n");
		done = 1;
		break;

	default:
		break;
	}

	if (done)
		return;

	//XXX: D(KERN_DEBUG "%s: %d\n", __func__, id);

	if (debug_mask & DEBUG_UNKNOWN_ID)
		D("%s: FIXME! disabling a clock that doesn't have an "
		  "ena bit: %u\n", __func__, id);
	return;
}

static int a11_clk_set_rate(unsigned id, unsigned long rate)
{
	int retval;
	retval = 0;

	if (DEBUG_MDNS)
		D("%s: id=%u rate=%lu\n", __func__, id, rate);

	retval = set_mdns_host_clock(id, rate);

	return retval;
}

static int a11_clk_set_min_rate(unsigned id, unsigned long rate)
{
	if (id < NR_CLKS)
		min_clk_rate[id] = rate;
	else if (debug_mask & DEBUG_UNKNOWN_ID)
		D(" FIXME! clk_set_min_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static int a11_clk_set_max_rate(unsigned id, unsigned long rate)
{
	if (id < NR_CLKS)
		max_clk_rate[id] = rate;
	else if (debug_mask & DEBUG_UNKNOWN_ID)
		D(" FIXME! clk_set_min_rate not implemented; %u:%lu NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static unsigned long a11_clk_get_rate(unsigned id)
{
	unsigned long rate = 0;

	switch (id) {
		/* known MD/NS clocks, MSM_CLK dump and arm/mach-msm/clock-7x30.c */
	case SDC1_CLK:
	case SDC2_CLK:
	case SDC3_CLK:
	case SDC4_CLK:
	case UART1DM_CLK:
	case UART2DM_CLK:
	case USB_HS_CLK:
	case SDAC_CLK:
	case TV_DAC_CLK:
	case TV_ENC_CLK:
	case USB_OTG_CLK:
		rate = get_mdns_host_clock(id);
		break;

	case SDC1_P_CLK:
	case SDC2_P_CLK:
	case SDC3_P_CLK:
	case SDC4_P_CLK:
		rate = 64000000;	/* g1 value */
		break;

	default:
		//TODO: support all clocks
		if (debug_mask & DEBUG_UNKNOWN_ID)
			D("%s: unknown clock: id=%u\n", __func__, id);
		rate = 0;
	}

	return rate;
}

static int a11_clk_set_flags(unsigned id, unsigned long flags)
{
	if (id == VFE_CLK) {
		if (flags & 0x100) {
			writel((readl(MSM_CLK_CTL_BASE + 0x44) | (1 << 14)),
			       MSM_CLK_CTL_BASE + 0x44);
			D("Setting external clock for VFE_CLK\n");
		} else if (flags & 0x200) {
			writel((readl(MSM_CLK_CTL_BASE + 0x44) & ~(1 << 14)),
			       MSM_CLK_CTL_BASE + 0x44);
			D("Setting internal clock for VFE_CLK\n");
		}
	} else if (debug_mask & DEBUG_UNKNOWN_CMD)
		D("%s not implemented for clock: id=%u, flags=%lu\n",
		  __func__, id, flags);
	return 0;
}

static int a11_clk_is_enabled(unsigned id)
{
	int is_enabled = 0;
	unsigned bit;
	bit = msm_clk_enable_bit(id);
	if (bit > 0) {
		is_enabled = (readl(MSM_CLK_CTL_BASE) & bit) != 0;
	} else if (debug_mask & DEBUG_UNKNOWN_CMD) {
		D("%s not implemented for clock: id=%d\n", __func__, id);
	}
	return is_enabled;
}

static long a11_clk_round_rate(unsigned id, unsigned rate)
{
	/* Not really supported; a11_clk_set_rate() does rounding on it's own. */
	return rate;
}

int a11_clk_reset(unsigned id, enum clk_reset_action action)
{
	if (debug_mask & DEBUG_UNKNOWN_CMD) {
		D("%s implemented for clock: id=%d, action = 0x%x\n", __func__,
		  id, action);
	}

	return 0;
}

void __init msm_clock_a11_fixup(void)
{
	if (nand_boot)
		mdelay(6000);

//      spin_lock_init(&clocks_lock);

	mutex_lock(&clocks_mutex);
	if (pll_get_rate(0) == 196608000) {
		// cdma pll0 = 196 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_196;
	} else {
		// default gsm pll0 = 245 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_245;
	}
	mutex_unlock(&clocks_mutex);
}

#define PMIC_API_PROG		0x30000055
#define PMIC_API_VERS 		0x0
#define PMIC_API_GET_KHZ_PROC	0x1
static void msm_clock_a11_reset_imem(void)
{
	struct msm_rpc_endpoint *pmic_ep;
	int rc;
	struct {
		struct rpc_request_hdr hdr;
		unsigned data[1];
	} req;

	if (nand_boot)
		return;

	writel(0, MSM_IMEM_BASE);
	pr_info("reset imem_config\n");
//	pmic_ep = msm_rpc_connect(PMIC_API_PROG, PMIC_API_VERS, 0);
//	if (IS_ERR(pmic_ep)) {
//		printk("%s: init rpc failed! error: %ld\n",
//		       __func__, PTR_ERR(pmic_ep));
//		goto close;
//	}
	pr_info("IMEM OLD: VAL = %d\n", readl(MSM_IMEM_BASE));
//	req.data[0] = cpu_to_be32(1);
//	rc = msm_rpc_call(pmic_ep, PMIC_API_GET_KHZ_PROC, &req, sizeof(req),
//			  5 * HZ);
//	if (rc < 0)
//		printk("%s: rpc call failed! (%d)\n", __func__, rc);

	msleep(100);
	pr_info("IMEM NEW: VAL = %d\n", readl(MSM_IMEM_BASE));

//close:
//	msm_rpc_close(pmic_ep);
}

struct clk_ops clk_ops_pcom = {
	.enable = a11_clk_enable,
	.disable = a11_clk_disable,
	.auto_off = a11_clk_disable,
	.reset = a11_clk_reset,
	.set_rate = a11_clk_set_rate,
	.set_min_rate = a11_clk_set_min_rate,
	.set_max_rate = a11_clk_set_max_rate,
	.set_flags = a11_clk_set_flags,
	.get_rate = a11_clk_get_rate,
	.is_enabled = a11_clk_is_enabled,
	.round_rate = a11_clk_round_rate,
	.late_init_clk = msm_clock_a11_reset_imem,
};

