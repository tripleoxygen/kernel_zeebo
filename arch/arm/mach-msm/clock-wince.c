/* arch/arm/mach-msm/clock-wince.c
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

static unsigned a11_clk_is_enabled(unsigned id);
static int a11_clk_enable(unsigned id);
static void a11_clk_disable(unsigned id);

#if 1
#define D(x...) printk(KERN_WARNING "clock-wince: " x)
#else
#define D(x...) do {} while (0)
#endif

#define MSM_GLBL_CLK_STATE      ( MSM_CLK_CTL_BASE+0x4 )
#define MSM_CAM_MD_REG          ( MSM_CLK_CTL_BASE+0x40 )
#define MSM_VFE_NS_REG          ( MSM_CLK_CTL_BASE+0x44 )
#define MSM_PRPH_WEB_NS_REG     ( MSM_CLK_CTL_BASE+0x80 )
#define MSM_GRP_NS_REG          ( MSM_CLK_CTL_BASE+0x84 )
#define MSM_CLK_HALT_STATEA     ( MSM_CLK_CTL_BASE+0x100 )
#define MSM_CLK_HALT_STATEB     ( MSM_CLK_CTL_BASE+0x104 )
#define MSM_AXI_RESET           ( MSM_CLK_CTL_BASE+0x208 )
#define MSM_APPS_RESET          ( MSM_CLK_CTL_BASE+0x210 )
#define MSM_ROW_RESET           ( MSM_CLK_CTL_BASE+0x214 )
#define MSM_VDD_VFE_GFS_CTL     ( MSM_CLK_CTL_BASE+0x280 )
#define MSM_VDD_GRP_GFS_CTL     ( MSM_CLK_CTL_BASE+0x284 )
#define MSM_VDD_VDC_GFS_CTL     ( MSM_CLK_CTL_BASE+0x288 )
#define MSM_RAIL_CLAMP_IO       ( MSM_CLK_CTL_BASE+0x290 )

#define REG_OR( reg, value ) do { u32 i = readl( (reg) ); writel( i | (value), (reg) ); } while(0)
#define REG_AND( reg, value ) do {	u32 i = readl( reg ); writel( i & ~value, reg); } while(0)
#define REG_AND_MASK( reg, value ) do {	u32 i = readl( reg ); writel( i & value, reg); } while(0)
#define REG_AND_OR( reg, value_and, value_or ) do { u32 i = readl( reg ); writel( (i & ~(value_and)) | (value_or), reg); } while(0)
#define REG_ANDM_OR( reg, value_and, value_or ) do { u32 i = readl( reg ); writel( (i & (value_and)) | (value_or), reg); } while(0)
#define REG_AND_ADD( reg, value_and, value_add ) do { u32 i = readl( reg ); writel( (i & ~(value_and)) + (value_add), reg); } while(0)
#define REG_SET( reg, value ) do { writel( value, reg ); } while(0)

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

enum pll {
	TCXO = -1,
	PLL0,
	PLL1,
	PLL2,
	PLL3,
	PLL_COUNT = PLL3 + 1,
};

enum pll_src {
	SRC_TCXO = 0,
	SRC_PLL_1 = 1,
	SRC_PLL_2 = 2,
	SRC_PLL_3 = 3,
	SRC_PLL_0 = 4,
};

#define TCXO_RATE			19200000
#define PLLn_BASE(n)	(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#define PLL_FREQ(l, m, n)	(TCXO_RATE * (l) + TCXO_RATE * (m) / (n))

unsigned int pll_get_rate(enum pll pll, bool dump)
{
	unsigned int mode, L, M, N, freq;

	if (pll == TCXO)
		return TCXO_RATE;
	if (pll >= PLL_COUNT)
		return 0;
	else {
		mode = readl(PLLn_BASE(pll) + 0x0);
		L = readl(PLLn_BASE(pll) + 0x4);
		M = readl(PLLn_BASE(pll) + 0x8);
		N = readl(PLLn_BASE(pll) + 0xc);
		freq = PLL_FREQ(L, M, N);
	}

	if (dump)
		D("PLL%d: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n",
				pll, mode, L, M, N, freq, freq / 1000000);

	return freq;
}

/* Note that some are not used
 * like, we use MicroP instead of GP CLK
 */
static struct msm_clock_params msm_clock_parameters[NR_CLKS] = {
	[ADSP_CLK] = {.offset = 0x34,.name = "ADSP_CLK",},
	[EMDH_CLK] = {.offset = 0x50,.name = "EMDH_CLK",},
	[GP_CLK] = {.offset = 0x5c,.name = "GP_CLK",},
	[GRP_3D_CLK] = {.idx = 3, .offset = 0x84, .name = "GRP_3D_CLK",},
	[IMEM_CLK] = {.idx = 3,.name = "IMEM_CLK",},
	[I2C_CLK] = {.offset = 0x68,.setup_mdns = 1,.name = "I2C_CLK",},
	[MDC_CLK] = {.offset = 0x7c,.name = "MDC_CLK",},
	[MDP_CLK] = {.idx = 9,.name = "MDP_CLK",},
	[PMDH_CLK] = {.offset = 0x8c,.name = "PMDH_CLK",},
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
	[USB_OTG_CLK] = {.offset = 0xe8, .name = "USB_OTG_CLK"},
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
	MSM_CLOCK_REG(7372800, 3, 0x64, 0x32, 0, 2, 4, 1, 245760000),	/*  460800*16, will be divided by 4 for 115200 */
	MSM_CLOCK_REG(12000000, 1, 0x20, 0x10, 1, 3, 1, 1, 768000000),	/* SD, 12MHz */
	MSM_CLOCK_REG(14745600, 3, 0x32, 0x19, 0, 2, 4, 1, 245760000),	/* BT, 921600 (*16) */
	MSM_CLOCK_REG(16000000,   1, 0x0c, 0x06, 0, 2, 4, 1, 245760000), /* BT, 1000000 (*16)*/
	MSM_CLOCK_REG(19200000,   1, 0x0a, 0x05, 3, 3, 1, 1, 768000000), /* SD, 19.2MHz */
	MSM_CLOCK_REG(24000000,   1, 0x08, 0x04, 3, 2, 1, 1, 768000000), /* SD & VFE_CLK, 24MHz */
	MSM_CLOCK_REG(32000000,   1, 0x0c, 0x06, 1, 3, 1, 1, 768000000), /* SD, 32MHz */
	MSM_CLOCK_REG(48000000,   1, 0x04, 0x02, 0, 2, 4, 1, 245760000), /* UART_HS, 48MHz */
	MSM_CLOCK_REG(58982400,   6, 0x19, 0x0c, 0, 2, 4, 1, 245760000), /* BT, 3686400 (*16) */
	MSM_CLOCK_REG(64000000,0x19, 0x60, 0x30, 0, 2, 4, 1, 245760000U), /* BT, 4000000 (*16) Normal */
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
	MSM_CLOCK_REG(64000000, 0x7d, 0x180, 0xC0, 0, 2, 4, 1, 196608000U),	/* BT, 4000000 (*16) */
	{0, 0, 0, 0, 0, 0},
};

static unsigned msm_gen_rate_simple_div_pll(unsigned id, unsigned freq) {
	int pll_freqs[PLL_COUNT], divs[PLL_COUNT], i;
	enum pll pll_idx = PLL0;
	unsigned freq0, delta, res, d, ns_val = 0;

	if (!freq)
		return 0;

	if (freq <= TCXO_RATE) {
		pll_idx = TCXO;
		goto set_ns;
	}

	for (i = 0; i < PLL_COUNT; i++) {
		pll_freqs[i] = pll_get_rate(i, false);
		divs[i] = pll_freqs[i] / freq;
		if (divs[i] == 0)
			divs[i] = 1;
	}
	freq0 = pll_freqs[pll_idx] / divs[pll_idx];
	delta = freq0 > freq ? freq0 - freq : freq - freq0;

	for (i = 1; i < PLL_COUNT; i++) {
		res = pll_freqs[i] / divs[i];
		d = res > freq ? res - freq : freq - res;
		if (d < delta) {
			pll_idx = i;
			break;
		}
	}

set_ns:
	switch (pll_idx) {
		case TCXO:
				ns_val = (((TCXO_RATE / freq) - 1) << 3) | SRC_TCXO;
		break;

		case PLL0:
				ns_val = ((divs[pll_idx] - 1) << 3) | SRC_PLL_0;
		break;

		case PLL1:
				ns_val = ((divs[pll_idx] - 1) << 3) | SRC_PLL_1;
		break;

		case PLL2:
				ns_val = ((divs[pll_idx] - 1) << 3) | SRC_PLL_2;
		break;

		case PLL3:
				ns_val = ((divs[pll_idx] - 1) << 3) | SRC_PLL_3;
		break;

		default:
		break;
	}
	printk("%s: rate=%d, ns_val=%08x\n", __func__, freq, ns_val);
	return ns_val;
}

static unsigned msm_decode_rate_simple_div_pll(unsigned ns_val) {
	unsigned div;
	enum pll pll_idx;

	pll_idx = ns_val & 7;
	div = ((ns_val >> 3) & 15) + 1;
	return pll_get_rate(pll_idx, false) / div;
}

static unsigned grp_ns = 0x99;

static void set_grp_rail(int enable)
{
	int i = 0;
	int status = 0;

	if (enable) {
		REG_OR(MSM_AXI_RESET, 0x20);
		REG_OR(MSM_ROW_RESET, 0x20000);
		REG_SET(MSM_VDD_GRP_GFS_CTL, 0x11f);
		mdelay(20);	// very rough delay

		REG_AND(MSM_GRP_NS_REG, 0x7f);
		REG_OR(MSM_GRP_NS_REG, grp_ns);
		REG_OR(MSM_GRP_NS_REG, 0x800);
		REG_OR(MSM_GRP_NS_REG, 0x80);
		REG_OR(MSM_GRP_NS_REG, 0x200);

		REG_OR(MSM_CLK_CTL_BASE, 0x8);	// grp idx

		REG_AND(MSM_RAIL_CLAMP_IO, 0x4);

		REG_AND(MSM_AXI_BASE + 0x10080, 0x1);

		REG_AND(MSM_AXI_RESET, 0x20);
		REG_AND(MSM_ROW_RESET, 0x20000);
	} else {
		REG_OR(MSM_GRP_NS_REG, 0x800);
		REG_OR(MSM_GRP_NS_REG, 0x80);
		REG_OR(MSM_GRP_NS_REG, 0x200);

		REG_OR(MSM_CLK_CTL_BASE, 0x8);	// grp idx

		REG_OR(MSM_AXI_BASE + 0x10080, 0x1);

		while (status == 0 && i < 100) {
			i++;
			status = readl(MSM_AXI_BASE + 0x10084) & 0x1;
		}

		REG_OR(MSM_AXI_RESET, 0x20);
		REG_OR(MSM_ROW_RESET, 0x20000);

		REG_AND(MSM_GRP_NS_REG, 0x800);
		REG_AND(MSM_GRP_NS_REG, 0x80);
		REG_AND(MSM_GRP_NS_REG, 0x200);

		REG_OR(MSM_RAIL_CLAMP_IO, 0x4);	// grp clk ramp

		REG_SET(MSM_VDD_GRP_GFS_CTL, 0x1f);
	}
}

#if 0
static int is_vfe_on(void)
{
	int rc = 0;

	if (!(readl(MSM_VFE_NS_REG) & 0x4000)) {
		if (!(readl(MSM_CLK_HALT_STATEA) & 0x200)) {
			rc = 1;
		} else {
			rc = !(readl(MSM_VFE_NS_REG) & 0x800);
		}
	} else {
		rc = !(readl(MSM_VFE_NS_REG) & 0x800);
	}

	return rc;
}
#endif

static void set_vdc_rail(int on)
{
	int status, i;
	static int state = 0;
	if (state == on)
		return;
	state = on;

	printk("+%s(%d)\n", __func__, on);
	if (state) {
		REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x800);
		if (readl(MSM_CLK_CTL_BASE + 0xF0) & 0x60) {
			REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x100);
		}
		REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x200);

		REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x34, 0x7FFFFF, 0x100000);
		REG_OR(MSM_CLK_CTL_BASE, 0x8);
		REG_ANDM_OR(MSM_AXI_RESET, 0x3FFF, 0x2);
		REG_ANDM_OR(MSM_ROW_RESET, 0x7FFFFFF, 0x1);
		REG_ANDM_OR(MSM_APPS_RESET, 0x1FFF, 0x80);

		REG_SET(MSM_VDD_VDC_GFS_CTL, 0x11f);
		mdelay(2);

		REG_AND(MSM_RAIL_CLAMP_IO, 0x2);

		REG_AND(MSM_AXI_BASE + 0x10080, 0x4);

		REG_AND(MSM_AXI_RESET, 0x2);
		REG_AND(MSM_ROW_RESET, 1);
		REG_AND(MSM_APPS_RESET, 0x80);
	} else {
		REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x800);
		if (readl(MSM_CLK_CTL_BASE + 0xF0) & 0x60) {
			REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x100);
		}
		REG_OR(MSM_CLK_CTL_BASE + 0xF0, 0x200);
		REG_OR(MSM_CLK_CTL_BASE, 0x8);

		REG_OR(MSM_AXI_BASE + 0x10080, 0x4);

		i = status = 0;
		while (status == 0 && i < 100) {
			i++;
			status = readl(MSM_AXI_BASE + 0x10084) & 0x4;
		}

		REG_OR(MSM_AXI_RESET, 0x2);
		REG_OR(MSM_ROW_RESET, 0x1);

		REG_AND(MSM_AXI_BASE + 0xF0, 0x200);
		REG_AND(MSM_AXI_BASE + 0xF0, 0x100);
		REG_AND(MSM_AXI_BASE + 0xF0, 0x800);

		REG_OR(MSM_RAIL_CLAMP_IO, 0x2);

		REG_SET(MSM_VDD_VDC_GFS_CTL, 0x1f);
	}
	printk("-%s(%d)\n", __func__, on);
}

static void set_vfe_rail(int enable)
{
	int i = 0;
	int status = 0;
	static int is_enabled = 0;

	printk("+%s(%d)\n", __func__, enable);
	if (is_enabled == enable)
		return;

	is_enabled = enable;

	if (enable) {
		REG_ANDM_OR(MSM_AXI_RESET, 0x3FFF, 1);
		REG_ANDM_OR(MSM_CLK_CTL_BASE + 0x210, 0x1FFF, 1);
		REG_SET(MSM_VDD_VFE_GFS_CTL, 0x11F);
		mdelay(20);

		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 9);
		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 13);
		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 11);
		REG_AND(MSM_CLK_CTL_BASE + 0x44, (1 << 1 | 1 << 2));

		REG_AND_MASK(MSM_RAIL_CLAMP_IO, 0x37);

		REG_AND_MASK(MSM_AXI_BASE + 0x20080, 0xFE);

		REG_AND_MASK(MSM_AXI_RESET, 0x3FFE);
		REG_AND_MASK(MSM_APPS_RESET, 0x1FFE);
	} else {
		REG_ANDM_OR(MSM_CLK_CTL_BASE, 0x3FFFFFFF, 4);
		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 9);
		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 13);
		REG_OR(MSM_CLK_CTL_BASE + 0x44, 1 << 11);
		REG_AND(MSM_CLK_CTL_BASE + 0x44, (1 << 1 | 1 << 2));

		REG_OR(MSM_AXI_BASE + 0x20080, 0x1);
		while (status == 0 && i < 100) {
			i++;
			status = readl(MSM_AXI_BASE + 0x20084) & 0x1;
		}

		REG_ANDM_OR(MSM_AXI_RESET, 0x3FFF, 1);
		REG_ANDM_OR(MSM_APPS_RESET, 0x1FFF, 1);

		REG_OR(MSM_CLK_CTL_BASE + 0x44, (1 << 1 | 1 << 2));
		REG_AND(MSM_CLK_CTL_BASE + 0x44, 1 << 11);
		REG_AND(MSM_CLK_CTL_BASE + 0x44, 1 << 13);
		REG_AND(MSM_CLK_CTL_BASE + 0x44, 1 << 9);

		REG_ANDM_OR(MSM_RAIL_CLAMP_IO, 0x3F, 8);
		REG_SET(MSM_VDD_VFE_GFS_CTL, 0x1F);
	}
	printk("-%s(%d)\n", __func__, enable);
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

static int set_mdns_host_clock(unsigned id, unsigned freq)
{
	int n = 0;
	unsigned offset, ns_val;
	bool found, enabled, needs_setup;
	struct msm_clock_params params;
	found = false;

	params = msm_clk_get_params(id);
	offset = params.offset;
	needs_setup = params.setup_mdns;

	if (debug_mask & DEBUG_MDNS)
		D("%s(%u, %u); bitidx=%u, offset=%x\n", __func__, id, freq,
		  params.idx, params.offset);

	if (!params.offset) {
		D("%s: FIXME! Don't know how to set clock %u - no known Md/Ns reg\n", __func__, id);
		return 0;
		//return -ENOTSUPP;
	}

	enabled = a11_clk_is_enabled(id);
	if (enabled)
		a11_clk_disable(id);

	switch (id) {
		case MDP_CLK:
		case PMDH_CLK:
		case ADSP_CLK:
		case EMDH_CLK:
		case GP_CLK:
			ns_val = msm_gen_rate_simple_div_pll(id, freq);
			ns_val = (0x7f & ns_val) | (readl(MSM_CLK_CTL_BASE + offset) & ~0x7f);
			writel(ns_val, MSM_CLK_CTL_BASE + offset);
			goto skip_clock;
		case GRP_3D_CLK:
			grp_ns = msm_gen_rate_simple_div_pll(id, freq);
			goto skip_clock;

		default:
		break;
	}

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
					writel((msm_clock_freq_parameters[n].ns
						& ~0x6a06) |
					       (readl(MSM_CLK_CTL_BASE + offset)
						& 0x6a06)
					       , MSM_CLK_CTL_BASE + offset);
		} else {
			writel(msm_clock_freq_parameters[n].ns,
			       MSM_CLK_CTL_BASE + offset);
		}
		if (debug_mask & DEBUG_MDNS)
			D("%s: %u, freq=%lu calc_freq=%u pll%d=%u expected pll =%u\n",
				__func__, id, msm_clock_freq_parameters[n].freq,
				msm_clock_freq_parameters[n].calc_freq,
				msm_clock_freq_parameters[n].ns & 7,
				pll_get_rate(msm_clock_freq_parameters[n].ns & 7, false),
				msm_clock_freq_parameters[n].pll_freq);

		found = true;
		break;
	}

 skip_clock:
	if (enabled)
		a11_clk_enable(id);

	if ((!found && needs_setup) && (debug_mask & DEBUG_UNKNOWN_FREQ)) {
		D("FIXME! set_sdcc_host_clock could not "
		  "find suitable parameter for freq %u\n", freq);
	}

	return 0;
}

static unsigned get_mdns_host_clock(unsigned id)
{
	int n;
	unsigned offset;
	unsigned mdreg;
	unsigned nsreg;
	unsigned freq = 0;

	offset = msm_clk_reg_offset(id);
	if (!offset) {
		D("%s: do not know how to get clock rate %d\n", __func__, id);
		return 0;
	}

	switch (id) {
		case MDP_CLK:
		case PMDH_CLK:
		case EBI1_CLK:
		case EBI2_CLK:
		case ADSP_CLK:
		case EMDH_CLK:
		case GP_CLK:
		case GRP_3D_CLK:
			nsreg = readl(MSM_CLK_CTL_BASE + offset);
			return msm_decode_rate_simple_div_pll(nsreg);

		default:
		break;
	}

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
	case USB_OTG_CLK:
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
		set_grp_rail(1);
		done = 1;
		break;

	case MDC_CLK:
		printk("Enabling MDC clock\n");
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) | 0x200,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case VFE_CLK:
		set_vfe_rail(1);
		done = 1;
		break;

	case VFE_MDC_CLK:
		done = 1;
		break;

	case VDC_CLK:
		set_vdc_rail(1);
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
		return;
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
	case USB_OTG_CLK:
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
		set_grp_rail(0);
		writel(readl(MSM_CLK_CTL_BASE) & ~(1U << params.idx),
		       MSM_CLK_CTL_BASE);
		done = 1;
		break;

	case MDC_CLK:
		D("disabling MDC clock\n");
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x200,
		       MSM_CLK_CTL_BASE + params.offset);
		writel(readl(MSM_CLK_CTL_BASE + params.offset) & ~0x800,
		       MSM_CLK_CTL_BASE + params.offset);
		done = 1;
		break;

	case VFE_CLK:
		set_vfe_rail(0);
		done = 1;
		break;

	case VFE_MDC_CLK:
		done = 1;
		break;

	case VDC_CLK:
		set_vdc_rail(0);
		done = 1;
		break;


	default:
		break;
	}

	if (done)
		return;

	if (debug_mask & DEBUG_UNKNOWN_ID)
		D("%s: FIXME! disabling a clock that doesn't have an "
		  "ena bit: %u\n", __func__, id);
	return;
}

static int a11_clk_set_rate(unsigned id, unsigned rate)
{
	int retval;
	retval = 0;

	if (DEBUG_MDNS)
		D("%s: id=%u rate=%u\n", __func__, id, rate);

	retval = set_mdns_host_clock(id, rate);

	return retval;
}

static int a11_clk_set_min_rate(unsigned id, unsigned rate)
{
	if (id < NR_CLKS)
		min_clk_rate[id] = rate;
	else if (debug_mask & DEBUG_UNKNOWN_ID)
		D(" FIXME! clk_set_min_rate not implemented; %u:%u NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static int a11_clk_set_max_rate(unsigned id, unsigned rate)
{
	if (id < NR_CLKS)
		max_clk_rate[id] = rate;
	else if (debug_mask & DEBUG_UNKNOWN_ID)
		D(" FIXME! clk_set_min_rate not implemented; %u:%u NR_CLKS=%d\n", id, rate, NR_CLKS);

	return 0;
}

static unsigned a11_clk_get_rate(unsigned id)
{
	switch (id) {
	case SDC1_P_CLK:
	case SDC2_P_CLK:
	case SDC3_P_CLK:
	case SDC4_P_CLK:
		return 64000000;
	}

	return get_mdns_host_clock(id);;
}

static int a11_clk_set_flags(unsigned id, unsigned flags)
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
		D("%s not implemented for clock: id=%u, flags=%u\n",
		  __func__, id, flags);
	return 0;
}

static unsigned a11_clk_is_enabled(unsigned id)
{
	unsigned is_enabled = 0;
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

	mutex_lock(&clocks_mutex);

	if (pll_get_rate(PLL0, true) == 196608000) {
		// cdma pll0 = 196 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_196;
	} else {
		// default gsm pll0 = 245 MHz
		msm_clock_freq_parameters = msm_clock_freq_parameters_pll0_245;
	}
	mutex_unlock(&clocks_mutex);
}

static void msm_clock_a11_reset_imem(void)
{
	if (!a11_clk_is_enabled(IMEM_CLK))
		return;

	writel(0, MSM_IMEM_BASE);
	pr_info("IMEM OLD: VAL = %d\n", readl(MSM_IMEM_BASE));
	msleep(100);
	pr_info("IMEM NEW: VAL = %d\n", readl(MSM_IMEM_BASE));
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
