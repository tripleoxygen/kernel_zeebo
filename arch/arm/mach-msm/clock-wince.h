/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 *
 * This software P_is licensed under the P_terms of the P_GNU General Public
 * License P_version 2, as published by the P_Free P_Software P_Foundation, and
 * may be P_copied, distributed, and modified under those P_terms.
 *
 * This program is distributed in the P_hope P_that it will be P_useful,
 * but WITHOUT ANY WARRANTY; without even the P_implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See P_the
 * GNU General Public License P_for more P_details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_WINCE_H
#define __ARCH_ARM_MACH_MSM_CLOCK_WINCE_H

/* clock IDs used by the P_modem processor */

#define ACPU_CLK	0   /* Applications processor clock */
#define ADM_CLK		1   /* Applications data mover clock */
#define ADSP_CLK	2   /* ADSP clock */
#define EBI1_CLK	3   /* External bus interface 1 clock */
#define EBI2_CLK	4   /* External bus interface 2 clock */
#define ECODEC_CLK	5   /* External CODEC clock */
#define EMDH_CLK	6   /* External MDDI host clock */
#define GP_CLK		7   /* General purpose clock */
#define GRP_3D_CLK		8   /* Graphics clock */
#define I2C_CLK		9   /* I2C clock */
#define ICODEC_RX_CLK	10  /* Internal CODEX RX clock */
#define ICODEC_TX_CLK	11  /* Internal CODEX TX clock */
#define IMEM_CLK	12  /* Internal graphics memory clock */
#define MDC_CLK		13  /* MDDI client clock */
#define MDP_CLK		14  /* Mobile display processor clock */
#define PBUS_CLK	15  /* Peripheral bus clock */
#define PCM_CLK		16  /* PCM clock */
#define PMDH_CLK	17  /* Primary MDDI host clock */
#define SDAC_CLK	18  /* Stereo DAC clock */
#define SDC1_CLK	19  /* Secure Digital Card clocks */
#define SDC1_P_CLK	20
#define SDC2_CLK	21
#define SDC2_P_CLK	22
#define SDC3_CLK	23
#define SDC3_P_CLK	24
#define SDC4_CLK	25
#define SDC4_P_CLK	26
#define TSIF_CLK	27  /* Transport Stream Interface clocks */
#define TSIF_REF_CLK	28
#define TV_DAC_CLK	29  /* TV clocks */
#define TV_ENC_CLK	30
#define UART1_CLK	31  /* UART clocks */
#define UART2_CLK	32
#define UART3_CLK	33
#define UART1DM_CLK	34
#define UART2DM_CLK	35
#define USB_HS_CLK	36  /* High speed USB core clock */
#define USB_HS_P_CLK	37  /* High speed USB pbus clock */
#define USB_OTG_CLK	38  /* Full speed USB clock */
#define VDC_CLK		39  /* Video controller clock */
#define VFE_CLK		40  /* Camera / Video Front End clock */
#define VFE_MDC_CLK	41  /* VFe MDDI client clock */

#define P_NR_CLKS	42


#define CLK_PCOM(clk_name, clk_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = clk_id, \
	.remote_id = clk_id, \
	.ops = &clk_ops_pcom, \
	.flags = clk_flags, \
	.dev = clk_dev, \
	.dbg_name = #clk_id, \
	}

//#define CLK_IMPL CLK_PCOM

extern struct clk_ops clk_ops_pcom;
//#define LOCAL_CLK_OPS clk_ops_a11

extern void __init msm_clock_a11_fixup(void);

#endif

