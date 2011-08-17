#ifndef __AMSS_FALLBACK_H__
#define __AMSS_FALLBACK_H__

#include <mach/msm_smd.h>

static struct amss_value amss_fallback_params[] = {
	{AMSS_RPC_DOG_KEEPALIVE_PROG, AMSS_VAL_UINT, {.value = 0x30000015}},
	{AMSS_RPC_DOG_KEEPALIVE_VERS, AMSS_VAL_UINT, {.value = 0}},
	{AMSS_RPC_DOG_KEEPALIVE_BEACON, AMSS_VAL_UINT, {.value = 1}},
	{AMSS_TIME_REMOTE_MTOA_VERS, AMSS_VAL_UINT, {.value = 0}},
	{AMSS_TIME_TOD_SET_APPS_BASES, AMSS_VAL_UINT, {.value = 2}},
};

static int n_amss_fallback_params = ARRAY_SIZE(amss_fallback_params);
#endif
