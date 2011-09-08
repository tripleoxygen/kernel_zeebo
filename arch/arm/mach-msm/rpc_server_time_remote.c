/* arch/arm/mach-msm/rpc_server_time_remote.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Iliyan Malchev <ibm@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>

/* time_remote_mtoa server definitions. */

#define TIME_REMOTE_MTOA_PROG 0x3000005d
#define RPC_TIME_REMOTE_MTOA_NULL   0

static uint32_t tod_app_bases;

struct rpc_time_tod_set_apps_bases_args {
	uint32_t tick;
	uint64_t stamp;
};

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	if (req->procedure == RPC_TIME_REMOTE_MTOA_NULL)
		return 0;

	if (req->procedure == tod_app_bases) {
		struct rpc_time_tod_set_apps_bases_args *args;
		args = (struct rpc_time_tod_set_apps_bases_args *)(req + 1);
		args->tick = be32_to_cpu(args->tick);
		args->stamp = be64_to_cpu(args->stamp);
		printk(KERN_INFO "RPC_TIME_TOD_SET_APPS_BASES:\n"
		       "\ttick = %d\n"
		       "\tstamp = %lld\n",
		       args->tick, args->stamp);
		return 0;
	}

	printk(KERN_WARNING "[TIME]: %s unhandled rpc call procedure=%08x\n", __func__, req->procedure);
	return -ENODEV;
}

static struct msm_rpc_server rpc_server = {
	.prog = TIME_REMOTE_MTOA_PROG,
	.vers = 0,
	.rpc_call = handle_rpc_call,
};

static int __init rpc_server_init(void)
{
	uint32_t time_vers;
	if (!amss_get_num_value(AMSS_TIME_REMOTE_MTOA_VERS, &time_vers)) {
		printk(KERN_ERR "%s: failed to get AMSS_TIME_REMOTE_MTOA_VERS\n",
				__func__);
		return -1;
	}
	if (!amss_get_num_value(AMSS_TIME_TOD_SET_APPS_BASES, &tod_app_bases)) {
		printk(KERN_ERR "%s: failed to get AMSS_TIME_TOD_SET_APPS_BASES\n",
				__func__);
		return -1;
	}
	rpc_server.vers = time_vers;
	/* Dual server registration to support backwards compatibility vers */
	return msm_rpc_create_server(&rpc_server);
}


module_init(rpc_server_init);
