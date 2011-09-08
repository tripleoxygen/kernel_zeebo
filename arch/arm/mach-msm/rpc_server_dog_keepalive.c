/* arch/arm/mach-msm/rpc_server_dog_keepalive.c
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

static uint32_t dog_prog;
static uint32_t dog_vers;
static uint32_t dog_beacon;
static const uint32_t dog_null = 0;

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	if (req->procedure == dog_null)
		return 0;
	if (req->procedure == dog_beacon) {
		printk(KERN_INFO "DOG KEEPALIVE PING\n");
		return 0;
	}
	return -ENODEV;
}

static struct msm_rpc_server rpc_server;

static int __init rpc_server_init(void)
{
	if (!amss_get_num_value(AMSS_RPC_DOG_KEEPALIVE_PROG, &dog_prog)) {
		printk(KERN_ERR "%s: failed to get AMSS_RPC_DOG_KEEPALIVE_PROG\n",
				__func__);
		return -1;
	}

	if (!amss_get_num_value(AMSS_RPC_DOG_KEEPALIVE_VERS, &dog_vers)) {
		printk(KERN_ERR "%s: failed to get AMSS_RPC_DOG_KEEPALIVE_VERS\n",
				__func__);
		return -1;
	}

	if (!amss_get_num_value(AMSS_RPC_DOG_KEEPALIVE_BEACON, &dog_beacon)) {
		printk(KERN_ERR "%s: failed to get AMSS_RPC_DOG_KEEPALIVE_BEACON\n",
				__func__);
		return -1;
	}
	/* Dual server registration to support backwards compatibility vers */
	rpc_server.prog = dog_prog;
	rpc_server.vers = dog_vers;
	rpc_server.rpc_call = handle_rpc_call;
	return msm_rpc_create_server(&rpc_server);
}

module_init(rpc_server_init);
