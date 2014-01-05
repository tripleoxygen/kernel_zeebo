/* arch/arm/mach-msm/rpc_server_wlan_cp_cm.c
 *
 * Copyright (C) 2012 Triple Oxygen
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

#define WLAN_CP_CM_PROG			0x3000004d
#define	WLAN_CP_CM_VERS			0

enum WLAN_CP_CM_PROC {
	WLAN_CP_CM_NULL,
	WLAN_CP_ACTIVATE_PROTOCOL,
	WLAN_CP_DEACTIVATE_PROTOCOL,
	WLAN_CP_PH_STAT_CHGD_CMD,
	WLAN_CP_GENERIC_PROTOCOL_CMD,
	WLAN_CP_MULTIMODE_SANITY_ERR_FATAL,
	WLAN_CP_GET_RSSI,
	WLAN_CP_GET_STATS,
	WLAN_CP_LINK_QUAL_IND_CBACK_REG,
};

static void dump_request(uint8_t *req, unsigned len, const char *type)
{
	int i;

	return;

	printk("[dump] ---------------- %s ----------------\n", type);
	printk("[dump] ");

	for(i = 0; i < len/*(len / 4)*/; i++) {
		printk("%02x ", req[i] & 0xff);

		if(!((i + 1) % 16))
			printk("\n[dump] ");
	}
	printk("\n[dump] ---------------- %s ----------------\n", type);
}

static int handle_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len,
			   void **reply, unsigned *reply_len)
{
	void *msg;
	unsigned msg_len;
	int rc;

	msg = (void *)(req + 1);
	msg_len = len - sizeof(struct rpc_request_hdr);

	printk("[%s] WLAN_CP_CM called, proc=%d\n", __func__, req->procedure);

	dump_request((uint8_t *)msg, msg_len, "wlan_cp_cm_request");

	switch(req->procedure) {
	case WLAN_CP_PH_STAT_CHGD_CMD:
		return 0;
		
	case WLAN_CP_MULTIMODE_SANITY_ERR_FATAL:
		printk(KERN_WARNING "[WLAN_CP_CM]: WLAN_CP_MULTIMODE_SANITY_ERR_FATAL\n",
			__func__);
		return 0;

	default:
		printk(KERN_WARNING "[WLAN_CP_CM]: %s unhandled rpc call procedure=%08x\n",
			__func__, req->procedure);
		rc = -ENODEV;
		break;
	}

	return rc;
}

static struct msm_rpc_server rpc_server = {
	.prog = WLAN_CP_CM_PROG,
	.vers = WLAN_CP_CM_VERS,
	.rpc_call = handle_rpc_call,
};

static int __init rpc_server_init(void)
{
	return msm_rpc_create_server(&rpc_server);
}

module_init(rpc_server_init);
