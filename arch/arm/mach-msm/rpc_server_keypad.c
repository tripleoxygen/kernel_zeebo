/* arch/arm/mach-msm/rpc_server_keypad.c
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
#include <linux/slab.h>

#define KEYPAD_PROG 				0x30000062
#define	KEYPAD_VERS				0

#define RPC_KEYPAD_NULL_PROC			0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC		1
#define RPC_KEYPAD_SET_PWRKEY_STATE_PROC	2

struct rpc_keypad_api_params {
	unsigned req_size;
	char *name;
};

static struct rpc_keypad_api_params rpc_keypad_apis[] = {
	[RPC_KEYPAD_NULL_PROC] = { .req_size = 0, .name = "KEYPAD_NULL" },
	[RPC_KEYPAD_PASS_KEY_CODE_PROC] = { .req_size = 0, .name = "KEYPAD_PASS_KEY_CODE" },
	[RPC_KEYPAD_SET_PWRKEY_STATE_PROC] = { .req_size = 0, .name = "KEYPAD_SET_PWRKEY_STATE" },
}; 

/* RPC_KEYPAD_PASS_KEY_CODE_PROC request
 * 00 00 00 82 00 00 00 00
 *
 * reply
 * ok, void
 *
 * request
 * 00 00 00 FF 00 00 00 82
 *
 * reply
 * 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
 * 0x00, 0x00, 0x15, 0xdc, 0x00, 0x00, 0x00, 0x19,
 * 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x19, 
 *
 * request
 * 00 00 00 9C 00 00 00 00 
 *
 * reply
 * ok, void
 */

uint8_t reply_pass_keycode[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x15, 0xdc, 0x00, 0x00, 0x00, 0x19,
	0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x19, 
};

static void keypad_dump_request(uint32_t *req, unsigned len, unsigned is_req)
{
	int i;

	if(len % 4) {
		printk("[KEYPAD] len is not multiple of 4. len=%d\n", len);
		return;
	}

	printk("[KEYPAD] ---------------- %s dump ----------------\n", (is_req?"request":"reply"));
	printk("[KEYPAD] ");

	for(i = 0; i < (len / 4); i++) {
		printk("0x%08x ", be32_to_cpu(req[i]));

		if(!((i + 1) % 4))
			printk("\n[KEYPAD] ");
	}
	printk("\n[KEYPAD] ---------------- %s dump ----------------\n", (is_req?"request":"reply"));
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
	*reply = NULL;
	printk("[%s] KEYPAD called, proc=%d [%s], total_len=%d, msg_len=%d\n",
		__func__, req->procedure, rpc_keypad_apis[req->procedure].name, len, msg_len);
	
	//keypad_dump_request((uint32_t *)msg, msg_len, 1);

	switch(req->procedure) {
	case RPC_KEYPAD_PASS_KEY_CODE_PROC:
		if(((uint8_t *)msg)[3] == 0xff) {
			*reply_len = sizeof(reply_pass_keycode);
			*reply = reply_pass_keycode;
			rc = 1;
		} else {
			rc = 0;
		}
		break;
	default:
		printk(KERN_WARNING "[KEYPAD]: %s unhandled rpc call procedure=%08x\n",
			__func__, req->procedure);
		rc = -ENODEV;
		break;
	}

	if(*reply) {
		//keypad_dump_request(*reply, *reply_len, 0);
		printk("[KEYPAD] reply_data=%p, reply_len=%d\n", *reply, *reply_len);
	}

	return rc;
}

static struct msm_rpc_server rpc_server = {
	.prog = KEYPAD_PROG,
	.vers = KEYPAD_VERS,
	.rpc_call = handle_rpc_call,
};

static int __init rpc_server_init(void)
{
	return msm_rpc_create_server(&rpc_server);
}

module_init(rpc_server_init);
