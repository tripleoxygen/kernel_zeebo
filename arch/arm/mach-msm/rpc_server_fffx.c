/* arch/arm/mach-msm/rpc_server_fffx.c
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

//#define FS_RAPI_PROG 				0x30000011
//#define	FS_RAPI_VERS				0
//#define RPC_FS_RAPI_NULL_PROC			0
//#define RPC_FS_RAPI_MOUNT_PROC			1
//#define RPC_FS_RAPI_GETSTAT_VFS_PROC		2
//#define RPC_FS_RAPI_GETSTAT_PROC		3
//#define RPC_FS_RAPI_LOOKUP_PROC			4
//#define RPC_FS_RAPI_CREATE_PROC			5
//#define RPC_FS_RAPI_READ_PROC			6
//#define RPC_FS_RAPI_WRITE_PROC			7
//#define RPC_FS_RAPI_MKDIR_PROC			8
//#define RPC_FS_RAPI_RMDIR_PROC			9
//#define RPC_FS_RAPI_UNLINK_PROC			10
//#define RPC_FS_RAPI_RENAME_PROC			11
//#define RPC_FS_RAPI_TRUNCATE_PROC		12
//#define RPC_FS_RAPI_SYMLINK_PROC		13
//#define RPC_FS_RAPI_READLINK_PROC		14
//#define RPC_FS_RAPI_READDIR_PROC		15
//#define RPC_FS_RAPI_CLOSEDIR_PROC		16
//#define RPC_FS_RAPI_MKNOD_PROC			17
//#define RPC_FS_RAPI_CHMOD_PROC			18
//#define RPC_FS_RAPI_CHOWN_PROC			19

//struct rpc_fs_rapi_mount_args {
	//uint32_t unk0;
	//uint32_t unk1;
	//uint32_t unk2;
	//uint32_t unk3;
//};

//struct rpc_fs_rapi_lookup_args_hdr {
	//uint32_t unk0;
	//uint32_t unk1;
	//uint32_t unk2;
	//uint32_t unk3;
	//uint32_t unk4;
	//uint32_t unk5;
	//uint32_t size;
//};

//struct rpc_fs_rapi_open_args_ftr {
	//uint32_t unk0;
	//uint32_t unk1;
	//uint32_t unk2;
//};

//struct rpc_fs_rapi_api_params {
	//unsigned req_size;
	//char *name;
//};

//static struct rpc_fs_rapi_api_params rpc_fs_rapi_apis[] = {
	//[RPC_FS_RAPI_NULL_PROC] = { .req_size = 0, .name = "FS_RAPI_NULL" },
	//[RPC_FS_RAPI_MOUNT_PROC] = { .req_size = 0, .name = "FS_RAPI_MOUNT" },
	//[RPC_FS_RAPI_GETSTAT_VFS_PROC] = { .req_size = 0, .name = "FS_RAPI_GETSTAT_VFS" },
	//[RPC_FS_RAPI_GETSTAT_PROC] = { .req_size = 0, .name = "FS_RAPI_GETSTAT" },
	//[RPC_FS_RAPI_LOOKUP_PROC] = { .req_size = 0, .name = "FS_RAPI_LOOKUP" },
	//[RPC_FS_RAPI_CREATE_PROC] = { .req_size = 0, .name = "FS_RAPI_CREATE" },
	//[RPC_FS_RAPI_READ_PROC] = { .req_size = 0, .name = "FS_RAPI_READ" },
	//[RPC_FS_RAPI_WRITE_PROC] = { .req_size = 0, .name = "FS_RAPI_WRITE" },
	//[RPC_FS_RAPI_MKDIR_PROC] = { .req_size = 0, .name = "FS_RAPI_MKDIR" },
	//[RPC_FS_RAPI_RMDIR_PROC] = { .req_size = 0, .name = "FS_RAPI_RMDIR" },
	//[RPC_FS_RAPI_UNLINK_PROC] = { .req_size = 0, .name = "FS_RAPI_UNLINK" },
	//[RPC_FS_RAPI_RENAME_PROC] = { .req_size = 0, .name = "FS_RAPI_RENAME" },
	//[RPC_FS_RAPI_TRUNCATE_PROC] = { .req_size = 0, .name = "FS_RAPI_TRUNCATE" },
	//[RPC_FS_RAPI_SYMLINK_PROC] = { .req_size = 0, .name = "FS_RAPI_SYMLINK" },
	//[RPC_FS_RAPI_READLINK_PROC] = { .req_size = 0, .name = "FS_RAPI_READLINK" },
	//[RPC_FS_RAPI_READDIR_PROC] = { .req_size = 0, .name = "FS_RAPI_READDIR" },
	//[RPC_FS_RAPI_CLOSEDIR_PROC] = { .req_size = 0, .name = "FS_RAPI_CLOSEDIR" },
	//[RPC_FS_RAPI_MKNOD_PROC] = { .req_size = 0, .name = "FS_RAPI_MKNOD" },
	//[RPC_FS_RAPI_CHMOD_PROC] = { .req_size = 0, .name = "FS_RAPI_CHMOD" },
	//[RPC_FS_RAPI_CHOWN_PROC] = { .req_size = 0, .name = "FS_RAPI_CHOWN" },
//}; 

//uint8_t reply_mount[] = {
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x60
//};

///* lookup request from AMSS
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x60,
 //* 0x00, 0x00, 0x00, 0x11, 0x55, 0x69, 0x6d, 0x45,
 //* 0x66, 0x73, 0x41, 0x50, 0x44, 0x55, 0x4c, 0x6f,
 //* 0x67, 0x2e, 0x54, 0x78, 0x74, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x01,
 //*
 //* lookup "err" (dir?)
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x60,
 //* 0x00, 0x00, 0x00, 0x03, 0x65, 0x72, 0x72, 0x00,
 //* 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x01,
 //*
 //* reply to lookup "err"
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x01, 0x8f, 0x80, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8f, 0x80,
 //* 0x00, 0x00, 0x41, 0xed, 0x00, 0x00, 0x00, 0x02,
 //* 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x08, 0x00,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 //* 0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x00, 
 //*/

//uint8_t reply_lookup[] = {
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x73, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73,
	//0x00, 0x00, 0x81, 0x80, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00,
//};




///* after unlink */
//uint8_t reply_lookup2[] = {
	//0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73,
	//0x00, 0x00, 0x81, 0x80, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00,
//};

///* create request
 //* 00 00 00 01 00 00 00 00
 //* 00 00 00 02 00 00 00 00
 //* 00 00 00 01 00 00 00 60
 //* 00 00 00 11 55 69 6D 45
 //* 66 73 41 50 44 55 4C 6F
 //* 67 2E 54 78 74 00 00 00
 //* 00 00 00 11 00 00 01 80
 //* 00 00 00 01 00 00 00 01
 //*/
//uint8_t reply_create[] = {
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x73, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73,
	//0x00, 0x00, 0x81, 0x80, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00, 
//};

///* unlink request
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x60,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x73,
 //* 0x00, 0x00, 0x00, 0x11, 0x55, 0x69, 0x6d, 0x45,
 //* 0x66, 0x73, 0x41, 0x50, 0x44, 0x55, 0x4c, 0x6f,
 //* 0x67, 0x2e, 0x54, 0x78, 0x74, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x11,
 //*/

//uint8_t reply_generic_success[] = {
	//0x00, 0x00, 0x00, 0x00,
//};

///* getstat request
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x8f, 0x80,
 //* 0x00, 0x00, 0x00, 0x01,
 //*
 //* reply to request above
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8f, 0x80,
 //* 0x00, 0x00, 0x41, 0xed, 0x00, 0x00, 0x00, 0x02,
 //* 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x08, 0x00,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 //* 0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
 //* 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 //* 0x00, 0x00, 0x00, 0x00, 
 //*/

//uint8_t reply_getstat[] = {
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8f, 0x80,
	//0x00, 0x00, 0x41, 0xed, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00, 
//};

///* read request
 //* 00 00 00 01 00 00 00 00
 //* 00 00 00 02 00 00 00 00
 //* 00 00 00 01 00 01 8F 82
 //* 00 00 00 00 00 00 00 01
 //* 00 00 00 02 00 00 00 02
 //* 00 00 00 01
 //*/

//uint8_t reply_read[] = {
	//0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
	//0x30, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8f, 0x82,
	//0x00, 0x00, 0x81, 0xed, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00, 
//};

///* write request
 //* 00 00 00 01 00 00 00 00
 //* 00 00 00 02 00 00 00 00
 //* 00 00 00 01 00 01 8F 82
 //* 00 00 00 00 00 00 00 02
 //* 30 32 00 00 00 00 00 02	-> 30 32 -> "02"
 //* 00 00 00 01
 //*/
//uint8_t reply_write[] = {
	//0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8f, 0x82,
	//0x00, 0x00, 0x81, 0xed, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	//0x12, 0xd5, 0x3d, 0x80, 0x12, 0xd5, 0x3d, 0x80,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00, 
//};

//uint8_t reply_mkdir[] = {
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x04, 0x20, 0x00, 0x00, 0x00, 0x01,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x20,
	//0x00, 0x00, 0x41, 0xc0, 0x00, 0x00, 0x00, 0x02,
	//0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x4e, 0x39, 0xbb, 0xf5, 0x4e, 0x39, 0xbb, 0xf5,
	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	//0x00, 0x00, 0x00, 0x00,
//};

static void fs_rapi_dump_request(uint32_t *req, unsigned len, unsigned is_req)
{
	int i;

	if(len % 4) {
		printk("[fffx] len is not multiple of 4. len=%d\n", len);
		return;
	}

	printk("[fffx] ---------------- %s dump ----------------\n", (is_req?"request":"reply"));
	printk("[fffx] ");

	for(i = 0; i < (len / 4); i++) {
		printk("0x%08x ", be32_to_cpu(req[i]));

		if(!((i + 1) % 4))
			printk("\n[fffx] ");
	}
	printk("\n[fffx] ---------------- %s dump ----------------\n", (is_req?"request":"reply"));
}

//static int fs_rapi_mount(const char *path)
//{
	//return 0;
//}

//static int _fs_rapi_mount(void *msg, unsigned len)
//{
	//struct rpc_fs_rapi_mount_args *args;
	//args = (struct rpc_fs_rapi_mount_args *)msg;

	//return fs_rapi_mount(0);
//}

//struct fs_rapi_vfs {
	//char *name;
//};

//static unsigned filecount = 0;

//#define MAX_VFS_FILES	20
//#define MAX_FILENAME_SIZE	64

//struct fs_rapi_vfs vfs[MAX_VFS_FILES];

//static int fs_rapi_lookup(void *msg, unsigned len)
//{
	//char *file = ((uint8_t *)msg) + 0x1c;
	//int i;
	
	//pr_info("[FS_RAPI] Lookup file '%s'\n", file);

	//for(i = 0; i < MAX_VFS_FILES; i++) {
		//if(!vfs[i].name)
			//continue;

		//if(!strcmp(file, vfs[i].name))
			//return 1;
	//}

	//return 0;
//}

//static int fs_rapi_unlink(void *msg, unsigned len)
//{
	//char *file = ((uint8_t *)msg) + 0x34;
	//int i;
	
	//pr_info("[FS_RAPI] Unlink file '%s'\n", file);

	//if(!filecount)
		//return 0;

	//for(i = 0; i < MAX_VFS_FILES; i++) {
		//if(!vfs[i].name)
			//continue;

		//if(!strcmp(file, vfs[i].name)) {
			//kfree(vfs[i].name);
			//vfs[i].name = NULL;

			//pr_info("[FS_RAPI] Unlinked file '%s' at pos %d\n", file, i);

			//filecount--;
		//}
	//}

	//return 0;
//}


//static int fs_rapi_create_impl(const char *n)
//{
	//int i;

	//if(filecount >= MAX_VFS_FILES) {
		//pr_info("[FS_RAPI] Max VFS file count reached.\n");
		//return -ENOMEM;
	//}
	
	//for(i = 0; i < MAX_VFS_FILES; i++) {
		//if(vfs[i].name)
			//continue;
			
		//vfs[i].name = kmalloc(MAX_FILENAME_SIZE, GFP_KERNEL);

		//if(!vfs[i].name) {
			//pr_info("[FS_RAPI] Failed to create file '%s' at pos %d\n",
				//n, i);
			//break;
		//}

		//memset(vfs[i].name, '\0', MAX_FILENAME_SIZE);
		//memcpy(vfs[i].name, n, strlen(n));
		//pr_info("[FS_RAPI] Created file '%s' at pos %d\n", n, i);
		//filecount++;
		
		//return 0;
	//}

	//return -ENOMEM;
//}

//static int fs_rapi_create(void *msg, unsigned len)
//{
	//char *file = ((uint8_t *)msg) + 0x1c;
	
	//pr_info("[FS_RAPI] Create file '%s'\n", file);

	//return fs_rapi_create_impl(file);
//}

//static int fs_rapi_mkdir(void *msg, unsigned len)
//{
	//char *file = ((uint8_t *)msg) + 0x1c;
	
	//pr_info("[FS_RAPI] Create dir '%s'\n", file);

	//return fs_rapi_create_impl(file);
//}

static int handle_rpc_call_misc_apps(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len,
			   void **reply, unsigned *reply_len)
{
	void *msg;
	unsigned msg_len;
	int rc;

	msg = (void *)(req + 1);
	msg_len = len - sizeof(struct rpc_request_hdr);
	
	printk("[%s] called, proc=%d, total_len=%d, msg_len=%d\n",
		__func__, req->procedure, len, msg_len);
	
	fs_rapi_dump_request((uint32_t *)msg, msg_len, 1);

	printk(KERN_WARNING "[%s] unhandled rpc call procedure=%08x\n",
		__func__, req->procedure);
	rc = 0;

	return rc;
}

static int handle_rpc_call_keypad(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len,
			   void **reply, unsigned *reply_len)
{
	void *msg;
	unsigned msg_len;
	int rc;

	msg = (void *)(req + 1);
	msg_len = len - sizeof(struct rpc_request_hdr);
	
	printk("[%s] called, proc=%d, total_len=%d, msg_len=%d\n",
		__func__, req->procedure, len, msg_len);
	
	fs_rapi_dump_request((uint32_t *)msg, msg_len, 1);

	printk(KERN_WARNING "[%s] unhandled rpc call procedure=%08x\n",
		__func__, req->procedure);
	rc = 0;

	return rc;
}

static int handle_rpc_call_fffe(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len,
			   void **reply, unsigned *reply_len)
{
	void *msg;
	unsigned msg_len;
	int rc;

	msg = (void *)(req + 1);
	msg_len = len - sizeof(struct rpc_request_hdr);
	
	printk("[%s] called, proc=%d, total_len=%d, msg_len=%d\n",
		__func__, req->procedure, len, msg_len);
	
	fs_rapi_dump_request((uint32_t *)msg, msg_len, 1);

	printk(KERN_WARNING "[%s] unhandled rpc call procedure=%08x\n",
		__func__, req->procedure);
	rc = 0;

	return rc;
}

static struct msm_rpc_server rpc_server_fffe = {
	.prog = 0x3000fffe,
	.vers = 0,
	.rpc_call = handle_rpc_call_fffe,
};

static struct msm_rpc_server rpc_server_keypad = {
	.prog = 0x30000062,
	.vers = 0,
	.rpc_call = handle_rpc_call_keypad,
};

static struct msm_rpc_server rpc_server_misc_apps = {
	.prog = 0x30000006,
	.vers = 0,
	.rpc_call = handle_rpc_call_misc_apps,
};

static int __init rpc_server_init(void)
{
	int rc = 0;

	//rc = msm_rpc_create_server(&rpc_server_keypad);
	rc = msm_rpc_create_server(&rpc_server_misc_apps);
	//rc = msm_rpc_create_server(&rpc_server_fffe);
	
	return rc;
}

/* request to: 0x30000064:6
 * send 00 00 00 10 33 35 35 38 30 30 30 32 30 31 35 39 33 33 35 00
 * ...proc 5
 * send 00 00 90 02
 * ... proc 3
 * send void
 * ... proc 9
 * send void x 2
 * ...proc 2
 * send void
 */


module_init(rpc_server_init);
