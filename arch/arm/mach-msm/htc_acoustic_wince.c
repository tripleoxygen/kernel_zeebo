/* arch/arm/mach-msm/htc_acoustic_wince.c
 *
 * Author: Jbruneaux
 *
 * Description : provide interface between userland and kernel for the 
 * acoustic management
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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <mach/msm_smd.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include "proc_comm_wince.h"

#include "smd_private.h"

/* Userspace interraction error printout */
#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
				__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

/* structs */
struct msm_acoustic_capabilities {
    char htc_voc_cal_fields_per_param;    /* Specifies the number of fields per parameter */
    bool bDualMicSupported;
    bool bUseTPA2016;
    /* TODO : add specific devices support like a1010, amplifiers, ... */
};

struct msm_audio_path {
    bool bEnableMic;
    bool bEnableDualMic;
    bool bEnableSpeaker;
    bool bEnableHeadset;
};

struct adie_table {
    int table_num;
    char* pcArray;
};

struct htc_voc_cal_table {
    uint16_t* pArray;
    int       size;
};

enum {
    PCOM_UPDATE_REQ = 0,
    ADIE_FORCE8K_REQ,
    ADIE_FORCE_ADIE_AWAKE_REQ,
    ADIE_FORCE_ADIE_UPDATE_REQ,
    ADIE_UPDATE_AUDIO_METHOD,
    
} AUDIO_UPDATE_REQ_TYPE;

struct audio_update_req {
    int type;       /* one of the AUDIO_UPDATE_REQ_TYPE */
    int value;      /* For PCOM_UPDATE, dex data. For ADIE updates, value of the setting */
};

/* IOCTLs */
#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_ARM11_DONE	                    _IOW(ACOUSTIC_IOCTL_MAGIC, 22, unsigned int)

#define ACOUSTIC_UPDATE_ADIE_TABLE              _IOW(ACOUSTIC_IOCTL_MAGIC,  1, struct adie_table* )
#define ACOUSTIC_UPDATE_VOLUME_TABLE            _IOW(ACOUSTIC_IOCTL_MAGIC,  2, uint16_t* )
#define ACOUSTIC_UPDATE_CE_TABLE                _IOW(ACOUSTIC_IOCTL_MAGIC,  3, uint16_t* )
#define ACOUSTIC_UPDATE_AUDIO_PATH_TABLE        _IOW(ACOUSTIC_IOCTL_MAGIC,  4, uint16_t* )
#define ACOUSTIC_UPDATE_AUDIO_SETTINGS          _IOW(ACOUSTIC_IOCTL_MAGIC,  5, struct audio_update_req* )
#define ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE _IOW(ACOUSTIC_IOCTL_MAGIC,  6, struct htc_voc_cal_table* )
#define ACOUSTIC_GET_CAPABILITIES               _IOW(ACOUSTIC_IOCTL_MAGIC,  8, struct msm_acoustic_capabilities* )
#define ACOUSTIC_SET_HW_AUDIO_PATH              _IOW(ACOUSTIC_IOCTL_MAGIC,  10, struct msm_audio_path* )


#define HTCRPOG 0x30100002
#define HTCVERS 0
#define ONCRPC_SET_MIC_BIAS_PROC       (1)
#define ONCRPC_ACOUSTIC_INIT_PROC      (5)
#define ONCRPC_ALLOC_ACOUSTIC_MEM_PROC (6)

#define D(fmt, args...) printk(KERN_INFO "htc-acoustic_wince: "fmt, ##args)
#define E(fmt, args...) printk(KERN_ERR "htc-acoustic_wince: "fmt, ##args)

struct set_smem_req {
	struct rpc_request_hdr hdr;
	uint32_t size;
};

struct set_smem_rep {
	struct rpc_reply_hdr hdr;
	int n;
};

struct set_acoustic_req {
	struct rpc_request_hdr hdr;
};

struct set_acoustic_rep {
	struct rpc_reply_hdr hdr;
	int n;
};

/* Acoustic smem addresses */
static uint32_t htc_acoustic_volume_table;
static uint32_t htc_acoustic_ce_table;
static uint32_t htc_acoustic_adie_table;
static uint32_t htc_voc_cal_codec_table;
static uint32_t htc_acoustic_mic_offset;


static struct msm_rpc_endpoint *endpoint = NULL;
static struct mutex api_lock;
static struct mutex rpc_connect_mutex;

/* Imports from qdsp5/external.c */
extern void headphone_amp_power(int status);
extern void speaker_amp_power(int status);


/* Fill the capabilities of the device */
// TODO : fill the capabilities more precisely
static int acoustic_get_capabilities(void __user *arg)
{
    int rc;
    struct msm_acoustic_capabilities capabilities;
    
    memset((void*)&capabilities, 0, sizeof(capabilities));

    switch(__machine_arch_type) {
        case MACH_TYPE_HTCTOPAZ:
            capabilities.htc_voc_cal_fields_per_param = 0xB;
            capabilities.bDualMicSupported = true;
            rc = 0;
        break;
        case MACH_TYPE_HTCRHODIUM:
            capabilities.htc_voc_cal_fields_per_param = 0xB;
            capabilities.bDualMicSupported = true;
            capabilities.bUseTPA2016 = true;
            rc = 0;
        break;
        case MACH_TYPE_HTCRAPHAEL:
        case MACH_TYPE_HTCDIAMOND_CDMA:
        case MACH_TYPE_HTCDIAMOND:
        case MACH_TYPE_HTCBLACKSTONE:
        case MACH_TYPE_HTCRAPHAEL_CDMA:
        case MACH_TYPE_HTCRAPHAEL_CDMA500:
        case MACH_TYPE_HTCKOVSKY:
            capabilities.htc_voc_cal_fields_per_param = 0xA;
            capabilities.bDualMicSupported = true;
            rc = 0;
        break;
        default:
            rc = -1;
        break;
    }    

    if ( copy_to_user((void __user *)arg, &capabilities, sizeof(struct msm_acoustic_capabilities)) ) {
        ERR_COPY_TO_USER();
        rc = -EFAULT;
    }

    return rc;
}

/* Table update routines */
static int update_volume_table(void __user *arg)
{
    uint16_t table[0xA0];

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
        memcpy((void*) htc_acoustic_volume_table, table, sizeof(table));
    }
    
    return 0;
}

static int update_ce_table(void __user *arg)
{
    uint16_t table[0x50];

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
        memcpy((void *) htc_acoustic_ce_table, table, sizeof(table));
    }
    
    return 0;
}

static int update_audio_adie_table(void __user *arg)
{
    struct adie_table table;
    char* pcArray;
    int rc = -EIO;  

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {        
        pcArray = kmalloc(0x80, GFP_ATOMIC);
        if ( pcArray != NULL ) {
            if (copy_from_user(pcArray, table.pcArray, 0x80)) {
		        ERR_COPY_FROM_USER();
                rc = -EFAULT;
		        goto free_exit;
	        }
            memcpy((void *) (htc_acoustic_adie_table + (table.table_num * 0x80)), pcArray, 0x80);
            rc = 0;
        }
    }

free_exit:
    if ( pcArray != NULL ) {
        kfree ( pcArray );
    }
    return rc;
}

static int update_htc_voc_cal_codec_table(void __user *arg)
{
	struct msm_dex_command dex = {.cmd = PCOM_UPDATE_AUDIO, .has_data = 1 } ;
    struct htc_voc_cal_table table;
    uint16_t*   table_array;
    int rc = -EIO;    

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
        printk("%s : table size = %d\n", __func__, table.size);
        table_array = kmalloc(table.size, GFP_ATOMIC);
        if ( table_array != NULL ) {
            if (copy_from_user(table_array, table.pArray, table.size)) {
		        ERR_COPY_FROM_USER();
                rc = -EFAULT;
		        goto free_exit;
	        }
            memcpy((void *) htc_voc_cal_codec_table, table_array, table.size);
	        dex.data=0x2;
	        msm_proc_comm_wince(&dex,0); 
            rc = 0;
        }
    } 

free_exit:
    if ( table_array != NULL ) {
        kfree ( table_array );
    }
    return rc;
}

/* Adie updates */
static void ADIE_Force8k(bool bOn) {
    int adie = readl(MSM_SHARED_RAM_BASE + 0xfc0d0);
    if (bOn) {
        adie |= 0x1;        
    } else {
        adie &= ~0x1;
    }
    writel(adie, MSM_SHARED_RAM_BASE + 0xfc0d0);
}

static void ADIE_ForceADIEAwake(bool bForce) {
    int adie = readl(MSM_SHARED_RAM_BASE + 0xfc0d0);
    if (bForce) {
        adie |= 0x8;        
    } else {
        adie &= ~0x8;
    }
    writel(adie, MSM_SHARED_RAM_BASE + 0xfc0d0);
}

static void ADIE_ForceADIEUpdate(bool bForce) {
    int adie = readl(MSM_SHARED_RAM_BASE + 0xfc0d0);
    if (bForce) {
        adie |= 0x2;        
    } else {
        adie &= ~0x2;
    }
    writel(adie, MSM_SHARED_RAM_BASE + 0xfc0d0);
}

static void ADIE_UpdateAudioMethod(bool bUpdate) {
    int adie = readl(MSM_SHARED_RAM_BASE + 0xfc0d0);
    if (bUpdate) {
        adie |= 0x4;        
    } else {
        adie &= ~0x4;
    }
    writel(adie, MSM_SHARED_RAM_BASE + 0xfc0d0);
}

static void ADIE_Dump(void) {
    int i, j;

    printk("Dump A Table:\n");
    for (i=0; i<24; i++) {
        printk("A[%2X]=%2X\n", readl(MSM_SHARED_RAM_BASE + 0xfde00 + (i*2)), 
                               readl(MSM_SHARED_RAM_BASE + 0xfde00 + (i*2) + 1));
    }    
    for (i=0; i<10; i++) {
        printk("Dump B Table %d: \n", i);
        for (i=0; i<20; i++) {
            printk("B[%2X]\n", readl(MSM_SHARED_RAM_BASE + 0xfe000 + i + (j*20)));
        }        
    }
}

static int pcom_update_audio_req(int data)
{
	struct msm_dex_command dex = {.cmd = PCOM_UPDATE_AUDIO, .has_data = 1 } ;

    printk("%s : req.value %d\n", __func__, data);

    dex.data=data;
    msm_proc_comm_wince(&dex,0);  

    return 0;
}

static int update_audio_setting(void __user *arg)
{
    int ret = -EFAULT;
    struct audio_update_req req;

	if (copy_from_user(&req, arg, sizeof(struct audio_update_req))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
        switch (req.type) {
            case PCOM_UPDATE_REQ:
                ret = pcom_update_audio_req(req.value);
            break;

            case ADIE_FORCE8K_REQ:
                ADIE_Force8k( (req.value)?true:false );
                ret = 0;
            break;

            case ADIE_FORCE_ADIE_AWAKE_REQ:
                ADIE_ForceADIEAwake( (req.value)?true:false );
                ret = 0;
            break;

            case ADIE_FORCE_ADIE_UPDATE_REQ:
                ADIE_ForceADIEUpdate( (req.value)?true:false );
                ret = 0;
            break;

            case ADIE_UPDATE_AUDIO_METHOD:
                ADIE_UpdateAudioMethod( (req.value)?true:false );
                ret = 0;
            break;

            default:
            break;
        }
    }
    return ret;
}

int turn_mic_bias_on(int on)
{
	struct msm_dex_command dex;

	printk(KERN_DEBUG "%s(%d)\n", __func__, on);

	dex.cmd=PCOM_UPDATE_AUDIO;
	dex.has_data=1;

	/*  enable handset mic */
	//writel(0xffff0080|(on?0x100:0), MSM_SHARED_RAM_BASE+mic_offset);
	*(unsigned *)htc_acoustic_mic_offset=0xffff0080 | (on?0x100:0);
	dex.data=0x10;
	msm_proc_comm_wince(&dex,0);

	/* some devices needs pm_mic_en */
	if (machine_is_htcdiamond_cdma() || machine_is_htcraphael_cdma()
		|| machine_is_htcraphael_cdma500() || machine_is_htckovsky()
		|| machine_is_htctopaz())
	{
		int ret;
		struct {
			struct rpc_request_hdr hdr;
			uint32_t data;
		} req;

		if (!endpoint)
			endpoint = msm_rpc_connect(0x30000061, 0x0, 0);
		if (!endpoint) {
			printk("Couldn't open rpc endpoint\n");
			return -EIO;
		}
		req.data=cpu_to_be32(0x1);
		ret = msm_rpc_call(endpoint, 0x1c, &req, sizeof(req), 5 * HZ);
	}

	return 0;
}

EXPORT_SYMBOL(turn_mic_bias_on);


static int update_hw_audio_path(void __user *arg)
{
    struct msm_audio_path audio_path;

	if (copy_from_user(&audio_path, arg, sizeof(audio_path))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

    printk("%s : bEnableMic = %d, bEnableDualMic = %d,"
           " bEnableSpeaker = %d, bEnableHeadset = %d\n", __func__,
             audio_path.bEnableMic, audio_path.bEnableDualMic,
             audio_path.bEnableSpeaker, audio_path.bEnableHeadset);

    /* Switch microphone on/off */
    turn_mic_bias_on( audio_path.bEnableMic );

    /* Switch headset HW on/off */
    headphone_amp_power( audio_path.bEnableHeadset );
    
    /* Switch Speaker HW on/off */
    speaker_amp_power( audio_path.bEnableSpeaker );

    return 0;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	int rc = -EIO;

	D("open\n");

	mutex_lock(&api_lock);

    BUG_ON(!htc_acoustic_volume_table);
    BUG_ON(!htc_acoustic_ce_table);
    BUG_ON(!htc_acoustic_adie_table);
    BUG_ON(!htc_voc_cal_codec_table);
    BUG_ON(!htc_acoustic_mic_offset);

	rc = 0;

	mutex_unlock(&api_lock);
	return rc;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("release\n");
	return 0;
}

static long acoustic_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int rc;

    struct msm_dex_command dex;

	mutex_lock(&api_lock);

	switch (cmd) {
        case ACOUSTIC_GET_CAPABILITIES:
            D("ioctl: ACOUSTIC_GET_CAPABILITIES called %d.\n", current->pid);
            rc = acoustic_get_capabilities((void __user *)arg);
        break;

        case ACOUSTIC_ARM11_DONE:
            D("ioctl: ACOUSTIC_ARM11_DONE called %d.\n", current->pid);
            dex.cmd=PCOM_UPDATE_AUDIO;
            dex.has_data=1;
            dex.data=0x10;
            rc = msm_proc_comm_wince(&dex,0);
        break;

        case ACOUSTIC_UPDATE_ADIE_TABLE:            
            rc = update_audio_adie_table((void __user *)arg);
            D("ioctl: ACOUSTIC_UPDATE_ADIE_TABLE called %d rc %d.\n", current->pid, rc);
        break;

        case ACOUSTIC_UPDATE_VOLUME_TABLE:
            rc = update_volume_table((void __user *)arg);
            D("ioctl: ACOUSTIC_UPDATE_VOLUME_TABLE called %d rc %d.\n", current->pid, rc);
        break;

        case ACOUSTIC_UPDATE_CE_TABLE:
            rc = update_ce_table((void __user *)arg);
            D("ioctl: ACOUSTIC_UPDATE_CE_TABLE called %d rc %d.\n", current->pid, rc);
        break;

        case ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE:
            rc = update_htc_voc_cal_codec_table((void __user *)arg);
            D("ioctl: ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE called %d rc %d.\n", current->pid, rc);
        break;

        case ACOUSTIC_UPDATE_AUDIO_SETTINGS:
            rc = update_audio_setting((void __user *)arg);
            D("ioctl: ACOUSTIC_UPDATE_AUDIO_SETTINGS called %d rc %d.\n", current->pid, rc);
        break;

        case ACOUSTIC_SET_HW_AUDIO_PATH:
            rc = update_hw_audio_path((void __user *)arg);
            D("ioctl: ACOUSTIC_SET_HW_AUDIO_PATH called %d rc %d.\n", current->pid, rc);
        break;

	    default:
		    E("ioctl: invalid command\n");
		    rc = -EINVAL;
	}

	mutex_unlock(&api_lock);
	return rc;
}


static struct file_operations acoustic_fops = {
	.owner = THIS_MODULE,
	.open = acoustic_open,
	.release = acoustic_release,
	.unlocked_ioctl = acoustic_ioctl,
};

static struct miscdevice acoustic_wince_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc-acoustic_wince",
	.fops = &acoustic_fops,
};

static int __init acoustic_init(void)
{
    printk("Initialize HTC acoustic driver for wince based devices\n");

	switch(__machine_arch_type) {
		case MACH_TYPE_HTCTOPAZ:
		case MACH_TYPE_HTCRHODIUM:
            htc_acoustic_volume_table = (uint32_t)(MSM_SHARED_RAM_BASE+0xfc300);
            htc_acoustic_ce_table     = (uint32_t)(MSM_SHARED_RAM_BASE+0xfc600);
            htc_acoustic_adie_table   = (uint32_t)(MSM_SHARED_RAM_BASE+0xf8000);
            htc_voc_cal_codec_table   = (uint32_t)(MSM_SHARED_RAM_BASE+0xf9000);
			htc_acoustic_mic_offset   = (uint32_t)(MSM_SHARED_RAM_BASE+0xfb9c0);
			break;
		case MACH_TYPE_HTCRAPHAEL:
		case MACH_TYPE_HTCDIAMOND_CDMA:
		case MACH_TYPE_HTCDIAMOND:
		case MACH_TYPE_HTCBLACKSTONE:
		case MACH_TYPE_HTCRAPHAEL_CDMA:
		case MACH_TYPE_HTCRAPHAEL_CDMA500:
		case MACH_TYPE_HTCKOVSKY:
            htc_acoustic_volume_table = (uint32_t)(MSM_SHARED_RAM_BASE+0xfc300);
            htc_acoustic_ce_table     = (uint32_t)(MSM_SHARED_RAM_BASE+0xfc600);
            htc_acoustic_adie_table   = (uint32_t)(MSM_SHARED_RAM_BASE+0xfd000);
            htc_voc_cal_codec_table   = (uint32_t)(MSM_SHARED_RAM_BASE+0xfdc00);
			htc_acoustic_mic_offset   = (uint32_t)(MSM_SHARED_RAM_BASE+0xfed00);
			break;
		default:
			printk(KERN_ERR "Unsupported device for htc_acoustic driver\n");
			return -1;
			break;
	}
	mutex_init(&api_lock);
	mutex_init(&rpc_connect_mutex);

	return misc_register(&acoustic_wince_misc);
}

static void __exit acoustic_exit(void)
{
	misc_deregister(&acoustic_wince_misc);
}

module_init(acoustic_init);
module_exit(acoustic_exit);

MODULE_AUTHOR("Jerome Bruneaux <jbruneaux@laposte.net>");
MODULE_DESCRIPTION("HTC acoustic driver for wince based devices");
MODULE_LICENSE("GPL");


/*******************************************************************************************************************
 * DEBUGFS
 *******************************************************************************************************************/
#include <linux/debugfs.h>

static int dump_memory(char *buf, int max, int offset, int size) {
    int i = 0, n, j;

    for(j=0; j<size; j+=16) {
        i += scnprintf(buf + i, max - i, "%x | ", offset + j);
        for(n=0; n<16; n+=4) {
            i += scnprintf(buf + i, max - i, "%08x ", readl(offset + j + n) );
        }
        i += scnprintf(buf + i, max - i, "\n");
    }

    return i;
}

static int dump_adie_memory(char *buf, int max) {
    return dump_memory(buf, max, htc_acoustic_adie_table, 0x1000);
}

static int dump_voc_cal_memory(char *buf, int max) {
    int i = 0, n, j;

    for(j=0; j<0x2C0; j+=0x16) {
        i += scnprintf(buf + i, max - i, "%x | ", htc_voc_cal_codec_table + j);
        for(n=0; n<0x16; n+=4) {
            i += scnprintf(buf + i, max - i, "%08x ", readl(htc_voc_cal_codec_table + j + n) );
        }
        i += scnprintf(buf + i, max - i, "\n");
    }

    return i;
}

static int dbg_reset(char *buf, int max) {
    return readl(MSM_AD5_BASE + 0x00400030);
}


#define DEBUG_BUFMAX 8192
static char debug_buffer[DEBUG_BUFMAX];

static ssize_t debug_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	int (*fill)(char *buf, int max) = file->private_data;
	int bsize = fill(debug_buffer, DEBUG_BUFMAX);
	return simple_read_from_buffer(buf, count, ppos, debug_buffer, bsize);
}

static int debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations debug_ops_readonly = {
	.read = debug_read,
	.open = debug_open,
};

static int __init acoustic_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("htc_acoustic", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

    debugfs_create_file("dump_adie", S_IRUSR, dent, &dump_adie_memory, &debug_ops_readonly);
    debugfs_create_file("dump_voc", S_IRUSR, dent, &dump_voc_cal_memory, &debug_ops_readonly);
    debugfs_create_file("reset", S_IRUSR, dent, &dbg_reset, &debug_ops_readonly);

	return 0;
}
device_initcall(acoustic_dbg_init);
