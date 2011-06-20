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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
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
#include <asm/mach-types.h>
#include <mach/msm_smd.h>

#include <mach/htc_acoustic_wince.h>
#include <mach/msm_iomap.h>
#include "proc_comm_wince.h"

#define ACOUSTIC_SHARED_MUTEX_ADDR      (MSM_SHARED_RAM_BASE + 0xfc280)
#define ACOUSTIC_ARM11_MUTEX_ID         0x11

/* structs */
struct msm_acoustic_capabilities {
    char voc_cal_field_size;    /* Specifies the number of fields per parameter */
    bool dual_mic_supported;
};

struct msm_audio_path {
	bool enable_mic;
	bool enable_dual_mic;
	bool enable_speaker;
	bool enable_headset;
};

struct adie_table {
	int   table_num;
	char *pcArray;
};

struct htc_voc_cal_table {
	uint16_t *pArray;
	int       size;
};

enum audio_update_req_type {
    PCOM_UPDATE_REQ = 0,
    ADIE_FORCE8K_REQ,
    ADIE_FORCE_ADIE_AWAKE_REQ,
    ADIE_FORCE_ADIE_UPDATE_REQ,
    ADIE_UPDATE_AUDIO_METHOD,

};

struct audio_update_req {
    enum audio_update_req_type type;
    int value;
};

enum dex_audio_cmd {
	DEX_UPDATE_VOC = 0x2,
	DEX_AUDIO_DONE = 0x10,
};

#define E(fmt, args...) printk(KERN_ERR "htc-acoustic_wince: "fmt, ##args)

#if 1
	#define D(fmt, args...) printk(KERN_INFO "htc-acoustic_wince: "fmt, ##args)
#else
	#define D(fmt, args...) do {} while (0)
#endif

#define ERR_USER_COPY(to) pr_err("%s(%d): copy %s user\n", \
		__func__, __LINE__, ((to) ? "to" : "from"))
#define ERR_COPY_FROM_USER() ERR_USER_COPY(0)
#define ERR_COPY_TO_USER() ERR_USER_COPY(1)

static struct mutex api_lock;
static struct mutex rpc_connect_mutex;

static struct htc_acoustic_wce_amss_data *amss_data = NULL;
struct htc_acoustic_wce_board_data *htc_acoustic_wce_board_data = NULL;

/* From board-htcrhodium-audio.c */
extern void ADC3001_wakeup(void);
extern void ADC3001_powerdown(void);

static void headphone_amp_power(bool status)
{
	if ( (htc_acoustic_wce_board_data) && (htc_acoustic_wce_board_data->set_headset_amp) )
		htc_acoustic_wce_board_data->set_headset_amp(status);
}

static void speaker_amp_power(bool status)
{
	if ( (htc_acoustic_wce_board_data) && (htc_acoustic_wce_board_data->set_speaker_amp) )
		htc_acoustic_wce_board_data->set_speaker_amp(status);
}

/* Fill the capabilities of the device */
static int acoustic_get_capabilities(void __user *arg)
{
	int rc = 0;
	struct msm_acoustic_capabilities capabilities = {};

	capabilities.voc_cal_field_size = amss_data->voc_cal_field_size;

	if (htc_acoustic_wce_board_data) {
		capabilities.dual_mic_supported =
			htc_acoustic_wce_board_data->dual_mic_supported;
	}


	if (copy_to_user((void __user *)arg,
					&capabilities,
					sizeof(struct msm_acoustic_capabilities))
	) {
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
		memcpy(amss_data->volume_table, table, sizeof(table));
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
		memcpy(amss_data->ce_table, table, sizeof(table));
	}

	return 0;
}

static int update_audio_adie_table(void __user *arg)
{
	struct adie_table table;
	char pcArray[0x80];
	int rc = -EIO;

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
		if (copy_from_user(pcArray, table.pcArray, 0x80)) {
			ERR_COPY_FROM_USER();
			return -EFAULT;
		}
		memcpy((amss_data->adie_table +
				(table.table_num * 0x80)), pcArray, 0x80);
		rc = 0;
	}

	return rc;
}

static int update_codec_table(void __user * arg)
{
	struct msm_dex_command dex = {
		.cmd = PCOM_UPDATE_AUDIO,
		.has_data = 1,
		.data = DEX_UPDATE_VOC,
	};
	struct htc_voc_cal_table table;
	uint16_t *table_array;
	int rc = -EIO;

	if (copy_from_user(&table, arg, sizeof(table))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	} else {
		D("%s : table size = %d\n", __func__, table.size);
		table_array = kmalloc(table.size, GFP_ATOMIC);
		if (table_array != NULL) {
			if (copy_from_user
			    (table_array, table.pArray, table.size)) {
				ERR_COPY_FROM_USER();
				rc = -EFAULT;
				goto free_exit;
			}
            memcpy(amss_data->codec_table, table_array, table.size);
	        msm_proc_comm_wince(&dex,0);
			rc = 0;
		}
	}

 free_exit:
	if (table_array != NULL) {
		kfree(table_array);
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
    int adie;

    printk("%s %d\n", __func__, bForce);

    smem_semaphore_down(ACOUSTIC_SHARED_MUTEX_ADDR, ACOUSTIC_ARM11_MUTEX_ID);

    adie = readl(MSM_SHARED_RAM_BASE + 0xfc0d0);

    if (bForce) {
        adie |= 0x2;
    } else {
        adie &= ~0x2;
    }
    writel(adie, MSM_SHARED_RAM_BASE + 0xfc0d0);

    smem_semaphore_up(ACOUSTIC_SHARED_MUTEX_ADDR, ACOUSTIC_ARM11_MUTEX_ID);
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

static int dex_update_audio(int data)
{
	struct msm_dex_command dex = {
		.cmd = PCOM_UPDATE_AUDIO,
		.has_data = 1,
		.data = data,
	};

	msm_proc_comm_wince(&dex, 0);

	return 0;
}

static int dex_update_audio_done(void) {
	struct msm_dex_command dex = {
		.cmd = PCOM_UPDATE_AUDIO,
		.has_data = 1,
		.data = DEX_AUDIO_DONE,
	};
	D("%s\n", __func__);
	return msm_proc_comm_wince(&dex, 0);
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
                ret = dex_update_audio(req.value);
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

static int turn_mic_bias_on_internal(bool on, bool bDualMicEn)
{
	char pmSpeakerGain[2][10] = { 
			{0x93, 0, 0x93, 7, 0x93, 1, 0x93, 7, 0xFF, 0xFF},
			{0x93, 0, 0x93, 4, 0x93, 1, 0x93, 4, 0xFF, 0xFF} };


	D("%s(%d)\n", __func__, on);

	/* enable handset mic */
	if ( machine_is_htcrhodium() && bDualMicEn && on ) {
		memcpy(amss_data->mic_offset, pmSpeakerGain[1], 10);
	} else {
		writel(0xffff0080 | (on ? 0x100 : 0), amss_data->mic_offset);   
	}
	dex_update_audio_done();

	if ( machine_is_htcrhodium() ) {
		if ( bDualMicEn && on ) {
			ADC3001_wakeup();
		} else {
			ADC3001_powerdown();
		}
	}

	if (amss_data->mic_bias_callback)
		amss_data->mic_bias_callback(on);

	return 0;
}

int turn_mic_bias_on(bool on)
{
	return turn_mic_bias_on_internal(on, false);
}
EXPORT_SYMBOL(turn_mic_bias_on);

static int update_hw_audio_path(void __user *arg)
{
	struct msm_audio_path audio_path;

	if (copy_from_user(&audio_path, arg, sizeof(audio_path))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	D("%s: mic=%d, dual_mic=%d, speaker=%d, headset = %d\n",
			__func__,
	       audio_path.enable_mic, audio_path.enable_dual_mic,
	       audio_path.enable_speaker, audio_path.enable_headset);

	/* Switch microphone on/off */
	turn_mic_bias_on_internal(audio_path.enable_mic,
                     audio_path.enable_dual_mic);

	/* Switch headset HW on/off */
	headphone_amp_power(audio_path.enable_headset);

	/* Switch Speaker HW on/off */
	speaker_amp_power(audio_path.enable_speaker);

	return 0;
}

static int acoustic_open(struct inode *inode, struct file *file)
{
	D("%s\n", __func__);
	return 0;
}

static int acoustic_release(struct inode *inode, struct file *file)
{
	D("%s\n", __func__);
	return 0;
}

static long acoustic_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int rc;

	mutex_lock(&api_lock);

	switch (cmd) {
		case ACOUSTIC_GET_CAPABILITIES:
			D("ioctl: ACOUSTIC_GET_CAPABILITIES called %d.\n",
				task_pid_nr(current));
			rc = acoustic_get_capabilities((void __user *)arg);
		break;

		case ACOUSTIC_ARM11_DONE:
			rc = dex_update_audio_done();
			D("ioctl: ACOUSTIC_ARM11_DONE pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_UPDATE_ADIE_TABLE:
			rc = update_audio_adie_table((void __user *)arg);
			D("ioctl: ACOUSTIC_UPDATE_ADIE_TABLE pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_UPDATE_VOLUME_TABLE:
			rc = update_volume_table((void __user *)arg);
			D("ioctl: ACOUSTIC_UPDATE_VOLUME_TABLE pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_UPDATE_CE_TABLE:
			rc = update_ce_table((void __user *)arg);
			D("ioctl: ACOUSTIC_UPDATE_CE_TABLE pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE:
			rc = update_codec_table((void __user *)arg);
			D("ioctl: ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_UPDATE_AUDIO_SETTINGS:
			rc = update_audio_setting((void __user *)arg);
			D("ioctl: ACOUSTIC_UPDATE_AUDIO_SETTINGS called pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		case ACOUSTIC_SET_HW_AUDIO_PATH:
			rc = update_hw_audio_path((void __user *)arg);
			D("ioctl: ACOUSTIC_SET_HW_AUDIO_PATH pid=%d result=%d\n",
				task_pid_nr(current), rc);
			break;

		default:
			E("ioctl: invalid command %d pid=%d\n", cmd, task_pid_nr(current));
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

static int htc_acoustic_wince_probe(struct platform_device *pdev)
{
	void *ret;
	struct htc_acoustic_wce_amss_data *pdata = pdev->dev.platform_data;

	printk("Initialize HTC acoustic driver for wince based devices\n");

	if (!pdata) {
		E("%s: no platform data\n", __func__);
		goto err_no_pdata;
	}
	ret = pdata->volume_table;
	if (!ret)
		goto err_pdata_incomplete;
	ret = pdata->ce_table;
	if (!ret)
		goto err_pdata_incomplete;

	ret = pdata->adie_table;
	if (!ret)
		goto err_pdata_incomplete;

	ret = pdata->codec_table;
	if (!ret)
		goto err_pdata_incomplete;

	ret = pdata->mic_offset;
	if (!ret)
		goto err_pdata_incomplete;

	amss_data = pdata;
	mutex_init(&api_lock);
	mutex_init(&rpc_connect_mutex);

	return misc_register(&acoustic_wince_misc);

err_pdata_incomplete:
	E("%s: offsets for some tables undefined in platform data\n", __func__);
err_no_pdata:
	return -EINVAL;
}

static int htc_acoustic_wince_remove(struct platform_device *pdev)
{
	misc_deregister(&acoustic_wince_misc);
	amss_data = NULL;
	return 0;
}

static struct platform_driver htc_acoustic_wince_driver = {
	.probe		= htc_acoustic_wince_probe,
	.remove		= htc_acoustic_wince_remove,
	.driver		= {
		.name		= "htc_acoustic",
		.owner		= THIS_MODULE,
	},
};

static int __init htc_acoustic_wince_init(void)
{
	return platform_driver_register(&htc_acoustic_wince_driver);
}

static void __exit htc_acoustic_wince_exit(void)
{
	platform_driver_unregister(&htc_acoustic_wince_driver);
}

module_init(htc_acoustic_wince_init);
module_exit(htc_acoustic_wince_exit);

MODULE_AUTHOR("Jerome Bruneaux <jbruneaux@laposte.net>");
MODULE_DESCRIPTION("HTC acoustic driver for wince based devices");
MODULE_LICENSE("GPL");

#ifdef CONFIG_DEBUG_FS
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
    return dump_memory(buf, max, (int)amss_data->adie_table, 0x1000);
}

static int dump_voc_cal_memory(char *buf, int max) {
    int i = 0, n, j;

    for(j=0; j<0x2C0; j+=0x16) {
        i += scnprintf(buf + i, max - i, "%x | ", (int)amss_data->codec_table + j);
        for(n=0; n<0x16; n+=4) {
            i += scnprintf(buf + i, max - i, "%08x ", readl(amss_data->codec_table + j + n) );
        }
        i += scnprintf(buf + i, max - i, "\n");
    }

    return i;
}

#if 0
static int dbg_reset(char *buf, int max) {
    return readl(MSM_AD5_BASE + 0x00400030);
}
#endif


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
//    debugfs_create_file("reset", S_IRUSR, dent, &dbg_reset, &debug_ops_readonly);

	return 0;
}
device_initcall(acoustic_dbg_init);
#endif
