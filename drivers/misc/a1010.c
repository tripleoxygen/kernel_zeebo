/* drivers/i2c/chips/a1010.c - a1010 voice processor driver
 *
 * Based on drivers/i2c/chips/A1010.c (Copyright (C) 2009 HTC Corporation.)
 * Adaptated for a1010 by Jerome BRUNEAUX
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

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/a1010.h>


#define DEBUG			(1)
#define ENABLE_DIAG_IOCTLS	(0)

static struct i2c_client *this_client;

static struct mutex A1010_lock;
static int A1010_opened = 0;
static int A1010_suspended = 1;;
static int A1010_current_config = A1010_PATH_SUSPEND;

// FIXME drop this again
extern int micropklt_dualmic_set(char on);
extern int micropklt_codec_set(char on);
int micropklt_get_codec_state(void);

static int A1010_i2c_read(char *rxData, int length)
{
	int rc;
	uint8_t buf[1] = {0x66};

	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	rc = i2c_transfer(this_client->adapter, msgs, 2);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: rx[%d] = %2x\n", __func__, i, rxData[i]);
	}
#endif

	return 0;
}

static int A1010_i2c_write(char *txData, int length)
{
	int rc;
	uint8_t buf[5];

	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		},
	};

	if ( length > 4 ) {
		printk("%s: buffer too long\n", __func__);
		return -1;
	}

	buf[0] = 0x66;
	memcpy(&buf[1], txData, length);

	rc = i2c_transfer(this_client->adapter, msg, 1);
	if (rc < 0) {
		pr_err("%s: transfer error %d\n", __func__, rc);
		return rc;
	}

#if DEBUG
	{
		int i = 0;
		for (i = 0; i < length; i++)
			pr_info("%s: tx[%d] = %2x\n", __func__, i, txData[i]);
	}
#endif

	return 0;
}

static int A1010_write_cmds(char* txData, int length)
{
	int rc = 0, msg_size = 0;
	int rd_retry_cnt;
	unsigned char *index_msg = 0;
	unsigned char ack_buf[A1010_CMD_FIFO_DEPTH * 4];
	unsigned char rdbuf[4];
#if DEBUG
	int i;
#endif

	pr_info("%s: block write start (size = %d)\n", __func__, length);
#if DEBUG
        for (i = 1; i <= length; i++) {
                pr_info("%x ", *(txData + i - 1));
                if ( !(i % 4))
                        pr_info("\n");
        }
#endif

    msg_size = length;
    index_msg = txData;
    do {
        rc = A1010_i2c_write(index_msg, (msg_size > 4)?4:msg_size);
	    if (rc < 0) {
		    pr_err("A1010 CMD block write error!\n");
		    return rc;
	    }

	    /* Don't need to get Ack after sending out a suspend command */
	    if (*index_msg == 0x80 && *(index_msg + 1) == 0x10
		    && *(index_msg + 2) == 0x00 && *(index_msg + 3) == 0x01) {
		    A1010_suspended = 1;
		    /* Disable A1010 clock */
		    msleep(120);
		    return rc;
	    }

	    memset(ack_buf, 0, sizeof(ack_buf));
	    msleep(20);
	    pr_info("%s: CMD ACK block read start\n", __func__);
	    rc = A1010_i2c_read(ack_buf, 4);
	    if (rc < 0) {
		    pr_err("%s: CMD ACK block read error\n", __func__);
		    return rc;
	    } else {
		    pr_info("%s: CMD ACK block read end\n", __func__);
#if DEBUG
		    for (i = 1; i <= 4; i++) {
			    pr_info("%x ", ack_buf[i-1]);
			    if ( !(i % 4))
				    pr_info("\n");
		    }
#endif
		    if (*ack_buf == 0x00) {
			    rd_retry_cnt = POLLING_RETRY_CNT;
rd_retry:
			    if (rd_retry_cnt--) {
				    memset(rdbuf, 0, sizeof(rdbuf));
				    rc = A1010_i2c_read(rdbuf, 4);
				    if (rc < 0)
					    return rc;
#if DEBUG
				    for (i = 0; i < sizeof(rdbuf); i++) {
					    pr_info("0x%x\n", rdbuf[i]);
				    }
				    pr_info("-----------------\n");
#endif
				    if (rdbuf[0] == 0x00) {
					    msleep(20);
					    goto rd_retry;
				    }
			    } else {
				    pr_err("%s: CMD ACK Not Ready\n",
					    __func__);
				    return -EBUSY;
			    }
		    } else if (*ack_buf == 0xff) { /* illegal cmd */
                pr_err("%s: CMD ACK Illegal cmd\n",
					    __func__);
			    return -ENOEXEC;
		    } else if ( (*ack_buf == 0x80) && (ack_buf[1] = index_msg[1]) ){
                if (msg_size >= 4 ) {
                    msg_size -= 4;
                    index_msg += 4;
                } else {
                    msg_size = 0;
                    index_msg += msg_size; 
                }
			    continue;
		    }
	    }


    } while ( msg_size != 0 );
	pr_info("%s: block write end\n", __func__);

	return rc;
}

static int A1010_open(struct inode *inode, struct file *file)
{
	int rc = 0;

	mutex_lock(&A1010_lock);

	if (A1010_opened) {
		pr_err("%s: busy\n", __func__);
		rc = -EBUSY;
		goto done;
	}

	A1010_opened = 1;
done:
	mutex_unlock(&A1010_lock);
	return rc;
}

static int A1010_release(struct inode *inode, struct file *file)
{
	mutex_lock(&A1010_lock);
	A1010_opened = 0;
	mutex_unlock(&A1010_lock);

	return 0;
}

unsigned char phonecall_receiver[] = {
    0x80,0x17,0x00,0x02, /* SetAlgorithmParmID, 0x0002:Microphone Configuration */
    0x80,0x18,0x00,0x00, /* SetAlgorithmParm, 0x0000:2-mic Close Talk (CT) */
    0x80,0x1B,0x00,0x00, /* SetTxDigitalInputGain: 0x0000 */
    0x80,0x15,0x00,0x05, /* SetDigitalOutputGain, 0x00:Tx, 0x05:(5 dB) */
    0x80,0x23,0xFF,0xFC, /* SetRcvDigitalInputGain: 0xFFFC */
    0x80,0x17,0x00,0x03, /* SetAlgorithmParmID, 0x0003:AEC Mode */
    0x80,0x18,0x00,0x03, /* SetAlgorithmParm, 0x0003: ?? */
    0x80,0x17,0x00,0x00, /* SetAlgorithmParmID, 0x0000:Suppression Strength */
    0x80,0x18,0x00,0x06, /* SetAlgorithmParm, 0x0006:??dB Max Suppression */
    0x80,0x0C,0x03,0x00, /* SetDeviceParmID, 0x0300: ?? */
    0x80,0x0D,0x00,0x02  /* SetDeviceParm, 0x0002: ?? */
};

unsigned char phonecall_speaker[] = {
    0x80,0x17,0x00,0x02, /* SetAlgorithmParmID, 0x0002:Microphone Configuration */
    0x80,0x18,0x00,0x00, /* SetAlgorithmParm, 0x0000:2-mic Close Talk (CT) */
    0x80,0x1B,0x00,0x09, /* SetTxDigitalInputGain: 0x0009 */
    0x80,0x15,0x00,0x08, /* SetDigitalOutputGain, 0x00:Tx, 0x08:(8 dB) */
    0x80,0x23,0xFF,0xF6, /* SetRcvDigitalInputGain: 0xFFF6 */
    0x80,0x17,0x00,0x03, /* SetAlgorithmParmID, 0x0003:AEC Mode */
    0x80,0x18,0x00,0x03, /* SetAlgorithmParm, 0x0003: ?? */
    0x80,0x17,0x00,0x00, /* SetAlgorithmParmID, 0x0000:Suppression Strength */
    0x80,0x18,0x00,0x01, /* SetAlgorithmParm, 0x0001:??dB Max Suppression */
    0x80,0x0C,0x03,0x00, /* SetDeviceParmID, 0x0300: ?? */
    0x80,0x0D,0x00,0x00, /* SetDeviceParm, 0x0000: ?? */
    0x80,0x0C,0x04,0x00, /* SetDeviceParmID, 0x0400: ?? */
    0x80,0x0D,0x00,0x00, /* SetDeviceParm, 0x0000: ?? */
    0x80,0x17,0x00,0x23, /* SetAlgorithmParmID, 0x0023: AEC_CNG */
    0x80,0x18,0x00,0x01, /* SetAlgorithmParm, 0x0001: ?? */
    0x80,0x17,0x00,0x2E, /* SetAlgorithmParmID, 0x002E: ?? */
    0x80,0x18,0xFF,0xBF  /* SetAlgorithmParm, 0xFFBF */
};

unsigned char phonecall_speaker_slide[] = {
    0x80,0x17,0x00,0x02, /* SetAlgorithmParmID, 0x0002:Microphone Configuration */
    0x80,0x18,0x00,0x00, /* SetAlgorithmParm, 0x0000:2-mic Close Talk (CT) */
    0x80,0x1B,0x00,0x09, /* SetTxDigitalInputGain: 0x0009 */
    0x80,0x15,0x00,0x08, /* SetDigitalOutputGain, 0x00:Tx, 0x08:(8 dB) */
    0x80,0x23,0xFF,0xF6, /* SetRcvDigitalInputGain: 0xFFF6 */
    0x80,0x17,0x00,0x03, /* SetAlgorithmParmID, 0x0003:AEC Mode */
    0x80,0x18,0x00,0x03, /* SetAlgorithmParm, 0x0003: ?? */
    0x80,0x17,0x00,0x00, /* SetAlgorithmParmID, 0x0000:Suppression Strength */
    0x80,0x18,0x00,0x01, /* SetAlgorithmParm, 0x0001:??dB Max Suppression */
    0x80,0x0C,0x03,0x00, /* SetDeviceParmID, 0x0300: ?? */
    0x80,0x0D,0x00,0x00, /* SetDeviceParm, 0x0000: ?? */
    0x80,0x0C,0x04,0x00, /* SetDeviceParmID, 0x0400: ?? */
    0x80,0x0D,0x00,0x00, /* SetDeviceParm, 0x0000: ?? */
    0x80,0x17,0x00,0x23, /* SetAlgorithmParmID, 0x0023: AEC_CNG */
    0x80,0x18,0x00,0x01, /* SetAlgorithmParm, 0x0001: ?? */
    0x80,0x17,0x00,0x2E, /* SetAlgorithmParmID, 0x002E: ?? */
    0x80,0x18,0xFF,0xBF  /* SetAlgorithmParm, 0xFFBF */
};

unsigned char suspend_mode[] = {
	0x80,0x10,0x00,0x01  /* Sleep */
};

unsigned char activate_APC[] = {
    /* From ActivateAPC() in wavedev.dll */
    0x80,0x17,0x00,0x17,    /* MAKE_APC_ON_COMMAND (pApcOn_1) */
    0x80,0x18,0x00,0x01,    /* MAKE_APC_ON_COMMAND (pApcOn_2) */
    0x80,0x17,0x00,0x18,    /* MAKE_APC_SET_THRESHOLD_COMMAND (pApcThreshold_1) */
    0x80,0x18,0xFF,0xD8,    /* MAKE_APC_SET_THRESHOLD_COMMAND (pApcThreshold_2) */
    0x80,0x17,0x00,0x2F,    /* MAKE_APC_TURN_OFF_TIME_COMMAND (pApcTurnOffTime_1) */
    0x80,0x18,0x00,0x04,    /* MAKE_APC_TURN_OFF_TIME_COMMAND (pApcTurnOffTime_2) */
};

unsigned char noiseReduction_enable[] = {
    0x80,0x16,0x00,0x1D,    /* GetAlgorithmParm, 0x001D */
};
    
unsigned char wakeup_prepare[] = {
    0x80,0x25,0x00,0x00
};

unsigned char speaker_volume[] = {
    0x80,0x17,0x00,0x12,    /* SetAlgorithmParmID, 0x0012:Speaker Volume */
    0x80,0x18,0xFF,0xF9,    /* SetAlgorithmParm, 0xFFF9: ?? */
};

unsigned char receiver_volume[] = {
    0x80,0x17,0x00,0x12,    /* SetAlgorithmParmID, 0x0012:Speaker Volume */
    0x80,0x18,0xFF,0xF2,    /* SetAlgorithmParm, 0xFFF9: ?? */
};

static ssize_t chk_wakeup_A1010(void)
{
    int codec_state = micropklt_get_codec_state();
#if DEBUG
    printk("%s: %d\n", __func__, codec_state);
#endif
    return !codec_state;
}

int A1010_set_config(char newid, int mode)
{
	int rc = 0, size = 0;
	unsigned char *i2c_cmds;

	if ((A1010_suspended) && (newid == A1010_PATH_SUSPEND))
		return rc;

	rc = chk_wakeup_A1010();
	if (rc <= 0) {
        printk("A1010 not powered\n");
        A1010_suspended = 1;
		return rc;
    }

	switch (newid) {
		case A1010_PATH_SUSPEND:
		    i2c_cmds = suspend_mode;
		    size = sizeof(suspend_mode);           
			break;
	    case A1010_PATH_EARCOUPLE:
		    i2c_cmds = phonecall_receiver;
		    size = sizeof(phonecall_receiver);
		    break;
	    case A1010_PATH_SPEAKER:
		    i2c_cmds = phonecall_speaker;
		    size = sizeof(phonecall_speaker);
		    break;
	    case A1010_PATH_SPKSLIDE:
		    i2c_cmds = phonecall_speaker_slide;
		    size = sizeof(phonecall_speaker_slide);
		    break;

	    default:
		    pr_err("%s: invalid newid %d\n", __func__, newid);
		    rc = -1;
		    goto input_err;
		    break;
	}

    if ( A1010_suspended ) {
        pr_info("%s: wakeup a1010\n", __func__);

        rc = A1010_write_cmds(wakeup_prepare, sizeof(wakeup_prepare));
	    if (rc < 0) {
		    pr_err("A1010 wakeup cmd error!\n");
		    return rc;
	    }
        A1010_suspended = 0;
    }

	A1010_current_config = newid;
	pr_info("%s: change to mode %d\n", __func__, newid);
    rc = A1010_write_cmds(i2c_cmds, size);
    if ( rc < 0 ) {
        goto input_err;
    }

    if ( newid == A1010_PATH_SUSPEND ) {
        A1010_suspended = 1;
    }

input_err:
	return rc;
}

static int
A1010_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rc = 0;
#if ENABLE_DIAG_IOCTLS
	char msg[4];
	int mic_cases = 0;
	int mic_sel = 0;
#endif
	int pathid = 0;

	switch (cmd) {
	
	    case A1010_SET_CONFIG:
		    if (copy_from_user(&pathid, argp, sizeof(pathid)))
			    return -EFAULT;
		    rc = A1010_set_config(pathid, A1010_CONFIG_FULL);
		    if (rc < 0)
			    pr_err("%s: A1010_SET_CONFIG (%d) error %d!\n",
				    __func__, pathid, rc);
		    break;
#if ENABLE_DIAG_IOCTLS
	    case A1010_READ_DATA:
		    rc = chk_wakeup_A1010();
		    if (rc < 0)
			    return rc;
		    rc = A1010_i2c_read(msg, 4);
		    if (copy_to_user(argp, &msg, 4))
			    return -EFAULT;
		    break;

	    case A1010_WRITE_MSG:
		    rc = chk_wakeup_A1010();
		    if (rc < 0)
			    return rc;
		    if (copy_from_user(msg, argp, sizeof(msg)))
			    return -EFAULT;
		    rc = A1010_i2c_write(msg, 4);
		    break;

	    case A1010_SYNC_CMD:
		    rc = chk_wakeup_A1010();
		    if (rc < 0)
			    return rc;
		    msg[0] = 0x80;
		    msg[1] = 0x00;
		    msg[2] = 0x00;
		    msg[3] = 0x00;
		    rc = A1010_i2c_write(msg, 4);
		    break;	
#endif /* ENABLE_DIAG_IOCTLS */
	    default:
		    pr_err("%s: invalid command %d\n", __func__, _IOC_NR(cmd));
		    rc = -EINVAL;
		    break;
	}

	return rc;
}

static const struct file_operations A1010_fops = {
	.owner = THIS_MODULE,
	.open = A1010_open,
	.release = A1010_release,
	.ioctl = A1010_ioctl,
};

static struct miscdevice A1010_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "audience_A1010",
	.fops = &A1010_fops,
};

static int A1010_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;

	this_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		rc = -ENODEV;
		goto a1010_probe_err;
	}

	rc = misc_register(&A1010_device);
	if (rc) {
		pr_err("%s: A1010_device register failed\n", __func__);
		goto a1010_probe_err;
	}

	return 0;

a1010_probe_err:
	return rc;
}

static int A1010_remove(struct i2c_client *client)
{
	struct A1010_platform_data *p1026data = i2c_get_clientdata(client);
	kfree(p1026data);

	return 0;
}

static int A1010_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}

static int A1010_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id A1010_id[] = {
	{ "audience_A1010", 0 },
	{ }
};

static struct i2c_driver A1010_driver = {
	.probe = A1010_probe,
	.remove = A1010_remove,
	.suspend = A1010_suspend,
	.resume	= A1010_resume,
	.id_table = A1010_id,
	.driver = {
		.name = "audience_A1010",
	},
};

static int __init A1010_init(void)
{
	pr_info("%s\n", __func__);
	mutex_init(&A1010_lock);

	return i2c_add_driver(&A1010_driver);
}

static void __exit A1010_exit(void)
{
	i2c_del_driver(&A1010_driver);
}

module_init(A1010_init);
module_exit(A1010_exit);

MODULE_DESCRIPTION("A1010 voice processor driver");
MODULE_LICENSE("GPL");

#ifdef CONFIG_DEBUG_FS
/*******************************************************************************************************************
 * DEBUGFS
 *******************************************************************************************************************/
#include <linux/debugfs.h>

extern void ADC3001_Enable(void);
static int dbg_pup(char *buf, int max) {
    micropklt_codec_set(0);
    micropklt_dualmic_set(1);
    mdelay(30);
    ADC3001_Enable();  
    micropklt_dualmic_set(0); 
    mdelay(30);
    return 0;
}

static int dbg_en(char *buf, int max) {
    micropklt_dualmic_set(1);
    mdelay(30);
    micropklt_codec_set(1);
    mdelay(30);
    return micropklt_codec_set(0);
}

static int dbg_dis(char *buf, int max) {
    micropklt_dualmic_set(0);
    return micropklt_codec_set(1);
}

static int dbg_test(char *buf, int max) {
    A1010_set_config(A1010_PATH_SPEAKER, 0);
    return 0;
}

ssize_t debug_write_a1010(struct file *file, const char __user *buf,
		     size_t count, loff_t *ppos)
{
    char *pbuffer = kzalloc(count, GFP_KERNEL);

    if ( copy_from_user(pbuffer, buf, count) ) {
		return -EFAULT;
	}
    
    while( (pbuffer[0] != 0) && (pbuffer[0] != '\n') ){ 
        switch(pbuffer[0]) { 
            case 'a':
                A1010_write_cmds(activate_APC, sizeof(activate_APC));
            break;
     
            case 'e':
                A1010_write_cmds(receiver_volume, sizeof(receiver_volume));
            break;

            case 's':
                A1010_write_cmds(speaker_volume, sizeof(speaker_volume));
            break;

            default:
        	printk(KERN_ERR "%s: cmd '%c' not supported\n", __func__, pbuffer[0]);
            break;
	    }
        pbuffer++;
    };

	return count;
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

static const struct file_operations debug_ops = {
	.read = debug_read,
	.open = debug_open,
    .write = debug_write_a1010,
};

static const struct file_operations debug_ops_readonly = {
	.read = debug_read,
	.open = debug_open,
};

static int __init a1010_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("a1010", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

    debugfs_create_file("en", S_IRUSR, dent, &dbg_en, &debug_ops_readonly);
    debugfs_create_file("dis", S_IRUSR, dent, &dbg_dis, &debug_ops_readonly);
    debugfs_create_file("test", S_IRUSR, dent, &dbg_test, &debug_ops);
    debugfs_create_file("pup", S_IRUSR, dent, &dbg_pup, &debug_ops_readonly);

	return 0;
}
device_initcall(a1010_dbg_init);
#endif
