
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <linux/platform_device.h>

//#include <asm/arch/msm_smd.h>
#include <mach/msm_smd.h>

//#include "diag.h"

#if 1
#define DBG(x...) do {} while (0)
#else
#define DBG(x...) printk(KERN_DEBUG x)
#endif

#if 1
#define DBG_OP(x...) do {} while (0)
#else
#define DBG_OP(x...) printk(KERN_DEBUG x)
#endif

#define TXN_MAX 8192
#define RXN_MAX 8192

#define HDLC_MAX 8192
#define ARM9_MAX 8192

#define TABLE_SIZE 10

/* number of rx requests to allocate */
#define RX_REQ_MAX 4

struct diag_request {
  void * buf;
  unsigned length;
  void (* complete) (struct diag_request *req);
  void * context;
  struct list_head list;
  int status;
  unsigned actual;
}; 

struct diag_context
{
	int online;
	int error;

	atomic_t open_arm9_excl;
	atomic_t read_arm9_excl;
	atomic_t write_arm9_excl;
		
	spinlock_t lock;
	spinlock_t lock_reg_num;

	struct list_head rx_arm9_idle;
	struct list_head rx_arm9_done;

	wait_queue_head_t read_arm9_wq;


	smd_channel_t *ch;
	int in_busy;
	int isRead;
	char is7E;
	/* assembly buffer for USB->A9 HDLC frames */
	unsigned char hdlc_buf[HDLC_MAX];
	unsigned hdlc_count;
	unsigned hdlc_escape;
	
	unsigned char id_table[TABLE_SIZE];
	unsigned char toARM9_buf[ARM9_MAX];
	
	/* the request we're currently reading from */
	struct diag_request *read_arm9_req;
	unsigned char *read_arm9_buf;
	unsigned read_arm9_count;
	struct platform_device *pdev;
	u64 tx_count; // To ARM9
	u64 rx_count; // From ARM9
};

static struct diag_context _context;

struct device diag_device;
enum data_access {
	data_set_clear = 0,
	data_set_rx,
	data_get_rx,
	data_set_tx,
	data_get_tx,
};

static void smd_try_to_send(struct diag_context *ctxt);
static void diag_configure(int configured, void *_ctxt);
static void diag_unbind(void *_ctxt);
static void diag_bind(void *_ctxt);

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static u64 smd_xfer_count_func(u64 data_length, enum data_access access_behavior)
{
	struct diag_context *ctxt = &_context;
	u64 ret = 0;
	const u64 max_value = 0x7FFFFFFF;
	switch(access_behavior)	{
		case data_set_clear:
			ctxt->tx_count = 0;
			ctxt->rx_count = 0;
		break;
		case data_set_rx:
			if((data_length + ctxt->rx_count) >= max_value)
				ctxt->rx_count = (ctxt->rx_count + data_length) - max_value;
			else
				ctxt->rx_count += data_length;
		break;
		case data_get_rx:
			ret = ctxt->rx_count;
		break;
		case data_set_tx:
			if((data_length + ctxt->tx_count) >= max_value)
				ctxt->tx_count = (ctxt->tx_count + data_length) - max_value;
			else
				ctxt->tx_count += data_length;
		break;
		case data_get_tx:
			ret = ctxt->tx_count;
		break;
	}
	return ret;
}
/* add a request to the tail of a list */
void put_req(struct diag_context *ctxt, struct list_head *head, struct diag_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

/* remove a request from the head of a list */
struct diag_request *get_req(struct diag_context *ctxt, struct list_head *head)
{
	unsigned long flags;
	struct diag_request *req;

	spin_lock_irqsave(&ctxt->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct diag_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	return req;
}

#if 0
static void diag_process_hdlc(struct diag_context *ctxt, void *_data, unsigned len)
{
	unsigned char *data = _data;
	unsigned count = ctxt->hdlc_count;
	unsigned escape = ctxt->hdlc_escape;
	unsigned char *hdlc = ctxt->hdlc_buf;

	while (len-- > 0) {
		unsigned char x = *data++;
		if (x == 0x7E) { 
			if (count > 2) {
				/* we're just ignoring the crc here */
				//TRACE("PC>", hdlc, count - 2, 0);
				if (ctxt->ch)	{
					smd_write(ctxt->ch, hdlc, count - 2);
					smd_xfer_count_func(count - 2, data_set_tx);
				}
			}
			count = 0;
			escape = 0;
		} else if (x == 0x7D) {
			escape = 1;
		} else {
			if (escape) {
				x = x ^ 0x20;
				escape = 0;
			}
			hdlc[count++] = x;

			/* discard frame if we overflow */
			if (count == HDLC_MAX)
				count = 0;
		}
	}

	ctxt->hdlc_count = count;
	ctxt->hdlc_escape = escape;
}
#endif

static void smd_try_to_send(struct diag_context *ctxt)
{
	if (ctxt->ch) {
		int r = smd_read_avail(ctxt->ch);
		if (r > RXN_MAX) {
			printk(KERN_ERR "The SMD data is too large to send (%d) !!\n", r);
//			return;
            r = RXN_MAX;
		}
		if (r > 0) {
			struct diag_request *req = get_req(ctxt, &ctxt->rx_arm9_idle);
			if (!req) {
				printk(KERN_ERR "There is no enough request to ARM11!!\n");
				return;
			}
			smd_read(ctxt->ch, req->buf, r);
			smd_xfer_count_func(r, data_set_rx);
			//req->length = r;
			//printk(KERN_ERR "ARM9 data to ARM11 %s\n", (char *)req->buf);
			req->actual = r;
			put_req(ctxt, &ctxt->rx_arm9_done, req);
			wake_up(&ctxt->read_arm9_wq);
		}
	}
}

static void smd_diag_notify(void *priv, unsigned event)
{
	struct diag_context *ctxt = priv;
	smd_try_to_send(ctxt);
}

static int msm_diag_smd_open(void)
{
	struct diag_context *ctxt = &_context;
	int r = 0;

    if ( !ctxt->ch ) {
	    r = smd_open("SMD_DIAG", &ctxt->ch, ctxt, smd_diag_notify);
    }
	return r;
}

static int msm_diag_smd_close(void)
{
	struct diag_context *ctxt = &_context;
	int r;

	r = smd_close(ctxt->ch);
    ctxt->ch = 0;
	return r;
}


static int diag2arm9_open(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;

    DBG_OP("+diag2arm9_open()\n");

	if (_lock(&ctxt->open_arm9_excl))
		return -EBUSY;

    msm_diag_smd_open();
    

    diag_configure(1, ctxt);
	/* clear the error latch */
	//ctxt->error = 0;

    DBG_OP("-diag2arm9_open()\n");
	return 0;
}

static int diag2arm9_release(struct inode *ip, struct file *fp)
{
	struct diag_context *ctxt = &_context;
	struct diag_request *req;

    DBG_OP("+%s\n", __func__);

    diag_configure(0, ctxt);
    
    msm_diag_smd_close();

	/* recycle unhandled rx reqs to user if any */
	while ((req = get_req(ctxt, &ctxt->rx_arm9_done)))
		put_req(ctxt, &ctxt->rx_arm9_idle, req);

//    diag_unbind(ctxt);
    /* Release readers that might be blocked */
	wake_up(&ctxt->read_arm9_wq);

	_unlock(&ctxt->open_arm9_excl);

    DBG_OP("-%s\n", __func__);
	return 0;
}

static ssize_t diag2arm9_write(struct file *fp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	int r = count;
	int writed = 0;

	DBG("diag2arm9_write(%d)\n", count);

	if (_lock(&ctxt->write_arm9_excl))
		return -EBUSY;

	while(count > 0) {
		/*
		if (ctxt->error) {
			r = -EIO;
			break;
		}
		*/
		writed = count > ARM9_MAX ? ARM9_MAX : count;
		if (copy_from_user(ctxt->toARM9_buf, buf, writed)) {
				r = -EFAULT;
				break;
		}
		smd_write(ctxt->ch, ctxt->toARM9_buf, writed);
		smd_xfer_count_func(writed, data_set_tx);
		buf += writed;
		count -= writed;
	}

	_unlock(&ctxt->write_arm9_excl);
	return r;
}

static ssize_t diag2arm9_read(struct file *fp, char __user *buf,
			size_t count, loff_t *pos)
{
	struct diag_context *ctxt = &_context;
	struct diag_request *req;
	int r = 0, xfer;
	int ret;
	//ctxt->isRead = 1;

	DBG("diag2arm9_read(%d)\n", count);
	
	if (_lock(&ctxt->read_arm9_excl))
		return -EBUSY;

	/* we will block until we're offline */
	/*
	while (ctxt->online) {
		ret = wait_event_interruptible(ctxt->read_arm9_wq, !(ctxt->online));
		if (ret < 0) {
			_unlock(&ctxt->read_arm9_excl);
			return ret;
		}
	}
	*/
	while (count > 0) {
		/*
		if (ctxt->error) {
			r = -EIO;
			break;
		}
		*/
		/* if we have idle read requests, get them queued */
		/*
		while ((req = get_req(ctxt, &ctxt->rx_idle))) {
requeue_req:
			req->length = TXN_MAX;
			ret = usb_ept_queue_xfer(ctxt->out, req);
			if (ret < 0) {
				DBG("diag_read: failed to queue req %p (%d)\n", req, ret);
				r = -EIO;
				ctxt->error = 1;
				put_req(ctxt, &ctxt->rx_idle, req);
				goto fail;
			} else {
				DBG("rx %p queue\n", req);
			}
		}
		*/
		
		/* if we have data pending, give it to userspace */
		if (ctxt->read_arm9_count > 0) {
			xfer = (ctxt->read_arm9_count < count) ? ctxt->read_arm9_count : count;
			if (copy_to_user(buf, ctxt->read_arm9_buf, xfer)) {
				DBG("diag: copy_to_user fail\n");
				r = -EFAULT;
				break;
			}
			ctxt->read_arm9_buf += xfer;
			ctxt->read_arm9_count -= xfer;
			buf += xfer;
			count -= xfer;
			r += xfer;

			/* if we've emptied the buffer, release the request */
			if (ctxt->read_arm9_count == 0) {
				put_req(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
				ctxt->read_arm9_req = 0;
			}
			continue;
		}

		/* wait for a request to complete */
		req = 0;
		ret = wait_event_interruptible(ctxt->read_arm9_wq,
					       ((req = get_req(ctxt, &ctxt->rx_arm9_done)) || ctxt->error));

		if (req != 0) {
			/* if we got a 0-len one we need to put it back into
			** service.  if we made it the current read req we'd
			** be stuck forever
			*/
			if (req->actual == 0) {
			//	goto requeue_req;
				put_req(ctxt, &ctxt->rx_arm9_idle, req);
				continue;
			}

			ctxt->read_arm9_req = req;
			ctxt->read_arm9_count = req->actual;
			ctxt->read_arm9_buf = req->buf;
			if (ctxt->read_arm9_count < count)
				count = ctxt->read_arm9_count;
			DBG("rx %p %d\n", req, req->actual);
		}

		if (ret < 0) {
            DBG_OP("%s: ret < 0\n", __func__);
			r = ret;
			break;
		}
	}

//fail:
	_unlock(&ctxt->read_arm9_excl);
	//ctxt->isRead = 0;
	return r;
}

#if 0
static ssize_t show_diag_xfer_count(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int length = 0;
	length = sprintf(buf, "tx_count: %llu, rx_count: %llu\n",
		smd_xfer_count_func(0, data_get_tx), smd_xfer_count_func(0, data_get_rx));
	return length;

}
#endif

static struct file_operations diag2arm9_fops = {
	.owner =   THIS_MODULE,
	.open =    diag2arm9_open,
	.release = diag2arm9_release,
	.write = diag2arm9_write,
	.read = diag2arm9_read,
};

static struct miscdevice diag2arm9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "diag_arm9",
	.fops = &diag2arm9_fops,
};

static void diag_unbind(void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	struct diag_request *req;

	printk(KERN_DEBUG "diag_unbind()\n");

//	misc_deregister(&diag2arm9_device);

	smd_xfer_count_func(0,data_set_clear);

	while ((req = get_req(ctxt, &ctxt->rx_arm9_idle))) {
		kfree(req->buf);
        kfree(req);
	}

	ctxt->online = 0;
	ctxt->error = 1;

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_arm9_wq);
}

static void diag_bind(void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;
	struct diag_request *req;
	int n;

	printk(KERN_DEBUG "diag_bind()\n");

	for (n = 0; n < RX_REQ_MAX; n++) {
		req = kmalloc(sizeof(struct diag_request), GFP_KERNEL);
		if (req == 0) {
			pr_err("%s: kmalloc out of memory\n",
				__func__);
            goto fail;
        }
        req->buf = kmalloc(RXN_MAX, GFP_KERNEL);
		if (!req->buf) {
			pr_err("%s: kmalloc out of memory\n",
                __func__);
    		kfree(req);
			goto fail;
		}

		req->context = ctxt;
		put_req(ctxt, &ctxt->rx_arm9_idle, req);
	}

	printk(KERN_DEBUG
	       "diag_bind() allocated %d rx requests\n",
	       RX_REQ_MAX);

	smd_xfer_count_func(0,data_set_clear);
	return;

fail:
	printk(KERN_WARNING "diag_bind() could not allocate requests\n");
	diag_unbind(ctxt);
}

static void diag_configure(int configured, void *_ctxt)
{
	struct diag_context *ctxt = _ctxt;

	DBG("diag_configure() %d\n", configured);

	if (configured) {
		ctxt->online = 1;

		/* if we have a stale request being read, recycle it */
		ctxt->read_arm9_buf = 0;
		ctxt->read_arm9_count = 0;

		if (ctxt->read_arm9_req) {
			put_req(ctxt, &ctxt->rx_arm9_idle, ctxt->read_arm9_req);
			ctxt->read_arm9_req = 0;
		}

	} else {
		ctxt->online = 0;
		ctxt->error = 1;
	}

	/* readers may be blocked waiting for us to go online */
	wake_up(&ctxt->read_arm9_wq);
}

static int __init diag_init(void)
{
	int i;
	struct diag_context *ctxt = &_context;
	DBG("diag_init()\n");
	
	ctxt->isRead = 0;
	ctxt->is7E = 0x7E;
	
	init_waitqueue_head(&ctxt->read_arm9_wq);
	
	atomic_set(&ctxt->open_arm9_excl, 0);
	atomic_set(&ctxt->read_arm9_excl, 0);
	atomic_set(&ctxt->write_arm9_excl, 0);
	
	spin_lock_init(&ctxt->lock);
	spin_lock_init(&ctxt->lock_reg_num);

	INIT_LIST_HEAD(&ctxt->rx_arm9_idle);
	INIT_LIST_HEAD(&ctxt->rx_arm9_done);

    printk("%s: heads: rx_arm9_idle = %p, rx_arm9_done = %p\n",
            __func__, &ctxt->rx_arm9_idle, &ctxt->rx_arm9_done);

	for (i = 0; i < TABLE_SIZE; i++)
		ctxt->id_table[i] = 0;

    diag_bind(ctxt);

	misc_register(&diag2arm9_device);

	return 0;
}

module_init(diag_init);
