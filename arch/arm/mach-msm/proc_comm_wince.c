/* arch/arm/mach-msm/proc_comm_wince.c
 *
 * Author: maejrep
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
 * Based on proc_comm.c by Brian Swetland
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/earlysuspend.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
#include <mach/system.h>
#include <mach/htc_battery.h>

#include "gpio_chip.h"
#include "proc_comm_wince.h"

#define MSM_A2M_INT(n) (MSM_CSR_BASE + 0x400 + (n) * 4)

static inline void notify_other_proc_comm(void)
{
	writel(1, MSM_A2M_INT(6));
}

#define PC_DEBUG 0
#define PC_DEBUG_INT 0

#define PC_COMMAND      0x00
#define PC_STATUS       0x04
#define PC_SERIAL       0x08
#define PC_SERIAL_CHECK 0x0C
#define PC_DATA         0x20
#define PC_DATA_RESULT  0x24

#if (PC_DEBUG > 0)
 #define DDEX(fmt, arg...) printk(KERN_DEBUG "[DEX] %s: " fmt "\n", __FUNCTION__, ## arg)
#else
 #define DDEX(fmt, arg...) do {} while (0)
#endif

#if (PC_DEBUG_INT > 0)
 #define DDEX_INT(fmt, arg...) printk(KERN_DEBUG "[DEX] %s: " fmt "\n", __FUNCTION__, ## arg)
#else
 #define DDEX_INT(fmt, arg...) do {} while (0)
#endif

#define DEX_A9_M2A_6_SHARED_MUTEX_ADDR   (MSM_SHARED_RAM_BASE + 0xfc138)
#define DEX_A9_M2A_6_PENDING_INT_ADDR   (MSM_SHARED_RAM_BASE + 0xfc128)
#define DEX_ARM11_MUTEX_ID          0x11
#define VIC_REG(off)                (MSM_VIC_BASE + (off))
#define VIC_INT_ENCLEAR0            VIC_REG(0x0020)
#define VIC_INT_ENSET0              VIC_REG(0x0030)
#define VIC_INT_CLEAR0              VIC_REG(0x00B0)

/* DEX A2M6 interrupts flags */
#define DEX_INT_RTC_ALARM                       0x1
#define DEX_INT_VBUS                            0x2
#define DEX_INT_ACK                             0x4
#define DEX_INT_BATT                            0x8
#define DEX_INT_USB_ID                          0x10
#define DEX_INT_SND_TEST_RX_OUTPUT_BUF          0x20
#define DEX_INT_SND_TEST_RX_INPUT_BUF           0x40
#define DEX_INT_SND_TEST_TX_OUTPUT_BUF          0x80
#define DEX_INT_SND_TEST_TX_INPUT_BUF           0x100
#define DEX_INT_AUD_REC_GET_BUF                 0x200
#define DEX_INT_UNK0                            0x400
#define DEX_INT_UNK1                            0x800
#define DEX_INT_M2A_NOTIFY_ARM9_REQ_RESTART     0x1000
#define DEX_INT_M2A_NOTIFY_ARM9_AT_CMD_READY    0x2000

static void msm_proc_comm_wince_interrupt_do_work(struct work_struct *work);
DECLARE_WORK(msm_proc_comm_wince_interrupt_work,
             msm_proc_comm_wince_interrupt_do_work);
/* this var hold the pending interrupts 'collected' by the proc_comm interrupt */
static int  msm_proc_comm_wince_pending_ints = 0;
static bool msm_proc_comm_wince_device_sleeping = false;

static DEFINE_SPINLOCK(proc_comm_lock);

/* The higher level SMD support will install this to
 * provide a way to check for and handle modem restart?
 */
int (*msm_check_for_modem_crash)(void);

#define TIMEOUT (10000000) /* 10s in microseconds */

int msm_proc_comm_wince(struct msm_dex_command * in, unsigned *out)
{
#if !defined(CONFIG_MSM_AMSS_VERSION_WINCE)
  #warning NON-WinCE compatible AMSS version selected. WinCE proc_comm implementation is disabled and stubbed to return -EIO.
        return -EIO;
#else
	unsigned base = (unsigned)(MSM_SHARED_RAM_BASE + 0xfc100);
	unsigned long flags;
	unsigned timeout;
	unsigned status;
	unsigned num;
	unsigned base_cmd, base_status;

	spin_lock_irqsave(&proc_comm_lock, flags);

	DDEX("waiting for modem; command=0x%02x data=0x%x", in->cmd, in->data);

	// Store original cmd byte
	base_cmd = in->cmd & 0xff;

	// Write only lowest byte
	writeb(base_cmd, base + PC_COMMAND);

	// If we have data to pass, add 0x100 bit and store the data
	if ( in->has_data )
	{
		writel(readl(base + PC_COMMAND) | DEX_HAS_DATA, base + PC_COMMAND);
		writel(in->data, base + PC_DATA);
	} else {
		writel(readl(base + PC_COMMAND) & ~DEX_HAS_DATA, base + PC_COMMAND);
		writel(0, base + PC_DATA);
	}
	
	// Increment last serial counter
	num = readl(base + PC_SERIAL) + 1;
	writel(num, base + PC_SERIAL);

	DDEX("command and data sent (cntr=0x%x) ...", num);

	// Notify ARM9 with int6
	notify_other_proc_comm();

	// Wait for response...  XXX: check irq stat?
	timeout = TIMEOUT;
	while ( --timeout && readl(base + PC_SERIAL_CHECK) != num )
		udelay(1);

	if ( ! timeout )
	{
		printk(KERN_WARNING "%s: DEX cmd timed out. status=0x%x, A2Mcntr=%x, M2Acntr=%x\n", 
			__func__, readl(base + PC_STATUS), num, readl(base + PC_SERIAL_CHECK));
		goto end;
	}
	
	DDEX("command result status = 0x%08x", readl(base + PC_STATUS));

	// Read status of command
	status = readl(base + PC_STATUS);
	writeb(0, base + PC_STATUS);
	base_status = status & 0xff;
	DDEX("status new = 0x%x; status base = 0x%x", 
		readl(base + PC_STATUS), base_status);


	if ( base_status == base_cmd )
	{
		if ( status & DEX_STATUS_FAIL )
		{
			DDEX("DEX cmd failed; status=%x, result=%x",
				readl(base + PC_STATUS),
				readl(base + PC_DATA_RESULT));

			writel(readl(base + PC_STATUS) & ~DEX_STATUS_FAIL, base + PC_STATUS);
		}
		else if ( status & DEX_HAS_DATA )
		{
			writel(readl(base + PC_STATUS) & ~DEX_HAS_DATA, base + PC_STATUS);
			if (out)
				*out = readl(base + PC_DATA_RESULT);
			DDEX("DEX output data = 0x%x", 
				readl(base + PC_DATA_RESULT));
		}
	} else {
		printk(KERN_WARNING "%s: DEX Code not match! a2m[0x%x], m2a[0x%x], a2m_num[0x%x], m2a_num[0x%x]\n", 
			__func__, base_cmd, base_status, num, readl(base + PC_SERIAL_CHECK));
	}

end:
	writel(0, base + PC_DATA_RESULT);
	writel(0, base + PC_STATUS);

	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return 0;
#endif
}

void msm_proc_comm_wince_vibrate(uint32_t val)
{
	struct msm_dex_command vibra;

	if (val == 0) {
		vibra.cmd = PCOM_VIBRA_OFF;
		msm_proc_comm_wince(&vibra, 0);
	} else if (val > 0) {
		if (val == 1 || val > 0xb22)
			val = 0xb22;
		writel(val, MSM_SHARED_RAM_BASE + 0xfc130);
		vibra.cmd = PCOM_VIBRA_ON;
		msm_proc_comm_wince(&vibra, 0);
	}
}

void msm_proc_comm_wince_vibrate_welcome(void)
{
	int i;

	for (i = 0; i < 2; i++) {
		msm_proc_comm_wince_vibrate(1);
		mdelay(150);
		msm_proc_comm_wince_vibrate(0);
		mdelay(75);
	}
}

static void msm_proc_comm_reset(void)
{
	struct msm_dex_command dex = {.cmd = PCOM_NOTIFY_ARM9_REBOOT };
	msm_proc_comm_wince(&dex, 0);
	mdelay(350);
	gpio_request(25, "MSM Reset");
	msm_gpio_set_flags(25, GPIOF_OWNER_ARM11);
	gpio_direction_output(25, 0);
	printk(KERN_INFO "%s: Soft reset done.\n", __func__);
}

#define PLLn_BASE(n)		(MSM_CLK_CTL_BASE + 0x300 + 28 * (n))
#define TCX0			19200000 // Hz
#define PLL_FREQ(l, m, n)	(TCX0 * (l) + TCX0 * (m) / (n))

#define DUMP_PLL(name, base) { \
	unsigned int mode, L, M, N, freq; \
	mode = readl(base); \
	L = readl(base + 0x4); \
	M = readl(base + 0x8); \
	N = readl(base + 0xc); \
	freq = PLL_FREQ(L, M, N); \
	printk(KERN_INFO "%s @ %p: MODE=%08x L=%08x M=%08x N=%08x freq=%u Hz (%u MHz)\n", \
		name, base, mode, L, M, N, freq, freq / 1000000); \
	}

// Dump useful debug stuff
void dump_debug_stuff(void)
{
	unsigned int pcb_xc;
	char amss_ver[16];

	// Dump PLL params (for debug purposes, no relation to proc_comm)
	DUMP_PLL("PLL0", PLLn_BASE(0));
	DUMP_PLL("PLL1", PLLn_BASE(1));
	DUMP_PLL("PLL2", PLLn_BASE(2));
	DUMP_PLL("PLL3", PLLn_BASE(3));

	// Dump PCB XC
	pcb_xc = readl(MSM_SHARED_RAM_BASE + 0xfc048);
	printk(KERN_INFO "PCB XC: %08x\n", pcb_xc);

	// Dump AMMS version
	*(unsigned int *) (amss_ver + 0x0) = readl(MSM_SHARED_RAM_BASE + 0xfc030 + 0x0);
	*(unsigned int *) (amss_ver + 0x4) = readl(MSM_SHARED_RAM_BASE + 0xfc030 + 0x4);
	*(unsigned int *) (amss_ver + 0x8) = readl(MSM_SHARED_RAM_BASE + 0xfc030 + 0x8);
	*(unsigned int *) (amss_ver + 0xc) = readl(MSM_SHARED_RAM_BASE + 0xfc030 + 0xc);
	amss_ver[15] = 0;
	printk(KERN_INFO "AMSS version: %s\n", amss_ver);
}

static void msm_proc_comm_wince_interrupt_do_work(struct work_struct *work)
{
    int processed_int = 0;

    DDEX_INT("pending ints = 0x%08x", msm_proc_comm_wince_pending_ints);
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_RTC_ALARM ) {
        DDEX_INT("DEX_RTC_ALARM");
        processed_int |= DEX_INT_RTC_ALARM;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_VBUS ) {
        DDEX_INT("DEX_VBUS");
        /* In case the device was woken up by the vbus irq, delay the vbus notification
         * so that device can finish wakeup before otherwise, usb will be notified
         * that vbus has changed and will start its power sequence before device is
         * fully out of sleep. It introduce a delay up to 10 seconds (something must 
         * be locking up in the usb power sequence)
         */
        if ( msm_proc_comm_wince_device_sleeping == false ) {
            htc_cable_status_update(0);
            processed_int |= DEX_INT_VBUS;
        }
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_ACK ) {
        DDEX_INT("DEX_ACK");
        processed_int |= DEX_INT_ACK;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_BATT ) {
        DDEX_INT("DEX_BATT");
        processed_int |= DEX_INT_BATT;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_USB_ID ) {
        DDEX_INT("DEX_USB_ID");
        processed_int |= DEX_INT_USB_ID;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_SND_TEST_RX_OUTPUT_BUF ) {
        DDEX_INT("DEX_SND_TEST_RX_OUTPUT_BUF");
        processed_int |= DEX_INT_SND_TEST_RX_OUTPUT_BUF;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_SND_TEST_RX_INPUT_BUF ) {
        DDEX_INT("DEX_SND_TEST_RX_INPUT_BUF");
        processed_int |= DEX_INT_SND_TEST_RX_INPUT_BUF;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_SND_TEST_TX_OUTPUT_BUF ) {
        DDEX_INT("DEX_SND_TEST_TX_OUTPUT_BUF");
        processed_int |= DEX_INT_SND_TEST_TX_OUTPUT_BUF;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_SND_TEST_TX_INPUT_BUF ) {
        DDEX_INT("DEX_SND_TEST_TX_INPUT_BUF");
        processed_int |= DEX_INT_SND_TEST_TX_INPUT_BUF;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_AUD_REC_GET_BUF ) {
        DDEX_INT("DEX_AUD_REC_GET_BUF");
        processed_int |= DEX_INT_AUD_REC_GET_BUF;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_UNK0 ) {
        processed_int |= DEX_INT_UNK0;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_UNK1 ) {
        processed_int |= DEX_INT_UNK1;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_M2A_NOTIFY_ARM9_REQ_RESTART ) {
        DDEX_INT("DEX_M2A_NOTIFY_ARM9_REQ_RESTART");
        /* Arm9 will generate INT_A9_M2A_6 interrupt until this bit to be cleared.
         * Should we reboot before clearing or does it ask for himself to be rebooted ??
         */
        processed_int |= DEX_INT_M2A_NOTIFY_ARM9_REQ_RESTART;
    }
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_M2A_NOTIFY_ARM9_AT_CMD_READY ) {
        DDEX_INT("DEX_M2A_NOTIFY_ARM9_AT_CMD_READY");
        processed_int |= DEX_INT_M2A_NOTIFY_ARM9_AT_CMD_READY;
    }
    if ( msm_proc_comm_wince_pending_ints > DEX_INT_M2A_NOTIFY_ARM9_AT_CMD_READY) {
        printk(KERN_WARNING "DEX_UNKNOWN_NOTIFY %x\n", msm_proc_comm_wince_pending_ints);
    }

    /* Clear pending interrupt flag of the processed interrupt */
    msm_proc_comm_wince_pending_ints &= ~processed_int;
}

static irqreturn_t msm_proc_comm_wince_irq(int irq, void *dev_id)
{
    int dex_pending_ints = readl(DEX_A9_M2A_6_PENDING_INT_ADDR);

    if ( dex_pending_ints ) {
        /* Update pending ints flags for the scheduled work */
        msm_proc_comm_wince_pending_ints |= dex_pending_ints;
        schedule_work(&msm_proc_comm_wince_interrupt_work);
    }

    /* Clear the matching flag in smem */
    smem_semaphore_down(DEX_A9_M2A_6_SHARED_MUTEX_ADDR, DEX_ARM11_MUTEX_ID);
    writel(readl(DEX_A9_M2A_6_PENDING_INT_ADDR) & ~dex_pending_ints,
                 DEX_A9_M2A_6_PENDING_INT_ADDR );
    smem_semaphore_up(DEX_A9_M2A_6_SHARED_MUTEX_ADDR, DEX_ARM11_MUTEX_ID);

    /* No more interrupts pending, clear the irq bit */
    if ( readl(DEX_A9_M2A_6_PENDING_INT_ADDR) == 0 ) {
        disable_irq_nosync(INT_A9_M2A_6);
        writel(1U << INT_A9_M2A_6, VIC_INT_CLEAR0);
        enable_irq(INT_A9_M2A_6);
    }

    return IRQ_HANDLED;
}

void msm_proc_comm_wince_enter_sleep(void)
{
    msm_proc_comm_wince_device_sleeping = true;
}

void msm_proc_comm_wince_exit_sleep(void)
{
    msm_proc_comm_wince_device_sleeping = false;
    /* If the VBUS interrupt was triggered while device was exiting sleep
     * state, then process it now that the device is fully woken.
     */
    if ( msm_proc_comm_wince_pending_ints & DEX_INT_VBUS ) {
        htc_cable_status_update(0);
        msm_proc_comm_wince_pending_ints &= DEX_INT_VBUS;
    }
}

static void msm_proc_comm_wince_early_suspend(struct early_suspend *h) {
	struct msm_dex_command dex;
	printk("Sending arm9_low_speed 2\n");
	dex.cmd = PCOM_ARM9_LOW_SPEED;
	dex.has_data = 1;
	dex.data = 2;
	msm_proc_comm_wince(&dex, 0);
}

static void msm_proc_comm_wince_late_resume(struct early_suspend *h) {
	struct msm_dex_command dex;
	printk("Sending arm9_low_speed 7\n");
	dex.cmd = PCOM_ARM9_LOW_SPEED;
	dex.has_data = 1;
	dex.data = 7;
	msm_proc_comm_wince(&dex, 0);
}

static struct early_suspend early_suspend = {
	.suspend = msm_proc_comm_wince_early_suspend,
	.resume = msm_proc_comm_wince_late_resume,
	.level = 48,
};

// Initialize PCOM registers
int msm_proc_comm_wince_init()
{
#if !defined(CONFIG_MSM_AMSS_VERSION_WINCE)
        return 0;
#else
	unsigned base = (unsigned)(MSM_SHARED_RAM_BASE + 0xfc100);
	unsigned long flags;

	spin_lock_irqsave(&proc_comm_lock, flags);

	writel(0, base + PC_DATA);
	writel(0, base + PC_DATA_RESULT);
	writel(0, base + PC_SERIAL);
	writel(0, base + PC_SERIAL_CHECK);
	writel(0, base + PC_STATUS);

	if (request_irq(INT_A9_M2A_6, msm_proc_comm_wince_irq, IRQF_TRIGGER_RISING,
					"proc_comm", NULL) < 0 ) {
		printk("%s: Error requesting proc_comm irq\n", __func__); 
	}

	spin_unlock_irqrestore(&proc_comm_lock, flags);

	msm_hw_reset_hook = msm_proc_comm_reset;
	register_early_suspend(&early_suspend);

	printk(KERN_INFO "%s: WinCE PCOM initialized.\n", __func__);

	dump_debug_stuff();

	return 0;
#endif
}
