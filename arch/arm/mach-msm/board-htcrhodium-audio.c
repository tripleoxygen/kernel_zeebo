#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/earlysuspend.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <mach/tpa2016d2.h>
#include "proc_comm_wince.h"
#include "board-htcrhodium.h"

#define MODULE_NAME "rhodium_audio"

struct data_t {
	struct i2c_client *adc3001;
};
static struct data_t _dat;

/* The ADC3001 setup / control could have been done in userland using IOCTLS
 * but as it's only used by rhodium and values written to registers are fixed
 * (among all DualMic.csv files) this can be hardcoded here.
 */

static unsigned char ADC3001_WakeUp[][2] = 
{
    /* reg, value */
    { 0x00, 0x00 },
    { 0x01, 0x01 },
    { 0x04, 0x03 },
    { 0x05, 0x11 },
    { 0x06, 0x05 },
    { 0x07, 0x04 },
    { 0x08, 0xB0 },
    { 0x12, 0xB0 },
    { 0x13, 0x82 },
    { 0x14, 0x80 },
    { 0x1B, 0x4C },
    { 0x1C, 0x01 },
    { 0x1D, 0x02 },
    { 0x1E, 0x81 },
    { 0x05, 0x91 },
    { 0x35, 0x02 },
    { 0x00, 0x01 },
    { 0x34, 0xFC },
    { 0x37, 0xFC },
    { 0x33, 0x08 },
    { 0x00, 0x00 },
    { 0x3D, 0x01 },
    { 0x51, 0xC0 },
    { 0x52, 0x00 },
};

#if 0
static unsigned char ADC3001_PowerDown[][2] = 
{
    /* reg, value */
    { 0x00, 0x00 },
    { 0x01, 0x01 },
};
#endif

int micropklt_dualmic_set(char on);
int micropklt_codec_set(char on);

static int ADC3001_write(unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
	{
		.addr = _dat.adc3001->addr,
		.flags = 0,
		.len = length + 1,
		.buf = txdata,
	},
	};

	if (i2c_transfer(_dat.adc3001->adapter, msg, 1) < 0) {
		pr_err("ADC3001_write failed\n");
		return -EIO;
	}

	return 0;
}

static void ADC3001_write_sequence(unsigned char adc3001_reg_seq[][2], int len) 
{
    int index;

    for (index = 0; index < len; index++ ) {
        printk("[%d] ADC3001[0x%x] = 0x%x\n", index, adc3001_reg_seq[index][0], adc3001_reg_seq[index][1]);
        ADC3001_write(adc3001_reg_seq[index], 2);
    }
}

void ADC3001_Enable(void)
{
    ADC3001_write_sequence(ADC3001_WakeUp, sizeof(ADC3001_WakeUp) / 2);
}

void ADC3001_wakeup(void)
{
    micropklt_dualmic_set(1);
    mdelay(30);
    micropklt_codec_set(1);
    mdelay(30);
    micropklt_codec_set(0);
    mdelay(30);
    ADC3001_Enable();
}

void ADC3001_powerdown(void)
{
//    ADC3001_write_sequence(ADC3001_PowerDown, sizeof(ADC3001_PowerDown) / 2);
    micropklt_dualmic_set(0);
    micropklt_codec_set(1);
}

//ADC3001 ADC + mic bias
static int adc_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{
	printk("%s\n", __func__);
	//Not much to do here uh ?
	_dat.adc3001=client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c check functionality error\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static const struct i2c_device_id adc_ids[] = {
        { "adc3001", 0 },
        { }
};

static struct i2c_driver adc_driver = {
	.driver = {
		.name	= "adc3001",
		.owner	= THIS_MODULE,
	},
	.id_table = adc_ids,
	.probe = adc_probe,
};

static int __init rhod_audio_init(void) {
	int rc;

	if(!machine_is_htcrhodium())
		return 0;

	printk(KERN_INFO "Rhodium audio registering drivers\n");

	rc=i2c_add_driver(&adc_driver);
	return rc;
}

module_init(rhod_audio_init);

void htcrhodium_set_headset_amp(bool enable) {
    if (enable)
    {
        /* Power up headphone amp */
        //~ gpio_configure(RHODIUM_HS_AMP_PWR, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
        
        gpio_tlmm_config(
			GPIO_CFG(RHODIUM_HS_AMP_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
    }
    else
    {
        /* Power down headphone amp */
        //~ gpio_configure(RHODIUM_HS_AMP_PWR, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
        
        gpio_tlmm_config(
			GPIO_CFG(RHODIUM_HS_AMP_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
    }
    //~ gpio_set_value(RHODIUM_HS_AMP_PWR, enable);
}
EXPORT_SYMBOL(htcrhodium_set_headset_amp);

/******************************************************************************
 * MicroP TEMPORARY
 ******************************************************************************/
static int rhod_states = 0;

static int micropklt_rhod_leds_set(uint8_t val)
{
	//~ int r;
	//~ struct microp_klt *data;
	//~ struct i2c_client *client;
	//~ uint8_t buffer[4] = { 0, 0, 0, 0 };

	printk(KERN_DEBUG "%s: val=%u\n", __func__, val);

	//~ data = micropklt_t;
	//~ if (!data) return -EAGAIN;
	//~ client = data->client;
//~ 
	//~ mutex_lock(&data->lock);
		//~ buffer[0] = MICROP_KLT_RHOD_LED_STATE;
		//~ buffer[1] = val;
//~ 
	//~ r = micropklt_write(client, buffer, 2);
//~ 
	//~ micropklt_t->rhod_states = val;
	//~ 
	//~ mutex_unlock(&data->lock);
	return 0;
}

int micropklt_dualmic_set(char on)
{
	//~ if(on)
		//~ return micropklt_rhod_leds_set(micropklt_t->rhod_states | RHOD_DUALMIC_EN);
	//~ return micropklt_rhod_leds_set(micropklt_t->rhod_states & (~RHOD_DUALMIC_EN));

	printk(KERN_DEBUG "%s(%d)\n", __func__, on);

	return 0;
}

int micropklt_codec_set(char on)
{
	//~ if(on)
		//~ return micropklt_rhod_leds_set(micropklt_t->rhod_states | RHOD_CODEC_EN);
	//~ return micropklt_rhod_leds_set(micropklt_t->rhod_states & (~RHOD_CODEC_EN));

	printk(KERN_DEBUG "%s(%d)\n", __func__, on);
	rhod_states = 1;
	return 0;
}

int micropklt_get_codec_state(void)
{
	//~ return ((micropklt_t->rhod_states & RHOD_CODEC_EN)?1:0);
	printk(KERN_DEBUG "%s: state=%d\n", __func__, rhod_states);
	return rhod_states;
}
