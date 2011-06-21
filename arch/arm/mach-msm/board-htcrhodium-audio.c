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
#include <linux/platform_device.h>
#include <linux/mfd/microp-ng.h>

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

	gpio_request(RHODIUM_HS_AMP_PWR, "HTC Rhodium HS Amp Power");

	rc=i2c_add_driver(&adc_driver);
	return rc;
}

module_init(rhod_audio_init);

void htcrhodium_set_headset_amp(bool enable) {
	if (enable) {
		/* Power up headphone amp */
		gpio_tlmm_config(
			GPIO_CFG(RHODIUM_HS_AMP_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
	} else {
		/* Power down headphone amp */
		gpio_tlmm_config(
			GPIO_CFG(RHODIUM_HS_AMP_PWR, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
			GPIO_CFG_DISABLE);
	}
}
EXPORT_SYMBOL(htcrhodium_set_headset_amp);

/******************************************************************************
 * MicroP Audio
 ******************************************************************************/
static struct htcrhodium_microp_audio_data {
	int states;
	struct i2c_client* client;
	struct mutex lock;
} audio_data;

static int htcrhodium_microp_audio_leds_set(uint8_t val)
{
	int ret;
	uint8_t buffer[4] = { 0, 0, 0, 0 };

	if (!audio_data.client) {
		printk(KERN_ERR "%s: i2c_client not initialized!\n", __func__);
		return -EAGAIN;
	}

	mutex_lock(&audio_data.lock);
	buffer[0] = RHOD_MICROP_KLT_LED_STATE;
	buffer[1] = val;
	ret = microp_ng_write(audio_data.client, buffer, 2);
	if (ret) {
		mutex_unlock(&audio_data.lock);
		printk(KERN_ERR "%s: i2c write failed with %d!\n", __func__, ret);
		return ret;
	}
	audio_data.states = val;
	mutex_unlock(&audio_data.lock);

	return ret;
}

int htcrhodium_microp_audio_dualmic_set(char on)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, on);

	if(on)
		return htcrhodium_microp_audio_leds_set(audio_data.states | RHOD_MICROP_KLT_DUALMIC_EN_BIT);
	return htcrhodium_microp_audio_leds_set(audio_data.states & (~RHOD_MICROP_KLT_DUALMIC_EN_BIT));
}

int htcrhodium_microp_audio_codec_set(char on)
{
	printk(KERN_DEBUG "%s(%d)\n", __func__, on);

	if(on)
		return htcrhodium_microp_audio_leds_set(audio_data.states | RHOD_MICROP_KLT_CODEC_EN_BIT);
	return htcrhodium_microp_audio_leds_set(audio_data.states & (~RHOD_MICROP_KLT_CODEC_EN_BIT));
}

int htcrhodium_microp_audio_get_codec_state(void)
{
	int state = (audio_data.states & RHOD_MICROP_KLT_CODEC_EN_BIT) ? 1 : 0;
	printk(KERN_DEBUG "%s: state=%d\n", __func__, state);
	return state;
}

static int htcrhodium_microp_audio_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);

	mutex_init(&audio_data.lock);
	mutex_lock(&audio_data.lock);
	audio_data.states = 0;
	audio_data.client = dev_get_drvdata(&pdev->dev);
	mutex_unlock(&audio_data.lock);

	return 0;
}

static int htcrhodium_microp_audio_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);

	audio_data.states = 0;
	audio_data.client = NULL;
	mutex_destroy(&audio_data.lock);

	return 0;
}

#if CONFIG_PM
static int htcrhodium_microp_audio_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int htcrhodium_microp_audio_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define htcrhodium_microp_audio_suspend NULL
#define htcrhodium_microp_audio_resume NULL
#endif

static struct platform_driver htcrhodium_microp_audio_driver = {
	.probe		= htcrhodium_microp_audio_probe,
	.remove		= htcrhodium_microp_audio_remove,
	.suspend	= htcrhodium_microp_audio_suspend,
	.resume		= htcrhodium_microp_audio_resume,
	.driver		= {
		.name	= "htcrhodium-microp-audio",
		.owner	= THIS_MODULE,
	},
};

static int __init htcrhodium_microp_audio_init(void)
{
	return platform_driver_register(&htcrhodium_microp_audio_driver);
}

static void __exit htcrhodium_microp_audio_exit(void)
{
	platform_driver_unregister(&htcrhodium_microp_audio_driver);
}

module_init(htcrhodium_microp_audio_init);
module_exit(htcrhodium_microp_audio_exit)

/* Compat for a1010.c */
int micropklt_dualmic_set(char on) {
	return htcrhodium_microp_audio_dualmic_set(on);
}

int micropklt_codec_set(char on) {
	return htcrhodium_microp_audio_codec_set(on);
}

int micropklt_get_codec_state(void) {
	return htcrhodium_microp_audio_get_codec_state();
}
