/*
 microp-keypad.c - i2c keyboard driver found on certain HTC Phones
 Depends on microp-ksc and microp-klt i2c chip drivers

 Joe Hansche <madcoder@gmail.com>
 Based in part on htc-spi-kbd.c from Kevin2

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/leds.h>

#include <linux/mfd/microp-ng.h>
#include <linux/microp-keypad.h>

#define MODULE_NAME "microp-keypad"

#define MICROP_DEBUG 0

#if defined(MICROP_DEBUG) && MICROP_DEBUG
	#define DLOG(fmt, arg...) printk(fmt, ## arg);
#else
	#define DLOG(fmt, arg...) do {} while(0)
#endif

static struct microp_keypad_t {
	struct mutex lock;
	struct i2c_client *client;
	struct input_dev *input;
	struct microp_keypad_platform_data *pdata;
	struct led_trigger *bl_trigger;
	struct work_struct keypad_work;
	struct work_struct clamshell_work;
	struct platform_device *pdev;
	bool last_clamshell_state;
} microp_keypad;

static irqreturn_t microp_keypad_interrupt(int irq, void *handle)
{
	schedule_work(&microp_keypad.keypad_work);
	return IRQ_HANDLED;
}

static irqreturn_t microp_clamshell_interrupt(int irq, void *handle)
{
	schedule_work(&microp_keypad.clamshell_work);
	return IRQ_HANDLED;
}

static void microp_keypad_work(struct work_struct *work)
{
	uint8_t buffer[3] = {};
	uint8_t key = 0;
	bool slider_open = false;
	bool isdown;
	mutex_lock(&microp_keypad.lock);

	do {
		microp_ng_read(microp_keypad.client,
						MICROP_KSC_ID_SCANCODE, buffer, 2);

		key = buffer[0] & MICROP_KSC_SCANCODE_MASK;
		isdown = !(buffer[0] & MICROP_KSC_RELEASED_MASK);

		if (microp_keypad.pdata->read_modifiers) {
			microp_ng_read(microp_keypad.client,
						MICROP_KSC_ID_MODIFIER, buffer, 2);
			slider_open = !(buffer[1] & MICROP_KSC_CLAMSHELL_MASK);
			input_report_switch(microp_keypad.input, SW_LID, !slider_open);
		}

		DLOG(KERN_DEBUG "%s: scancode=%u\n", __func__, key);

		if (key != 0) {
			input_event(microp_keypad.input, EV_MSC, MSC_SCAN, key);
			if (key < microp_keypad.pdata->keypad_scancodes_size) {
				input_report_key(microp_keypad.input,
						microp_keypad.pdata->keypad_scancodes[key],
						isdown);

				DLOG(KERN_DEBUG "%s: scancode=%u keycode=%d\n", __func__,
						key, microp_keypad.pdata->keypad_scancodes[key]);
			}
		}
	} while (key);
	input_sync(microp_keypad.input);
	if (microp_keypad.last_clamshell_state != slider_open) {
		led_trigger_event(microp_keypad.bl_trigger,
			slider_open ? LED_FULL : LED_OFF);
		microp_keypad.last_clamshell_state = slider_open;
	}
	mutex_unlock(&microp_keypad.lock);
}

static void microp_clamshell_work(struct work_struct *work)
{
	int open = 1;

	mutex_lock(&microp_keypad.lock);
		open = !gpio_get_value(microp_keypad.pdata->gpio_clamshell);
		input_report_switch(microp_keypad.input, SW_LID, open);
		input_sync(microp_keypad.input);
		if (microp_keypad.last_clamshell_state != open) {
		led_trigger_event(microp_keypad.bl_trigger,
			open ? LED_FULL : LED_OFF);
		microp_keypad.last_clamshell_state = open;
		}
	mutex_unlock(&microp_keypad.lock);
}

static int microp_keypad_probe(struct platform_device *pdev)
{
	struct input_dev *input = NULL;
	struct i2c_client *client;
	struct microp_keypad_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0, i;

	printk(KERN_INFO MODULE_NAME ": Initializing MicroP keypad driver\n");

	client = dev_get_drvdata(&pdev->dev);

	if (!client) {
		printk(KERN_ERR MODULE_NAME ": failed to get i2c_client via drvdata\n");
		goto fail_invalid_pdata;
	}
	microp_keypad.client = client;

	if (pdev->id != -1) {
		printk(KERN_ERR MODULE_NAME ": device id != -1, aborting\n");
		goto fail_invalid_pdata;
	}

	if (pdata->irq_keypress < 0)
		goto fail_invalid_pdata;

	if (!pdata->keypad_scancodes || !pdata->keypad_scancodes_size)
		goto fail_invalid_pdata;

	if (pdata->init)
		ret = pdata->init(&pdev->dev);
	if (ret)
		goto fail_init;

	mutex_init(&microp_keypad.lock);
	microp_keypad.pdata = pdata;
	INIT_WORK(&microp_keypad.keypad_work, microp_keypad_work);
	// Initialize input device
	input = input_allocate_device();
	if (!input)
		goto fail_input_alloc;

	input->name = MODULE_NAME;

	// Tell input subsystem we can provide KEYs
	set_bit(EV_KEY, input->evbit);

	// Tell input subsystem to handle auto-repeat of keys for us
	set_bit(EV_REP, input->evbit);

	// Tell input subsystem we can provide overridable scancodes
	set_bit(EV_MSC, input->evbit);
	set_bit(MSC_SCAN, input->mscbit);

	if (pdata->read_modifiers || pdata->gpio_clamshell >= 0) {
		set_bit(EV_SW, input->evbit);
		input_set_capability(input, EV_SW, SW_LID);
	}

	input->keycodesize = sizeof(pdata->keypad_scancodes[0]);
	input->keycodemax = pdata->keypad_scancodes_size;
	input->keycode = pdata->keypad_scancodes;

	for (i = 0; i < input->keycodemax; i++)
	{
		if (pdata->keypad_scancodes[i] != KEY_RESERVED)
		{
			set_bit(pdata->keypad_scancodes[i], input->keybit);
		}
	}

	ret = input_register_device(input);
	if (ret)
		goto fail_input_register;

	microp_keypad.pdev = pdev;
	microp_keypad.input = input;
	platform_set_drvdata(pdev, &microp_keypad);

	ret = request_irq(pdata->irq_keypress, microp_keypad_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, MODULE_NAME, NULL);
	if (ret) {
		printk(KERN_ERR "Couldn't request keypress IRQ %d; error: %d\n",
				 pdata->irq_keypress, ret);
		goto fail_irq;
	}

	if (pdata->gpio_clamshell >= 0) {
		INIT_WORK(&microp_keypad.clamshell_work, microp_clamshell_work);

		ret = request_irq(pdata->irq_clamshell, microp_clamshell_interrupt,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"microp-keypad-sw", NULL);

		if (ret) {
			printk(KERN_ERR "Couldn't request clamshel IRQ %d; error: %d\n",
					 pdata->irq_clamshell, ret);
			goto fail_clamshell;
		}
	}

	led_trigger_register_simple("microp-keypad", &microp_keypad.bl_trigger);
	if (!microp_keypad.bl_trigger)
		goto fail_ledtrig;

	device_init_wakeup(&pdev->dev, 1);

	return 0;

fail_ledtrig:
	if (pdata->gpio_clamshell >= 0)
		free_irq(pdata->irq_clamshell, pdev);
fail_clamshell:
	free_irq(microp_keypad.pdata->irq_keypress, pdev);
fail_irq:
	input_unregister_device(input);
fail_init:
fail_input_register:
	input_free_device(input);
fail_input_alloc:
	if (pdata->exit)
		pdata->exit(&pdev->dev);
fail_invalid_pdata:
	return -EINVAL;
}

static int microp_keypad_remove(struct platform_device *pdev)
{
	free_irq(microp_keypad.pdata->irq_keypress, pdev);
	input_unregister_device(microp_keypad.input);
	input_free_device(microp_keypad.input);
	led_trigger_unregister_simple(microp_keypad.bl_trigger);

	if (microp_keypad.pdata->exit)
		microp_keypad.pdata->exit(&pdev->dev);

	microp_keypad.client = NULL;
	return 0;
}

#if CONFIG_PM
static int microp_keypad_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	if (microp_keypad.pdata->gpio_clamshell > 0)
		cancel_work_sync(&microp_keypad.clamshell_work);
	cancel_work_sync(&microp_keypad.keypad_work);
	led_trigger_event(microp_keypad.bl_trigger, LED_OFF);
	return 0;
}

static int microp_keypad_resume(struct platform_device *pdev)
{
	schedule_work(&microp_keypad.keypad_work);
	if (microp_keypad.pdata->gpio_clamshell > 0)
		schedule_work(&microp_keypad.clamshell_work);
	return 0;
}
#else
#define microp_keypad_suspend NULL
#define microp_keypad_resume NULL
#endif

static struct platform_driver microp_keypad_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe = microp_keypad_probe,
	.remove = microp_keypad_remove,
	.suspend = microp_keypad_suspend,
	.resume = microp_keypad_resume,

};

static int __init microp_keypad_init(void)
{
	printk(KERN_INFO "Registering MicroP keypad driver\n");
	return platform_driver_register(&microp_keypad_driver);
}

static void __exit microp_keypad_exit(void)
{
	printk(KERN_INFO "Unregistered MicroP keypad driver\n");
	platform_driver_unregister(&microp_keypad_driver);
}

MODULE_AUTHOR("Joe Hansche <madcoder@gmail.com>");
MODULE_DESCRIPTION("MicroP keypad driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");

module_init(microp_keypad_init);
module_exit(microp_keypad_exit);
