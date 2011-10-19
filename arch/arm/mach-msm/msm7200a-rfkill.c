/* linux/arch/arm/mach-msm/msm720xA-rfkill.c
 *
 * Copyright (C) 2010-2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
*/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/msm7200a_rfkill.h>
#include <mach/vreg.h>

static struct rfkill *bt_rfk;
static struct msm7200a_rfkill_pdata *rfk_pdata;

static struct msm_gpio msm_hsuart_2_on_table[] = {
    {	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_RTS, 4,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT RTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_CTS, 4,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT CTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_RX,  4,
			GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT RXD"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_TX,  2,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT TXD"},
};
static struct msm_gpio msm_hsuart_2_off_table[] = {
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_RTS, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT RTS"},
    {	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_CTS, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		.label = "BT CTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_RX,  0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
		.label = "BT RXD"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART2DM_TX,  0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "BT TXD"},
};

static struct msm_gpio msm_hsuart_1_on_table[] = {
    {	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_RTS, 2,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT RTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_CTS, 2,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT CTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_RX,  2,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT RXD"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_TX,  3,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT TXD"},
};

static struct msm_gpio msm_hsuart_1_off_table[] = {
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_RTS, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT RTS"},
    {	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_CTS, 0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT CTS"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_RX,  0,
			GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT RXD"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_UART1DM_TX,  0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),
		.label = "BT TXD"},
};

static struct msm_gpio msm_pcm_on_table[] = {
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_DOUT,	1,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_DOUT"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_DIN,	1,
			GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_DIN"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_SYNC,	2,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_SYNC"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_CLK,	2,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_CLK"},
};

static struct msm_gpio msm_pcm_off_table[] = {
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_DOUT,	1,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_DOUT"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_DIN,	1,
			GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_DIN"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_SYNC,	2,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_SYNC"},
	{	.gpio_cfg = GPIO_CFG(MSM7200A_PCM_CLK,	2,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		.label = "PCM_CLK"},
};

static void msm_hs_uart_configure(unsigned id, bool enable) {
	switch (id) {
		case 0:
		break;

		case 1:
			if (enable) {
				msm_gpios_enable(msm_hsuart_1_on_table,
					ARRAY_SIZE(msm_hsuart_1_on_table));
			}
			else {
				msm_gpios_disable(msm_hsuart_1_off_table,
					ARRAY_SIZE(msm_hsuart_1_off_table));
			}
		break;

		case 2:
			if (enable) {
				msm_gpios_enable(msm_hsuart_2_on_table,
					ARRAY_SIZE(msm_hsuart_2_on_table));
			}
			else {
				msm_gpios_disable(msm_hsuart_2_off_table,
					ARRAY_SIZE(msm_hsuart_2_off_table));
			}
		break;
	}
}

static void msm_pcm_configure(bool enable) {
	if (enable) {
		msm_gpios_enable(msm_pcm_on_table,
			ARRAY_SIZE(msm_pcm_on_table));
	}
	else {
		msm_gpios_enable(msm_pcm_off_table,
			ARRAY_SIZE(msm_pcm_off_table));
	}
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
		msm_hs_uart_configure(rfk_pdata->uart_number, true);
		if (rfk_pdata->set_power)
			rfk_pdata->set_power(data, blocked);
		if (rfk_pdata->configure_bt_pcm)
			msm_pcm_configure(true);
	} else {
		if (rfk_pdata->configure_bt_pcm)
			msm_pcm_configure(false);
		if (rfk_pdata->set_power)
			rfk_pdata->set_power(data, blocked);
		msm_hs_uart_configure(rfk_pdata->uart_number, false);
	}
	return 0;
}

static struct rfkill_ops msm_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static ssize_t msm_rfkill_bdaddr_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	if (!rfk_pdata->get_bdaddr) {
		return -ENOTSUPP;
	}

	return sprintf(ret_buf, "%s\n", rfk_pdata->get_bdaddr());
}

static ssize_t msm_rfkill_bdaddr_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	return -ENOTSUPP;
}

static DEVICE_ATTR(bdaddr, 0644, msm_rfkill_bdaddr_get,
	msm_rfkill_bdaddr_set);

static int msm_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct msm7200a_rfkill_pdata *pdata = pdev->dev.platform_data;

	if (!pdata) {
		printk(KERN_ERR "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	if (pdata->init) {
		rc = pdata->init(pdev);
		if (rc)
			goto fail_platform_init;
	}

	rfk_pdata = pdata;

	switch (rfk_pdata->uart_number) {
		case 0:
		break;

		case 1:
		rc = msm_gpios_request(msm_hsuart_1_off_table,
			ARRAY_SIZE(msm_hsuart_1_off_table));
		break;

		case 2:
		rc = msm_gpios_request(msm_hsuart_2_off_table,
			ARRAY_SIZE(msm_hsuart_2_off_table));
		break;

		default:
			printk(KERN_ERR "%s: unknown uart number %d\n",
					__func__, rfk_pdata->uart_number);
			rc = -EINVAL;
	}

	if (rc)
		goto fail_gpios;

	if (pdata->configure_bt_pcm) {
		rc = msm_gpios_request(msm_pcm_off_table,
			ARRAY_SIZE(msm_pcm_off_table));
		if (rc)
			goto fail_pcm_gpios;
	}

	bt_rfk = rfkill_alloc(pdata->rfkill_name ? pdata->rfkill_name : "msm_bt",
		&pdev->dev, RFKILL_TYPE_BLUETOOTH,
		&msm_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto fail_rfk_alloc;
	}

	rfkill_set_states(bt_rfk, true, false);

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto fail_rfk_reg;

	if (pdata->get_bdaddr) {
		rc = device_create_file(&pdev->dev, &dev_attr_bdaddr);
		if (rc)
			goto fail_bdaddr_attr;
	}

	printk(KERN_INFO "%s: initialized\n", __func__);
	return 0;

fail_bdaddr_attr:
	rfkill_unregister(bt_rfk);
fail_rfk_reg:
	rfkill_destroy(bt_rfk);
fail_rfk_alloc:
	if (rfk_pdata->configure_bt_pcm)
		msm_gpios_disable_free(msm_pcm_off_table,
			ARRAY_SIZE(msm_pcm_off_table));
fail_pcm_gpios:
	switch (rfk_pdata->uart_number) {
		case 0:
		break;

		case 1:
		msm_gpios_disable_free(msm_hsuart_1_off_table,
			ARRAY_SIZE(msm_hsuart_1_off_table));
		break;

		case 2:
		msm_gpios_disable_free(msm_hsuart_2_off_table,
			ARRAY_SIZE(msm_hsuart_2_off_table));
		break;
	}
fail_gpios:
	if (pdata->exit)
		pdata->exit(pdev);
fail_platform_init:
	printk(KERN_ERR "%s: init failure %d\n", __func__, rc);
	return rc;
}

static int msm_rfkill_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	if (rfk_pdata->exit)
		rfk_pdata->exit(pdev);

	if (rfk_pdata->configure_bt_pcm)
		msm_gpios_disable_free(msm_pcm_off_table,
			ARRAY_SIZE(msm_pcm_off_table));

	switch (rfk_pdata->uart_number) {
		case 0:
		break;

		case 1:
		msm_gpios_disable_free(msm_hsuart_1_off_table,
			ARRAY_SIZE(msm_hsuart_1_off_table));
		break;

		case 2:
		msm_gpios_disable_free(msm_hsuart_2_off_table,
			ARRAY_SIZE(msm_hsuart_2_off_table));
		break;
	}
	rfk_pdata = NULL;
	return 0;
}

static struct platform_driver msm_rfkill_driver = {
	.probe = msm_rfkill_probe,
	.remove = msm_rfkill_remove,
	.driver = {
		.name = "msm7200a_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init msm_rfkill_init(void)
{
	return platform_driver_register(&msm_rfkill_driver);
}

static void __exit msm_rfkill_exit(void)
{
	platform_driver_unregister(&msm_rfkill_driver);
}

module_init(msm_rfkill_init);
module_exit(msm_rfkill_exit);
MODULE_DESCRIPTION("MSM7200A simple rfkill driver");
MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_LICENSE("GPL");
