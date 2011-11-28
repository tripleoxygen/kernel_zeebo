/* linux/arch/arm/mach-msm/board-htcrhodium.h
 */
#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCRHODIUM_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCRHODIUM_H

#include <mach/board.h>


#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

#define RHODIUM_BT_SHUTDOWN_N	31
#define RHODIUM_BT_nRST			91
#define RHODIUM_BT_HOST_WAKE	94
#define RHODIUM_BT_WAKE			35

#define RHODIUM_CABLE_IN1		42
#define RHODIUM_CABLE_IN2		45
#define RHODIUM_H2W_CLK			46
#define RHODIUM_H2W_DATA		92
#define RHODIUM_H2W_UART_MUX	103
#define RHODIUM_GSENROR_MOT		49	/* microp interrupt for 3.5mm detect */
#define RHODIUM_GPIO_UP_RESET_N	43

#define RHODIUM_GPIO_PROXIMITY_EN		102
#define RHODIUM_GPIO_PROXIMITY_INT_N	90

#define RHODIUM_BAT_IRQ			28  // GPIO IRQ
#define RHODIUM_USB_AC_PWR		32
#define RHODIUM_CHARGE_EN_N		44
#define RHODIUM_KPD_IRQ			27  //Keyboard IRQ
#define RHODIUM_KB_SLIDER_IRQ		37  //Keyboard Slider IRQ //Currently Unknown, using stylus detect GPIO right now (37).
#define RHODIUM_BKL_PWR			86  //Keyboard blacklight  //Currently Unknown if this is right

#define RHODIUM_END_KEY			18
#define RHODIUM_VOLUMEUP_KEY		39
#define RHODIUM_VOLUMEDOWN_KEY		40
#define RHODIUM_POWER_KEY		83  //Power key
#define RHODIUM_SPKR_PWR        84
#define RHODIUM_HS_AMP_PWR      85

#define RHOD_LCD_RST			82
#define RHOD_LCD_VSYNC			97
#define RHOD_LCD_PWR1			98
#define RHOD_LCD_PWR2			99

#define RHODIUM_USBPHY_RST		100

/* MicroP */
#define RHOD_MICROP_KLT_VERSION_REG		0x30
#define RHOD_MICROP_KLT_VERSION			0x0a88
#define RHOD_MICROP_KSC_VERSION_REG		0x12
#define RHOD_MICROP_KSC_VERSION			0x0488

/* MicroP LED & Audio */
#define RHOD_MICROP_KLT_LED_STATE		0x25

#define RHOD_MICROP_KLT_CAPS_BIT		(1<<0)
#define RHOD_MICROP_KLT_FN_BIT			(1<<1)
#define RHOD_MICROP_KLT_DUALMIC_EN_BIT	(1<<2)
#define RHOD_MICROP_KLT_CODEC_EN_BIT	(1<<4)

/* MicropP Keypad */
#define RHOD_MICROP_KSC_LED_STATE		0x30
#define RHOD_MICROP_KSC_LED_BRIGHTNESS	0x32

#endif 
