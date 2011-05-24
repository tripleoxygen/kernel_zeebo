#ifndef _MICROP_KEYPAD_H
#define _MICROP_KEYPAD_H

#include <linux/platform_device.h>

#define MICROP_KSC_RELEASED_MASK	0x80
#define MICROP_KSC_SCANCODE_MASK	(MICROP_KSC_RELEASED_MASK - 1)
#define MICROP_KSC_CLAMSHELL_MASK	0x80

#define MICROP_KSC_ID_SCANCODE		0x10
#define MICROP_KSC_ID_MODIFIER		0x11

enum microp_key_state {
	KEY_RELEASED,
};

struct microp_keypad_platform_data {
	int *keypad_scancodes;
	int keypad_scancodes_size;
	int irq_keypress;
	bool read_modifiers;
	int gpio_clamshell;
	int irq_clamshell;
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
};

#endif
