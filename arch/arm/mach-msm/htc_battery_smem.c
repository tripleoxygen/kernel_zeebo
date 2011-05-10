/* arch/arm/mach-msm/htc_battery_smem.c
 * Based on: htc_battery.c by HTC and Google
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <linux/io.h>

#include "proc_comm_wince.h"

#include <mach/msm_iomap.h>
#include <mach/htc_battery.h>
#include <mach/htc_battery_smem.h>

static struct wake_lock vbus_wake_lock;
static struct work_struct bat_work;
static int bat_suspended = 0;

static int htc_pcb_id = 0xFF, htc_hwboard_id = 0xFF, batt_vref = 0, batt_vref_half = 0;

static int g_usb_online;
static int fake_charger=0;
module_param_named(fake, fake_charger, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* boot option for minimum battery volume */
static int vol_min=0;
module_param_named(vol_min, vol_min, int, S_IRUGO | S_IWUSR | S_IWGRP);
static int vol_max=100;
module_param_named(vol_max, vol_max, int, S_IRUGO | S_IWUSR | S_IWGRP);


enum {
	DEBUG_BATT	= 1<<0,
	DEBUG_CABLE	= 1<<1,
	DEBUG_LOG	= 1<<2,
};
static int debug_mask = 0;
module_param_named(debug, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* Default battery will be the generic 1800mAh
 * batt_param will be set after battery detection but must be initialized
 * because it may be used before battery is correctly detected
 * sBattery_Parameters is defined in htc_battery_smem.h.
 * sBatParams_1800mAh and other are also in this file. To add a new battery profile, just add
 * it in the htc_battery_smem.h and modify the battery detection routine of the device
 */
static struct sBattery_Parameters* batt_param = (struct sBattery_Parameters*)&sBatParams_1800mAh;

#define MODULE_NAME "htc_battery"

#define TRACE_BATT 1

#if TRACE_BATT
 #define BATT(x...) printk(KERN_INFO "[BATT] " x)
#else
 #define BATT(x...) do {} while (0)
#endif

#define BATT_ERR(x...) printk(KERN_ERR "[BATT_ERR] " x)


/* battery detail logger */
#define HTC_BATTERY_BATTLOGGER		1

#if HTC_BATTERY_BATTLOGGER
 #include <linux/rtc.h>
 #define BATTLOG(x...) do { \
 struct timespec ts; \
 struct rtc_time tm; \
 getnstimeofday(&ts); \
 rtc_time_to_tm(ts.tv_sec, &tm); \
 printk(KERN_INFO "[BATTLOG];%d-%02d-%02d %02d:%02d:%02d", \
 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
 tm.tm_hour, tm.tm_min, tm.tm_sec); \
 printk(";" x); \
 } while (0)
#else
 #define BATTLOG(x...) do {} while (0)
#endif


/* module debugger */
#define HTC_BATTERY_DEBUG		1
#define BATTERY_PREVENTION		1

/* Enable this will shut down if no battery */
#define ENABLE_BATTERY_DETECTION	0

typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

/* This order is the same as htc_power_supplies[]
 * And it's also the same as htc_cable_status_update()
 */
typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C)corrected value from formula and ADC */
	s32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
	u32 batt_tempRAW;	/* Battery Temperature (C) from formula and ADC */
};

struct htc_battery_info {
	int present;
	unsigned long update_time;

	/* lock to protect the battery info */
	struct mutex lock;

	struct battery_info_reply rep;
	smem_batt_t *resources;
};

static struct htc_battery_info htc_batt_info;

static unsigned int cache_time = 1000;

static int htc_battery_initial = 0;
static bool not_yet_started = true;

/* simple maf filter stuff - how much old values should be used for recalc ...*/
#define BATT_MAF_SIZE 6
static short volt_maf_buffer[BATT_MAF_SIZE];
static short volt_maf_size = 0;
static short volt_maf_last = 0;

static void maf_add_value( short volt )
{
	// check if we need to correct the index
	if ( volt_maf_last == BATT_MAF_SIZE-1 )
		volt_maf_last = 0;

	// add value to filter buffer
	volt_maf_buffer[volt_maf_last] = volt;
	volt_maf_last++;

	if ( volt_maf_size != BATT_MAF_SIZE-1 )
		volt_maf_size++;	
}

/* calculated on the fly.... no caching */
static short maf_get_avarage(void)
{
	int i;
	int maf_temp;

	// make sure we only do it when we have data
	if ( volt_maf_size == 0 )
		return 0;

	// no need todo the avaraging
	if ( volt_maf_size == 1 )
		return volt_maf_buffer[0];

	// our start value is the first sample
	maf_temp = volt_maf_buffer[0];

	for (i=1; i < volt_maf_size; i++) {
		maf_temp = ( maf_temp + volt_maf_buffer[i] ) / 2;		
	}

	return maf_temp;
}

static void maf_clear(void)
{
	int i;
	for ( i = 0; i < BATT_MAF_SIZE;i++ )
		volt_maf_buffer[i] = 0;

	volt_maf_size = 0;
	volt_maf_last = 0;
}

/* ADC linear correction numbers.
 */
static u32 htc_adc_a = 0;					// Account for Divide Resistors
static u32 htc_adc_b = 0;
static u32 htc_adc_range = 0x1000;	// 12 bit adc range correction.
static u32 batt_vendor = 0;

#define GET_BATT_ID         readl(MSM_SHARED_RAM_BASE + 0xFC0DC)
#define GET_ADC_VREF        readl(MSM_SHARED_RAM_BASE + 0xFC0E0)
#define GET_ADC_0_5_VREF    readl(MSM_SHARED_RAM_BASE + 0xFC0E4)
#define GET_PCB_ID          readb(MSM_SHARED_RAM_BASE + 0xFC0EF)
#define GET_HWBOARD_ID      readl(MSM_SHARED_RAM_BASE + 0xFC048)

static int get_battery_id_detection( struct battery_info_reply *buffer );
static int htc_get_batt_info( struct battery_info_reply *buffer );

static int init_battery_settings( struct battery_info_reply *buffer ) {
	//u32 batt_vref = 0;
	//u32 batt_vref_half = 0;

	if ( buffer == NULL )
		return -EINVAL;

	if ( htc_get_batt_info( buffer ) < 0 )
		return -EINVAL;

	mutex_lock( &htc_batt_info.lock );

	batt_vref = GET_ADC_VREF;
	batt_vref_half = GET_ADC_0_5_VREF;
	htc_pcb_id = GET_PCB_ID;
	htc_hwboard_id = GET_HWBOARD_ID;


	if ( batt_vref - batt_vref_half >= 500 ) {
		// set global correction var
		htc_adc_a = 625000 / ( batt_vref - batt_vref_half );
		htc_adc_b = 1250000 - ( batt_vref * htc_adc_a );
	}


	// calculate the current adc range correction.
	if( machine_is_htctopaz() ) {
		/* extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz */
		htc_adc_range = ( ( batt_vref + 1250 ) * 0x1000 ) / 2500;
		if(htc_adc_range < 1000) {
			htc_adc_range = 0x1000;
		}
	} else {
		/* original formula, also found in battdrvr.dll ver.1.93.0.0 of HTC diamond */
		htc_adc_range = ( batt_vref * 0x1000 ) / 1250;
	}

	if ( get_battery_id_detection( buffer ) < 0 ) {
		mutex_unlock(&htc_batt_info.lock);
		if(debug_mask&DEBUG_LOG)
			BATT_ERR("Critical Error on: get_battery_id_detection: VREF=%d; 0.5-VREF=%d; PCB_ID=0x%02x; HWBOARD_ID=0x%08x; ADC_A=%d; ADC_B=%d; htc_adc_range=%d; batt_id=%d; batt_vendor=%d; full_bat=%d\n", \
			batt_vref, batt_vref_half, htc_pcb_id, htc_hwboard_id, htc_adc_a, htc_adc_b, htc_adc_range, buffer->batt_id, batt_vendor, buffer->full_bat);
		return -EINVAL;
	}

	mutex_unlock(&htc_batt_info.lock);
	if(debug_mask&DEBUG_LOG)
		BATT("init_battery_settings: VREF=%d; 0.5-VREF=%d; PCB_ID=0x%02x; HWBOARD_ID=0x%08x; ADC_A=%d; ADC_B=%d; htc_adc_range=%d; batt_id=%d; batt_vendor=%d; full_bat=%d\n", \
		batt_vref, batt_vref_half, htc_pcb_id, htc_hwboard_id, htc_adc_a, htc_adc_b, htc_adc_range, buffer->batt_id, batt_vendor, buffer->full_bat);

	return 0;
}

/* Device dependent battery detection, may be moved to board files ?? */
static void htcraphael_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;  // apply the adc range correction.
	//batt_id = ( ( htc_adc_a * batt_id ) + htc_adc_b )  / 1000;
	//buffer->batt_id = batt_id;

	if ( buffer->batt_id > 800 ) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100; // hack
	}

	// battery capacity part is a hack..
	if ( buffer->batt_id < 280 ) {
		batt_vendor = 1;
		buffer->full_bat = 1800000;
	} else {
		batt_vendor = 2;
		buffer->full_bat = 1340000;
	}
}

/* Device dependent battery detection, may be moved to board files ?? */
static void htcblackstone_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;  // apply the adc range correction.

	if ( buffer->batt_id > 800 ) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100; // hack
	}

	// battery capacity part is a hack..
	if ( buffer->batt_id < 280 ) {
		batt_vendor = 1;
		buffer->full_bat = 1500000;
	} else {
		batt_vendor = 2;
		buffer->full_bat = 1350000;
	}
}

static void htckovsky_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;  // apply the adc range correction.

	if ( buffer->batt_id > 800 ) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100; // hack
	}

	// battery capacity part is a hack..
	if ( buffer->batt_id < 280 ) {
		batt_vendor = 1;
		buffer->full_bat = 2000000;
	} else {
		batt_vendor = 2;
		buffer->full_bat = 1500000;
	}
}

static void htcrhodium_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;  // apply the adc range correction.

	if ( buffer->batt_id > 800 ) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100; // hack
	}

	// battery capacity part is a hack..
	if ( buffer->batt_id < 280 ) {
		batt_vendor = 1;
		buffer->full_bat = 1500000;
	} else {
		batt_vendor = 2;
		buffer->full_bat = 2200000;
	}
}


/* Hardcoded values extracted from nk.exe of official HTC diamond ROM 1.93.483.4 */
static int htcdiamond_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;

	if ( buffer->batt_id > 800 ) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100;
	} else if ( buffer->batt_id <= 400 ) {
		batt_vendor = 1;
		buffer->full_bat = 1340000;
	} else {
		batt_vendor = 2;
		buffer->full_bat = 900000;
	}
	return 0;
}

/* Hardcoded values extracted from nk.exe of official HTC topaz ROM 2.16.406.1 */
static int htctopaz_battery_id_detection( struct battery_info_reply *buffer )
{
	buffer->batt_id *= 10;
	if(buffer->batt_id >= 530) {
		batt_vendor = 0xFF;
		buffer->full_bat = 100;
	} else if (buffer->batt_id >= 450 ) {
		batt_vendor = 5;
		buffer->full_bat = 2150000;
	} else if (buffer->batt_id >= 350 ) {
		batt_vendor = 4;
		buffer->full_bat = 1350000;
	} else if (buffer->batt_id >= 240 ) {
		batt_vendor = 3;
		buffer->full_bat = 1350000;
	} else if (buffer->batt_id >= 100 ) {
		batt_vendor = 2;
		buffer->full_bat = 1110000;
	} else {
		batt_vendor = 1;
		buffer->full_bat = 1110000;
	}
	return 0;
}

static int get_battery_id_detection( struct battery_info_reply *buffer ) {
	// dono incorparate these in the battery info.
	u32 batt_id;
	struct msm_dex_command dex;

	dex.cmd = PCOM_GET_BATTERY_ID;
	msm_proc_comm_wince( &dex, 0 );

	batt_id = GET_BATT_ID;

	/* buffer->batt_id will be overwritten on next battery reading so we can use it as
	 * a temp variable to pass it to machine specific battery detection
	 */
	buffer->batt_id = batt_id;

	if( machine_is_htctopaz() ) {
		htctopaz_battery_id_detection( buffer );
	} else if( machine_is_htcdiamond() ) {
		htcdiamond_battery_id_detection( buffer );
	} else if( machine_is_htcblackstone() ) {
		htcblackstone_battery_id_detection( buffer );
	} else if( machine_is_htckovsky() ) {
		htckovsky_battery_id_detection( buffer );
	} else if( machine_is_htcrhodium() ) {
		htcrhodium_battery_id_detection( buffer );
	} else {
		htcraphael_battery_id_detection( buffer ); /* Original values of this driver, used as generic */
	}
	// XXX : Add battery parameters for each devices if not same as generic parameters (like for diamond)
	if( machine_is_htcdiamond() ) {
		if ( batt_vendor != 1 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_diamond[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_diamond[0];
		}
	} else if ( machine_is_htcraphael() ) {
		if ( batt_vendor != 1 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_raphael[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_raphael[0];
		}
	} else if ( machine_is_htcblackstone() ) {
		if ( batt_vendor != 1 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_blackstone[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_blackstone[0];
		}
	} else if( machine_is_htctopaz() ) {
		if ( (batt_vendor <= 5) && (batt_vendor) ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_topaz[batt_vendor - 1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_topaz[0];
		}
	} else if ( machine_is_htckovsky() ) {
		if ( batt_vendor != 2 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_kovsky[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_kovsky[0];
		}
	} else if ( machine_is_htcrhodium() ) {
                if ( ( batt_vendor == 1 ) && ( batt_vref == 1254 ) && ( htc_hwboard_id == 0x80 ) ) /* rhod300-1500 */
			batt_param = (struct sBattery_Parameters*) sBatParams_rhodium[2];
		else if ( batt_vendor != 1 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_rhodium[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_rhodium[0];
		}
	} else {
		if ( batt_vendor != 1 ) {
			batt_param = (struct sBattery_Parameters*) sBatParams_generic[1];
		} else {
			batt_param = (struct sBattery_Parameters*) sBatParams_generic[0];
		}
	}

	return 0;
}

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

/* HTC dedicated attributes */
static ssize_t htc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

/* -------------------------------------------------------------------------- */

#if defined(CONFIG_DEBUG_FS)
int htc_battery_set_charging(batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
	return htc_battery_set_charging((batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
	struct dentry *dent;

	/*prepared for future: to calc via ds2746_battery.c+ boardfiles  from Alex */
	//if (machine_is_htckovsky())
	//	return -1;

	dent = debugfs_create_dir("htc_battery", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

	return 0;
}

device_initcall(batt_debug_init);
#endif

static int init_batt_gpio(void)
{
	if (gpio_request(htc_batt_info.resources->gpio_battery_detect, "batt_detect") < 0)
		goto gpio_failed;
	if (gpio_request(htc_batt_info.resources->gpio_charger_enable, "charger_en") < 0)
		goto gpio_failed;
	if (gpio_request(htc_batt_info.resources->gpio_charger_current_select, "charge_current") < 0)
		goto gpio_failed;
	if ( machine_is_htckovsky() || machine_is_htctopaz() )
		if (gpio_request(htc_batt_info.resources->gpio_ac_detect, "ac_detect") < 0)
			goto gpio_failed;

	return 0;

gpio_failed:
	return -EINVAL;
}

/*
 *	battery_charging_ctrl - battery charing control.
 * 	@ctl:			battery control command
 *
 */
static int battery_charging_ctrl(batt_ctl_t ctl)
{
	int result = 0;

	switch (ctl) {
	case DISABLE:
		if(debug_mask&DEBUG_CABLE)
			BATT("charger OFF\n");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 1);
		break;
	case ENABLE_SLOW_CHG:
		if(debug_mask&DEBUG_CABLE)
			BATT("charger ON (SLOW)\n");
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_current_select, 0);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 0);
		break;
	case ENABLE_FAST_CHG:
		if(debug_mask&DEBUG_CABLE)
			BATT("charger ON (FAST)\n");
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_current_select, 1);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 0);
		break;
	default:
		BATT_ERR("Not supported battery ctr called.!\n");
		result = -EINVAL;
		break;
	}

	return result;
}

int htc_battery_set_charging(batt_ctl_t ctl)
{
	int rc;

	if ((rc = battery_charging_ctrl(ctl)) < 0)
		goto result;

	if (!htc_battery_initial) {
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	} else {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
		mutex_unlock(&htc_batt_info.lock);
	}
result:
	return rc;
}

int htc_battery_status_update(u32 curr_level)
{
	int notify;
	unsigned charge = 0;

	if (!htc_battery_initial)
		return 0;

	mutex_lock(&htc_batt_info.lock);
	notify = (htc_batt_info.rep.level != curr_level);
	htc_batt_info.rep.level = curr_level;
	/* If battery is above 95%, switch to slow charging.
	 * If battery is below 90%, switch to fast charging.
	 */
	if (curr_level > 95 && htc_batt_info.rep.charging_enabled == ENABLE_FAST_CHG)
		charge = ENABLE_SLOW_CHG;
	else if (curr_level < 90 && htc_batt_info.rep.charging_enabled == ENABLE_SLOW_CHG)
		charge = ENABLE_FAST_CHG;
	mutex_unlock(&htc_batt_info.lock);

	if (notify)
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	if (charge)
		htc_battery_set_charging(charge);
	return 0;
}

static bool on_battery;
int htc_cable_status_update(int status)
{
	int rc = 0;
	unsigned source;
	unsigned last_source;
	unsigned vbus_status;
	unsigned charger;
	vbus_status = readl(MSM_SHARED_RAM_BASE+0xfc00c);

	if (!htc_battery_initial)
		return 0;
	
	mutex_lock(&htc_batt_info.lock);
	if(vbus_status && g_usb_online) {
		status=CHARGER_USB;	/* vbus present, usb connection online (perhaps breaks kovsky ?) */
		on_battery = false;
		charger = ENABLE_FAST_CHG;
	} else if (vbus_status && !g_usb_online) {
		status=CHARGER_AC;	/* vbus present, no usb */
		on_battery = false;
		charger = ENABLE_FAST_CHG;
	} else {
		g_usb_online = 0;
		status=CHARGER_BATTERY;
		on_battery = true;
		charger = DISABLE;
	}

	if ( fake_charger && (status == CHARGER_BATTERY) ) {
		status=CHARGER_USB;
        }

	last_source = htc_batt_info.rep.charging_source;

	switch(status) {
	case CHARGER_BATTERY:
		if(debug_mask&DEBUG_CABLE)
			BATT("cable NOT PRESENT\n");
		htc_batt_info.rep.charging_source = CHARGER_BATTERY;
		break;
	case CHARGER_USB:
		if(debug_mask&DEBUG_CABLE)
			BATT("cable USB\n");
		htc_batt_info.rep.charging_source = CHARGER_USB;
		break;
	case CHARGER_AC:
		if(debug_mask&DEBUG_CABLE)
			BATT("cable AC\n");
		htc_batt_info.rep.charging_source = CHARGER_AC;
		break;
	default:
		BATT_ERR("%s - Not supported cable status received!\n", __FUNCTION__);
		rc = -EINVAL;
	}
	source = htc_batt_info.rep.charging_source;
	mutex_unlock(&htc_batt_info.lock);

	if (charger == ENABLE_FAST_CHG && htc_batt_info.rep.level > 95)
		charger = ENABLE_SLOW_CHG;
	htc_battery_set_charging(charger);
	msm_hsusb_set_vbus_state((source==CHARGER_USB) || (source==CHARGER_AC));

	if (  source == CHARGER_USB || source==CHARGER_AC ) {
		wake_lock(&vbus_wake_lock);
	} else if(last_source != source) {
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ / 2);
	} else {
		wake_unlock(&vbus_wake_lock);
	}

	/* make sure that we only change the powersupply state if we really have to */
	if (source == CHARGER_BATTERY || last_source == CHARGER_BATTERY)
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	if (source == CHARGER_USB || last_source == CHARGER_USB)
		power_supply_changed(&htc_power_supplies[CHARGER_USB]);
	if (source == CHARGER_AC || last_source == CHARGER_AC)
		power_supply_changed(&htc_power_supplies[CHARGER_AC]);

	return rc;
}

/* A9 reports USB charging when helf AC cable in and China AC charger. */
/* Work arround: notify userspace AC charging first,
and notify USB charging again when receiving usb connected notification from usb driver. */
void notify_usb_connected(int online)
{
	g_usb_online = online;
	if (not_yet_started) return;
	
	mutex_lock(&htc_batt_info.lock);
	if (online && htc_batt_info.rep.charging_source == CHARGER_AC) {
		mutex_unlock(&htc_batt_info.lock);
		htc_cable_status_update(CHARGER_USB);
		mutex_lock(&htc_batt_info.lock);
	} else if (online) {
		BATT("warning: usb connected but charging source=%d\n", htc_batt_info.rep.charging_source);
	}
	mutex_unlock(&htc_batt_info.lock);
}

static void htc_bat_work(struct work_struct *work) {
	int rc = 0;
	int ac_detect = 1;

	if (htc_batt_info.resources->gpio_ac_detect == 0)
		return;

	ac_detect = gpio_get_value(htc_batt_info.resources->gpio_ac_detect);
	if (debug_mask & DEBUG_CABLE) BATT("ac_detect=%d\n", ac_detect);
	if (ac_detect == 0) {
		rc = battery_charging_ctrl(ENABLE_SLOW_CHG);
		if (debug_mask & DEBUG_CABLE) BATT("charging enable rc=%d\n", rc);
	} else {
		rc = battery_charging_ctrl(DISABLE);
		if (debug_mask & DEBUG_CABLE) BATT("charging disable rc=%d\n", rc);
	}
}

#if 0
static short battery_table_4[] = {
	0,      0,
	0xe11,	0,
	0xe1e,	5,
	0xe3c,	10,
	0xe7e,	20,
	0xe97,	30,
	0xeab,	40,
	0xec1,	50,
	0xed5,	60,
	0xf0c,	70,
	0xf43,	80,
	0xf93,	90,
	0xfcb,  100,
	0x1000, 100,
	0,	0
};
#endif

/* 2 byte alligned */
struct htc_batt_info_u16 {
	volatile u16 batt_id;
	volatile u16 batt_temp;
	volatile u16 batt_vol;
	volatile s16 batt_charge;
	volatile u16 batt_discharge;
};

/* 4 byte alligned */
struct htc_batt_info_u32 {
	volatile u32 batt_id;
	volatile u32 batt_temp;
	volatile u32 batt_vol;
	volatile s32 batt_charge;
	volatile u32 batt_discharge;
};

//#define TEST_OLD_TEMP_CORR

#ifdef TEST_OLD_TEMP_CORR
/* todo convert to more readable code : see batt_current_temp_correction */
static int batt_current_temp_correction_orig(s64 R0/* temp */, s64 R1, s64 R2)
{
		s64 R3;
		s64 R4;

		if ( R0 > 250 ) {
			R0 = 0;
		} else {
			R1 = abs((int)R1);
			R3 = 250 - R0;
			R1 = R3 * R1;
			R0 = R1 * R2;
			R4 = R0 * 8;

			R1 = (R4 * 0x32C88B7BLL) >> 32;
			R3 = (R0 * 8) - R1;
			R3 = R1 + ( R3 / 2 );
			R0 = R3 >> 20;
		}

		R1 = R2 * 8;
		R3 = (R1 * 0xCCCCCCCD) >> 32; // part of div 10
		R3 = R3 >> 3;

		if (R0 < R3)
			R0 = R3;

		return R0;
}
#endif

static int batt_current_temp_correction(int temp, int ccurrent, int temp_correction_factor)
{
	int correction;
	int max_correction;
	int tmp_corr;

	if ( temp > 250 ) {
		correction = 0;
	} else {
		correction = 8 * ( 250 - temp ) * abs((int)ccurrent) * temp_correction_factor; /* R4 */

		tmp_corr = (correction * 0x32C88B7BLL) >> 32;
		tmp_corr += ( correction - tmp_corr ) / 2;
		correction = tmp_corr >> 20;
		/* The lines above are equal to :
		 * corrected_temp = (int) corrected_temp / 1750000;
		 */
	}

	max_correction = ((s64)(temp_correction_factor * 8) * 0xCCCCCCCD) >> 32; // part of div 10
	max_correction = max_correction >> 3;
	/* The lines above are equal to :
	 * max_correction = (int) temp_correction_factor / 1.25 = (temp_correction_factor * 8) / 10;
	 */

	if (correction > max_correction)
		correction = max_correction;

	return correction;
}

/* Common routine to retrieve temperature from lookup table */
static int htc_battery_temperature_lut( int av_index )
{
	// everything below 8 is HOT
	if ( av_index < 8 )
		av_index = 8;

	// max size of the table, everything higher than 1347 would
	// cause the battery to freeze in a instance.
	if ( av_index > 1347 )
		av_index = 1347;

	//if(temp_table[ av_index - 8 ] > 330)
	//	//>60°C ? You're already burnt, you don't care about actual temperature.
	//	return 280;//7°C

	return temp_table[ av_index - 8 ];

}

int iStabilizedVoltageLevel = 0xFF;

//used for last charge_status
#define CHARGE_STATUS_AGESIZE 6
static int charge_status_age[CHARGE_STATUS_AGESIZE];
//used for extreme values
static int corrected_volt_old;

/* Common routine to compute the battery level */
static int old_level = 100;
static void htc_battery_level_compute( struct battery_info_reply *buffer )
{
	//temp result for batt_vol
	int result = 0;
	//only used to see which logc will be used ...
	int calc_algo = 0;
	//trigger for next loops
	int i;
	int volt;
	int ccurrent;
	int volt_discharge_resistor;
	int corrected_volt;
	int temp = buffer->batt_temp;
	int temp_correct_volt = 0;	
	int corr_return = 0;
	volt = buffer->batt_vol;	
	ccurrent = buffer->batt_current;	

	/* aging, not to calc with first values after charging status will be changed */
	for ( ( i = CHARGE_STATUS_AGESIZE - 1); i > 0; i--) {
		charge_status_age[i] = charge_status_age[(i - 1)];
	}
	charge_status_age[0] = buffer->charging_enabled+1;// 0 will be used on empty values / 1 = batt / 2 = charging
	for (i=1; i < CHARGE_STATUS_AGESIZE; i++) {
		if ( charge_status_age[i] < 1 )
			charge_status_age[i] = charge_status_age[0];
		if ( charge_status_age[0] != charge_status_age[i] ) {
			if ( debug_mask&DEBUG_LOG )
				BATT("Charger status changed: Charge_New=%d; Charge_Old[%d/%d]=%d\n",
				charge_status_age[0], (i + 1), (CHARGE_STATUS_AGESIZE - 1), charge_status_age[i] );
			buffer->level = old_level;
			return;
		}
	}

	/* this should make sure that if we are discharging the current is negative */
	if ( on_battery )
	    buffer->batt_current= 0 - buffer->batt_current;

	/*  here comes the algo calc -> Look at htc_battery_smem.h */
	if ( !on_battery  && ( buffer->batt_current > 0 ) ) {
		// Computes the battery charged capacity if AC or USB connected
		if ( ( buffer->batt_vol >= batt_param->full_volt_threshold ) && ( buffer->batt_current <= batt_param->termination_current ) ) {
			temp_correct_volt = -100;
			calc_algo = 16;
		} else if ( ( buffer->batt_vol >= batt_param->full_volt_threshold ) && ( buffer->batt_current <= 220 ) ) {
			temp_correct_volt = ( buffer->batt_current - batt_param->termination_current - 40 );
			calc_algo = 32;
		} else if ( ( buffer->batt_vol ) >= batt_param->full_volt_threshold ) {
			temp_correct_volt = 115;
			calc_algo = 64;
		} else if ( ( buffer->batt_vol ) >= batt_param->max_volt_threshold ) {
			temp_correct_volt = 135;
			calc_algo = 128;
		} else if ( ( buffer->batt_vol ) >= batt_param->med_volt_threshold ) {
			temp_correct_volt = 155;
			calc_algo = 256;
		} else if ( ( buffer->batt_vol ) >= batt_param->mid_volt_threshold ) {
			temp_correct_volt = 145;
			calc_algo = 512;
		} else if ( ( buffer->batt_vol ) >= batt_param->min_volt_threshold ) {
			temp_correct_volt = 135;
			calc_algo = 1024;
		} else if ( ( buffer->batt_vol ) >= batt_param->low_volt_threshold ) {
			temp_correct_volt = 120;
			calc_algo = 2048;
		} else if ( ( buffer->batt_vol ) >= batt_param->cri_volt_threshold ) {
			temp_correct_volt = 110;
			calc_algo = 4096;
		} else {
			temp_correct_volt = 100;
			calc_algo = 8192;
		}
	}

	/* Computes the battery charged capacity if AC or USB connected ...*/
	/* Retrieves the battery voltage level - if not charging */
	/* Battery level and battery struct are in htc_battery_smem.h */
	/* if we are charging we don't have to calculate the voltage over the discharge current resistor */
	if ( ccurrent > 0 ) {
		volt_discharge_resistor = 0;
	} else {
		volt_discharge_resistor = ( abs( ccurrent ) * batt_param->volt_discharge_res_coeff ) / 100;
		/* wierd code I know... :S something related to overflow shit :S */
		if ( volt_discharge_resistor < 0 )	//volt_discharge_resistor += volt_discharge_resistor >> 31;
			volt_discharge_resistor = volt_discharge_resistor + 1;
	}
	/* if battery is too low */
	if ( temp > 250 )
		temp_correct_volt = temp_correct_volt;
	else
		temp_correct_volt = ( temp_correct_volt + ( ( batt_param->temp_correction_const * ( ( 250 - temp ) * abs( ccurrent ) ) ) / 10000 ) );
	corrected_volt = volt + volt_discharge_resistor;

	/* sometimes 1 single extreme value comes from device ...if the value is extrem to old value then ignore it ....*/
	/* if the difference is more as 1,25% - then ignore this value */
	if ( corrected_volt_old == 0 ) {
		corrected_volt_old = corrected_volt;
	} else if ( 125 > abs( (corrected_volt*100) / corrected_volt_old ) * 100 ) {  
		corrected_volt_old = 0;
		return;
	} else {
		corrected_volt_old = corrected_volt;
	}	

	if( !machine_is_htctopaz() ) {
		/* calculate the voltage offset because of the temperature and current use */
		corr_return = batt_current_temp_correction( temp, ccurrent, batt_param->temp_correction );
	}
	
	if ( (corrected_volt - temp_correct_volt ) >= batt_param->full_volt_threshold ) {
		result = 100;
		calc_algo = calc_algo + 0;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->max_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->max_volt_threshold ) * 10 ) / batt_param->max_volt_dynslope ) + ( batt_param->max_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 1;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->med_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->med_volt_threshold ) * 10 ) / batt_param->med_volt_dynslope ) + ( batt_param->med_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 2;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->mid_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->mid_volt_threshold ) * 10 ) / batt_param->mid_volt_dynslope ) + ( batt_param->mid_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 3;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->min_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->min_volt_threshold ) * 10 ) / batt_param->min_volt_dynslope ) + ( batt_param->min_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 4;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->low_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->low_volt_threshold ) * 10 ) / batt_param->low_volt_dynslope ) + ( batt_param->low_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 5;
	} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->cri_volt_threshold ) {
		result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->cri_volt_threshold ) * 10 ) / batt_param->cri_volt_dynslope ) + ( batt_param->cri_volt_perc_start / 10 ) );
		calc_algo = calc_algo + 6;
	} else {
		result = 0;
		calc_algo = calc_algo + 7;
	}

	/* would be stupid if we report more than 100% */
	/* Cleanup */
	if (result > 99) {
		buffer->level = 100;
	} else if (result < vol_min) {
		buffer->level = vol_min;
	} else if (result > vol_max) {
		buffer->level = vol_max;
	/* check against extrem values */
	} else if ( ( result > (old_level + 2) ) && (result < 98) ) {
		buffer->level = old_level + 2;
	} else if ( result < (old_level - 2) ) {
		buffer->level = old_level - 2;
	/* do normal calc value */
	} else {
		buffer->level = result;
	}
                
#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATTLOG("STAT; level=;%d; level_old=;%d; level-calc=;%d; volt=;%d; temp=;%d; current=;%d; Charge=;%d; calc_ago=;%d; corr_volt=;%d; corr_temp_volt=;%d; corr_return=;%d; discharge_volt_resist=;%d;\n", \
		buffer->level, old_level, result, buffer->batt_vol, buffer->batt_temp, buffer->batt_current, htc_batt_info.rep.charging_source, calc_algo, corrected_volt, temp_correct_volt, corr_return, volt_discharge_resistor );
#endif

	old_level = buffer->level;


}

static void fix_batt_values(struct battery_info_reply *buffer) {                   

	if (debug_mask & DEBUG_LOG)
		BATT("RAW-battcorr:fix_values:OLD vol=;%d;current=;%d;temp=;%d;tempRAW=;%d;htc_adc_range=;%d;on_battery=;%d\n",
	buffer->batt_vol, buffer->batt_current, buffer->batt_temp, buffer->batt_tempRAW, htc_adc_range, on_battery);
	
	/*if there are wrong values */                                              
	if ( buffer->batt_vol > 4250 )
		buffer->batt_vol = 4250;                                            
	if ( buffer->batt_vol < 2600 )
		buffer->batt_vol = 2600;
	if ( buffer->batt_current > 500 )
		buffer->batt_current = 500;                                        
	if ( buffer->batt_current < 0 )
		buffer->batt_current = 0;                                           
	if ( buffer->batt_tempRAW > 500 )
		buffer->batt_temp = 500;                                        
	else if ( buffer->batt_tempRAW < 0 )
		buffer->batt_temp = 0;
	else
		buffer->batt_temp = buffer->batt_tempRAW;

	if (debug_mask & DEBUG_LOG)
		BATT("RAW-battcorr:fix_values:NEW vol=;%d;current=;%d;temp=;%d;tempRAW=;%d;htc_adc_range=;%d;on_battery=;%d\n",
	buffer->batt_vol, buffer->batt_current, buffer->batt_temp, buffer->batt_tempRAW, htc_adc_range, on_battery);

}  

/* Topaz battery data corrections */
static int htc_topaz_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
	/* TODO : Test the htc_adc_range computed value is valid (extracted from battdrvr.dll ver.2008.6.22.0 of HTC topaz) */
	/* TODO : test the htc_pcb_id stuff */

#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:topaz vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif
	/* From battdrvr.dll inspection, htc_diamond and raphael are mostly identical, except for batteries capacities */
	buffer->batt_vol = ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) ;  // apply a linear correction.	    

	if ( !on_battery ) {
		if ( buffer->charging_source == CHARGER_AC ) {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 33);
			if ( buffer->batt_current > 240 )
				buffer->batt_current += 250;
		} else { // USB Charger
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 70);
		}
	} else {
		buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 106) / 10);
	}

	//temp algo is not ok..TODO someone must look into the driver 
	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 2600 ) / htc_adc_range;   // apply a linear correction.
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

/* battery data device correction
 * apply the variouse corrections over the raw battery data
 */
static int htc_diamond_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
	/* initial corrected voltage reporting
	 * These value's are read with a 12 bit adc.
	 * The best thing todo is to make sure that the work area in the measurement
	 * are evenly divided over the adc. This means that after we have read the value
	 * we need todo some corrections to the measured value before we get usuable
	 * readings.
	 * note: this is WIP so don't trust it.
	 * note: the linear correction value can be different on every model.
	 * todo: add ADC_REF and ADC_REF 1/2 corrections.
	 */
	 /* diamond100-900mAh: init_battery_settings: VREF=1233; 0.5-VREF=611; PCB_ID=0x80; HWBOARD_ID=0x00000001; ADC_A=1004; ADC_B=12068; htc_adc_range=4040; batt_id=461; batt_vendor=2; full_bat=900000 */

#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:raph vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif

	/* From battdrvr.dll inspection, htc_diamond and raphael are mostly identical, except for batteries capacities */
	buffer->batt_vol = ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) ;  // apply a linear correction.	    

	/* Convert readed value to mA */
	if ( !on_battery ) {
		if ( buffer->charging_source == CHARGER_AC ) {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
			if ( buffer->batt_current > 240 )
				buffer->batt_current += 250;
		} else { // USB Charger
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
		}
	} else {
		buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 181) / 10);
	}
	
	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 2600 ) / htc_adc_range;  // apply a linear correction.
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

/* battery data device correction
 * apply the variouse corrections over the raw battery data
 */
static int htc_raph_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
	/* initial corrected voltage reporting
	 * These value's are read with a 12 bit adc.
	 * The best thing todo is to make sure that the work area in the measurement
	 * are evenly divided over the adc. This means that after we have read the value
	 * we need todo some corrections to the measured value before we get usuable
	 * readings.
	 * note: this is WIP so don't trust it.
	 * note: the linear correction value can be different on every model.
	 * todo: add ADC_REF and ADC_REF 1/2 corrections.
	 */


// raph100-1340:      init_battery_settings: VREF=1254; 0.5-VREF=632; PCB_ID=0x80; HWBOARD_ID=0x00000001; ADC_A=1004; ADC_B=-9016; htc_adc_range=4109; batt_id=327; batt_vendor=2; full_bat=1340000
//              USB charging
//              RAW-battcorr:raph vol=;2992;current=;2356;temp=;3240;htc_adc_range=;4109 
//              AC
//              RAW-battcorr:raph vol=;3305;current=;1299;temp=;3218;htc_adc_range=;4109
//		BATTERY
// 		RAW-battcorr:raph vol=;3206;current=;31;temp=;3362;htc_adc_range=;4109
//DONE


// diamond100-900mAh: init_battery_settings: VREF=1233; 0.5-VREF=611; PCB_ID=0x80; HWBOARD_ID=0x00000001; ADC_A=1004; ADC_B=12068; htc_adc_range=4040; batt_id=461; batt_vendor=2; full_bat=900000

#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:raph vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif

	/* From battdrvr.dll inspection, htc_diamond and raphael are mostly identical, except for batteries capacities */
	buffer->batt_vol = ( ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) + 15 );  // apply a linear correction.	    

	/* Convert readed value to mA */
	if ( htc_hwboard_id == 0 ) {
		if ( !on_battery ) {
			if ( buffer->charging_source == CHARGER_AC ) {
				buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 30);
				if ( buffer->batt_current > 240 )
					buffer->batt_current += 250;
			} else { // CHARGER USB
				buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 30);
			}
		} else {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 181) / 10);
		}
	} else {

		if ( !on_battery ) {
			if ( buffer->charging_source == CHARGER_AC ) {
				buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
				if ( buffer->batt_current > 240 )
					buffer->batt_current += 250;
			} else { // USB Charger
				buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
			}
		} else {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 181) / 10);
		}
	}
	
	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 2600 ) / htc_adc_range;  // apply a linear correction.
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

/* battery data device correction
 * apply the variouse corrections over the raw battery data
 */
static int htc_blackstone_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
	/* initial corrected voltage reporting
	 * These value's are read with a 12 bit adc.
	 * The best thing todo is to make sure that the work area in the measurement
	 * are evenly divided over the adc. This means that after we have read the value
	 * we need todo some corrections to the measured value before we get usuable
	 * readings.
	 * note: this is WIP so don't trust it.
	 * note: the linear correction value can be different on every model.
	 * todo: add ADC_REF and ADC_REF 1/2 corrections.
	 */

#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:blackstone vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif

	if ( (htc_hwboard_id == 0) || (htc_hwboard_id == 0xFF) ) {
		buffer->batt_vol = ( ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) + 30 );  // apply a linear correction.     
	} else if ( htc_hwboard_id < 2 ) {
		buffer->batt_vol = ( ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) + 30 );  // apply a linear correction.     
	} else { /*htc_hwboard_id >= 2 */
		buffer->batt_vol = ( ( ( buffer->batt_vol * 5200 ) / htc_adc_range ) + 25 );  // apply a linear correction.     
	}

	if ( !on_battery ) {
		if ( buffer->charging_source == CHARGER_AC ) {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
			if ( buffer->batt_current > 240 )
				buffer->batt_current += 250;
		} else { // USB Charger
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 60);
		}
	} else {
		buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 151) / 10);
	}

	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 2600 ) / htc_adc_range;  // apply a linear correction.
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

/* todo modify for rhodium */
static int htc_rhod_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
	
#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:rhod vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif

	buffer->batt_vol = ( buffer->batt_vol * 0x11BC ) / 0x1000;
//	buffer->batt_vol = ( ( buffer->batt_vol * 4500 ) / htc_adc_range ) + 35 ;
//	buffer->batt_current = ( buffer->batt_current * 2600 ) / 0x1000;
	if ( !on_battery ) {
		if ( buffer->charging_source == CHARGER_AC ) {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 90);
			if ( buffer->batt_current > 240 )
				buffer->batt_current += 250;
		} else { // USB Charger
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 12) / 10);
		}
	} else {
		buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 400) / 10);
	}
	
        /* TODO: temp algo is not ok..if someone have the timte -> must look into the driver */
	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 5200 ) / htc_adc_range;
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );
	if ( buffer->batt_tempRAW < 250 ) /* fixed til so long we can get constant values - otherwise the algo wouln'T work good */
    		buffer->batt_tempRAW = 250;

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

/* todo adjustment for kovsky */
static int htc_kovsky_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;
/* todo : someone should be RE the battery driver from kovsky */	
#if HTC_BATTERY_BATTLOGGER
	if(debug_mask&DEBUG_LOG)
		BATT("RAW-battcorr:kovsky vol=;%d;current=;%d;temp=;%d;htc_adc_range=;%d\n",
		buffer->batt_vol, buffer->batt_current, buffer->batt_tempRAW, htc_adc_range);
#endif

	buffer->batt_vol = ( ( ( buffer->batt_vol * 7600 ) / htc_adc_range ) - 1230 );  // apply a linear correction.
	
	/* Convert readed value to mA */
	if ( !on_battery ) {
		if ( buffer->charging_source == CHARGER_AC ) {
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 31);
			if ( buffer->batt_current > 240 )
				buffer->batt_current += 250;
		} else { // USB Charger
			buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 31);
		}
	} else {
		//buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 151) / 10);
		buffer->batt_current = ( ( ( ( buffer->batt_current * 2600 ) / htc_adc_range ) * 10) / 35);
	}

        /* TODO: temp algo is not ok..if someone have the timte -> must look into the driver */
	buffer->batt_tempRAW = ( buffer->batt_tempRAW * 7600 ) / htc_adc_range;  // apply a linear correction.
	av_index = ( buffer->batt_tempRAW * 18 ) / ( 2600 - buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );
        if ( buffer->batt_tempRAW < 250 ) /* fixed til so long we can get constant values - otherwise the algo wouln'T work good */
		buffer->batt_tempRAW = 250;

	fix_batt_values(buffer);

	htc_battery_level_compute( buffer );

	return 0;
}

static int htc_get_batt_smem_info(struct battery_info_reply *buffer)
{
	volatile struct htc_batt_info_u32 *batt_32 = NULL;
	volatile struct htc_batt_info_u16 *batt_16 = NULL;
	struct msm_dex_command dex;

	dex.cmd = PCOM_GET_BATTERY_DATA;
	msm_proc_comm_wince(&dex, 0);

	if (htc_batt_info.resources->smem_field_size == 4) {
		batt_32 = (void *)(MSM_SHARED_RAM_BASE + htc_batt_info.resources->smem_offset);

		buffer->batt_vol = batt_32->batt_vol;
		buffer->batt_current = batt_32->batt_charge;
		buffer->batt_tempRAW = batt_32->batt_temp;
		buffer->batt_id = batt_32->batt_id;

	} else if (htc_batt_info.resources->smem_field_size == 2) {
		batt_16 = (void *)(MSM_SHARED_RAM_BASE + htc_batt_info.resources->smem_offset);

		buffer->batt_vol = batt_16->batt_vol;
		buffer->batt_current = batt_16->batt_charge;
		buffer->batt_tempRAW = batt_16->batt_temp;
		buffer->batt_id = batt_16->batt_id;
	} else {
		BATT_ERR("[BATT]: unsupported smem_field_size\n");
		return -ENOTSUPP;
	}

	return 0;
}

static int htc_get_batt_info(struct battery_info_reply *buffer)
{
	int i;

	int volt_lowest_val = 0xFFFF, volt_highest_val = 0, current_lowest_val = 0xFFFF, current_highest_val = 0;
	int volt_sum = 0, current_sum = 0;
	int last_source, new_source;

	if ( buffer == NULL )
		return -EINVAL;

	if ( !htc_batt_info.resources || !htc_batt_info.resources->smem_offset ) {
		BATT_ERR("smem_offset not set\n" );
		return -EINVAL;
	}

	last_source = htc_batt_info.rep.charging_source;

	/* Don't know why, diamond battdrvr.dll reads 5 times ADC level,
	 * removes the highest and lowest value and use the average value
	 */
	if ( machine_is_htcdiamond() || machine_is_htcblackstone() ) {
		i = 0;
		do
		{
			if(htc_get_batt_smem_info(buffer) == -ENOTSUPP)
						return -ENOTSUPP;

			/* Saves the lowest and highest value of batt_vol and current */
			if(buffer->batt_vol < volt_lowest_val) {
				volt_lowest_val = buffer->batt_vol;
			}

			if(buffer->batt_vol > volt_highest_val) {
				volt_highest_val = buffer->batt_vol;
			}

			volt_sum += buffer->batt_vol;

			if(buffer->batt_current < current_lowest_val) {
				current_lowest_val = buffer->batt_current;
			}

			if(buffer->batt_current > current_highest_val) {
				current_highest_val = buffer->batt_current;
			}

			current_sum += buffer->batt_current;

			i++;

			mdelay(2);
		}
		while(i < 5);

		mutex_lock(&htc_batt_info.lock);

		/* Remove the highest and lowest value */
		volt_sum = volt_sum - volt_lowest_val - volt_highest_val;
		current_sum = current_sum - current_lowest_val - current_highest_val;

		buffer->batt_vol = volt_sum / 3;
		buffer->batt_current = current_sum / 3;
	} else {
		if(htc_get_batt_smem_info(buffer) == -ENOTSUPP)
			return -ENOTSUPP;
	}

	// add the corrected value to the maf filter
	maf_add_value( buffer->batt_vol );

	// calculate the avarage
	buffer->batt_vol = maf_get_avarage();

	if (gpio_get_value(htc_batt_info.resources->gpio_charger_enable) == 0) {
		buffer->charging_enabled = 1;
		if(htc_batt_info.resources->gpio_ac_detect) {
			if (gpio_get_value(htc_batt_info.resources->gpio_ac_detect)) {
				buffer->charging_source = CHARGER_AC;	// 900mA
			} else {
				buffer->charging_source = CHARGER_USB;	// 500mA
			}
		} else {
			buffer->charging_source = CHARGER_AC;	// 900mA
		}
	} else {
		buffer->charging_enabled = 0;
		buffer->charging_source = CHARGER_BATTERY;
	}
	if(debug_mask&DEBUG_LOG)
		BATT("CHARGER_enabled=%d; CHARGER_source=%d\n", buffer->charging_enabled, buffer->charging_source);

//	if(!machine_is_htckovsky() && htc_batt_info.resources->smem_field_size==2) {
//		// these were taken out in a kovsky commit, so assume it doesn't need them.
//		//buffer->charging_enabled = ( buffer->batt_current > 0x700 );
//		buffer->charging_enabled = ( buffer->batt_current >= 0x200 );
//		buffer->charging_source =  ( buffer->batt_current < 0x200 ) ? CHARGER_BATTERY : CHARGER_USB;
//	}
                                   
	/* should it be done before correction */
	new_source = buffer->charging_source;
	buffer->charging_source = last_source;
	htc_cable_status_update(new_source);

	/* platform spec battery correction */
	if ( machine_is_htcraphael() || machine_is_htcraphael_cdma() || machine_is_htcraphael_cdma500() ||
	     machine_is_htcdiamond_cdma() ) {
		htc_raph_batt_corr( buffer );
	} else if ( machine_is_htcdiamond() ) {
		htc_diamond_batt_corr( buffer );
	} else if ( machine_is_htcblackstone() ) {
		htc_blackstone_batt_corr( buffer );
	} else if ( machine_is_htctopaz() ) {
		htc_topaz_batt_corr( buffer );
	} else if ( machine_is_htcrhodium() ) {
		htc_rhod_batt_corr( buffer );
	} else if ( machine_is_htckovsky() ) {
		htc_kovsky_batt_corr( buffer );
	} else {
		/* fallback for not supported devices */
		htc_raph_batt_corr( buffer );
	}

	mutex_unlock(&htc_batt_info.lock);

	return 0;
}

/* -------------------------------------------------------------------------- */
static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;

	charger = htc_batt_info.rep.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int htc_battery_get_charging_status(void)
{
	u32 level;
	charger_type_t charger;
	int ret;

	charger = htc_batt_info.rep.charging_source;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		level = htc_batt_info.rep.level;
		if (level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = htc_batt_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = htc_batt_info.rep.level;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

void htc_battery_external_power_changed(struct power_supply *psy) {
	if (debug_mask & DEBUG_CABLE)
		BATT("external power changed\n");
	maf_clear();
	schedule_work(&bat_work);
	return;
}

static irqreturn_t htc_bat_gpio_isr(int irq, void *data) {
	if (debug_mask & DEBUG_CABLE)
		BATT("IRQ %d for GPIO \n", irq);
	schedule_work(&bat_work);
	return IRQ_HANDLED;
}

#define HTC_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
	.show = htc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
};

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
};

static int htc_battery_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	goto succeed;

htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
	return rc;
}

static ssize_t htc_battery_show_property(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	/* check cache time to decide if we need to update */
	if (htc_batt_info.update_time &&
            time_before(jiffies, htc_batt_info.update_time +
                                msecs_to_jiffies(cache_time)))
                goto dont_need_update;

	if (htc_get_batt_info(&htc_batt_info.rep) < 0) {
		BATT_ERR("%s: get_batt_info failed!!!\n", __FUNCTION__);
	} else {
		htc_batt_info.update_time = jiffies;
	}
dont_need_update:
	mutex_lock(&htc_batt_info.lock);
	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_enabled);
		break;
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.full_bat);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&htc_batt_info.lock);

	return i;
}

static int htc_battery_thread(void *data)
{
	struct battery_info_reply buffer;
	daemonize("battery");
	allow_signal(SIGKILL);

	while (!signal_pending((struct task_struct *)current)) {
		msleep(10000);
		if (!bat_suspended && !htc_get_batt_info(&buffer)) {
			htc_batt_info.update_time = jiffies;
			htc_battery_status_update(buffer.level);
		}
	}
	return 0;
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc;

	INIT_WORK(&bat_work, htc_bat_work);
	htc_batt_info.resources = (smem_batt_t *)pdev->dev.platform_data;

	if (!htc_batt_info.resources) {
		BATT_ERR("%s: no pdata resources!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* init battery gpio */
	if ((rc = init_batt_gpio()) < 0) {
		BATT_ERR("%s: init battery gpio failed!\n", __FUNCTION__);
		return rc;
	}

	/* init structure data member */
	htc_batt_info.update_time 	= jiffies;
	htc_batt_info.present 		= gpio_get_value(htc_batt_info.resources->gpio_battery_detect);

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("Failed to register power supply (%d)\n", rc);
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	/* init static battery settings */
	if ( init_battery_settings( &htc_batt_info.rep ) < 0)
		BATT_ERR("%s: init battery settings failed\n", __FUNCTION__);

	htc_battery_initial = 1;

	htc_batt_info.update_time = jiffies;
	kernel_thread(htc_battery_thread, NULL, CLONE_KERNEL);
	if ( machine_is_htckovsky() || machine_is_htctopaz() ) {
		rc = request_irq(gpio_to_irq(htc_batt_info.resources->gpio_ac_detect),
				htc_bat_gpio_isr,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"htc_ac_detect", &htc_batt_info);
        if (rc)
	    BATT_ERR("IRQ-request rc=%d\n", rc);
    }

	not_yet_started = false;
	return 0;
}

#if CONFIG_PM
static int htc_battery_suspend(struct platform_device* device, pm_message_t mesg)
{
	bat_suspended = 1;
	return 0;
}

static int htc_battery_resume(struct platform_device* device)
{
	bat_suspended = 0;
	return 0;
}
#else
 #define htc_battery_suspend NULL
 #define htc_battery_resume NULL
#endif

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = htc_battery_suspend,
	.resume = htc_battery_resume,
};

static int __init htc_battery_init(void)
{
	// this used to be WAKE_LOCK_SUSPEND, but make it an idle lock in order to
	// prevent msm_sleep() try to collapse arm11 (using idle_sleep mode) several
	// times a second which sooner or later get's the device to freeze when usb
	// is connected
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_IDLE, "vbus_present");
	mutex_init(&htc_batt_info.lock);
	platform_driver_register(&htc_battery_driver);
	BATT("HTC Battery Driver initialized\n");

	return 0;
}

late_initcall(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
