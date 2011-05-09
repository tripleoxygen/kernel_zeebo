struct smem_battery_resources {
	unsigned short gpio_battery_detect;
	unsigned short gpio_charger_enable;
	unsigned short gpio_charger_current_select;
	unsigned short gpio_ac_detect;
	unsigned smem_offset;
	unsigned short smem_field_size;
};

typedef struct smem_battery_resources smem_batt_t;

/* This can get called by the proc_comm interrupt routine */
extern int htc_cable_status_update(int status);
