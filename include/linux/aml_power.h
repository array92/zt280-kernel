/*
 * Common power driver for Amlogic Devices with one or two external
 * power supplies (AC/USB) connected to main and backup batteries,
 * and optional builtin charger.
 *
 * Copyright © 2010 Larson Jiang <larson.jiang@amlogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AML_POWER_H__
#define __AML_POWER_H__

#define AML_POWER_CHARGE_AC  (1 << 0)
#define AML_POWER_CHARGE_USB (1 << 1)

struct device;

struct aml_power_pdata {
	int (*init)(struct device *dev);
	int (*is_ac_online)(void);
	int (*is_usb_online)(void);
	int (*get_bat_vol)(void);
	int (*get_charge_status)(void);
	void (*set_charge)(int flags);
	void (*exit)(struct device *dev);
	void (*set_bat_off)(void);
	void (*ic_control)(int flag);
	void (*powerkey_led_onoff)(int onoff);

	char **supplied_to;
	size_t num_supplicants;

#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
    int adc_stagnate_delta_low;
    int adc_stagnate_delta_high;
    int adc_stagnate_second;        //jiffies
    int adc_stagnate_second_steady;
    int capacity_compensation_max;  //percent(<=100)
#endif

	unsigned int polling_interval; /* msecs*/
	unsigned int fast_polling_interval;
	unsigned int critical_polling_interval;
};

#endif /* __AML_POWER_H__ */
