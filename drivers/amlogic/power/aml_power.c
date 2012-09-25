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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/aml_power.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/usb/otg.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


//#define AML_POWER_DBG

int BAT_DEBUG_ON =
#if defined(AML_POWER_DBG)
    1;
#else
    0;
#endif


#define BATTERY_VALID_NUM_SHIFT 2
#define BATTERY_ARROW_NUM  ((1<<BATTERY_VALID_NUM_SHIFT)+2)  //add min,max

static inline unsigned int get_irq_flags(struct resource *res)
{
	unsigned int flags = IRQF_SAMPLE_RANDOM | IRQF_SHARED;

	flags |= res->flags & IRQF_TRIGGER_MASK;

	return flags;
}


#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)    

struct adc_watch{
    int time_start;
    int adc;
    int count;
    int steady;
    int suspend;
};

#endif

struct aml_power_data {
    struct device *dev;
	const struct aml_power_pdata *pdata;
    struct power_supply psy;
    struct power_supply psy_ac;
    struct power_supply psy_usb;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif

    struct workqueue_struct *queue;
    struct delayed_work work;

    int adc_cache[BATTERY_ARROW_NUM];
    int adc_cache_ns;
    int adc_cache_nr;

    int ac;
    int usb;
    int charging_full;
    int voltage;
    int capacity;
    
    int suspend;

    int capacity15_lock;
    int capacity0_lock;

#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
    struct adc_watch adc_stagnate_watch;
#endif
};

static struct aml_power_data *aml_pm;

static enum power_supply_property aml_battery_props[] = {
    /*POWER_SUPPLY_PROP_ONLINE,*/
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property aml_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property aml_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};



struct voltage_table{
    unsigned int  voltage;
    unsigned int  adc;
};

struct energy_table{
    unsigned int  voltage;
    unsigned int  percent;
};

#if defined(CONFIG_BATT_3V7_110_110)

#   define BATT_FULL_ADC_VALUE  661 //640
#   define BATT_EMPTY_ADC_VALUE 520 //508

#   define BATT_EMPTY_VOL_VALUE 3300 
#   define BATT_FULL_VOL_VALUE  4250

#   define BATT_ADC_STEP 15  //adc value about 0.1mV

const struct voltage_table g_voltage_table[]={
    {BATT_EMPTY_VOL_VALUE,BATT_EMPTY_ADC_VALUE},
    {3400,528},
    {3500,542},
    {3600,559},
    {3700,574},
    {3750,583},
    {3800,590},
    {3850,598},
    {3900,606},
    {4000,622},
    {4100,637},
    {4200,653},
    {BATT_FULL_VOL_VALUE,BATT_FULL_ADC_VALUE},

    {(unsigned int)-1,(unsigned int)-1},  //table end tag: adc is MAX
};

const struct energy_table g_engery_table[]={
    {BATT_EMPTY_VOL_VALUE,0},
    {3350,1},
    {3400,2},
    {3500,7},
    {3600,20},
    {3700,35},
    {3800,18},
    {3900,12},
    {4000,5},
    {4100,0},
    {BATT_FULL_VOL_VALUE,0},

    {(unsigned int)-1,0},  //table end tag: voltage is MAX
};

#elif defined(CONFIG_BATT_3V7_110_110_V2)

#   define BATT_FULL_ADC_VALUE  666
#   define BATT_EMPTY_ADC_VALUE 527

#   define BATT_EMPTY_VOL_VALUE 3300 
#   define BATT_FULL_VOL_VALUE  4250

#   define BATT_ADC_STEP 15  //adc value about 0.1mV

const struct voltage_table g_voltage_table[]={
    {BATT_EMPTY_VOL_VALUE,BATT_EMPTY_ADC_VALUE},
    {3400,533},
    {3500,548},
    {3600,565},
    {3700,580},
    {3750,587},
    {3800,595},
    {3850,603},
    {3900,612},
    {4000,626},
    {4100,642},
    {4200,659},
    {BATT_FULL_VOL_VALUE,BATT_FULL_ADC_VALUE},

    {(unsigned int)-1,(unsigned int)-1},  //table end tag: adc is MAX
};

const struct energy_table g_engery_table[]={
    {BATT_EMPTY_VOL_VALUE,0},
    {3400,1},
    {3500,5},
    {3600,20},
    {3700,35},
    {3800,20},
    {3900,12},
    {4000,5},
    {4100,2},
    {BATT_FULL_VOL_VALUE,0},

    {(unsigned int)-1,0},  //table end tag: voltage is MAX
};


#elif defined(CONFIG_BATT_7V4_330_150)
#   define BATT_FULL_ADC_VALUE  816
#   define BATT_EMPTY_ADC_VALUE 574

#   define BATT_EMPTY_VOL_VALUE 3000 
#   define BATT_FULL_VOL_VALUE  4250

#   define BATT_ADC_STEP 19  //adc value about 0.1mV

const struct voltage_table g_voltage_table[]={
    {BATT_EMPTY_VOL_VALUE,BATT_EMPTY_ADC_VALUE},
    {3100,594},
    {3200,613},
    {3300,633},
    {3400,653},
    {3500,672},
    {3600,691},
    {3700,710},
    {3750,719},
    {3800,729},
    {3850,739},
    {3900,748},
    {4000,767},
    {4100,787},
    {4200,806},
    {BATT_FULL_VOL_VALUE,BATT_FULL_ADC_VALUE},

    {(unsigned int)-1,(unsigned int)-1},  //table end tag: adc is MAX
};

const struct energy_table g_engery_table[]={
    {BATT_EMPTY_VOL_VALUE,0},
    {3100,1},
    {3200,3},
    {3300,5},
    {3400,5},
    {3500,8},
    {3600,15},
    {3700,30},
    {3800,18},
    {3900,10},
    {4000,5},
    {4100,0},
    {BATT_FULL_VOL_VALUE,0},

    {(unsigned int)-1,0},  //table end tag: voltage is MAX
};

#elif defined(CONFIG_BATT_7V4_330_150_V2)
#   define BATT_FULL_ADC_VALUE  830
#   define BATT_EMPTY_ADC_VALUE 586

#   define BATT_EMPTY_VOL_VALUE 3000 
#   define BATT_FULL_VOL_VALUE  4250

#   define BATT_ADC_STEP 19  //adc value about 0.1mV

const struct voltage_table g_voltage_table[]={
    {BATT_EMPTY_VOL_VALUE,BATT_EMPTY_ADC_VALUE},
    {3100,604},
    {3200,626},
    {3300,646},
    {3400,664},
    {3500,684},
    {3600,703},
    {3700,722},
    {3750,731},
    {3800,741},
    {3850,751},
    {3900,762},
    {4000,781},
    {4100,800},
    {4200,820},
    {BATT_FULL_VOL_VALUE,BATT_FULL_ADC_VALUE},

    {(unsigned int)-1,(unsigned int)-1},  //table end tag: adc is MAX
};

const struct energy_table g_engery_table[]={
    {BATT_EMPTY_VOL_VALUE,0},
    {3100,1},
    {3200,3},
    {3300,5},
    {3400,5},
    {3500,8},
    {3600,15},
    {3700,30},
    {3800,18},
    {3900,10},
    {4000,5},
    {4100,0},
    {BATT_FULL_VOL_VALUE,0},

    {(unsigned int)-1,0},  //table end tag: voltage is MAX
};


#else
# error not battery select
#endif


static int aml_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
    struct aml_power_data *pmdata;

    if(psy->type == POWER_SUPPLY_TYPE_BATTERY)
        pmdata = container_of(psy, struct aml_power_data, psy);
    else if(psy->type == POWER_SUPPLY_TYPE_MAINS)
        pmdata = container_of(psy, struct aml_power_data, psy_ac);
    else if(psy->type == POWER_SUPPLY_TYPE_USB)
        pmdata = container_of(psy, struct aml_power_data, psy_usb);
    else{
        printk("unsupport power supply type %d\n",psy->type);
        return 0;
    }
    
    //int capacty;
	switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:	
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = pmdata->ac;
		else
			val->intval = pmdata->usb;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	    //uV
		val->intval = pmdata->voltage*1000;
#if defined(CONFIG_BATT_7V4_330_150)||defined(CONFIG_BATT_7V4_330_150_V2)
        val->intval <<= 1;
#endif
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if(pmdata->ac) {
            if(pmdata->charging_full)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:	
		val->intval = pmdata->capacity;
		break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;        
	default:
		return -EINVAL;
	}
	return 0;
}


static struct power_supply aml_psy_bat = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = aml_battery_props,
	.num_properties = ARRAY_SIZE(aml_battery_props),
	.get_property = aml_power_get_property,
};

static struct power_supply aml_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = aml_ac_props,
	.num_properties = ARRAY_SIZE(aml_ac_props),
	.get_property = aml_power_get_property,
};

static struct power_supply aml_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = aml_usb_props,
	.num_properties = ARRAY_SIZE(aml_usb_props),
	.get_property = aml_power_get_property,
};

#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
static void pending_adc_stagnate(struct aml_power_data *pmdata)
{
    if(pmdata->capacity <= pmdata->pdata->capacity_compensation_max)
        pmdata->adc_stagnate_watch.suspend = 1;

}
#endif

static int capacity_recalibrate(struct aml_power_data *pmdata,int capacity,int adc)
{
#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
    int capacity_compensation = pmdata->pdata->capacity_compensation_max;
    int stagnate_time;
    int adc_delta;

    if(!capacity){ // no power    
        adc_delta = abs(pmdata->adc_stagnate_watch.adc - adc);
        if(adc_delta <= pmdata->pdata->adc_stagnate_delta_low)
            pmdata->adc_stagnate_watch.count++;
        else{
            if(adc_delta >= pmdata->pdata->adc_stagnate_delta_high)
                pmdata->adc_stagnate_watch.steady = 0;
                
            if(!pmdata->adc_stagnate_watch.steady){  // renew stagnate record
                pmdata->adc_stagnate_watch.adc = adc;
                pmdata->adc_stagnate_watch.count = 0;
                pmdata->adc_stagnate_watch.time_start = jiffies;
            }
        }

        if(pmdata->adc_stagnate_watch.count){
            stagnate_time = jiffies - pmdata->adc_stagnate_watch.time_start;
            
            if(pmdata->adc_stagnate_watch.steady){ // current steady, prepare shutdown
                if(stagnate_time > pmdata->pdata->adc_stagnate_second_steady){
                    printk("battery stagnate accur\n");
                    capacity_compensation = 0;
                }else{
                    capacity_compensation = (pmdata->pdata->adc_stagnate_second_steady - stagnate_time) * pmdata->pdata->capacity_compensation_max / pmdata->pdata->adc_stagnate_second_steady;
                }
            }else{  //set steady flag
                if(stagnate_time > pmdata->pdata->adc_stagnate_second){
                    printk("battery stagnate steady\n");
                    pmdata->adc_stagnate_watch.time_start = jiffies;
                    pmdata->adc_stagnate_watch.steady = 1;
                }
            }
        }
        
        if(BAT_DEBUG_ON){
            printk("adc %d(watch %d delta %d) capacity %d(comp %d) count %d steady %d stagnate_time %d(%d - %d)\n",
                adc,pmdata->adc_stagnate_watch.adc,adc_delta,
                capacity,capacity_compensation,
                pmdata->adc_stagnate_watch.count,
                pmdata->adc_stagnate_watch.steady,
                stagnate_time,(int)jiffies,pmdata->adc_stagnate_watch.time_start);
        }        
    }else{
        // clean all stagnate record
        memset(&pmdata->adc_stagnate_watch.adc,0,sizeof(pmdata->adc_stagnate_watch));
    }
    
    if(!pmdata->adc_stagnate_watch.suspend && capacity_compensation){
        capacity = (capacity * (100 - pmdata->pdata->capacity_compensation_max)/100) + capacity_compensation;
    }
#endif

    if(capacity && capacity <= 15)
        pmdata->capacity15_lock++;

    if(pmdata->capacity15_lock && capacity > 15)
        capacity = 15;

    if(capacity == 0)
        pmdata->capacity0_lock++;
    
    if(pmdata->capacity0_lock && capacity > 0)
        capacity = 0;

    return capacity;
}

static void capacity_lock_reset(struct aml_power_data *pmdata)
{
    pmdata->capacity0_lock =
        pmdata->capacity15_lock = 0;
}

static int adc_average(struct aml_power_data *pmdata,int adc)
{
    int nr;
    int i;
    int avg,min,max;
    
    if(adc > 0){
        pmdata->adc_cache[pmdata->adc_cache_ns++] = adc;
        if(pmdata->adc_cache_ns > BATTERY_ARROW_NUM-1)
            pmdata->adc_cache_ns = 0;
        if(pmdata->adc_cache_nr < BATTERY_ARROW_NUM)
            pmdata->adc_cache_nr ++;
    }

    nr = pmdata->adc_cache_nr;

    if(!nr){
        return 0;
    }
    
    avg = min = max = pmdata->adc_cache[0];
    for (i = 1; i < nr; i++) {
        avg += pmdata->adc_cache[i];
        if(max < pmdata->adc_cache[i])  max = pmdata->adc_cache[i];
        if(min > pmdata->adc_cache[i])  min = pmdata->adc_cache[i];
    }

    if(nr > 2){  //more than 2, delete max and min
        avg =avg - min -max;
        nr -= 2;
    }

    if( nr > 4 || nr == 3){
        avg /= nr;       // use divide
    }else if( nr ==4 ){  // equal 4, use shift 2
        avg >>= 2;
    }else if( nr ==2 ){  // equal 2, use shift 1
        avg >>= 1;
    }

    return avg;
}

void adc_cache_reset(struct aml_power_data *pmdata)
{
    pmdata->adc_cache_ns = 0;
    pmdata->adc_cache_nr = 0;

    memset(pmdata->adc_cache,0,sizeof(pmdata->adc_cache));
}

static unsigned int adc_to_voltage_simple(unsigned int adc)
{
    unsigned int voltage,volhigh,vollow;
    unsigned int adchigh,adclow;

    volhigh=BATT_FULL_VOL_VALUE;
    vollow=BATT_EMPTY_VOL_VALUE;

    adchigh = BATT_FULL_ADC_VALUE;
    adclow = BATT_EMPTY_ADC_VALUE;

    if(adc<=adclow){
        voltage=vollow;
    }else if(adc>=adchigh){
        voltage=volhigh;
    }else{
        voltage=(unsigned int)(adc-adclow)*(volhigh-vollow)/(adchigh-adclow)+vollow;
    }
    return voltage;
}

static unsigned int adc_to_voltage(unsigned int adc)
{
    const struct voltage_table *vt;
    unsigned int voltage;
    unsigned int step;

    unsigned int dw;
    
    vt=g_voltage_table;

    voltage=0;
    dw=0;
    while(adc>vt[dw].adc){
        dw++;
    }

    if(dw>0){        
        step=(((adc-vt[dw-1].adc)*
                    (vt[dw].voltage-vt[dw-1].voltage))<<8)/
                    (vt[dw].adc-vt[dw-1].adc);
        step>>=8;            
        voltage = step+vt[dw-1].voltage;
    }else{
        voltage = vt[0].voltage;
    }

    return (int)voltage;

}


static int calculate_capacity(unsigned int vol)
{
    const struct energy_table *et;
    unsigned int percent;
    unsigned int step;

    unsigned int dw;
    
    et=g_engery_table;

    percent=0;
    dw=0;
    while(vol>et[dw].voltage){
        percent+=et[dw].percent;
        dw++;
    }

    if(dw>0){        
        step=((((vol-et[dw-1].voltage)*
                    et[dw-1].percent))<<8)/
                    (et[dw].voltage-et[dw-1].voltage);
        step>>=8;            
        percent-=et[dw-1].percent;
        percent+=step;
    }

    return (int)percent;
}

static int aml_power_poscheck(struct aml_power_data *pmdata) {

    const struct aml_power_pdata *pdata =
                    pmdata->pdata;
    int polling_interval;
    int adc,adc_avg,adc_delta;
    int vol,capacity;
    
    int ac = 0,usb = 0,charging_full = 0;
    int status_change = 0;

    if(pdata->is_ac_online)
        ac = (*pdata->is_ac_online)();

    if(pdata->is_usb_online)
        usb = (*pdata->is_usb_online)();

    if(pdata->get_charge_status)
        charging_full = ( ac /*|| usb*/ )?(*pdata->get_charge_status)():0;

    if(ac != pmdata->ac)
        adc_cache_reset(pmdata);

    adc = pmdata->pdata->get_bat_vol();
    adc_avg = adc_average(pmdata,adc);
    adc_delta = abs(adc - adc_avg);
    
    vol = (int)adc_to_voltage(adc_avg);
    capacity = calculate_capacity(vol);

    //percent 0 lock
    if(!ac){
        capacity = capacity_recalibrate(pmdata,capacity,adc_avg);
    }else{
        capacity_lock_reset(pmdata);
        if(!charging_full){
            if(capacity)
                capacity--;
            capacity = capacity*capacity/100;
        }
        if(capacity <= 15)
            capacity = 16;
    }

    if(BAT_DEBUG_ON){
        printk("adc %d %d vol %d capacity %d (ac %d usb %d full %d)\n",
            adc,adc_avg,vol,capacity,ac,usb,charging_full);
        //capacity = 100;
    }

    //notify change
    if(ac != pmdata->ac){
        pmdata->ac = ac;
        status_change = 1;
    }

    if(usb != pmdata->usb){
        pmdata->usb = usb;
        status_change = 1;
    }

    if(charging_full != pmdata->charging_full){
        pmdata->charging_full = charging_full;
        status_change = 1;
    }
    
    if(capacity != pmdata->capacity || pmdata->voltage != vol){
        pmdata->capacity = capacity;
        pmdata->voltage = vol;
        status_change = 1;
    }
    
    if(status_change)
        power_supply_changed(&pmdata->psy);

    if(adc_delta > BATT_ADC_STEP)
        polling_interval = (pdata->critical_polling_interval);
    if(adc_delta > (BATT_ADC_STEP>>1))
        polling_interval = (pdata->fast_polling_interval);
    else{
        if(status_change)
            polling_interval = pdata->fast_polling_interval;
        else if(!ac && capacity < 5)
            polling_interval = pdata->fast_polling_interval;
        else
            polling_interval = pdata->polling_interval;
    }
    if(BAT_DEBUG_ON){
        if(polling_interval > pdata->critical_polling_interval)
            polling_interval = pdata->critical_polling_interval;
    }

    return polling_interval;
}
static void aml_power_work(struct work_struct *work)
{
	struct aml_power_data *pmdata = container_of(work,struct aml_power_data,work.work);
    int polling_interval;

	do {
		polling_interval = aml_power_poscheck(pmdata);

        if(!pmdata->suspend)
            msleep((unsigned int)polling_interval);
            
	} while (!pmdata->suspend);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void aml_power_early_suspend(struct early_suspend *h)
{
	struct aml_power_data *pmdata =  container_of(h, struct aml_power_data, early_suspend);

    printk("aml_power_early_suspend\n");
    pmdata->suspend = 1;
}

static void aml_power_late_resume(struct early_suspend *h)
{   
	struct aml_power_data *pmdata =  container_of(h, struct aml_power_data, early_suspend);

	printk("aml_power_late_resume\n");
	adc_cache_reset(pmdata);

#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
    pending_adc_stagnate(pmdata);
#endif

	pmdata->suspend = 0;
	queue_delayed_work(pmdata->queue, &pmdata->work, HZ/8);
}
#endif

static ssize_t store_powerhold(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    struct aml_power_data *pmdata = aml_pm;

	if(buf[0] == 'y'){
        printk("system off\n");    

        if(pmdata->pdata->set_bat_off)
            pmdata->pdata->set_bat_off();
    }

	return count;
}

static ssize_t store_debug(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
	if(buf[0] == '1'){
	   BAT_DEBUG_ON = 1; 
    }
    else{
	   BAT_DEBUG_ON = 0;         
    }        
	return count;
}

static ssize_t show_debug(struct class *class, 
			struct class_attribute *attr,	char *buf)
{
	return sprintf(buf, "bat-debug value is %d\n", BAT_DEBUG_ON);
}

static struct class_attribute powerhold_class_attrs[] = {
    __ATTR(bat-off,  S_IRUGO | S_IWUSR, NULL,    store_powerhold),
    __ATTR(bat-debug,  S_IRUGO | S_IWUSR, show_debug,    store_debug),    
    __ATTR_NULL
};

static struct class powerhold_class = {
    .name = "powerhold",
    .class_attrs = powerhold_class_attrs,
};

static int aml_power_probe(struct platform_device *pdev)
{
    struct aml_power_data *pmdata;
    const struct aml_power_pdata *pdata =
                    pdev->dev.platform_data;
    int error;

    printk("aml_power_probe\n");
    
    if (!pdata) {
        dev_err(&pdev->dev, "platform data not defined\n");
        return -EINVAL;
    }

    pmdata = kzalloc(sizeof(*pmdata), GFP_KERNEL);
    if (!pmdata) {
        dev_err(&pdev->dev, "Failed to allocate driver data!\n");
        error = -ENOMEM;
        dev_set_drvdata(&pdev->dev, NULL);
        return error;
    }

    dev_set_drvdata(&pdev->dev, pmdata);

	pmdata->dev= &pdev->dev;
	pmdata->pdata= pdata;
    memcpy(&pmdata->psy,&aml_psy_bat,sizeof(pmdata->psy));
    memcpy(&pmdata->psy_ac,&aml_psy_ac,sizeof(pmdata->psy_ac));
    memcpy(&pmdata->psy_usb,&aml_psy_usb,sizeof(pmdata->psy_usb));

   	error = class_register(&powerhold_class);
	if(error){
		printk(" class register powerhold_class fail!\n");
		goto out;
	}

	if (pdata->init) {
		pdata->init(pmdata->dev);
	}
	if (pdata->set_charge) {
		pdata->set_charge(0);
	}

	pmdata->queue = create_singlethread_workqueue("bat polling");
	INIT_DELAYED_WORK(&pmdata->work, aml_power_work);
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    pmdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    pmdata->early_suspend.suspend = aml_power_early_suspend;
    pmdata->early_suspend.resume = aml_power_late_resume;
    pmdata->early_suspend.param = pdev;
	register_early_suspend(&pmdata->early_suspend);
#endif

    aml_pm = pmdata;

    //power supply bat
	error = power_supply_register(pmdata->dev, &pmdata->psy);
	if (error) {
		dev_err(pmdata->dev, "failed to register psy battery\n");
		goto out;
	}
    //power supply ac
	error = power_supply_register(pmdata->dev, &pmdata->psy_ac);
	if (error) {
		dev_err(pmdata->dev, "failed to register psy ac\n");
		goto out;
	}
    //power supply usb
	error = power_supply_register(pmdata->dev, &pmdata->psy_usb);
	if (error) {
		dev_err(pmdata->dev, "failed to register psy usb\n");
		goto out;
	}

    queue_delayed_work(pmdata->queue, &pmdata->work, HZ * 5);

    printk("aml_power_probe success\n");
    
out:
	return error;
}

static int aml_power_remove(struct platform_device *pdev)
{
    const struct aml_power_pdata *pdata =
                    pdev->dev.platform_data;
    struct aml_power_data *pmdata;

    pmdata = (struct aml_power_data *)dev_get_drvdata(&pdev->dev);
    if(!pmdata)
        return 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&pmdata->early_suspend);
#endif

	power_supply_unregister(&pmdata->psy);
	power_supply_unregister(&pmdata->psy_ac);
	power_supply_unregister(&pmdata->psy_usb);

    if (pdata->exit)
        pdata->exit(&pdev->dev);

    kfree(pmdata);
    dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int aml_power_suspend(struct platform_device *pdev, pm_message_t state)
{
    printk("aml_power_suspend\n");
	return 0;
}

static int aml_power_resume(struct platform_device *pdev)
{
    printk("aml_power_resume\n");

    return 0;
}
#else
#define aml_power_suspend NULL
#define aml_power_resume NULL
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:aml-power");

static struct platform_driver aml_power_pdrv = {
	.driver = {
		.name = "aml-power",
	},
	.probe = aml_power_probe,
	.remove = aml_power_remove,
	.suspend = aml_power_suspend,
	.resume = aml_power_resume,
};

static int __init aml_power_init(void)
{
	printk("amlogic power supply init\n");
	return platform_driver_register(&aml_power_pdrv);
}

static void __exit aml_power_exit(void)
{
	platform_driver_unregister(&aml_power_pdrv);
}

module_init(aml_power_init);
module_exit(aml_power_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Larson Jiang");
