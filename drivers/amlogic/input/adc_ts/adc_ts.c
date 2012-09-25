/*
 * drivers/input/touchscreen/adc_ts.c
 *
 * Using code from:
 *  - tsc2007.c
 *	Copyright (c) 2008 MtekVision Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/saradc.h>
#include <linux/adc_ts.h>

#define TS_POLL_DELAY       1 /* ms delay between samples */
#define TS_POLL_PERIOD      5 /* ms delay between samples */

#define MAX_10BIT           ((1 << 10) - 1)


#define DELTA_Y 75
#define READ_X  1
#define READ_Y  2
#define READ_Z1 3
#define READ_Z2 4

#ifdef CONFIG_AML_TCON_K8_800x600
#define LCD_RES_X			800
#define LCD_RES_Y			600
#endif

#ifdef CONFIG_AML_TCON_K8_1024x768
#define LCD_RES_X			1024
#define LCD_RES_Y			768
#define MAX_X   950		//860 		/* the max X value get from the raw adc data */
#define MAX_Y   950		//920 		/* the max Y value get from the raw adc data*/
#define MIN_X   60		//120 		/* the min X value get from the raw adc data */
#define MIN_Y   80		//100 		/* the min Y value get from the raw adc data */

#endif

#ifdef CONFIG_AML_TCON_K8_1024x600
#define LCD_RES_X			1024
#define LCD_RES_Y			600
#define MAX_X   960		//860 		/* the max X value get from the raw adc data */
#define MAX_Y   935		//920 		/* the max Y value get from the raw adc data*/
#define MIN_X   48		//120 		/* the min X value get from the raw adc data */
#define MIN_Y   110		//100 		/* the min Y value get from the raw adc data */

#endif


struct ts_event {
	u16	x;
	u16	y;
	u16	z1;
	u16	z2;
};

struct adcts {
	struct input_dev *input;
	char phys[32];
	struct delayed_work work;
	struct ts_event event;
	bool pendown;
	int seq;
	
	u16 x_plate_ohms;
	int irq;
	int (*service)(int cmd);
};

static u32 adcts_calculate_pressure(struct adcts *ts, struct ts_event *tc)
{
	u32 rt = 0;

    /* range filtering */
    if (tc->x == MAX_10BIT)
        tc->x = 0;

	if (likely(tc->x && tc->z1)) {
		/* compute touch pressure resistance using equation #1 */
		rt = tc->z2 - tc->z1;
		rt *= tc->x;
		rt *= ts->x_plate_ohms;
		rt /= tc->z1;
		rt = (rt + 2047) >> 12;
	}

	return rt;
}

static void adcts_send_up_event(struct adcts *ts)
{
	struct input_dev *input = ts->input;

	dev_dbg(&ts->input->dev, "UP\n");

	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);
}


static void adcts_work(struct work_struct *work)
{
    struct adcts *ts =
        container_of(to_delayed_work(work), struct adcts, work);
    struct input_dev *input = ts->input;
    u32 rt;
    int up_down_flag = 0;
    static int ts_x_cache = 0;
    static int ts_y_cache = 0;

	/*
	 * NOTE: We can't rely on the pressure to determine the pen down
	 * state, even though this controller has a pressure sensor.
	 * The pressure value can fluctuate for quite a while after
	 * lifting the pen and in some cases may not even settle at the
	 * expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * work function by reading the pen signal state (it's a GPIO
	 * and IRQ). Unfortunately such callback is not always available,
	 * in that case we have rely on the pressure anyway.
	 */

    
    if (ts->seq == 0) { 
        if (ts->service(CMD_GET_PENDOWN)) {
            if (!ts->pendown) {
                ts->pendown = 1;
                up_down_flag = 1;
                input_report_key(input, BTN_TOUCH, 1);
            }
            ts->seq ++;
            ts->service(CMD_CLEAR_PENIRQ);
        }
        else  if (ts->pendown) {
            ts->pendown = 0;
            adcts_send_up_event(ts);
            ts_x_cache = 0;
            ts_y_cache = 0;
        }
    }
    else if (ts->seq == 1) { 
        ts->event.x = ts->service(CMD_GET_X);
        ts->seq ++;
    }
    else if (ts->seq == 2) { 
        ts->event.y = ts->service(CMD_GET_Y);
        if((ts_x_cache != 0)&&(ts_y_cache != 0)){
			
			int x = (ts_x_cache-MIN_X)*LCD_RES_X/(MAX_X-MIN_X);
			int y = (ts_y_cache-MIN_Y)*LCD_RES_Y/(MAX_Y-MIN_Y);
	
#ifdef CONFIG_AML_TCON_K8_1024x600
			x = LCD_RES_X - x;							// snowwan
#endif
			input_report_abs(input, ABS_X, x);
			input_report_abs(input, ABS_Y, y);
			rt = 500;   //debug
			input_report_abs(input, ABS_PRESSURE, rt);
            input_sync(input);
			//printk(KERN_INFO "cachex=%d, cachey=%d", ts_x_cache, ts_y_cache);
			//printk(KERN_INFO "x=%d, y=%d\n", x, y);
            up_down_flag = 1;
        }
        ts_x_cache = ts->event.x;
        ts_y_cache = ts->event.y;
        ts->seq = 0;
        ts->service(CMD_SET_PENIRQ);
    }

	if (ts->pendown || (ts->irq < 0)) {
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	}
	else {
		ts->service(CMD_SET_PENIRQ);
		enable_irq(ts->irq);
		printk(KERN_INFO "exit adc irq\n");
	}
}


static irqreturn_t adcts_irq(int irq, void *handle)
{
	struct adcts *ts = handle;
	printk(KERN_INFO "enter adc irq\n");

	if (ts->service(CMD_GET_PENDOWN)) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}
	
	return IRQ_HANDLED;
}

static void adcts_free_irq(struct adcts *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit adcts_probe(struct platform_device *pdev)
{
	struct adcts *ts;
	struct adc_ts_platform_data *pdata = pdev->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&pdev->dev, "platform data is required!\n");
		return -EINVAL;
	}

	ts = kzalloc(sizeof(struct adcts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts->input = input_dev;

	platform_set_drvdata(pdev, ts);
	INIT_DELAYED_WORK(&ts->work, adcts_work);

	ts->x_plate_ohms = pdata->x_plate_ohms;
	ts->service = saradc_ts_service;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "adcts Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    input_set_abs_params(input_dev, ABS_X, 0, LCD_RES_X, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, LCD_RES_Y, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_10BIT, 0, 0);

	ts->seq = 0;
	ts->pendown = 0;
	ts->irq = pdata->irq;
	ts->service(CMD_INIT_PENIRQ);
	if (ts->irq < 0) {
		schedule_delayed_work(&ts->work,
			      msecs_to_jiffies(TS_POLL_DELAY));
	}
	else {
		err = request_irq(ts->irq, adcts_irq, 0, pdev->dev.driver->name, ts);
		if (err < 0) {
			dev_err(&pdev->dev, "irq %d busy?\n", ts->irq);
			goto err_free_mem;
		}
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	return 0;

 err_free_irq:
	adcts_free_irq(ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit adcts_remove(struct platform_device *pdev)
{
	struct adcts *ts = platform_get_drvdata(pdev);
	adcts_free_irq(ts);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}


static struct platform_driver adcts_driver = {
	.probe      = adcts_probe,
	.remove     = adcts_remove,
	.suspend    = NULL,
	.resume     = NULL,
	.driver     = {
		.name   = "adc_ts",
	},
};

static int __devinit adcts_init(void)
{
	printk(KERN_INFO "ADC Touchscreen Driver init.\n");
	return platform_driver_register(&adcts_driver);
}

static void __exit adcts_exit(void)
{
	printk(KERN_INFO "ADC Touchscreen  Driver exit.\n");
	platform_driver_unregister(&adcts_driver);
}

module_init(adcts_init);
module_exit(adcts_exit);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("ADC TouchScreen Driver");
MODULE_LICENSE("GPL");



