/*
 *  linux/drivers/input/touchscreen/nas_touch.c
 *
 *  touch screen driver for nas_touch capacitive touch controller
 *
 *  Copyright (C) 2010, Marvell Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
//#include <mach/touchscreen.h>

#include <linux/sysctl.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#include <linux/i2c/touch_common.h>

#ifdef CONFIG_ZTKEY
// FB_OSD1_ZTKEY_LEN
#define ZTKEY_TOUCH  ( CONFIG_FB_OSD2_DEFAULT_WIDTH - CONFIG_FB_OSD1_ZTKEY_LEN) //3750
extern int m_ztRotate;
extern int start_ztkey;

#define POWER_CODE  116
#define HOME_CODE   102
#define MENU_CODE   125
#define BACK_CODE   15
#define VOL0_CODE   104
#define VOL1_CODE   109

struct zt_ket_str {	int x;	int y;	int pressure;	int state;	};

struct zt_ket_str zt_key_input;
extern struct completion zt_key_comp;
extern void add_zt_key(int x, int y, int p, int state);
extern void zt_key_android(int code);
#endif

/* Register Map for nas touchscreen */
#define NAS_FINGER_REG	0x00
#define NAS_OLDFINGER_REG 0x01

#define NAS_X0_LSB_REG	0x02
#define NAS_X0_MSB_REG	0x03
#define NAS_Y0_LSB_REG	0x04
#define NAS_Y0_MSB_REG	0x05

#define NAS_X1_LSB_REG	0x06
#define NAS_X1_MSB_REG	0x07
#define NAS_Y1_LSB_REG	0x08
#define NAS_Y1_MSB_REG	0x09

#define NAS_X0_W_REG 0x0a
#define NAS_Y0_W_REG 0x0b
#define NAS_X1_W_REG 0x0c
#define NAS_Y1_W_REG 0x0d

#define NAS_X0_Z_REG 0xe
#define NAS_Y0_Z_REG 0xf
#define NAS_X1_Z_REG 0x10
#define NAS_Y1_Z_REG 0x11

#define NAS_POWER_MODE 0x14
#define NAS_INT_MODE 0x15
#define NAS_INT_WIDTH 0x16

#define NAS_VERSION_START 0x30
#define NAS_VERSION_END   0x34

#define NAS_SPECOP 0x37

#define SHOW_VERSION
//#define DEBUG_PRINTK 
#define NAS_START_REG NAS_FINGER_REG
//#define DEBUG_REG 

#define ACTIVE_SLEEP_SWITCH 0xa4
#define DEEP_SLEEP 0x02
#define FREEZE_SLEEP 0x03

#define INT_MODE_FMOV 0x0d

static int x_min = 0;
static int y_min = 0;
static int x_max = 1024 - CONFIG_FB_OSD1_ZTKEY_LEN;
static int y_max = 600;
static atomic_t nastech_status;
static atomic_t nastech_calibration;

struct nas_ts_priv {
	struct input_dev *dev;
	struct work_struct work;
	struct i2c_client *client;
	struct early_suspend early_suspend;
    struct touch_platform_data *pdata;
	
	int reported_finger_count;
};


static int nastech_set_int_mode(struct i2c_client *client, int data)
{
	unsigned char datareg;
	int ret = 0;
	datareg = data;
	ret = i2c_smbus_write_i2c_block_data(client, NAS_INT_MODE, 1, &datareg);
	if (ret < 0) {
		printk(KERN_ERR "%s: Failed to set INT MODE\n", __func__);
		return ret;
	} else {
		ret = i2c_smbus_read_i2c_block_data(client, NAS_INT_MODE, 1,
						  &datareg);
		printk(KERN_DEBUG
		       "%s: Success to set INT MODE, INT MODE =0x%x\n",__func__, datareg);
	}
	return ret;
}


static int nastech_set_power_mode(struct i2c_client *client, int on)
{
	int ret = 0;
	unsigned char datareg[2];

	if (!on) {
		datareg[0] = FREEZE_SLEEP;
		ret = i2c_smbus_write_i2c_block_data(client, NAS_POWER_MODE, 1,
						   &datareg[0]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed to enter Deep Sleep Mode\n",__func__);
			return ret;
		} else {
			ret = i2c_smbus_read_i2c_block_data(client,
							  NAS_POWER_MODE, 1,
							  &datareg[1]);
			printk(KERN_DEBUG
			       "%s: Success to enter Deep Slepp Mode,POWER MODE = 0x%x\n",
			       __func__, datareg[1]);
		}
	} else {
		datareg[1] = ACTIVE_SLEEP_SWITCH;
		ret = i2c_smbus_write_i2c_block_data(client, NAS_POWER_MODE, 1,
						   &datareg[1]);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s: Failed to enter Active <-> Sleep Mode\n",
			       __func__);
			return ret;
		} else {
			ret = i2c_smbus_read_i2c_block_data(client,
							  NAS_POWER_MODE, 1,
							  &datareg[0]);
			printk(KERN_DEBUG
			       "%s: Success to active<->sleep mode(10s),POWER MODE =0x%x\n",
			       __func__, datareg[0]);
		}
	}
	return ret;
}

static int nastech_set_power_mode_old(struct i2c_client *client, int on)
{
	int ret = 0;
	unsigned char datareg[2];

	if (!on) {
		datareg[0] = DEEP_SLEEP;
		ret = i2c_smbus_write_i2c_block_data(client, NAS_POWER_MODE, 1,
						   &datareg[0]);
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed to enter Deep Sleep Mode\n",__func__);
			return ret;
		} else {
			ret = i2c_smbus_read_i2c_block_data(client,
							  NAS_POWER_MODE, 1,
							  &datareg[1]);
			printk(KERN_DEBUG
			       "%s: Success to enter Deep Slepp Mode,POWER MODE = 0x%x\n",
			       __func__, datareg[1]);
		}
	} else {
		datareg[1] = ACTIVE_SLEEP_SWITCH;
		ret = i2c_smbus_write_i2c_block_data(client, NAS_POWER_MODE, 1,
						   &datareg[1]);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s: Failed to enter Active <-> Sleep Mode\n",
			       __func__);
			return ret;
		} else {
			ret = i2c_smbus_read_i2c_block_data(client,
							  NAS_POWER_MODE, 1,
							  &datareg[0]);
			printk(KERN_DEBUG
			       "%s: Success to active<->sleep mode(10s),POWER MODE =0x%x\n",
			       __func__, datareg[0]);
		}
	}
	return ret;
}

static int nastech_write_proc(struct file *file,
			      const char __user * buffer, unsigned long count,
			      void *data)
{
	int index;
	char kbuf[2];
	struct nas_ts_priv *ts = data;
	unsigned char datareg;
	int ret = 0;
	struct i2c_client *client = ts->client;

	if (!ts)
		return -EFAULT;
	if (count > 2)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	if (index == 0) {
		if (atomic_cmpxchg(&nastech_status, 1, 0) == 1) {
			disable_irq(ts->client->irq);
			flush_work(&ts->work);
			nastech_set_power_mode(client, 0);
		}
	} else if (index == 1) {
		if (atomic_cmpxchg(&nastech_status, 0, 1) == 0) {
			nastech_set_power_mode(client, 1);
			enable_irq(ts->client->irq);
		}
	} else if (index == 2) {
		datareg = 0x3;
		ret = i2c_smbus_write_i2c_block_data(client, NAS_SPECOP,
						   1, &datareg);
		if (ret < 0) {
			printk(KERN_ERR
			       "%s: Calibration : error = 0x%x\n", __func__, ret);
			return -EFAULT;
		} else
			printk(KERN_ERR
			"%s: Calibration OK and 0x37=0x%x\n", __func__, datareg);
		atomic_set(&nastech_calibration, 2);
	}
	return count;
}

static int nastech_read_proc(char *buffer, char **start, off_t offset,
			     int count, int *peof, void *data)
{
	return sprintf(buffer, "enable touch:%d\ncalibration:%d\n",
		       atomic_read(&nastech_status),
		       atomic_read(&nastech_calibration));
}

static void nastech_create_proc_file(void *data)
{
	struct proc_dir_entry *proc_file =
	    create_proc_entry("driver/nastech", 0644, NULL);

	if (proc_file) {
		proc_file->write_proc = nastech_write_proc;
		proc_file->read_proc = nastech_read_proc;
		proc_file->data = data;
	} else {
		printk(KERN_INFO "proc file create failed!\n");
	}
}

extern struct proc_dir_entry proc_root;
static void nastech_remove_proc_file(void)
{
	remove_proc_entry("driver/nastech", &proc_root);
}

static void nastech_ts_work(struct work_struct *work)
{
	unsigned short xpos[2] = { 0 };
	unsigned short ypos[2] = { 0 };
	unsigned char finger;	/* number of fingers touching */
	int ret;
	int len = 0;
	unsigned char data[8];
	int z, w;
	int ky = 0;
	struct nas_ts_priv *nas_priv =
	    container_of(work, struct nas_ts_priv, work);
	struct i2c_client *client = nas_priv->client;
	struct input_dev *input_dev = nas_priv->dev;

#ifdef DEBUG_PRINTK
	printk("+-----------------------------------------+\n");
	printk("|	nastech_ts_work!                      |\n");
	printk("+-----------------------------------------+\n");
#endif
	/* Read the finger value */
	ret = i2c_smbus_read_i2c_block_data(client, NAS_FINGER_REG, 1, data);
	if (ret < 0) {
		printk(KERN_ERR "%s: touch read FINGER failed:finger=0x%x\n",
		       __func__, ret);
		goto err;
	}
	finger = data[0];
	if (finger > 0) {
		if (finger == 1)
			len = 4;
		else if (finger == 2)
			len = 8;
		ret = i2c_smbus_read_i2c_block_data(client, NAS_X0_LSB_REG, len,
						  data);
		if (ret < 0) {
			printk(KERN_ERR "%s: touch read data failed\n",
			       __func__);
			goto err;
		}
		/* Read the first point */
		if (finger > 0) {
			xpos[0] = (unsigned short)(data[1] << 8 | data[0]);
			ypos[0] = (unsigned short)(data[3] << 8 | data[2]);
			ypos[0] = y_max - ypos[0];
		}
		/* Read the second point */
		if (finger > 1) {
			xpos[1] = (unsigned short)(data[5] << 8 | data[4]);
			ypos[1] = (unsigned short)(data[7] << 8 | data[6]);
			ypos[1] = y_max - ypos[1];
		}
	}
	//printk("nastech_ts_work: X0 = %d , Y0 = %d, finger = %d\n", xpos[0], ypos[0], finger);
#ifdef DEBUG_PRINTK
	/* Debug info */
	printk("nastech_ts_work: finger : %d\n", finger);
/*	printk("nastech_ts_work: X0 = 0x%x , Y0 = 0x%x\n", xpos[0], ypos[0]); */
	printk("nastech_ts_work: X0 = %d , Y0 = %d\n", xpos[0], ypos[0]);
/*	printk("nastech_ts_work: X1 = 0x%x , Y1 = 0x%x\n", xpos[1], ypos[1]); */
	printk("nastech_ts_work: X1 = %d , Y1 = %d\n", xpos[1], ypos[1]);
#endif
#ifdef DEBUG_REG
	ret = i2c_smbus_read_i2c_block_data(client, NAS_SPECOP, 1, &data[0]);
	ret = i2c_smbus_read_i2c_block_data(client, NAS_POWER_MODE, 2, &data[1]);
	printk(KERN_ERR
	       "NAS_SPECOP = 0x%x, NAS_POWER_MODE = 0x%x, NAS_INT_MODE = 0x%x\n",
	       data[0], data[1], data[2]);
#endif
	if (!finger) {
		z = 0;
		w = 0;
	} else {
		z = 255;
		w = 15;
	}
	
	if(xpos[0] > ZTKEY_TOUCH){
		printk("down outx=%d\n",xpos[0]);
		zt_key_input.x = xpos[0];
		zt_key_input.y = ypos[0];
		add_zt_key(xpos[0],ypos[0],0,m_ztRotate);
		complete(&zt_key_comp);		
	} else {	
		if( xpos[0] || ypos[0] ) {
			zt_key_input.x = 0;
		}
	
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(input_dev, ABS_MT_POSITION_X, xpos[0]);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, ypos[0]);
		input_mt_sync(input_dev);
		
		/*
		input_report_abs(input_dev, ABS_PRESSURE, z);
		input_report_abs(input_dev, ABS_TOOL_WIDTH, w);
		input_report_abs(input_dev, ABS_X, xpos[0]);
		input_report_abs(input_dev, ABS_Y, ypos[0]);
		*/
	}

	if (!finger){
		
		//printk("up zt_key_input.x=%d\n",zt_key_input.x);
		if(zt_key_input.x > ZTKEY_TOUCH) {
			
			if(start_ztkey) {
				add_zt_key(0,0,0,m_ztRotate);
				complete(&zt_key_comp);
			}
			//ky = zt_key_input.y * 600/4096;
			ky = zt_key_input.y;
		
			printk("up : s=0 ,x=%d, y=%d\n",zt_key_input.x,ky);
			if(ky > 0 && ky < 120) {
				zt_key_android(VOL0_CODE);
			}
			if(ky > 120 && ky < 240) {
				zt_key_android(VOL1_CODE);
			}							
			if(ky > 240 && ky < 360) {
				zt_key_android(MENU_CODE);
			}
			if(ky > 360 && ky < 480) {
				zt_key_android(BACK_CODE);
			}
			if(ky > 480 && ky < 600) {
				zt_key_android(HOME_CODE);
			}
			nas_priv->reported_finger_count = finger;
			return;							
		} else {
		    input_report_key(input_dev, BTN_TOUCH, 0);
	    }
	} else {
		input_report_key(input_dev, BTN_TOUCH, 1);
	}
	
	if (finger > 1) {
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, z);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, w);
		input_report_abs(input_dev, ABS_MT_POSITION_X, xpos[1]);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, ypos[1]);
		input_mt_sync(input_dev);
	}

	else if (nas_priv->reported_finger_count > 1) {
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X, 0);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, 0);
		input_mt_sync(input_dev);

	}
	nas_priv->reported_finger_count = finger;
	input_sync(input_dev);
	return;
err:
	return;
}

static irqreturn_t nastech_ts_isr(int irq, void *dev_id)
{
	struct nas_ts_priv *ts = (struct nas_ts_priv *)dev_id;

	schedule_work(&ts->work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void nastech_ts_early_suspend(struct early_suspend *h)
{
	struct nas_ts_priv *ts =
	    container_of(h, struct nas_ts_priv, early_suspend);
	
	printk("nastech_ts_early_suspend\r\n");
	disable_irq(ts->client->irq);
	flush_work(&ts->work);
	//nastech_set_power_mode(ts->client, 0);
    if(ts ->pdata->enable)
        ts ->pdata->enable(0);
	return;
}

static void nastech_ts_late_resume(struct early_suspend *h)
{
	struct nas_ts_priv *ts =
	    container_of(h, struct nas_ts_priv, early_suspend);

	printk("nastech_ts_late_resume\r\n");
    msleep(100);
    if(ts ->pdata->enable)
        ts ->pdata->enable(1);
	msleep(100);
	//nastech_set_power_mode(ts->client, 1);
	enable_irq(ts->client->irq);
	return;
}
#endif

static int __devinit nastech_ts_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct nas_ts_priv *ts;
	struct input_dev *input_dev;
	int ret = 0;
	unsigned char datareg[2];
#ifdef  SHOW_VERSION
	unsigned char version[10];
#endif

	struct touch_platform_data *pdata = pdata = client->dev.platform_data;

	printk("nastech_ts_probe\r\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(struct nas_ts_priv), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!ts || !input_dev) {
		ret = -ENOMEM;
		goto alloc_fail;
	}
	i2c_set_clientdata(client, ts);

	ts->dev = input_dev;

	input_dev->name = "nastech-ts";

	input_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0Y, y_min, y_max, 0, 0);
	/*Android MT */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	ret = input_register_device(ts->dev);
	if (ret) {
		printk(KERN_ERR "%s: Failed to allocate input device\n",
		       __func__);
		goto input_dev_fail;
	}

	if (pdata->init_irq)		
		pdata->init_irq();

	if (pdata->enable)		
		pdata->enable(1);
	ts->client = client;
	ts->pdata = pdata;
	INIT_WORK(&ts->work, nastech_ts_work);

	ret = request_irq(client->irq, nastech_ts_isr,
			  IRQF_DISABLED | IRQF_TRIGGER_RISING,
			  "nastech-ts irq", ts);
	if (ret) {
		printk(KERN_ERR "%s: request irq failed\n", __func__);
		goto irq_fail;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	ts->early_suspend.suspend = nastech_ts_early_suspend;
	ts->early_suspend.resume = nastech_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	dev_set_drvdata(&input_dev->dev, ts);
	atomic_set(&nastech_status, 1);	/* on */
	ret = i2c_smbus_read_i2c_block_data(client, NAS_SPECOP, 1, &datareg[0]);
	atomic_set(&nastech_calibration, 0);
	nastech_create_proc_file(ts);
	nastech_set_power_mode(ts->client, 1);
	nastech_set_int_mode(ts->client, INT_MODE_FMOV);
#ifdef SHOW_VERSION
	ret = i2c_smbus_read_i2c_block_data(client, NAS_VERSION_START,
					  NAS_VERSION_END - NAS_VERSION_START +
					  1, version);
	if (ret < 0)
		printk(KERN_ERR "touch read version: error = 0x%x\n", ret);
	else
		printk(KERN_ERR
		       "touch read verion OK and 0x30=0x%x 0x31=0x%x 0x32=0x%x 0x33=0x%x 0x33=0x%x\n",
		       version[0], version[1], version[2], version[3],
		       version[4]);
#endif
	return 0;

irq_fail:
	free_irq(client->irq, client);

input_dev_fail:
	i2c_set_clientdata(client, NULL);
	input_free_device(input_dev);

alloc_fail:
	kfree(ts);
err_check_functionality_failed:
	return ret;
}

static int __devexit nastech_ts_remove(struct i2c_client *client)
{
	struct nas_ts_priv *ts = i2c_get_clientdata(client);

#ifdef DEBUG_PRINTK
	printk("+-----------------------------------------+\n");
	printk("|   nastech_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
#endif
	if (client->irq)
		free_irq(client->irq, client);
	nastech_remove_proc_file();
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->dev);
	kfree(ts);

	return 0;
}

static struct i2c_device_id nastech_ts_idtable[] = {
	{"nastech-ts", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, nastech_ts_idtable);

static struct i2c_driver nastech_ts_driver = {
	.driver = {
		   .name = "nastech-ts",
		   },
	.id_table = nastech_ts_idtable,
	.probe = nastech_ts_probe,
	.remove = __devexit_p(nastech_ts_remove),
};

static int __init nastech_ts_init(void)
{
	return i2c_add_driver(&nastech_ts_driver);
}

static void __exit nastech_ts_exit(void)
{
	i2c_del_driver(&nastech_ts_driver);
}

module_init(nastech_ts_init);
module_exit(nastech_ts_exit);

MODULE_AUTHOR("xiazh@marvell.com");
MODULE_DESCRIPTION("nastech touch screen driver");
MODULE_LICENSE("GPL");
