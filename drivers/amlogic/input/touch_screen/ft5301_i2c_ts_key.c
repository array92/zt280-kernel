/* drivers/input/touchscreen/ft5301_i2c_ts.c
 *
 * Copyright (C) 2010-2011 Ft5301, Inc.
 * V1.0
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "ft5301_i2c_ts.h"

#include <linux/adc_ts_key.h>
#include <linux/i2c/touch_common.h>


//#define DEBUG

struct ft5301_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	wait_queue_head_t wait;
	const struct touch_platform_data *chip;
	unsigned int attb_pin;
	int irq;

	int config_major;
	char config_name[20];
	struct class *config_class;
	struct device *config_dev;	
	int suspend;
    int key_status;  //max support 32 key
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5301_ts_early_suspend(struct early_suspend *h);
static void ft5301_ts_late_resume(struct early_suspend *h);
#endif

static struct i2c_client *ft5301_client;

static const struct file_operations ft5301_i2c_ts_fops;

static int register_touchpad_dev(struct ft5301_i2c_ts_data  *tsdata)
{
    int ret=0;
    strcpy(tsdata->config_name,"ft5301_i2c_ts");
    ret=register_chrdev(0, tsdata->config_name, &ft5301_i2c_ts_fops);
    if(ret<=0) {
        printk("ft5301 register char device error\r\n");
        return  ret ;
    }
    tsdata->config_major=ret;
    printk("adc touchpad major:%d\r\n",ret);
    tsdata->config_class=class_create(THIS_MODULE,tsdata->config_name);
    tsdata->config_dev=device_create(tsdata->config_class,	NULL,
    		MKDEV(tsdata->config_major,0),NULL,/*tsdata->config_name*/"ft5301_i2c_ts%d",0);
    				
    return ret;
}

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    

int ft5301_ts_key(struct ft5301_i2c_ts_data *data,int status)
{
    struct ft5301_i2c_ts_data *tsdata = data;

    const struct touch_platform_data *pdata = tsdata->chip;
    int i;

    if(status > (1<<(pdata->key_num + 1))){
        printk(KERN_INFO "jumper ft5301 key 0x%x\n",status);
        status = 0;
    }

    if(status != tsdata->key_status){
        int change = tsdata->key_status^status;

        for (i = 0; i < pdata->key_num; i++) {
            if((change >> i)&0x1){
#if defined(DEBUG)            
                printk(KERN_INFO "%s key(%d) press %d\n", pdata->key[i].name, pdata->key[i].code,(status >> i)&0x1);
#endif
                if(pdata->key[i].code >= 0)
                    input_event(tsdata->input, EV_KEY, pdata->key[i].code,(status >> i)&0x1);
            }
        }

        tsdata->key_status = status;
    }
    return 0;
}
#endif
static int ft5301_ts_poscheck(struct ft5301_i2c_ts_data *data)
{
	struct ft5301_i2c_ts_data *tsdata = data;
	const struct touch_platform_data *pdata = tsdata->chip;

	u8 touching;
	u16 offset,high_byte,low_byte;
	int posx,posy;
	u8 rdbuf[PROTOCOL_LEN];
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    
    int key_status = 0;
    int point_status[POINTS_MAX] = {0};
#endif

	int i;
	int ret;

	int z = 50;
	//int w = 15;

	memset(rdbuf, 0, sizeof(rdbuf));

    //touch event(get point 1 also)
	ret = i2c_smbus_read_i2c_block_data(tsdata->client, PROTOCOL_ADDR + I2C_STARTTCH_READ, PROTOCOL_HEAD_LEN + PROTOCOL_POINT_LEN, &rdbuf[I2C_STARTTCH_READ]);
    if(ret == (PROTOCOL_HEAD_LEN + PROTOCOL_POINT_LEN) &&
        rdbuf[0] == 0 &&
        rdbuf[2] <= POINTS_MAX){
    	touching = rdbuf[2];

#if defined(DEBUG)       
        printk("ft5301 %d level %d 0x%x,0x%x,0x%x 0x%x ",touching,tsdata->chip->get_irq_level(),rdbuf[0],rdbuf[1],rdbuf[2],rdbuf[3]);
#endif

    }else{
        dev_err(&tsdata->client->dev, "Ft5301 data error buf[0~3] = 0x%x,0x%x,0x%x ret %d\n",rdbuf[0],rdbuf[1],rdbuf[2],ret);
        memset(rdbuf,0,sizeof(rdbuf));
        touching = 0;
    }

    if(touching){
        if(touching > 1){
            //touch event(get point left)
            ret = i2c_smbus_read_i2c_block_data(tsdata->client, PROTOCOL_ADDR + I2C_STARTTCH_POINT1, (touching - 1) * PROTOCOL_POINT_LEN, &rdbuf[I2C_STARTTCH_POINT1]);
            if(ret == (touching - 1) * PROTOCOL_POINT_LEN){
            }else{
                dev_err(&tsdata->client->dev, "Ft5301 data point1(%d) error ret %d\n",touching,ret);
                //memset(rdbuf,0,sizeof(rdbuf));
                touching = 1;
            }
        }
    }
    
    if(touching){
        for(i = 0; i < touching; i++) {
            offset = I2C_STARTTCH_READ + PROTOCOL_HEAD_LEN + (i*PROTOCOL_POINT_LEN);
            high_byte = rdbuf[offset] & 0xf;
            high_byte <<= 8;
            low_byte = rdbuf[offset+1];
            posx = (high_byte |low_byte) & 0x0fff;
            
            high_byte = rdbuf[offset+2] & 0xf;
            high_byte <<= 8;
            low_byte = rdbuf[offset+3];
            posy = (high_byte |low_byte) & 0x0fff;

#if defined(DEBUG)
            //printk("[0x%x 0x%x 0x%x 0x%x] ",rdbuf[offset],rdbuf[offset+1],rdbuf[offset+2],rdbuf[offset+3]);
            printk("(%d,%d) ",posx,posy);
#endif
            if(pdata->convert2)
                pdata->convert2(&posx,&posy);

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)
            if(pdata->to_key)
                point_status[i] = pdata->to_key(pdata->key,pdata->key_num,posx,posy);
            key_status |= point_status[i];
            if(point_status[i])
                continue;
#endif
            input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
            //input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w); 
            input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx);
            input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy);
            input_mt_sync(tsdata->input);
        }
    }else{
        input_mt_sync(tsdata->input);
    }
    input_sync(tsdata->input);

#if defined(DEBUG)            
    printk("\n");
#endif

    //key event
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)
    if(rdbuf[1] <= 0xF){
        key_status |= rdbuf[1];
    }
    touching |= key_status?(1<<4):0;
    
    tsdata->key_status = 
        pdata->report_key(tsdata->input,pdata->key,pdata->key_num,tsdata->key_status,key_status);
#endif
	return touching;
}

static irqreturn_t ft5301_ts_isr(int irq, void *dev_id)
{
	struct ft5301_i2c_ts_data *tsdata = dev_id;
    int touching;

	do {
		touching = ft5301_ts_poscheck(tsdata);
		tsdata->attb_pin = tsdata->chip->get_irq_level();
		if (tsdata->attb_pin == ATTB_PIN_LOW || touching){
			/*wait_event_timeout(tsdata->wait, TRUE,
					msecs_to_jiffies(20));*/
            msleep(5);
	    }
	} while (!tsdata->suspend && (tsdata->attb_pin == ATTB_PIN_LOW /*|| touching*/));

	return IRQ_HANDLED;
}

static void ft5301_ts_close(struct input_dev *dev)
{

}

static int ft5301_i2c_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct ft5301_i2c_ts_data *tsdata;
	struct input_dev *input;
	const struct touch_platform_data *pdata =
					client->dev.platform_data;
	int error;
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    	
    int i;
#endif
	if (!pdata) {
		dev_err(&client->dev, "platform data not defined\n");
		return -EINVAL;
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata) {
		dev_err(&client->dev, "Failed to allocate driver data!\n");
		error = -ENOMEM;
		dev_set_drvdata(&client->dev, NULL);
		return error;
	}

	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "Failed to allocate input device!\n");
		error = -ENOMEM;
		input_free_device(input);
		kfree(tsdata);
	}

	init_waitqueue_head(&tsdata->wait);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->close = ft5301_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;
	tsdata->chip = pdata;
	tsdata->irq = client->irq;

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REP, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	//input_set_abs_params(input, ABS_X, 0, pdata->abs_xmax, 0, 0);
	//input_set_abs_params(input, ABS_Y, 0, pdata->abs_ymax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->abs_xmax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->abs_ymax, 0, 0);
    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, 25, 0, 0);

    //register kbd
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)        
    for (i=0; i<pdata->key_num; i++) {
        if(pdata->key[i].code <= 0)
            continue;
        set_bit(pdata->key[i].code, input->keybit);
        printk(KERN_INFO "%s key(%d) registed.\n", pdata->key[i].name, pdata->key[i].code);
    }
    
    input->phys = "FT5301-touch-kbd/input0";

    input->id.bustype = BUS_ISA;
    input->id.vendor = 0x0001;
    input->id.product = 0x0001;
    input->id.version = 0x0100;

    input->rep[REP_DELAY]=0xffffffff;
    input->rep[REP_PERIOD]=0xffffffff;

    input->keycodesize = sizeof(unsigned short);
    input->keycodemax = 0x1ff;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
    tsdata->early_suspend.suspend = ft5301_ts_early_suspend;
    tsdata->early_suspend.resume = ft5301_ts_late_resume;
    register_early_suspend(&tsdata->early_suspend);
#endif

	if (input_register_device(input)) {
		input_free_device(input);
		kfree(tsdata);
	}

    if(pdata->init_irq)
        pdata->init_irq();

    if(pdata->enable)
        pdata->enable(1);

	if (request_threaded_irq(tsdata->irq, NULL, ft5301_ts_isr,
			IRQF_TRIGGER_FALLING, client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
		return -1;
	}

	device_init_wakeup(&client->dev, 1);

    register_touchpad_dev(tsdata);

    ft5301_client = client;

	return 0;
}

static int ft5301_i2c_ts_remove(struct i2c_client *client)
{
	struct ft5301_i2c_ts_data *tsdata;

	tsdata = i2c_get_clientdata(client);
	wake_up(&tsdata->wait);
	free_irq(tsdata->irq, tsdata);
	input_unregister_device(tsdata->input);
    if(tsdata->config_class){
        if(tsdata->config_dev)
            device_destroy(tsdata->config_class,MKDEV(tsdata->config_major,0));
        class_destroy(tsdata->config_class);
	}
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);

    ft5301_client=NULL ;
	
	return 0;
}

#ifdef CONFIG_PM
static int ft5301_i2c_ts_suspend(struct device *dev)/*(struct i2c_client *client, pm_message_t mesg)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct ft5301_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
    printk("ft5301_i2c_ts_suspend\n");

    disable_irq_nosync(tsdata->irq);

    tsdata->suspend = 1;

	return 0;
}

static int ft5301_i2c_ts_resume(struct device *dev)/*(struct i2c_client *client)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct ft5301_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
    printk("ft5301_i2c_ts_resume\n");

    enable_irq(tsdata->irq);
    
    tsdata->suspend = 0;
    
	return 0;
}

static const struct dev_pm_ops ft5301_dev_pm_ops = {
	.suspend	= ft5301_i2c_ts_suspend,
	.resume		= ft5301_i2c_ts_resume,
};

#else
#define	ft5301_i2c_ts_suspend	NULL
#define	ft5301_i2c_ts_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5301_ts_early_suspend(struct early_suspend *h) {
	struct ft5301_i2c_ts_data *tsdata = container_of(h, struct ft5301_i2c_ts_data, early_suspend);
	//unsigned char buf[1] = {0x03};
	
	printk("ft5301_ts_early_suspend\n");
		
    //i2c_smbus_write_i2c_block_data(tsdata->client,0x3a,1,buf);

    disable_irq_nosync(tsdata->irq);

    tsdata->suspend = 1;
}

static void ft5301_ts_late_resume(struct early_suspend *h) {
	struct ft5301_i2c_ts_data *tsdata =  container_of(h, struct ft5301_i2c_ts_data, early_suspend);
	
	printk("ft5301_ts_late_resume\n");
    /*
    if(tsdata->chip->enable)
        tsdata->chip->enable(0);

    msleep(5);

    if(tsdata->chip->enable)
        tsdata->chip->enable(1);
    */
    tsdata->suspend = 0;

    enable_irq(tsdata->irq);
}
#endif

static const struct i2c_device_id ft5301_i2c_ts_id[] = {
	{ "ft5301_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5301_i2c_ts_id);

static struct i2c_driver ft5301_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ft5301_ts",
#ifndef CONFIG_HAS_EARLYSUSPEND
//#ifdef CONFIG_PM
		.pm	= &ft5301_dev_pm_ops,
#endif
	},
	.probe		= ft5301_i2c_ts_probe,
	.remove		= ft5301_i2c_ts_remove,
	.id_table	= ft5301_i2c_ts_id,
};

static int ft5301_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long ft5301_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static ssize_t ft5301_write(struct file *file,const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client = ft5301_client;
    char *data;

    int ret = 0;

	data = kmalloc(count,GFP_KERNEL);
	if (data==NULL)
		return -ENOMEM;
	if (copy_from_user(data,buf,count))	{ 	
		printk("CALIBRATION_FLAG copy_from_user error\n");
		kfree(data);
		return -EFAULT;
	}
	ret = i2c_master_send(client,data,count);
	if(ret!=count){
		printk("CALIBRATION_FLAG,Unable to write to i2c page for calibration!\n");
	}

	kfree(data);

	return ret;
}

static int ft5301_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ft5301_i2c_ts_fops =
{	.owner = THIS_MODULE,  
	.write = ft5301_write,
	.open = ft5301_open, 
	.unlocked_ioctl = ft5301_ioctl,
	.release = ft5301_release, 
};

static int __init ft5301_i2c_ts_init(void)
{
	/*ft5301_wq = create_singlethread_workqueue("ft5301_wq");
	if (!ft5301_wq)
		return -ENOMEM;*/
		
	return i2c_add_driver(&ft5301_i2c_ts_driver);
}

static void __exit ft5301_i2c_ts_exit(void)
{
	i2c_del_driver(&ft5301_i2c_ts_driver);
	/*if (ft5301_wq)
		destroy_workqueue(ft5301_wq);*/
}

MODULE_AUTHOR("Jianchun Bian<jcbian@xxxxxxxxxxxxx>,Dequan eng<dqmeng@xxxxxxxxxxxxx>");
MODULE_DESCRIPTION("FT5301 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

module_init(ft5301_i2c_ts_init);
module_exit(ft5301_i2c_ts_exit);
