/* drivers/input/touchscreen/novatek_i2c_ts.c
 *
 * Copyright (C) 2010-2011 novatek, Inc.
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

#include "novatek_i2c_ts.h"

#include <linux/adc_ts_key.h>
#include <linux/i2c/touch_common.h>


#define DEBUG

struct novatek_i2c_ts_data {
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

    int points;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void novatek_ts_early_suspend(struct early_suspend *h);
static void novatek_ts_late_resume(struct early_suspend *h);
#endif

static struct i2c_client *novatek_client;

static const struct file_operations novatek_i2c_ts_fops;

static int register_touchpad_dev(struct novatek_i2c_ts_data  *tsdata)
{
    int ret=0;
    strcpy(tsdata->config_name,"novatek_i2c_ts");
    ret=register_chrdev(0, tsdata->config_name, &novatek_i2c_ts_fops);
    if(ret<=0) {
        printk("novatek register char device error\r\n");
        return  ret ;
    }
    tsdata->config_major=ret;
    printk("adc touchpad major:%d\r\n",ret);
    tsdata->config_class=class_create(THIS_MODULE,tsdata->config_name);
    tsdata->config_dev=device_create(tsdata->config_class,	NULL,
    		MKDEV(tsdata->config_major,0),NULL,/*tsdata->config_name*/"novatek_i2c_ts%d",0);
    				
    return ret;
}

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    

int novatek_ts_key(struct novatek_i2c_ts_data *data,int status)
{
    struct novatek_i2c_ts_data *tsdata = data;

    const struct touch_platform_data *pdata = tsdata->chip;
    int i;

    if(status > (1<<(pdata->key_num + 1))){
        printk(KERN_INFO "jumper novatek key 0x%x\n",status);
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

static int navatek_scan_touch_number(int last_points)
{
    const int next_point_array[] = {1,2,3,5,8,13,POINTS_MAX};
    
    int i;
    
    for(i = 0; i < ARRAY_SIZE(next_point_array); i++){
        if(last_points < next_point_array[i]){
            last_points = next_point_array[i];
            break;
        }
    }

    if(last_points > POINTS_MAX)
        last_points = POINTS_MAX;

    return last_points;
}

static int navatek_get_touch_number(const u8 rdbuf[PROTOCOL_LEN],int points)
{
    u8 touching = 0;
    u16 offset,action,pid;
    int i;
    
    for(i = 0; i < points; i++){
        offset = I2C_STARTTCH_READ + PROTOCOL_HEAD_LEN + (i*PROTOCOL_POINT_LEN);
        if(rdbuf[offset] == 0xff)
            break;
        
        action = rdbuf[offset] & 0x3;
        pid = rdbuf[offset]>>3;
        /*
        printk("t %d a %d id %d\n",touching,action,pid);        
        printk("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
            rdbuf[offset],
            rdbuf[offset+1],
            rdbuf[offset+2],
            rdbuf[offset+3],
            rdbuf[offset+4],
            rdbuf[offset+5]);
        */
        if(action == 1 || action == 2){
            if(pid < BUTTON_PID_START){
                touching++;
            }else{
                touching = BUTTON_MASK;
                break;
            }
        }else{
            break;
        }  
    } 

    return touching;
}

static int novatek_ts_poscheck(struct novatek_i2c_ts_data *data)
{
    struct novatek_i2c_ts_data *tsdata = data;
    const struct touch_platform_data *pdata = tsdata->chip;

    u8 touching,points,len;
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
    points = navatek_scan_touch_number(tsdata->points);
    len = PROTOCOL_HEAD_LEN + (points*PROTOCOL_POINT_LEN);
    ret = i2c_smbus_read_i2c_block_data(tsdata->client, PROTOCOL_ADDR + I2C_STARTTCH_READ, len , &rdbuf[I2C_STARTTCH_READ]);
    if(ret == len){
        touching = navatek_get_touch_number(rdbuf,points);       
#if defined(DEBUG)       
        printk("novatek %d(%d) level %d 0x%x,0x%x,0x%x 0x%x ",touching,points,tsdata->chip->get_irq_level(),rdbuf[0],rdbuf[1],rdbuf[2],rdbuf[3]);
#endif

    }else{
        dev_err(&tsdata->client->dev, "novatek data error buf[0~3] = 0x%x,0x%x,0x%x ret %d(%d)\n",rdbuf[0],rdbuf[1],rdbuf[2],ret,len);
        memset(rdbuf,0,sizeof(rdbuf));
        touching = 0;
    }

    if(touching && !(touching & BUTTON_MASK)){
        for(i = 0; i < touching; i++) {
            offset = I2C_STARTTCH_READ + PROTOCOL_HEAD_LEN + (i*PROTOCOL_POINT_LEN);
            high_byte = rdbuf[offset+1];
            high_byte <<= 4;
            low_byte = rdbuf[offset + 3] >> 4;
            posx = (high_byte |low_byte);
            
            high_byte = rdbuf[offset+2];
            high_byte <<= 4;
            low_byte = rdbuf[offset+3] & 0xf;
            posy = (high_byte |low_byte);

            if((posx <= pdata->abs_xmin) && (posy <= pdata->abs_ymin))
                continue;

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
    if(touching & BUTTON_MASK){
        if(rdbuf[0] & 0x1){
            key_status |= 1<<((rdbuf[0]>>3) - BUTTON_PID_START);
        }
    }
    
    tsdata->key_status = 
        pdata->report_key(tsdata->input,pdata->key,pdata->key_num,tsdata->key_status,key_status);
#endif

    tsdata->points = (touching & ~BUTTON_MASK);

    return touching;
}



static irqreturn_t novatek_ts_isr(int irq, void *dev_id)
{
	struct novatek_i2c_ts_data *tsdata = dev_id;
    int touching;

	do {
		touching = novatek_ts_poscheck(tsdata);
		tsdata->attb_pin = tsdata->chip->get_irq_level();
        
		if (tsdata->attb_pin == ATTB_PIN_LOW || touching){
            msleep(5);
	    }
	} while (!tsdata->suspend && (tsdata->attb_pin == ATTB_PIN_LOW || touching));

	return IRQ_HANDLED;
}

static void novatek_ts_close(struct input_dev *dev)
{

}

static int novatek_i2c_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct novatek_i2c_ts_data *tsdata;
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

	input->close = novatek_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;
	tsdata->chip = pdata;
	tsdata->irq = client->irq;

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REP, input->evbit);
	set_bit(EV_ABS, input->evbit);
	//set_bit(BTN_TOUCH, input->keybit);
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
    
    input->phys = "NOVATEK-touch-kbd/input0";

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
    tsdata->early_suspend.suspend = novatek_ts_early_suspend;
    tsdata->early_suspend.resume = novatek_ts_late_resume;
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

#if !defined(CONFIG_IMAGE_RECOVERY)
//#if 0
	if (request_threaded_irq(tsdata->irq, NULL, novatek_ts_isr,
			IRQF_TRIGGER_FALLING, client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
		return -1;
	}

	device_init_wakeup(&client->dev, 1);
#endif

    register_touchpad_dev(tsdata);

    novatek_client = client;

	return 0;
}

static int novatek_i2c_ts_remove(struct i2c_client *client)
{
	struct novatek_i2c_ts_data *tsdata;

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

    novatek_client=NULL ;
	
	return 0;
}

#ifdef CONFIG_PM
static int novatek_i2c_ts_suspend(struct device *dev)/*(struct i2c_client *client, pm_message_t mesg)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct novatek_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
    printk("novatek_i2c_ts_suspend\n");

    disable_irq_nosync(tsdata->irq);

    tsdata->suspend = 1;

	return 0;
}

static int novatek_i2c_ts_resume(struct device *dev)/*(struct i2c_client *client)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct novatek_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
    printk("novatek_i2c_ts_resume\n");

    enable_irq(tsdata->irq);
    
    tsdata->suspend = 0;
    
	return 0;
}

static const struct dev_pm_ops novatek_dev_pm_ops = {
	.suspend	= novatek_i2c_ts_suspend,
	.resume		= novatek_i2c_ts_resume,
};

#else
#define	novatek_i2c_ts_suspend	NULL
#define	novatek_i2c_ts_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void novatek_ts_early_suspend(struct early_suspend *h) {
	struct novatek_i2c_ts_data *tsdata = container_of(h, struct novatek_i2c_ts_data, early_suspend);
	//unsigned char buf[2] = {0x1};
	
	printk("novatek_ts_early_suspend\n");
		
    //i2c_smbus_write_i2c_block_data(tsdata->client,0x80,1,buf);

    disable_irq_nosync(tsdata->irq);

    tsdata->suspend = 1;
}

static void novatek_ts_late_resume(struct early_suspend *h) {
	struct novatek_i2c_ts_data *tsdata =  container_of(h, struct novatek_i2c_ts_data, early_suspend);
	
	printk("novatek_ts_late_resume\n");
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

static const struct i2c_device_id novatek_i2c_ts_id[] = {
	{ "novatek_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, novatek_i2c_ts_id);

static struct i2c_driver novatek_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "novatek_ts",
#ifndef CONFIG_HAS_EARLYSUSPEND
//#ifdef CONFIG_PM
		.pm	= &novatek_dev_pm_ops,
#endif
	},
	.probe		= novatek_i2c_ts_probe,
	.remove		= novatek_i2c_ts_remove,
	.id_table	= novatek_i2c_ts_id,
};

static int novatek_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long novatek_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static ssize_t novatek_write(struct file *file,const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client = novatek_client;
    char *data;

    int ret = 0;

	data = kmalloc(count,GFP_KERNEL);
	if (data==NULL)
		return -ENOMEM;
	if (copy_from_user(data,buf,count))	{ 	
		printk("novatek_write copy_from_user error\n");
		kfree(data);
		return -EFAULT;
	}
	ret = i2c_master_send(client,data,count);
	if(ret!=count){
		printk("novatek_write,Unable to write to i2c page(ret %d count %d)!\n",ret,count);
	}

	kfree(data);

	return ret;
}

static ssize_t novatek_read(struct file *file,char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client = novatek_client;
    char *data;

    int ret = 0;

	data = kmalloc(count,GFP_KERNEL);
	if (data==NULL)
		return -ENOMEM;
		
	if (copy_from_user(data,buf,1))	{ 	
		printk("novatek_read copy_from_user error\n");
		kfree(data);
		return -EFAULT;
	}

    ret = i2c_smbus_read_i2c_block_data(client, data[0], count , &data[1]);
    if(ret != count){
		printk("novatek_read Unable to read to i2c page(ret %d count %d)\n",ret,count);
		kfree(data);
		return -EFAULT;    
    }

	if (copy_to_user(buf,&data[1],count))	{ 	
		printk("novatek_read copy_to_user error\n");
		kfree(data);
		return -EFAULT;
	}

	kfree(data);

	return ret;
}


static int novatek_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations novatek_i2c_ts_fops =
{	.owner = THIS_MODULE,  
	.write = novatek_write,
	.read = novatek_read,
	.open = novatek_open, 
	.unlocked_ioctl = novatek_ioctl,
	.release = novatek_release, 
};

static int __init novatek_i2c_ts_init(void)
{
	/*novatek_wq = create_singlethread_workqueue("novatek_wq");
	if (!novatek_wq)
		return -ENOMEM;*/
		
	return i2c_add_driver(&novatek_i2c_ts_driver);
}

static void __exit novatek_i2c_ts_exit(void)
{
	i2c_del_driver(&novatek_i2c_ts_driver);
	/*if (novatek_wq)
		destroy_workqueue(novatek_wq);*/
}

MODULE_AUTHOR("Jianchun Bian<jcbian@xxxxxxxxxxxxx>,Dequan eng<dqmeng@xxxxxxxxxxxxx>");
MODULE_DESCRIPTION("NOVATEK I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

module_init(novatek_i2c_ts_init);
module_exit(novatek_i2c_ts_exit);
