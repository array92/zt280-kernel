/* drivers/input/touchscreen/pixcir_i2c_ts.c
 *
 * Copyright (C) 2010-2011 Pixcir, Inc.
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

#include "pixcir_c44_i2c_ts.h"

#include <linux/adc_ts_key.h>
#include <linux/i2c/touch_common.h>

//#define DEBUG


struct point_node_t{
	unsigned char 	active ;
	unsigned char	finger_id;
	unsigned int	posx;
	unsigned int	posy;
};

struct pixcir_i2c_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	//wait_queue_head_t wait;
	const struct touch_platform_data *chip;
	unsigned int attb_pin;
	int irq;

	int config_major;
	char config_name[20];
	struct class *config_class;
	struct device *config_dev;	

	int suspend;
    int key_status;  //max support 32 key

    int calibrate;
    int checktime;

    struct point_node_t points[PROTOCOL_POINT_LEN*2];
    
    struct workqueue_struct *queue;
    struct delayed_work work;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_i2c_early_suspend(struct early_suspend *h);
static void pixcir_i2c_late_resume(struct early_suspend *h);
#endif
static struct i2c_client *pixcir_client;

static const struct file_operations pixcir_i2c_ts_fops;

static int register_touchpad_dev(struct pixcir_i2c_ts_data  *tsdata)
{
    int ret=0;
    strcpy(tsdata->config_name,"pixcir_i2c_ts");
    ret=register_chrdev(0, tsdata->config_name, &pixcir_i2c_ts_fops);
    if(ret<=0) {
        printk(KERN_ERR "pixcir register char device error\r\n");
        return  ret ;
    }
    tsdata->config_major=ret;
    printk(KERN_INFO "adc touchpad major:%d\r\n",ret);
    tsdata->config_class=class_create(THIS_MODULE,tsdata->config_name);
    tsdata->config_dev=device_create(tsdata->config_class,	NULL,
    		MKDEV(tsdata->config_major,0),NULL,/*tsdata->config_name*/"pixcir_i2c_ts%d",0);
    				
    return ret;
}

int pixcir_ts_suspend(struct pixcir_i2c_ts_data *tsdata,int suspend)
{

	/**************************************************************
	wrbuf[1]:	0x00: Active mode
			0x01: Sleep mode
			0xA4: Sleep mode automatically switch
			0x03: Freeze mode
	More details see application note 710 power manangement section
	****************************************************************/

    u8 value;
    int ret;
    
    if(suspend)
        //value = 0x2;  //Deep Sleep Mode
        value = 0x3; //Freeze mode
    else
        value = 0;    //active
    ret = i2c_smbus_write_i2c_block_data(tsdata->client, 0x33, 1, &value);
    if(ret){
        printk(KERN_ERR "pixcir_ts_suspend failed %d\n",ret);
    }

    return ret;

    return 0;
}

int pixcir_ts_int_init(struct pixcir_i2c_ts_data *tsdata)
{
    return 0;
}

int pixcir_ts_calibration(struct pixcir_i2c_ts_data *tsdata)
{
    u8 value = 0x03;
    int ret;
    ret = i2c_smbus_write_i2c_block_data(tsdata->client, 0x3A, 1, &value);
    if(ret){
        printk(KERN_ERR "pixcir_ts_calibration failed %d\n",ret);
    }

    return ret;
}

void do_pixcir_ts_calibration(void)
{
    struct i2c_client *client = pixcir_client;
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

    if(!tsdata->suspend)
        pixcir_ts_calibration(tsdata);
}

int pixcir_calibration_status(struct pixcir_i2c_ts_data *tsdata,int calibrate)
{
    return (tsdata->calibrate == calibrate);
}

int pixcir_need_calibration(struct pixcir_i2c_ts_data *tsdata)
{
    if(!tsdata->checktime)
        tsdata->checktime = jiffies;

#if defined(CONFIG_PIXCIR_SKIP_CALIBRATION)||!defined(CONFIG_IMAGE_RECOVERY)
    tsdata->calibrate = PIXCIR_CALIBRATE_SUCCESS;
    return 0;
#endif
    
    //don't calibrate when ac on
#if !defined(CONFIG_IMAGE_RECOVERY)
    if(tsdata->chip->is_ac_online && (*tsdata->chip->is_ac_online)()){
        tsdata->calibrate = PIXCIR_CALIBRATE_SUCCESS;
        return 0;
    }
#endif

    return ((jiffies - tsdata->checktime) < MAX_CALIBRATE_CHECKTIME &&
                ((tsdata->calibrate != PIXCIR_CALIBRATE_SUCCESS)&&
                    (tsdata->calibrate != PIXCIR_CALIBRATE_FAILED)));

}

void pixcir_set_calibration(struct pixcir_i2c_ts_data *tsdata,int calibrate)
{
    if(!tsdata->checktime)
        tsdata->checktime = jiffies;

    if(tsdata->calibrate != PIXCIR_CALIBRATE_SUCCESS){
        if((jiffies - tsdata->checktime) >= MAX_CALIBRATE_CHECKTIME)
            calibrate = PIXCIR_CALIBRATE_FAILED;
    }
    tsdata->calibrate = calibrate;
}

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    

int pixcir_ts_key(struct pixcir_i2c_ts_data *data,int status)
{
    struct pixcir_i2c_ts_data *tsdata = data;
    const struct touch_platform_data *pdata = tsdata->chip;
    int change = 0;
    int i;

    if(status > (1<<(pdata->key_num + 1))){
        printk(KERN_INFO "jumper pixcir key 0x%x\n",status);
        status = 0;
    }

    if(status != tsdata->key_status){
        change = tsdata->key_status^status;

        for (i = 0; i < pdata->key_num; i++) {
            if((change >> i)&0x1){
#if defined(DEBUG)            
                printk(KERN_INFO "%s key(%d) press %d\n", pdata->key[i].name, pdata->key[i].code,(status >> i)&0x1);
#endif
                input_event(tsdata->input, EV_KEY, pdata->key[i].code,(status >> i)&0x1);
            }
        }

        tsdata->key_status = status;
    }
    return !!change;
}
#endif



static int pixcir_ts_precheck(struct pixcir_i2c_ts_data *data)
{
	struct pixcir_i2c_ts_data *tsdata = data;

	int touching = -1;
	u8 rdbuf[2];
	int ret;

	memset(rdbuf, 0, sizeof(rdbuf));

    //touch event
	ret = i2c_smbus_read_i2c_block_data(tsdata->client, 0x0, 2, &rdbuf[0]);
    if(ret == 2){
    	touching = rdbuf[0] & 0x7;
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)            	
    	touching |= rdbuf[1]?(1<<4):0; //key event
#endif
    }else{
		dev_err(&tsdata->client->dev, "pixcir unable to read i2c page 0 ret %d\n",ret);
    }
    
	return touching;
}

static int pixcir_ts_precalibrate(struct pixcir_i2c_ts_data *data)
{
	struct pixcir_i2c_ts_data *tsdata = data;
    int calibrate = tsdata->calibrate;
    int touching;
    int i;

    if(
#if defined(CONFIG_IMAGE_RECOVERY)            
        pixcir_calibration_status(tsdata,PIXCIR_CALIBRATE_UNKNOW)||
#endif
        pixcir_calibration_status(tsdata,PIXCIR_CALIBRATING)
        ){

#if defined(CONFIG_IMAGE_RECOVERY)
#   define FORCE_CALIBRATE_TIME 2
#else
#   define FORCE_CALIBRATE_TIME 1
#endif 
        for(i = 0; i < FORCE_CALIBRATE_TIME; i++){
            printk(KERN_INFO "pixcir step0 : force calibrating (%d)\n",i);
            calibrate = PIXCIR_CALIBRATING;
            if(pixcir_ts_calibration(tsdata)!=0){
                printk(KERN_ERR "pixcir force calibrating failed (%d)\n",i);
                goto out;
            }
            msleep(3000);
        }
    }


    printk(KERN_INFO "pixcir step1 : wait touch idle\n");
    for(i = 0;i < 10 && !tsdata->suspend; i++){
        if(tsdata->chip->get_irq_level() == ATTB_PIN_LOW||
            (touching = pixcir_ts_precheck(tsdata)) > 0){
            printk(KERN_ERR "calibrating wait idle failed\n");
            calibrate = PIXCIR_CALIBRATE_FAILED;
            goto out;
        }
        msleep(100);
    }

    if(i < 10 || touching != 0)
        goto out;

    printk(KERN_INFO "pixcir step2 : start calibrating\n");
    calibrate = PIXCIR_CALIBRATING;
    if(pixcir_ts_calibration(tsdata)!=0){
        printk(KERN_INFO "pixcir calibrating failed\n");
        goto out;
    }
    msleep(200);

    printk(KERN_INFO "pixcir step3 : check touch idle\n");
    for(i = 0;i < 30 && !tsdata->suspend; i++){
        if(tsdata->chip->get_irq_level() == ATTB_PIN_LOW ||
            (touching = pixcir_ts_precheck(tsdata))>0){
            printk(KERN_ERR "calibrating check touch idle failed\n");
            goto out;
        }
        msleep(100);
    }

    if(i < 30 || touching != 0)
       goto out;

    printk(KERN_INFO "pixcir calibrating success\n");
    calibrate = PIXCIR_CALIBRATE_SUCCESS;
out:

    pixcir_set_calibration(tsdata,calibrate);

    return !(calibrate == PIXCIR_CALIBRATE_SUCCESS);
}

static int pixcir_ts_poscheck(struct pixcir_i2c_ts_data *data)
{
	struct pixcir_i2c_ts_data *tsdata = data;
	const struct touch_platform_data *pdata = tsdata->chip;

	u8 touching;
	u16 offset,pix_id,slot_id,high_byte,low_byte;
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
        (rdbuf[0]&0x7) <= POINTS_MAX){
    	touching = rdbuf[0]&0x7;

#if defined(DEBUG)       
        printk("pixcir %d level %d 0x%x,0x%x,0x%x 0x%x ",touching,tsdata->chip->get_irq_level(),rdbuf[0],rdbuf[1],rdbuf[2],rdbuf[3]);
#endif

    }else{
        dev_err(&tsdata->client->dev, "pixcir data error buf[0~3] = 0x%x,0x%x,0x%x ret %d\n",rdbuf[0],rdbuf[1],rdbuf[2],ret);
        memset(rdbuf,0,sizeof(rdbuf));
        touching = 0;
    }

    if(touching){
        if(touching > 1){
            //touch event(get point left)
            ret = i2c_smbus_read_i2c_block_data(tsdata->client, PROTOCOL_ADDR + I2C_STARTTCH_POINT1, (touching - 1) * PROTOCOL_POINT_LEN, &rdbuf[I2C_STARTTCH_POINT1]);
            if(ret == (touching - 1) * PROTOCOL_POINT_LEN){
            }else{
                dev_err(&tsdata->client->dev, "pixcir data point1(%d) error ret %d\n",touching,ret);
                //memset(rdbuf,0,sizeof(rdbuf));
                touching = 1;
            }
        }
    }
    
    if(touching){
        for(i = 0; i < touching; i++) {
            offset = I2C_STARTTCH_READ + PROTOCOL_HEAD_LEN + (i*PROTOCOL_POINT_LEN);
            high_byte = rdbuf[offset+1];
            high_byte <<= 8;
            low_byte = rdbuf[offset];
            posx = (high_byte |low_byte);
            
            high_byte = rdbuf[offset+3];
            high_byte <<= 8;
            low_byte = rdbuf[offset+2];
            posy = (high_byte |low_byte);
            if((posx <= pdata->abs_xmin) && (posy <= pdata->abs_ymin))
                continue;

            pix_id = rdbuf[offset+4];
            slot_id = ((pix_id & 7)<<1) | ((pix_id & 8)>>3);
            //slot_id = i;

            if(slot_id >= ARRAY_SIZE(tsdata->points))
                continue;
#if defined(DEBUG)
            //printk("[0x%x 0x%x 0x%x 0x%x] ",rdbuf[offset],rdbuf[offset+1],rdbuf[offset+2],rdbuf[offset+3]);
            printk("(%d,%d,0x%x,%d) ",posx,posy,pix_id,slot_id);
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
            tsdata->points[slot_id].active = POINTS_ACTIVE_CYCLE;
            tsdata->points[slot_id].finger_id = pix_id; 
            tsdata->points[slot_id].posx = posx;
            tsdata->points[slot_id].posy = posy;
        }
    }

    if(touching){
        for(i = 0; i < PROTOCOL_POINT_LEN*2; i++) {
            posx = tsdata->points[i].posx;
            posy = tsdata->points[i].posy;
            
            if(tsdata->points[i].active){
                tsdata->points[i].active--;
            }else{
                memset(&tsdata->points[i],0,sizeof(tsdata->points[i]));
            }
            if(posx || posy){
                printk("[%d,%d] ",posx,posy);
                input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, z);
                //input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, w); 
                //input_report_key(tsdata->input, ABS_MT_TRACKING_ID, i);
                input_report_abs(tsdata->input, ABS_MT_POSITION_X, posx);
                input_report_abs(tsdata->input, ABS_MT_POSITION_Y, posy);
                input_mt_sync(tsdata->input);
            }
        }
    }else{
        memset(&tsdata->points[0],0,sizeof(tsdata->points));
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


static void pixcir_ts_work(struct work_struct *work)
{
	struct pixcir_i2c_ts_data *tsdata = container_of(work,struct pixcir_i2c_ts_data,work.work);
#if !defined(CONFIG_IMAGE_RECOVERY)    
    int points;
#endif

    if(pixcir_need_calibration(tsdata)){        
        pixcir_ts_precalibrate(tsdata);
        queue_delayed_work(tsdata->queue, &tsdata->work, HZ);
    }else{
#if !defined(CONFIG_IMAGE_RECOVERY)    
    	do {
    		points = pixcir_ts_poscheck(tsdata);
            
    		if (points > 0){
                msleep(2);
            }          
    	} while (!tsdata->suspend && (points || !tsdata->chip->get_irq_level()));
#endif
    }
}

static irqreturn_t pixcir_ts_isr(int irq,void *dev_id)
{
    struct pixcir_i2c_ts_data *tsdata = dev_id;

#if defined(DEBUG)
    printk("pixcir_ts_isr\n");
#endif
    if(pixcir_calibration_status(tsdata,PIXCIR_CALIBRATE_SUCCESS)||
        pixcir_calibration_status(tsdata,PIXCIR_CALIBRATE_FAILED));
        queue_delayed_work(tsdata->queue, &tsdata->work, 1);

    return IRQ_HANDLED;
}



static void pixcir_ts_close(struct input_dev *dev)
{

}

static int pixcir_i2c_ts_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct pixcir_i2c_ts_data *tsdata;
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

	//init_waitqueue_head(&tsdata->wait);
	tsdata->queue = create_singlethread_workqueue(client->name);
	INIT_DELAYED_WORK(&tsdata->work, pixcir_ts_work);

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	input->close = pixcir_ts_close;

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;
	tsdata->chip = pdata;
	tsdata->irq = client->irq;
    tsdata->calibrate = PIXCIR_CALIBRATE_UNKNOW;

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_REP, input->evbit);
	set_bit(EV_ABS, input->evbit);
	//set_bit(BTN_TOUCH, input->keybit);
	//input_set_abs_params(input, ABS_X, 0, pdata->abs_xmax, 0, 0);
	//input_set_abs_params(input, ABS_Y, 0, pdata->abs_ymax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->abs_xmax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->abs_ymax, 0, 0);
	//input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
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
    
    input->phys = "PIXCIR-touch-kbd/input0";

    input->id.bustype = BUS_ISA;
    input->id.vendor = 0x0001;
    input->id.product = 0x0001;
    input->id.version = 0x0100;

    input->rep[REP_DELAY]=0xffffffff;
    input->rep[REP_PERIOD]=0xffffffff;

    input->keycodesize = sizeof(unsigned short);
    input->keycodemax = 0x1ff;
#endif

	if (input_register_device(input)) {
		input_free_device(input);
		kfree(tsdata);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
        tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
        tsdata->early_suspend.suspend = pixcir_i2c_early_suspend;
        tsdata->early_suspend.resume = pixcir_i2c_late_resume;
        register_early_suspend(&tsdata->early_suspend);
#endif

    if(pdata->init_irq)
        pdata->init_irq();

    if(pdata->enable)
        pdata->enable(1);

    /*
	if (request_threaded_irq(tsdata->irq, NULL, pixcir_ts_isr,
			IRQF_TRIGGER_FALLING, client->name, tsdata)) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		input_unregister_device(input);
		input = NULL;
		return -1;
	}*/

    pixcir_ts_int_init(tsdata);
	
    error = request_irq(tsdata->irq, pixcir_ts_isr, IRQF_TRIGGER_FALLING,client->name, tsdata);
    if(error < 0){
        dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
        input_unregister_device(tsdata->input);
        tsdata->input = NULL;
        return error;
    }
	device_init_wakeup(&client->dev, 1);

    register_touchpad_dev(tsdata);

    pixcir_client = client;

    queue_delayed_work(tsdata->queue, &tsdata->work, HZ*5);

	return 0;
}

static int pixcir_i2c_ts_remove(struct i2c_client *client)
{
	struct pixcir_i2c_ts_data *tsdata;

	tsdata = i2c_get_clientdata(client);
	//wake_up(&tsdata->wait);
	free_irq(tsdata->irq, tsdata);
	input_unregister_device(tsdata->input);
    if(tsdata->config_class){
        if(tsdata->config_dev)
            device_destroy(tsdata->config_class,MKDEV(tsdata->config_major,0));
        class_destroy(tsdata->config_class);
	}
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);

    pixcir_client=NULL ;
	
	return 0;
}

#ifdef CONFIG_PM
static int pixcir_i2c_ts_suspend(struct device *dev)/*(struct i2c_client *client, pm_message_t mesg)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
    printk(KERN_INFO "pixcir_i2c_ts_suspend\n");

    tsdata->suspend++;

	pixcir_ts_suspend(tsdata,1);
	
	return 0;
}

static int pixcir_i2c_ts_resume(struct device *dev)/*(struct i2c_client *client)*/
{
    struct i2c_client   *client = i2c_verify_client(dev);
	struct pixcir_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);

    printk(KERN_INFO "pixcir_i2c_ts_resume\n");
    if(tsdata->chip->enable)
        tsdata->chip->enable(0);

	return 0;
}

static const struct dev_pm_ops pixcir_dev_pm_ops = {
	.suspend	= pixcir_i2c_ts_suspend,
	.resume		= pixcir_i2c_ts_resume,
};

#else
#define	pixcir_i2c_ts_suspend	NULL
#define	pixcir_i2c_ts_resume	NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void pixcir_i2c_early_suspend(struct early_suspend *h) {
	struct pixcir_i2c_ts_data *ts;
	ts = container_of(h, struct pixcir_i2c_ts_data, early_suspend);

	printk(KERN_INFO "pixcir_i2c_early_suspend\n");
	
	disable_irq_nosync(ts->irq);
	ts->suspend++;
}

static void pixcir_i2c_late_resume(struct early_suspend *h) {
	struct pixcir_i2c_ts_data *ts;
	ts = container_of(h, struct pixcir_i2c_ts_data, early_suspend);
	
	printk(KERN_INFO "pixcir_i2c_late_resume\n");
#if defined(DEBUG)	
    printk(KERN_INFO "suspend %d\n",ts->suspend);
#endif

    if(ts->suspend > 1){
        if(ts->chip->enable)
            ts->chip->enable(1);

        ts->suspend--;
        //msleep(200);
    }

    ts->suspend = 0;
    
    enable_irq(ts->irq);
        
    queue_delayed_work(ts->queue, &ts->work, HZ/10);
}
#endif


static const struct i2c_device_id pixcir_i2c_ts_id[] = {
	{ "pixcir_c44_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pixcir_i2c_ts_id);

static struct i2c_driver pixcir_i2c_ts_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "pixcir_c44_ts",
#if defined(CONFIG_PM)
		.pm	= &pixcir_dev_pm_ops,
#endif
	},
	.probe		= pixcir_i2c_ts_probe,
	.remove		= pixcir_i2c_ts_remove,
	.id_table	= pixcir_i2c_ts_id,
};

static int pixcir_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long pixcir_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static ssize_t pixcir_write(struct file *file,const char __user *buf,size_t count, loff_t *ppos)
{
	struct i2c_client *client = pixcir_client;
    char *data;

    int ret = 0;

	data = kmalloc(count,GFP_KERNEL);
	if (data==NULL)
		return -ENOMEM;
	if (copy_from_user(data,buf,count))	{ 	
		printk(KERN_ERR "CALIBRATION_FLAG copy_from_user error\n");
		kfree(data);
		return -EFAULT;
	}
	ret = i2c_master_send(client,data,count);
	if(ret!=count){
		printk(KERN_ERR "CALIBRATION_FLAG,Unable to write to i2c page for calibration!\n");
	}

	kfree(data);

	return ret;
}

static int pixcir_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations pixcir_i2c_ts_fops =
{	.owner = THIS_MODULE,  
	.write = pixcir_write,
	.open = pixcir_open, 
	.unlocked_ioctl = pixcir_ioctl,
	.release = pixcir_release, 
};

static int __init pixcir_i2c_ts_init(void)
{
	/*pixcir_wq = create_singlethread_workqueue("pixcir_wq");
	if (!pixcir_wq)
		return -ENOMEM;*/
		
	return i2c_add_driver(&pixcir_i2c_ts_driver);
}

static void __exit pixcir_i2c_ts_exit(void)
{
	i2c_del_driver(&pixcir_i2c_ts_driver);
	/*if (pixcir_wq)
		destroy_workqueue(pixcir_wq);*/
}

MODULE_AUTHOR("Jianchun Bian<jcbian@xxxxxxxxxxxxx>,Dequan eng<dqmeng@xxxxxxxxxxxxx>");
MODULE_DESCRIPTION("Pixcir I2C Touchscreen Driver");
MODULE_LICENSE("GPL");

module_init(pixcir_i2c_ts_init);
module_exit(pixcir_i2c_ts_exit);
