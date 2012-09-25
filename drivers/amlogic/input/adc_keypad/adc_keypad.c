
/*
 * linux/drivers/input/adc_kbd/adc_keypad.c
 *
 * ADC Keypad Driver
 *
 * Copyright (C) 2010 Amlogic Corporation
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * author :   Robin Zhu
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <asm/irq.h>
#include <asm/io.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/saradc.h>
#include <linux/adc_keypad.h>
#include <mach/gpio.h>

#define DEBUG

struct kp {
    struct platform_device *pdev;
	struct input_dev *input;

	int config_major;
	char config_name[20];
	struct class *config_class;
	struct device *config_dev;
	int chan[SARADC_CHAN_NUM];
	int chan_num;

    struct workqueue_struct *queue;
    struct delayed_work work;
    const struct key_list *list;

	int suspend;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif

    int key_status;
#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)
    int fn;   //record fn key status
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kp_early_suspend(struct early_suspend *h);
static void kp_late_resume(struct early_suspend *h);
#endif

static struct kp *gp_kp=NULL;


static int search_key(struct kp *kp,int *fn_sts)
{
	const struct key_list *list = kp->list;
	int value,delta, i, j;

    int status = 0,status_shift = 0;
    int retry = 10;

#define ADC_JITTA 5

    //ADC KEY
	
	for (i=0; i<kp->chan_num; i++){
	    //scan adc channel i
        do{
            value = get_adc_sample(kp->chan[i]);
            mdelay(10);
            delta = value - get_adc_sample(kp->chan[i]);
            if(delta < 0)
                delta = -delta;
        }while(delta > ADC_JITTA && --retry);

        if(!retry){
            printk("adc %d delta %d\n", value,delta);
            continue;
        }

#if defined(DEBUG)
        /*
        if(value < 1020)
            printk(KERN_INFO "adc[%d] %d\n",kp->chan[i],value);*/
#endif        
        //compare adc value with defined value
		for (j=0; j<list->key_num; j++){		
			if ((list->key[j].chan == kp->chan[i])
				&& (value >= list->key[j].value - list->key[j].tolerance)
				&& (value <= list->key[j].value + list->key[j].tolerance)){
			    //printk("adc[%d] key code %d \n",key->chan,key->code);
                status |= (1<<j);
                break;   //one channel only support one key
			}
		}
	}
    status_shift += list->key_num;

    //GPIO KEY
#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)
	for (j=0; j<list->gpio_key_num; j++){
        if(get_gpio_val(list->gpio_key[j].bank,list->gpio_key[j].bit)==list->gpio_key[j].checkdata)
            status |= (1<<(j+status_shift));
	}
	status_shift += list->gpio_key_num;
#endif
    //FN KEY
#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)
    if(fn_sts&&list->fn_pressed && (*list->fn_pressed)() == 0)
        *fn_sts = 1;
#endif
	return status;
}


void report_key(struct kp *kp,int sts,int fn)
{
	const struct key_list *list = kp->list;
	int status = sts,status_shift = 0;
	int code;
    int i;

    if(status != kp->key_status){
        int change = kp->key_status^status;
#if defined(DEBUG)            
        //printk(KERN_INFO "status 0x%x key_status 0x%x change 0x%x\n", status,kp->key_status,change);
#endif         
        //ADC KEY
        for (i = 0; i < list->key_num; i++) {
            if((change >> i)&0x1){
                code = list->key[i].code;
#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)
                if(list->fn_convert)
                    code = (*list->fn_convert)(list->fn_key,list->fn_key_num,code,(status >> i)&0x1,fn);
#endif
#if defined(DEBUG)            
                printk(KERN_INFO "key(%d) press %d\n", code,(status >> i)&0x1);
#endif
                if(code > 0)
                    input_event(kp->input, EV_KEY, code,(status >> i)&0x1);
            }
        }
        status_shift += list->key_num;

        //GPIO KEY
#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)
        status >>= status_shift;
        change >>= status_shift;
        for (i = 0; i < list->gpio_key_num; i++) {
            if((change >> i)&0x1){
                code = list->gpio_key[i].code;
#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)
                if(list->fn_convert)
                    code = (*list->fn_convert)(list->fn_key,list->fn_key_num,code,(status >> i)&0x1,fn);
#endif

#if defined(DEBUG)
                printk(KERN_INFO "key(%d) press %d\n", code,(status >> i)&0x1);
#endif
                if(code > 0)
                    input_event(kp->input, EV_KEY, code,(status >> i)&0x1);
            }
        }
        status_shift += list->gpio_key_num;
#endif        
    }
    
    kp->key_status = sts;
#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)    
    kp->fn = fn;
#endif
}

static void kp_work(struct work_struct *work)
{
	struct kp *kp_data = container_of(work,struct kp,work.work);
    int status;
    int fn = 0;

    status = search_key(kp_data,&fn);

#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)    
    if(kp_data->fn != fn && kp_data->key_status){  //emulate all key unpress when fn change
        report_key(kp_data,0,kp_data->fn);
        msleep(10);
    }
#endif
    report_key(kp_data,status,fn);

    if(!kp_data->suspend)
        queue_delayed_work(kp_data->queue, &kp_data->work, CONFIG_ADC_KEYPADS_SCAN_TICKS);
}


static int
adckpd_config_open(struct inode *inode, struct file *file)
{
    file->private_data = gp_kp;
    return 0;
}

static int
adckpd_config_release(struct inode *inode, struct file *file)
{
    file->private_data=NULL;
    return 0;
}

static const struct file_operations keypad_fops = {
    .owner      = THIS_MODULE,
    .open       = adckpd_config_open,
    .ioctl      = NULL,
    .release    = adckpd_config_release,
};

static int register_keypad_dev(struct kp  *kp)
{
    int ret=0;
    strcpy(kp->config_name,"am_adc_kpd");
    ret=register_chrdev(0, kp->config_name, &keypad_fops);
    if(ret<=0)
    {
        printk("register char device error\r\n");
        return  ret ;
    }
    kp->config_major=ret;
    printk("adc keypad major:%d\r\n",ret);
    kp->config_class=class_create(THIS_MODULE,kp->config_name);
    kp->config_dev=device_create(kp->config_class,	NULL,
    		MKDEV(kp->config_major,0),NULL,kp->config_name);
    return ret;
}

static int __init kp_probe(struct platform_device *pdev)
{
    struct kp *kp;
    struct input_dev *input_dev;
    int i, j, ret;
    struct adc_kp_platform_data *pdata = pdev->dev.platform_data;
    const struct key_list *list = pdata->list;
    int new_chan_flag;

    if (!pdata) {
        dev_err(&pdev->dev, "platform data is required!\n");
        return -EINVAL;
    }
   
    kp = kzalloc(sizeof(struct kp), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!kp || !input_dev) {
        kfree(kp);
        input_free_device(input_dev);
        return -ENOMEM;
    }
    gp_kp=kp;

	kp->queue = create_singlethread_workqueue("adc_keypad");
	INIT_DELAYED_WORK(&kp->work, kp_work);


    platform_set_drvdata(pdev, kp);
    kp->pdev = pdev;
    kp->input = input_dev;
    kp->list = list;    

    /* setup input device */
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_REP, input_dev->evbit);

    //adc key
    kp->chan_num = 0;
    for (i=0; i<list->key_num; i++) {
        if(list->key[i].code > 0)
            set_bit(list->key[i].code, input_dev->keybit);      
        /* search the key chan */
        new_chan_flag = 1;
        for (j=0; j<kp->chan_num; j++) {
            if (list->key[i].chan == kp->chan[j]) {
                new_chan_flag = 0;
                break;
            }
        }
        if (new_chan_flag) {
            kp->chan[kp->chan_num] = list->key[i].chan;
            printk(KERN_INFO "chan #%d used for ADC key\n", list->key[i].chan);
            kp->chan_num++;
        }    
        printk(KERN_INFO "%s key(%d) registed.\n", list->key[i].name, list->key[i].code);
    }
#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)        
    for (i=0; i<list->gpio_key_num; i++) {
        set_gpio_mode(list->gpio_key[i].bank, list->gpio_key[i].bit, GPIO_INPUT_MODE);
        if(list->gpio_key[i].code > 0)
            set_bit(list->gpio_key[i].code, input_dev->keybit);
        printk(KERN_INFO "%s key(%d) registed.\n", list->gpio_key[i].name, list->gpio_key[i].code);
    }
#endif

#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)
    for (i=0; i<list->fn_key_num; i++) {
        if(list->fn_init)
            (*list->fn_init)();
        if(list->fn_key[i].fn_code > 0)    
            set_bit(list->fn_key[i].fn_code, input_dev->keybit);
        printk(KERN_INFO "%s key(%d) registed.\n", list->fn_key[i].name, list->fn_key[i].fn_code);
    }        
#endif 

    
    input_dev->name = "adc_keypad";
    input_dev->phys = "adc_keypad/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_ISA;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;

    input_dev->rep[REP_DELAY]=0xffffffff;
    input_dev->rep[REP_PERIOD]=0xffffffff;

    input_dev->keycodesize = sizeof(unsigned short);
    input_dev->keycodemax = 0x1ff;

    ret = input_register_device(kp->input);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register keypad input device.\n");
		    kfree(kp);
		    input_free_device(input_dev);
		    return -EINVAL;
    }
    printk("adc keypad register input device completed.\r\n");

    register_keypad_dev(kp);

#ifdef CONFIG_HAS_EARLYSUSPEND
    kp->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
    kp->early_suspend.suspend = kp_early_suspend;
    kp->early_suspend.resume = kp_late_resume;
    register_early_suspend(&kp->early_suspend);
#endif

    queue_delayed_work(kp->queue, &kp->work, HZ*2);
    
    return 0;
}

static int kp_remove(struct platform_device *pdev)
{
    struct kp *kp = platform_get_drvdata(pdev);

    input_unregister_device(kp->input);
    input_free_device(kp->input);
    unregister_chrdev(kp->config_major,kp->config_name);
    if(kp->config_class)
    {
        if(kp->config_dev)
        device_destroy(kp->config_class,MKDEV(kp->config_major,0));
        class_destroy(kp->config_class);
    }
    kfree(kp);
    gp_kp=NULL ;
    return 0;
}

#if defined(CONFIG_PM)
static int kp_suspend(struct platform_device *dev, pm_message_t state)
{
    struct kp *kp_data = platform_get_drvdata(dev);

    //printk("kp_suspend\n");

    kp_data->suspend = 1;
    
    return 0;
}

static int kp_resume(struct platform_device *dev)
{
    struct kp *kp_data = platform_get_drvdata(dev);

    //printk("kp_resume\n");
    
    kp_data->suspend = 0;

    queue_delayed_work(kp_data->queue, &kp_data->work, HZ/8);

    return 0;
}
#else
#define kp_suspend NULL
#define kp_resume  NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kp_early_suspend(struct early_suspend *h) {
	struct kp *kp_data = container_of(h, struct kp, early_suspend);

	printk(KERN_INFO "kp_early_suspend\n");

    kp_suspend(kp_data->pdev,PMSG_SUSPEND);
}

static void kp_late_resume(struct early_suspend *h) {
	struct kp *kp_data = container_of(h, struct kp, early_suspend);
	
	printk(KERN_INFO "kp_late_resume\n");
        
    kp_resume(kp_data->pdev);
}
#endif


static struct platform_driver kp_driver = {
    .probe      = kp_probe,
    .remove     = kp_remove,
//#if defined(CONFIG_PM)
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = kp_suspend,
    .resume = kp_resume,
#endif
    .driver     = {
        .name   = "m1-adckp",
    },
};

static int __devinit kp_init(void)
{
    printk(KERN_INFO "ADC Keypad Driver init.\n");
    return platform_driver_register(&kp_driver);
}

static void __exit kp_exit(void)
{
    printk(KERN_INFO "ADC Keypad Driver exit.\n");
    platform_driver_unregister(&kp_driver);
}

module_init(kp_init);
module_exit(kp_exit);

MODULE_AUTHOR("Robin Zhu");
MODULE_DESCRIPTION("ADC Keypad Driver");
MODULE_LICENSE("GPL");




