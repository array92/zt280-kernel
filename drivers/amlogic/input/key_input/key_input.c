/*
 * linux/drivers/input/key_input/key_input.c
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
 * author :   Elvis Yu
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
#include <linux/sched.h>

#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

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

#include <mach/am_regs.h>
#include <mach/pinmux.h>
#include <linux/major.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/thread_info.h>
#include <linux/kthread.h>
#include <linux/input/key_input.h>


//#define USE_RTC_INTR
//#define AML_KEYINPUT_DBG
#define AML_KEYINPUT_INTR     0
#define AML_KEYINPUT_POLLING   2

//#ifdef USE_RTC_INTR
static void keyinput_tasklet(unsigned long data);
DECLARE_TASKLET_DISABLED(ki_tasklet, keyinput_tasklet, 0);
//#endif

struct key_input {
    struct input_dev *input;
    struct timer_list timer;
    int* key_state_list_0;
    int* key_state_list_1;
    int* key_hold_time_list;
    int major;
    char name[20];
    struct class *class;
    struct device *dev;
    struct key_input_platform_data *pdata;
    unsigned status;
    unsigned pending;
    unsigned suspend;
    struct task_struct *tsk;
    struct delayed_work work;
};

static struct key_input *KeyInput = NULL;

//#ifndef USE_RTC_INTR
void key_input_polling(unsigned long data)
{
    int i;
    struct key_input *ki_data=(struct key_input*)data;

    if(ki_data->pdata->scan_func(ki_data->key_state_list_0) >= 0)
    {
        for(i = 0; i < ki_data->pdata->key_num; i++)
        {
            if(ki_data->key_state_list_0[i])
            {
                if((ki_data->key_hold_time_list[i] += ki_data->pdata->scan_period) > ki_data->pdata->fuzz_time)
                {
#ifdef AML_KEYINPUT_DBG                    
                    print_dbg("key %d pressed.\n", ki_data->pdata->key_code_list[i]);
#endif                    
                    input_report_key(ki_data->input, ki_data->pdata->key_code_list[i], 1);
                    ki_data->key_state_list_1[i] = 1;
                    ki_data->key_hold_time_list[i] = 0;
                }
            }
            else
            {
                if(ki_data->key_state_list_1[i])
                {
#ifdef AML_KEYINPUT_DBG                         
                    print_dbg("key %d released.\n", ki_data->pdata->key_code_list[i]);
#endif                    
                    input_report_key(ki_data->input, ki_data->pdata->key_code_list[i], 0);
                    ki_data->key_state_list_1[i] = 0;
                    ki_data->key_hold_time_list[i] = 0;
                }
            }
        }
    }
    mod_timer(&ki_data->timer,jiffies+msecs_to_jiffies(ki_data->pdata->fuzz_time));
}
//#endif

static int
key_input_open(struct inode *inode, struct file *file)
{
    file->private_data = KeyInput;
    return 0;
}

static int
key_input_release(struct inode *inode, struct file *file)
{
    file->private_data=NULL;
    return 0;
}

static const struct file_operations key_input_fops = {
    .owner      = THIS_MODULE,
    .open       = key_input_open,
    .ioctl      = NULL,
    .release    = key_input_release,
};

static int register_key_input_dev(struct key_input  *ki_data)
{
    int ret=0;
    strcpy(ki_data->name,"am_key_input");
    ret=register_chrdev(0, ki_data->name, &key_input_fops);
    if(ret<=0)
    {
        printk("register char device error\r\n");
        return  ret ;
    }
    ki_data->major=ret;
    printk("key_input major:%d\r\n",ret);
    ki_data->class=class_create(THIS_MODULE,ki_data->name);
    ki_data->dev=device_create(ki_data->class, NULL,
    		MKDEV(ki_data->major,0), NULL, ki_data->name);
    return ret;
}

extern void amlogic_reset();
extern void set_bat_off();
// delay 5s to halt
static int halt_amlogic_thread(void *data){
	unsigned long end_time = jiffies + msecs_to_jiffies(6000);
	int alive;
	unsigned int halt = 0,i;
	
	struct key_input *ki_data = (struct key_input *)data;
	
	do {
		if(kthread_should_stop()){
			goto off;	
		}
		
	    if(ki_data->pdata->scan_func(ki_data->key_state_list_0) >= 0)
	    {
	        for(i = 0; i < ki_data->pdata->key_num; i++)
	        {
	        	// power down
	            if(ki_data->key_state_list_0[i])
	            {
					//printk("halt_amlogic down\r\n");
					halt = 1;
	            }
	            else // power up
	            {
					printk("halt_amlogic up\r\n");
					halt = 0;
					goto off;
	            }
	        }
	    }
		
		schedule_timeout_interruptible(1);
		
	} while (time_before(jiffies, end_time));

off:
	
	if( 1 == halt ) {
		
		printk("halt_amlogic halt\r\n");
		set_bat_off();
	} else {
		printk("halt_amlogic ..........\r\n");
	}
}

static int halt_amlogic(struct key_input *ki_data,unsigned int time){
	unsigned long end_time = jiffies + msecs_to_jiffies(time);
	int alive;
	unsigned int halt = 0,i;
	
	do {
	    if(ki_data->pdata->scan_func(ki_data->key_state_list_0) >= 0)
	    {
	        for(i = 0; i < ki_data->pdata->key_num; i++)
	        {
	        	// power down
	            if(ki_data->key_state_list_0[i])
	            {
					//printk("halt_amlogic down\r\n");
					halt = 1;
	            }
	            else // power up
	            {
					//printk("halt_amlogic up\r\n");
					halt = 0;
					goto off;
	            }
	        }
	    }
		
		schedule_timeout_interruptible(1);
		
	} while (time_before(jiffies, end_time));

off:
	
	if( 1 == halt ) {
		
		printk("halt_amlogic halt\r\n");
		set_bat_off();
	} else {
		printk("halt_amlogic ..........\r\n");
	}
}

static void gpio_keys_report_event(struct work_struct *work)
{
	struct key_input *bdata =
		container_of(work, struct key_input, work);
	printk("gpio_keys_report_event\r\n");
	
    KeyInput->tsk = kthread_run(halt_amlogic_thread, KeyInput, "haltDown");
    if(IS_ERR(KeyInput->tsk)){
    	printk("halt_amlogic_thread is err\r\n");
    	KeyInput->tsk = NULL;	
    }
}


void zt_shutdown(){


	msleep(100);
	//input_sync(input);
	input_event(KeyInput->input, EV_KEY, 116, 0);
	input_event(KeyInput->input, EV_KEY, 116, 1);
	
	//input_event(KeyInput->input, EV_KEY, 116, 1);
	//input_event(KeyInput->input, EV_KEY, 116, 1);
	//input_event(KeyInput->input, EV_KEY, 116, 1);
	//input_event(KeyInput->input, EV_KEY, 116, 1);
	msleep(2000);
	input_event(KeyInput->input, EV_KEY, 116, 0);
	input_sync(KeyInput->input);
	msleep(100);	
}

//#ifdef USE_RTC_INTR
static void keyinput_tasklet(unsigned long data)
{
    if (KeyInput->status){
        input_report_key(KeyInput->input, KeyInput->pdata->key_code_list[0], 0);
        printk(KERN_INFO "=== key %d up ===\n", KeyInput->pdata->key_code_list[0]);
    }
    else{
		schedule_delayed_work(&KeyInput->work,HZ/10);
        input_report_key(KeyInput->input, KeyInput->pdata->key_code_list[0], 1);
        printk(KERN_INFO "=== key %d down ===\n", KeyInput->pdata->key_code_list[0]);

    }
}

static irqreturn_t am_key_interrupt(int irq, void *dev)
{
    KeyInput->status = (READ_CBUS_REG(RTC_ADDR1)>>2)&1;
    WRITE_CBUS_REG(RTC_ADDR1, (READ_CBUS_REG(RTC_ADDR1) | (0x0000c000)));
//    if (!KeyInput->suspend)
        tasklet_schedule(&ki_tasklet);
//    else
//        printk("key interrupt when suspend\n");
    return IRQ_HANDLED;
}
//#endif

static int __init key_input_probe(struct platform_device *pdev)
{
    struct key_input  *ki_data = NULL;
    struct input_dev *input_dev = NULL;
    int i,ret = 0;
    struct key_input_platform_data *pdata = pdev->dev.platform_data;

    if (!pdata) {
        dev_err(&pdev->dev, "platform data is required!\n");
        ret = -EINVAL;
        goto    CATCH_ERR;
    }
    
    ki_data = kzalloc(sizeof(struct key_input), GFP_KERNEL);
    input_dev = input_allocate_device();
    ki_data->key_state_list_0 = kzalloc((sizeof(int)*pdata->key_num), GFP_KERNEL);
    ki_data->key_state_list_1 = kzalloc((sizeof(int)*pdata->key_num), GFP_KERNEL);
    ki_data->key_hold_time_list = kzalloc((sizeof(int)*pdata->key_num), GFP_KERNEL);
    if (!ki_data || !input_dev || !ki_data->key_state_list_0 || !ki_data->key_state_list_1) {
        ret = -ENOMEM;
        goto    CATCH_ERR;
    }
    KeyInput = ki_data;

    platform_set_drvdata(pdev, ki_data);
    ki_data->input = input_dev;
    ki_data->pdata = pdata;

    if(ki_data->pdata->init_func != NULL)
    {
        if(ki_data->pdata->init_func() < 0)
        {
            printk(KERN_ERR "ki_data->pdata->init_func() failed.\n");
            ret = -EINVAL;
            goto    CATCH_ERR;
        }
    }
    if(ki_data->pdata->fuzz_time <= 0)
    {
        ki_data->pdata->fuzz_time = 100;
    }

//#ifndef USE_RTC_INTR
    if(ki_data->pdata->config == AML_KEYINPUT_POLLING){
        setup_timer(&ki_data->timer, key_input_polling, ki_data) ;
        mod_timer(&ki_data->timer, jiffies+msecs_to_jiffies(ki_data->pdata->fuzz_time));
    }
//#endif

    /* setup input device */
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_REP, input_dev->evbit);

    for(i = 0; i < pdata->key_num; i++)
    {
        set_bit(pdata->key_code_list[i], input_dev->keybit);
        printk(KERN_INFO "Key %d registed.\n", pdata->key_code_list[i]);
    }
    
    input_dev->name = "key_input";
    input_dev->phys = "key_input/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_ISA;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;

    input_dev->rep[REP_DELAY]=0xffffffff;
    input_dev->rep[REP_PERIOD]=0xffffffff;

    input_dev->keycodesize = sizeof(unsigned short);
    input_dev->keycodemax = 0x1ff;

    ret = input_register_device(ki_data->input);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register key input device.\n");
        ret = -EINVAL;
        goto    CATCH_ERR;
    }
	
	INIT_DELAYED_WORK(&ki_data->work, gpio_keys_report_event);
	
//#ifdef USE_RTC_INTR
    if(ki_data->pdata->config == AML_KEYINPUT_INTR){

        tasklet_enable(&ki_tasklet);
        ki_tasklet.data = (unsigned long)KeyInput;
        request_irq(INT_RTC, (irq_handler_t) am_key_interrupt, IRQF_SHARED, "power key", (void*)am_key_interrupt);
        WRITE_CBUS_REG(RTC_ADDR0, (READ_CBUS_REG(RTC_ADDR0) | (0x0000c000)));
    //    enable_irq(INT_RTC);
     }
//#endif

    printk("Key input register input device completed.\r\n");
    register_key_input_dev(KeyInput);
    return 0;

CATCH_ERR:
    if(input_dev)
    {
        input_free_device(input_dev);
    }
    if(ki_data->key_state_list_0)
    {
        kfree(ki_data->key_state_list_0);
    }
    if(ki_data->key_state_list_1)
    {
        kfree(ki_data->key_state_list_1);
    }
    if(ki_data->key_hold_time_list)
    {
        kfree(ki_data->key_hold_time_list);
    }
    if(ki_data)
    {
        kfree(ki_data);
    }
    return ret;
}

static int key_input_remove(struct platform_device *pdev)
{
    struct key_input *ki_data = platform_get_drvdata(pdev);

//#ifdef USE_RTC_INTR
    if(ki_data->pdata->config == AML_KEYINPUT_INTR){
        tasklet_disable(&ki_tasklet);
        tasklet_kill(&ki_tasklet);
        disable_irq(INT_RTC);
        free_irq(INT_RTC, am_key_interrupt);
    }
//#endif
    input_unregister_device(ki_data->input);
    input_free_device(ki_data->input);
    unregister_chrdev(ki_data->major,ki_data->name);
    if(ki_data->class)
    {
        if(ki_data->dev)
        device_destroy(ki_data->class,MKDEV(ki_data->major,0));
        class_destroy(ki_data->class);
    }

    kfree(ki_data->key_state_list_0);
    kfree(ki_data->key_state_list_1);
    kfree(ki_data->key_hold_time_list);
    kfree(ki_data);
    KeyInput = NULL ;
    return 0;
}

#ifdef CONFIG_PM
static int key_input_suspend(struct platform_device *dev, pm_message_t state)
{
    KeyInput->suspend = 1;
    return 0;
}

static int key_input_resume(struct platform_device *dev)
{
    KeyInput->suspend = 0;
    return 0;
}
#else
#define key_input_suspend NULL
#define key_input_resume  NULL
#endif

static struct platform_driver key_input_driver = {
    .probe      = key_input_probe,
    .remove     = key_input_remove,
    .suspend    = key_input_suspend,
    .resume     = key_input_resume,
    .driver     = {
        .name   = "m1-keyinput",
    },
};

static int __devinit key_input_init(void)
{
    printk(KERN_INFO "Key input Driver init.\n");
    return platform_driver_register(&key_input_driver);
}

static void __exit key_input_exit(void)
{
    printk(KERN_INFO "Key input Driver exit.\n");
    platform_driver_unregister(&key_input_driver);
}

module_init(key_input_init);
module_exit(key_input_exit);

MODULE_AUTHOR("Elvis Yu");
MODULE_DESCRIPTION("Key Input Driver");
MODULE_LICENSE("GPL");
