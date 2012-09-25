/*
 * drivers/input/touchscreen/tsc2007.c
 *
 * Copyright (c) 2008 MtekVision Co., Ltd.
 *	Kwangwoo Lee <kwlee@mtekvision.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 *	Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <linux/major.h>

#include <asm/uaccess.h>

typedef struct str_rt_encrypt {
	char			phys[32];
	struct i2c_client	*client;
} rt_encrypt_t;
rt_encrypt_t rt_encrypt;


int  rt_encrypt_exec(u8 value);
// encrypt usb and sd
int rt_jia_usb = 0;
int rt_jia_sd  = 0;
#define ENCRY_MAJOR 237
#define RT_ENCRYPT_RETRY_COUNT 10

static int rt_encrypt_i2c_tx_data(char *buf, int len)
{
	uint8_t i;
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= rt_encrypt.client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < RT_ENCRYPT_RETRY_COUNT; i++) {
		if ( (ret = i2c_transfer(rt_encrypt.client->adapter, msg, 1)) == 1) {
			break;
		}

		printk("rt_encrypt_i2c_tx_data : ret = 0x%x\r\n",ret);

		mdelay(10);
	}

	if (i >= RT_ENCRYPT_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, RT_ENCRYPT_RETRY_COUNT);
		return -EIO;
	}

	return ret;
}
static int rt_encrypt_i2c_rx_data(char *buf, int len)
{

	uint8_t i;
	int ret = 0;
	struct i2c_msg msgs[] = {
		/*{
			.addr	= rt_encrypt.client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},*/
		{
			.addr	= rt_encrypt.client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < RT_ENCRYPT_RETRY_COUNT; i++) {
		if ( (ret = i2c_transfer(rt_encrypt.client->adapter, msgs, 1)) == 1) {
			break;
		}
		printk("rt_encrypt_i2c_rx_data : ret = 0x%x\r\n",ret);
		mdelay(10);
	}

	if (i >= RT_ENCRYPT_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, RT_ENCRYPT_RETRY_COUNT);
		return -EIO;
	}

	return ret;
}


static inline int rt_encrypt_write(u8 reg, u8 value)
{
	u8 data[2];
	int ret;

	data[0] = reg;
	data[1] = value;

	//ret = i2c_master_send(rt_encrypt.client, data, 2);
	ret = rt_encrypt_i2c_tx_data(data,2);

	return ret;
}

static inline u8 rt_encrypt_xfer(u8 reg, u8* value)
{
	int ret;

	//ret = i2c_master_send(rt_encrypt.client, &reg, 1);
	ret = rt_encrypt_i2c_tx_data(&reg,1);

	if (ret < 0)
		return ret;
	//ret = i2c_master_recv(rt_encrypt.client, value, 1);
	ret = rt_encrypt_i2c_rx_data(value,1);
	if (ret < 0)
		return ret;
	
	rt_encrypt_exec(*value);
	return ret;
}

static inline u8 rt_encrypt_read(u8 reg, u8* value)
{
	int ret;

	//ret = i2c_master_send(rt_encrypt.client, &reg, 1);
	ret = rt_encrypt_i2c_tx_data(&reg,1);

	if (ret < 0)
		return ret;
	//ret = i2c_master_recv(rt_encrypt.client, value, 1);
	ret = rt_encrypt_i2c_rx_data(value,1);
	if (ret < 0)
		return ret;

	
	return ret;
}

static int rt_encrypt_get(u8* src, u8* dec)
{
	u8 reg = 0x90;
	int retry,i;
	int ret;

	for( retry = 0; retry < 3; retry++ ){
		// encode data
		for( i = 0; i < 8; i++ ){
			ret = rt_encrypt_write( reg + i, src[i]);
			if(ret < 0) {
				//printk("rt_encrypt_get : r2c encode rror!\r\n");
				break;
			}
		}

		// retry
		if( i != 8 )
			continue;

		// decode data
		for( i = 0; i < 8; i++ ){
			ret = rt_encrypt_xfer( reg + i, dec + i );
			if(ret < 0) {
				//printk("rt_encrypt_get : r2c dncode rror!\r\n");
				break;
			}
		}
		if( i == 8 )
			return 0;
	}
	return 1;
}

int  rt_encrypt_exec(u8 value)
{
    //null is {0xcf,0x88,0x3,0x41,0xb,0xaa,0x21,0xe2}

	//printk("rt_encrypt_exec : value = 0x%x\r\n",value);
	switch(value){
	    /*
		case 0x2b :
		case 0xab :
		case 0x57 :
		case 0xc3 :
			rt_jia_usb = 99;
			break;
		case 0xad :
		case 0x2f :
		case 0x61 :
		case 0x34 :
			rt_jia_sd = 99;
			break;*/
        case 0xc3 :
        case 0xf6 :
        case 0xaa :
        case 0xac :
            rt_jia_usb = 99;
            break;
        case 0x90 :
        case 0xe9 :
        case 0xd  :
        case 0x57 :
            rt_jia_sd = 99;
            break;
		
		// fake
		case 0x79:
			rt_jia_usb = 1;
			rt_jia_sd = 1;
			break;
		case 0x78:
			rt_jia_usb = 1;
			rt_jia_sd = 1;
			break;
		case 0x77:
		case 0x76:
		case 0x75:
		case 0x74:
			rt_jia_usb = 1;
			rt_jia_sd = 1;
			break;									
		default :
			rt_jia_usb = 1;
			rt_jia_sd = 1;
			break;
	}
        return 0;
}

/*********************************/
static int encry_open(struct inode *inode, struct file *file)
{
	printk("encry_open\r\n");
	return 0;	
}
static int encry_release(struct inode *inode, struct file *file)
{
	printk("encry_release\r\n");
	return 0;	
}
static int encry_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int i=0,ret=0;
    unsigned char factory[128];
    unsigned char data[8];
    
	//printk("encry_ioctl , cmd = %d\r\n",cmd);
	switch(cmd)
	{
		case 0 :
			//if (copy_from_user(&param, arg, sizeof(int))){
			//	printk("encry_ioctl , return\r\n");
			//	return -EFAULT;	
			//}
                         memset(factory,0xff,128);

			for( i = 0; i < 128; i++ ){
				ret = rt_encrypt_read( i, &factory[i] );
                                //printk("read %d:%x\r\n",i, factory[i]);
				if(ret < 0) {
					printk("rt_encrypt_get : r2c dncode rror!\r\n");
                                        ret = -EIO;
					break;
				}
			}

			if(copy_to_user((unsigned char*)arg, factory, 128))
			{
				ret = -EFAULT;
			}
			
			//printk("encry_ioctl 0\r\n");
			break;

		case 5 :

            memset(data,0xff,8);

			for( i = 0x90; i < 8; i++ ){
				ret = rt_encrypt_read( i, &data[i] );
				if(ret < 0) {
					printk("rt_encrypt_get : r2c dncode rror!\r\n");
                                        ret = -EIO;
					break;
				}
			}

			if(copy_to_user((unsigned char*)arg, data, 8))
			{
				ret = -EFAULT;
			}
			
			break;			
			
		// wifi power on
		case 10:
                        if(copy_from_user(factory,(unsigned char*) arg,128))
			{
				ret = -EFAULT;
			}
                        for( i=0;i<128;i++)
                        {
                         //       printk("write %d:%x\r\n",i, factory[i]);
                           ret =  rt_encrypt_write(i, factory[i]);
        	           if(ret < 0) 
                           {
				printk("rt_encrypt_write : r2c dncode rror!\r\n");
                                ret = -EIO;
				break;
			   }
                           mdelay(5);
                        }
			//printk("encry_ioctl 10\r\n");
			break;
			
		// wifi power down
		case 11:
			break;
		default :
			break;
	}
	return 0;	
}
static struct file_operations encry_fops = 
{
	.owner		= THIS_MODULE,
	.open		= encry_open,
	.release	= encry_release,
	.ioctl		= encry_ioctl,
};

static int app_state_written = 0;

static ssize_t show_encrypt_data(struct class *class, 
			struct class_attribute *attr,	char *buf)
{
    u8 reg = 0x90;
    u8 data[8];
    int i;
    int ret;

    if(!app_state_written)
        return 0;

    for( i = 0; i < 8; i++ ){
    	ret = rt_encrypt_i2c_tx_data(&reg,1);

    	if (ret < 0)
    		return ret;

    	ret = rt_encrypt_i2c_rx_data(data+i,1);
    	if (ret < 0)
    		return ret;
        reg++;
    }

    return sprintf(buf,"%x %x %x %x %x %x %x %x",
        data[0],data[1],data[2],data[3],
        data[4],data[5],data[6],data[7]);
}

static ssize_t write_encrypt_data(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{
    u8 reg = 0x90;
    int data[8];

    int i;
    int ret;

    if(count < 24){
        printk("data short len = %d \n",count);
        return -1;
    }

    sscanf(buf,"%x %x %x %x %x %x %x %x",
        data,data+1,data+2,data+3,
        data+4,data+5,data+6,data+7);
    
	for( i = 0; i < 8; i++ ){
		ret = rt_encrypt_write( reg + i, (u8)data[i]);
		if(ret < 0) {
			printk("rt_encrypt_get : r2c encode rror!\r\n");
			return ret;
		}
	}

	if(count >=24)
	    app_state_written = 1;
	return count;
}

char zygote_value = 0;
static ssize_t show_zygote_data(struct class *class, 
			struct class_attribute *attr,	char *buf)
{
	int ret = 0;
	
	ret = sprintf(buf,"%c",zygote_value);

    return ret;
}

static ssize_t write_zygote_data(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{

    //sscanf(buf,"%d", &zygote_value);
    zygote_value = buf[0];
	return 1;
}

// update OEM
static ssize_t show_update_data(struct class *class, 
			struct class_attribute *attr,	char *buf)
{
	int ret = 0;

	if(!strcmp( CONFIG_BOARD_NAME, "H1_2n" )) {	
		ret = sprintf(buf,"%s","H1_2n");
	} else if(!strcmp( CONFIG_BOARD_NAME, "G0_2n" )) {
		ret = sprintf(buf,"%s","G0_2n");
	}

    return ret;
}

static ssize_t write_update_data(struct class *class, 
			struct class_attribute *attr,	const char *buf, size_t count)
{

	return 1;
}

static struct class_attribute encrypt_class_attrs[] = {
    __ATTR(pri,   S_IRUGO | S_IWUSR, show_encrypt_data, write_encrypt_data),
    __ATTR(value, S_IRUGO | S_IWUSR, show_zygote_data,  write_zygote_data),
    __ATTR(update, S_IRUGO | S_IWUSR, show_update_data,  write_update_data),
    __ATTR_NULL
};

static struct class private_class = {
    .name = "private_app",
    .class_attrs = encrypt_class_attrs,
};

static int rt_encrypt_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	u8 src[8] = {0xfe,0x95,0x25,0x29,0x05,0x8a,0x00,0xa7};
	u8 dec[8];
	int res = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		return res;
	}

	printk("rt_encrypt_probe\r\n");

	rt_encrypt.client = client;
	i2c_set_clientdata(client, &rt_encrypt);
	
	rt_encrypt_get(src,dec);

	if(class_register(&private_class)){
		printk(" class register private_class fail!\n");
		return -ENODEV;
	}
	
	printk("rt_encrypt_probe is OK!!!\r\n");
	//printk("dec : %x %x %x %x %x %x %x %x\r\n",dec[0],dec[1],dec[2],dec[3],dec[4],dec[5],dec[6],dec[7]);
	return 0;
}

static int rt_encrypt_remove(struct i2c_client *client)
{
	struct rt_encrypt	*ts = i2c_get_clientdata(client);

	kfree(ts);

	return 0;
}

static struct i2c_device_id rt_encrypt_idtable[] = {
	{ "rt_encrypt", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsc2007_idtable);

static struct i2c_driver rt_encrypt_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rt_encrypt"
	},
	.id_table	= rt_encrypt_idtable,
	.probe		= rt_encrypt_probe,
	.remove		= rt_encrypt_remove,
};

static struct class *encry_class;

static int __init rt_encrypt_init(void)
{

	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	int ret;
	/*
	if (register_chrdev(ENCRY_MAJOR, "encry", &encry_fops) < 0)
	{
		printk("Register char device for encry error\n");
		return -EFAULT;
	}

	encry_class = class_create(THIS_MODULE, "encry");
	device_create(encry_class, NULL, MKDEV(ENCRY_MAJOR, 0), NULL, "encry");	
	*/
	printk("rt_encrypt_init\r\n");
	ret = i2c_add_driver(&rt_encrypt_driver);
	if(ret < 0){
		printk("rt e...... failed\r\n");
		goto err_driver;
	}

	printk("rt_encrypt_init success\r\n");
	return 0;

err_driver:
	i2c_del_driver(&rt_encrypt_driver);
	return -ENODEV;

}

static void __exit rt_encrypt_exit(void)
{
	i2c_del_driver(&rt_encrypt_driver);
}

subsys_initcall(rt_encrypt_init);
module_exit(rt_encrypt_exit);

MODULE_AUTHOR("michael cui");
MODULE_DESCRIPTION("zt e Driver");
MODULE_LICENSE("GPL");
