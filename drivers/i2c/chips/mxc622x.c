/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * Initial Code:
 *	Robbie Cao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */


//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <asm/uaccess.h>

#include "mxc622x.h"

#define DEBUG							0

#define MAX_FAILURE_COUNT				3

#define MXC622X_DELAY_PWRON				300							/* ms, >= 300 ms */
#define MXC622X_DELAY_PWRDN				1							/* ms */
#define MXC622X_DELAY_SETDETECTION		MXC622X_DELAY_PWRON

#define MXC622X_RETRY_COUNT				3

static struct i2c_client *this_client;
static int sensor_sts=SENSOR_UNKNOW;  //when power on, reset z value to 0,up level will reset cache 

extern int get_gsensor_direction(void);

static int mxc622x_i2c_tx_data(char *buf, int len)
{
	uint8_t i;

//	printk("mxc622x i2c send data\n");


	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < MXC622X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0) {
			break;
		}

		mdelay(10);
	}

	if (i >= MXC622X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MXC622X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}
static int mxc622x_i2c_rx_data(char *buf, int len)
{

	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MXC622X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}

		mdelay(10);
	}

	if (i >= MXC622X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MXC622X_RETRY_COUNT);
		return -EIO;
	}

	return len;
}

static int reg_write_1(u8 reg,u8 data)
{
	int ret =0;
    char buf[2];

	buf[0]=reg;
	buf[1]=data;
	//ret = i2c_master_send(this_client, buf, 2);
	mxc622x_i2c_tx_data(buf,2);
	if (ret < 0){
		printk("%s reg 0x%x data 0x%x failed\n",__func__,reg,data);
		goto error;
}

error:	
	return ret;
}


static int reg_read_n(u8 reg,u8 * data,int n)
{
    int ret;
    
    /* Set address */
	ret = mxc622x_i2c_tx_data(&reg, 1);
    //ret = i2c_master_send(this_client, &reg, 1);
    if (ret  < 0){
        printk("%s reg 0x%x failed #1 ret %d\n",__func__,reg ,ret);
        ret = -1;
        goto error;
    }

    //ret = i2c_master_recv(this_client, data, n);
    ret = mxc622x_i2c_rx_data(data, n);
	if (ret != n){
        printk("%s reg 0x%x failed #2 ret %d\n",__func__,reg ,ret);
        ret = -1;
        goto error;
    }

    //printk("%s reg 0x%x : 0x%x %d\n",__func__,reg,*data,ret);

error:
    return ret;
}

static int mxc622x_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int mxc622x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int mxc622x_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	void __user *pa = (void __user *)arg;
	unsigned char data[16] = {0};
	int vec[3] = {0};

	switch (cmd) 
	{
		case MXC622X_IOC_PWRON:
			//printk("mxc622X power on\n");
			
            if(sensor_sts == SENSOR_UNKNOW){
                //put init code here
			}
            sensor_sts = SENSOR_OPENED;
			data[0] = MXC622X_REG_CTRL;
			data[1] = MXC622X_CTRL_PWRON;
			if (mxc622x_i2c_tx_data(data, 2) < 0) {
				return -EFAULT;
			}
/*	wait PWRON done */
			msleep(MXC622X_DELAY_PWRON);
			break;

		case MXC622X_IOC_PWRDN:

			//printk("mxc622X power down\n");

            sensor_sts = SENSOR_CLOSED;

			data[0] = MXC622X_REG_CTRL;
			data[1] = MXC622X_CTRL_PWRDN;
			if (mxc622x_i2c_tx_data(data, 2) < 0) {
				return -EFAULT;
			}
/* wait PWRDN done */
			msleep(MXC622X_DELAY_PWRDN);
			break;

		case MXC622X_IOC_READXYZ:

			if (reg_read_n(MXC622X_REG_DATA,data, 2) < 0){
				return -EFAULT;
			}

			vec[0] =(int)GET_BITS(data[MXC_XOUT],MXC_xOUT_DATA);  //abs value
			vec[1] =(int)GET_BITS(data[MXC_YOUT],MXC_xOUT_DATA);

            if(GET_BITS(data[MXC_XOUT],MXC_xOUT_DIRECT)){   //pola
                vec[0]-=ADC_MAX;
            }
            if(GET_BITS(data[MXC_YOUT],MXC_xOUT_DIRECT)){
                vec[1]-=ADC_MAX;
            }

			vec[2] = vec[0]*vec[0] + vec[1]*vec[1];
			if(vec[2] < 8100)   //make z more than 1/3 G (64 count/g)
                vec[2] = 8100;

            //64 count/g
            if(sensor_sts == SENSOR_OPENED){
                vec[2]=0;
                sensor_sts = SENSOR_WORKING;
            }/*else{
                vec[2] = 1<<3;  //no z info,emulate it
            }*/
#if DEBUG
            printk("[X: %08x] [Y: %08x] [Z: %08x],[X: %d] [Y: %d] [Z: %d]\n", 
                vec[0], vec[1], vec[2],vec[0], vec[1], vec[2]);
#endif
			if (copy_to_user(pa, vec, sizeof(vec))) {
				return -EFAULT;
			}
			break;

		case MXC622X_IOC_READSTATUS:
			printk("mxc622X read status\n");
			//data[0] = MXC622X_REG_DATA;

			if (reg_read_n(MXC622X_REG_DATA, data, 3) < 0){
				return -EFAULT;
			}

			vec[0] = (int)data[0];
			vec[1] = (int)data[1];
			vec[2] = (int)data[2];
#if DEBUG
			printk("[X - %04x] [Y - %04x] [STATUS - %04x]\n", 
				vec[0], vec[1], vec[2]);
#endif
			if (copy_to_user(pa, vec, sizeof(vec))) {
				return -EFAULT;
			}
			break;

		case MXC622X_IOC_SETDETECTION:
			printk("mxc622X set detection\n");

			data[0] = MXC622X_REG_CTRL;

			if (copy_from_user(&(data[1]), pa, sizeof(unsigned char))) {
				return -EFAULT;
			}

			if (mxc622x_i2c_tx_data(data, 2) < 0) {
				return -EFAULT;
			}
/* wait SETDETECTION done */
			msleep(MXC622X_DELAY_SETDETECTION);
			break;
        case MXC622X_IOC_GET_DIR:
            //printk("mxc622x get dir\n");
            data[0] = get_gsensor_direction();
			if (copy_to_user(pa, data, 1)) 
			{
				return -EFAULT;
			}
            break;

		default:
			break;
	}

	return 0;
}

static ssize_t mxc622x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "MXC622X");
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(mxc622x, S_IRUGO, mxc622x_show, NULL);

static struct file_operations mxc622x_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc622x_open,
	.release	= mxc622x_release,
	.ioctl		= mxc622x_ioctl,
};

static struct miscdevice mxc622x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mxc622x",
	.fops = &mxc622x_fops,
};

int mxc622x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;

	printk("mxc622x_probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: functionality check failed\n", __FUNCTION__);
		res = -ENODEV;
		goto out;
	}
	
	this_client = client;

	res = misc_register(&mxc622x_device);
	if (res) {
		pr_err("%s: mxc622x_device register failed\n", __FUNCTION__);
		goto out;
	}

	res = device_create_file(&client->dev, &dev_attr_mxc622x);
	if (res) {
		pr_err("%s: device_create_file failed\n", __FUNCTION__);
		goto out_deregister;
	}

	printk("mxc622x_probe start ok\n");
	return 0;

out_deregister:
	misc_deregister(&mxc622x_device);
out:
	return res;
}

static int mxc622x_remove(struct i2c_client *client)
{
	device_remove_file(&client->dev, &dev_attr_mxc622x);
	misc_deregister(&mxc622x_device);

	return 0;
}

int mxc622x_suspend(struct device *dev)
{    
    return 0;
}
int mxc622x_resume(struct device *dev)
{
    sensor_sts=SENSOR_UNKNOW;
    
    return 0;
}

static struct dev_pm_ops mxc622x_i2c_pm_ops = {
	.suspend	= mxc622x_suspend,
	.resume	= mxc622x_resume,
};


static const struct i2c_device_id mxc622x_id[] = {
	{ MXC622X_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver mxc622x_driver = {
	.probe 		= mxc622x_probe,
	.remove 	= mxc622x_remove,
	.id_table	= mxc622x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name = MXC622X_I2C_NAME,
		.pm = &mxc622x_i2c_pm_ops,
	},
};

static int __init mxc622x_init(void)
{
	pr_info("mxc622x driver: init\n");
	return i2c_add_driver(&mxc622x_driver);
}

static void __exit mxc622x_exit(void)
{
	pr_info("mxc622x driver: exit\n");
	i2c_del_driver(&mxc622x_driver);
}

module_init(mxc622x_init);
module_exit(mxc622x_exit);

MODULE_AUTHOR("Robbie Cao<hjcao@memsic.com>");
MODULE_DESCRIPTION("MEMSIC MXC622X (DTOS) Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");


