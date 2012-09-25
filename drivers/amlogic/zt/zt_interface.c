#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/major.h>
#include <asm/uaccess.h>

#include <mach/pinmux.h>
#include <mach/gpio.h>

#include <mach/am_regs.h>

#define ZTKEY_LEN CONFIG_FB_OSD1_ZTKEY_LEN

#define POWER_CODE  116
#define HOME_CODE   102
#define MENU_CODE   139
#define BACK_CODE   158
#define VOL0_CODE   115  //up
#define VOL1_CODE   114  // down

//extern void zt_key_android(int code);
extern struct zt_ket_str zt_key;
extern struct completion zt_key_comp;

extern void add_zt_key(int x, int y, int p, int state);


static struct class *zt_interface_class;
int m_ztRotate = 1;

static int zt_interface_open(struct inode *inode, struct file *file)
{
	return 0;	
}
static int zt_interface_release(struct inode *inode, struct file *file)
{

	return 0;	
}

extern void zt_shutdown(void);

extern void gps_device_reset(void);
extern void gps_device_on(int on);

static int zt_interface_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	switch(cmd)
	{
	    //motor
		case 0 :
			//g_motor	= arg;
			break;
		
		// amlogic shutdown	
		case 1 :
			zt_shutdown();
			break;
			
		// wifi power on
		case 10:		    
			break;
			
		// wifi power down
		case 11:

			break;
		//IOCTL_GPS_HW_INIT
		case 12:
#if defined(CONFIG_UART_GPS)
            gps_device_reset();
#endif
			break;
		//IOCTL_GPS_HW_CONTROL
		case 13:
#if defined(CONFIG_UART_GPS)
            gps_device_on(!!arg);
#endif
            break;
		case 15:
			break;
		// factory reset
		case 16:
			break;
        // image update
		case 17:
			break;		
        // image update
		case 18:
			break;
        case 22:
            //gpio_set_value(IMAPX200_GPE(0),!!arg);
            break;
		default:
			//g_motor = 0;
			break;
	}
	return 0;
	
}

ssize_t zt_interface_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	printk("zt_interface_read\r\n");
	return 0;
}
ssize_t zt_interface_write(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	char enable;
	//printk("zt_interface_write : %d\r\n",((unsigned char*)buf)[0]);
	enable = ((unsigned char*)buf)[0];

	if( 101 == enable ) {
		zt_shutdown();
	} 
	// hide ztkey
	if( 10 == enable ) {
		add_zt_key(10000,10000,0,1);
		complete(&zt_key_comp);
	} 

	// show ztkey
	if( 11 == enable ) {
		add_zt_key(0,0,0,1);
		complete(&zt_key_comp);
	}
    /*
	if( 21 == enable ) {
		zt_key_android(HOME_CODE);
	} else if( 22 == enable ){
		zt_key_android(MENU_CODE);
	} else if( 23 == enable ){
		zt_key_android(BACK_CODE);
	} else if( 24 == enable ){
		zt_key_android(VOL0_CODE);
	} else if( 25 == enable ){
		zt_key_android(VOL1_CODE);
	}
	*/
	// g-sesnor
	if( enable >= 0 && enable <= 3 ) {
		add_zt_key(0,0,0,enable);
		m_ztRotate = enable;
		complete(&zt_key_comp);
	} 

	// ztkey material
	if( enable >= 30 ) {
		add_zt_key(10002,10002,enable - 30,1);
		complete(&zt_key_comp);
	} 	
	
	return 0;
}


static const struct file_operations zt_interface_fops = 
{
	.owner		= THIS_MODULE,
	.open		= zt_interface_open,
	.release	= zt_interface_release,
	.ioctl		= zt_interface_ioctl,
	.read       = zt_interface_read,
	.write      = zt_interface_write,
};

static ssize_t store_ztkey_len(struct device *device, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/*
	struct fb_info *fb_info = dev_get_drvdata(device);
	struct myfb_dev *fbdev = (struct myfb_dev *)fb_info->par;
	int err;
	fbdev->scale = simple_strtoul(buf, NULL, 0);
	if ((err = osd_ioctl(fb_info,FBIOPUT_OSD_2X_SCALE,fbdev->scale)))
		return err;
	*/
	return 0;
}

static ssize_t show_ztkey_len(struct device *device, struct device_attribute *attr,
			char *buf)
{
	//struct fb_info *fb_info = dev_get_drvdata(device);
	//struct myfb_dev *fbdev = (struct myfb_dev *)fb_info->par;
	return snprintf(buf, PAGE_SIZE, "%d\n",ZTKEY_LEN);
}

static struct device_attribute ztkey_attrs[] = {
	__ATTR(ztkey_len, S_IRUGO|S_IWUSR, show_ztkey_len, store_ztkey_len),
};

static int __init zt_interface_init(void)
{

	int ret,i;
	struct device *dev;
	
	printk("zt_interface_init\r\n");
	/* create motor dev */
	ret = register_chrdev(ZT_INTERFACE_MAJOR, "zt_interface", &zt_interface_fops);
	if ( ret < 0)
	{
		printk("zt_interface_init register_chrdev error = 0x%x\n",ret);
		return -EFAULT;
	}

	zt_interface_class = class_create(THIS_MODULE, "zt_interface");
	dev = device_create(zt_interface_class, NULL, MKDEV(ZT_INTERFACE_MAJOR, 0), NULL, "zt_interface");	
	for(i=0;i<ARRAY_SIZE(ztkey_attrs);i++)
		device_create_file(dev, &ztkey_attrs[i]);

	return 0;

}

static void __exit zt_interface_exit(void)
{
	unregister_chrdev(ZT_INTERFACE_MAJOR, "zt_interface");
}

module_init(zt_interface_init);
module_exit(zt_interface_exit);

MODULE_AUTHOR("yqcui <yqcui@zenithink.com>");
MODULE_DESCRIPTION("zenithink interface");
MODULE_LICENSE("GPL");
