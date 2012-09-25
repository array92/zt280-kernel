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

static struct class *zt_key_class;
#define ZTKEY_LEN CONFIG_FB_OSD1_ZTKEY_LEN

struct zt_ket_str {
	int x;
	int y;
	int pressure;
	int state;
	//int sign; // ÊÇ·ñÎª¿Õ
} zt_ket_struct;

#define ZT_KEY_COUNT 10
struct zt_ket_str zt_key;
struct completion zt_key_comp;
int start_ztkey = 0;

extern int m_ztRotate;

void add_zt_key(int x, int y, int p, int state)
{
	zt_key.x = x;
	zt_key.y = y;
	zt_key.pressure = p;
	zt_key.state = state;
}

void zt_key_report(int x,int y)
{
    if(start_ztkey){
        add_zt_key(x,y,0,m_ztRotate);
        complete(&zt_key_comp);
    }
}

static int zt_key_open(struct inode *inode, struct file *file)
{
	init_completion(&zt_key_comp);
	printk("zt_key_open\r\n");
	return 0;
}
static int zt_key_release(struct inode *inode, struct file *file)
{
	printk("zt_key_release\r\n");
	return 0;	
}


static int zt_key_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ztkey_len = ZTKEY_LEN;
	void __user *argp = (void __user *)arg;
	printk("zt_key_ioctl , cmd = %d\r\n",cmd);
	
	switch(cmd)
	{
		case 1 :
			if (copy_to_user(argp, &ztkey_len, sizeof(ztkey_len)))
				return -EFAULT;
			break;

		default:
			break;
	}
	return 0;	
}

ssize_t zt_key_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	struct seq_file *m = (struct seq_file *)file->private_data;
	
	start_ztkey = 1; // set it , otherwise kernel dump 
	wait_for_completion_interruptible(&zt_key_comp);
	copy_to_user(buf,&zt_key, sizeof(struct zt_ket_str));
	
	return 0;
}

static struct file_operations zt_key_fops = 
{
	.owner		= THIS_MODULE,
	.open		= zt_key_open,
	.release	= zt_key_release,
	.ioctl		= zt_key_ioctl,
	.read       = zt_key_read,
	//.write      = zt_key_write,
};

static int __init zt_key_init(void)
{

	int ret;
	struct device *dev;
	
	printk("zt_key_init\r\n");
	
	/* create motor dev */
	// ZT_KEY_MAJOR
	ret = register_chrdev(ZT_KEY_MAJOR, "zt_key", &zt_key_fops);
	if ( ret < 0)
	{
		printk("zt_key_init register_chrdev error = 0x%x\n",ret);
		return -EFAULT;
	}

	zt_key_class = class_create(THIS_MODULE, "zt_key");
	dev = device_create(zt_key_class, NULL, MKDEV(ZT_KEY_MAJOR, 0), NULL, "zt_key");	
	

	return 0;

}

static void __exit zt_key_exit(void)
{
}

module_init(zt_key_init);
module_exit(zt_key_exit);

MODULE_AUTHOR("yqcui <yqcui@zenithink.com>");
MODULE_DESCRIPTION("zenithink key");
MODULE_LICENSE("GPL");
