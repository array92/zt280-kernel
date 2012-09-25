/*
 *
 * arch/arm/mach-meson/meson.c
 *
 *  Copyright (C) 2010 AMLOGIC, INC.
 *
 * License terms: GNU General Public License (GPL) version 2
 * Platform machine definition.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/spi/flash.h>
#include <mach/hardware.h>
#include <mach/platform.h>
#include <mach/memory.h>
#include <mach/clock.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/lm.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <mach/am_eth_pinmux.h>
#include <mach/nand.h>
#include <linux/i2c.h>
#include <linux/i2c-aml.h>
#include <mach/power_gate.h>
#include <linux/aml_bl.h>
#include <linux/reboot.h>
#include <linux/kthread.h>

#ifdef CONFIG_AM_UART_WITH_S_CORE 
#include <linux/uart-aml.h>
#endif
#include <mach/card_io.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <mach/pinmux.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <mach/clk_set.h>
#include "board-8726m-refb04.h"
#ifdef CONFIG_EFUSE
#include <linux/efuse.h>
#endif

#include <linux/adc_ts_key.h>
#if defined(CONFIG_TOUCHSCREEN_ADS7846)
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/spi/ads7846.h>
#endif

#ifdef CONFIG_ANDROID_PMEM
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#endif

#ifdef CONFIG_SN7325
#include <linux/sn7325.h>
#endif

#ifdef CONFIG_AMLOGIC_PM
#include <linux/power_supply.h>
#include <linux/aml_power.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#ifdef CONFIG_SUSPEND
#include <mach/pm.h>
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE
#include <media/amlogic/aml_camera.h>
#endif

#if defined(CONFIG_SND_AML_M1_MID_WM8988)||defined(CONFIG_SND_AML_M1_MID_ES8388)
int audio_dai_type = 0;
#include <sound/audio_common.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_INDEPENDENT_I2C)

# define GPIO_LCD_PWR_PIN 14
# define GPIO_VCCx3_PIN 13
# define GPIO_LCD_CTL_MUX 1

#else

# define GPIO_LCD_PWR_PIN 22
# define GPIO_VCCx3_PIN 21
# define GPIO_LCD_CTL_MUX 4

#endif

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
# define GPIO_PWR_HOLD_PIN 0
#else
# define GPIO_PWR_HOLD_PIN 8
#endif


#if defined(CONFIG_JPEGLOGO)
static struct resource jpeglogo_resources[] = {
    [0] = {
        .start = CONFIG_JPEGLOGO_ADDR,
        .end   = CONFIG_JPEGLOGO_ADDR + CONFIG_JPEGLOGO_SIZE - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device jpeglogo_device = {
    .name = "jpeglogo-dev",
    .id   = 0,
    .num_resources = ARRAY_SIZE(jpeglogo_resources),
    .resource      = jpeglogo_resources,
};
#endif

#if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_KEYPADS_AM_MODULE)
static struct resource intput_resources[] = {
    {
        .start = 0x0,
        .end = 0x0,
        .name="8726",
        .flags = IORESOURCE_IO,
    },
};

static struct platform_device input_device = {
    .name = "m1-kp",
    .id = 0,
    .num_resources = ARRAY_SIZE(intput_resources),
    .resource = intput_resources,
    
};
#endif

#ifdef CONFIG_SARADC_AM
#include <linux/saradc.h>
static struct platform_device saradc_device = {
    .name = "saradc",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};
#endif

#ifdef CONFIG_EFUSE
static bool efuse_data_verify(unsigned char *usid)
{  int len;
  
    len = strlen(usid);
    if((len > 8)&&(len<31) )
        return true;
		else
				return false;
}

static struct efuse_platform_data aml_efuse_plat = {
    .pos = 337,
    .count = 30,
    .data_verify = efuse_data_verify,
};

static struct platform_device aml_efuse_device = {
    .name	= "efuse",
    .id	= -1,
    .dev = {
                .platform_data = &aml_efuse_plat,
           },
};
#endif

static struct platform_device am_net8218_device = {
    .name = "am_net8218",
    .id = 0,
    .dev = {
        .platform_data = NULL,
    },
};


#ifdef CONFIG_ADC_TOUCHSCREEN_AM
#include <linux/adc_ts.h>

static struct adc_ts_platform_data adc_ts_pdata = {
    .irq = -1,  //INT_SAR_ADC
    .x_plate_ohms = 400,
};

static struct platform_device adc_ts_device = {
    .name = "adc_ts",
    .id = 0,
    .dev = {
        .platform_data = &adc_ts_pdata,
    },
};
#endif

#if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
#include <linux/input.h>
#include <linux/adc_keypad.h>

static struct adc_key adc_kp_key[] = {

#if defined(CONFIG_BOARD_H0)
	{114,			"vol-", CHAN_4, 000, 60},						
	{115,			"vol+", CHAN_4, 180, 60},
    {102,           "home", CHAN_4, -1, 0},
    {139,           "menu", CHAN_4, -1, 0},
    {158,            "back", CHAN_4, -1, 0},  	
#elif defined(CONFIG_BOARD_H1)
    {114,			"vol-", CHAN_4, 180, 60},
    {115,			"vol+", CHAN_4, 000, 60},
    {212,           "camara", CHAN_4, 400, 60},
 	{217,           "search", CHAN_4, -1, 0},    
	{102,			"home", CHAN_4, -1, 0},						
	{139,			"menu", CHAN_4, -1, 0},
	{158,			"back", CHAN_4, -1, 0},	    
#elif defined(CONFIG_BOARD_K0)||defined(CONFIG_BOARD_K1)
    {102,           "home", CHAN_4, 000, 20},
    {139,           "menu", CHAN_4, 180, 20},
    {158,            "back", CHAN_4, 400, 20},    
    {114,           "vol-", CHAN_4, 538, 20},                       
    {115,           "vol+", CHAN_4, 625, 20},
    {-100,          "gps",  CHAN_0, /*180*/1010, 20},
#else
	{102,			"home", CHAN_4, 000, 60},						//= vol *1023/3.3v
	{139,			"menu", CHAN_4, 180, 60},
	{158,			"back", CHAN_4, 400, 60},
#endif
};


#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)
static struct gpio_key gpio_kp_key[] = {
    {212,   "camara" ,  GPIOD_bank_bit2_24(12), GPIOD_bit_bit2_24(12),  0},
};
#endif


#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)

static void gps_key_callback(int code,int pressed);
static void ts_calibration_callback(int code,int pressed);

static struct fn_key adc_kp_fn_key[] = {
#if 0
    {102,   127,        "search"},
    {125,   109,        "vol-"},
    {15,    104,        "vol+"},
#elif defined(CONFIG_BOARD_H1)
    {212,  212,     "calibration",ts_calibration_callback},
#elif defined(CONFIG_BOARD_K0)||defined(CONFIG_BOARD_K1)
    {-100,  0,      "lock gps",gps_key_callback},
#else
#   error "not fn key defined"
#endif
};

#define FN_KEY_BANK GPIOD_bank_bit2_24(12)
#define FN_KEY_BIT  GPIOD_bit_bit2_24(12)

int fn_key_pressed(void)
{
#if 0
    return get_gpio_val(FN_KEY_BANK,FN_KEY_BIT);
#else
    return 0;
#endif
}

int fn_key_convert(const struct fn_key *fn_key,int fn_key_num,int code,int pressed,int fn)
{
    int i;
    
    if(fn){
        for(i = 0; i < fn_key_num; i++){
            if(code == fn_key[i].code){
                code = fn_key[i].fn_code;

                //fn can use call callback
                if(fn_key[i].fn_callback){
                    (*fn_key[i].fn_callback)(code,pressed);
                }
                break;
            }
        }
    }

    return code;
}

#endif

static struct key_list pb_key_list={
    .key = &adc_kp_key[0],
    .key_num = ARRAY_SIZE(adc_kp_key),
    
#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)
    .gpio_key = &gpio_kp_key[0],
    .gpio_key_num = ARRAY_SIZE(gpio_kp_key),
#endif

#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)    
    .fn_key = &adc_kp_fn_key[0],
    .fn_key_num = ARRAY_SIZE(adc_kp_fn_key),
    .fn_init = NULL,
    .fn_pressed = fn_key_pressed,
    .fn_convert = fn_key_convert,
#endif
};


static struct adc_kp_platform_data adc_kp_pdata = {
    .list = &pb_key_list
};

static struct platform_device adc_kp_device = {
    .name = "m1-adckp",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
    .platform_data = &adc_kp_pdata,
    }
};
#endif

#if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
#include <linux/input.h>
#include <linux/input/key_input.h>

int _key_code_list[] = {KEY_POWER};

static int key_input_init_func(void)
{
    WRITE_CBUS_REG(0x21d0/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d0/*RTC_ADDR0*/) &~(1<<11)));
    WRITE_CBUS_REG(0x21d1/*RTC_ADDR0*/, (READ_CBUS_REG(0x21d1/*RTC_ADDR0*/) &~(1<<3)));

    return 0;
}
static int key_scan(int *key_state_list)
{
    int ret = 0;
    key_state_list[0] = ((READ_CBUS_REG(0x21d1/*RTC_ADDR1*/) >> 2) & 1) ? 0 : 1;
    return ret;
}

static  struct key_input_platform_data  key_input_pdata = {
    .scan_period = 20,
    .fuzz_time = 60,
    .key_code_list = &_key_code_list[0],
    .key_num = ARRAY_SIZE(_key_code_list),
    .scan_func = key_scan,
    .init_func = key_input_init_func,
    .config = 0,
};

static struct platform_device input_device_key = {
    .name = "m1-keyinput",
    .id = 0,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &key_input_pdata,
    }
};
#endif

#ifdef CONFIG_SN7325
static int sn7325_pwr_rst(void)
{
    //reset
    set_gpio_val(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), 0); //low
    set_gpio_mode(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), GPIO_OUTPUT_MODE);

    udelay(2); //delay 2us

    set_gpio_val(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), 1); //high
    set_gpio_mode(GPIOD_bank_bit2_24(20), GPIOD_bit_bit2_24(20), GPIO_OUTPUT_MODE);
    //end

    return 0;
}

static struct sn7325_platform_data sn7325_pdata = {
    .pwr_rst = &sn7325_pwr_rst,
};
#endif

#if defined(CONFIG_FB_AM)
static struct resource fb_device_resources[] = {
    [0] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#if defined(CONFIG_FB_OSD2_ENABLE)
    [1] = {
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
#endif
};

static struct platform_device fb_device = {
    .name       = "mesonfb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(fb_device_resources),
    .resource      = fb_device_resources,
};
#endif
#ifdef CONFIG_USB_DWC_OTG_HCD

#if 0
static void set_usb_a_vbus_power(char is_power_on)
{
#define USB_A_POW_GPIO          PREG_EGPIO
#define USB_A_POW_GPIO_BIT      3
#define USB_A_POW_GPIO_BIT_ON   1
#define USB_A_POW_GPIO_BIT_OFF  0
    if(is_power_on) {
        //set_gpio_val(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), 1);
        //set_gpio_mode(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), GPIO_OUTPUT_MODE);
        printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_ON);
    }
    else {
        printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        //set_gpio_val(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), 0);
        //set_gpio_mode(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), GPIO_OUTPUT_MODE);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_OFF);
    }
}
//usb_a is OTG port
static struct lm_device usb_ld_a = {
    .type = LM_DEVICE_TYPE_USB,
    .id = 0,
    .irq = INT_USB_A,
    .resource.start = IO_USB_A_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_OTG,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE,
    .set_vbus_power = set_usb_a_vbus_power,
};
#else
//-----------------------------------------------------------------------------
static void set_usb_a_vbus_power(char is_power_on)
{
#define USB_A_POW_GPIO          PREG_EGPIO
#define USB_A_POW_GPIO_BIT      3
#define USB_A_POW_GPIO_BIT_ON   1
#define USB_A_POW_GPIO_BIT_OFF  0

    if(is_power_on) 
	{
        set_gpio_val(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), 1);
        set_gpio_mode(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), GPIO_OUTPUT_MODE);
        printk(KERN_INFO "set usb port power on (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_ON);
    }
    else
	{
        printk(KERN_INFO "set usb port power off (board gpio %d)!\n",USB_A_POW_GPIO_BIT);
        set_gpio_val(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), 0);
        set_gpio_mode(GPIOA_bank_bit(26), GPIOA_bit_bit23_26(26), GPIO_OUTPUT_MODE);
        set_gpio_mode(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,GPIO_OUTPUT_MODE);
        set_gpio_val(USB_A_POW_GPIO,USB_A_POW_GPIO_BIT,USB_A_POW_GPIO_BIT_OFF);
    }
}
//usb_a is OTG port
static struct lm_device usb_ld_a = 
{
    .type = LM_DEVICE_TYPE_USB,
    .id = 0,
    .irq = INT_USB_A,
    .resource.start = IO_USB_A_BASE,
    .resource.end = -1,
    .dma_mask_room = DMA_BIT_MASK(32),
    .port_type = USB_PORT_TYPE_OTG,
    .port_speed = USB_PORT_SPEED_DEFAULT,
    .dma_config = USB_DMA_BURST_SINGLE,
    .set_vbus_power = set_usb_a_vbus_power,
};

static struct lm_device usb_ld_b = 
{
	.type = LM_DEVICE_TYPE_USB,
	.id = 1,
	.irq = INT_USB_B,
	.resource.start = IO_USB_B_BASE,
	.resource.end = -1,
	.dma_mask_room = DMA_BIT_MASK(32),
	.port_type = USB_PORT_TYPE_HOST,
	.port_speed = USB_PORT_SPEED_DEFAULT,
	.dma_config = USB_DMA_BURST_SINGLE,
	.set_vbus_power = set_usb_a_vbus_power,
};


//-----------------------------------------------------------------------------
#endif


#endif







#ifdef CONFIG_SATA_DWC_AHCI
static struct lm_device sata_ld = {
    .type = LM_DEVICE_TYPE_SATA,
    .id = 2,
    .irq = INT_SATA,
    .dma_mask_room = DMA_BIT_MASK(32),
    .resource.start = IO_SATA_BASE,
    .resource.end = -1,
};
#endif

#if defined(CONFIG_AM_STREAMING)
static struct resource codec_resources[] = {
    [0] = {
        .start =  CODEC_ADDR_START,
        .end   = CODEC_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = STREAMBUF_ADDR_START,
	 .end = STREAMBUF_ADDR_END,
	 .flags = IORESOURCE_MEM,
    },
};

static struct platform_device codec_device = {
    .name       = "amstream",
    .id         = 0,
    .num_resources = ARRAY_SIZE(codec_resources),
    .resource      = codec_resources,
};
#endif

#if defined(CONFIG_AM_VIDEO)
static struct resource deinterlace_resources[] = {
    [0] = {
        .start =  DI_ADDR_START,
        .end   = DI_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device deinterlace_device = {
    .name       = "deinterlace",
    .id         = 0,
    .num_resources = ARRAY_SIZE(deinterlace_resources),
    .resource      = deinterlace_resources,
};
#endif

#if defined(CONFIG_TVIN_VDIN)
static struct resource vdin_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,  //pbufAddr
        .end   = VDIN_ADDR_END,     //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = VDIN_ADDR_START,
        .end   = VDIN_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
    [3] = {
        .start = INT_VDIN_VSYNC,
        .end   = INT_VDIN_VSYNC,
        .flags = IORESOURCE_IRQ,
    },
};

static struct platform_device vdin_device = {
    .name       = "vdin",
    .id         = -1,
    .num_resources = ARRAY_SIZE(vdin_resources),
    .resource      = vdin_resources,
};
#endif

#ifdef CONFIG_TVIN_BT656IN
//add pin mux info for bt656 input
#if 0
static struct resource bt656in_resources[] = {
    [0] = {
        .start =  VDIN_ADDR_START,      //pbufAddr
        .end   = VDIN_ADDR_END,             //pbufAddr + size
        .flags = IORESOURCE_MEM,
    },
    [1] = {     //bt656/camera/bt601 input resource pin mux setting
        .start =  0x3000,       //mask--mux gpioD 15 to bt656 clk;  mux gpioD 16:23 to be bt656 dt_in
        .end   = PERIPHS_PIN_MUX_5 + 0x3000,
        .flags = IORESOURCE_MEM,
    },

    [2] = {         //camera/bt601 input resource pin mux setting
        .start =  0x1c000,      //mask--mux gpioD 12 to bt601 FIQ; mux gpioD 13 to bt601HS; mux gpioD 14 to bt601 VS;
        .end   = PERIPHS_PIN_MUX_5 + 0x1c000,
        .flags = IORESOURCE_MEM,
    },

    [3] = {         //bt601 input resource pin mux setting
        .start =  0x800,        //mask--mux gpioD 24 to bt601 IDQ;;
        .end   = PERIPHS_PIN_MUX_5 + 0x800,
        .flags = IORESOURCE_MEM,
    },

};
#endif

static struct platform_device bt656in_device = {
    .name       = "amvdec_656in",
    .id         = -1,

};

#endif

#if defined(CONFIG_CARDREADER)
static struct resource amlogic_card_resource[] = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};

void extern_wifi_power(int is_power)
{
    if (0 == is_power)
    {
        #ifdef CONFIG_SN7325
        printk("power on 7325 3\n");
        configIO(0, 0);
        setIO_level(0, 0, 5);
        #else
        return;
        #endif
    }
    else
    {
        #ifdef CONFIG_SN7325
        printk("power on 7325 4\n");
        configIO(0, 0);
        setIO_level(0, 0, 0);//OD0
        setIO_level(0, 1, 4);//OD4
        setIO_level(0, 1, 5);//OD5
        setIO_level(0, 0, 6);//OD6
        configIO(1, 0);
        setIO_level(1, 1, 0);//PP0
        setIO_level(1, 0, 1);//PP1
        setIO_level(1, 1, 5);//PP5
        setIO_level(1, 0, 6);//PP6
        setIO_level(1, 0, 7);//PP7
        #else
        return;
        #endif
    }
    return;
}

void sdio_extern_init(void)
{
    extern_wifi_power(1);
}

static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_GPIOA_9_14,
        .card_ins_en_reg = EGPIO_GPIOC_ENABLE,
        .card_ins_en_mask = PREG_IO_0_MASK,
        .card_ins_input_reg = EGPIO_GPIOC_INPUT,
        .card_ins_input_mask = PREG_IO_0_MASK,
        .card_power_en_reg = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev = 0,
        .card_wp_en_reg = EGPIO_GPIOA_ENABLE,
        .card_wp_en_mask = PREG_IO_11_MASK,
        .card_wp_input_reg = EGPIO_GPIOA_INPUT,
        .card_wp_input_mask = PREG_IO_11_MASK,
        .card_extern_init = 0,
    },
    /*
    [1] = {
        .name = "sdio_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_GPIOB_2_7,
        .card_ins_en_reg = 0,
        .card_ins_en_mask = 0,
        .card_ins_input_reg = 0,
        .card_ins_input_mask = 0,
        .card_power_en_reg = EGPIO_GPIOD_ENABLE,
        .card_power_en_mask = PREG_IO_10_MASK,
        .card_power_output_reg = EGPIO_GPIOD_OUTPUT,
        .card_power_output_mask = PREG_IO_10_MASK,
        .card_power_en_lev = 1,
        .card_wp_en_reg = 0,
        .card_wp_en_mask = 0,
        .card_wp_input_reg = 0,
        .card_wp_input_mask = 0,
        .card_extern_init = sdio_extern_init,
    },
    */
};

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = { 
    .name = "AMLOGIC_CARD", 
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};

#endif

#if defined(CONFIG_AML_AUDIO_DSP)
static struct resource audiodsp_resources[] = {
    [0] = {
        .start = AUDIODSP_ADDR_START,
        .end   = AUDIODSP_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device audiodsp_device = {
    .name       = "audiodsp",
    .id         = 0,
    .num_resources = ARRAY_SIZE(audiodsp_resources),
    .resource      = audiodsp_resources,
};
#endif

#if defined(CONFIG_AML_HDMI_TX)
extern int m_control_codec_power;
#endif

static int audio_is_hp_pluged(void)
{

	int level = 0;
	int cs_no = 0;

//	Enable VBG_EN
	WRITE_CBUS_REG_BITS(PREG_AM_ANALOG_ADDR, 1, 0, 1);

//	wire pm_gpioA_7_led_pwm = pin_mux_reg0[22];
	WRITE_CBUS_REG(LED_PWM_REG0,(0 << 31)   |       // disable the overall circuit
                                (0 << 30)   |       // 1:Closed Loop  0:Open Loop
                                (0 << 16)   |       // PWM total count
                                (0 << 13)   |       // Enable
                                (1 << 12)   |       // enable
                                (0 << 10)   |       // test
                                (7 << 7)    |       // CS0 REF, Voltage FeedBack: about 0.505V
                                (7 << 4)    |       // CS1 REF, Current FeedBack: about 0.505V
                                (0 << 0));           // DIMCTL Analog dimmer

	cs_no = READ_CBUS_REG(LED_PWM_REG3);

	if(cs_no & (1<<14) ) {
	    //level |= (1<<0);
	    level = 1;
        } else {
	    level = 0;
	}

#if defined(CONFIG_AML_HDMI_TX)
	//printk("wm8988_is_hp_pluged level = 0x%x, cs_no = 0x%x,power = 0x%x\n",level,cs_no,m_control_codec_power);
	
	if(m_control_codec_power){
		return 0x2;	
	}
#endif

    return level;
}

static void audio_codec_power(int enable){
	
	if( enable ){		
        set_gpio_val(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), 1);
        set_gpio_mode(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), GPIO_OUTPUT_MODE);
	} else {
        set_gpio_val(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), 0);
        set_gpio_mode(GPIOD_bank_bit2_24(4), GPIOD_bit_bit2_24(4), GPIO_OUTPUT_MODE);
	}
	return;	
}

static void audio_amp_power(int enable){
	//speaker en
	if( enable ){		
        set_gpio_val(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), 1);
        set_gpio_mode(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), GPIO_OUTPUT_MODE);
	} else {		
        set_gpio_val(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), 0);
        set_gpio_mode(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), GPIO_OUTPUT_MODE);
	}
	return;	
}

static struct audio_platform_data audio_pdata = {
    .is_hp_pluged = audio_is_hp_pluged,
    .codec_power_control = audio_codec_power,
    .amp_power_control = audio_amp_power,
};

static struct resource aml_m1_audio_resource[]={
    [0] =   {
            .start  =   0,
            .end        =   0,
            .flags  =   IORESOURCE_MEM,
    },
};



static struct platform_device aml_audio={
    .name               = "aml_m1_codec",
    .id                     = -1,
    .resource       =   aml_m1_audio_resource,
    .num_resources  =   ARRAY_SIZE(aml_m1_audio_resource),
    .dev = {
        .platform_data = (void *)&audio_pdata,
    },
};

#include <linux/i2c/touch_common.h>

//touch reset pin
/*
#   if  defined(CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN)||defined(CONFIG_PIXCIR_C44_CAPACITIVE_TOUCHSCREEN)||defined(CONFIG_FT5301_TOUCHSCREEN)||defined(CONFIG_NOVATEK_TOUCHSCREEN)
#define GPIO_TOUCH_EN_BANK (GPIOD_bank_bit2_24(5))
#define GPIO_TOUCH_EN_BIT  (GPIOD_bit_bit2_24(5))
#   if defined(CONFIG_PIXCIR_C44_CAPACITIVE_TOUCHSCREEN)
#   define GPIO_TOUCH_EN_VAL  1
#   else
#   define GPIO_TOUCH_EN_VAL  0
#   endif

// init enable
#   if defined(CONFIG_NOVATEK_TOUCHSCREEN)
#   define TOUCH_INIT_ENABLE 0
#   else
#   define TOUCH_INIT_ENABLE 1
#   endif
#endif
*/
#define GPIO_TOUCH_EN_BANK (GPIOD_bank_bit2_24(5))
#define GPIO_TOUCH_EN_BIT  (GPIOD_bit_bit2_24(5))

//touch irq pin
#define GPIO_TOUCH_IRQ_PIN 13
#define GPIO_TOUCH_PENIRQ ((GPIOD_bank_bit2_24(GPIO_TOUCH_IRQ_PIN)<<16) |GPIOD_bit_bit2_24(GPIO_TOUCH_IRQ_PIN))
#define GPIO_TOUCH_IRQ_IDX     (GPIOD_IDX + GPIO_TOUCH_IRQ_PIN)

#define GPIO_TOUCH_INT_GRUP   INT_GPIO_0
static int touch_init_irq(void)
{
/* memson
    Bit(s)  Description
    256-105 Unused
    104     JTAG_TDO
    103     JTAG_TDI
    102     JTAG_TMS
    101     JTAG_TCK
    100     gpioA_23
    99      gpioA_24
    98      gpioA_25
    97      gpioA_26
    98-76    gpioE[21:0]
    75-50   gpioD[24:0]
    49-23   gpioC[26:0]
    22-15   gpioB[22;15]
    14-0    gpioA[14:0]
 */
    printk("touch_init_irq \n");
    /* set input mode */
    gpio_direction_input(GPIO_TOUCH_PENIRQ);
    /* set gpio interrupt #0 source=GPIOD_24, and triggered by falling edge(=1) */
    gpio_enable_edge_int(GPIO_TOUCH_IRQ_IDX, 1, GPIO_TOUCH_INT_GRUP -INT_GPIO_0);
    
    return 0;
}
static int touch_get_irq_level(void)
{
    return gpio_get_value(GPIO_TOUCH_PENIRQ);
}

static int touch_enable(int en)
{
    printk("touch_enable %d\n",en);

#if defined(CONFIG_TOUCHSCREEN_ENABLE_PIN_LEVEL)
    if(en)
	    set_gpio_val(GPIO_TOUCH_EN_BANK, GPIO_TOUCH_EN_BIT, CONFIG_TOUCHSCREEN_ENABLE_PIN_LEVEL); 
    else
        set_gpio_val(GPIO_TOUCH_EN_BANK, GPIO_TOUCH_EN_BIT, !CONFIG_TOUCHSCREEN_ENABLE_PIN_LEVEL);
	set_gpio_mode(GPIO_TOUCH_EN_BANK, GPIO_TOUCH_EN_BIT, GPIO_OUTPUT_MODE); 
#endif
    return 0; 
}

static int touch_init(void)
{
#if defined(CONFIG_TOUCHSCREEN_INIT_ENABLED)
       return touch_enable(CONFIG_TOUCHSCREEN_INIT_ENABLED);
#endif
    return 0;
}

#if defined(CONFIG_UOR6X5X_RESISTIVE_TOUCHSCREEN)

#define TOUCH_ADC_SHIFT 12
#    if defined(CONFIG_BOARD_K0)
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    1
#define TOUCH_YPOL    1
#    else
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0
#    endif

#define TOUCH_XMIN 0
#define TOUCH_XMAX (1<<TOUCH_ADC_SHIFT)
#define TOUCH_YMIN 0
#define TOUCH_YMAX (1<<TOUCH_ADC_SHIFT)
#define TOUCH_PMIN 0
#define TOUCH_PMAX (1<<TOUCH_ADC_SHIFT)

#elif defined(CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN)||defined(CONFIG_PIXCIR_C44_CAPACITIVE_TOUCHSCREEN)

#   if defined(CONFIG_PIXCIR_RESOLUTION_STD_1024_600_STD0)
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0

#define TOUCH_XMIN 0
#define TOUCH_XMAX 1024
#define TOUCH_YMIN 0
#define TOUCH_YMAX 600
#define TOUCH_PMIN 0
#define TOUCH_PMAX 1024

#   elif defined(CONFIG_PIXCIR_RESOLUTION_STD_1024_600_STD1)
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    1

#define TOUCH_XMIN 0
#define TOUCH_XMAX 1024
#define TOUCH_YMIN 0
#define TOUCH_YMAX 600
#define TOUCH_PMIN 0
#define TOUCH_PMAX 1024

#   elif defined(CONFIG_PIXCIR_RESOLUTION_STD_1024_600_NTD0)
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    1

#define TOUCH_XMIN 1
#define TOUCH_XMAX 1024
#define TOUCH_YMIN 1
#define TOUCH_YMAX 600
#define TOUCH_PMIN 0
#define TOUCH_PMAX 1024

#   else
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0

#define TOUCH_XMIN 0
#define TOUCH_XMAX 19968
#define TOUCH_YMIN 0
#define TOUCH_YMAX 11264
#define TOUCH_PMIN 0
#define TOUCH_PMAX 20000
#   endif

#elif defined(CONFIG_FT5301_TOUCHSCREEN)


#define TOUCH_SWAP_XY 1
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0

#define TOUCH_XMIN 0
#define TOUCH_XMAX 1024
#define TOUCH_YMIN 0
#define TOUCH_YMAX 600
#define TOUCH_PMIN 0
#define TOUCH_PMAX 255


//#define TOUCH_Y_K0  1060
//#define TOUCH_Y_K0_SHIFT 10

#elif defined(CONFIG_NOVATEK_TOUCHSCREEN)


#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0

#define TOUCH_XMIN 0
#define TOUCH_XMAX (2368 + 64)
#define TOUCH_YMIN 0
#define TOUCH_YMAX (1344 + 64)
#define TOUCH_PMIN 0
#define TOUCH_PMAX 255


#elif  defined(CONFIG_GOODIX_CAPACITIVE_TOUCHSCREEN) 
#define TOUCH_ADC_SHIFT 12
#define TOUCH_SWAP_XY 1
#define TOUCH_XPOL    0
#define TOUCH_YPOL    1

#define TOUCH_XMIN 55
#define TOUCH_XMAX 7635
#define TOUCH_YMIN 55
#define TOUCH_YMAX 5075
#define TOUCH_PMIN 0
#define TOUCH_PMAX 0

#else
#define TOUCH_SWAP_XY 0
#define TOUCH_XPOL    0
#define TOUCH_YPOL    0

#define TOUCH_XMIN 0
#define TOUCH_XMAX 0
#define TOUCH_YMIN 0
#define TOUCH_YMAX 0
#define TOUCH_PMIN 0
#define TOUCH_PMAX 0
#endif

#define X_OSD_TO_TS(_x)     (((_x)*(TOUCH_XMAX-TOUCH_XMIN)/(CONFIG_FB_OSD1_DEFAULT_WIDTH + CONFIG_FB_OSD1_ZTKEY_LEN)) + TOUCH_XMIN)
#define Y_OSD_TO_TS(_y)     (((_y)*(TOUCH_YMAX-TOUCH_YMIN)/(CONFIG_FB_OSD1_DEFAULT_HEIGHT)) + TOUCH_YMIN)
#define X_TS_TO_OSD(_x)     (((_x)-TOUCH_XMIN)*(CONFIG_FB_OSD1_DEFAULT_WIDTH + CONFIG_FB_OSD1_ZTKEY_LEN)/(TOUCH_XMAX-TOUCH_XMIN))
#define Y_TS_TO_OSD(_y)     (((_y)-TOUCH_YMIN)*(CONFIG_FB_OSD1_DEFAULT_HEIGHT)/(TOUCH_YMAX-TOUCH_YMIN))

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    

#define ZT_KEY_X0 (TOUCH_XMAX - X_OSD_TO_TS(CONFIG_FB_OSD1_ZTKEY_LEN - 5))
#define ZT_KEY_X1 (TOUCH_XMAX)
#define ZT_KEY_DY ((TOUCH_YMAX - TOUCH_YMIN)/5)

static struct ts_key ts_kp_key[]={
#if defined(CONFIG_BOARD_H0)
    {158,  "back", 3800,1100,4000,1400},  //(3840,1220)
    {139, "menu", 3800,1900,4000,2200},  //(3840,2050)
    {102, "home", 3800,2700,4000,3000},  //(3840,2860)
#elif defined(CONFIG_BOARD_H1)
    {127, "search"},  
    {158,  "back"},  
    {102, "home"},
    {139, "menu"},
#endif

#if (CONFIG_FB_OSD1_ZTKEY_LEN > 0)  //the key is at (min,max)
    {115,  "vol+", ZT_KEY_X0,TOUCH_YMIN,                                 ZT_KEY_X1,TOUCH_YMIN + ZT_KEY_DY - Y_OSD_TO_TS(5)},
    {114,  "vol-", ZT_KEY_X0,TOUCH_YMIN + ZT_KEY_DY + Y_OSD_TO_TS(5),    ZT_KEY_X1,TOUCH_YMIN + ZT_KEY_DY*2 - Y_OSD_TO_TS(5)},
    {139,  "menu", ZT_KEY_X0,TOUCH_YMIN + ZT_KEY_DY*2 + Y_OSD_TO_TS(5),  ZT_KEY_X1,TOUCH_YMIN + ZT_KEY_DY*3 - Y_OSD_TO_TS(5)},
    {158,   "back", ZT_KEY_X0,TOUCH_YMIN + ZT_KEY_DY*3 + Y_OSD_TO_TS(5),  ZT_KEY_X1,TOUCH_YMIN + ZT_KEY_DY*4 - Y_OSD_TO_TS(5)},
    {102,  "home", ZT_KEY_X0,TOUCH_YMIN + ZT_KEY_DY*4 + Y_OSD_TO_TS(5),  ZT_KEY_X1,TOUCH_YMIN + ZT_KEY_DY*5 - Y_OSD_TO_TS(5)},
#endif   

};
#endif

static int touch_convert(int x, int y)
{
#if (TOUCH_SWAP_XY == 1)
    swap(x, y);
#endif
    if (x < TOUCH_XMIN) x = TOUCH_XMIN;
    if (x > TOUCH_XMAX) x = TOUCH_XMAX;
    if (y < TOUCH_YMIN) y = TOUCH_YMIN;
    if (y > TOUCH_YMAX) y = TOUCH_YMAX;
#if (TOUCH_XPOL == 1)
    x = TOUCH_XMAX + TOUCH_XMIN - x;
#endif
#if (TOUCH_YPOL == 1)
    y = TOUCH_YMAX + TOUCH_YMIN - y;
#endif
	//printk("*** cachex=%d cachey=%d......\n", x, y);
	/*
    x = (x- TOUCH_XMIN) * TOUCH_XMAX / (TOUCH_XMAX - TOUCH_XMIN);
    y = (y- TOUCH_YMIN) * TOUCH_YMAX / (TOUCH_YMAX - TOUCH_YMIN);
    */
    //y = y - 32 * UOR6X5X_YLCD / (y / 2 + UOR6X5X_YLCD);
    //printk("### x=%d y=%d\n", x, y);
    return (x << 16) | y;
}

static int adjust_middle_k0(int val,int min,int max,int k0,int power){
    int delta,delta_adjust,screen_m;

    if(!k0 || !power)
        return val;

    val-=min;
    
    screen_m = (max - min)>>1;
    delta = (val > screen_m) ? (val - screen_m):(screen_m - val);

    if(delta){
        delta_adjust = ((delta * k0) >> power);
    
        if(val > screen_m)
            val = delta_adjust + screen_m + min;
        else
            val = screen_m - delta_adjust;
        if(val > max)
            val = max;
        if(val < min)
            val = min;
    }
    return val;
}

static void touch_convert2(int *in_out_x, int *in_out_y)
{
    int x,y;

    if(!in_out_x || !in_out_y)
        return;
    
    x = *in_out_x;
    y = *in_out_y;

#if (TOUCH_SWAP_XY == 1)
    swap(x, y);
#endif

#if (TOUCH_XPOL == 1)
    x = TOUCH_XMAX + TOUCH_XMIN - x;
#endif
#if (TOUCH_YPOL == 1)
    y = TOUCH_YMAX + TOUCH_YMIN - y;
#endif

if (x <= TOUCH_XMIN) x = TOUCH_XMIN + 1;
if (x >= TOUCH_XMAX) x = TOUCH_XMAX - 1;
if (y <= TOUCH_YMIN) y = TOUCH_YMIN + 1;
if (y >= TOUCH_YMAX) y = TOUCH_YMAX - 1;

#if defined(TOUCH_X_K0)
    x = adjust_middle_k0(x,TOUCH_XMIN,TOUCH_XMAX,TOUCH_X_K0,TOUCH_X_K0_SHIFT);
#endif
#if defined(TOUCH_Y_K0)
    y = adjust_middle_k0(y,TOUCH_YMIN,TOUCH_YMAX,TOUCH_Y_K0,TOUCH_Y_K0_SHIFT);
#endif

    //printk("(%d,%d) -> (%d,%d)\n",*in_out_x,*in_out_y,x,y);

    *in_out_x = x;
    *in_out_y = y;
}

#if defined(CONFIG_TOUCHSCREEN_TS_KEY)

static int touch_to_key(const struct ts_key *key,int key_num,int x,int y)
{
    int key_status = 0;
    int i;

    for (i=0; i<key_num; i++) {
        if((x > key[i].x0 && x <key[i].x1)&&
            (y > key[i].y0 && y <key[i].y1)){
            //printk(KERN_INFO "%s ts key(%d)\n", key[i].name, key[i].code);
            key_status |= 1<<i;
            break;
        }
    }

    return key_status;
}

#if (CONFIG_FB_OSD1_ZTKEY_LEN > 0)
extern void zt_key_report(int x,int y);
#endif
int touch_report_key(struct input_dev *input,const struct ts_key *key,int key_num,int last_status,int status)
{
    int change = 0;
    int i;

    if(status > (1<<(key_num + 1))){
        printk(KERN_INFO "invalid key status  0x%x\n",status);
        status = 0;
    }

    if(status != last_status){
        change = last_status^status;
        for (i = 0; i < key_num; i++) {
            if((change >> i)&0x1){
                printk(KERN_INFO "%s key(%d) press %d\n", key[i].name, key[i].code,(status >> i)&0x1);
                if(key[i].code > 0){
                    int pressed = ((status >> i)&0x1);
#if (CONFIG_FB_OSD1_ZTKEY_LEN > 0)
                    int x,y;
                    if((key[i].x0 >= 0||key[i].x1 > key[i].x0)&&
                        (key[i].y0 >= 0||key[i].y1 > key[i].y0)){ //valid soft ts key
                        if(pressed){
                            x = (key[i].x0 + key[i].x1)>>1;
                            y = (key[i].y0 + key[i].y1)>>1;
#if ((CONFIG_FB_OSD1_DEFAULT_WIDTH + CONFIG_FB_OSD1_ZTKEY_LEN) != (TOUCH_XMAX - TOUCH_XMIN))
                            x = X_TS_TO_OSD(x);
#endif
#if (CONFIG_FB_OSD1_DEFAULT_HEIGHT != (TOUCH_YMAX - TOUCH_YMIN))
                            y = Y_TS_TO_OSD(y);
#endif       
                        }else{
                            x = y = 0;
                        }
                        zt_key_report(x,y);
                    }
#endif
                    input_event(input, EV_KEY, key[i].code,pressed);
                }
            }
        }
    }
    return status;
}


#endif


static int is_ac_connected(void);

static struct touch_platform_data touch_pdata = {
    .init_irq = &touch_init_irq,
    .get_irq_level = &touch_get_irq_level,
    .enable = &touch_enable,
    .abs_xmin = TOUCH_XMIN,
    .abs_xmax = TOUCH_XMAX - X_OSD_TO_TS(CONFIG_FB_OSD1_ZTKEY_LEN),
    .abs_ymin = TOUCH_YMIN,
    .abs_ymax = TOUCH_YMAX,
    .abs_pmin = TOUCH_PMIN,
    .abs_pmax = TOUCH_PMAX,
#if defined(CONFIG_TOUCHSCREEN_TS_KEY)    
    .key_num = ARRAY_SIZE(ts_kp_key),
    .key = &ts_kp_key[0],
    .to_key = touch_to_key,
    .report_key = touch_report_key,
#endif
    .convert = touch_convert,     //convert x,y and return ((x << 16) | y )
    .convert2 = touch_convert2,   //convert x,y and out x,y though pointer
    .is_ac_online = is_ac_connected,
};

#if defined(CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN)||defined(CONFIG_PIXCIR_C44_CAPACITIVE_TOUCHSCREEN)

atomic_t atomic_ts_calibration_key_status;

extern void do_pixcir_ts_calibration(void);
static int do_ts_calibration_thread(void *data){
	unsigned long end_time = jiffies + msecs_to_jiffies(6000);
	int stop = 0;

    printk("do_ts_calibration_thread ++\n");

	do {
		if(kthread_should_stop()){
			break;	
		}

        if(!atomic_read(&atomic_ts_calibration_key_status)){
            stop = 1;
            break;
        }
		
		schedule_timeout_interruptible(20);
		
	} while (time_before(jiffies, end_time));

    if(!stop){
        printk("thread do ts calibration\n");
        do_pixcir_ts_calibration();
    }

    printk("do_ts_calibration_thread --\n");

    return 0;
}

static void ts_calibration_callback(int code,int pressed){
    atomic_set(&atomic_ts_calibration_key_status,pressed);

    if(pressed){
        kthread_run(do_ts_calibration_thread, NULL, "ts_calibration_thread");
    }
}

#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_data =
{
    .name = "pmem",
    .start = PMEM_START,
    .size = PMEM_SIZE,
    .no_allocator = 1,
    .cached = 1,
};

static struct platform_device android_pmem_device =
{
    .name = "android_pmem",
    .id = 0,
    .dev = {
        .platform_data = &pmem_data,
    },
};
#endif

#if defined(CONFIG_AML_RTC)
static  struct platform_device aml_rtc_device = {
            .name            = "aml_rtc",
            .id               = -1,
    };
#endif

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)

#define LED_PWM_TCNT       (600-1)
#define LED_PWM_MAX_VAL    (420)
//set pwm_freq=24MHz/PWM_MAX (Base on crystal frequence: 24MHz, 0<PWM_MAX<65535)

static void aml_8726m_pwmb_init(void)
{
    unsigned val;

    printk("aml_8726m_pwmb_init\n");
    
    WRITE_CBUS_REG(LED_PWM_REG0, 0);
    WRITE_CBUS_REG(LED_PWM_REG1, 0);
    WRITE_CBUS_REG(LED_PWM_REG2, 0);
    WRITE_CBUS_REG(LED_PWM_REG3, 0);
    WRITE_CBUS_REG(LED_PWM_REG4, 0);
    val = (0 << 31)           |       // disable the overall circuit
          (0 << 30)           |       // 1:Closed Loop  0:Open Loop
          (LED_PWM_TCNT << 16)    |       // PWM total count
          (0 << 13)           |       // Enable
          (1 << 12)           |       // enable
          (0 << 10)           |       // test
          (3 << 7)            |       // CS0 REF, Voltage FeedBack: about 0.27V
          (7 << 4)            |       // CS1 REF, Current FeedBack: about 0.54V
          (0 << 0);                   // DIMCTL Analog dimmer
    WRITE_CBUS_REG(LED_PWM_REG0, val);
    val = (1 << 30)           |       // enable high frequency clock
          (LED_PWM_MAX_VAL << 16) |       // MAX PWM value
          (0 << 0);                  // MIN PWM value
    WRITE_CBUS_REG(LED_PWM_REG1, val);
    val = (0 << 31)       |       // disable timeout test mode
          (0 << 30)       |       // timeout based on the comparator output
          (0 << 16)       |       // timeout = 10uS
          (0 << 13)       |       // Select oscillator as the clock (just for grins)
          (1 << 11)       |       // 1:Enable OverCurrent Portection  0:Disable
          (3 << 8)        |       // Filter: shift every 3 ticks
          (0 << 6)        |       // Filter: count 1uS ticks
          (0 << 5)        |       // PWM polarity : negative
          (0 << 4)        |       // comparator: negative, Different with NikeD3
          (1 << 0);               // +/- 1
    WRITE_CBUS_REG(LED_PWM_REG2, val);
    val = (   1 << 16) |    // Feedback down-sampling = PWM_freq/1 = PWM_freq
          (   1 << 14) |    // enable to re-write MATCH_VAL
          ( 210 <<  0) ;  // preset PWM_duty = 50%
    WRITE_CBUS_REG(LED_PWM_REG3, val);
    val = (   0 << 30) |    // 1:Digital Dimmer  0:Analog Dimmer
          (   2 << 28) |    // dimmer_timebase = 1uS
          (1000 << 14) |    // Digital dimmer_duty = 0%, the most darkness
          (1000 <<  0) ;    // dimmer_freq = 1KHz
    WRITE_CBUS_REG(LED_PWM_REG4, val);
    
    WRITE_MPEG_REG(PWM_PWM_B, (0<<16) | (0<<0));

    WRITE_MPEG_REG(PWM_MISC_REG_AB, (READ_MPEG_REG(PWM_MISC_REG_AB) & ~(1<<3)) | (1<<1));

    //LED_BL_PWM 12-6 0-8 2-30 2-28 9-23(0-21)
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<8)|(1<<21));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<28)|(1<<30));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_9, (1<<23));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<6));
    //SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<30));
}

static void aml_8726m_pwmb_enable(int en)
{
    printk("aml_8726m_pwmb_enable %d\n",en);

    if(en){
        SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<30));
        msleep(50);
    }else{
        //gpio a-8
        set_gpio_val(GPIOA_bank_bit(8), GPIOA_bit_bit23_26(8), 0);
        set_gpio_mode(GPIOA_bank_bit(8), GPIOA_bit_bit23_26(8), GPIO_OUTPUT_MODE);
        
        CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<30));
    }
}

#endif


#if defined (CONFIG_AMLOGIC_VIDEOIN_MANAGER)
static struct resource vm_resources[] = {
    [0] = {
        .start =  VM_ADDR_START,
        .end   = VM_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device vm_device =
{
	.name = "vm",
	.id = 0,
    .num_resources = ARRAY_SIZE(vm_resources),
    .resource      = vm_resources,
};
#endif /* AMLOGIC_VIDEOIN_MANAGER */


#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308)

int gc0308_init(void)
{
    //camera power down off
    set_gpio_val(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), 0);
    set_gpio_mode(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), GPIO_OUTPUT_MODE);
    return 0;
}

static int gc0308_v4l2_init(void)
{
#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
    aml_8726m_pwmb_enable(1);
#endif
	gc0308_init();

    return 0;
}
static int gc0308_v4l2_uninit(void)
{
	//pp0

    //camera power down on
    set_gpio_val(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), 1);
    set_gpio_mode(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), GPIO_OUTPUT_MODE);
#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
    aml_8726m_pwmb_enable(0);
#endif
    return 0;
}
aml_plat_cam_data_t video_gc0308_data = {
	.name="video-gc0308",
	.video_nr=0,//1,
	.device_init= gc0308_v4l2_init,
	.device_uninit=gc0308_v4l2_uninit,
};


#endif

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005)

int gt2005_init(void)
{
    //camera power down off
    printk("gt2005 init\n");
    set_gpio_val(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), 1);
    set_gpio_mode(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), GPIO_OUTPUT_MODE);

    return 0;
}

static void gt2005_v4l2_init(void)
{
#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
    aml_8726m_pwmb_enable(1);
#endif
    gt2005_init();

    return 0;
}
static void gt2005_v4l2_uninit(void)
{
    //pp0
    printk("gt2005 uninit\n");
    //camera power down on
    set_gpio_val(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), 0);
    set_gpio_mode(GPIOE_bank_bit16_21(21), GPIOE_bit_bit16_21(21), GPIO_OUTPUT_MODE);
#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
    aml_8726m_pwmb_enable(0);
#endif
    return 0;

}
static void gt2005_v4l2_disable(void)
{

}

static void gt2005_v4l2_early_suspend(void)
{
}

static void gt2005_v4l2_late_resume(void)
{

}

aml_plat_cam_data_t video_gt2005_data = {
	.name="video-gt2005",
	.video_nr=0,   //    1
	.device_init= gt2005_v4l2_init,
	.device_uninit=gt2005_v4l2_uninit,
	.early_suspend = gt2005_v4l2_early_suspend,
	.late_resume = gt2005_v4l2_late_resume,
	.device_disable=gt2005_v4l2_disable,
};
#endif /* VIDEO_AMLOGIC_CAPTURE_GT2005 */


#if defined(CONFIG_SUSPEND)
static void set_vccx2(int power_on)
{
    if(power_on)
    {
        set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 1);
        set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);          
        //set clk for wifi
    }
    else
    {
        set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 0);
        set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);   
        //disable wifi clk
    }
}
static struct meson_pm_config aml_pm_pdata = {
    .pctl_reg_base = IO_APB_BUS_BASE,
    .mmc_reg_base = APB_REG_ADDR(0x1000),
    .hiu_reg_base = CBUS_REG_ADDR(0x1000),
    .power_key = CBUS_REG_ADDR(RTC_ADDR1),
    .ddr_clk = 0x00110820,
    .sleepcount = 128,
    .set_vccx2 = set_vccx2,
    .core_voltage_adjust = 5,
};

static struct platform_device aml_pm_device = {
    .name           = "pm-meson",
    .dev = {
        .platform_data  = &aml_pm_pdata,
    },
    .id             = -1,
};
#endif

#if defined(CONFIG_I2C_SW_AML)

static struct aml_sw_i2c_platform aml_sw_i2c_plat = {
    .sw_pins = {
        .scl_reg_out        = MESON_I2C_PREG_GPIOB_OUTLVL,
        .scl_reg_in     = MESON_I2C_PREG_GPIOB_INLVL,
        .scl_bit            = 2,    /*MESON_I2C_MASTER_A_GPIOB_2_REG*/
        .scl_oe         = MESON_I2C_PREG_GPIOB_OE,
        .sda_reg_out        = MESON_I2C_PREG_GPIOB_OUTLVL,
        .sda_reg_in     = MESON_I2C_PREG_GPIOB_INLVL,
        .sda_bit            = 3,    /*MESON_I2C_MASTER_A_GPIOB_3_BIT*/
        .sda_oe         = MESON_I2C_PREG_GPIOB_OE,
    },  
    .udelay         = 2,
    .timeout            = 100,
};

static struct platform_device aml_sw_i2c_device = {
    .name         = "aml-sw-i2c",
    .id       = -1,
    .dev = {
        .platform_data = &aml_sw_i2c_plat,
    },
};

#endif

#if defined(CONFIG_I2C_AML)
static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count     = /*1000000*/10000,
    .wait_ack_interval  = 2,
    .wait_read_interval = 5,
    .wait_xfer_interval = 5,
    .master_no      = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_400K,

    .ext_master_no  = AML_I2C_MASTER_A,
    .ext_use_pio        = 0,
    .ext_master_i2c_speed   = 
#if !defined(CONFIG_IMAGE_RECOVERY)
#if defined(CONFIG_UOR6X5X_RESISTIVE_TOUCHSCREEN)
    AML_I2C_SPPED_300K
#else
    AML_I2C_SPPED_400K
#endif
#else
    AML_I2C_SPPED_100K
#endif
    ,
    .master_a_pinmux = {
        .scl_reg    = PERIPHS_PIN_MUX_7,
        .scl_bit    = 9,
        .sda_reg    = PERIPHS_PIN_MUX_7,
        .sda_bit    = 6,     
    },
    .master_a_sw_pins = {
        .scl_bank = GPIOC_bank_bit0_26(21),
        .scl_bit = GPIOC_bit_bit0_26(21),
        .sda_bank = GPIOC_bank_bit0_26(22),
        .sda_bit = GPIOC_bit_bit0_26(22),
    },
    .master_b_pinmux = {
        .scl_reg    = PERIPHS_PIN_MUX_2,
        .scl_bit    = 5,
        .sda_reg    = PERIPHS_PIN_MUX_2,
        .sda_bit    = 2,     
    },
    .master_b_sw_pins = {
        .scl_bank = GPIOB_bank_bit0_7(0),
        .scl_bit = GPIOB_bit_bit0_7(0),
        .sda_bank = GPIOB_bank_bit0_7(1),
        .sda_bit = GPIOB_bit_bit0_7(1),
    },    
};

static void aml_i2c_pre_init(void)
{
    struct aml_i2c_platform *plat = &aml_i2c_plat;

    //config to gpio low
    set_gpio_val(plat->master_a_sw_pins.scl_bank, plat->master_a_sw_pins.scl_bit, 0);
    set_gpio_val(plat->master_b_sw_pins.sda_bank, plat->master_b_sw_pins.sda_bit, 0);
    
    set_gpio_mode(plat->master_a_sw_pins.scl_bank, plat->master_a_sw_pins.scl_bit, GPIO_OUTPUT_MODE);
    set_gpio_mode(plat->master_b_sw_pins.sda_bank, plat->master_b_sw_pins.sda_bit, GPIO_OUTPUT_MODE);
}

static struct resource aml_i2c_resource[] = {
    [0] = {/*master a*/
        .start =    MESON_I2C_MASTER_A_START,
        .end   =    MESON_I2C_MASTER_A_END,
        .flags =    IORESOURCE_MEM,
    },
    [1] = {/*master b*/
        .start =    MESON_I2C_MASTER_B_START,
        .end   =    MESON_I2C_MASTER_B_END,
        .flags =    IORESOURCE_MEM,
    },
    [2] = {/*slave*/
        .start =    MESON_I2C_SLAVE_START,
        .end   =    MESON_I2C_SLAVE_END,
        .flags =    IORESOURCE_MEM,
    },
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = -1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};

static struct platform_device aml_i2c_ext_device = {
    .name         = "aml-i2c-ext",
    .id       = -1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};
#endif

#ifdef CONFIG_AMLOGIC_PM

static int is_ac_connected(void)
{
	return (READ_CBUS_REG(ASSIST_HW_REV)&(1<<9))? 1:0;//GP_INPUT1
}

#ifdef CONFIG_USB_ANDROID
static int usb_status = 0;

int pc_connect(int status) 
{
    usb_status = status;
    return 0;
} 
static int is_usb_online(void)
{
    return usb_status;
}
EXPORT_SYMBOL(pc_connect);
#endif



static void set_charge(int flags)
{

}

#ifdef CONFIG_SARADC_AM
extern int get_adc_sample(int chan);
#endif
static int get_bat_vol(void)
{
#ifdef CONFIG_SARADC_AM
    return get_adc_sample(5);
#else
    return 0;
#endif
}

static int get_charge_status(void)
{
    return (READ_CBUS_REG(ASSIST_HW_REV)&(1<<8))? 1:0;//GP_INPUT0
    //return 0;
}

void set_bat_off(void)
{
	printk("set_bat_off\r\n");
	
    //speaker power off
    set_gpio_val(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), 0);
    set_gpio_mode(GPIOA_bank_bit(4), GPIOA_bit_bit0_14(4), GPIO_OUTPUT_MODE);  

    //bk off
    set_gpio_val(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), 0);
    set_gpio_mode(GPIOA_bank_bit(7), GPIOA_bit_bit0_14(7), GPIO_OUTPUT_MODE);  

    //vccx3 (lcd power anologic)
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_VCCx3_PIN), GPIOC_bit_bit0_26(GPIO_VCCx3_PIN), 0);
    set_gpio_mode(GPIOC_bank_bit0_26(GPIO_VCCx3_PIN), GPIOC_bit_bit0_26(GPIO_VCCx3_PIN), GPIO_OUTPUT_MODE);  

    //lcd vcc(lcd power digit)
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_LCD_PWR_PIN), GPIOC_bit_bit0_26(GPIO_LCD_PWR_PIN), 0);
    set_gpio_mode(GPIOC_bank_bit0_26(GPIO_LCD_PWR_PIN), GPIOC_bit_bit0_26(GPIO_LCD_PWR_PIN), GPIO_OUTPUT_MODE);  

    //set_vccx2 power down    
    set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 0);
    set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);  
    //if(is_ac_connected()){ //AC in after power off press
       // kernel_restart("reboot");
    //}

    //power en
    set_gpio_val(GPIOA_bank_bit(GPIO_PWR_HOLD_PIN), GPIOA_bit_bit0_14(GPIO_PWR_HOLD_PIN), 0);
    set_gpio_mode(GPIOA_bank_bit(GPIO_PWR_HOLD_PIN), GPIOA_bit_bit0_14(GPIO_PWR_HOLD_PIN), GPIO_OUTPUT_MODE);

    printk("set_bat_off end\n");
}
EXPORT_SYMBOL_GPL(set_bat_off);

static struct aml_power_pdata power_pdata = {
	.is_ac_online	= is_ac_connected,
#ifdef CONFIG_USB_ANDROID	
	.is_usb_online	= is_usb_online,
#endif
	.set_charge = set_charge,
	.get_bat_vol = get_bat_vol,
	.get_charge_status = get_charge_status,
	.set_bat_off = set_bat_off,

    .polling_interval = 30000,  //ms
    .fast_polling_interval = 10000,  //ms
    .critical_polling_interval = 3000,  //ms

#if defined(CONFIG_AMLOGIC_PM_COMPENSATION)
    .adc_stagnate_delta_low = 2,
    .adc_stagnate_delta_high = 10,
    .adc_stagnate_second = CONFIG_AMLOGIC_PM_ADC_STAGNATE_SECOND * HZ,         //jiffies
    .adc_stagnate_second_steady = CONFIG_AMLOGIC_PM_ADC_STAGNATE_SECOND_STEADY * HZ, 
    .capacity_compensation_max = 16,    //percent max 100
#endif    
};

static struct platform_device power_dev = {
    .name       = "aml-power",
    .id     = -1,
    .dev = {
        .platform_data  = &power_pdata,
    },
};
#endif

#define PINMUX_UART_A   UART_A_GPIO_B2_B3
#define PINMUX_UART_B   UART_B_GPIO_E18_E19

#if defined(CONFIG_AM_UART_WITH_S_CORE)

#if defined(CONFIG_AM_UART0_SET_PORT_A)
#define UART_0_PORT     UART_A
#define UART_1_PORT     UART_B
#elif defined(CONFIG_AM_UART0_SET_PORT_B)
#define UART_0_PORT     UART_B
#define UART_1_PORT     UART_A
#endif

static struct aml_uart_platform aml_uart_plat = {
    .uart_line[0]       =   UART_0_PORT,
    .uart_line[1]       =   UART_1_PORT
};

static struct platform_device aml_uart_device = {
    .name         = "am_uart",  
    .id       = -1, 
    .num_resources    = 0,  
    .resource     = NULL,   
    .dev = {        
                .platform_data = &aml_uart_plat,
           },
};
#endif

#ifdef CONFIG_AM_NAND
static struct mtd_partition multi_partition_info[] = 
{    
    {
        .name   = "system",
        .offset = 0x10000000,
        .size   = 0x18000000,
    },
    {
        .name   = "cache",
        .offset = 0x28000000,
        .size   = 0x08000000,
    },
    {
        .name   = "userdata",
        .offset = 0x30000000,
        .size   = 0x40000000,
    },
    {
        .name   = "NFTL_Part",
        .offset = 0x70000000,
        .size   = MTDPART_SIZ_FULL,
    },
};


static struct aml_nand_platform aml_nand_mid_platform[] = {
/*
{
		.name = NAND_BOOT_NAME,
		.chip_enable_pad = AML_NAND_CE0,
		.ready_busy_pad = AML_NAND_CE0,
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 1,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH16_MODE),
			},
    	},
		.T_REA = 20,
		.T_RHOH = 15,
	},*/
{
		.name = NAND_MULTI_NAME,
		.chip_enable_pad = (AML_NAND_CE0 | (AML_NAND_CE1 << 4) /*| (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)*/),
		.ready_busy_pad = (AML_NAND_CE0 | (AML_NAND_CE1 << 4) /*| (AML_NAND_CE0 << 8) | (AML_NAND_CE1 << 12)*/),
		.platform_nand_data = {
			.chip =  {
				.nr_chips = 2,
				.nr_partitions = ARRAY_SIZE(multi_partition_info),
				.partitions = multi_partition_info,
				.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH16_MODE | NAND_TWO_PLANE_MODE),
			},
    	},
		.T_REA = 20,
		.T_RHOH = 15,
	}
};

struct aml_nand_device aml_nand_mid_device = {
	.aml_nand_platform = aml_nand_mid_platform,
	.dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device aml_nand_device = {
    .name = "aml_m1_nand",
    .id = 0,
    .num_resources = ARRAY_SIZE(aml_nand_resources),
    .resource = aml_nand_resources,
    .dev = {
		.platform_data = &aml_nand_mid_device,
    },
};
#endif

#if defined(CONFIG_AMLOGIC_BACKLIGHT)
#include <linux/aml_bl.h>

#define VGHL_PWM_TCNT        (600-1)
#define VGHL_PWM_MAX_VAL    (420)
//set pwm_freq=24MHz/PWM_MAX (Base on crystal frequence: 24MHz, 0<PWM_MAX<65535)
#define VGHL_PWM_CNT_SHIFT  10
#define VGHL_PWM_CNT  (1<<VGHL_PWM_CNT_SHIFT)

static void aml_8726m_bl_init(void)
{
    unsigned val;
    
    WRITE_CBUS_REG(VGHL_PWM_REG0, 0);
    WRITE_CBUS_REG(VGHL_PWM_REG1, 0);
    WRITE_CBUS_REG(VGHL_PWM_REG2, 0);
    WRITE_CBUS_REG(VGHL_PWM_REG3, 0);
    WRITE_CBUS_REG(VGHL_PWM_REG4, 0);
    val = (0 << 31)           |       // disable the overall circuit
          (0 << 30)           |       // 1:Closed Loop  0:Open Loop
          (VGHL_PWM_TCNT << 16)    |       // PWM total count
          (0 << 13)           |       // Enable
          (1 << 12)           |       // enable
          (0 << 10)           |       // test
          (3 << 7)            |       // CS0 REF, Voltage FeedBack: about 0.27V
          (7 << 4)            |       // CS1 REF, Current FeedBack: about 0.54V
          (0 << 0);                   // DIMCTL Analog dimmer
    WRITE_CBUS_REG(VGHL_PWM_REG0, val);
    val = (1 << 30)           |       // enable high frequency clock
          (VGHL_PWM_MAX_VAL << 16) |       // MAX PWM value
          (0 << 0);                  // MIN PWM value
    WRITE_CBUS_REG(VGHL_PWM_REG1, val);
    val = (0 << 31)       |       // disable timeout test mode
          (0 << 30)       |       // timeout based on the comparator output
          (0 << 16)       |       // timeout = 10uS
          (0 << 13)       |       // Select oscillator as the clock (just for grins)
          (1 << 11)       |       // 1:Enable OverCurrent Portection  0:Disable
          (3 << 8)        |       // Filter: shift every 3 ticks
          (0 << 6)        |       // Filter: count 1uS ticks
          (0 << 5)        |       // PWM polarity : negative
          (0 << 4)        |       // comparator: negative, Different with NikeD3
          (1 << 0);               // +/- 1
    WRITE_CBUS_REG(VGHL_PWM_REG2, val);
    val = (   1 << 16) |    // Feedback down-sampling = PWM_freq/1 = PWM_freq
          (   1 << 14) |    // enable to re-write MATCH_VAL
          ( 210 <<  0) ;  // preset PWM_duty = 50%
    WRITE_CBUS_REG(VGHL_PWM_REG3, val);
    val = (   0 << 30) |    // 1:Digital Dimmer  0:Analog Dimmer
          (   2 << 28) |    // dimmer_timebase = 1uS
          (1000 << 14) |    // Digital dimmer_duty = 0%, the most darkness
          (1000 <<  0) ;    // dimmer_freq = 1KHz
    WRITE_CBUS_REG(VGHL_PWM_REG4, val);

    WRITE_MPEG_REG(PWM_PWM_A, ((VGHL_PWM_CNT>>1)<<16) | ((VGHL_PWM_CNT>>1)<<0));
    WRITE_MPEG_REG(PWM_MISC_REG_AB, (READ_MPEG_REG(PWM_MISC_REG_AB) & ~(1<<2)) | (1<<0));

    //LCD_VGHL_PWM 12-7 0-9 2-31 2-29 9-22(0-22)
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<9)|(1<<22));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, (1<<29));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_9, (1<<22));
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<7));
    SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<31));

    
}

static unsigned aml_8726m_get_bl_level(void)
{
    unsigned cs_level = READ_MPEG_REG(PWM_PWM_A);
    unsigned level;

    level = ((cs_level>>16)>>(VGHL_PWM_CNT_SHIFT-8));
    if(level>1)
        level-=1;
    
    return level;
}

static int lcd_boot_up = 0;
extern void aml_8726m_power_on_bl(void);

static void aml_8726m_set_bl_level(unsigned level)
{
    /*
    unsigned cs_level;

    if(level > 255)
        level = 255;

    cs_level = 15 - (level>>4);
    WRITE_CBUS_REG_BITS(VGHL_PWM_REG0, cs_level, 0, 4);*/

    int hi,lo;

    hi = (level+1)<<(VGHL_PWM_CNT_SHIFT-8);
    lo = VGHL_PWM_CNT-hi;
    
    WRITE_MPEG_REG(PWM_PWM_A, (hi<<16) | (lo<<0));

    if(level && !lcd_boot_up){
        lcd_boot_up = 1;
        aml_8726m_power_on_bl();
    }
}

void power_on_lcd(void)
{
    printk("power_on_lcd...\n");

    set_mio_mux(0, ((1<<11)|(1<<14)|(1<<15)/*|(1<<16)*/));  //TCON
    set_mio_mux(4,(1<<4)|(1<<2)|(1<<0));   //LCD DATA

    //LCD PWR
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_LCD_PWR_PIN), GPIOC_bit_bit0_26(GPIO_LCD_PWR_PIN), 0);
    set_gpio_mode(GPIOC_bank_bit0_26(GPIO_LCD_PWR_PIN), GPIOC_bit_bit0_26(GPIO_LCD_PWR_PIN), GPIO_OUTPUT_MODE);
    msleep(10);
    
    //VCCx3_EN (bk power en)
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_VCCx3_PIN), GPIOC_bit_bit0_26(GPIO_VCCx3_PIN), 1);
    set_gpio_mode(GPIOC_bank_bit0_26(GPIO_VCCx3_PIN), GPIOC_bit_bit0_26(GPIO_VCCx3_PIN), GPIO_OUTPUT_MODE); 
    msleep(100);

    //set_mio_mux(2,1<<31);
}

void power_off_lcd(void)
{
    printk("power_off_lcd...\n");
    
    //LCD DATA

    //LCDR2~7
    //set gpio c 15~20 output 0
    WRITE_CBUS_REG_BITS(PREG_FGPIO_O,0,15,6);
    WRITE_CBUS_REG_BITS(PREG_FGPIO_EN_N,0,15,6);

    //LCDG2~5
    //set gpio c 23~26 output 0
    WRITE_CBUS_REG_BITS(PREG_FGPIO_O,0,23,4);
    WRITE_CBUS_REG_BITS(PREG_FGPIO_EN_N,0,23,4);

    //LCDG6~7 B2~7
    //set gpio d 0~1  output 0
    WRITE_CBUS_REG_BITS(PREG_GGPIO_O,0,0,2);
    WRITE_CBUS_REG_BITS(PREG_GGPIO_EN_N,0,0,2);
    //set gpio d 6~11  output 0
    WRITE_CBUS_REG_BITS(PREG_GGPIO_O,0,4,6);
    WRITE_CBUS_REG_BITS(PREG_GGPIO_EN_N,0,4,6);
		
    clear_mio_mux(4,(1<<4)|(1<<2)|(1<<0));

    //TCON

    //set gpio a 1,2,5
    WRITE_CBUS_REG_BITS(PREG_EGPIO_O,0,5,2);
    WRITE_CBUS_REG_BITS(PREG_EGPIO_EN_N,0,5,2);
    WRITE_CBUS_REG_BITS(PREG_EGPIO_O,0,9,1);
    WRITE_CBUS_REG_BITS(PREG_EGPIO_EN_N,0,9,1);

    clear_mio_mux(0, ((1<<11)|(1<<14)|(1<<15)/*|(1<<16)*/));  //TCON

    //PWM
    /*
    set_gpio_val(GPIOA_bank_bit0_14(7), GPIOA_bit_bit0_14(7), 0);
    set_gpio_mode(GPIOA_bank_bit0_14(7), GPIOA_bit_bit0_14(7), GPIO_OUTPUT_MODE);
    clear_mio_mux(2,1<<31);*/

    //POWER
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_VCCx3_PIN), GPIOC_bit_bit0_26(GPIO_VCCx3_PIN), 0);
    set_gpio_val(GPIOC_bank_bit0_26(GPIO_LCD_PWR_PIN), GPIOC_bit_bit0_26(GPIO_LCD_PWR_PIN), 1);
}


void aml_8726m_power_on_bl(void)
{
	printk("aml_8726m_power_on_bl\n");

	if(lcd_boot_up)
	    set_mio_mux(2,1<<31);
}

void aml_8726m_power_off_bl(void)
{
	printk("aml_8726m_power_off_bl\n");

    set_gpio_val(GPIOA_bank_bit0_14(7), GPIOA_bit_bit0_14(7), 0);
    set_gpio_mode(GPIOA_bank_bit0_14(7), GPIOA_bit_bit0_14(7), GPIO_OUTPUT_MODE);
    clear_mio_mux(2,1<<31);
}

struct aml_bl_platform_data aml_bl_platform =
{
    .bl_init = aml_8726m_bl_init,
    .power_on_bl = aml_8726m_power_on_bl,
    .power_off_bl = aml_8726m_power_off_bl,
    .get_bl_level = aml_8726m_get_bl_level,
    .set_bl_level = aml_8726m_set_bl_level,
};

static struct platform_device aml_bl_device = {
    .name = "aml-bl",
    .id = -1,
    .num_resources = 0,
    .resource = NULL,
    .dev = {
        .platform_data = &aml_bl_platform,
    },
};
#endif
#if  defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
static struct resource vout_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vout_device = {
    .name       = "mesonvout",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout_device_resources),
    .resource      = vout_device_resources,
};
#endif

#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns = 2,
       .vendor = "ZENITHINK",
       .product = "ZT280",
       .release = 0x0100,
};
static struct platform_device usb_mass_storage_device = {
       .name = "usb_mass_storage",
       .id = -1,
       .dev = {
               .platform_data = &mass_storage_pdata,
               },
};
#endif
static char *usb_functions[] = { "usb_mass_storage" };
static char *usb_functions_adb[] = { 
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
"usb_mass_storage", 
#endif

#ifdef CONFIG_USB_ANDROID_ADB
"adb" 
#endif
};
static struct android_usb_product usb_products[] = {
       {
               .product_id     = 0x0c01,
               .num_functions  = ARRAY_SIZE(usb_functions),
               .functions      = usb_functions,
       },
       {
               .product_id     = 0x0c02,
               .num_functions  = ARRAY_SIZE(usb_functions_adb),
               .functions      = usb_functions_adb,
       },
};

static struct android_usb_platform_data android_usb_pdata = {
       .vendor_id      = 0x0bb4,
       .product_id     = 0x0c01,
       .version        = 0x0100,
       .product_name   = "ZT280",
       .manufacturer_name = "ZENITHINK",
       .num_products = ARRAY_SIZE(usb_products),
       .products = usb_products,
       .num_functions = ARRAY_SIZE(usb_functions_adb),
       .functions = usb_functions_adb,
};

static struct platform_device android_usb_device = {
       .name   = "android_usb",
       .id             = -1,
       .dev            = {
               .platform_data = &android_usb_pdata,
       },
};
#endif

#ifdef CONFIG_BT_DEVICE
#include <linux/bt-device.h>

static struct platform_device bt_device = {
	.name             = "bt-dev",
	.id               = -1,
};
static void bt_device_init(void)
{
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<29));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<22));
	
	//CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<19));
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<20));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<17));
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<14));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
	
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<4));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<13));
	
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<12));
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<21));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<28));
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<23));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<14));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<17));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<5));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<27));
	
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<27));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<18));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<9));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<23));
	
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_12, (1<<26));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, (1<<17));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<17));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_0, (1<<12));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_5, (1<<8));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_8, (1<<24));
	
	/* WLBT_REGON */
	//CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<18));//D20
	//SET_CBUS_REG_MASK(PREG_GGPIO_O, (1<<18));
        #ifdef CONFIG_SN7325
        printk("power on 7325 5\n");
        configIO(0, 0);
        setIO_level(0, 1, 5);//OD5
        #endif
	
	/* reset */
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<12));//D14
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
	msleep(200);	
	SET_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
	
	/* BG/GPS low */
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<19));//D21
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<19));	
	
	/* UART RTS */
	/*
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<16));//D18
    CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<16));*/
		
	/* BG wakeup high 
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<14));
	SET_CBUS_REG_MASK(PREG_GGPIO_O, (1<<14));*/
}

static void bt_device_on(void)
{
    /* reset */
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<12));
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
	msleep(200);	
	SET_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
}

static void bt_device_off(void)
{
    /* reset */
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_EN_N, (1<<12));
	CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
	msleep(200);	
	//CLEAR_CBUS_REG_MASK(PREG_GGPIO_O, (1<<12));	
}

struct bt_dev_data bt_dev = {
    .bt_dev_init    = bt_device_init,
    .bt_dev_on      = bt_device_on,
    .bt_dev_off     = bt_device_off,
};
#endif

int get_gsensor_direction(void)
{
    int dir = 

#if defined(CONFIG_BOARD_G0)
    1
#elif defined(CONFIG_BOARD_H0)
    7
#elif defined(CONFIG_BOARD_H1)
    0
#elif defined(CONFIG_BOARD_K0)
    0
#else
    0
#endif
    ;

    return dir;
}

#if defined(CONFIG_UART_GPS)

#define GPIO_GPS_EN_BANK (GPIOE_bank_bit16_21(17))
#define GPIO_GPS_EN_BIT  (GPIOE_bit_bit16_21(17))

#define GPIO_GPS_RESET_BANK (GPIOE_bank_bit16_21(16))
#define GPIO_GPS_RESET_BIT  (GPIOE_bit_bit16_21(16))

/*
#   if defined(CONFIG_GPS_POWER_CONTROL)
#define GPS_EN_VAL  0
#   else
#define GPS_EN_VAL  1
#   endif
*/
#define GPS_EN_VAL  0

#define GPS_RESET_VAL 0

static int gps_switch_on =  // gps key control
#if defined(CONFIG_GPS_SWITCH_CONTROL)
    0
#else
    1
#endif
; 

static int is_gps_switch_on(){
    return gps_switch_on;
}
static void set_gps_swtich(int on){
    printk("set_gps_swtich %d\n",on);
    gps_switch_on = on;
}

extern void gps_device_on(int on);

static void gps_key_callback(int code,int pressed)
{
    if(pressed){
        set_gps_swtich(1);
        gps_device_on(1);
    }else{
        set_gps_swtich(0);
        gps_device_on(0);
    }
}

static void gps_pinmux_init(void)
{
    /*
    clear_mio_mux(5, (1<<25)|(1<<26)|(1<<27)|(1<<28));
    clear_mio_mux(7, (1<<22)|(1<<23));*/

    clear_mio_mux(6, (1<<13)|(1<<14));  //mux nandflash ce3/ce4

    //gps on/off pin
    set_gpio_val(GPIO_GPS_EN_BANK, GPIO_GPS_EN_BIT, !GPS_EN_VAL);
    set_gpio_mode(GPIO_GPS_EN_BANK, GPIO_GPS_EN_BIT, GPIO_OUTPUT_MODE);

    //gps reset pin
    set_gpio_val(GPIO_GPS_RESET_BANK, GPIO_GPS_RESET_BIT, GPS_RESET_VAL);
    set_gpio_mode(GPIO_GPS_RESET_BANK, GPIO_GPS_RESET_BIT, GPIO_OUTPUT_MODE);
}

void gps_device_on(int on)
{
    if(on && is_gps_switch_on()){
        //gps on/off pin
        set_gpio_val(GPIO_GPS_EN_BANK, GPIO_GPS_EN_BIT, GPS_EN_VAL);
        msleep(10);
        //gps reset pin
        set_gpio_val(GPIO_GPS_RESET_BANK, GPIO_GPS_RESET_BIT, !GPS_RESET_VAL);
    }else{
        //gps reset pin
        set_gpio_val(GPIO_GPS_RESET_BANK, GPIO_GPS_RESET_BIT, GPS_RESET_VAL);
        //gps on/off pin
        set_gpio_val(GPIO_GPS_EN_BANK, GPIO_GPS_EN_BIT, !GPS_EN_VAL);
    }
}

void gps_device_reset(void)
{
    gps_device_on(0);
}

#endif

#ifdef CONFIG_POST_PROCESS_MANAGER
static struct resource ppmgr_resources[] = {
    [0] = {
        .start =  PPMGR_ADDR_START,
        .end   = PPMGR_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};
static struct platform_device ppmgr_device = {
    .name       = "ppmgr",
    .id         = 0,
    .num_resources = ARRAY_SIZE(ppmgr_resources),
    .resource      = ppmgr_resources,
};
#endif


static struct platform_device __initdata *platform_devs[] = {

#ifdef CONFIG_EFUSE
	&aml_efuse_device,
#endif 

    #if defined(CONFIG_I2C_SW_AML)
        &aml_sw_i2c_device,
    #endif
    #if defined(CONFIG_I2C_AML)
        &aml_i2c_device,
        &aml_i2c_ext_device,
    #endif
	
    #if defined(CONFIG_JPEGLOGO)
        &jpeglogo_device,
    #endif
    
    #if defined (CONFIG_AMLOGIC_PM)
        &power_dev,
    #endif  
    #if defined(CONFIG_FB_AM)
        &fb_device,
    #endif
    #if defined(CONFIG_AM_STREAMING)
        &codec_device,
    #endif
    #if defined(CONFIG_AM_VIDEO)
        &deinterlace_device,
    #endif
    #if defined(CONFIG_TVIN_VDIN)
        &vdin_device,
    #endif
    #if defined(CONFIG_TVIN_BT656IN)
		&bt656in_device,
    #endif
    #if defined(CONFIG_AML_AUDIO_DSP)
        &audiodsp_device,
    #endif
    #if defined(CONFIG_SND_AML_M1_MID_WM8988)||defined(CONFIG_SND_AML_M1_MID_ES8388)
        &aml_audio,
    #endif
    #if defined(CONFIG_KEYPADS_AM)||defined(CONFIG_VIRTUAL_REMOTE)||defined(CONFIG_KEYPADS_AM_MODULE)
        &input_device,
    #endif
    #ifdef CONFIG_SARADC_AM
    &saradc_device,
    #endif
	#if defined(CONFIG_AM_ETHERNET)
    &am_net8218_device,
	#endif
    #ifdef CONFIG_ADC_TOUCHSCREEN_AM
        &adc_ts_device,
    #endif
    #if defined(CONFIG_ADC_KEYPADS_AM)||defined(CONFIG_ADC_KEYPADS_AM_MODULE)
        &adc_kp_device,
    #endif
    #if defined(CONFIG_KEY_INPUT_CUSTOM_AM) || defined(CONFIG_KEY_INPUT_CUSTOM_AM_MODULE)
        &input_device_key,  //changed by Elvis
    #endif
    #if defined(CONFIG_TOUCHSCREEN_ADS7846)
        &spi_gpio,
    #endif
    #ifdef CONFIG_AM_NAND
        &aml_nand_device,
    #endif
    #if defined(CONFIG_NAND_FLASH_DRIVER_MULTIPLANE_CE)
        &aml_nand_device,
    #endif
    #if defined(CONFIG_AML_RTC)
        &aml_rtc_device,
    #endif
    #ifdef CONFIG_AMLOGIC_VIDEOIN_MANAGER
		&vm_device,
    #endif
    #if defined(CONFIG_SUSPEND)
        &aml_pm_device,
    #endif
    #if defined(CONFIG_ANDROID_PMEM)
        &android_pmem_device,
    #endif

    #if defined(CONFIG_CARDREADER)
        &amlogic_card_device,
    #endif    
    #if defined(CONFIG_AM_UART_WITH_S_CORE)
        &aml_uart_device,
    #endif
    #if defined(CONFIG_AMLOGIC_BACKLIGHT)
        &aml_bl_device,
    #endif
    #if defined(CONFIG_AM_TV_OUTPUT)||defined(CONFIG_AM_TCON_OUTPUT)
        &vout_device,   
    #endif
    #ifdef CONFIG_USB_ANDROID
        &android_usb_device,
        #ifdef CONFIG_USB_ANDROID_MASS_STORAGE
            &usb_mass_storage_device,
        #endif
    #endif	
    #ifdef CONFIG_BT_DEVICE  
        &bt_device,
    #endif    
    #ifdef CONFIG_POST_PROCESS_MANAGER
    	&ppmgr_device,
    #endif 

      	
};
static struct i2c_board_info __initdata aml_i2c_bus_info[] = {

#ifdef CONFIG_ZT_ENCRYPT
    {
        I2C_BOARD_INFO("rt_encrypt",  0x51),
    },
#endif

#if !defined(CONFIG_TOUCHSCREEN_INDEPENDENT_I2C) 
#ifdef CONFIG_TOUCHSCREEN_NAS_MUTIL
    {
        I2C_BOARD_INFO("nastech-ts",  0x5c),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,        
    },
#endif
#if defined(CONFIG_UOR6X5X_RESISTIVE_TOUCHSCREEN)
    {
        I2C_BOARD_INFO("uor6x5x", 0x48),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif
#ifdef CONFIG_ITK_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("itk", 0x41),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif

#ifdef CONFIG_UOR7X5X_RESISTIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("uor7x5x", 0x48),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif

#endif

#if defined(CONFIG_SENSORS_AAC)
    {
        I2C_BOARD_INFO("mxc622x",  0x15),
    },

    {
        I2C_BOARD_INFO("mma7660",  0x4c),
    },
#endif
#if defined(CONFIG_SENSORS_ECOMPASS)
    {
        I2C_BOARD_INFO("mmc31xx",  0x30),
    },
#endif
#ifdef CONFIG_SND_AML_M1_MID_WM8988
	{
		I2C_BOARD_INFO("wm8988", 0x1A),
		.platform_data = (void *)&audio_pdata,
	},
#endif

#ifdef CONFIG_SND_AML_M1_MID_ES8388
	{
		I2C_BOARD_INFO("es8388", 0x10),
		.platform_data = (void *)&audio_pdata,
	},
#endif


#ifdef CONFIG_SN7325
    {
        I2C_BOARD_INFO("sn7325", 0x59),
        .platform_data = (void *)&sn7325_pdata,
    },
#endif

#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GC0308
	{
        /*gc0308 i2c address is 0x42/0x43*/
		I2C_BOARD_INFO("gc0308_i2c",  0x21),
		.platform_data = (void *)&video_gc0308_data,
	},
#endif
#ifdef CONFIG_VIDEO_AMLOGIC_CAPTURE_GT2005
    {
    	//gt2005 i2c address is 0x78/0x79
    	I2C_BOARD_INFO("gt2005_i2c",  0x3c ),
    	.platform_data = (void *)&video_gt2005_data
    },
#endif

};

static struct i2c_board_info __initdata aml_i2c_ext_bus_info[] = {
#if defined(CONFIG_TOUCHSCREEN_INDEPENDENT_I2C)
#if defined(CONFIG_UOR6X5X_RESISTIVE_TOUCHSCREEN)
    {
        I2C_BOARD_INFO("uor6x5x", 0x48),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif
#if defined(CONFIG_FT5301_TOUCHSCREEN)
    {
    	I2C_BOARD_INFO("ft5301_ts", 0x38),
    	.irq = GPIO_TOUCH_INT_GRUP,
    	.platform_data = (void *)&touch_pdata,
    },
#endif
#if defined(CONFIG_NOVATEK_TOUCHSCREEN)
    {
    	I2C_BOARD_INFO("novatek_ts", 
#if defined(CONFIG_IMAGE_RECOVERY)        
            0x7f
#else
            0x1
#endif
    	),
    	.irq = GPIO_TOUCH_INT_GRUP,
    	.platform_data = (void *)&touch_pdata,
    },
#endif

#ifdef CONFIG_TOUCHSCREEN_NAS_MUTIL
    {
        I2C_BOARD_INFO("nastech-ts",  0x5c),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,        
    },
#endif

#ifdef CONFIG_ILITEK_CAPACITIVE_TOUCHSCREEN
    {
        I2C_BOARD_INFO("ilitek_i2c",  0x41),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,        
    },
#endif
#if defined(CONFIG_PIXCIR_CAPACITIVE_TOUCHSCREEN)
    {
    	
        I2C_BOARD_INFO("pixcir_ts", 0x5c),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif
#if defined(CONFIG_PIXCIR_C44_CAPACITIVE_TOUCHSCREEN)
    {
    	
        I2C_BOARD_INFO("pixcir_c44_ts", 0x5c),
        .irq = GPIO_TOUCH_INT_GRUP,
        .platform_data = (void *)&touch_pdata,
    },
#endif

#endif
};

static struct i2c_board_info __initdata aml_i2c_ext_bus_slow_info[] = {

#if defined(CONFIG_TOUCHSCREEN_INDEPENDENT_I2C)
/*
#if defined(CONFIG_FT5301_TOUCHSCREEN)
    {
    	I2C_BOARD_INFO("ft5301_ts", 0x38),
    	.irq = GPIO_TOUCH_INT_GRUP,
    	.platform_data = (void *)&touch_pdata,
    },
#endif
*/

#endif
};


static int __init aml_i2c_init(void)
{
    aml_i2c_pre_init();

    i2c_register_board_info(0, aml_i2c_bus_info,
        ARRAY_SIZE(aml_i2c_bus_info));

    i2c_register_board_info(2, aml_i2c_ext_bus_info,
        ARRAY_SIZE(aml_i2c_ext_bus_info));

    i2c_register_board_info(3, aml_i2c_ext_bus_slow_info,
        ARRAY_SIZE(aml_i2c_ext_bus_slow_info));

    return 0;
}

#if defined(CONFIG_TVIN_BT656IN)
static void __init bt656in_pinmux_init(void)
{
    set_mio_mux(3, 0xf000);   //mask--mux gpio_c3 to bt656 clk;  mux gpioc[4:11] to be bt656 dt_in
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_2, 0x0f000000);
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_3, 0x01be07fc);
    CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_4, 0x0c000000);
}


#endif
void  eth_pinmux_init(void)
{
	printk("eth pinmux init\n");

	eth_clk_set(ETH_CLKSRC_APLL_CLK,400*CLK_1M,50*CLK_1M);
	eth_set_pinmux(ETH_BANK2_GPIOD15_D23,ETH_CLK_OUT_GPIOD24_REG5_1,0);
	//CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, 3<<18);
	CLEAR_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
	SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, (1 << 1));
	SET_CBUS_REG_MASK(PREG_ETHERNET_ADDR0, 1);
	udelay(100);

	/*reset*/
	set_gpio_val(GPIOD_bank_bit2_24(12), GPIOD_bit_bit2_24(12), 0); 
	set_gpio_mode(GPIOD_bank_bit2_24(12), GPIOD_bit_bit2_24(12), GPIO_OUTPUT_MODE); 
    mdelay(100);  
    set_gpio_val(GPIOD_bank_bit2_24(12), GPIOD_bit_bit2_24(12), 1); 
    mdelay(10);	//waiting reset end;
}

static void inline  nand_pinmux_init(void)
{
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_6, 0x7fff); //disable all nand pin
	SET_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, ((1<<30) | (1<<28) | (1<<26) | (1<<24)));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_1, ((1<<29) | (1<<27) | (1<<25) | (1<<23)));
	CLEAR_CBUS_REG_MASK(PERIPHS_PIN_MUX_7, ((1<<29) | (1<<28) | (1<<27) | (1<<26) | (1<<25) | (1<<24)));

	SET_CBUS_REG_MASK(PREG_HGPIO_EN_N, 0x1fffff);  //all pin as input
}

static void __init device_pinmux_init(void )
{

  //clearall_pinmux();
    
    /*other deivce power on*/
    /*GPIOA_200e_bit4..usb/eth/YUV power on*/
    uart_set_pinmux(UART_PORT_A,PINMUX_UART_A);
    uart_set_pinmux(UART_PORT_B,PINMUX_UART_B);
    /*pinmux of eth*/
    eth_pinmux_init();
    aml_i2c_init();
#if defined(CONFIG_TVIN_BT656IN)
    bt656in_pinmux_init();
#endif
    set_audio_pinmux(AUDIO_OUT_TEST_N);
    set_audio_pinmux(AUDIO_IN_JTAG);
    //set clk for wifi
    //nand pin
    nand_pinmux_init();

    //GPS HW control pin
#if defined(CONFIG_UART_GPS)
    gps_pinmux_init();
#endif

    touch_init();

#if defined(CONFIG_VIDEO_AMLOGIC_CAPTURE_CLK_PWM)
    aml_8726m_pwmb_init();
#endif

    
	printk("device pinmux init\n");

}

static void __init  device_clk_setting(void)
{
    /*Demod CLK for eth and sata*/
    demod_apll_setting(0,1200*CLK_1M);
    /*eth clk*/
    //eth_clk_set(ETH_CLKSRC_APLL_CLK,400*CLK_1M,50*CLK_1M);
	
    printk("device_clk_setting\r\n");
    //msleep(10000);
}

static void disable_unused_model(void)
{
    CLK_GATE_OFF(VIDEO_IN);
    CLK_GATE_OFF(BT656_IN);
    CLK_GATE_OFF(ETHERNET);
    CLK_GATE_OFF(SATA);
    CLK_GATE_OFF(WIFI);
    video_dac_disable();
    //audio_internal_dac_disable();
     //disable wifi
    SET_CBUS_REG_MASK(HHI_GCLK_MPEG2, (1<<5)); 
    SET_CBUS_REG_MASK(HHI_WIFI_CLK_CNTL, (1<<0));
    __raw_writel(0xCFF,0xC9320ED8);
    __raw_writel((__raw_readl(0xC9320EF0))&0xF9FFFFFF,0xC9320EF0);
    CLEAR_CBUS_REG_MASK(HHI_GCLK_MPEG2, (1<<5)); 
    CLEAR_CBUS_REG_MASK(HHI_WIFI_CLK_CNTL, (1<<0));
    ///disable demod
    SET_CBUS_REG_MASK(HHI_DEMOD_CLK_CNTL, (1<<8));//enable demod core digital clock
    SET_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL, (1<<15));//enable demod adc clock
    CLEAR_APB_REG_MASK(0x4004,(1<<31));  //disable analog demod adc
    CLEAR_CBUS_REG_MASK(HHI_DEMOD_PLL_CNTL, (1<<15));//disable demod adc clock  
    CLEAR_CBUS_REG_MASK(HHI_DEMOD_CLK_CNTL, (1<<8));//disable demod core digital clock
}
static void __init power_hold(void)
{
    printk(KERN_INFO "power hold set high!\n");
    
        /* PIN28, GPIOA_6, Pull high, For En_5V */
    set_gpio_val(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), 1);
	set_gpio_mode(GPIOA_bank_bit(6), GPIOA_bit_bit0_14(6), GPIO_OUTPUT_MODE);

//----------------------------------------------------------------------------------
//**** next add by snowwan for USB *************************************************
//----------------------------------------------------------------------------------


    //touch power en: temp put here
	/*set_gpio_val(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), 1); 
	set_gpio_mode(GPIOD_bank_bit2_24(5), GPIOD_bit_bit2_24(5), GPIO_OUTPUT_MODE); */
//----------------------------------------------------------------------------------
//**********************************************************************************
//----------------------------------------------------------------------------------
};

static __init void m1_init_machine(void)
{

    meson_cache_init();

    power_hold();
    device_clk_setting();
    device_pinmux_init();

    platform_add_devices(platform_devs, ARRAY_SIZE(platform_devs));

#ifdef CONFIG_USB_DWC_OTG_HCD
    set_usb_phy_clk(USB_PHY_CLOCK_SEL_XTAL_DIV2);
    lm_device_register(&usb_ld_a);
	lm_device_register(&usb_ld_b);
#endif
#ifdef CONFIG_SATA_DWC_AHCI
    set_sata_phy_clk(SATA_PHY_CLOCK_SEL_DEMOD_PLL);
    lm_device_register(&sata_ld);
#endif
#if defined(CONFIG_TOUCHSCREEN_ADS7846)
    ads7846_init_gpio();
    spi_register_board_info(spi_board_info_list, ARRAY_SIZE(spi_board_info_list));
#endif
    disable_unused_model();
    aml_8726m_power_off_bl();
//------------------------------------------------------------------------------
    pm_power_off = set_bat_off;

}

/*VIDEO MEMORY MAPING*/
static __initdata struct map_desc meson_video_mem_desc[] = {
    {
        .virtual    = PAGE_ALIGN(__phys_to_virt(RESERVED_MEM_START)),
        .pfn        = __phys_to_pfn(RESERVED_MEM_START),
        .length     = RESERVED_MEM_END-RESERVED_MEM_START+1,
        .type       = MT_DEVICE,
    },
};

static __init void m1_map_io(void)
{
    meson_map_io();
    iotable_init(meson_video_mem_desc, ARRAY_SIZE(meson_video_mem_desc));
}

static __init void m1_irq_init(void)
{
    meson_init_irq();
}

void amlogic_reset(void)
{
	WRITE_MPEG_REG(VENC_VDAC_SETTING, 0xf);
	WRITE_MPEG_REG(WATCHDOG_RESET, 0);
	WRITE_MPEG_REG(WATCHDOG_TC, 1 << WATCHDOG_ENABLE_BIT | 100);
}
EXPORT_SYMBOL_GPL(amlogic_reset);

static __init void m1_fixup(struct machine_desc *mach, struct tag *tag, char **cmdline, struct meminfo *m)
{
    struct membank *pbank;
    m->nr_banks = 0;
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(PHYS_MEM_START);
    pbank->size  = SZ_64M & PAGE_MASK;
    pbank->node  = PHYS_TO_NID(PHYS_MEM_START);
    m->nr_banks++;
    pbank=&m->bank[m->nr_banks];
    pbank->start = PAGE_ALIGN(RESERVED_MEM_END+1);
    pbank->size  = (PHYS_MEM_END-RESERVED_MEM_END) & PAGE_MASK;
    pbank->node  = PHYS_TO_NID(RESERVED_MEM_END+1);
    m->nr_banks++;
}

MACHINE_START(MESON_8726M, "AMLOGIC MESON-M1 8726M SZ")
    .phys_io        = MESON_PERIPHS1_PHYS_BASE,
    .io_pg_offst    = (MESON_PERIPHS1_PHYS_BASE >> 18) & 0xfffc,
    .boot_params    = BOOT_PARAMS_OFFSET,
    .map_io         = m1_map_io,
    .init_irq       = m1_irq_init,
    .timer          = &meson_sys_timer,
    .init_machine   = m1_init_machine,
    .fixup          = m1_fixup,
    .video_start    = RESERVED_MEM_START,
    .video_end      = RESERVED_MEM_END,
MACHINE_END
