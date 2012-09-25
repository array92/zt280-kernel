#ifndef __LINUX_ADC_KEYPAD_H
#define __LINUX_ADC_KEYPAD_H

struct adc_key{
	int code;	/* input key code */
	unsigned char *name;
	int chan;
	int value;	/* voltage/3.3v * 1023 */
	int tolerance;
};

struct fn_key{
	int code;	    /* code original */
	int fn_code;    /* code when fn pressed */
	unsigned char *name;
	void (*fn_callback)(int code,int pressed);
};

struct gpio_key{
	int code;	/* input key code */
	unsigned char *name;
    unsigned int bank;
    unsigned int bit;
    unsigned int checkdata;
};

struct key_list{
	struct adc_key *key;
	int key_num;

#if defined(CONFIG_ADC_KEYPADS_GPIO_SUPPORT)    
    struct gpio_key *gpio_key;
    int gpio_key_num;
#endif

#if defined(CONFIG_ADC_KEYPADS_FN_SUPPORT)    
	struct fn_key *fn_key;
	int fn_key_num;

    int (*fn_init)(void);
    int (*fn_pressed)(void);  //return 0 pressed
    int (*fn_convert)(const struct fn_key *fn_key,int fn_key_num,int code,int pressed,int fn);
#endif
};


struct adc_kp_platform_data{
	int (*led_control)(void *param);
	int led_control_param_num;

	const struct key_list *list;
};

#endif
