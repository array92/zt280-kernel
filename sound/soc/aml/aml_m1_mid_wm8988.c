#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>


#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "aml_dai.h"
#include "aml_pcm.h"

#include <sound/audio_common.h>
#include "../codecs/wm8988.h"

struct aml_m1_mid_data{
    struct device *dev;
    struct audio_platform_data *pdata;
    struct snd_soc_codec* codec;
    struct platform_device *platform_soc_audio;
    
    struct workqueue_struct *work_queue;
    struct delayed_work work;
    int hp_level;
};

static struct aml_m1_mid_data *m1_data;
extern int audio_dai_type;


static int aml_m1_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;

	int ret;


	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
	if(ret<0){
        printk("snd_soc_dai_set_fmt failed ret %d\n",ret);
		return ret;
    }
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_NB_NF|SND_SOC_DAIFMT_CBS_CFS);
	if(ret<0){
        printk("snd_soc_dai_set_fmt failed ret %d\n",ret);
		return ret;
    }
	return 0;
}
    
static struct snd_soc_ops aml_m1_ops = {
    .hw_params = aml_m1_hw_params,
};

static int aml_m1_set_bias_level(struct snd_soc_card *card,
	enum snd_soc_bias_level level)
{
	int ret = 0;
	//struct snd_soc_codec *codec = card->codec;


	switch (level) 
	{
		case SND_SOC_BIAS_ON:
		case SND_SOC_BIAS_PREPARE:
			break;

		case SND_SOC_BIAS_OFF:
		case SND_SOC_BIAS_STANDBY:
			break;
	};

    return ret;
}

static const struct snd_soc_dapm_widget aml_m1_dapm_widgets[] = {
    SND_SOC_DAPM_HP("Headphone Jack", NULL),
    SND_SOC_DAPM_MIC("Mic Jack", NULL),
    SND_SOC_DAPM_SPK("Ext Spk", NULL),
    SND_SOC_DAPM_LINE("Line Jack", NULL),
};

/* tcc machine audio map (connections to the codec pins) */
static const struct snd_soc_dapm_route intercon[] = {

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"LINPUT2", NULL, "Mic Jack"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "Left Out 1"},
	{"Headphone Jack", NULL, "Right Out 1"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "Left Out 2"},
	{"Ext Spk", NULL, "Right Out 2"},

};


/* Hook switch */

static struct snd_soc_jack hp_jack;

static struct snd_soc_jack_pin hp_jack_pins[] = {
    { .pin = "Headset Jack", .mask = SND_JACK_HEADSET },
};


static void aml_m1_hp_detect_queue(struct work_struct* work)
{    
    struct aml_m1_mid_data *au_data = container_of(work,struct aml_m1_mid_data, work.work);
    struct audio_platform_data *pdata = au_data->pdata;
    struct snd_soc_codec* codec = au_data->codec;

    int level;

    if(pdata->is_hp_pluged)
        level = (*pdata->is_hp_pluged)();
    else
        level = 0;

    if(level != au_data->hp_level){
        au_data->hp_level = level;
        if(level == 0x0){
            printk("Headphone pluged in\n");
            snd_soc_dapm_disable_pin(codec, "Ext Spk");
            snd_soc_dapm_enable_pin(codec, "Headphone Jack");
            snd_soc_dapm_sync(codec);

            msleep(100);

            if(pdata->amp_power_control)
                (*pdata->amp_power_control)(0);
        }else if(level == 0x1){
            printk("Speaker on\n");
            if(pdata->amp_power_control)
                (*pdata->amp_power_control)(1);

            msleep(100);
            
            snd_soc_dapm_enable_pin(codec, "Ext Spk");
            snd_soc_dapm_disable_pin(codec, "Headphone Jack");
            snd_soc_dapm_sync(codec);
        }else{
            printk("Sound disable\n");
            snd_soc_dapm_disable_pin(codec, "Ext Spk");
            snd_soc_dapm_disable_pin(codec, "Headphone Jack");
            snd_soc_dapm_sync(codec);

            msleep(100);

            if(pdata->amp_power_control)
                (*pdata->amp_power_control)(0);            
        }
    }

    queue_delayed_work(au_data->work_queue, &au_data->work, HZ*2);
}

static int aml_m1_codec_init(struct snd_soc_codec *codec)
{
	struct snd_soc_card *card = codec->socdev->card;

	int err;
    //Add board specific DAPM widgets and routes

	
	err = snd_soc_dapm_new_controls(codec, aml_m1_dapm_widgets, ARRAY_SIZE(aml_m1_dapm_widgets));
	if(err)	{
		dev_warn(card->dev, "Failed to register DAPM widgets\n");
		return err;
	}

	err = snd_soc_dapm_add_routes(codec, intercon,
		ARRAY_SIZE(intercon));
	if(err)	{
		dev_warn(card->dev, "Failed to setup dapm widgets routine\n");
		return err;
	}

	err = snd_soc_jack_new(card, "hp_switch",
		SND_JACK_HEADSET, &hp_jack);
    if(err){
        dev_warn(card->dev, "Failed to alloc resource for hook switch\n");
    }else{
        err = snd_soc_jack_add_pins(&hp_jack, ARRAY_SIZE(hp_jack_pins), hp_jack_pins);
        if(err){
            dev_warn(card->dev, "Failed to setup hook hp jack pin\n");
        }
    }

    m1_data->codec = codec;

    snd_soc_dapm_nc_pin(codec,"LINPUT1");
    snd_soc_dapm_nc_pin(codec,"RINPUT1");

    snd_soc_dapm_disable_pin(codec, "Ext Spk");
    snd_soc_dapm_disable_pin(codec, "Headphone Jack");
    snd_soc_dapm_enable_pin(codec, "Mic Jack");    

    snd_soc_dapm_sync(codec);

    queue_delayed_work(m1_data->work_queue, &m1_data->work, HZ*2);


    return 0;
}


static struct snd_soc_dai_link aml_m1_dai = {

    .name = "AML-M1-WM8988",
    .stream_name = "AML M1 PCM WM8988",
    .cpu_dai = &aml_dai[0],  //////
    .codec_dai = &wm8988_dai,
    .init = aml_m1_codec_init,
    .ops = &aml_m1_ops,
};

static struct snd_soc_card snd_soc_aml_m1 = {
    .name = "AML-M1-WM8988",
    .platform = &aml_soc_platform,
    .dai_link = &aml_m1_dai,
    .num_links = 1,
    .set_bias_level = aml_m1_set_bias_level,
};

static struct snd_soc_device aml_m1_snd_devdata = {
    .card = &snd_soc_aml_m1,
    .codec_dev = &soc_codec_dev_wm8988,
};

#ifdef CONFIG_ZT_ENCRYPT
extern int rt_jia_usb;
extern int rt_jia_sd;
#endif

static int aml_m1_audio_probe(struct platform_device *pdev)
{
    struct aml_m1_mid_data *au_data;
    struct platform_device *platform_soc_audio = NULL;
    int ret = 0;


    

#ifdef CONFIG_ZT_ENCRYPT	
	if(99 != rt_jia_usb)
		return 0;
#endif    

	au_data = kmalloc(sizeof(struct aml_m1_mid_data), GFP_KERNEL);
	if (au_data == NULL) {
	    printk(KERN_ERR "ASoC: au code allocation failed\n");
		ret = -ENOMEM;
		goto error;
	}
    platform_set_drvdata(pdev,au_data);
    au_data->dev = &pdev->dev;
    au_data->pdata = pdev->dev.platform_data;
    au_data->hp_level = -1;

	au_data->work_queue = create_singlethread_workqueue("aml_m1_mid_es8388");
	if(au_data->work_queue == NULL){
        printk(KERN_ERR "Failed allocation queue\n");
		goto error;
	}
	INIT_DELAYED_WORK(&au_data->work, aml_m1_hp_detect_queue);
    m1_data = au_data;

    platform_soc_audio = platform_device_alloc("soc-audio", -1);
    if (!platform_soc_audio) {
        printk(KERN_ERR "ASoC: Platform device allocation failed\n");
        ret = -ENOMEM;
        goto error;
    }
    
    platform_set_drvdata(platform_soc_audio,&aml_m1_snd_devdata);
    aml_m1_snd_devdata.dev = &platform_soc_audio->dev;
    au_data->platform_soc_audio = platform_soc_audio;

    ret = platform_device_add(platform_soc_audio);
    if (ret) {
        printk(KERN_ERR "ASoC: Platform device allocation failed\n");
        goto error;
    }

    return 0;
error:
    if(platform_soc_audio)
        platform_device_put(platform_soc_audio);
    return ret;
}

static int aml_m1_audio_remove(struct platform_device *pdev)
{
    struct aml_m1_mid_data *au_data = (struct aml_m1_mid_data *)platform_get_drvdata(pdev);
    
    printk("***Entered %s:%s\n", __FILE__,__func__);

    if(au_data){
        if(au_data->platform_soc_audio)
            platform_device_put(au_data->platform_soc_audio);
        if(au_data->work_queue)
            destroy_workqueue(au_data->work_queue);

        kfree(au_data);
    }

    m1_data = NULL;

    return 0;
}

#ifdef CONFIG_PM

static int aml_m1_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct aml_m1_mid_data *au_data = (struct aml_m1_mid_data *)platform_get_drvdata(pdev);
    struct audio_platform_data *pdata = au_data->pdata;
    struct snd_soc_codec* codec = au_data->codec;

    printk("aml_m1_suspend\n");

    snd_soc_dapm_disable_pin(codec, "Ext Spk");
    snd_soc_dapm_disable_pin(codec, "Headset Jack");
    snd_soc_dapm_sync(codec);

    au_data->hp_level = -1;
    
    if(pdata->amp_power_control)
        (*pdata->amp_power_control)(0);

	return 0;
}

static int aml_m1_resume(struct platform_device *pdev)
{
    struct aml_m1_mid_data *au_data = (struct aml_m1_mid_data *)platform_get_drvdata(pdev);
    printk("aml_m1_resume\n");

    queue_delayed_work(au_data->work_queue, &au_data->work, HZ*2);
    return 0;
}

#endif /* CONFIG_PM */

static struct platform_driver aml_m1_audio_driver = {
    .probe  = aml_m1_audio_probe,
    .remove = aml_m1_audio_remove,
    .driver = {
        .name = "aml_m1_codec",
        .owner = THIS_MODULE,
    },
    .suspend = aml_m1_suspend,
    .resume = aml_m1_resume,
    
};

static int __init aml_m1_wm8988_init(void)
{
	printk("aml_m1_wm8988_init\r\n");
	if(audio_dai_type != 1)
		return 0;	
    return platform_driver_register(&aml_m1_audio_driver);
}

static void __exit aml_m1_wm8988_exit(void)
{
    platform_driver_unregister(&aml_m1_audio_driver);
}

module_init(aml_m1_wm8988_init);
module_exit(aml_m1_wm8988_exit);

/* Module information */
MODULE_AUTHOR("ZeniThink, Inc.");
MODULE_DESCRIPTION("ALSA SoC ZNT AUDIO");
MODULE_LICENSE("GPL");


