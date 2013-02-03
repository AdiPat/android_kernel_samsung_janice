/*
 *  Copyright (C) 2013, Aditya Patange "Adi_Pat" <adithemagnificent@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Sound Control for AB8500. 
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <sound/soc.h>
#include "ab8500_audio.h"
#include <sound/ab8500_sound_control.h>

#define DRIVER_NAME                        "AB8500 Sound boost control"
#define SOUND_BOOST_CONTROL_VERSION             2
/* Boost Audio */ 
#define REG_ANAGAIN3_BOOST                      0
#define REG_ANAGAIN3_DEFAULT                    68
/* Toggle */ 
static int sound_boost_status = 0;
extern int ab850x_audio_power_control(bool power_on);

/* Check whether headset is plugged in */ 
extern int jack_is_detected;

static struct snd_soc_codec *sound_boost_codec;

int sound_boost_register(struct snd_soc_codec *codec)
{
	printk("%s\n",__func__);
	sound_boost_codec = codec;
	if(sound_boost_codec == NULL)
		printk(KERN_ERR "[ab8500-Sound-Control] Failed to register ab8500 codec!\n");
	return 0;
}

/* Check status of headset jack */

int get_jack_status(void)
{
	return jack_is_detected;
}

static ssize_t jack_status_read(struct kobject *kobj, struct kobject_attribute *attr,
				char *buf)
{
	int status = get_jack_status();
	return sprintf(buf, "%d\n", status);
}

static ssize_t sound_boost_control_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n",sound_boost_status);		
}

static ssize_t sound_boost_control_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d" ,&sound_boost_status);
	if (sound_boost_status == 0)
	{
		printk(KERN_DEBUG "[ab8500-Sound-Control] Sound boost disabled\n");
		snd_soc_write(sound_boost_codec, REG_ANAGAIN3, REG_ANAGAIN3_DEFAULT);
	}
	if (sound_boost_status == 1)
	{
		/* Power on the ab8500 SoC */ 
		ab850x_audio_power_control(true);		
		printk(KERN_DEBUG "[ab8500-Sound-Control] Sound boost enabled\n");
		snd_soc_write(sound_boost_codec, REG_ANAGAIN3, REG_ANAGAIN3_BOOST);
	}
	else
	{
		printk(KERN_DEBUG "[ab8500-Sound-Control] Invalid Value, 1 to enable, 0 to disable\n");
	}
	return count;
}

static ssize_t sound_boost_control_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",SOUND_BOOST_CONTROL_VERSION);
}

static struct kobj_attribute sound_boost_control_interface   = __ATTR(sound_boost, 0666, sound_boost_control_show, sound_boost_control_store);
static struct kobj_attribute sound_boost_control_version     = __ATTR(version, 0666, sound_boost_control_version_show, NULL);
static struct kobj_attribute jack_status                     = __ATTR(jack_status, 0666, jack_status_read, NULL);

static struct attribute *snd_boost_control_attrs[] = {
	&sound_boost_control_interface.attr,
	&sound_boost_control_version.attr, 
	&jack_status.attr, 
	NULL,
};

static struct attribute_group snd_interface_group = {
	.attrs = snd_boost_control_attrs,
};

static struct kobject *snd_kobject;

static int __init snd_control_init(void)
{
	int ret;
	printk(KERN_DEBUG "[%s]\n",__func__);
	
	snd_kobject = kobject_create_and_add("sound_control", kernel_kobj);

	if(!snd_kobject) 
		return -ENOMEM;

	ret = sysfs_create_group(snd_kobject, &snd_interface_group);
	
	if(ret)
		kobject_put(snd_kobject);

	return ret;
}
 
static void __exit snd_control_exit(void)
{
	kobject_put(snd_kobject);
}

module_init(snd_control_init);
module_exit(snd_control_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aditya Patange");
