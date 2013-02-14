/*
 *  Copyright (C) 2013, 
 * 
 *  Author: Aditya Patange "Adi_Pat" <adithemagnificent@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Touch Boost Interface. 
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/mfd/dbx500-prcmu.h>
#include <linux/input/touchboost.h>

#define TOUCHBOOST_VERSION     1
#define TOUCHBOOST_DEBUG       1

/* Supported Frequencies to boost in Mhz */
#define TOUCHBOOST_FREQUENCY1  200
#define TOUCHBOOST_FREQUENCY2  400
#define TOUCHBOOST_FREQUENCY3  800
#define TOUCHBOOST_FREQUENCY4  1000
/* Max time out in msecs */
#define TOUCHBOOST_MAX_TIMEOUT 10000
/* On/Off Toggle- Disable by default to save battery */
static int touchboost_status = 0; 
/* Timeout- in milli seconds */
static int touchboost_timeout = 0;
/* CPU Frequency boost */ 
static unsigned int freq_boost = TOUCHBOOST_FREQUENCY3; 
/* Timer */
static struct hrtimer touchboost_timer;
static struct kobject *touchboost_kobj;
char *driver_name = "touchboost";
DEFINE_MUTEX(touchboost_mutex);


static ssize_t touchboost_version_read(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf) 
{
	return sprintf(buf, "%d\n", TOUCHBOOST_VERSION);
}

static ssize_t touchboost_status_read(struct kobject *kobj, struct kobj_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", touchboost_status);
}

static ssize_t touchboost_status_write(struct kobject *kobj, struct kobj_attribute *attr,
					const char *buf, ssize_t count)
{
	int status;
	sscanf(buf, "%d", &status);

	if(status)
	{
		touchboost_status = 1;
#ifdef TOUCHBOOST_DEBUG
		printk(KERN_DEBUG "CPU Boost on Touch input ENABLED");
		printk(KERN_DEBUG "touchboost_status=%d",touchboost_status);
#endif			
	}
	return count;
}

static ssize_t freq_boost_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u Mhz\n",freq_boost);
}

static ssize_t freq_boost_write(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, ssize_t count)
{
	unsigned int freq;
	sscanf(buf, "%u\n", &freq);
	if((freq != TOUCHBOOST_FREQUENCY1) && (freq != TOUCHBOOST_FREQUENCY2) &&
		(freq != TOUCHBOOST_FREQUENCY3) && (freq != TOUCHBOOST_FREQUENCY4))
			printk(KERN_ERR "Invalid Value for touchboost!"); 
	else {
		printk(KERN_DEBUG "(%s) freq_boost set to %u Mhz\n",__func__,freq);
		freq_boost = freq;
	}

	return count;
}

static ssize_t timeout_read(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", touchboost_timeout);
}

static ssize_t timeout_write(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, ssize_t count)
{
	int t_timeout;
	sscanf(buf, "%d", &t_timeout);
	/* Limit timeout to a max of 10 seconds */ 
	if(t_timeout > TOUCHBOOST_MAX_TIMEOUT || t_timeout < 0)
		printk(KERN_ERR "(%s) Timeout cannot be set to %d\n msecs",__func__,t_timeout); 
	else {
		printk(KERN_DEBUG "(%s) Timeout set to %d\n",__func__,t_timeout);
		touchboost_timeout = t_timeout;
	}
	return count;
}


static struct kobj_attribute touchboost_version = __ATTR(version, 0666, touchboost_version_read, NULL);
static struct kobj_attribute touchboost_toggle = __ATTR(enable, 0666, touchboost_status_read, touchboost_status_write);
static struct kobj_attribute touchboost_freq = __ATTR(freq_boost, 0666, freq_boost_show, freq_boost_write);
static struct kobj_attribute touchboost_tout = __ATTR(timeout, 0666, timeout_read, timeout_write);

static struct attribute *touchboost_attrs[] = 
{
	&touchboost_version.attr,
	&touchboost_toggle.attr,
	&touchboost_freq.attr,
	&touchboost_tout.attr,
	NULL, 
};

static struct attribute_group touchboost_attr_group = 
{
	.attrs = touchboost_attrs,
};

static enum hrtimer_restart touchboost_timer_func(struct hrtimer *timer)
{
#ifdef TOUCHBOOST_DEBUG
	printk(KERN_DEBUG "%s is called\n", __func__);
#endif
	return HRTIMER_NORESTART;
}

static void __init touchboost_timer_init(void)
{
	hrtimer_init(&touchboost_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	touchboost_timer.function = touchboost_timer_func;
}

/* Helper prcmu functions */ 

static void __init touchboost_prcmu_add_req(void)
{
#ifdef TOUCHBOOST_DEBUG
	printk(KERN_DEBUG "%s is called\n", __func__);
#endif
	prcmu_qos_add_requirement(PRCMU_QOS_APE_OPP, driver_name, PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_DDR_OPP, driver_name, PRCMU_QOS_DEFAULT_VALUE);
	prcmu_qos_add_requirement(PRCMU_QOS_ARM_KHZ, driver_name, PRCMU_QOS_DEFAULT_VALUE);
}

static void __exit touchboost_prcmu_remove_req(void)
{
#ifdef TOUCHBOOST_DEBUG
	printk(KERN_DEBUG "%s is called\n", __func__);
#endif
	prcmu_qos_remove_requirement(PRCMU_QOS_APE_OPP, driver_name);
	prcmu_qos_remove_requirement(PRCMU_QOS_DDR_OPP, driver_name);
	prcmu_qos_remove_requirement(PRCMU_QOS_ARM_KHZ, driver_name);
}

static void prcmu_touchboost(unsigned int frequency)
{
	prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP, driver_name, PRCMU_QOS_APE_OPP_MAX);
	prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP, driver_name, PRCMU_QOS_DDR_OPP_MAX);
	prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ, driver_name, frequency * 1000);
}

/* 
*
*
* Restore OPPs back to default   
*
*/

void touchboost_restore(void)
{
	if(touchboost_status == 1) {
#ifdef TOUCHBOOST_DEBUG
		printk(KERN_DEBUG "(%s) is called\n", __func__);
#endif
		mutex_lock(&touchboost_mutex);
		prcmu_qos_update_requirement(PRCMU_QOS_APE_OPP,driver_name,PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_DDR_OPP,driver_name,PRCMU_QOS_DEFAULT_VALUE);
		prcmu_qos_update_requirement(PRCMU_QOS_ARM_KHZ,driver_name,PRCMU_QOS_DEFAULT_VALUE);
		mutex_unlock(&touchboost_mutex);
	}
}

/*
*
* Core function of the driver 
* 
*/

int input_boost(unsigned int frequency)
{

	mutex_lock(&touchboost_mutex);
	prcmu_touchboost(frequency);
	mutex_unlock(&touchboost_mutex);
	return 0;
}

/* Call from the touchscreen driver */
 
void touchboost(void)
{
#ifdef TOUCHBOOST_DEBUG
	printk(KERN_DEBUG "[%s] is called\n", __func__);
	printk(KERN_DEBUG "[TOUCHBOOST] touchboost_status=%d \n",touchboost_status);
	printk(KERN_DEBUG "[%s] Frequency:(%u) Mhz, Timeout:(%d) msecs\n",__func__,freq_boost,touchboost_timeout);
#endif
	if(touchboost_status == 1) {
		input_boost(freq_boost);
	}
}
EXPORT_SYMBOL(touchboost);

/* Starts the timer, restore back once timeout ends */ 

void start_timeout(void)
{
  if(touchboost_status == 1) {
   	if(touchboost_timeout) {	
		hrtimer_cancel(&touchboost_timer);
		hrtimer_start(&touchboost_timer, ns_to_ktime((u64)touchboost_timeout * NSEC_PER_MSEC),HRTIMER_MODE_REL);
		touchboost_restore();
   	}
   	else
		touchboost_restore();
  }	
}
EXPORT_SYMBOL(start_timeout);
 
static int __init touchboost_driver_init(void)
{
	int ret;
	printk(KERN_DEBUG "[%s]\n",__func__);
	touchboost_timer_init();
	touchboost_prcmu_add_req();
	touchboost_kobj = kobject_create_and_add("touchboost", kernel_kobj);

	if(!touchboost_kobj) 
		return -ENOMEM;

	ret = sysfs_create_group(touchboost_kobj, &touchboost_attr_group);
	
	if(ret)
		kobject_put(touchboost_kobj);

	return ret;

}

device_initcall(touchboost_driver_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aditya Patange");
