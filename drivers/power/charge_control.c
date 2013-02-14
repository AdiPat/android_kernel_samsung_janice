/*
 *  Copyright (C) 2013
 *  
 *  Author: Aditya Patange "Adi_Pat" <adithemagnificent@gmail.com>
 *  
 *  Thanks to Cocafe for the idea.  
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Charger Current Control for AB8500 Charger. 
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define CHARGE_CONTROL_VERSION   1
#define CHARGE_CONTROL_DEBUG     0
/* In mA */
#define USB_CURR_MAX            900
#define USB_CURR_DEFAULT        500
#define USB_CURR_MIN            350
 
#define AC_CURR_MAX             900
#define AC_CURR_MIN             350
#define AC_CURR_DEFAULT	        600

static int ac_curr;
static int usb_curr;

void charger_control_register_ac_curr(int ac_current)
{
	ac_curr = ac_current;
#ifdef CHARGE_CONTROL_DEBUG
	pr_debug("[Charge Control] Registered AC Current: %dmA\n",ac_curr);
#endif
}
EXPORT_SYMBOL(charger_control_register_ac_curr);

void charger_control_register_usb_curr(int usb_current)
{
	usb_curr = usb_current;
#ifdef CHARGE_CONTROL_DEBUG
	pr_debug("[Charge Control] Registered USB Current: %dmA\n",usb_curr);
#endif
}
EXPORT_SYMBOL(charger_control_register_usb_curr);

int get_ac_curr(void)
{
#ifdef CHARGE_CONTROL_DEBUG
	pr_debug("[Charge Control](%s) called! ac_curr=%dmA\n",__func__,ac_curr);
#endif
	return ac_curr;
}
EXPORT_SYMBOL(get_ac_curr);

int get_usb_curr(void)
{
#ifdef CHARGE_CONTROL_DEBUG
	pr_debug("[Charge Control](%s) called! usb_curr=%dmA\n",__func__,usb_curr);
#endif
	return usb_curr;
}
EXPORT_SYMBOL(get_usb_curr);

static ssize_t usb_curr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",usb_curr);
}

static ssize_t usb_curr_store(struct device *dev, struct device_attribute *attr, const char *buf, 
				ssize_t count)
{
	int u_curr;
	sscanf(buf, "%d",&u_curr);
	if(u_curr > USB_CURR_MAX || u_curr < USB_CURR_MIN) 
		pr_err("[Charge Control] Invalid Value set for USB Current: %dmA \n",u_curr);
	else {
		usb_curr = u_curr;
		pr_debug("[Charge Control] USB Current set to: %dmA\n",usb_curr);
	}

	return count;
}

static ssize_t ac_curr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",ac_curr);
}	

static ssize_t ac_curr_store(struct device *dev, struct device_attribute *attr, const char *buf,
				ssize_t count)
{
	int a_curr;
	sscanf(buf, "%d", &a_curr);
	if(a_curr > AC_CURR_MAX || a_curr < AC_CURR_MIN) 
		pr_err("[Charge Control] Invalid Value set for AC Current: %dmA \n",a_curr);
	else {
		ac_curr = a_curr;
		pr_debug("[Charge Control] AC Current set to: %dmA\n",ac_curr);
	}
	return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", CHARGE_CONTROL_VERSION);
}

static DEVICE_ATTR(usb_current, S_IRUGO | S_IWUGO, usb_curr_show, usb_curr_store);
static DEVICE_ATTR(ac_current, S_IRUGO | S_IWUGO, ac_curr_show, ac_curr_store);
static DEVICE_ATTR(version, S_IRUGO, version_show, NULL);

static struct attribute *charge_control_attributes[] = {
	&dev_attr_usb_current.attr,
	&dev_attr_ac_current.attr,
	&dev_attr_version.attr,
	NULL
};

static struct attribute_group charge_control_attr_group = {
	.attrs = charge_control_attributes,
};

static struct miscdevice charge_control_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "charge_control",
    };


static int __init charge_control_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __func__, charge_control_device.name);

    ret = misc_register(&charge_control_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __func__, charge_control_device.name);
	    return 1;
	}

    if (sysfs_create_group(&charge_control_device.this_device->kobj, &charge_control_attr_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", charge_control_device.name);
	}
    return 0;
}
device_initcall(charge_control_init);

MODULE_AUTHOR("Aditya Patange");
MODULE_LICENSE("GPL");
