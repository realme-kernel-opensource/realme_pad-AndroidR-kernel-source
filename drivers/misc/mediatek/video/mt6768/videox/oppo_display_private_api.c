/***************************************************************
** Copyright (C),  2018,  OPPO Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oppo_display_private_api.h
** Description : oppo display private api implement
** Version : 1.0
** Date : 2018/03/20
** Author : Jie.Hu@PSW.MM.Display.Stability
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Hu.Jie          2018/03/20        1.0           Build this moudle
**   Guo.Ling        2018/10/11        1.1           Modify for SDM660
**   Guo.Ling        2018/11/27        1.2           Modify for mt6779
**   Lin.Hao         2019/11/01        1.3           Modify for MT6779_Q
******************************************************************/
#include "oppo_display_private_api.h"
#include "disp_drv_log.h"
#include <linux/fb.h>
#include <linux/time.h>
#include <linux/timekeeping.h>
/* Zhijun.ye@PSW.MM.Display.LCD.Stability, 2019/11/22,
 * add for enable dc by default on special version */
#include <soc/oppo/oppo_project.h>

/*
 * we will create a sysfs which called /sys/kernel/oppo_display,
 * In that directory, oppo display private api can be called
 */
extern void disp_aal_set_dre_en(int enable);
extern int primary_display_set_cabc_mode(unsigned int level);
extern int primary_display_setbacklight_nolock(unsigned int level);

unsigned long oplus_max_normal_brightness = 0;
unsigned long CABC_mode1 = 2;

/*
* add dre only use for camera
*/
extern void disp_aal_set_dre_en(int enable);

static ssize_t LCM_CABC_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    printk("%s CABC_mode=%ld\n", __func__, CABC_mode1);
    return sprintf(buf, "%ld\n", CABC_mode1);
}

static ssize_t LCM_CABC_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t num)
{
    int ret = 0;

    ret = kstrtoul(buf, 10, &CABC_mode1);
    if( CABC_mode1 > 3 ){
        CABC_mode1 = 3;
    }
    pr_err("%s CABC_mode=%ld\n", __func__, CABC_mode1);

#if 0
    if (CABC_mode1 == 3) {
        disp_aal_set_dre_en(1);
        pr_err("%s enable dre\n", __func__);

    } else {
        disp_aal_set_dre_en(0);
        pr_err("%s disable dre\n", __func__);
    }
#endif
    /*
	 * modify for close cabc function
     */
	pr_err("%s cabc primary_display_set_cabc_mode\n", __func__);
    ret = primary_display_set_cabc_mode((unsigned int)CABC_mode1);
    return num;
}

static struct kobject *oppo_display_kobj;

static DEVICE_ATTR(cabc, S_IRUGO|S_IWUSR, LCM_CABC_show, LCM_CABC_store);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *oppo_display_attrs[] = {
	&dev_attr_cabc.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group oppo_display_attr_group = {
	.attrs = oppo_display_attrs,
};

static int __init oppo_display_private_api_init(void)
{
	int retval;

	/* Zhijun.ye@PSW.MM.Display.LCD.Stability, 2019/11/22,
	 * add for enable dc by default on special version */

	oppo_display_kobj = kobject_create_and_add("oppo_display", kernel_kobj);
	if (!oppo_display_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(oppo_display_kobj, &oppo_display_attr_group);
	if (retval)
		kobject_put(oppo_display_kobj);

	return retval;
}

static void __exit oppo_display_private_api_exit(void)
{
	kobject_put(oppo_display_kobj);
}

module_init(oppo_display_private_api_init);
module_exit(oppo_display_private_api_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hujie");
