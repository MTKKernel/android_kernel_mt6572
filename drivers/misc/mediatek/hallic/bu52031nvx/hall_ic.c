/*
 * Copyright (C) 2013 LG Electironics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>

#include <mach/eint.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#ifndef TPD_NO_GPIO
#include "cust_eint.h"
#include "cust_gpio_usage.h"
#endif

#define COVER_GPIO_OPEN (1)
#define COVER_GPIO_CLOSE (0)

#define CRADLE_NO_DEV (0)
#define CRADLE_DESKDOCK (1)
#define CRADLE_CARKIT (2)
#define CRADLE_SMARTCOVER (5)
#define CRADLE_SMARTCOVER_NO_DEV (6)

#define COVER_NAME "smartcover"

static struct workqueue_struct *hallic_wq;
static struct delayed_work hallic_work;
static int hallic_status;
static spinlock_t hallic_lock;
static struct switch_dev hallic_switch_dev;
static struct wake_lock hallic_wakelock;


static ssize_t hall_ic_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int gpio_status;
	
	gpio_status = mt_get_gpio_in(GPIO_HALL_INT);
	
	return sprintf(buf, "%d\n", gpio_status);
}

static ssize_t hall_ic_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	value = !!value;
	spin_lock(&hallic_lock);

	if(value == COVER_GPIO_OPEN) {
		switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER_NO_DEV);
	}
	else {
		switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER);
	}

	spin_unlock(&hallic_lock);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, hall_ic_enable_show, hall_ic_enable_store);

static void hall_ic_eint_interrupt_handler(void)
{
	mt65xx_eint_mask(CUST_EINT_MHALL_NUM);
	queue_delayed_work(hallic_wq, &hallic_work, 0);
}

static void hall_ic_work_func(struct work_struct *work)
{
	int gpio_status;
	unsigned int polarity;
	u32 pull;

	spin_lock_irq(&hallic_lock);
	
	gpio_status = mt_get_gpio_in(GPIO_HALL_INT);

	if(hallic_status != gpio_status) {
		printk("[HALL_IC] hall_ic_work_func: %d\n", gpio_status);
		hallic_status = gpio_status;
		if(gpio_status == 1) {
			polarity = CUST_EINT_POLARITY_LOW;
			pull = GPIO_PULL_UP;
		}
		else {
			polarity = CUST_EINT_POLARITY_HIGH;
			pull = GPIO_PULL_DOWN;
		}

		wake_lock_timeout(&hallic_wakelock, msecs_to_jiffies(3000));

		if(hallic_status == COVER_GPIO_OPEN) {
			switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER_NO_DEV);
		}
		else {
			switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER);
		}

		mt_set_gpio_pull_select(GPIO_HALL_INT, pull);
		mt65xx_eint_set_polarity(CUST_EINT_MHALL_NUM, polarity);
	}
	else {
		printk("[HALL_IC] ignore int: %d\n", gpio_status);
	}
		
	mt65xx_eint_unmask(CUST_EINT_MHALL_NUM);
	spin_unlock(&hallic_lock);
}

static ssize_t hall_ic_print_name(struct switch_dev *sdev, char *buf)
{
	switch(switch_get_state(sdev)) {
		case CRADLE_NO_DEV:
			return sprintf(buf, "UNDOCKED\n");
		case CRADLE_CARKIT:
			return sprintf(buf, "CARKIT\n");
    }
    return -EINVAL;
}

static int hall_ic_probe(struct platform_device *pdev)
{
	int ret;
	int gpio_status;
	unsigned int polarity;
	u32 pull;

	hallic_wq = create_singlethread_workqueue("hallic_wq");
	if(!hallic_wq) {
		printk("[HALL_IC] failed to create singlethread workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&hallic_work, hall_ic_work_func);

	spin_lock_init(&hallic_lock);

	wake_lock_init(&hallic_wakelock, WAKE_LOCK_SUSPEND, "hallic_wakelock");

	hallic_switch_dev.name = COVER_NAME;
	hallic_switch_dev.print_name = hall_ic_print_name;

	ret = switch_dev_register(&hallic_switch_dev);
	if(ret) {
		printk("[HALL_IC] switch_dev_register failed: %d\n", ret);
		return -ENOMEM;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_enable);
	if(ret) {
		printk("[HALL_IC] device_create_file failed: %d\n", ret);
		return -ENOMEM;
	}

	gpio_status = mt_get_gpio_in(GPIO_HALL_INT);
	hallic_status = gpio_status;

	if(gpio_status == 1) {
		polarity = CUST_EINT_POLARITY_LOW;
		pull = GPIO_PULL_UP;
	}
	else {
		polarity = CUST_EINT_POLARITY_HIGH;
		pull = GPIO_PULL_DOWN;
	}

	if(hallic_status == COVER_GPIO_OPEN) {
		switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER_NO_DEV);
	}
	else {
		switch_set_state(&hallic_switch_dev, CRADLE_SMARTCOVER);
	}

	mt_set_gpio_mode(GPIO_HALL_INT, GPIO_HALL_INT_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_INT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_INT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_INT, pull);

	mt65xx_eint_set_sens(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_MHALL_NUM, polarity);
	mt65xx_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_EN, polarity, hall_ic_eint_interrupt_handler, 0);
	mt65xx_eint_unmask(CUST_EINT_MHALL_NUM);

	printk("[HALL_IC] hall_ic_init: %d\n", gpio_status);

	return 0;
}

static int hall_ic_remove(struct platform_device *pdev)
{
	return 0;
}


static struct platform_driver hallic_driver = {
	.probe = hall_ic_probe,
	.remove = hall_ic_remove,
	.driver = {
		.name = "hall-ic",
		.owner = THIS_MODULE,
	},
};

static int __init hall_ic_init(void)
{
	return platform_driver_register(&hallic_driver);
}

static void __exit hall_ic_exit(void)
{
	platform_driver_unregister(&hallic_driver);
}


module_init(hall_ic_init);
module_exit(hall_ic_exit);

MODULE_DESCRIPTION("BU52031NVX Hall IC Driver for MTK platform");
MODULE_AUTHOR("TY Kang <taiyou.kang@lge.com>");
MODULE_LICENSE("GPL");

