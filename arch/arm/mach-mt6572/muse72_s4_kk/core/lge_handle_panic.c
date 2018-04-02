/*
 * arch/arm/mach-msm/lge/lge_handle_panic.c
 *
 * Copyright (C) 2010 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/setup.h>
#include <linux/module.h>
#include "mach/devs.h"

#ifdef CONFIG_CPU_CP15_MMU
 /* LGE_CHANGE: lge crash handler, refer to D1L model
  * 2012-07-18, kyeongdon.kim@lge.com  */

#include <linux/ptrace.h>
#endif

#define PANIC_HANDLER_NAME "panic-handler"
#define PANIC_DUMP_CONSOLE 0
#define PANIC_MAGIC_KEY	0x12345678
#define CRASH_ARM9		0x87654321
#define CRASH_REBOOT	0x618E1000

struct crash_log_dump {
	unsigned int magic_key;
	unsigned int size;
	unsigned char buffer[0];
};

static struct crash_log_dump *crash_dump_log;
static unsigned int crash_buf_size;
static int crash_store_flag;

#ifdef CONFIG_CPU_CP15_MMU
 /* LGE_CHANGE: lge crash handler, refer to D1L model
  * 2012-07-18, kyeongdon.kim@lge.com  */
unsigned long *cpu_crash_ctx=NULL;
#endif

static DEFINE_SPINLOCK(lge_panic_lock);

static int dummy_arg;
static int gen_bug(const char *val, struct kernel_param *kp)
{
	BUG();

	return 0;
}
module_param_call(gen_bug, gen_bug, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int gen_panic(const char *val, struct kernel_param *kp)
{
	//panic("generate test-panic");
	int *panic_test;
	panic_test = 0;
	*panic_test = 0x1234;
	return 0;
}
module_param_call(gen_panic, gen_panic, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static DEFINE_SPINLOCK(watchdog_timeout_lock);
static int gen_wdog(const char *val, struct kernel_param *kp)
{
	unsigned long flags;
	
	spin_lock_irqsave(&watchdog_timeout_lock, flags);
	while(1);

	return 0;
}
module_param_call(gen_wdog, gen_wdog, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

extern void die(const char *str, struct pt_regs *regs, int err);
static int gen_die(const char *val, struct kernel_param *kp)
{
	struct pt_regs *reg;

	reg = (struct pt_regs *)cpu_crash_ctx;

	die("test die", reg, 0);

	return 0;
}
module_param_call(gen_die, gen_die, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);

static int crash_handle_enable = 1;
module_param_named(crash_handle_enable, crash_handle_enable,
				   int, S_IRUGO | S_IWUSR | S_IWGRP);

void set_crash_store_enable(void)
{
	crash_store_flag = 1;

	return;
}

void set_crash_store_disable(void)
{
	crash_store_flag = 0;

	return;
}

void store_crash_log(char *p)
{
	if (!crash_store_flag)
		return;

	if (crash_dump_log->size == crash_buf_size)
		return;

	for ( ; *p; p++) {
		if (*p == '[') {
			for ( ; *p != ']'; p++)
				;
			p++;
			if (*p == ' ')
				p++;
		}

		if (*p == '<') {
			for ( ; *p != '>'; p++)
				;
			p++;
		}

		crash_dump_log->buffer[crash_dump_log->size] = *p;
		crash_dump_log->size++;
	}
	crash_dump_log->buffer[crash_dump_log->size] = 0;

	return;
}

#ifdef CONFIG_CPU_CP15_MMU
 /* LGE_CHANGE: lge crash handler, refer to D1L model
  * 2012-07-18, kyeongdon.kim@lge.com  */
void lge_save_ctx(struct pt_regs* regs, unsigned int ctrl, unsigned int transbase, unsigned int dac)
{
	/* save cpu register for simulation */
	cpu_crash_ctx[0] = regs->ARM_r0;
	cpu_crash_ctx[1] = regs->ARM_r1;
	cpu_crash_ctx[2] = regs->ARM_r2;
	cpu_crash_ctx[3] = regs->ARM_r3;
	cpu_crash_ctx[4] = regs->ARM_r4;
	cpu_crash_ctx[5] = regs->ARM_r5;
	cpu_crash_ctx[6] = regs->ARM_r6;
	cpu_crash_ctx[7] = regs->ARM_r7;
	cpu_crash_ctx[8] = regs->ARM_r8;
	cpu_crash_ctx[9] = regs->ARM_r9;
	cpu_crash_ctx[10] = regs->ARM_r10;
	cpu_crash_ctx[11] = regs->ARM_fp;
	cpu_crash_ctx[12] = regs->ARM_ip;
	cpu_crash_ctx[13] = regs->ARM_sp;
	cpu_crash_ctx[14] = regs->ARM_lr;
	cpu_crash_ctx[15] = regs->ARM_pc;
	cpu_crash_ctx[16] = regs->ARM_cpsr;
	/* save mmu register for simulation */
	cpu_crash_ctx[17] = ctrl;
	cpu_crash_ctx[18] = transbase;
	cpu_crash_ctx[19] = dac;

}
#endif

static int restore_crash_log(struct notifier_block *this, unsigned long event,
		void *ptr)
{
	unsigned long flags;

	crash_store_flag = 0;

	spin_lock_irqsave(&lge_panic_lock, flags);

	//	lge_set_reboot_reason(CRASH_REBOOT);	//kyeongdon.kim@lge.com temp block
	crash_dump_log->magic_key = PANIC_MAGIC_KEY;

	spin_unlock_irqrestore(&lge_panic_lock, flags);

	return NOTIFY_DONE;
}

static struct notifier_block panic_handler_block = {
	.notifier_call  = restore_crash_log,
};

/* LGE_CHANGE_S: 2012-11-18 bk.seol@lge.com lge silent reset */
#ifdef  CONFIG_LGE_HIDDEN_RESET
#define HRESET_MAGIC    0x12345678
//#define LGE_SET_HRESET_FBCON

struct hidden_reset_flag    {
    unsigned int magic_key;
    unsigned long fb_addr;
    unsigned long fb_size;
};

struct hidden_reset_flag *hreset_flag   = NULL;

int hreset_enable   = 0;
static int hreset_enable_set(const char *val, struct kernel_param *kp)
{
    int ret;

    ret = param_set_int(val, kp);
    if (ret)
        return ret;

    if (hreset_enable)  {
        if (hreset_flag)
            hreset_flag->magic_key  = HRESET_MAGIC;
        printk(KERN_INFO "[LGE] hidden reset activated\n");
    } else {
        if (hreset_flag)
            hreset_flag->magic_key  = 0;
        printk(KERN_INFO "[LGE] hidden reset deactivated\n");
    }
    return  0;
}
module_param_call(hreset_enable, hreset_enable_set, param_get_int,
    &hreset_enable, S_IRUGO|S_IWUSR|S_IWGRP);

int on_hidden_reset = 0;
module_param_named(on_hidden_reset, on_hidden_reset, int, S_IRUGO|S_IWUSR|S_IWGRP);

static int __init check_hidden_reset(char *reset_mode)
{
    if (!strncmp(reset_mode, "on", 2))  {
        on_hidden_reset = 1;
        printk(KERN_INFO "[LGE] reboot mode : hidden reset %s\n", "on");
    }
    return  1;
}
__setup("lge.hreset=", check_hidden_reset);

static ssize_t is_hidden_show(struct device *dev, struct device_attribute *addr,
    char *buf)
{
    return sprintf(buf, "%d\n", on_hidden_reset);
}
static DEVICE_ATTR(is_hreset, S_IRUGO|S_IWUSR|S_IWGRP, is_hidden_show, NULL);
#endif  /* CONFIG_LGE_HIDDEN_RESET */
/* LGE_CHANGE_E: 2012-11-18 bk.seol@lge.com */

static int __init lge_panic_handler_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	int ret = 0;
#ifdef CONFIG_CPU_CP15_MMU
 /* LGE_CHANGE: lge crash handler, refer to D1L model
  * 2012-07-18, kyeongdon.kim@lge.com  */
	void *ctx_buf;
	size_t ctx_start;
#endif
/* LGE_CHANGE_S: 2012-11-18 bk.seol@lge.com lge silent reset */
#ifdef  CONFIG_LGE_HIDDEN_RESET
#if defined(LGE_SET_HRESET_FBCON)
    void *hreset_flag_buf;
#endif
#endif  /* CONFIG_LGE_HIDDEN_RESET */
/* LGE_CHANGE_E: 2012-11-18 bk.seol@lge.com */

	if (res == NULL || pdev->num_resources != 1 ||
			!(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "lge_panic_handler: invalid resource, %p %d flags "
				"%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}

	buffer_size 	= res->end - res->start + 1;
	start 			= res->start;
	printk(KERN_INFO "lge_panic_handler: got buffer at %zx, size %zx\n",
			start, buffer_size);

	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "lge_panic_handler: failed to map memory\n");
		return -ENOMEM;
	}

	crash_dump_log = (struct crash_log_dump *)buffer;
	memset(crash_dump_log, 0, buffer_size);
	crash_dump_log->magic_key 	= 0;
	crash_dump_log->size 		= 0;
	crash_buf_size 				= buffer_size - offsetof(struct crash_log_dump, buffer);

#ifdef CONFIG_CPU_CP15_MMU
 /* LGE_CHANGE: lge crash handler, refer to D1L model
  * 2012-07-18, kyeongdon.kim@lge.com  */
	ctx_start = LGE_CRASH_CTX_BUF_PHY_ADDR;
	ctx_buf = ioremap(ctx_start, LGE_CRASH_CTX_BUF_SIZE);
	if (ctx_buf == NULL) {
		printk(KERN_ERR "cpu crash ctx buffer: failed to map memory\n");
		return -ENOMEM;
	}
	cpu_crash_ctx = (unsigned long *)ctx_buf;
/* LGE_CHANGE_S: [2012-05-09] duvallee.lee@lge.com  : buffer clear */
    memset(cpu_crash_ctx, 0, LGE_CRASH_CTX_BUF_SIZE);
/* LGE_CHANGE_E: [2012-05-09] duvallee.lee@lge.com */
#endif
/* LGE_CHANGE_S: 2012-11-18 bk.seol@lge.com lge silent reset */
#ifdef  CONFIG_LGE_HIDDEN_RESET
#if defined(LGE_SET_HRESET_FBCON)
    hreset_flag_buf = ioremap((res->end + 1024 + 1), 1024);
    if (!hreset_flag_buf)   {
        printk(KERN_ERR "[LGE] hreset flag buffer: failed to map memory\n");
        return -ENOMEM;
    }

    hreset_flag = (struct hidden_reset_flag *)hreset_flag_buf;

    ret = device_create_file(&pdev->dev, &dev_attr_is_hreset);
    if( ret < 0 ) {
        printk(KERN_ERR "[LGE] device_create_file error!\n");
        return ret;
    }

    if (hreset_enable)  {
        hreset_flag->magic_key  = HRESET_MAGIC;
    } else  {
        hreset_flag->magic_key  = 0;
    }

    if (lge_get_fb_phys_info(&hreset_flag->fb_addr, &hreset_flag->fb_size)) {
        hreset_flag->magic_key = 0;
        printk("[LGE] hreset_flag: failed to get_fb_phys_info\n");
    }
#endif  /* LGE_SET_HRESET_FBCON */
#endif  /* CONFIG_LGE_HIDDEN_RESET */
/* LGE_CHANGE_E: 2012-11-18 bk.seol@lge.com */

	/* Setup panic notifier */
	atomic_notifier_chain_register(&panic_notifier_list, &panic_handler_block);

	return ret;
}

static int __devexit lge_panic_handler_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver panic_handler_driver __refdata = {
	.probe = lge_panic_handler_probe,
	.remove = __devexit_p(lge_panic_handler_remove),
	.driver = {
		.name = PANIC_HANDLER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init lge_panic_handler_init(void)
{
	return platform_driver_register(&panic_handler_driver);
}

static void __exit lge_panic_handler_exit(void)
{
	platform_driver_unregister(&panic_handler_driver);
}

module_init(lge_panic_handler_init);
module_exit(lge_panic_handler_exit);

MODULE_DESCRIPTION("LGE panic handler driver");
MODULE_AUTHOR("SungEun Kim <cleaneye.kim@lge.com>");
MODULE_LICENSE("GPL");
