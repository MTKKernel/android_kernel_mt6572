#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>

#include "core/met_drv.h"
#include "core/trace.h"

#include "mt6577_typedefs.h"
#include "mt6577_reg_base.h"
#include "mt6577_emi_bm.h"
#include "sync_write.h"
#include "plf_trace.h"

#define DEF_BM_RW_TYPE      (BM_BOTH_READ_WRITE)
extern struct metdevice met_emi;
static struct kobject *kobj_emi = NULL;
static volatile int rwtype = BM_BOTH_READ_WRITE;

static ssize_t rwtype_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t rwtype_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n);
static ssize_t emi_clock_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static struct kobj_attribute rwtype_attr = __ATTR(rwtype, 0644, rwtype_show, rwtype_store);
static struct kobj_attribute emi_clock_rate_attr = __ATTR_RO(emi_clock_rate);


extern unsigned int mt6577_get_bus_freq(void);
static ssize_t emi_clock_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	i = snprintf(buf, PAGE_SIZE, "%d\n", mt6577_get_bus_freq());
	return i;
}

static ssize_t rwtype_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	i = snprintf(buf, PAGE_SIZE, "%d\n", rwtype);
	return i;
}

static ssize_t rwtype_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	int value;

	if ((n == 0) || (buf == NULL)) {
		return -EINVAL;
	}
	if (sscanf(buf, "%d", &value) != 1) {
		return -EINVAL;
	}
	if (value < 0 && value > BM_BOTH_READ_WRITE) {
		return -EINVAL;
	}
	rwtype = value;
	return n;
}

static void emi_init(void)
{
	BM_Init();
	/* init EMI bus monitor */
	//BM_SetReadWriteType(DEF_BM_RW_TYPE);
	BM_SetReadWriteType(rwtype);
	BM_SetReadWriteType(DEF_BM_RW_TYPE);
	BM_SetMonitorCounter(1, BM_MASTER_MULTIMEDIA, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	BM_SetMonitorCounter(2, BM_MASTER_AP_MCU, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	BM_SetMonitorCounter(3, BM_MASTER_MD_DSP | BM_MASTER_MD_MCU | BM_MASTER_2G_3G_MDDMA, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	BM_SetMonitorCounter(4, BM_MASTER_PERI, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
}

static void emi_start(void)
{
	BM_Enable(1);
}

static void emi_stop(void)
{
	BM_Enable(0);
}

static int do_emi(void)
{
	return met_emi.mode;
}

static unsigned int emi_polling(unsigned int *emi_value)
{
	int i;
	int bcnt;
	int *wsctp;

	void BM_Pause(void);

	// read counter
	//Get Bus Cycle Count
	bcnt = BM_GetBusCycCount();
	emi_value[0] = bcnt;

	//Get Word Count
	wsctp = emi_value + 1;
	for (i=0; i<4; i++) {
		wsctp[i] = BM_GetWordCount(i+1);
	}
#if 0
	mt6577_mon_log_buff[cur].BM_WSCT = BM_GetWordCount(1);	/*MM SUBSYS MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT2 = BM_GetWordCount(2); /*APMCU MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT3 = BM_GetWordCount(3); /*MD SUBSYS MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT4 = BM_GetWordCount(4); /*PERIPHERAL SUBSYS*/
#endif
	//printk("[%d, %d, %d, %d, %d]\n", bcnt,
	//	wsctp[0], wsctp[1], wsctp[2], wsctp[3]);

	BM_Enable(0);
	BM_Enable(1);

	return 5;
}

static void emi_uninit(void)
{
	BM_DeInit();
}

static int met_emi_create(struct kobject *parent)
{
	int ret = 0;

	kobj_emi =parent;

	ret = sysfs_create_file(kobj_emi, &rwtype_attr.attr);
	if (ret != 0) {
		pr_err("Failed to create rwtype in sysfs\n");
		return ret;
	}
	ret = sysfs_create_file(kobj_emi, &emi_clock_rate_attr.attr);
	if (ret != 0) {
		pr_err("Failed to create emi_clock_rate in sysfs\n");
		return ret;
	}
    return ret;
}


static void met_emi_delete(void)
{
	sysfs_remove_file(kobj_emi, &rwtype_attr.attr);
	sysfs_remove_file(kobj_emi, &emi_clock_rate_attr.attr);
	kobj_emi = NULL;
}

static void met_emi_start(void)
{
	if (do_emi()) {
		emi_init();
		emi_stop();
		emi_start();
	}
}

static void met_emi_stop(void)
{
	if (do_emi()) {
		emi_stop();
		emi_uninit();
	}

}

static void met_emi_polling(unsigned long long stamp, int cpu)
{
	unsigned char count;
	unsigned int emi_value[5];

	if (do_emi()) {
		count = emi_polling(emi_value);
		if (count) {
			ms_emi(stamp, count, emi_value);
		}
	}
}

static char help[] = "  --emi                                 monitor EMI banwidth\n";
static char header[] =
"met-info [000] 0.0: met_emi_clockrate: %d000\n"
"# ms_emi: timestamp,BUS_CYCLE,MM,APMCU,MD,PERIPHERAL\n"
"met-info [000] 0.0: met_emi_header: BUS_CYCLE,MM,APMCU,MD,PERIPHERAL\n";

static int emi_print_help(char *buf, int len)
{
	return snprintf(buf, PAGE_SIZE, help);
}

static int emi_print_header(char *buf, int len)
{
	return snprintf(buf, PAGE_SIZE, header, mt6577_get_bus_freq());
}

struct metdevice met_emi = {
	.name = "emi",
	.owner = THIS_MODULE,
	.type = MET_TYPE_BUS,
	.create_subfs = met_emi_create,
	.delete_subfs = met_emi_delete,
	.cpu_related = 0,
	.start = met_emi_start,
	.stop = met_emi_stop,
	.timed_polling = met_emi_polling,
	.tagged_polling = met_emi_polling,
	.print_help = emi_print_help,
	.print_header = emi_print_header,
};

