#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>

#include "core/met_drv.h"
#include "core/trace.h"

#include "mt_typedefs.h"
#include "mt_reg_base.h"
#include "mt_emi_bm.h"
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

extern unsigned int mt_get_emi_freq(void);
static ssize_t emi_clock_rate_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i;
	i = snprintf(buf, PAGE_SIZE, "%d\n", mt_get_emi_freq());
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
	MET_BM_Init();
	/* init EMI bus monitor */
	//MET_BM_SetReadWriteType(DEF_BM_RW_TYPE);
	MET_BM_SetReadWriteType(rwtype);
	MET_BM_SetMonitorCounter(1, BM_MASTER_MMSYS, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	MET_BM_SetMonitorCounter(2, BM_MASTER_AP_MCU, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	MET_BM_SetMonitorCounter(3, BM_MASTER_MD_MCU | BM_MASTER_MD_HW , BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	//MET_BM_SetMonitorCounter(4, BM_MASTER_MM0_PERI, BM_TRANS_TYPE_4BEAT | BM_TRANS_TYPE_8Byte | BM_TRANS_TYPE_BURST_WRAP);
	MET_BM_SetLatencyCounter();
}

static void emi_start(void)
{
	MET_BM_Enable(1);
}

static void emi_stop(void)
{
	MET_BM_Enable(0);
}

static int do_emi(void)
{
	return met_emi.mode;
}

static unsigned int emi_polling(unsigned int *emi_value)
{
	int i;
	int j = -1;

	MET_BM_Pause();

	//Get Bus Cycle Count
	emi_value[++j] = MET_BM_GetBusCycCount();
	//Get Word Count
	for (i=0; i<3; i++) {
		emi_value[++j] = MET_BM_GetWordCount(i+1);
	}
	//Get all Word Count
	emi_value[++j] = MET_BM_GetWordAllCount();

	//To derive the bandwidth % usage (BSCT / BACT)

	emi_value[++j] = MET_BM_GetBusBusyCount(1);
	emi_value[++j] = MET_BM_GetBusBusyCount(2);
	emi_value[++j] = MET_BM_GetBusBusyCount(3);
	emi_value[++j] = MET_BM_GetBusBusyAllCount();

	emi_value[++j] = MET_BM_GetEMIClockCount();


#if 0
	mt6577_mon_log_buff[cur].BM_WSCT = MET_BM_GetWordCount(1);	/*MM SUBSYS MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT2 = MET_BM_GetWordCount(2); /*APMCU MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT3 = MET_BM_GetWordCount(3); /*MD SUBSYS MEMORY*/
	mt6577_mon_log_buff[cur].BM_WSCT4 = MET_BM_GetWordCount(4); /*PERIPHERAL SUBSYS*/
#endif
	//printk("[%d, %d, %d, %d, %d]\n", bcnt,
	//	wsctp[0], wsctp[1], wsctp[2], wsctp[3]);

	MET_BM_Enable(0);
//delay
	ndelay(1);
	MET_BM_Enable(1);

	return j+1;
}

static void emi_uninit(void)
{
	MET_BM_DeInit();
}

//static struct kobject *emi_parent;
static int met_emi_create(struct kobject *parent)
{
	int ret = 0;

	kobj_emi = parent;

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
	unsigned int emi_value[17];

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
"met-info [000] 0.0: met_emi_header: BUS_CYCLE,MM_WSCT,APMCU_WSCT,MD_WSCT,ALL_WACT,MM_BSCT,APMCU_BSCT,MD_BSCT,ALL_BACT,EMI_TSCT\n";

static int emi_print_help(char *buf, int len)
{
	return snprintf(buf, PAGE_SIZE, help);
}

static int emi_print_header(char *buf, int len)
{
	return snprintf(buf, PAGE_SIZE, header, mt_get_emi_freq());
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

