#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include "core/met_drv.h"
#include "core/trace.h"

#include "sync_write.h"
#include "mt_dramc.h"
#include "dramc.h"
#include "plf_trace.h"

extern struct metdevice met_dramc;
extern struct dramc_desc_t dramc_desc[DRAMC_MX_CHANNUM][DRAMC_MX_GRPNUM][DRAMC_MX_NUM_IN_AGRP];
static struct dramc_chann_t dramc_chann[DRAMC_MX_CHANNUM];

static void dramc_value_init(void)
{
	int idx = 0;

	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		dramc_chann[idx].mode = DRAMC_DISABLE;
		dramc_chann[idx].curr_agent = -1;
		dramc_chann[idx].toggle_mask = 0ULL;
	}
}

static void dramc_init(void)
{
	int idx = 0;

	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) 
		DRAMC_Init(idx);
}

static int do_dramc(int chann)
{
	return dramc_chann[chann].mode;
}

static void dramc_start(void)
{
	int idx = 0; 
	u64 mask = 0;
	int agent = 0;
	int group = 0;

	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		if (DRAMC_DISABLE == do_dramc(idx))
			continue;
		agent = dramc_chann[idx].curr_agent;
		mask = dramc_chann[idx].toggle_mask;
		//printk("chann: %d next agent: %d agent: %d mask: %llX\n",idx, FIND_NEXT_AGENT(mask, agent), agent, mask);
		agent = dramc_chann[idx].curr_agent = FIND_NEXT_AGENT(mask, agent);
		group = GROUP_ID(agent);
		if (DRAMC_ALL_GROUP_AGENT_ID == agent) {
			agent = 0x1F;
			group = 1;
		}
		DRAMC_ConfigTargetCount(idx,0xFFFFFFFF);
		DRAMC_Enable(idx, group, agent);
	}
}

static void dramc_stop(void)
{
	int idx = 0; 

	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		if (DRAMC_DISABLE == do_dramc(idx))
			continue;
		DRAMC_Disable(idx);
	}
}

static unsigned int dramc_polling(int chann, u32 *value)
{
	int j = -1;
	int agent = dramc_chann[chann].curr_agent;
	int group = GROUP_ID(agent);
	u32 total_cyccnt = 0;
	//printk("chann: %d agent: %d\n",chann, agent);
	DRAMC_Freeze(chann);

	value[++j] = agent;
	// redefine group id and agent	
	if (DRAMC_ALL_GROUP_AGENT_ID == agent) {
		agent = 0x1F;
		group = 1;
	}
	value[++j] = DRAMC_GetCycleCount(chann, group);
	value[++j] = total_cyccnt = DRAMC_GetTotalCycleCount(chann);

	return total_cyccnt;
}

static void dramc_uninit(void)
{
	int idx = 0;
	
	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		dramc_chann[idx].mode = DRAMC_DISABLE;
		dramc_chann[idx].curr_agent = -1;
		dramc_chann[idx].toggle_mask = 0ULL;
	}
}

static int met_dramc_create(struct kobject *parent)
{
	dramc_value_init();
    return 0;
}

static void met_dramc_delete(void)
{
	// do nothing
}

static void met_dramc_start(void)
{
	dramc_init();
	dramc_stop();
	dramc_start();
}

static void met_dramc_stop(void)
{
	dramc_stop();
	dramc_uninit();
}

static void met_dramc_polling(unsigned long long stamp, int cpu)
{
	int idx = 0;
	u32 total_cyccnt = 0;
	u32 dramc_value[3] = {0, 0, 0};

	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		if (DRAMC_DISABLE != do_dramc(idx)) {
			total_cyccnt = dramc_polling(idx, dramc_value);
			if (DRAMC_CheckCntIsOverFlow(total_cyccnt))
				tv_dramc_overflow(stamp, idx);
		 	tv_dramc(stamp, idx, dramc_value);
		}
	}
	dramc_stop();
	dramc_start();
}

static char header[] =
"# ms_dramc: timestamp,CHANNEL_ID,AGENT_ID,TARGET_BUS_CYCLE,TOTAL_BUS_CYCLE\n"
"met-info [000] 0.0: met_dramc_header:CHANNEL_ID,AGENT_ID,TARGET_BUS_CYCLE,TOTAL_BUS_CYCLE\n";

static char help[] = "  --dramc:[normal|toggle]:[agent name]                             monitor DRAMC bandwidth\n"
"  --dramc=normal:[channel]:all                                     monitor DRAMC bandwith of all group\n"
"  --dramc=normal:[channel]:[agent name]                            monitor DRAMC bandwidth with agent that you care\n"
"  --dramc=toggle:[channel]:all                                     monitor DRAMC bandwidth with toggle all mode\n"
"  --dramc=toggle:[channel]:[agent name]                            monitor DRAMC bandwidth with agent LIST that you care\n";

static int dramc_print_help(char *buf, int len)
{
	int l = 0;
	int ch_idx = 0;
	int gp_idx = 0;
	int ag_idx = 0;

	l = snprintf(buf, PAGE_SIZE, help);
	for (ch_idx = 0; ch_idx < DRAMC_MX_CHANNUM; ch_idx++) {
		for (gp_idx = 0; gp_idx < DRAMC_MX_GRPNUM; gp_idx++) {
			for (ag_idx = 0; ag_idx < DRAMC_MX_NUM_IN_AGRP; ag_idx++) {
				if (-1 == dramc_desc[ch_idx][gp_idx][ag_idx].agent_id) 
					break;
				if (0 == strncmp("none",  dramc_desc[ch_idx][gp_idx][ag_idx].name, DRAMC_MX_AGENT_NAMEBUF)) 
					continue;
				l += snprintf(buf + l, PAGE_SIZE - l, "  --dramc=toggle:%d:%s\n",ch_idx, dramc_desc[ch_idx][gp_idx][ag_idx].name);
			}
		}
	}
	return l;
}

static int dramc_print_header(char *buf, int len)
{
	int l = 0;
	int ch_idx = 0;
	int gp_idx = 0;
	int ag_idx = 0;

	l += snprintf(buf, PAGE_SIZE, header);
	for (ch_idx = 0; ch_idx < DRAMC_MX_CHANNUM; ch_idx++) {
		l += snprintf(buf + l, PAGE_SIZE - l, "met-info [000] 0.0: met_dramc_all:%d:%s:%x\n", ch_idx, "all_group", DRAMC_ALL_GROUP_AGENT_ID);
		l += snprintf(buf + l, PAGE_SIZE - l, "met-info [000] 0.0: met_dramc_map:%d", ch_idx);
		for (gp_idx = 0; gp_idx < DRAMC_MX_GRPNUM; gp_idx++) {
			l += snprintf(buf + l, PAGE_SIZE - l, ":%d:",gp_idx + 1);
			for (ag_idx = 0; ag_idx < DRAMC_MX_NUM_IN_AGRP; ag_idx++) {
				if (-1 == dramc_desc[ch_idx][gp_idx][ag_idx].agent_id)
					break;
				l += snprintf(buf + l, PAGE_SIZE - l, "%s,", dramc_desc[ch_idx][gp_idx][ag_idx].name);
			}
		}
		l += snprintf(buf + l, PAGE_SIZE - l, "\n");
	}
	return l;
}

static int dramc_set_module_mask(int chann, char* m_name)
{
	int found = 0;
	int gp_idx = 0;
	int ag_idx = 0;
	
	for (gp_idx = 0; gp_idx < DRAMC_MX_GRPNUM; gp_idx++) {
		for (ag_idx = 0; ag_idx < DRAMC_MX_NUM_IN_AGRP; ag_idx++) {
			if (0 == strncmp(m_name, "all", DRAMC_MX_AGENT_NAMEBUF)) {
				dramc_chann[chann].toggle_mask |= (1ULL << DRAMC_ALL_GROUP_AGENT_ID);
				//printk("dramc%d_desc mask: %llX", chann, dramc_chann[chann].toggle_mask);
				found = 1;
			}
			// we do not allow to specify module name "none" 
			if (0 == strncmp("none", dramc_desc[chann][gp_idx][ag_idx].name, DRAMC_MX_AGENT_NAMEBUF))
				continue;
			if (-1 == dramc_desc[chann][gp_idx][ag_idx].agent_id) 
				break;
			if (0 == strncmp(m_name, "all", DRAMC_MX_AGENT_NAMEBUF) || 0 == strncmp(m_name, dramc_desc[chann][gp_idx][ag_idx].name, DRAMC_MX_AGENT_NAMEBUF)) {
				dramc_chann[chann].toggle_mask |= (1ULL << dramc_desc[chann][gp_idx][ag_idx].agent_id);
				//printk("dramc%d_desc mask: %llX", chann, dramc_chann[chann].toggle_mask);
				found = 1;
			}
		}
	}
	return found;
}

/*
 * There are serveral cases as follows:
 *
 * 1. "met-cmd --start --dramc=normal:0:audio"
 *
 */
static int dramc_process_argument(const char *arg, int len)
{
	char* temp = NULL; 
	int chann = 0;
	int target_mode = DRAMC_DISABLE;
	int idx = 0;

	// can be refine 
	if (0 == strncmp("normal", arg, strlen("normal"))) {
		if (':' ==arg[strlen("normal")]){
			char ch = arg[strlen("normal:")];

			chann = ch - '0';	
			if (chann >= 0 && chann < DRAMC_MX_CHANNUM)
				temp = (char*)arg + strlen("normal:x:");
			else
				goto err;
			target_mode = DRAMC_NORMAL;
		}
		else goto err;
	}
	else if (0 == strncmp("toggle", arg, strlen("toggle"))) {
		if (':' ==arg[strlen("toggle")]){
			char ch = arg[strlen("toggle:")];

			chann = ch - '0';	
			if (chann >= 0 && chann < DRAMC_MX_CHANNUM)
				temp = (char*)arg + strlen("toggle:x:");
			else
				goto err;
			target_mode = DRAMC_TOGGLE;
		}
		else goto err;
	}
	else {
		printk("%s %d only support normal or toggle\n", __FUNCTION__, __LINE__);
		goto err;
	}

	if (0 == strncmp(temp, "none", strlen("none"))) {
		printk("%s %d Seriously, why do you set \"none\" as module name??\n", __FUNCTION__, __LINE__);
		goto err;
	}

	// set module mask
	if (0 == dramc_set_module_mask(chann, temp)) {
		printk("%s %d %s is not in dramc channel list %d\n", __FUNCTION__, __LINE__, temp, chann);
		goto err;
	}

	if (DRAMC_NORMAL == target_mode) {
		if (DRAMC_DISABLE != dramc_chann[chann].mode) {
			printk("%s %d set normal and toggle is conflit\n", __FUNCTION__, __LINE__);
			printk("%s %d set normal twice is not allow\n", __FUNCTION__, __LINE__);
			goto err;
		}
		// focus on normal all group agent
		if (0 == strncmp("all",temp, strlen("all")))
			dramc_chann[chann].toggle_mask = (1ULL << DRAMC_ALL_GROUP_AGENT_ID);
		dramc_chann[chann].mode = DRAMC_NORMAL;
	}
	else if (DRAMC_TOGGLE == target_mode) {
		if (DRAMC_NORMAL == dramc_chann[chann].mode) {
			printk("%s %d set normal and toggle is conflit\n", __FUNCTION__, __LINE__);
			goto err;
		}
		dramc_chann[chann].mode = DRAMC_TOGGLE;
	}

	met_dramc.mode = 2;
	return 0;
err:
	// reset dramc_chann value
	for (idx = 0; idx < DRAMC_MX_CHANNUM; idx++) {
		dramc_chann[idx].mode = DRAMC_DISABLE;
		dramc_chann[idx].curr_agent = -1;
		dramc_chann[idx].toggle_mask = 0ULL;
	}
	//met_dramc.mode = 0;
	return -EINVAL;
}

struct metdevice met_dramc = {
	.name = "dramc",
	.owner = THIS_MODULE,
	.type = MET_TYPE_BUS,
	.create_subfs = met_dramc_create,
	.delete_subfs = met_dramc_delete,
	.cpu_related = 0,
	.start = met_dramc_start,
	.stop = met_dramc_stop,
	.polling_interval = 0,//ms
	.timed_polling = met_dramc_polling,
	.tagged_polling = met_dramc_polling,
	.print_help = dramc_print_help,
	.print_header = dramc_print_header,
	.process_argument = dramc_process_argument,
};

