#ifndef __MT_DRAMC_H__
#define __MT_DRAMC_H__
#include <linux/types.h>

// define BM function base address
#define MET_DRAMC0_BASE 	0xF0006000
#define MET_DRAMC1_BASE 	0xF0010000

// define BM function offset
#define DRAMC_MON_BM(i)		(i + 0x80)
#define DRAMC_MON_BMCYC(i)	(i + 0x8C)
#define DRAMC_MON_ROBM0(i)	(i + 0x90)
#define DRAMC_MON_ROBM1(i)	(i + 0x94)
#define DRAMC_MON_ROBM2(i)	(i + 0x98)
#define DRAMC_MON_ROBM3(i)	(i + 0x9C)

// define BM function shift
#define DRAMC_BM_FREEZE_SHIFT			(28)
#define DRAMC_BM_GROUP1_AGENT_ID_SHIFT	(8)
#define DRAMC_BM_GROUP1_AGENT_ID_LEN	(5)
#define DRAMC_BM_GROUP1_ENABLE_SHIFT	(15)
#define DRAMC_BM_GROUP2_AGENT_ID_SHIFT	(16)
#define DRAMC_BM_GROUP2_AGENT_ID_LEN	(3)
#define DRAMC_BM_GROUP2_ENABLE_SHIFT	(19)
#define DRAMC_BM_GROUP3_AGENT_ID_SHIFT	(20)
#define DRAMC_BM_GROUP3_AGENT_ID_LEN	(3)
#define DRAMC_BM_GROUP3_ENABLE_SHIFT	(23)

// generate random cycle to return fake report
//#define DRAMC_FAKE_REPORT

// define dramc spec
#define DRAMC_MX_CHANNUM			(2)
#define DRAMC_MX_GRPNUM				(3)
#define DRAMC_MX_NUM_IN_AGRP		(32)
#define DRAMC_MX_AGENT_NAMEBUF		(64)

// define all group id
#define DRAMC_ALL_GROUP_AGENT_ID	(0x20)

struct dramc_desc_t {
	int agent_id;
	char name[DRAMC_MX_AGENT_NAMEBUF];
};

// some marco and static inline function define
static inline int FIND_NEXT_AGENT(u64 mask, int agent)
{
	int idx = 0;
	int mx_cnt = sizeof(u64)*8;
	int temp_agent = agent + 1;

	for (idx = 0 ; idx < mx_cnt; idx++,temp_agent++) {
		if (mx_cnt == temp_agent)
			temp_agent = 0;
		if ((1ULL<<temp_agent) & mask)
			break;
	}
	if (idx == mx_cnt)	
		return -1;
	return temp_agent;
}
	
#define GROUP_ID(agent) ((agent <= 0xF)?1:((agent<=17)?2:3))



// function prototype
extern void DRAMC_Init(int chan);
extern void DRAMC_Enable(int chan, int group, int agent);
extern void DRAMC_Disable(int chan);
extern void DRAMC_Freeze(int chan);
extern void DRAMC_ConfigTargetCount(int chan, u32 count);
extern u32 DRAMC_GetCycleCount(int chan, int group);
extern u32 DRAMC_GetTotalCycleCount(int chan);
extern int DRAMC_CheckCntIsOverFlow(u32 count);
#endif
