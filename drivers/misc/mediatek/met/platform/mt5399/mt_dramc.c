//#include "mt_clkmgr.h"

#include "mt_dramc.h"
#include "sync_write.h"
//#define DRAMC_DEBUG
#ifndef DRAMC_DEBUG
#define DRAMC_LOG(fmt, ...)
#else
#define DRAMC_LOG(fmt, arg...) printk("%s:%d:"fmt, __FILE__, __LINE__,##arg);
#endif
struct dramc_desc_t dramc_desc[DRAMC_MX_CHANNUM][DRAMC_MX_GRPNUM][DRAMC_MX_NUM_IN_AGRP] =
{
	///////////////////////////////////////////////////////////
	// define 1st channel releationship between group and name
	///////////////////////////////////////////////////////////
	{
		{
			// GRP 1
			{0, 	"audio"},
			{1, 	"demux/gcpu"},
			{2, 	"vbi/3d/tve/demod"},
			{3, 	"osd/od/mmu"},
			{4, 	"mib"},
			{5, 	"b2r"},
			{6, 	"cpu"},
			{7, 	"scpos/od_table"},
			{8,		"vdec_mc"},
			{9, 	"vld1"},
			{10, 	"3d_gpu"},
			{11,	"2d_graph/jpgdec/imgrsz/png"},
			{12,	"venc"},
			{13,	"mjc_in"},
			{14,	"mjc_out/pr_gen"},
			{15,	"test0/arm11/audio_dsp_0"},
			{-1,	""}, // GRP END
		},
		{
			// GRP 2
			{16,	"usb2"},
			{17,	"ethernet"},
			{18,	"demux/ddi/gcpu"},
			{19,	"usb3"},
			{20,	"audio_dsp0"},
			{21,	"msdc"},
			{22,	"none"},
			{23,	"gdma"},
			{-1,	""}, //GRP END
		},
		{
			// GRP 3
			{24,	"2d_graph"},
			{25,	"nfi_dma/sfalsh_dma/lzhs/ci_spi"},
			{26,	"ether_chksum"},
			{27,	"rs232"},
			{28,	"spis"},
			{29,	"none"},
			{30,	"none"},
			{31,	"none"},
			{-1,	""}, //GRP END
		},
	},
	///////////////////////////////////////////////////////////
	// define 2nd channel releationship between group and name
	///////////////////////////////////////////////////////////
	{
		{
			// GRP 1
			{0, 	"none"},
			{1, 	"demux/gcpu"},
			{2, 	"vbi/3d/tve/demod"},
			{3, 	"osd/od/mmu"},
			{4, 	"mib"},
			{5, 	"pr_gen"},
			{6, 	"cpu"},
			{7, 	"scpos/od_table"},
			{8,		"demod"},
			{9, 	"none"},
			{10, 	"3d_gpu"},
			{11,	"2d_graph/jpgdec/imgrsz/png"},
			{12,	"venc"},
			{13,	"mjc_in"},
			{14,	"mjc_out"},
			{15,	"test0"},
			{-1,	""}, // GRP END
		},
		{
			// GRP 2
			{16,	"usb2"},
			{17,	"ethernet1"},
			{18,	"demux/ddi/gcpu"},
			{19,	"usb3"},
			{20,	"ethernet2"},
			{21,	"demux/gcpu"},
			{22,	"none"},
			{23,	"gdma"},
			{-1,	""}, //GRP END
		},
		{
			// GRP 3
			{24,	"none"},
			{25,	"none"},
			{26,	"none"},
			{27,	"rs232"},
			{28,	"none"},
			{29,	"none"},
			{30,	"none"},
			{31,	"none"},
			{-1,	""}, //GRP END
		},
	},
	// define end
};

#ifndef DRAMC_FAKE_REPORT
static unsigned long dramc_base[DRAMC_MX_CHANNUM] =
{
	MET_DRAMC0_BASE,
	MET_DRAMC1_BASE
};

static inline unsigned int dramc_reg_read(unsigned int addr)
{
	unsigned int value = readl(addr);
	mb();
	return value;
}

static inline void dramc_reg_write(unsigned int addr, unsigned int value)
{
	writel(value, addr);
	// make sure writel() be completed before outer_sync()
	mb();
	outer_sync();
	// make sure outer_sync() be completed before following readl();
	mb();
}

#define DRAMC_SET_VALUE(target, value, shift, bit) \
do { \
	volatile u32 temp = dramc_reg_read(target); \
	u32 mask1 = (~(((0xFFFFFFFF >> (32 - bit))<< shift))); \
	u32 mask2 = ((0xFFFFFFFF >> (32 - bit))<< shift); \
	dramc_reg_write(target,(temp & mask1) | ((value << shift) & mask2)); \
} while (0)

void DRAMC_Init(int chan)
{
	unsigned long base = dramc_base[chan];
	volatile u32 temp_reg = 0; 

	// disable all group
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP1_ENABLE_SHIFT, 1);
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP2_ENABLE_SHIFT, 1);
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP3_ENABLE_SHIFT, 1);
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_FREEZE_SHIFT, 1);
	temp_reg = dramc_reg_read(DRAMC_MON_BM(base));
	DRAMC_LOG("%s MON_BM: %X\n", __FUNCTION__, temp_reg);
}


void DRAMC_Enable(int chan, int group, int agent)
{
	unsigned long base = dramc_base[chan];
	volatile u32 temp_reg = 0; 

	// disable all group
	switch (group) {
		case 1: {
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), agent, DRAMC_BM_GROUP1_AGENT_ID_SHIFT, DRAMC_BM_GROUP1_AGENT_ID_LEN);
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), 1, DRAMC_BM_GROUP1_ENABLE_SHIFT, 1);
		}
		break;
		case 2: {
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), agent, DRAMC_BM_GROUP2_AGENT_ID_SHIFT, DRAMC_BM_GROUP3_AGENT_ID_LEN);
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), 1, DRAMC_BM_GROUP2_ENABLE_SHIFT, 1);
		}
		break;
		case 3: {
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), agent, DRAMC_BM_GROUP3_AGENT_ID_SHIFT, DRAMC_BM_GROUP3_AGENT_ID_LEN);
		DRAMC_SET_VALUE(DRAMC_MON_BM(base), 1, DRAMC_BM_GROUP3_ENABLE_SHIFT, 1);
		}
		break;
	}
	temp_reg = dramc_reg_read(DRAMC_MON_BM(base));
	DRAMC_LOG("%s MON_BM: %X\n", __FUNCTION__, temp_reg);
}


void DRAMC_Disable(int chan)
{
	unsigned long base = dramc_base[chan];
	volatile u32 temp_reg = 0; 

	// disable all group
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP1_ENABLE_SHIFT, 1);
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP2_ENABLE_SHIFT, 1);
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_GROUP3_ENABLE_SHIFT, 1);
	// fire the BM function
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 0, DRAMC_BM_FREEZE_SHIFT, 1);
	temp_reg = dramc_reg_read(DRAMC_MON_BM(base));
	DRAMC_LOG("%s MON_BM: %X\n", __FUNCTION__, temp_reg);
}

void DRAMC_Freeze(int chan)
{
	unsigned long base = dramc_base[chan];
	volatile u32 temp_reg = dramc_reg_read(DRAMC_MON_BM(base));

	// Freeze the BM function
	DRAMC_SET_VALUE(DRAMC_MON_BM(base), 1, DRAMC_BM_FREEZE_SHIFT, 1);
	temp_reg = dramc_reg_read(DRAMC_MON_BM(base));
	DRAMC_LOG("%s MON_BM: %X\n", __FUNCTION__, temp_reg);
}

void DRAMC_ConfigTargetCount(int chan, u32 count)
{
	unsigned long base = dramc_base[chan];
	volatile u32 temp_reg = 0;

	dramc_reg_write(DRAMC_MON_BMCYC(base), 0xFFFFFFFF);
	temp_reg = dramc_reg_read(DRAMC_MON_BMCYC(base));
	DRAMC_LOG("%s MON_BMCYC: %X\n", __FUNCTION__, temp_reg);
	return;
}
u32 DRAMC_GetCycleCount(int chan, int group)
{
	unsigned long base = dramc_base[chan];

	switch (group) {
		case 1:
			return dramc_reg_read(DRAMC_MON_ROBM0(base));
		break;
		case 2:
			return dramc_reg_read(DRAMC_MON_ROBM1(base));
		break;
		case 3:
			return dramc_reg_read(DRAMC_MON_ROBM2(base));
		break;
	}
	return 0;
}

u32 DRAMC_GetTotalCycleCount(int chan)
{
	unsigned long base = dramc_base[chan];

	return dramc_reg_read(DRAMC_MON_ROBM3(base));
}

int DRAMC_CheckCntIsOverFlow(u32 count) 
{
	if (0xFFFFFFFF == count) {
		return 1;
	}
	return 0;
}

#else
#include <linux/random.h>
char rnd_cyc[4];
char rnd_tcyc[4];
void DRAMC_Init(int chan)
{
}

void DRAMC_Enable(int chan, int group, int agent)
{
}


void DRAMC_Disable(int chan)
{
}

void DRAMC_Freeze(int chan)
{
}


void DRAMC_ConfigTargetCount(int chan, u32 count)
{
	return;
}

u32 DRAMC_GetCycleCount(int chan, int group)
{
	u32* count = (void*)rnd_cyc;

	get_random_bytes(rnd_cyc, sizeof(rnd_cyc));
	return (*count)%0xFFFF;
}

u32 DRAMC_GetTotalCycleCount(int chan)
{
	u32* count = (void*)rnd_tcyc;
	get_random_bytes(rnd_tcyc, sizeof(rnd_tcyc));
	return (*count)%0xFFFFFF;
}
int DRAMC_CheckCntIsOverFlow(u32 count) 
{
	return 0;
}
#endif
