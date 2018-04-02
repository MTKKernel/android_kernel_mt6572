#ifndef _DRAMC_H_
#define _DRAMC_H_

#include <linux/device.h>
#define DRAMC_MX_TOGGLE_BUF (1024)
enum DRAMC_IS_VALID 	{DRAMC_IS_VALID=0, DRAMC_IS_INVALID=1};
struct dramc_bm_unit_t
{
	enum DRAMC_IS_VALID is_valid;
	unsigned int bm_cnt;
	unsigned int total_bm_cnt;
};
//static struct dramc_bm_unit_t dramc_bm_unit[DRAMC_MX_GRPNUM][DRAMC_MX_NUM_IN_AGRP];

enum DRAMC_MODE 	{DRAMC_DISABLE=0,DRAMC_NORMAL=1, DRAMC_TOGGLE=2};
struct dramc_chann_t
{	
	enum DRAMC_MODE mode; // support toggle or normal mode
	int curr_agent;
	u64 toggle_mask;
};

//static struct dramc_chan_t dram_chan[DRAMC_MX_CHANNUM];


/*
struct smi_cfg {
	unsigned long master;
	unsigned long port;
	unsigned long rwtype;//0 for R+W, 1 for read, 2 for write
	unsigned long desttype;//0 for EMI+internal mem, 1 for EMI, 3 for internal mem
	unsigned long bustype;//0 for GMC, 1 for AXI
	//unsigned long requesttype;// 0:All, 1:ultra high, 2:pre-ultrahigh, 3:normal.
};

struct smi_mon_con {
	unsigned long requesttype;// 0:All, 1:ultra high, 2:pre-ultrahigh, 3:normal.
};

void smi_init(void);
void smi_uninit(void);

void smi_start(void);
void smi_stop(void);

int do_smi(void);
unsigned int smi_polling(unsigned int *smi_value);
*/

#endif // _DRAMC_H_

