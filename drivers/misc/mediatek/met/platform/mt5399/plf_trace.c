#include <linux/kernel.h>
#include <linux/module.h>

#include "core/trace.h"


//#define DRAMC_REAL_TIMESTAMP
#ifndef DRAMC_REAL_TIMESTAMP
#define TV_DRAMC_FMT	""
#define TV_DRAMC_UD_FMT	"%x,%x,%x,%x\n"
#define TV_DRAMC_UD_VAL chann,value[0],value[1],value[2]
#else
#define TV_DRAMC_FMT	"%5lu.%06lu"
#define TV_DRAMC_UD_FMT	",%x,%x,%x,%x\n"
#define TV_DRAMC_UD_VAL (unsigned long)(timestamp),nano_rem/1000,chann,value[0],value[1],value[2]
#endif
void tv_dramc(unsigned long long timestamp, int chann, unsigned int *value)
{
#ifdef DRAMC_REAL_TIMESTAMP
	unsigned long nano_rem = do_div(timestamp, 1000000000);
#endif

	trace_printk(TV_DRAMC_FMT TV_DRAMC_UD_FMT, TV_DRAMC_UD_VAL);
}

#ifndef DRAMC_REAL_TIMESTAMP
#define TV_DRAMC_OVERFLOW_FMT	"%x\n"	
#define TV_DRAMC_OVERFLOW_VAL	 chann
#else
#define TV_DRAMC_OVERFLOW_FMT	",%x,%x\n"	
#define TV_DRAMC_OVERFLOW_VAL	 (unsigned long)(timestamp),nano_rem/1000,chann
#endif
void tv_dramc_overflow(unsigned long long timestamp, int chann)
{
#ifdef DRAMC_REAL_TIMESTAMP
	unsigned long nano_rem = do_div(timestamp, 1000000000);
#endif

	trace_printk(TV_DRAMC_FMT TV_DRAMC_OVERFLOW_FMT,TV_DRAMC_OVERFLOW_VAL);
}


#define MP_2P_FMT	"%5lu.%06lu"
#define MP_2P_VAL	(unsigned long)(timestamp), nano_rem/1000
void mp_2p(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 1: trace_printk(MP_2P_FMT FMT1, MP_2P_VAL VAL1); break;
	case 2: trace_printk(MP_2P_FMT FMT2, MP_2P_VAL VAL2); break;
	case 3: trace_printk(MP_2P_FMT FMT3, MP_2P_VAL VAL3); break;
	case 4: trace_printk(MP_2P_FMT FMT4, MP_2P_VAL VAL4); break;
	case 5: trace_printk(MP_2P_FMT FMT5, MP_2P_VAL VAL5); break;
	case 6: trace_printk(MP_2P_FMT FMT6, MP_2P_VAL VAL6); break;
	case 7: trace_printk(MP_2P_FMT FMT7, MP_2P_VAL VAL7); break;
	case 8: trace_printk(MP_2P_FMT FMT8, MP_2P_VAL VAL8); break;
	case 9: trace_printk(MP_2P_FMT FMT9, MP_2P_VAL VAL9); break;
	}
}

void mp_2pr(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 1: trace_printk(MP_2P_FMT FMT1, MP_2P_VAL VAL1); break;
	case 2: trace_printk(MP_2P_FMT FMT2, MP_2P_VAL VAL2); break;
	case 3: trace_printk(MP_2P_FMT FMT3, MP_2P_VAL VAL3); break;
	case 4: trace_printk(MP_2P_FMT FMT4, MP_2P_VAL VAL4); break;
	case 5: trace_printk(MP_2P_FMT FMT5, MP_2P_VAL VAL5); break;
	case 6: trace_printk(MP_2P_FMT FMT6, MP_2P_VAL VAL6); break;
	case 7: trace_printk(MP_2P_FMT FMT7, MP_2P_VAL VAL7); break;
	case 8: trace_printk(MP_2P_FMT FMT8, MP_2P_VAL VAL8); break;
	case 9: trace_printk(MP_2P_FMT FMT9, MP_2P_VAL VAL9); break;
	}
}

void mp_2pw(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 1: trace_printk(MP_2P_FMT FMT1, MP_2P_VAL VAL1); break;
	case 2: trace_printk(MP_2P_FMT FMT2, MP_2P_VAL VAL2); break;
	case 3: trace_printk(MP_2P_FMT FMT3, MP_2P_VAL VAL3); break;
	case 4: trace_printk(MP_2P_FMT FMT4, MP_2P_VAL VAL4); break;
	case 5: trace_printk(MP_2P_FMT FMT5, MP_2P_VAL VAL5); break;
	case 6: trace_printk(MP_2P_FMT FMT6, MP_2P_VAL VAL6); break;
	case 7: trace_printk(MP_2P_FMT FMT7, MP_2P_VAL VAL7); break;
	case 8: trace_printk(MP_2P_FMT FMT8, MP_2P_VAL VAL8); break;
	case 9: trace_printk(MP_2P_FMT FMT9, MP_2P_VAL VAL9); break;
	}
}

#define MS_EMI_FMT	"%5lu.%06lu"
#define MS_EMI_VAL	(unsigned long)(timestamp), nano_rem/1000
void ms_emi(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 1: trace_printk(MS_EMI_FMT FMT1, MS_EMI_VAL VAL1); break;
	case 2: trace_printk(MS_EMI_FMT FMT2, MS_EMI_VAL VAL2); break;
	case 3: trace_printk(MS_EMI_FMT FMT3, MS_EMI_VAL VAL3); break;
	case 4: trace_printk(MS_EMI_FMT FMT4, MS_EMI_VAL VAL4); break;
	case 5: trace_printk(MS_EMI_FMT FMT5, MS_EMI_VAL VAL5); break;
	case 6: trace_printk(MS_EMI_FMT FMT6, MS_EMI_VAL VAL6); break;
	case 7: trace_printk(MS_EMI_FMT FMT7, MS_EMI_VAL VAL7); break;
	case 8: trace_printk(MS_EMI_FMT FMT8, MS_EMI_VAL VAL8); break;
	case 9: trace_printk(MS_EMI_FMT FMT9, MS_EMI_VAL VAL9); break;
	}
}

#define MS_SMI_FMT	"%5lu.%06lu"
#define MS_SMI_VAL	(unsigned long)(timestamp), nano_rem/1000
void ms_smi(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 30: trace_printk(MS_SMI_FMT FMT30, MS_SMI_VAL VAL30); break;
	case 37: trace_printk(MS_SMI_FMT FMT37, MS_SMI_VAL VAL37); break;
	case 44: trace_printk(MS_SMI_FMT FMT44, MS_SMI_VAL VAL44); break;
	}
}

void ms_smit(unsigned long long timestamp, unsigned char cnt, unsigned int *value)
{
	unsigned long nano_rem = do_div(timestamp, 1000000000);
	switch (cnt) {
	case 10: trace_printk(MS_SMI_FMT FMT10, MS_SMI_VAL VAL10); break;
	}
}
