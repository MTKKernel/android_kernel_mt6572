/*****************************************************************************
*                E X T E R N A L      R E F E R E N C E S
******************************************************************************
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/xlog.h>
#include <mach/mt_gpio.h>
#include "yusu_android_speaker.h"

/*****************************************************************************
*                C O M P I L E R      F L A G S
******************************************************************************
*/
//#define CONFIG_DEBUG_MSG
#ifdef CONFIG_DEBUG_MSG
#define PRINTK(format, args...) printk( KERN_EMERG format,##args )
#else
#define PRINTK(format, args...)
#endif

static bool eamp_log_on = true;

#define EAMP_PRINTK(fmt, arg...) \
	do { \
		if (eamp_log_on) xlog_printk(ANDROID_LOG_INFO,"EAMP", "[EAMP]: %s() "fmt"\n", __func__,##arg); \
	}while (0)

#define AMP_CLASS_AB
//#define AMP_CLASS_D
//#define ENABLE_2_IN_1_SPK

#if !defined(AMP_CLASS_AB) && !defined(AMP_CLASS_D)
#error "MT6323 SPK AMP TYPE does not be defined!!!"
#endif
/*****************************************************************************
*                          C O N S T A N T S
******************************************************************************
*/

#define SPK_WARM_UP_TIME        (55) //unit is ms
#define SPK_AMP_GAIN            (4)  //4:15dB
#define RCV_AMP_GAIN            (1)  //1:-3dB
#define SPK_R_ENABLE            (1)
#define SPK_L_ENABLE            (1)
/*****************************************************************************
*                         D A T A      T Y P E S
******************************************************************************
*/
static int Speaker_Volume=0;
static bool gsk_on=false; // speaker is open?
static bool gep_on = false;
static bool ghp_on = false;
//static bool gsk_resume=false;
static bool gsk_forceon=false;

//response time
static int const speaker_response_time = 6; //ms
static int const headphone_response_time = 12; //ms

/*****************************************************************************
*                  F U N C T I O N        D E F I N I T I O N
******************************************************************************
*/
extern void Yusu_Sound_AMP_Switch(BOOL enable);

static ssize_t eamp_openheadPhone(void)
{
	EAMP_PRINTK("eamp_openheadPhone");

	mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ONE);

  	ghp_on = true;

    //msleep(headphone_response_time);

	return 0;
}

static ssize_t eamp_closeheadPhone(void)
{
	EAMP_PRINTK("eamp_closeheadPhone");

	mt_set_gpio_out(GPIO_AUDIO_SEL, GPIO_OUT_ZERO);

	ghp_on = false;

	return 0;
}

static ssize_t eamp_suspend(void)
{
	EAMP_PRINTK("eamp_suspend");
	return 0;
}

static ssize_t eamp_resume(void)
{
	EAMP_PRINTK("eamp_resume");
	if(gsk_on)
	{
		//eamp_openspeaker();
	}
	if(ghp_on)
	{
		eamp_openheadPhone();
	}
	if(gep_on)
	{
		//eamp_openEarpiece();
	}
	return 0;
}

static void eamp_poweron(void)
{
	EAMP_PRINTK("eamp_poweron");

	return;
}

static void eamp_powerdown(void)
{
	EAMP_PRINTK("eamp_powerdown");

	return;
}

static int eamp_init(void)
{
	EAMP_PRINTK("eamp_init");

	eamp_poweron();

	mt_set_gpio_mode(GPIO_AUDIO_SEL, GPIO_AUDIO_SEL_M_GPIO);
	mt_set_gpio_pull_enable(GPIO_AUDIO_SEL, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_AUDIO_SEL, GPIO_DIR_OUT);
	
	return 0;
}

static int eamp_deinit(void)
{
	EAMP_PRINTK("eamp_deinit");
	eamp_powerdown();
	return 0;
}

static int eamp_command( unsigned int  type, unsigned long args,unsigned int count)
{
	EAMP_PRINTK("eamp_command type(%u)",type);
	switch(type)
	{
		case EAMP_SPEAKER_CLOSE:
		{
			//eamp_closespeaker();
			break;
		}
		case EAMP_SPEAKER_OPEN:
		{
			//eamp_openspeaker();
			break;
		}
		case EAMP_HEADPHONE_CLOSE:
		{
			eamp_closeheadPhone();
			break;
		}
		case EAMP_HEADPHONE_OPEN:
		{
			eamp_openheadPhone();
			break;
		}
		case EAMP_EARPIECE_OPEN:
		{
			//eamp_openEarpiece();
			break;
		}
		case EAMP_EARPIECE_CLOSE:
		{
			//eamp_closeEarpiece();
			break;
		}
		case EAMP_GETREGISTER_VALUE:
		{
			//return eamp_getRegister(args);
			break;
		}
		case EAMP_GETAMP_GAIN:
		{
			//return eamp_getGainVolume();
			break;
		}
		case EAMP_SETAMP_GAIN:
		{
			//eamp_changeGainVolume(args);
			break;
		}
		case EAMP_SETREGISTER_VALUE:
		{
			//eamp_setRegister(args);
			break;
		}
		case EAMP_GET_CTRP_NUM:
		{
			//return eamp_getCtrlPointNum();
			break;
		}
		case EAMP_GET_CTRP_BITS:
		{
			//return eamp_getCtrPointBits(args);
			break;
		}
		case EAMP_GET_CTRP_TABLE:
		{
			//eamp_getCtrlPointTable(args);
			break;
		}
		case EAMP_SETMODE:
		{
			//eamp_setMode(args);
		}
		default:
		return 0;
	}
	return 0;
}

bool Speaker_Init(void)
{
   PRINTK("+Speaker_Init Success");
#if defined(AMP_CLASS_AB)

#elif defined(AMP_CLASS_D)

#endif

   eamp_init();

   PRINTK("-Speaker_Init Success");
   return true;
}

bool Speaker_Register(void)
{
    return false;
}

int ExternalAmp(void)
{
	return 0;
}

bool Speaker_DeInit(void)
{
	EAMP_PRINTK("Speaker_DeInit");

	eamp_deinit();
	return true;
}

void Sound_SpeakerL_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerL_SetVolLevel level=%d\n",level);
}

void Sound_SpeakerR_SetVolLevel(int level)
{
   PRINTK(" Sound_SpeakerR_SetVolLevel level=%d\n",level);
}

void Sound_Speaker_Turnon(int channel)
{
    PRINTK("Sound_Speaker_Turnon channel = %d\n",channel);
    if(gsk_on)
        return;
#if defined(ENABLE_2_IN_1_SPK)
#if defined(AMP_CLASS_D)

#endif
#endif
#if defined(AMP_CLASS_AB)

#elif defined(AMP_CLASS_D)

#endif
    //msleep(SPK_WARM_UP_TIME);
    gsk_on = true;
}

void Sound_Speaker_Turnoff(int channel)
{
    PRINTK("Sound_Speaker_Turnoff channel = %d\n",channel);
	if(!gsk_on)
		return;
#if defined(AMP_CLASS_AB)

#elif defined(AMP_CLASS_D)

#endif
	gsk_on = false;
}

void Sound_Speaker_SetVolLevel(int level)
{
    Speaker_Volume =level;
}

void Sound_Headset_Turnon(void)
{
    EAMP_PRINTK("Sound_Headset_Turnon");
    eamp_command(EAMP_HEADPHONE_OPEN,0,1);
}

void Sound_Headset_Turnoff(void)
{
    EAMP_PRINTK("Sound_Headset_Turnoff");
    eamp_command(EAMP_HEADPHONE_CLOSE,0,1);
}

void Sound_Earpiece_Turnon(void)
{
#if defined(ENABLE_2_IN_1_SPK)

#if defined(AMP_CLASS_D)

#endif

#endif
}

void Sound_Earpiece_Turnoff(void)
{
#if defined(ENABLE_2_IN_1_SPK)

#if defined(AMP_CLASS_D)

#endif

#endif
}

//kernal use
void AudioAMPDevice_Suspend(void)
{
#if 0
	PRINTK("AudioDevice_Suspend\n");
	if(gsk_on)
	{
		Sound_Speaker_Turnoff(Channel_Stereo);
		gsk_resume = true;
	}
#else
	EAMP_PRINTK("AudioAMPDevice_Suspend");
	eamp_suspend();
#endif
}
void AudioAMPDevice_Resume(void)
{
#if 0
	PRINTK("AudioDevice_Resume\n");
	if(gsk_resume)
		Sound_Speaker_Turnon(Channel_Stereo);
	gsk_resume = false;
#else
	EAMP_PRINTK("AudioAMPDevice_Resume");
	eamp_resume();
#endif
}
void AudioAMPDevice_SpeakerLouderOpen(void)
{
	PRINTK("AudioDevice_SpeakerLouderOpen\n");
	gsk_forceon = false;
	if(gsk_on)
		return;
	Sound_Speaker_Turnon(Channel_Stereo);
	gsk_forceon = true;
	return ;

}
void AudioAMPDevice_SpeakerLouderClose(void)
{
	PRINTK("AudioDevice_SpeakerLouderClose\n");

	if(gsk_forceon)
		Sound_Speaker_Turnoff(Channel_Stereo);
	gsk_forceon = false;

}
void AudioAMPDevice_mute(void)
{
	PRINTK("AudioDevice_mute\n");
	if(gsk_on)
		Sound_Speaker_Turnoff(Channel_Stereo);
	if(ghp_on)
		eamp_closeheadPhone();

}

int Audio_eamp_command(unsigned int type, unsigned long args, unsigned int count)
{
	EAMP_PRINTK("Audio_eamp_command");

	return eamp_command(type,args,count);
}
static char *ExtFunArray[] =
{
    "InfoMATVAudioStart",
    "InfoMATVAudioStop",
    "End",
};

kal_int32 Sound_ExtFunction(const char* name, void* param, int param_size)
{
	int i = 0;
	int funNum = -1;

	//Search the supported function defined in ExtFunArray
	while(strcmp("End",ExtFunArray[i]) != 0 ) {		//while function not equal to "End"

	    if (strcmp(name,ExtFunArray[i]) == 0 ) {		//When function name equal to table, break
	    	funNum = i;
	    	break;
	    }
	    i++;
	}

	switch (funNum) {
	    case 0:			//InfoMATVAudioStart
	        printk("RunExtFunction InfoMATVAudioStart \n");
	        break;

	    case 1:			//InfoMATVAudioStop
	        printk("RunExtFunction InfoMATVAudioStop \n");
	        break;

	    default:
	    	 break;
	}

	return 1;
}
