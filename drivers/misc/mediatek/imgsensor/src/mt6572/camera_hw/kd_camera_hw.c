#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
 ******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
//#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_DBG_FUNC printk

#define DEBUG_CAMERA_HW_K
#ifdef  DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_XLOG_INFO(fmt, args...) \
	do {    \
		xlog_printk(ANDROID_LOG_INFO, "kd_camera_hw", fmt, ##args); \
	} while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif
extern void main_cam_dvdd_power(int on_off); // 1.2V
extern void cam_avdd_power(int on_off); // 2.8V
extern void cam_iovdd_power(int on_off); // 1.8V
extern void hi707_cam_power(int on_off); //hi707 power
extern void hi351_cam_power(int on_off); //hi351 power

extern void ISP_MCLK1_EN(bool En);
extern int g_lge_camera; // 0:3MP, 1:5MP //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check

static void MainCameraDigtalPowerCtrl(kal_bool on){
#if 0 // M4_TEMP
	if(mt_set_gpio_mode(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,0)){PK_DBG("[[CAMERA SENSOR] Set MAIN CAMERA_DIGITAL POWER_PIN ! \n");}
	if(mt_set_gpio_dir(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");}
	if(mt_set_gpio_out(GPIO_MAIN_CAMERA_12V_POWER_CTRL_PIN,on)){PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n");;}
#endif
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;//default main sensor
	u32 pinSetIdxTmp = 0;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3
	u32 pinSet[2][8] = {
		//for main sensor
		{GPIO_MAIN_CAM0_RESET_N,
			GPIO_MAIN_CAM0_RESET_N_M_GPIO,   /* mode */
			GPIO_OUT_ONE,                   /* ON state */
			GPIO_OUT_ZERO,                  /* OFF state */
			GPIO_MAIN_CAM0_PWDN,
			GPIO_MAIN_CAM0_PWDN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		},
		//for sub sensor
		{GPIO_VT_CAM_RESET_N,
			GPIO_VT_CAM_RESET_N_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
			GPIO_VT_CAM_PWDN,
			GPIO_VT_CAM_PWDN_M_GPIO,
			GPIO_OUT_ONE,
			GPIO_OUT_ZERO,
		}
	};

	//[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check
	printk("kdCISModulePowerOn: g_lge_camera:%d [0:hi351, 1:isx012, -1:Unknown]\n",g_lge_camera);
	PK_DBG("kdCISModulePowerOn: 4EC 8AA\n");

	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
		pinSetIdx = 0;
	}
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
		pinSetIdx = 1;
	}

	//power ON
	if (On)
	{
		PK_DBG("kdCISModulePowerOn -on:currSensorName=%s;\n",currSensorName);
		MainCameraDigtalPowerCtrl(1);
		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_ISX012_MIPI_YUV,currSensorName))&& (pinSetIdx==0))
		{
			if(g_lge_camera == 0) return 0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check
			PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---ISX012_MIPI_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			ISP_MCLK1_EN(FALSE);
			mdelay(10); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-30] MCLK set to low when MCLK is disabled.
			main_cam_dvdd_power(TRUE);
			mdelay(1);
			cam_iovdd_power(TRUE);
			mdelay(3);
			cam_avdd_power(TRUE);
			mdelay(5);
			//cam_vcm_power(TRUE);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

			mdelay(5);
			ISP_MCLK1_EN(TRUE);

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mdelay(5);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]);
			/* ISX012 CMPDN is controlled at the driver file. */
			//mdelay(10);
			//mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			//mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			//mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			//mdelay(1);
			//mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]);
		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI707_YUV,currSensorName)))
		{
			PK_DBG("SENSOR_DRVNAME_HI707_YUV power!! \n");

			mt_set_gpio_mode(GPIO87,GPIO_MODE_00);
			mt_set_gpio_dir(GPIO87,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO87,GPIO_OUT_ZERO);
			mt_set_gpio_mode(GPIO88,GPIO_MODE_00);
			mt_set_gpio_dir(GPIO88,GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO88,GPIO_OUT_ZERO);

			mt_set_gpio_mode(GPIO67,GPIO_MODE_01);
			mt_set_gpio_dir(GPIO67,GPIO_DIR_IN);
			mt_set_gpio_mode(GPIO68,GPIO_MODE_01);
			mt_set_gpio_dir(GPIO68,GPIO_DIR_IN);
			mdelay(2);

			mt_set_gpio_mode(GPIO71,GPIO_MODE_01);
			mt_set_gpio_dir(GPIO71,GPIO_DIR_IN);
			mt_set_gpio_mode(GPIO72,GPIO_MODE_01);
			mt_set_gpio_dir(GPIO72,GPIO_DIR_IN);
			mdelay(2);

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);

#if 0
			//ISP_MCLK1_EN(FALSE);
			cam_iovdd_power(false);
			cam_avdd_power(false);
			mdelay(2);

			cam_iovdd_power(TRUE);
			mdelay(1);
			cam_avdd_power(TRUE);
			mdelay(1);
			//ISP_MCLK1_EN(TRUE);
#else
			ISP_MCLK1_EN(FALSE);
			mdelay(10); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-30] MCLK set to low when MCLK is disabled.
			hi707_cam_power(TRUE);
			mdelay(1);
			ISP_MCLK1_EN(TRUE);
#endif

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mdelay(1);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			mdelay(1);

			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]);
			mdelay(30);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]);
			mdelay(1);
		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI351_MIPI_YUV,currSensorName))&& (pinSetIdx==0))
		{
			if(g_lge_camera == 1) return 0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check
			PK_DBG("[CAMERA SENSOR] kdCISModulePowerOn get in---SENSOR_DRVNAME_HI351_MIPI_YUV sensorIdx:%d; pinSetIdx=%d\n",SensorIdx, pinSetIdx);

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			ISP_MCLK1_EN(FALSE);
			mdelay(10); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-30] MCLK set to low when MCLK is disabled.

#if 0
			cam_iovdd_power(TRUE);
			//mdelay(1);
			cam_avdd_power(TRUE);
			//mdelay(1);
			//cam_vcm_power(TRUE);
			//mdelay(5);
			main_cam_dvdd_power(TRUE);
			//mdelay(5);
#else
			hi351_cam_power(TRUE);
#endif
			mdelay(2);
			ISP_MCLK1_EN(TRUE);

			//PDN/STBY pin
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mdelay(2);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON]);
			mdelay(30);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON]);
			mdelay(1);
		}
	}
	else
	{//power OFF
		PK_DBG("kdCISModulePowerOn -off:currSensorName=%s\n",currSensorName);

		if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_ISX012_MIPI_YUV,currSensorName)))
		{
			if(g_lge_camera == 0) return 0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			ISP_MCLK1_EN(FALSE);

			main_cam_dvdd_power(FALSE);
			cam_iovdd_power(FALSE);
			cam_avdd_power(FALSE);
			//cam_vcm_power(FALSE);
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
				//return -EIO;
				goto _kdCISModulePowerOn_exit_;
			}

		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI707_YUV,currSensorName)))
		{
			PK_DBG("SENSOR_DRVNAME_HI707_YUV power off !! \n");

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mdelay(30);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
			mdelay(1);
			ISP_MCLK1_EN(FALSE);
			mdelay(1);
#if 0
			cam_iovdd_power(false);
			mdelay(1);
			cam_avdd_power(false);
#else
			hi707_cam_power(false);
#endif
		}
		else if (currSensorName && (0 == strcmp(SENSOR_DRVNAME_HI351_MIPI_YUV,currSensorName)))
		{
			if(g_lge_camera == 1) return 0; //[LGE_UPDATE][yonghwan.lym@lge.com][2014-04-22] SBP Camera Sendor Check

			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF]);
			mdelay(1);
			ISP_MCLK1_EN(FALSE);
			mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE]);
			mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT);
			mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF]);
#if 0
			cam_iovdd_power(FALSE);
			cam_avdd_power(FALSE);
			main_cam_dvdd_power(FALSE);
#else
			hi351_cam_power(false);
#endif
			//cam_vcm_power(FALSE);
		}
	}

	return 0;
_kdCISModulePowerOn_exit_:
	return -EIO;
}


EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//
