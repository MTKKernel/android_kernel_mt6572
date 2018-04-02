#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/mtk_rtc.h>
/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
#include <asm/io.h>
#include "mach/devs.h"
//#include "mach/mt_gpio.h"
#include "mach/mt_reg_base.h"
#include "mach/mt_gpio_base.h"

//#include "../../../drivers/video/console/fbcon.h"
#include <mach/fbcon.h>

/*                                                                   */
#include <linux/fb.h>
#include "../../../drivers/misc/mediatek/video/mtkfb.h"
#include "../../../drivers/misc/mediatek/video/disp_drv.h"
/*                                                                   */
/* #include <video/dpi_reg.h>*/
#include <linux/disp_assert_layer.h>

//#include "video/lcd_drv.h"
//#include "../../../drivers/misc/mediatek/video/mt6572/disp_drv.h"

#include <mach/mt_gpio.h>

#include <cust_vibrator.h>
#endif
/*                                                                      */

extern void wdt_arch_reset(char);

#if 1  /*                                                       */
	/*                                                 */
#ifndef CONFIG_LOCAL_WDT
	enum wk_wdt_type {
		WK_WDT_LOC_TYPE,
		WK_WDT_EXT_TYPE,
		WK_WDT_LOC_TYPE_NOLOCK,
		WK_WDT_EXT_TYPE_NOLOCK,
	};

	extern void mtk_wdt_restart(enum wk_wdt_type type);
#endif

	/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
	static struct fbcon_config mipi_fb_cfg =
	{
		.width     = 256,
		.height     = 320,
		.stride     = 256, /* .stride  = (240 + 31) & (~31), */
	    .format         = FB_FORMAT_RGB888,
	    .bpp            = 32,
	    .update_start   = NULL,
	    .update_done    = NULL,
	};

	struct crash_log_dump {
		unsigned int magic_key;
		unsigned int size;
		unsigned char buffer[0];
	};

	static struct crash_log_dump *crash_dump_log;
	static unsigned long *cpu_crash_ctx = NULL;

	static volatile GPIO_REGS *reg_gpio = (GPIO_REGS*) GPIO_BASE;
	volatile u16 key_input_val = 0;

	extern struct fb_info *mtkfb_fbi;

#endif
	/*                                                                      */

	/*                                                           */
#if defined(CONFIG_LGE_HIDDEN_RESET)
	extern int hreset_enable;
#endif
	/*                                                                             */
	extern void DISP_Crash_UpdateScreen(void);
	extern void fbcon_setup(struct fbcon_config *cfg);
	extern void fbcon_puts(char *str);
#if 1  /*                                                                      */
	extern void DISP_display_crash(void);
	extern void mtkfb_crash_late_resume();
#endif  /*                                                                      */

#if 1  /*                                                       */
struct lge_crash_dal_setcolor {
	unsigned int foreground;
	unsigned int background;
};

#endif /*              */

	void set_crash_display(void)
	{
#if defined(TARGET_S4)
	char szMsg[1024];

	DAL_SetLGE_CrashScreen();

	DAL_Printf("========================================\n");
	DAL_Printf("Kernel Crash!\n");
	DAL_Printf("========================================\n");
	DAL_Printf(" \n");
	DAL_Printf("Please do following action. \n");
	DAL_Printf("  1) Connect USB\n");
	DAL_Printf("  2) Press key to Volumn Up or Down \n");
	DAL_Printf("  3) Get the ram dump image \n");
	DAL_Printf("     by using MTK's ram dump downloader \n");
	DAL_Printf("     ex)mtk_ramdump.exe [USB port num]\n");
	DAL_Printf(" \n");
	memset (szMsg, 0, sizeof(szMsg));
	sprintf(szMsg, " R0 : 0x%08X  R1 : 0x%08X  R2 : 0x%08X \n", *cpu_crash_ctx, *(cpu_crash_ctx + 1), *(cpu_crash_ctx + 2));
	DAL_Printf(szMsg);
	sprintf(szMsg, " R3 : 0x%08X  R4 : 0x%08X  R5 : 0x%08X \n", *(cpu_crash_ctx + 3), *(cpu_crash_ctx + 4), *(cpu_crash_ctx + 5));
	DAL_Printf(szMsg);
	sprintf(szMsg, " R6 : 0x%08X  R7 : 0x%08X  R8 : 0x%08X \n", *(cpu_crash_ctx + 6), *(cpu_crash_ctx + 7), *(cpu_crash_ctx + 8));
	DAL_Printf(szMsg);
	sprintf(szMsg, " R9 : 0x%08X R10 : 0x%08X R11 : 0x%08X \n", *(cpu_crash_ctx + 9), *(cpu_crash_ctx + 10), *(cpu_crash_ctx + 11));
	DAL_Printf(szMsg);
	sprintf(szMsg, "R12 : 0x%08X  SP : 0x%08X  LR : 0x%08X \n", *(cpu_crash_ctx + 12), *(cpu_crash_ctx + 13), *(cpu_crash_ctx + 14));
	DAL_Printf(szMsg);
	sprintf(szMsg, " PC : 0x%08X  CPSR : 0x%08X \n", *(cpu_crash_ctx + 15), *(cpu_crash_ctx + 16));
	DAL_Printf(szMsg);
	sprintf(szMsg, "CTRL : 0x%08X TRANSBASE : 0x%08X  DAC : 0x%08X \n", *(cpu_crash_ctx + 17), *(cpu_crash_ctx + 18), *(cpu_crash_ctx + 19));
	DAL_Printf(szMsg);
	DAL_Printf("----------------------------------------\n");
	if(crash_dump_log->buffer != NULL) {
		memcpy(szMsg, crash_dump_log->buffer, sizeof(szMsg));
		szMsg[sizeof(szMsg)-1] = '\0';
		DAL_Printf(szMsg);
	}
#else
	char szMsg[128];
	int i;
	struct mtkfb_device    *fbdev = NULL;
	fbdev = (struct mtkfb_device *) mtkfb_fbi->par;

	//for ( i = 0; i < 3; i++) {
	//mipi_fb_cfg.base	= (void*) ioremap_nocache((unsigned int) fbdev->fb_pa_base + ((mipi_fb_cfg.width * mipi_fb_cfg.height*4)*i),
	//											 (mipi_fb_cfg.width * mipi_fb_cfg.height * (mipi_fb_cfg.bpp / 8)));

	fbcon_setup(&mipi_fb_cfg);

	fbcon_puts("========================================\n");
	fbcon_puts("Kernel Crash!\n");
	fbcon_puts("========================================\n");
	//fbcon_puts("Crash Handler : Kernel Crash! \n");
	fbcon_puts(" \n");
	fbcon_puts("Please do following action. \n");
	fbcon_puts("  1) Connect USB\n");
	fbcon_puts("  2) Press key to Volumn Up or Down \n");
	fbcon_puts("  3) Get the ram dump image \n");
	fbcon_puts("     by using MTK's ram dump downloader \n");
	fbcon_puts("     ex)mtk_ramdump.exe [USB port num]\n");
	fbcon_puts(" \n");
	memset (szMsg, 0, sizeof(szMsg));
	sprintf(szMsg, " R0 : 0x%08X  R1 : 0x%08X  R2 : 0x%08X \n", *cpu_crash_ctx, *(cpu_crash_ctx + 1), *(cpu_crash_ctx + 2));
	fbcon_puts(szMsg);
	sprintf(szMsg, " R3 : 0x%08X  R4 : 0x%08X  R5 : 0x%08X \n", *(cpu_crash_ctx + 3), *(cpu_crash_ctx + 4), *(cpu_crash_ctx + 5));
	fbcon_puts(szMsg);
	sprintf(szMsg, " R6 : 0x%08X  R7 : 0x%08X  R8 : 0x%08X \n", *(cpu_crash_ctx + 6), *(cpu_crash_ctx + 7), *(cpu_crash_ctx + 8));
	fbcon_puts(szMsg);
	sprintf(szMsg, " R9 : 0x%08X R10 : 0x%08X R11 : 0x%08X \n", *(cpu_crash_ctx + 9), *(cpu_crash_ctx + 10), *(cpu_crash_ctx + 11));
	fbcon_puts(szMsg);
	sprintf(szMsg, "R12 : 0x%08X  SP : 0x%08X  LR : 0x%08X \n", *(cpu_crash_ctx + 12), *(cpu_crash_ctx + 13), *(cpu_crash_ctx + 14));
	fbcon_puts(szMsg);
	sprintf(szMsg, " PC : 0x%08X  CPSR : 0x%08X \n", *(cpu_crash_ctx + 15), *(cpu_crash_ctx + 16));
	fbcon_puts(szMsg);
	sprintf(szMsg, "CTRL : 0x%08X TRANSBASE : 0x%08X  DAC : 0x%08X \n", *(cpu_crash_ctx + 17), *(cpu_crash_ctx + 18), *(cpu_crash_ctx + 19));
	fbcon_puts(szMsg);
	fbcon_puts("----------------------------------------\n");
	//}
#endif
}

#endif /*              */

/* Fix-me! Add for porting because mark mt6589_dcm.c*/
#if 0
void arch_idle(void)
{
  //do nothing
}
#endif
#if 1  /*                                                       */
void lcd_backlight_Blink(void)
{
	unsigned long i = 0;
#if 0  /*                                                                        */
	mt_set_gpio_mode(GPIO144, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO144, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO144,GPIO_OUT_ONE);
	while (++i < 0x100000);
	i=0;
#endif  /*                                                                        */
	mt_set_gpio_mode(GPIO144, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO144, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO144,GPIO_OUT_ZERO);
	while (++i < 0x100000);

	mt_set_gpio_mode(GPIO144, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO144, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO144,GPIO_OUT_ONE);
}
#endif /*              */

void arch_reset(char mode, const char *cmd)
{
    char reboot = 0;
#if 1  /*                                                       */
/*                                                                      */
#if defined(CONFIG_LGE_HANDLE_PANIC)
//    DPI_REG_STATUS      reg_status;
    char szMsg[128];
    unsigned long i = 0;

/*                                                                                */

   volatile u32 *lge_boot_magic_number 	= (volatile u32*)(INTERNAL_SRAM_BASE + 0x1C00 - 32);
   volatile u32 *lge_boot_reason 		= lge_boot_magic_number + 1; // next 4 byte

#if 0  /*                                                       */
	   hwPowerOn(13, 2800, "VIBR");
#endif /*              */


/*                                                 */
#endif
/*                                                                      */
#endif /*              */

#if 0 //=======
    int res=0;
    struct wd_api*wd_api = NULL;

    res = get_wd_api(&wd_api);
#endif //>>>>>>> KK0.AOSP.MP1.TC1SP_PRB:drivers/misc/mediatek/kernel/system.c
    printk("arch_reset: cmd = %s\n", cmd ? : "NULL");

    if (cmd && !strcmp(cmd, "charger")) {
        /* do nothing */
    }
#if 1 //                                             
    else if ( cmd && !strcmp(cmd, "charge_reset") )
    {
        lge_save_boot_reason(0x776655AA, 0, 0);
    }
#endif //                                             
    else if (cmd && !strcmp(cmd, "recovery")) {
        rtc_mark_recovery();
    }
    else if (cmd && !strcmp(cmd, "--bnr_recovery")) {
        lge_save_boot_reason(LGE_BOOT_BNR_RECOVERY_MODE_REBOOT, 0, 0);
        rtc_mark_recovery();
    }
    else if (cmd && !strcmp(cmd, "bootloader")){
        rtc_mark_fast();
    }
#if 0 //                                             
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	else if (cmd && !strcmp(cmd, "kpoc")){
		rtc_mark_kpoc();
	}
#endif
#endif //                                             
    else {
    	reboot = 1;
    }

#if 1  /*                                                       */

#if defined(CONFIG_LGE_HANDLE_PANIC)
/*                                                                                */
/*                                                         */

    if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE && *lge_boot_reason == LGE_BOOT_KERNEL_CRASH
        && hreset_enable == 0)
    {
/*                                                                   */
#if 1
		struct mtkfb_device    *fbdev = NULL;
		if (mtkfb_fbi)
		{
			fbdev = (struct mtkfb_device *) mtkfb_fbi->par;
			mipi_fb_cfg.width	= 256;
			mipi_fb_cfg.height	= 320;
			mipi_fb_cfg.stride	= 256;
#if 1  /*                                                                      */
                    mipi_fb_cfg.base    = (void*) ioremap_nocache((unsigned int) fbdev->ovl_pa_base, (mipi_fb_cfg.width * mipi_fb_cfg.height * (mipi_fb_cfg.bpp / 8)));
#else
			mipi_fb_cfg.base	= (void*) ioremap_nocache((unsigned int) fbdev->fb_pa_base,
													 (mipi_fb_cfg.width * mipi_fb_cfg.height * (mipi_fb_cfg.bpp / 8)));
#endif  /*                                                                      */
#if 0 //=======
    if(res){
        printk("arch_reset, get wd api error %d\n",res);
    } else {
        wd_api->wd_sw_reset(reboot);
    }
#endif // >>>>>>> KK0.AOSP.MP1.TC1SP_PRB:drivers/misc/mediatek/kernel/system.c
}
#else
		/*                                                 */
				reg_status			= DPI_REG->STATUS;

				mipi_fb_cfg.width	= DPI_REG->SIZE.WIDTH;
				mipi_fb_cfg.height	= DPI_REG->SIZE.HEIGHT;
				mipi_fb_cfg.stride	= DPI_REG->SIZE.WIDTH;

				mipi_fb_cfg.base	= (void*) ioremap_cached((unsigned int)DPI_REG->FB[reg_status.FB_INUSE].ADDR,
															 (mipi_fb_cfg.width * mipi_fb_cfg.height * (mipi_fb_cfg.bpp / 3)));
#endif


        crash_dump_log      = (struct crash_log_dump *) ioremap_cached(LGE_BSP_CRASH_LOG_PHY_ADDR, LGE_BSP_CRASH_LOG_SIZE);
        cpu_crash_ctx       = (unsigned long *) ioremap_cached(LGE_CRASH_CTX_BUF_PHY_ADDR, LGE_CRASH_CTX_BUF_SIZE);

#if 1  //                                              
	if (crash_dump_log && cpu_crash_ctx)
#else
        if (mipi_fb_cfg.base && crash_dump_log && cpu_crash_ctx)
#endif
        {
        	/*                                                                   */
		#if 1 //                                              
			set_crash_display();
#if defined(TARGET_S4)
			DISP_DirectOverlayTransfer();
#else
                #if 1  /*                                                                      */
                    mtkfb_crash_late_resume();
                    DISP_display_crash();
                #else
			DISP_Crash_UpdateScreen();
                #endif  /*                                                                      */
                lcd_backlight_Blink();
#endif
		#endif
			/*                                                                   */

            /* Volumn down key : KP_ROW1(GPIO_97) & KP_COL1(GPIO_108) */
            /* Volumn up key : KP_ROW0() & KP_COL1(GPIO_108) */

#if 1 //                                              
            do
            {
                /*                                                                                                                */
#ifndef CONFIG_LOCAL_WDT
                mtk_wdt_restart(WK_WDT_EXT_TYPE);
#endif
                /*                                                                                                                */

                key_input_val = 0;
                key_input_val = reg_gpio->din[3].val;
/*                                                                  */
                key_input_val = key_input_val & 0x1000;
                i = 0;
                while (++i < 0x10000000)
                {
                }
            } while (key_input_val != 0);
#endif

        }
/*                                                                                */
    }
/*                                                  */
/*                                                           */
	else if (*lge_boot_magic_number == LGE_BOOT_REASON_MAGIC_CODE && *lge_boot_reason == LGE_BOOT_KERNEL_CRASH
	&& hreset_enable == 1)
	{
		lge_save_boot_reason(LGE_BOOT_HIDDEN_RESET_REBOOT, 0, 0);
	}
/*                                                           */

#endif
#endif /*              */

    wdt_arch_reset(reboot);

}
