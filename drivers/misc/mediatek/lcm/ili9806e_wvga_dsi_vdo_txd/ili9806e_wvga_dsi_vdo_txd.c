#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER


#if defined(BUILD_LK)
	#define LCM_DBG printf
#else
	#define LCM_DBG printk
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
#define ILI9806E_LCM_ID 									(0x0604)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    
    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            
            case REGFLAG_END_OF_TABLE :
                break;
            
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);				
            
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
        }
    }
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;

    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 16;
    params->dsi.vertical_frontporch					= 20;	
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 10;//10;
    params->dsi.horizontal_backporch				= 40;//50;
    params->dsi.horizontal_frontporch				= 70;//50;
   // params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
   //params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4
    params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =16;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif
}


static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000104;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001008;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000121;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000230;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000231;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001a40;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00006641;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000242;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000943;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00008644;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00007850;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00007851;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000052;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00005053;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000760;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000461;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000662;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000263;
    dsi_set_cmdq(data_array, 2, 1);

    //*************gamma  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000104;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000000A0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000AA1;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000009A2;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000017A3;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000007A4;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001DA5;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000AA6;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000BA7;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000003A8;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000EA9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000005AA;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000001AB;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000014AC;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000030AD;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000033AE;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000000AF;
    dsi_set_cmdq(data_array, 2, 1);

    //*************gamma  NAGITIVE *******************//

    data_array[0]=0x00023902;
    data_array[1]=0x000000C0;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000CC1;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000025C2;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000FC3;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000FC4;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000014C5;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000007C6;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000003C7;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000005C8;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000003C9;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000007CA;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000006CB;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000005CC;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001FCD;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000016CE;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000000CF;
    dsi_set_cmdq(data_array, 2, 1);

    //*************PAGE 6  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000604;
    dsi_set_cmdq(data_array, 3, 1);


    data_array[0]=0x00023902;
    data_array[1]=0x00002000;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000601;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002002;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000203;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000104;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000105;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00009806;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000407;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000508;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000009;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000000A;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000000B;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000010C;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000010D;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000000E;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000000F;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000FF10;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000F011;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000012;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000013;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000014;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000C015;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000816;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000017;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000018;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000019;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000001A;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000001B;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000001C;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000001D;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000120;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002321;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00004522;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00006723;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00000124;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002325;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00004526;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00006727;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001230;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002231;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002232;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002233;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00008734;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00009635;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000BA36;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000AB37;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000DC38;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000CD39;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000783A;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000693B;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000223C;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000223D;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000223E;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000223F;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002240;
    dsi_set_cmdq(data_array, 2, 1);

    //*************PAGE 7 *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000704;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00001D18;
    dsi_set_cmdq(data_array, 2, 1);
	
    data_array[0]=0x00023902;
    data_array[1]=0x00001217;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x00002240;
    dsi_set_cmdq(data_array, 2, 1);

    //*************PAGE 0 *******************//

    data_array[0]=0x00063902;
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000004;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);
    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	init_lcm_registers();

	unsigned int data_array[16];
	data_array[0]=0x00063902;
	data_array[1]=0x0698ffff;
	data_array[2]=0x00000804;
	dsi_set_cmdq(data_array, 3, 1);


//LCM_DBG("###ILI9806E_WVGA_DSI_TXD: %s, %d\n", __FUNCTION__,__LINE__);
}

extern int disp_bls_set_backlight(unsigned int level);
static void lcm_suspend(void)
{
    unsigned int data_array[16];
//LCM_DBG("###ILI9806E_WVGA_DSI_TXD: %s, %d\n", __FUNCTION__,__LINE__);

#if 1
    data_array[0]=0x00063902; 
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000004;
    dsi_set_cmdq(data_array, 3, 1);
 
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(50);
    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);
 
    data_array[0]=0x00063902; 
    data_array[1]=0x0698ffff;
    data_array[2]=0x00000804;
    dsi_set_cmdq(data_array, 3, 1);

#else
    disp_bls_set_backlight(0);

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

    
    SET_RESET_PIN(0);  
//    data_array[0] = 0x014F1500;
//    dsi_set_cmdq(data_array, 1, 1);
//    MDELAY(40);
#endif
}


static void lcm_resume(void)
{
	//LCM_DBG("###ILI9806E_WVGA_DSI_TXD: %s, %d\n", __FUNCTION__,__LINE__);
	lcm_init();
}
         

static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


    if(level > 255) 
        level = 255;
    
    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(data_array, 2, 1);
}


////ESD start ///

static unsigned int lcm_esd_check(void)
{

#ifndef BUILD_UBOOT
	unsigned char buffer[2];
	int array[4];

	unsigned int data_array[16];
	data_array[0]=0x00063902;
	data_array[1]=0x0698ffff;
	data_array[2]=0x00000004;
	dsi_set_cmdq(data_array, 3, 1);

	array[0]=0x00043700;//0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 4);

	LCM_DBG(" 9806e lcm_esd_check:%x",buffer[0]);

	data_array[0]=0x00063902; 
	data_array[1]=0x0698ffff;
	data_array[2]=0x00000804;
	dsi_set_cmdq(data_array, 3, 1);
	
	if((buffer[0] == 0x9C) ||(buffer[0] == 0x0))
	{
	    return FALSE;
	}

	LCM_DBG(" ####ili9806e_txd-lcm_esd_check:%x",buffer[0]);
	return TRUE;
	
#endif
}

static unsigned int lcm_esd_recover(void)
{
LCM_DBG("###ILI9806E_WVGA_DSI_TXD: %s, %d\n", __FUNCTION__,__LINE__);  
#if 1
	lcm_init();
	return TRUE;
#else
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(20);
    
    init_lcm_registers();
    MDELAY(20);
    lcm_resume();
    MDELAY(20);
//    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
//    MDELAY(10);    
	 return TRUE;
#endif
   
}
/////ESD end///
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static unsigned int lcm_compare_id(void)
{
#if 1
        unsigned int id = 0;
        unsigned char buffer[2];
        unsigned int array[16];

        int Channel=1;
        int data[4];
        int adcVol=0;
        
        int res=IMM_GetOneChannelValue(Channel,data,0);
        adcVol=data[0]*1000+data[1]*10;
        
	LCM_DBG("###ILI9806E_WVGA_DSI_TXD: LINE=%d %s, res=%d,adcVol = %d \n", __LINE__,__func__,res,adcVol);
		
       if(adcVol >= 800 && adcVol <=1000) //txd adcVol = 900
       {
       		LCM_DBG("###ILI9806E_WVGA_DSI_TXD: Succecss!");
		return 1;
       }
       else
       {
		LCM_DBG("###ILI9806E_WVGA_DSI_TXD: Fail!");
		return 0;
       }
#else
        int array[4];
        char buffer[5];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0;

        //Do reset here
        SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(25);       
        SET_RESET_PIN(1);
        MDELAY(120);   
        array[0]=0x00063902;
        array[1]=0x0698ffff;
        array[2]=0x00000104;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
 
        array[0]=0x00023700;//0x00023700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
    
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; ///////////////////////0x98

        array[0]=0x00043700;//0x00023700;
        dsi_set_cmdq(array, 1, 1); 
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[0]; ///////////////////////0x06

        array[0]=0x00043700;//0x00023700;
        dsi_set_cmdq(array, 1, 1); 
        read_reg_v2(0x02, buffer,1);
        id_low = buffer[0]; ////////////////////////0x04
 
        id = (id_midd << 8) | id_low;
	

LCM_DBG("###ILI9806E_WVGA_DSI_TXD: %s, id = 0x%08x\n", __func__, id);  

//return 0;
	return (ILI9806E_LCM_ID == id)?1:0;  //id = 0x00006868
#endif	

}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_wvga_dsi_vdo_txd_drv = 
{
    .name			= "ili9806e_dsi_vdo_byd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
//ESD start ///    
    .esd_check   = lcm_esd_check,
    .esd_recover   = lcm_esd_recover,  
//ESD end ///      
    .compare_id    = lcm_compare_id,
};

