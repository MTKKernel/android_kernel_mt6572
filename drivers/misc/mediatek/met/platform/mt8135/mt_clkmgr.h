// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/**
 * @file mt_clkmgr_internal.h
 * @brief Clock Manager driver private interface
 */

#ifndef __MT_CLKMGR_INTERNAL_H__
#define __MT_CLKMGR_INTERNAL_H__

#define CLKMGR_8135     1
#define CLKMGR_6572     0

#if CLKMGR_8135
    #if !defined(BUILD_WORKAROUND_6589_COMPATIBLE)
        #define BUILD_WORKAROUND_6589_COMPATIBLE    0
    #endif

    #define BUILD_WORKAROUND_6572_COMPATIBLE        1
    #define BUILD_WORKAROUND_8135                   1
    #define AUDIO_WORKAROUND                        1
#endif // CLKMGR_8135

#ifdef __cplusplus
// extern "C" { // TODO: disable temp
#endif

#if defined(__WIN32) || defined(__CYGWIN__) // MinGW32 or Cygwin gcc

#define CONFIG_CLKMGR_EMULATION

#endif

/*=============================================================*/
// Include files
/*=============================================================*/

// system includes
#include <linux/bitops.h>

// project includes
#include "mach/mt_reg_base.h"

// local includes

// forward references


/*=============================================================*/
// Macro definition
/*=============================================================*/

//
// Register Base Address
//

//
// APMIXED
//

#if CLKMGR_8135

#define APMIXEDSYS_BASE         APMIXED_BASE

#define AP_PLL_CON0             (APMIXEDSYS_BASE + 0x0000)
#define AP_PLL_CON1             (APMIXEDSYS_BASE + 0x0004)
#define AP_PLL_CON2             (APMIXEDSYS_BASE + 0x0008)
#define AP_PLL_CON3             (APMIXEDSYS_BASE + 0x000C)
#define AP_PLL_CON4             (APMIXEDSYS_BASE + 0x0010)

#define PLL_HP_CON0             (APMIXEDSYS_BASE + 0x0014)

#define ARMPLL_CON0             (APMIXEDSYS_BASE + 0x0200)
#define ARMPLL_CON1             (APMIXEDSYS_BASE + 0x0204)
#define ARMPLL_CON2             (APMIXEDSYS_BASE + 0x0208)
#define ARMPLL_PWR_CON0         (APMIXEDSYS_BASE + 0x0218)

#define MAINPLL_CON0            (APMIXEDSYS_BASE + 0x021C)
#define MAINPLL_CON1            (APMIXEDSYS_BASE + 0x0220)
#define MAINPLL_CON2            (APMIXEDSYS_BASE + 0x0224)
#define MAINPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x0234)

#define UNIVPLL_CON0            (APMIXEDSYS_BASE + 0x0238)
#define UNIVPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x0250)

#define MMPLL_CON0              (APMIXEDSYS_BASE + 0x0254)
#define MMPLL_PWR_CON0          (APMIXEDSYS_BASE + 0x026C)

#define MSDCPLL_CON0            (APMIXEDSYS_BASE + 0x0278)
#define MSDCPLL_CON1            (APMIXEDSYS_BASE + 0x027C)
#define MSDCPLL_CON2            (APMIXEDSYS_BASE + 0x0280)
#define MSDCPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x0290)

#define TVDPLL_CON0             (APMIXEDSYS_BASE + 0x0294)
#define TVDPLL_CON1             (APMIXEDSYS_BASE + 0x0298)
#define TVDPLL_CON2             (APMIXEDSYS_BASE + 0x029C)
#define TVDPLL_CON3             (APMIXEDSYS_BASE + 0x02A0)
#define TVDPLL_PWR_CON0         (APMIXEDSYS_BASE + 0x02AC)

#define LVDSPLL_CON0            (APMIXEDSYS_BASE + 0x02B0)
#define LVDSPLL_CON1            (APMIXEDSYS_BASE + 0x02B4)
#define LVDSPLL_CON2            (APMIXEDSYS_BASE + 0x02B8)
#define LVDSPLL_CON3            (APMIXEDSYS_BASE + 0x02BC)
#define LVDSPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x02C8)

#define ARMPLL2_CON0            (APMIXEDSYS_BASE + 0x02CC)
#define ARMPLL2_PWR_CON0        (APMIXEDSYS_BASE + 0x02E4)

#define AUDPLL_CON0             (APMIXEDSYS_BASE + 0x02E8)
#define AUDPLL_PWR_CON0         (APMIXEDSYS_BASE + 0x0300)

#define VDECPLL_CON0            (APMIXEDSYS_BASE + 0x0304)
#define VDECPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x031C)

#elif CLKMGR_6572

#define APMIXEDSYS_BASE         APMIXED_BASE

#define AP_PLL_CON0             (APMIXEDSYS_BASE + 0x0000)
#define AP_PLL_CON1             (APMIXEDSYS_BASE + 0x0004)
#define AP_PLL_CON2             (APMIXEDSYS_BASE + 0x0008)
#define AP_PLL_CON3             (APMIXEDSYS_BASE + 0x000C)

#define PLL_HP_CON0             (APMIXEDSYS_BASE + 0x0014)

#define ARMPLL_CON0             (APMIXEDSYS_BASE + 0x0100)
#define ARMPLL_CON1             (APMIXEDSYS_BASE + 0x0104)
#define ARMPLL_CON2             (APMIXEDSYS_BASE + 0x0108)
#define ARMPLL_PWR_CON0         (APMIXEDSYS_BASE + 0x0110)

#define MAINPLL_CON0            (APMIXEDSYS_BASE + 0x0120)
#define MAINPLL_CON1            (APMIXEDSYS_BASE + 0x0124)
#define MAINPLL_CON2            (APMIXEDSYS_BASE + 0x0128)
#define MAINPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x0130)

#define UNIVPLL_CON0            (APMIXEDSYS_BASE + 0x0140)
#define UNIVPLL_PWR_CON0        (APMIXEDSYS_BASE + 0x0150)

#define WHPLL_CON0              (APMIXEDSYS_BASE + 0x0240)
#define WHPLL_CON1              (APMIXEDSYS_BASE + 0x0244)
#define WHPLL_CON2              (APMIXEDSYS_BASE + 0x0248)
#define WHPLL_PWR_CON0          (APMIXEDSYS_BASE + 0x0250)
#define WHPLL_PATHSEL_CON       (APMIXEDSYS_BASE + 0x0254)
#define RSV_RW0_CON1            (APMIXEDSYS_BASE + 0x0F04)

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// TOP_CLOCK_CTRL
//

#if CLKMGR_8135

#define TOP_CLOCK_CTRL_BASE     TOPRGU_BASE

#define CLK_CFG_0               (TOP_CLOCK_CTRL_BASE + 0x0140)
#define CLK_CFG_1               (TOP_CLOCK_CTRL_BASE + 0x0144)
#define CLK_CFG_2               (TOP_CLOCK_CTRL_BASE + 0x0148)
#define CLK_CFG_3               (TOP_CLOCK_CTRL_BASE + 0x014C)
#define CLK_CFG_4               (TOP_CLOCK_CTRL_BASE + 0x0150)
#define CLK_CFG_5               (TOP_CLOCK_CTRL_BASE + 0x0154)
#define CLK_CFG_6               (TOP_CLOCK_CTRL_BASE + 0x0158)
#define CLK_CFG_7               (TOP_CLOCK_CTRL_BASE + 0x015C)
#define CLK_CFG_8               (TOP_CLOCK_CTRL_BASE + 0x0164)
#define CLK_CFG_9               (TOP_CLOCK_CTRL_BASE + 0x0168)

#elif CLKMGR_6572

#define TOP_CLOCK_CTRL_BASE     TOPCKGEN_BASE

#define CLK_MUX_SEL             (TOP_CLOCK_CTRL_BASE + 0x0000)
#define CLK_GATING_CTRL0        (TOP_CLOCK_CTRL_BASE + 0x0020)
#define CLK_GATING_CTRL1        (TOP_CLOCK_CTRL_BASE + 0x0024)
#define INFRABUS_DCMCTL1        (TOP_CLOCK_CTRL_BASE + 0x002C)
#define MPLL_FREDIV_EN          (TOP_CLOCK_CTRL_BASE + 0x0030)
#define UPLL_FREDIV_EN          (TOP_CLOCK_CTRL_BASE + 0x0034)
#define SET_CLK_GATING_CTRL0    (TOP_CLOCK_CTRL_BASE + 0x0050)
#define SET_CLK_GATING_CTRL1    (TOP_CLOCK_CTRL_BASE + 0x0054)
#define SET_MPLL_FREDIV_EN      (TOP_CLOCK_CTRL_BASE + 0x0060)
#define SET_UPLL_FREDIV_EN      (TOP_CLOCK_CTRL_BASE + 0x0064)
#define CLR_CLK_GATING_CTRL0    (TOP_CLOCK_CTRL_BASE + 0x0080)
#define CLR_CLK_GATING_CTRL1    (TOP_CLOCK_CTRL_BASE + 0x0084)
#define CLR_MPLL_FREDIV_EN      (TOP_CLOCK_CTRL_BASE + 0x0090)
#define CLR_UPLL_FREDIV_EN      (TOP_CLOCK_CTRL_BASE + 0x0094)

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// PERI / INFRA
//

#if CLKMGR_8135

#define PERI_PDN0_SET           (PERICFG_BASE + 0x0008)
#define PERI_PDN0_CLR           (PERICFG_BASE + 0x0010)
#define PERI_PDN0_STA           (PERICFG_BASE + 0x0018)
#define PERI_PDN1_SET           (PERICFG_BASE + 0x000C)
#define PERI_PDN1_CLR           (PERICFG_BASE + 0x0014)
#define PERI_PDN1_STA           (PERICFG_BASE + 0x001C)
        
#define INFRA_PDN0              (INFRACFG_BASE + 0x0040)
#define INFRA_PDN1              (INFRACFG_BASE + 0x0044)
#define INFRA_PDN_STA           (INFRACFG_BASE + 0x0048)

#endif // CLKMGR_8135

//
// MMSYS_CONFIG
//

#if CLKMGR_6572

#define MMSYS_CG_CON0           (MMSYS_CONFIG_BASE + 0x100)
#define MMSYS_CG_SET0           (MMSYS_CONFIG_BASE + 0x104)
#define MMSYS_CG_CLR0           (MMSYS_CONFIG_BASE + 0x108)
#define MMSYS_CG_CON1           (MMSYS_CONFIG_BASE + 0x110)
#define MMSYS_CG_SET1           (MMSYS_CONFIG_BASE + 0x114)
#define MMSYS_CG_CLR1           (MMSYS_CONFIG_BASE + 0x118)

#endif // CLKMGR_6572

//
// MFG_CONFIG
//

#if CLKMGR_8135

#define MFG_PD_STATUS           (MFG_CONFIG_BASE + 0x0000)
#define MFG_PD_SET              (MFG_CONFIG_BASE + 0x0004)
#define MFG_PD_CLR              (MFG_CONFIG_BASE + 0x0008)

#elif CLKMGR_6572

#define MFG_CONFIG_BASE         G3D_CONFIG_BASE

#define MFG_CG_CON              (MFG_CONFIG_BASE + 0x0000)
#define MFG_CG_SET              (MFG_CONFIG_BASE + 0x0004)
#define MFG_CG_CLR              (MFG_CONFIG_BASE + 0x0008)

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// IMG (ISP), VENC, VDEC, DISP
//

#if CLKMGR_8135

#define IMG_CG_CON              (IMGSYS_CONFG_BASE + 0x0000)
#define IMG_CG_SET              (IMGSYS_CONFG_BASE + 0x0004)
#define IMG_CG_CLR              (IMGSYS_CONFG_BASE + 0x0008)

#define VENC_GLOBAL_CON_BASE    VENC_TOP_BASE

#define VENCSYS_CG_CON          (VENC_GLOBAL_CON_BASE + 0x0000)
#define VENCSYS_CG_SET          (VENC_GLOBAL_CON_BASE + 0x0004)
#define VENCSYS_CG_CLR          (VENC_GLOBAL_CON_BASE + 0x0008)

#define VDEC_CKEN_SET           (VDEC_GCON_BASE + 0x0000)
#define VDEC_CKEN_CLR           (VDEC_GCON_BASE + 0x0004)
#define VDEC_LARB_CKEN_SET      (VDEC_GCON_BASE + 0x0008)
#define VDEC_LARB_CKEN_CLR      (VDEC_GCON_BASE + 0x000C)

#define DISP_CG_CON0            (DISPSYS_BASE + 0x0100)
#define DISP_CG_SET0            (DISPSYS_BASE + 0x0104)
#define DISP_CG_CLR0            (DISPSYS_BASE + 0x0108)
#define DISP_CG_CON1            (DISPSYS_BASE + 0x0110)
#define DISP_CG_SET1            (DISPSYS_BASE + 0x0114)
#define DISP_CG_CLR1            (DISPSYS_BASE + 0x0118)

#endif // CLKMGR_8135

//
// AUDIO_SYS_TOP
//

#if CLKMGR_8135

#define AUDIO_TOP_CON0          (AUDIO_TOP_BASE + 0x0000)
    
#elif CLKMGR_6572

#define AUDIO_SYS_TOP_BASE      AUDIO_BASE

#define AUDIO_TOP_CON0          (AUDIO_SYS_TOP_BASE + 0x0000)

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// Register Address & Mask
//

#if CLKMGR_8135

// MIXEDSYS
#define MIX_MAINPLL_806M_EN_BIT         BIT(31)
#define MIX_MAINPLL_537P3M_EN_BIT       BIT(30)
#define MIX_MAINPLL_322P4M_EN_BIT       BIT(29)
#define MIX_MAINPLL_230P3M_EN_BIT       BIT(28)

// TODO: set @ init is better
#define CG_MAINPLL_MASK                (MIX_MAINPLL_806M_EN_BIT     \
                                      | MIX_MAINPLL_537P3M_EN_BIT   \
                                      | MIX_MAINPLL_322P4M_EN_BIT   \
                                      | MIX_MAINPLL_230P3M_EN_BIT)

#define MIX_UNIVPLL_624M_EN_BIT         BIT(31)
#define MIX_UNIVPLL_416M_EN_BIT         BIT(30)
#define MIX_UNIVPLL_249P6M_EN_BIT       BIT(29)
#define MIX_UNIVPLL_178P3M_EN_BIT       BIT(28)
#define MIX_UNIVPLL_48M_EN_BIT          BIT(25)
#define MIX_UNIVPLL_USB48M_EN_BIT       BIT(24)

// TODO: set @ init is better
#define CG_UNIVPLL_MASK                (MIX_UNIVPLL_624M_EN_BIT     \
                                      | MIX_UNIVPLL_416M_EN_BIT     \
                                      | MIX_UNIVPLL_249P6M_EN_BIT   \
                                      | MIX_UNIVPLL_178P3M_EN_BIT   \
                                      | MIX_UNIVPLL_48M_EN_BIT      \
                                      | MIX_UNIVPLL_USB48M_EN_BIT)

#define MIX_MMPLL_DIV2_EN_BIT           BIT(31)
#define MIX_MMPLL_DIV3_EN_BIT           BIT(30)
#define MIX_MMPLL_DIV5_EN_BIT           BIT(29)
#define MIX_MMPLL_DIV7_EN_BIT           BIT(28)
// TODO: set @ init is better
#define CG_MMPLL_MASK                  (MIX_MMPLL_DIV2_EN_BIT   \
                                      | MIX_MMPLL_DIV3_EN_BIT   \
                                      | MIX_MMPLL_DIV5_EN_BIT   \
                                      | MIX_MMPLL_DIV7_EN_BIT)

// TOPCKGEN
#define TOP_PDN_SMI_BIT                 BIT(15)
#define TOP_PDN_MFG_BIT                 BIT(23)
#define TOP_PDN_IRDA_BIT                BIT(31)

// TODO: set @ init is better
#define CG_TOP0_MASK                   (TOP_PDN_SMI_BIT     \
                                      | TOP_PDN_MFG_BIT     \
                                      | TOP_PDN_IRDA_BIT)

#define TOP_PDN_CAM_BIT                 BIT(7)
#define TOP_PDN_AUD_INTBUS_BIT          BIT(15)
#define TOP_PDN_JPG_BIT                 BIT(23)
#define TOP_PDN_DISP_BIT                BIT(31)

// TODO: set @ init is better
#define CG_TOP1_MASK                   (TOP_PDN_CAM_BIT         \
                                      | TOP_PDN_AUD_INTBUS_BIT  \
                                      | TOP_PDN_JPG_BIT         \
                                      | TOP_PDN_DISP_BIT)

#define TOP_PDN_MSDC30_1_BIT            BIT(7)
#define TOP_PDN_MSDC30_2_BIT            BIT(15)
#define TOP_PDN_MSDC30_3_BIT            BIT(23)
#define TOP_PDN_MSDC30_4_BIT            BIT(31)

// TODO: set @ init is better
#define CG_TOP2_MASK                   (TOP_PDN_MSDC30_1_BIT    \
                                      | TOP_PDN_MSDC30_2_BIT    \
                                      | TOP_PDN_MSDC30_3_BIT    \
                                      | TOP_PDN_MSDC30_4_BIT)

#define TOP_PDN_USB20_BIT               BIT(7)

// TODO: set @ init is better
#define CG_TOP3_MASK                    TOP_PDN_USB20_BIT

#define TOP_PDN_VENC_BIT                BIT(15)
#define TOP_PDN_SPI_BIT                 BIT(23)
#define TOP_PDN_UART_BIT                BIT(31)

// TODO: set @ init is better
#define CG_TOP4_MASK                   (TOP_PDN_VENC_BIT    \
                                      | TOP_PDN_SPI_BIT     \
                                      | TOP_PDN_UART_BIT)

#define TOP_PDN_MEM_BIT                 BIT(7)
#define TOP_PDN_CAMTG_BIT               BIT(15)
#define TOP_PDN_AUDIO_BIT               BIT(31)

// TODO: set @ init is better
#define CG_TOP6_MASK                   (TOP_PDN_MEM_BIT     \
                                      | TOP_PDN_CAMTG_BIT   \
                                      | TOP_PDN_AUDIO_BIT)

#define TOP_PDN_FIX_BIT                 BIT(7)
#define TOP_PDN_VDEC_BIT                BIT(15)
#define TOP_PDN_DDRPHYCFG_BIT           BIT(23)
#define TOP_PDN_DPILVDS_BIT             BIT(31)

// TODO: set @ init is better
#define CG_TOP7_MASK                   (TOP_PDN_FIX_BIT         \
                                      | TOP_PDN_VDEC_BIT        \
                                      | TOP_PDN_DDRPHYCFG_BIT   \
                                      | TOP_PDN_DPILVDS_BIT)

#define TOP_PDN_PMICSPI_BIT             BIT(7)
#define TOP_PDN_MSDC30_0_BIT            BIT(15)
#define TOP_PDN_SMI_MFG_AS_BIT          BIT(23)
#define TOP_PDN_GCPU_BIT                BIT(31)

// TODO: set @ init is better
#define CG_TOP8_MASK                   (TOP_PDN_PMICSPI_BIT     \
                                      | TOP_PDN_MSDC30_0_BIT    \
                                      | TOP_PDN_SMI_MFG_AS_BIT  \
                                      | TOP_PDN_GCPU_BIT)

#define TOP_PDN_DPI1_BIT                BIT(7)
#define TOP_PDN_CCI_BIT                 BIT(15)
#define TOP_PDN_APLL_BIT                BIT(23)
#define TOP_PDN_HDMIPLL_BIT             BIT(31)

// TODO: set @ init is better
#define CG_TOP9_MASK                   (TOP_PDN_DPI1_BIT    \
                                      | TOP_PDN_CCI_BIT     \
                                      | TOP_PDN_APLL_BIT    \
                                      | TOP_PDN_HDMIPLL_BIT)

// PERI, INFRA
#define PERI_I2C5_PDN_BIT               BIT(31)
#define PERI_I2C4_PDN_BIT               BIT(30)
#define PERI_I2C3_PDN_BIT               BIT(29)
#define PERI_I2C2_PDN_BIT               BIT(28)
#define PERI_I2C1_PDN_BIT               BIT(27)
#define PERI_I2C0_PDN_BIT               BIT(26)
#define PERI_UART3_PDN_BIT              BIT(25)
#define PERI_UART2_PDN_BIT              BIT(24)
#define PERI_UART1_PDN_BIT              BIT(23)
#define PERI_UART0_PDN_BIT              BIT(22)
#define PERI_IRDA_PDN_BIT               BIT(21)
#define PERI_NLI_PDN_BIT                BIT(20)
#define PERI_MD_HIF_PDN_BIT             BIT(19)
#define PERI_AP_HIF_PDN_BIT             BIT(18)
#define PERI_MSDC30_3_PDN_BIT           BIT(17)
#define PERI_MSDC30_2_PDN_BIT           BIT(16)
#define PERI_MSDC30_1_PDN_BIT           BIT(15)
#define PERI_MSDC20_2_PDN_BIT           BIT(14)
#define PERI_MSDC20_1_PDN_BIT           BIT(13)
#define PERI_AP_DMA_PDN_BIT             BIT(12)
#define PERI_USB1_PDN_BIT               BIT(11)
#define PERI_USB0_PDN_BIT               BIT(10)
#define PERI_PWM_PDN_BIT                BIT(9)
#define PERI_PWM7_PDN_BIT               BIT(8)
#define PERI_PWM6_PDN_BIT               BIT(7)
#define PERI_PWM5_PDN_BIT               BIT(6)
#define PERI_PWM4_PDN_BIT               BIT(5)
#define PERI_PWM3_PDN_BIT               BIT(4)
#define PERI_PWM2_PDN_BIT               BIT(3)
#define PERI_PWM1_PDN_BIT               BIT(2)
#define PERI_THERM_PDN_BIT              BIT(1)
#define PERI_NFI_PDN_BIT                BIT(0)

// TODO: set @ init is better
#define CG_PERI0_MASK                  (PERI_I2C5_PDN_BIT       \
                                      | PERI_I2C4_PDN_BIT       \
                                      | PERI_I2C3_PDN_BIT       \
                                      | PERI_I2C2_PDN_BIT       \
                                      | PERI_I2C1_PDN_BIT       \
                                      | PERI_I2C0_PDN_BIT       \
                                      | PERI_UART3_PDN_BIT      \
                                      | PERI_UART2_PDN_BIT      \
                                      | PERI_UART1_PDN_BIT      \
                                      | PERI_UART0_PDN_BIT      \
                                      | PERI_IRDA_PDN_BIT       \
                                      | PERI_NLI_PDN_BIT        \
                                      | PERI_MD_HIF_PDN_BIT     \
                                      | PERI_AP_HIF_PDN_BIT     \
                                      | PERI_MSDC30_3_PDN_BIT   \
                                      | PERI_MSDC30_2_PDN_BIT   \
                                      | PERI_MSDC30_1_PDN_BIT   \
                                      | PERI_MSDC20_2_PDN_BIT   \
                                      | PERI_MSDC20_1_PDN_BIT   \
                                      | PERI_AP_DMA_PDN_BIT     \
                                      | PERI_USB1_PDN_BIT       \
                                      | PERI_USB0_PDN_BIT       \
                                      | PERI_PWM_PDN_BIT        \
                                      | PERI_PWM7_PDN_BIT       \
                                      | PERI_PWM6_PDN_BIT       \
                                      | PERI_PWM5_PDN_BIT       \
                                      | PERI_PWM4_PDN_BIT       \
                                      | PERI_PWM3_PDN_BIT       \
                                      | PERI_PWM2_PDN_BIT       \
                                      | PERI_PWM1_PDN_BIT       \
                                      | PERI_THERM_PDN_BIT      \
                                      | PERI_NFI_PDN_BIT)

#define PERI_USBSLV_PDN_BIT             BIT(8)
#define PERI_USB1_MCU_PDN_BIT           BIT(7)
#define PERI_USB0_MCU_PDN_BIT           BIT(6)
#define PERI_GCPU_PDN_BIT               BIT(5)
#define PERI_FHCTL_PDN_BIT              BIT(4)
#define PERI_SPI1_PDN_BIT               BIT(3)
#define PERI_AUXADC_PDN_BIT             BIT(2)
#define PERI_PERI_PWRAP_PDN_BIT         BIT(1)
#define PERI_I2C6_PDN_BIT               BIT(0)

// TODO: set @ init is better
#define CG_PERI1_MASK                  (PERI_USBSLV_PDN_BIT     \
                                      | PERI_USB1_MCU_PDN_BIT   \
                                      | PERI_USB0_MCU_PDN_BIT   \
                                      | PERI_GCPU_PDN_BIT       \
                                      | PERI_FHCTL_PDN_BIT      \
                                      | PERI_SPI1_PDN_BIT       \
                                      | PERI_AUXADC_PDN_BIT     \
                                      | PERI_PERI_PWRAP_PDN_BIT \
                                      | PERI_I2C6_PDN_BIT)

#define INFRA_PMIC_WRAP_PDN_BIT         BIT(23)
#define INFRA_PMICSPI_PDN_BIT           BIT(22)
#define INFRA_CCIF1_AP_CTRL_PDN_BIT     BIT(21)
#define INFRA_CCIF0_AP_CTRL_PDN_BIT     BIT(20)
#define INFRA_KP_PDN_BIT                BIT(16)
#define INFRA_CPUM_PDN_BIT              BIT(15)
#define INFRA_MD2AHB_BUS_PDN_BIT        BIT(14)
#define INFRA_MD2HWMIX_BUS_PDN_BIT      BIT(13)
#define INFRA_MD2MCU_BUS_PDN_BIT        BIT(12)
#define INFRA_MD1AHB_BUS_PDN_BIT        BIT(11)
#define INFRA_MD1HWMIX_BUS_PDN_BIT      BIT(10)
#define INFRA_MD1MCU_BUS_PDN_BIT        BIT(9)
#define INFRA_M4U_PDN_BIT               BIT(8)
#define INFRA_MFGAXI_PDN_BIT            BIT(7)
#define INFRA_DEVAPC_PDN_BIT            BIT(6)
#define INFRA_AUDIO_PDN_BIT             BIT(5)
#define INFRA_MFG_BUS_PDN_BIT           BIT(2)
#define INFRA_SMI_PDN_BIT               BIT(1)
#define INFRA_DBGCLK_PDN_BIT            BIT(0)

// TODO: set @ init is better
#define CG_INFRA_MASK                  (INFRA_PMIC_WRAP_PDN_BIT     \
                                      | INFRA_PMICSPI_PDN_BIT       \
                                      | INFRA_CCIF1_AP_CTRL_PDN_BIT \
                                      | INFRA_CCIF0_AP_CTRL_PDN_BIT \
                                      | INFRA_KP_PDN_BIT            \
                                      | INFRA_CPUM_PDN_BIT          \
                                      | INFRA_MD2AHB_BUS_PDN_BIT    \
                                      | INFRA_MD2HWMIX_BUS_PDN_BIT  \
                                      | INFRA_MD2MCU_BUS_PDN_BIT    \
                                      | INFRA_MD1AHB_BUS_PDN_BIT    \
                                      | INFRA_MD1HWMIX_BUS_PDN_BIT  \
                                      | INFRA_MD1MCU_BUS_PDN_BIT    \
                                      | INFRA_M4U_PDN_BIT           \
                                      | INFRA_MFGAXI_PDN_BIT        \
                                      | INFRA_DEVAPC_PDN_BIT        \
                                      | INFRA_AUDIO_PDN_BIT         \
                                      | INFRA_MFG_BUS_PDN_BIT       \
                                      | INFRA_SMI_PDN_BIT           \
                                      | INFRA_DBGCLK_PDN_BIT)

// MFG
#define MFG_BAXI_PDN_BIT                BIT(0)
#define MFG_BMEM_PDN_BIT                BIT(1)
#define MFG_BG3D_PDN_BIT                BIT(2)
#define MFG_B26M_PDN_BIT                BIT(3)

// TODO: set @ init is better
#define CG_MFG_MASK                    (MFG_BAXI_PDN_BIT    \
                                      | MFG_BMEM_PDN_BIT    \
                                      | MFG_BG3D_PDN_BIT    \
                                      | MFG_B26M_PDN_BIT)

// ISP (IMG)
#define ISP_FPC_CKPDN_BIT               BIT(13)
#define ISP_JPGENC_JPG_CKPDN_BIT        BIT(12)
#define ISP_JPGENC_SMI_CKPDN_BIT        BIT(11)
#define ISP_JPGDEC_JPG_CKPDN_BIT        BIT(10)
#define ISP_JPGDEC_SMI_CKPDN_BIT        BIT(9)
#define ISP_SEN_CAM_CKPDN_BIT           BIT(8)
#define ISP_SEN_TG_CKPDN_BIT            BIT(7)
#define ISP_CAM_CAM_CKPDN_BIT           BIT(6)
#define ISP_CAM_SMI_CKPDN_BIT           BIT(5)
#define ISP_COMM_SMI_CKPDN_BIT          BIT(4)
#define ISP_LARB4_SMI_CKPDN_BIT         BIT(2)
#define ISP_LARB3_SMI_CKPDN_BIT         BIT(0)

// TODO: set @ init is better
#define CG_ISP_MASK                    (ISP_FPC_CKPDN_BIT           \
                                      | ISP_JPGENC_JPG_CKPDN_BIT    \
                                      | ISP_JPGENC_SMI_CKPDN_BIT    \
                                      | ISP_JPGDEC_JPG_CKPDN_BIT    \
                                      | ISP_JPGDEC_SMI_CKPDN_BIT    \
                                      | ISP_SEN_CAM_CKPDN_BIT       \
                                      | ISP_SEN_TG_CKPDN_BIT        \
                                      | ISP_CAM_CAM_CKPDN_BIT       \
                                      | ISP_CAM_SMI_CKPDN_BIT       \
                                      | ISP_COMM_SMI_CKPDN_BIT      \
                                      | ISP_LARB4_SMI_CKPDN_BIT     \
                                      | ISP_LARB3_SMI_CKPDN_BIT)

// VENC
#define VENC_CKE_BIT                    BIT(0)

// TODO: set @ init is better
#define CG_VENC_MASK                    VENC_CKE_BIT

// VDEC
#define VDEC_CKEN_BIT                   BIT(0)
#define VDEC_LARB_CKEN_BIT              BIT(0)

// TODO: set @ init is better
#define CG_VDEC0_MASK                   VDEC_CKEN_BIT
#define CG_VDEC1_MASK                   VDEC_LARB_CKEN_BIT

// DISP
#define DISP_SMI_LARB2_BIT              BIT(0)
#define DISP_ROT_DISP_BIT               BIT(1)
#define DISP_ROT_SMI_BIT                BIT(2)
#define DISP_SCL_DISP_BIT               BIT(3)
#define DISP_OVL_DISP_BIT               BIT(4)
#define DISP_OVL_SMI_BIT                BIT(5)
#define DISP_COLOR_DISP_BIT             BIT(6)
#define DISP_TDSHP_DISP_BIT             BIT(7)
#define DISP_BLS_DISP_BIT               BIT(8)
#define DISP_WDMA0_DISP_BIT             BIT(9)
#define DISP_WDMA0_SMI_BIT              BIT(10)
#define DISP_WDMA1_DISP_BIT             BIT(11)
#define DISP_WDMA1_SMI_BIT              BIT(12)
#define DISP_RDMA0_DISP_BIT             BIT(13)
#define DISP_RDMA0_SMI_BIT              BIT(14)
#define DISP_RDMA0_OUTPUT_BIT           BIT(15)
#define DISP_RDMA1_DISP_BIT             BIT(16)
#define DISP_RDMA1_SMI_BIT              BIT(17)
#define DISP_RDMA1_OUTPUT_BIT           BIT(18)
#define DISP_GAMMA_DISP_BIT             BIT(19)
#define DISP_GAMMA_PIXEL_BIT            BIT(20)
#define DISP_CMDQ_DISP_BIT              BIT(21)
#define DISP_CMDQ_SMI_BIT               BIT(22)
#define DISP_G2D_DISP_BIT               BIT(23)
#define DISP_G2D_SMI_BIT                BIT(24)

// TODO: set @ init is better
#define CG_DISP0_MASK                  (DISP_SMI_LARB2_BIT  \
                                      | DISP_ROT_DISP_BIT   \
                                      | DISP_ROT_SMI_BIT    \
                                      | DISP_SCL_DISP_BIT   \
                                      | DISP_OVL_DISP_BIT   \
                                      | DISP_OVL_SMI_BIT    \
                                      | DISP_COLOR_DISP_BIT \
                                      | DISP_TDSHP_DISP_BIT \
                                      | DISP_BLS_DISP_BIT   \
                                      | DISP_WDMA0_DISP_BIT \
                                      | DISP_WDMA0_SMI_BIT  \
                                      | DISP_WDMA1_DISP_BIT \
                                      | DISP_WDMA1_SMI_BIT  \
                                      | DISP_RDMA0_DISP_BIT \
                                      | DISP_RDMA1_DISP_BIT \
                                      | DISP_GAMMA_DISP_BIT \
                                      | DISP_CMDQ_DISP_BIT  \
                                      | DISP_CMDQ_SMI_BIT   \
                                      | DISP_G2D_DISP_BIT   \
                                      | DISP_G2D_SMI_BIT)

#define DISP_DSI_DISP_BIT               BIT(3)
#define DISP_DSI_DSI_BIT                BIT(4)
#define DISP_DSI_DIV2_DSI_BIT           BIT(5)
#define DISP_DPI1_BIT                   BIT(7)
#define DISP_LVDS_DISP_BIT              BIT(10)
#define DISP_LVDS_CTS_BIT               BIT(11)
#define DISP_HDMI_DISP_BIT              BIT(12)
#define DISP_HDMI_PLL_BIT               BIT(13)
#define DISP_HDMI_AUDIO_BIT             BIT(14)
#define DISP_HDMI_SPDIF_BIT             BIT(15)
#define DISP_MUTEX_26M_BIT              BIT(18)
#define DISP_UFO_DISP_BIT               BIT(19)

// TODO: set @ init is better
#define CG_DISP1_MASK                  (DISP_DSI_DISP_BIT       \
                                      | DISP_DSI_DSI_BIT        \
                                      | DISP_DSI_DIV2_DSI_BIT   \
                                      | DISP_DPI1_BIT           \
                                      | DISP_LVDS_DISP_BIT      \
                                      | DISP_LVDS_CTS_BIT       \
                                      | DISP_HDMI_DISP_BIT      \
                                      | DISP_HDMI_PLL_BIT       \
                                      | DISP_HDMI_AUDIO_BIT     \
                                      | DISP_HDMI_SPDIF_BIT     \
                                      | DISP_MUTEX_26M_BIT      \
                                      | DISP_UFO_DISP_BIT)

// AUDIO
#define AUDIO_PDN_AFE_BIT               BIT(2)
#define AUDIO_PDN_I2S_BIT               BIT(6)
#define AUDIO_PDN_APLL_TUNER_BIT        BIT(19)
#define AUDIO_PDN_HDMI_CK_BIT           BIT(20)
#define AUDIO_PDN_SPDF_CK_BIT           BIT(21)

// TODO: set @ init is better
#define CG_AUDIO_MASK                  (AUDIO_PDN_AFE_BIT           \
                                      | AUDIO_PDN_I2S_BIT           \
                                      | AUDIO_PDN_APLL_TUNER_BIT    \
                                      | AUDIO_PDN_HDMI_CK_BIT       \
                                      | AUDIO_PDN_SPDF_CK_BIT)

#elif CLKMGR_6572

// CG_MIXED
#define USB48M_EN_BIT           BIT(28)
#define UNIV48M_EN_BIT          BIT(29)

// TODO: set @ init is better
#define CG_MIXED_MASK           USB48M_EN_BIT \
                              | UNIV48M_EN_BIT

// CG_MPLL
#define MPLL_D2_EN_BIT          BIT(0)
#define MPLL_D3_EN_BIT          BIT(1)
#define MPLL_D5_EN_BIT          BIT(2)
#define MPLL_D7_EN_BIT          BIT(3)

#define MPLL_D4_EN_BIT          BIT(8)
#define MPLL_D6_EN_BIT          BIT(9)
#define MPLL_D10_EN_BIT         BIT(10)

#define MPLL_D8_EN_BIT          BIT(16)
#define MPLL_D12_EN_BIT         BIT(17)
#define MPLL_D20_EN_BIT         BIT(18)

#define MPLL_D24_EN_BIT         BIT(24)

// TODO: set @ init is better
#define CG_MPLL_MASK           (MPLL_D2_EN_BIT  \
                              | MPLL_D3_EN_BIT  \
                              | MPLL_D5_EN_BIT  \
                              | MPLL_D7_EN_BIT  \
                              | MPLL_D4_EN_BIT  \
                              | MPLL_D6_EN_BIT  \
                              | MPLL_D10_EN_BIT \
                              | MPLL_D8_EN_BIT  \
                              | MPLL_D12_EN_BIT \
                              | MPLL_D20_EN_BIT \
                              | MPLL_D24_EN_BIT)

// CG_UPLL
#define UPLL_D2_EN_BIT          BIT(0)
#define UPLL_D3_EN_BIT          BIT(1)
#define UPLL_D5_EN_BIT          BIT(2)
#define UPLL_D7_EN_BIT          BIT(3)

#define UPLL_D4_EN_BIT          BIT(8)
#define UPLL_D6_EN_BIT          BIT(9)
#define UPLL_D10_EN_BIT         BIT(10)

#define UPLL_D8_EN_BIT          BIT(16)
#define UPLL_D12_EN_BIT         BIT(17)
#define UPLL_D20_EN_BIT         BIT(18)

#define UPLL_D16_EN_BIT         BIT(24)
#define UPLL_D24_EN_BIT         BIT(25)

// TODO: set @ init is better
#define CG_UPLL_MASK           (UPLL_D2_EN_BIT  \
                              | UPLL_D3_EN_BIT  \
                              | UPLL_D5_EN_BIT  \
                              | UPLL_D7_EN_BIT  \
                              | UPLL_D4_EN_BIT  \
                              | UPLL_D6_EN_BIT  \
                              | UPLL_D10_EN_BIT \
                              | UPLL_D8_EN_BIT  \
                              | UPLL_D12_EN_BIT \
                              | UPLL_D20_EN_BIT \
                              | UPLL_D16_EN_BIT \
                              | UPLL_D24_EN_BIT)

// CG_CTRL0 (clk_swcg_0)
#define PWM_MM_SW_CG_BIT        BIT(0)
#define CAM_MM_SW_CG_BIT        BIT(1)
#define MFG_MM_SW_CG_BIT        BIT(2)
#define SPM_52M_SW_CG_BIT       BIT(3)
#define MIPI_26M_DBG_EN_BIT     BIT(4)
#define DBI_BCLK_SW_CG_BIT      BIT(5)
#define SC_26M_CK_SEL_EN_BIT    BIT(6)
#define SC_MEM_CK_OFF_EN_BIT    BIT(7)

#define DBI_PAD0_SW_CG_BIT      BIT(16)
#define DBI_PAD1_SW_CG_BIT      BIT(17)
#define DBI_PAD2_SW_CG_BIT      BIT(18)
#define DBI_PAD3_SW_CG_BIT      BIT(19)
#define GPU_491P52M_EN_BIT      BIT(20)
#define GPU_500P5M_EN_BIT       BIT(21)

#define ARMDCM_CLKOFF_EN_BIT    BIT(31)

// TODO: set @ init is better
#define CG_CTRL0_MASK          (PWM_MM_SW_CG_BIT        \
                              | CAM_MM_SW_CG_BIT        \
                              | MFG_MM_SW_CG_BIT        \
                              | SPM_52M_SW_CG_BIT       \
                              | MIPI_26M_DBG_EN_BIT     \
                              | DBI_BCLK_SW_CG_BIT      \
                              | SC_26M_CK_SEL_EN_BIT    \
                              | SC_MEM_CK_OFF_EN_BIT    \
                              | DBI_PAD0_SW_CG_BIT      \
                              | DBI_PAD1_SW_CG_BIT      \
                              | DBI_PAD2_SW_CG_BIT      \
                              | DBI_PAD3_SW_CG_BIT      \
                              | GPU_491P52M_EN_BIT      \
                              | GPU_500P5M_EN_BIT       \
                              | ARMDCM_CLKOFF_EN_BIT)

// enable bit @ CG_CTRL0
#define CG_CTRL0_EN_MASK       (MIPI_26M_DBG_EN_BIT     \
                              | SC_26M_CK_SEL_EN_BIT    \
                              | SC_MEM_CK_OFF_EN_BIT    \
                              | GPU_491P52M_EN_BIT      \
                              | GPU_500P5M_EN_BIT       \
                              | ARMDCM_CLKOFF_EN_BIT)

// CG_CTRL1 (rg_peri_sw_cg)
#define EFUSE_SW_CG_BIT         BIT(0)
#define THEM_SW_CG_BIT          BIT(1)
#define APDMA_SW_CG_BIT         BIT(2)
#define I2C0_SW_CG_BIT          BIT(3)
#define I2C1_SW_CG_BIT          BIT(4)
#define AUX_SW_CG_MD_BIT        BIT(5) // XXX: NOT USED @ AP SIDE
#define NFI_SW_CG_BIT           BIT(6)
#define NFIECC_SW_CG_BIT        BIT(7)

#define DEBUGSYS_SW_CG_BIT      BIT(8)
#define PWM_SW_CG_BIT           BIT(9)
#define UART0_SW_CG_BIT         BIT(10)
#define UART1_SW_CG_BIT         BIT(11)
#define BTIF_SW_CG_BIT          BIT(12)
#define USB_SW_CG_BIT           BIT(13)
#define FHCTL_SW_CG_BIT         BIT(14)
#define AUX_SW_CG_THERM_BIT     BIT(15)

#define SPINFI_SW_CG_BIT        BIT(16)
#define MSDC0_SW_CG_BIT         BIT(17)
#define MSDC1_SW_CG_BIT         BIT(18)

#define PMIC_SW_CG_AP_BIT       BIT(20)
#define SEJ_SW_CG_BIT           BIT(21)
#define MEMSLP_DLYER_SW_CG_BIT  BIT(22)

#define APXGPT_SW_CG_BIT        BIT(24)
#define AUDIO_SW_CG_BIT         BIT(25)
#define SPM_SW_CG_BIT           BIT(26)
#define PMIC_SW_CG_MD_BIT       BIT(27) // XXX: NOT USED @ AP SIDE
#define PMIC_SW_CG_CONN_BIT     BIT(28) // XXX: NOT USED @ AP SIDE
#define PMIC_26M_SW_CG_BIT      BIT(29)
#define AUX_SW_CG_ADC_BIT       BIT(30)
#define AUX_SW_CG_TP_BIT        BIT(31)

// TODO: set @ init is better
#define CG_CTRL1_MASK          (EFUSE_SW_CG_BIT         \
                              | THEM_SW_CG_BIT          \
                              | APDMA_SW_CG_BIT         \
                              | I2C0_SW_CG_BIT          \
                              | I2C1_SW_CG_BIT          \
                              | NFI_SW_CG_BIT           \
                              | NFIECC_SW_CG_BIT        \
                              | DEBUGSYS_SW_CG_BIT      \
                              | PWM_SW_CG_BIT           \
                              | UART0_SW_CG_BIT         \
                              | UART1_SW_CG_BIT         \
                              | BTIF_SW_CG_BIT          \
                              | USB_SW_CG_BIT           \
                              | FHCTL_SW_CG_BIT         \
                              | SPINFI_SW_CG_BIT        \
                              | MSDC0_SW_CG_BIT         \
                              | MSDC1_SW_CG_BIT         \
                              | PMIC_SW_CG_AP_BIT       \
                              | SEJ_SW_CG_BIT           \
                              | MEMSLP_DLYER_SW_CG_BIT  \
                              | APXGPT_SW_CG_BIT        \
                              | AUDIO_SW_CG_BIT         \
                              | SPM_SW_CG_BIT           \
                              | PMIC_26M_SW_CG_BIT      \
                              | AUX_SW_CG_ADC_BIT)

/*
                              | AUX_SW_CG_THERM_BIT     \
                              | AUX_SW_CG_MD_BIT        \
                              | AUX_SW_CG_TP_BIT        \
                              | PMIC_SW_CG_MD_BIT       \
                              | PMIC_SW_CG_CONN_BIT     \
*/

// CG_MMSYS0
#define SMI_COMMON_SW_CG_BIT        BIT(0)
#define SMI_LARB0_SW_CG_BIT         BIT(1)
#define MM_CMDQ_SW_CG_BIT           BIT(2)
#define MM_CMDQ_SMI_IF_SW_CG_BIT    BIT(3)
#define DISP_COLOR_SW_CG_BIT        BIT(4)
#define DISP_BLS_SW_CG_BIT          BIT(5)
#define DISP_WDMA_SW_CG_BIT         BIT(6)
#define DISP_RDMA_SW_CG_BIT         BIT(7)
#define DISP_OVL_SW_CG_BIT          BIT(8)
#define MDP_TDSHP_SW_CG_BIT         BIT(9)
#define MDP_WROT_SW_CG_BIT          BIT(10)
#define MDP_WDMA_SW_CG_BIT          BIT(11)
#define MDP_RSZ1_SW_CG_BIT          BIT(12)
#define MDP_RSZ0_SW_CG_BIT          BIT(13)
#define MDP_RDMA_SW_CG_BIT          BIT(14)
#define MDP_BLS_26M_SW_CG_BIT       BIT(15)
#define MM_CAM_SW_CG_BIT            BIT(16)
#define MM_SENINF_SW_CG_BIT         BIT(17)
#define MM_CAMTG_SW_CG_BIT          BIT(18)
#define MM_CODEC_SW_CG_BIT          BIT(19)
#define DISP_FAKE_ENG_SW_CG_BIT     BIT(20)
#define MUTEX_SLOW_CLOCK_SW_CG_BIT  BIT(21)

// TODO: set @ init is better
#define CG_MMSYS0_MASK         (SMI_COMMON_SW_CG_BIT        \
                              | SMI_LARB0_SW_CG_BIT         \
                              | MM_CMDQ_SW_CG_BIT           \
                              | MM_CMDQ_SMI_IF_SW_CG_BIT    \
                              | DISP_COLOR_SW_CG_BIT        \
                              | DISP_BLS_SW_CG_BIT          \
                              | DISP_WDMA_SW_CG_BIT         \
                              | DISP_RDMA_SW_CG_BIT         \
                              | DISP_OVL_SW_CG_BIT          \
                              | MDP_TDSHP_SW_CG_BIT         \
                              | MDP_WROT_SW_CG_BIT          \
                              | MDP_WDMA_SW_CG_BIT          \
                              | MDP_RSZ1_SW_CG_BIT          \
                              | MDP_RSZ0_SW_CG_BIT          \
                              | MDP_RDMA_SW_CG_BIT          \
                              | MDP_BLS_26M_SW_CG_BIT       \
                              | MM_CAM_SW_CG_BIT            \
                              | MM_SENINF_SW_CG_BIT         \
                              | MM_CAMTG_SW_CG_BIT          \
                              | MM_CODEC_SW_CG_BIT          \
                              | DISP_FAKE_ENG_SW_CG_BIT     \
                              | MUTEX_SLOW_CLOCK_SW_CG_BIT)

// CG_MMSYS1
#define DSI_ENGINE_SW_CG_BIT        BIT(0)
#define DSI_DIGITAL_SW_CG_BIT       BIT(1)
#define DISP_DPI_ENGINE_SW_CG_BIT   BIT(2)
#define DISP_DPI_IF_SW_CG_BIT       BIT(3)
#define DISP_DBI_ENGINE_SW_CG_BIT   BIT(4)
#define DISP_DBI_SMI_SW_CG_BIT      BIT(5)
#define DISP_DBI_IF_SW_CG_BIT       BIT(6)

// TODO: set @ init is better
#define CG_MMSYS1_MASK         (DSI_ENGINE_SW_CG_BIT        \
                              | DSI_DIGITAL_SW_CG_BIT       \
                              | DISP_DPI_ENGINE_SW_CG_BIT   \
                              | DISP_DPI_IF_SW_CG_BIT       \
                              | DISP_DBI_ENGINE_SW_CG_BIT   \
                              | DISP_DBI_SMI_SW_CG_BIT      \
                              | DISP_DBI_IF_SW_CG_BIT)

// CG_MFG
#define MFG_PDN_BG3D_SW_CG_BIT  BIT(0)

// TODO: set @ init is better
#define CG_MFG_MASK            (MFG_PDN_BG3D_SW_CG_BIT)

// CG_AUDIO
#define AUD_PDN_AFE_EN_BIT          BIT(2)
#define AUD_PDN_I2S_EN_BIT          BIT(6)
#define AUD_PDN_ADC_EN_BIT          BIT(24)
#define AUD_PDN_DAC_EN_BIT          BIT(25)
#define AUD_PDN_DAC_PREDIS_EN_BIT   BIT(26)
#define AUD_PDN_TML_EN_BIT          BIT(27)

// TODO: set @ init is better
#define CG_AUDIO_MASK          (AUD_PDN_AFE_EN_BIT          \
                              | AUD_PDN_I2S_EN_BIT          \
                              | AUD_PDN_ADC_EN_BIT          \
                              | AUD_PDN_DAC_EN_BIT          \
                              | AUD_PDN_DAC_PREDIS_EN_BIT   \
                              | AUD_PDN_TML_EN_BIT)

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

/*=============================================================*/
// Type definition
/*=============================================================*/


/*=============================================================*/
// Global variable definition
/*=============================================================*/


/*=============================================================*/
// Global function definition
/*=============================================================*/


#ifdef __cplusplus
// } // TODO: disable temp
#endif

#endif // __MT_CLKMGR_INTERNAL_H__

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/**
 * @file mt_clkmgr.h
 * @brief Clock Manager driver public interface
 */

#ifndef __MT_CLKMGR_H__
#define __MT_CLKMGR_H__

#ifdef __cplusplus
// extern "C" { // TODO: disable temp
#endif

#ifdef __MT_CLKMGR_C__
  #define EXTERN
#else
  #define EXTERN extern
#endif

/*=============================================================*/
// Include files
/*=============================================================*/

// system includes
#include <linux/list.h>

// project includes
#include "mach/mt_typedefs.h"

// local includes

// forward references


/*=============================================================*/
// Macro definition
/*=============================================================*/


/*=============================================================*/
// Type definition
/*=============================================================*/

//
// CG_GPR ID
//

#if CLKMGR_8135

typedef enum
{
    CG_MAINPLL,
    CG_UNIVPLL,
    CG_MMPLL,
    CG_TOP0,
    CG_TOP1,
    CG_TOP2,
    CG_TOP3,
    CG_TOP4,
    CG_TOP6,
    CG_TOP7,
    CG_TOP8,
    CG_TOP9,
    CG_INFRA,
    CG_PERI0,
    CG_PERI1,
    CG_MFG,
    CG_ISP,
    CG_VENC,
    CG_VDEC0,
    CG_VDEC1,
    CG_DISP0,
    CG_DISP1,
    CG_AUDIO,

    NR_GRPS,
    
    // Virtual CG Group
    CG_VIRTUAL,
    
} cg_grp_id;

#elif CLKMGR_6572

typedef enum
{
    CG_MIXED    = 0,
    CG_MPLL     = 1,
    CG_UPLL     = 2,
    CG_CTRL0    = 3,
    CG_CTRL1    = 4,
    CG_MMSYS0   = 5,
    CG_MMSYS1   = 6,
    CG_MFG      = 7,
    CG_AUDIO    = 8,

    NR_GRPS,
} cg_grp_id;

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// CG_CLK ID
//
#define BEG_CG_GRP(grp_name, item)      item, grp_name##_FROM = item
#define END_CG_GRP(grp_name, item)      item, grp_name##_TO = item

#define BEG_END_CG_GRP(grp_name, item)  item, grp_name##_FROM = item, grp_name##_TO = item

#define FROM_CG_GRP_ID(grp_name)        grp_name##_FROM
#define TO_CG_GRP_ID(grp_name)          grp_name##_TO
#define NR_CG_GRP_CLKS(grp_name)        (grp_name##_TO - grp_name##_FROM + 1)

#if CLKMGR_8135

typedef enum
{
   // CG_MAINPLL
    BEG_CG_GRP(CG_MAINPLL,  MT_CG_MIX_MAINPLL_806M_EN),
                            MT_CG_MIX_MAINPLL_537P3M_EN,
                            MT_CG_MIX_MAINPLL_322P4M_EN,
    END_CG_GRP(CG_MAINPLL,  MT_CG_MIX_MAINPLL_230P3M_EN),
    
   // CG_UNIVPLL
    BEG_CG_GRP(CG_UNIVPLL,  MT_CG_MIX_UNIVPLL_624M_EN),
                            MT_CG_MIX_UNIVPLL_416M_EN,
                            MT_CG_MIX_UNIVPLL_249P6M_EN,
                            MT_CG_MIX_UNIVPLL_178P3M_EN,
                            MT_CG_MIX_UNIVPLL_48M_EN,
    END_CG_GRP(CG_UNIVPLL,  MT_CG_MIX_UNIVPLL_USB48M_EN),
                         
   // CG_MMPLL
    BEG_CG_GRP(CG_MMPLL,    MT_CG_MIX_MMPLL_DIV2_EN),
                            MT_CG_MIX_MMPLL_DIV3_EN,
                            MT_CG_MIX_MMPLL_DIV5_EN,
    END_CG_GRP(CG_MMPLL,    MT_CG_MIX_MMPLL_DIV7_EN),

    // CG_TOP0
    BEG_CG_GRP(CG_TOP0,     MT_CG_TOP_PDN_SMI),
                            MT_CG_TOP_PDN_MFG,
    END_CG_GRP(CG_TOP0,     MT_CG_TOP_PDN_IRDA),

    // CG_TOP1
    BEG_CG_GRP(CG_TOP1,     MT_CG_TOP_PDN_CAM),
                            MT_CG_TOP_PDN_AUD_INTBUS,
                            MT_CG_TOP_PDN_JPG,
    END_CG_GRP(CG_TOP1,     MT_CG_TOP_PDN_DISP),

    // CG_TOP2
    BEG_CG_GRP(CG_TOP2,     MT_CG_TOP_PDN_MSDC30_1),
                            MT_CG_TOP_PDN_MSDC30_2,
                            MT_CG_TOP_PDN_MSDC30_3,
    END_CG_GRP(CG_TOP2,     MT_CG_TOP_PDN_MSDC30_4),

    // CG_TOP3
    BEG_END_CG_GRP(CG_TOP3, MT_CG_TOP_PDN_USB20),

    // CG_TOP4
    BEG_CG_GRP(CG_TOP4,     MT_CG_TOP_PDN_VENC),
                            MT_CG_TOP_PDN_SPI,
    END_CG_GRP(CG_TOP4,     MT_CG_TOP_PDN_UART),

    // CG_TOP6
    BEG_CG_GRP(CG_TOP6,     MT_CG_TOP_PDN_MEM),
                            MT_CG_TOP_PDN_CAMTG,
    END_CG_GRP(CG_TOP6,     MT_CG_TOP_PDN_AUDIO),

    // CG_TOP7
    BEG_CG_GRP(CG_TOP7,     MT_CG_TOP_PDN_FIX),
                            MT_CG_TOP_PDN_VDEC,
                            MT_CG_TOP_PDN_DDRPHYCFG,
    END_CG_GRP(CG_TOP7,     MT_CG_TOP_PDN_DPILVDS),

    // CG_TOP8
    BEG_CG_GRP(CG_TOP8,     MT_CG_TOP_PDN_PMICSPI),
                            MT_CG_TOP_PDN_MSDC30_0,
                            MT_CG_TOP_PDN_SMI_MFG_AS,
    END_CG_GRP(CG_TOP8,     MT_CG_TOP_PDN_GCPU),

    // CG_TOP9
    BEG_CG_GRP(CG_TOP9,     MT_CG_TOP_PDN_DPI1),
                            MT_CG_TOP_PDN_CCI,
                            MT_CG_TOP_PDN_APLL,
    END_CG_GRP(CG_TOP9,     MT_CG_TOP_PDN_HDMIPLL),

    // CG_INFRA
    BEG_CG_GRP(CG_INFRA,    MT_CG_INFRA_PMIC_WRAP_PDN),
                            MT_CG_INFRA_PMICSPI_PDN,
                            MT_CG_INFRA_CCIF1_AP_CTRL_PDN,
                            MT_CG_INFRA_CCIF0_AP_CTRL_PDN,
                            MT_CG_INFRA_KP_PDN,
                            MT_CG_INFRA_CPUM_PDN,
                            MT_CG_INFRA_MD2AHB_BUS_PDN,
                            MT_CG_INFRA_MD2HWMIX_BUS_PDN,
                            MT_CG_INFRA_MD2MCU_BUS_PDN,
                            MT_CG_INFRA_MD1AHB_BUS_PDN,
                            MT_CG_INFRA_MD1HWMIX_BUS_PDN,
                            MT_CG_INFRA_MD1MCU_BUS_PDN,
                            MT_CG_INFRA_M4U_PDN,
                            MT_CG_INFRA_MFGAXI_PDN,
                            MT_CG_INFRA_DEVAPC_PDN,
                            MT_CG_INFRA_AUDIO_PDN,
                            MT_CG_INFRA_MFG_BUS_PDN,
                            MT_CG_INFRA_SMI_PDN,
    END_CG_GRP(CG_INFRA,    MT_CG_INFRA_DBGCLK_PDN),

    // CG_PERI0
    BEG_CG_GRP(CG_PERI0,    MT_CG_PERI_I2C5_PDN),
                            MT_CG_PERI_I2C4_PDN,
                            MT_CG_PERI_I2C3_PDN,
                            MT_CG_PERI_I2C2_PDN,
                            MT_CG_PERI_I2C1_PDN,
                            MT_CG_PERI_I2C0_PDN,
                            MT_CG_PERI_UART3_PDN,
                            MT_CG_PERI_UART2_PDN,
                            MT_CG_PERI_UART1_PDN,
                            MT_CG_PERI_UART0_PDN,
                            MT_CG_PERI_IRDA_PDN,
                            MT_CG_PERI_NLI_PDN,
                            MT_CG_PERI_MD_HIF_PDN,
                            MT_CG_PERI_AP_HIF_PDN,
                            MT_CG_PERI_MSDC30_3_PDN,
                            MT_CG_PERI_MSDC30_2_PDN,
                            MT_CG_PERI_MSDC30_1_PDN,
                            MT_CG_PERI_MSDC20_2_PDN,
                            MT_CG_PERI_MSDC20_1_PDN,
                            MT_CG_PERI_AP_DMA_PDN,
                            MT_CG_PERI_USB1_PDN,
                            MT_CG_PERI_USB0_PDN,
                            MT_CG_PERI_PWM_PDN,
                            MT_CG_PERI_PWM7_PDN,
                            MT_CG_PERI_PWM6_PDN,
                            MT_CG_PERI_PWM5_PDN,
                            MT_CG_PERI_PWM4_PDN,
                            MT_CG_PERI_PWM3_PDN,
                            MT_CG_PERI_PWM2_PDN,
                            MT_CG_PERI_PWM1_PDN,
                            MT_CG_PERI_THERM_PDN,
    END_CG_GRP(CG_PERI0,    MT_CG_PERI_NFI_PDN),

    // CG_PERI1
    BEG_CG_GRP(CG_PERI1,    MT_CG_PERI_USBSLV_PDN),
                            MT_CG_PERI_USB1_MCU_PDN,
                            MT_CG_PERI_USB0_MCU_PDN,
                            MT_CG_PERI_GCPU_PDN,
                            MT_CG_PERI_FHCTL_PDN,
                            MT_CG_PERI_SPI1_PDN,
                            MT_CG_PERI_AUXADC_PDN,
                            MT_CG_PERI_PERI_PWRAP_PDN,
    END_CG_GRP(CG_PERI1,    MT_CG_PERI_I2C6_PDN),

    // CG_MFG
    BEG_CG_GRP(CG_MFG,      MT_CG_MFG_BAXI_PDN),
                            MT_CG_MFG_BMEM_PDN,
                            MT_CG_MFG_BG3D_PDN,
    END_CG_GRP(CG_MFG,      MT_CG_MFG_B26M_PDN),

    // CG_ISP
    BEG_CG_GRP(CG_ISP,      MT_CG_ISP_FPC_CKPDN),
                            MT_CG_ISP_JPGENC_JPG_CKPDN,
                            MT_CG_ISP_JPGENC_SMI_CKPDN,
                            MT_CG_ISP_JPGDEC_JPG_CKPDN,
                            MT_CG_ISP_JPGDEC_SMI_CKPDN,
                            MT_CG_ISP_SEN_CAM_CKPDN,
                            MT_CG_ISP_SEN_TG_CKPDN,
                            MT_CG_ISP_CAM_CAM_CKPDN,
                            MT_CG_ISP_CAM_SMI_CKPDN,
                            MT_CG_ISP_COMM_SMI_CKPDN,
                            MT_CG_ISP_LARB4_SMI_CKPDN,
    END_CG_GRP(CG_ISP,      MT_CG_ISP_LARB3_SMI_CKPDN),

    // CG_VENC
    BEG_END_CG_GRP(CG_VENC, MT_CG_VENC_CKE),

    // CG_VDEC0
    BEG_END_CG_GRP(CG_VDEC0,    MT_CG_VDEC_CKEN),
    
    // CG_VDEC1
    BEG_END_CG_GRP(CG_VDEC1,    MT_CG_VDEC_LARB_CKEN),

    // CG_DISP0
    BEG_CG_GRP(CG_DISP0,    MT_CG_DISP_SMI_LARB2),
                            MT_CG_DISP_ROT_DISP,
                            MT_CG_DISP_ROT_SMI,
                            MT_CG_DISP_SCL_DISP,
                            MT_CG_DISP_OVL_DISP,
                            MT_CG_DISP_OVL_SMI,
                            MT_CG_DISP_COLOR_DISP,
                            MT_CG_DISP_TDSHP_DISP,
                            MT_CG_DISP_BLS_DISP,
                            MT_CG_DISP_WDMA0_DISP,
                            MT_CG_DISP_WDMA0_SMI,
                            MT_CG_DISP_WDMA1_DISP,
                            MT_CG_DISP_WDMA1_SMI,
                            MT_CG_DISP_RDMA0_DISP,
                            MT_CG_DISP_RDMA0_SMI,
                            MT_CG_DISP_RDMA0_OUTPUT,
                            MT_CG_DISP_RDMA1_DISP,
                            MT_CG_DISP_RDMA1_SMI,
                            MT_CG_DISP_RDMA1_OUTPUT,
                            MT_CG_DISP_GAMMA_DISP,
                            MT_CG_DISP_GAMMA_PIXEL,
                            MT_CG_DISP_CMDQ_DISP,
                            MT_CG_DISP_CMDQ_SMI,
                            MT_CG_DISP_G2D_DISP,
    END_CG_GRP(CG_DISP0,    MT_CG_DISP_G2D_SMI),

    // CG_DISP1
    BEG_CG_GRP(CG_DISP1,    MT_CG_DISP_DSI_DISP),
                            MT_CG_DISP_DSI_DSI,
                            MT_CG_DISP_DSI_DIV2_DSI,
                            MT_CG_DISP_DPI1,
                            MT_CG_DISP_LVDS_DISP,
                            MT_CG_DISP_LVDS_CTS,
                            MT_CG_DISP_HDMI_DISP,
                            MT_CG_DISP_HDMI_PLL,
                            MT_CG_DISP_HDMI_AUDIO,
                            MT_CG_DISP_HDMI_SPDIF,
                            MT_CG_DISP_MUTEX_26M,
    END_CG_GRP(CG_DISP1,    MT_CG_DISP_UFO_DISP),

    // CG_AUDIO
    BEG_CG_GRP(CG_AUDIO,    MT_CG_AUDIO_PDN_AFE),
                            MT_CG_AUDIO_PDN_I2S,
                            MT_CG_AUDIO_PDN_APLL_TUNER,
                            MT_CG_AUDIO_PDN_HDMI_CK,
    END_CG_GRP(CG_AUDIO,    MT_CG_AUDIO_PDN_SPDF_CK),

    // Virtual CGs
    
    BEG_CG_GRP(CG_VIRTUAL,  MT_CG_V_AD_AUDPLL_CK),
                            MT_CG_V_AD_LVDSPLL_CK,
                            MT_CG_V_AD_MAINPLL_CK,
                            MT_CG_V_AD_MMPLL_CK,
                            MT_CG_V_AD_MSDCPLL_CK,
                            MT_CG_V_AD_TVDPLL_CK,
                            MT_CG_V_AD_UNIVPLL_CK,
                            MT_CG_V_AD_VDECPLL_CK,
                            MT_CG_V_AXI_CK,
                            MT_CG_V_DPI0_CK,
                            MT_CG_V_26M_CK,
                            MT_CG_V_AD_DSI0_LNTC_DSICLK,
                            MT_CG_V_AD_HDMITX_CLKDIG_CTS,
                            MT_CG_V_AD_MEM2MIPI_26M_CK,
                            MT_CG_V_AD_MIPI_26M_CK,
                            MT_CG_V_CLKPH_MCK,
                            MT_CG_V_CPUM_TCK_IN,
                            MT_CG_V_PAD_RTC32K_CK,
                            MT_CG_V_AD_APLL_CK,
                            MT_CG_V_APLL_D16,
                            MT_CG_V_APLL_D24,
                            MT_CG_V_APLL_D4,
                            MT_CG_V_APLL_D8,
                            MT_CG_V_LVDSPLL_CLK,
                            MT_CG_V_LVDSPLL_D2,
                            MT_CG_V_LVDSPLL_D4,
                            MT_CG_V_LVDSPLL_D8,
                            MT_CG_V_AD_LVDSTX_CLKDIG_CTS,
                            MT_CG_V_AD_VPLL_DPIX_CK,
                            MT_CG_V_AD_TVHDMI_H_CK,
                            MT_CG_V_HDMITX_CLKDIG_D2,
                            MT_CG_V_HDMITX_CLKDIG_D3,
                            MT_CG_V_TVHDMI_D2,
                            MT_CG_V_TVHDMI_D4,
                            MT_CG_V_CKSQ_MUX_CK,
                            MT_CG_V_MEMPLL_MCK_D4,
                            MT_CG_V_RTC_CK,
                            MT_CG_V_AXI_DDRPHYCFG_AS_CK,
                            MT_CG_V_JPEG_CK,
                            MT_CG_V_MMPLL_D2,
                            MT_CG_V_MMPLL_D3,
                            MT_CG_V_MMPLL_D4,
                            MT_CG_V_MMPLL_D5,
                            MT_CG_V_MMPLL_D6,
                            MT_CG_V_MMPLL_D7,
                            MT_CG_V_SMI_DISP_CK,
                            MT_CG_V_SYSPLL_D10,
                            MT_CG_V_SYSPLL_D12,
                            MT_CG_V_SYSPLL_D16,
                            MT_CG_V_SYSPLL_D2,
                            MT_CG_V_SYSPLL_D24,
                            MT_CG_V_SYSPLL_D2P5,
                            MT_CG_V_SYSPLL_D3,
                            MT_CG_V_SYSPLL_D3P5,
                            MT_CG_V_SYSPLL_D4,
                            MT_CG_V_SYSPLL_D5,
                            MT_CG_V_SYSPLL_D6,
                            MT_CG_V_SYSPLL_D8,
                            MT_CG_V_UNIVPLL1_D10,
                            MT_CG_V_UNIVPLL1_D2,
                            MT_CG_V_UNIVPLL1_D4,
                            MT_CG_V_UNIVPLL1_D6,
                            MT_CG_V_UNIVPLL1_D8,
                            MT_CG_V_UNIVPLL2_D2,
                            MT_CG_V_UNIVPLL2_D4,
                            MT_CG_V_UNIVPLL2_D6,
                            MT_CG_V_UNIVPLL2_D8,
                            MT_CG_V_UNIVPLL_D10,
                            MT_CG_V_UNIVPLL_D26,
                            MT_CG_V_UNIVPLL_D3,
                            MT_CG_V_UNIVPLL_D5,
                            MT_CG_V_UNIVPLL_D7,
    END_CG_GRP(CG_VIRTUAL,  MT_CG_V_MIPI26M_CK),

    NR_CLKS,

    // Special CGs
    MT_CG_V_NULL,
    MT_CG_V_CLOCK_ON,
    MT_CG_V_CLOCK_OFF,
    MT_CG_V_PLL,
    MT_CG_V_MUX,
} cg_clk_id;

#if 0 // disable CG alias, use virtual CG instead
// CG alias
#define MT_CG_V_26M_CK                  MT_CG_V_CLOCK_ON
#define MT_CG_V_AD_DSI0_LNTC_DSICLK     MT_CG_V_CLOCK_ON
#define MT_CG_V_AD_HDMITX_CLKDIG_CTS    MT_CG_V_CLOCK_ON
#define MT_CG_V_AD_MEM2MIPI_26M_CK      MT_CG_V_CLOCK_ON
#define MT_CG_V_AD_MIPI_26M_CK          MT_CG_V_CLOCK_ON
#define MT_CG_V_CLKPH_MCK               MT_CG_V_CLOCK_ON
#define MT_CG_V_CPUM_TCK_IN             MT_CG_V_CLOCK_ON
#define MT_CG_V_PAD_RTC32K_CK           MT_CG_V_CLOCK_ON
#define MT_CG_V_AD_APLL_CK              MT_CG_V_AD_AUDPLL_CK
#define MT_CG_V_APLL_D16                MT_CG_V_AD_AUDPLL_CK
#define MT_CG_V_APLL_D24                MT_CG_V_AD_AUDPLL_CK
#define MT_CG_V_APLL_D4                 MT_CG_V_AD_AUDPLL_CK
#define MT_CG_V_APLL_D8                 MT_CG_V_AD_AUDPLL_CK
#define MT_CG_V_LVDSPLL_CLK             MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_LVDSPLL_D2              MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_LVDSPLL_D4              MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_LVDSPLL_D8              MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_AD_LVDSTX_CLKDIG_CTS    MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_AD_VPLL_DPIX_CK         MT_CG_V_AD_LVDSPLL_CK
#define MT_CG_V_AD_TVHDMI_H_CK          MT_CG_V_AD_TVDPLL_CK
#define MT_CG_V_HDMITX_CLKDIG_D2        MT_CG_V_AD_HDMITX_CLKDIG_CTS
#define MT_CG_V_HDMITX_CLKDIG_D3        MT_CG_V_AD_HDMITX_CLKDIG_CTS
#define MT_CG_V_TVHDMI_D2               MT_CG_V_AD_TVHDMI_H_CK
#define MT_CG_V_TVHDMI_D4               MT_CG_V_AD_TVHDMI_H_CK
#define MT_CG_V_CKSQ_MUX_CK             MT_CG_V_26M_CK
#define MT_CG_V_MEMPLL_MCK_D4           MT_CG_V_CLKPH_MCK
#define MT_CG_V_RTC_CK                  MT_CG_V_PAD_RTC32K_CK
#define MT_CG_V_AXI_DDRPHYCFG_AS_CK     MT_CG_V_AXI_CK
#define MT_CG_V_JPEG_CK                 MT_CG_TOP_PDN_JPG
#define MT_CG_V_MMPLL_D2                MT_CG_MIX_MMPLL_DIV2_EN
#define MT_CG_V_MMPLL_D3                MT_CG_MIX_MMPLL_DIV3_EN
#define MT_CG_V_MMPLL_D4                MT_CG_MIX_MMPLL_DIV2_EN
#define MT_CG_V_MMPLL_D5                MT_CG_MIX_MMPLL_DIV5_EN
#define MT_CG_V_MMPLL_D6                MT_CG_MIX_MMPLL_DIV3_EN
#define MT_CG_V_MMPLL_D7                MT_CG_MIX_MMPLL_DIV7_EN
#define MT_CG_V_SMI_DISP_CK             MT_CG_TOP_PDN_SMI
#define MT_CG_V_SYSPLL_D10              MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D12              MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D16              MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D2               MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D24              MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D2P5             MT_CG_MIX_MAINPLL_322P4M_EN
#define MT_CG_V_SYSPLL_D3               MT_CG_MIX_MAINPLL_537P3M_EN
#define MT_CG_V_SYSPLL_D3P5             MT_CG_MIX_MAINPLL_230P3M_EN
#define MT_CG_V_SYSPLL_D4               MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D5               MT_CG_MIX_MAINPLL_322P4M_EN
#define MT_CG_V_SYSPLL_D6               MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_SYSPLL_D8               MT_CG_MIX_MAINPLL_806M_EN
#define MT_CG_V_UNIVPLL1_D10            MT_CG_MIX_UNIVPLL_624M_EN
#define MT_CG_V_UNIVPLL1_D2             MT_CG_MIX_UNIVPLL_624M_EN
#define MT_CG_V_UNIVPLL1_D4             MT_CG_MIX_UNIVPLL_624M_EN
#define MT_CG_V_UNIVPLL1_D6             MT_CG_MIX_UNIVPLL_624M_EN
#define MT_CG_V_UNIVPLL1_D8             MT_CG_MIX_UNIVPLL_624M_EN
#define MT_CG_V_UNIVPLL2_D2             MT_CG_MIX_UNIVPLL_416M_EN
#define MT_CG_V_UNIVPLL2_D4             MT_CG_MIX_UNIVPLL_416M_EN
#define MT_CG_V_UNIVPLL2_D6             MT_CG_MIX_UNIVPLL_416M_EN
#define MT_CG_V_UNIVPLL2_D8             MT_CG_MIX_UNIVPLL_416M_EN
#define MT_CG_V_UNIVPLL_D10             MT_CG_MIX_UNIVPLL_249P6M_EN
#define MT_CG_V_UNIVPLL_D26             MT_CG_MIX_UNIVPLL_48M_EN
#define MT_CG_V_UNIVPLL_D3              MT_CG_MIX_UNIVPLL_416M_EN
#define MT_CG_V_UNIVPLL_D5              MT_CG_MIX_UNIVPLL_249P6M_EN
#define MT_CG_V_UNIVPLL_D7              MT_CG_MIX_UNIVPLL_178P3M_EN
#endif // 0 // disable CG alias, use virtual CG instead

#elif CLKMGR_6572

typedef enum
{
    // CG_MIXED
    BEG_CG_GRP(CG_MIXED, MT_CG_SYS_26M),
                      // MT_CG_UNIV_48M,
    END_CG_GRP(CG_MIXED, MT_CG_USB_48M),

    // CG_MPLL
    BEG_CG_GRP(CG_MPLL,  MT_CG_MPLL_D2),
                         MT_CG_MPLL_D3,
                         MT_CG_MPLL_D5,
                         MT_CG_MPLL_D7,

                         MT_CG_MPLL_D4,
                         MT_CG_MPLL_D6,
                         MT_CG_MPLL_D10,

                         MT_CG_MPLL_D8,
                         MT_CG_MPLL_D12,
                         MT_CG_MPLL_D20,

    END_CG_GRP(CG_MPLL,  MT_CG_MPLL_D24),

    // CG_UPLL
    BEG_CG_GRP(CG_UPLL,  MT_CG_UPLL_D2),
                         MT_CG_UPLL_D3,
                         MT_CG_UPLL_D5,
                         MT_CG_UPLL_D7,

                         MT_CG_UPLL_D4,
                         MT_CG_UPLL_D6,
                         MT_CG_UPLL_D10,

                         MT_CG_UPLL_D8,
                         MT_CG_UPLL_D12,
                         MT_CG_UPLL_D20,

                         MT_CG_UPLL_D16,
    END_CG_GRP(CG_UPLL,  MT_CG_UPLL_D24),

    // CG_CTRL0 (PERI/INFRA)
    BEG_CG_GRP(CG_CTRL0, MT_CG_PWM_MM_SW_CG),
                         MT_CG_CAM_MM_SW_CG,
                         MT_CG_MFG_MM_SW_CG,
                         MT_CG_SPM_52M_SW_CG,
                         MT_CG_MIPI_26M_DBG_EN,
                         MT_CG_DBI_BCLK_SW_CG,
                         MT_CG_SC_26M_CK_SEL_EN,
                         MT_CG_SC_MEM_CK_OFF_EN,

                         MT_CG_DBI_PAD0_SW_CG,
                         MT_CG_DBI_PAD1_SW_CG,
                         MT_CG_DBI_PAD2_SW_CG,
                         MT_CG_DBI_PAD3_SW_CG,
                         MT_CG_GPU_491P52M_EN,
                         MT_CG_GPU_500P5M_EN,

    END_CG_GRP(CG_CTRL0, MT_CG_ARMDCM_CLKOFF_EN),

    // CG_CTRL1 (PERI/INFRA)
    BEG_CG_GRP(CG_CTRL1, MT_CG_EFUSE_SW_CG),
                         MT_CG_THEM_SW_CG,
                         MT_CG_APDMA_SW_CG,
                         MT_CG_I2C0_SW_CG,
                         MT_CG_I2C1_SW_CG,
                      // MT_CG_AUX_SW_CG_MD,

                         MT_CG_NFI_SW_CG,
                         MT_CG_NFIECC_SW_CG,

                         MT_CG_DEBUGSYS_SW_CG,
                         MT_CG_PWM_SW_CG,
                         MT_CG_UART0_SW_CG,
                         MT_CG_UART1_SW_CG,
                         MT_CG_BTIF_SW_CG,
                         MT_CG_USB_SW_CG,
                         MT_CG_FHCTL_SW_CG,
                      // MT_CG_AUX_SW_CG_THERM,

                         MT_CG_SPINFI_SW_CG,
                         MT_CG_MSDC0_SW_CG,
                         MT_CG_MSDC1_SW_CG,

                         MT_CG_PMIC_SW_CG_AP,
                         MT_CG_SEJ_SW_CG,
                         MT_CG_MEMSLP_DLYER_SW_CG,

                         MT_CG_APXGPT_SW_CG,
                         MT_CG_AUDIO_SW_CG,
                         MT_CG_SPM_SW_CG,
                      // MT_CG_PMIC_SW_CG_MD,
                      // MT_CG_PMIC_SW_CG_CONN,
                         MT_CG_PMIC_26M_SW_CG,
    END_CG_GRP(CG_CTRL1, MT_CG_AUX_SW_CG_ADC),
                      // MT_CG_AUX_SW_CG_TP,

    // CG_MMSYS0
    BEG_CG_GRP(CG_MMSYS0, MT_CG_SMI_COMMON_SW_CG),
                          MT_CG_SMI_LARB0_SW_CG,
                          MT_CG_MM_CMDQ_SW_CG,
                          MT_CG_MM_CMDQ_SMI_IF_SW_CG,
                          MT_CG_DISP_COLOR_SW_CG,
                          MT_CG_DISP_BLS_SW_CG,
                          MT_CG_DISP_WDMA_SW_CG,
                          MT_CG_DISP_RDMA_SW_CG,
                          MT_CG_DISP_OVL_SW_CG,
                          MT_CG_MDP_TDSHP_SW_CG,
                          MT_CG_MDP_WROT_SW_CG,
                          MT_CG_MDP_WDMA_SW_CG,
                          MT_CG_MDP_RSZ1_SW_CG,
                          MT_CG_MDP_RSZ0_SW_CG,
                          MT_CG_MDP_RDMA_SW_CG,
                          MT_CG_MDP_BLS_26M_SW_CG,
                          MT_CG_MM_CAM_SW_CG,
                          MT_CG_MM_SENINF_SW_CG,
                          MT_CG_MM_CAMTG_SW_CG,
                          MT_CG_MM_CODEC_SW_CG,
                          MT_CG_DISP_FAKE_ENG_SW_CG,
    END_CG_GRP(CG_MMSYS0, MT_CG_MUTEX_SLOW_CLOCK_SW_CG),

    // CG_MMSYS1
    BEG_CG_GRP(CG_MMSYS1, MT_CG_DSI_ENGINE_SW_CG),
                          MT_CG_DSI_DIGITAL_SW_CG,
                          MT_CG_DISP_DPI_ENGINE_SW_CG,
                          MT_CG_DISP_DPI_IF_SW_CG,
                          MT_CG_DISP_DBI_ENGINE_SW_CG,
                          MT_CG_DISP_DBI_SMI_SW_CG,
    END_CG_GRP(CG_MMSYS1, MT_CG_DISP_DBI_IF_SW_CG),

    // CG_MFG
    BEG_END_CG_GRP(CG_MFG, MT_CG_MFG_PDN_BG3D_SW_CG),

    // CG_AUDIO
    BEG_CG_GRP(CG_AUDIO, MT_CG_AUD_PDN_AFE_EN),
                         MT_CG_AUD_PDN_I2S_EN,
                         MT_CG_AUD_PDN_ADC_EN,
                         MT_CG_AUD_PDN_DAC_EN,
                         MT_CG_AUD_PDN_DAC_PREDIS_EN,
    END_CG_GRP(CG_AUDIO, MT_CG_AUD_PDN_TML_EN),

    NR_CLKS,

    // for AUTO SEL
    MT_CG_SRC_DBI,      // MT_CG_MPLL_D12 (for DDR2) or MT_CG_MPLL_D10 (for DDR3)
    MT_CG_SRC_SMI,      // MT_CG_MPLL_D6 (for DDR2) or MT_CG_MPLL_D5 (for DDR3)

    // Special
    MT_CG_CLOCK_OFF,
    MT_CG_INVALID,
} cg_clk_id;

#define MT_CG_UNIV_48M MT_CG_USB_48M // alias

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// CLKMUX ID
//

#if CLKMGR_8135

typedef enum
{
    MT_CLKMUX_AXI_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D4, MT_CG_V_SYSPLL_D6, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL2_D2, MT_CG_V_SYSPLL_D3P5
    MT_CLKMUX_SMI_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_CLKPH_MCK, MT_CG_V_SYSPLL_D2P5, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D8, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL1_D2, MT_CG_V_UNIVPLL1_D6, MT_CG_V_MMPLL_D3, MT_CG_V_MMPLL_D4, MT_CG_V_MMPLL_D5, MT_CG_V_MMPLL_D6, MT_CG_V_MMPLL_D7, MT_CG_V_AD_VDECPLL_CK, MT_CG_V_LVDSPLL_CLK
    MT_CLKMUX_MFG_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL1_D4, MT_CG_V_SYSPLL_D2, MT_CG_V_SYSPLL_D2P5, MT_CG_V_SYSPLL_D3, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL1_D2, MT_CG_V_MMPLL_D2, MT_CG_V_MMPLL_D3, MT_CG_V_MMPLL_D4, MT_CG_V_MMPLL_D5, MT_CG_V_MMPLL_D6, MT_CG_V_MMPLL_D7
    MT_CLKMUX_IRDA_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL2_D8, MT_CG_V_UNIVPLL1_D6
    MT_CLKMUX_CAM_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D3P5, MT_CG_V_SYSPLL_D4, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL2_D2, MT_CG_V_UNIVPLL_D7, MT_CG_V_UNIVPLL1_D4
    MT_CLKMUX_AUD_INTBUS_SEL,       // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_UNIVPLL_D10
    MT_CLKMUX_JPG_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D5, MT_CG_V_SYSPLL_D4, MT_CG_V_SYSPLL_D3, MT_CG_V_UNIVPLL_D7, MT_CG_V_UNIVPLL2_D2, MT_CG_V_UNIVPLL_D5
    MT_CLKMUX_DISP_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D3P5, MT_CG_V_SYSPLL_D3, MT_CG_V_UNIVPLL2_D2, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL1_D2, MT_CG_V_LVDSPLL_CLK, MT_CG_V_AD_VDECPLL_CK
    MT_CLKMUX_MSDC30_1_SEL,         // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D5, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_MSDCPLL_CK
    MT_CLKMUX_MSDC30_2_SEL,         // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D5, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_MSDCPLL_CK
    MT_CLKMUX_MSDC30_3_SEL,         // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D5, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_MSDCPLL_CK
    MT_CLKMUX_MSDC30_4_SEL,         // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D5, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_MSDCPLL_CK
    MT_CLKMUX_USB20_SEL,            // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL2_D6, MT_CG_V_UNIVPLL1_D10
    MT_CLKMUX_VENC_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D8, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL1_D6, MT_CG_V_MMPLL_D4, MT_CG_V_MMPLL_D5, MT_CG_V_MMPLL_D6
    MT_CLKMUX_SPI_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D8, MT_CG_V_SYSPLL_D10, MT_CG_V_UNIVPLL1_D6, MT_CG_V_UNIVPLL1_D8
    MT_CLKMUX_UART_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL2_D8
    MT_CLKMUX_MEM_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_CLKPH_MCK
    MT_CLKMUX_CAMTG_SEL,            // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL_D26, MT_CG_V_UNIVPLL1_D6, MT_CG_V_SYSPLL_D16, MT_CG_V_SYSPLL_D8
    MT_CLKMUX_AUDIO_SEL,            // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D24
    MT_CLKMUX_FIX_SEL,              // MT_CG_V_RTC_CK, MT_CG_V_26M_CK, MT_CG_V_UNIVPLL_D5, MT_CG_V_UNIVPLL_D7, MT_CG_V_UNIVPLL1_D2, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL1_D6, MT_CG_V_UNIVPLL1_D8
    MT_CLKMUX_VDEC_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AD_VDECPLL_CK, MT_CG_V_CLKPH_MCK, MT_CG_V_SYSPLL_D2P5, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D3P5, MT_CG_V_SYSPLL_D4, MT_CG_V_SYSPLL_D5, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D8, MT_CG_V_UNIVPLL1_D2, MT_CG_V_UNIVPLL2_D2, MT_CG_V_UNIVPLL_D7, MT_CG_V_UNIVPLL_D10, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_LVDSPLL_CK
    MT_CLKMUX_DDRPHYCFG_SEL,        // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AXI_DDRPHYCFG_AS_CK, MT_CG_V_SYSPLL_D12
    MT_CLKMUX_DPILVDS_SEL,          // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AD_LVDSPLL_CK, MT_CG_V_LVDSPLL_D2, MT_CG_V_LVDSPLL_D4, MT_CG_V_LVDSPLL_D8
    MT_CLKMUX_PMICSPI_SEL,          // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_UNIVPLL2_D6, MT_CG_V_SYSPLL_D8, MT_CG_V_SYSPLL_D10, MT_CG_V_UNIVPLL1_D10, MT_CG_V_MEMPLL_MCK_D4, MT_CG_V_UNIVPLL_D26, MT_CG_V_SYSPLL_D24
    MT_CLKMUX_MSDC30_0_SEL,         // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D6, MT_CG_V_SYSPLL_D5, MT_CG_V_UNIVPLL1_D4, MT_CG_V_UNIVPLL2_D4, MT_CG_V_AD_MSDCPLL_CK
    MT_CLKMUX_SMI_MFG_AS_SEL,       // MT_CG_V_CKSQ_MUX_CK, MT_CG_TOP_PDN_SMI, MT_CG_TOP_PDN_MFG, MT_CG_TOP_PDN_MEM
    MT_CLKMUX_GCPU_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_SYSPLL_D4, MT_CG_V_UNIVPLL_D7, MT_CG_V_SYSPLL_D5, MT_CG_V_SYSPLL_D6
    MT_CLKMUX_DPI1_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AD_TVHDMI_H_CK, MT_CG_V_TVHDMI_D2, MT_CG_V_TVHDMI_D4
    MT_CLKMUX_CCI_SEL,              // MT_CG_V_CKSQ_MUX_CK, MT_CG_MIX_MAINPLL_537P3M_EN, MT_CG_V_UNIVPLL_D3, MT_CG_V_SYSPLL_D2P5, MT_CG_V_SYSPLL_D3, MT_CG_V_SYSPLL_D5
    MT_CLKMUX_APLL_SEL,             // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AD_APLL_CK, MT_CG_V_APLL_D4, MT_CG_V_APLL_D8, MT_CG_V_APLL_D16, MT_CG_V_APLL_D24
    MT_CLKMUX_HDMIPLL_SEL,          // MT_CG_V_CKSQ_MUX_CK, MT_CG_V_AD_HDMITX_CLKDIG_CTS, MT_CG_V_HDMITX_CLKDIG_D2, MT_CG_V_HDMITX_CLKDIG_D3

    NR_CLKMUXS,

    // Special
    MT_CLKMUX_INVALID,
} clkmux_id;

#elif CLKMGR_6572

typedef enum
{
    MT_CLKMUX_UART0_GFMUX_SEL,      // MT_CG_SYS_26M, MT_CG_UPLL_D24
    MT_CLKMUX_EMI2X_GFMUX_SEL,      // MT_CG_SYS_26M, MT_CG_MPLL_D3, MT_CG_MPLL_D4, MT_CG_MPLL_D2
    MT_CLKMUX_AXIBUS_GFMUX_SEL,     // MT_CG_SYS_26M, MT_CG_MPLL_D10, MT_CG_MPLL_D12
    MT_CLKMUX_MFG_MUX_SEL,          // MT_CG_UPLL_D3, MT_CG_GPU_491P52M_EN, MT_CG_GPU_500P5M_EN, MT_CG_MPLL_D3, MT_CG_UPLL_D2, MT_CG_SYS_26M, MT_CG_MPLL_D2 (MT_CLKMUX_MFG_GFMUX_SEL)
    MT_CLKMUX_MSDC0_MUX_SEL,        // MT_CG_MPLL_D12, MT_CG_MPLL_D10, MT_CG_MPLL_D8, MT_CG_UPLL_D7, MT_CG_MPLL_D7, MT_CG_MPLL_D8, MT_CG_SYS_26M, MT_CG_UPLL_D6
    MT_CLKMUX_SPINFI_MUX_SEL,       // MT_CG_SYS_26M, MT_CG_MPLL_D24, MT_CG_MPLL_D20, MT_CG_UPLL_D20, MT_CG_UPLL_D16, MT_CG_UPLL_D12, MT_CG_UPLL_D10, MT_CG_MPLL_D12, MT_CG_MPLL_D10 (MT_CLKMUX_SPINFI_GFMUX_SEL)
    MT_CLKMUX_CAM_MUX_SEL,          // MT_CG_UNIV_48M, MT_CG_UPLL_D6
    MT_CLKMUX_PWM_MM_MUX_SEL,       // MT_CG_SYS_26M, MT_CG_UPLL_D12
    MT_CLKMUX_UART1_GFMUX_SEL,      // MT_CG_SYS_26M, MT_CG_UPLL_D24
    MT_CLKMUX_MSDC1_MUX_SEL,        // MT_CG_MPLL_D12, MT_CG_MPLL_D10, MT_CG_MPLL_D8, MT_CG_UPLL_D7, MT_CG_MPLL_D7, MT_CG_MPLL_D8, MT_CG_SYS_26M, MT_CG_UPLL_D6
    MT_CLKMUX_SPM_52M_CK_SEL,       // MT_CG_SYS_26M, MT_CG_UPLL_D24
    MT_CLKMUX_PMICSPI_MUX_SEL,      // MT_CG_MPLL_D24, MT_CG_UNIV_48M, MT_CG_UPLL_D16, MT_CG_SYS_26M
    // MT_CLKMUX_AUD_HF_26M_SEL,       // MT_CG_SYS_26M
    MT_CLKMUX_AUD_INTBUS_SEL,       // MT_CG_SYS_26M, MT_CG_MPLL_D24, MT_CG_MPLL_D12

    NR_CLKMUXS,

    // Special
    MT_CLKMUX_INVALID,
} clkmux_id;

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// PLL ID
//

#if CLKMGR_8135

typedef enum
{
    ARMPLL1,
    ARMPLL2,
    MAINPLL,
    UNIVPLL,
    MMPLL,
    MSDCPLL,
    TVDPLL,
    LVDSPLL,
    AUDPLL,
    VDECPLL,

    NR_PLLS,
} pll_id;

#elif CLKMGR_6572

typedef enum
{
    ARMPLL,
    MAINPLL,
    UNIVPLL,
    WHPLL,

    NR_PLLS,
} pll_id;

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// SUBSYS ID
//

#if CLKMGR_8135

typedef enum
{
    SYS_VDE,
    SYS_MFG_2D,
    SYS_MFG,
    SYS_VEN,
    SYS_ISP,
    SYS_DIS,
    SYS_IFR,

    NR_SYSS,
} subsys_id;

#elif CLKMGR_6572

typedef enum
{
    SYS_MD1,
    SYS_CON,
    SYS_DIS,
    SYS_MFG,

    NR_SYSS,
} subsys_id;

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

//
// LARB ID
//

#if CLKMGR_8135

typedef enum
{
    MT_LARB0,
    MT_LARB1,
    MT_LARB2,
    MT_LARB3,
    MT_LARB4,

    NR_LARBS,
} larb_id;

#elif CLKMGR_6572

typedef enum
{
    MT_LARB0,

    NR_LARBS,
} larb_id;

#else // !(CLKMGR_8135 || CLKMGR_6572)
    #error
#endif // CLKMGR_8135

/* larb monitor mechanism definition*/
enum
{
    LARB_MONITOR_LEVEL_HIGH     = 10,
    LARB_MONITOR_LEVEL_MEDIUM   = 20,
    LARB_MONITOR_LEVEL_LOW      = 30,
};

struct larb_monitor
{
    struct list_head link;
    int level;
    void (*backup)(struct larb_monitor *h, int larb_idx);       /* called before disable larb clock */
    void (*restore)(struct larb_monitor *h, int larb_idx);      /* called after enable larb clock */
};


/*=============================================================*/
// Global variable definition
/*=============================================================*/


/*=============================================================*/
// Global function definition
/*=============================================================*/

#if 0 // defined(CONFIG_MT8135_FPGA)

static int enable_clock(int id, char *mod_name) { return 0; }
static int disable_clock(int id, char *mod_name) { return 0; }
static int clock_is_on(int id) { return 0; }
static int enable_pll(int id, char *mod_name) { return 0; }
static int disable_pll(int id, char *mod_name) { return 0; }
static int pll_hp_switch_on(int id, int hp_on) { return 0; }
static int pll_is_on(int id) { return 0; }
static int subsys_is_on(int id) { return 0; }

#else // !defined(CONFIG_MT8135_FPGA)

EXTERN int enable_clock(cg_clk_id id, const char *name);
EXTERN int disable_clock(cg_clk_id id, const char *name);
EXTERN int clock_is_on(cg_clk_id id);

EXTERN int clkmux_sel(clkmux_id id, cg_clk_id clksrc, const char *name);
EXTERN int clkmux_get(clkmux_id id, const char *name);

EXTERN int enable_pll(pll_id id, const char *name);
EXTERN int disable_pll(pll_id id, const char *name);
EXTERN int pll_hp_switch_on(pll_id id, int hp_on);
EXTERN int pll_fsel(pll_id id, unsigned int value);
EXTERN int pll_is_on(pll_id id);

EXTERN int enable_subsys(subsys_id id, const char *name);
EXTERN int disable_subsys(subsys_id id, const char *name);
EXTERN int disable_subsys_force(subsys_id id, const char *name);
EXTERN void register_larb_monitor(struct larb_monitor *handler);
EXTERN void unregister_larb_monitor(struct larb_monitor *handler);
EXTERN int subsys_is_on(subsys_id id);

#endif // defined(CONFIG_MT8135_FPGA)

#if BUILD_WORKAROUND_6589_COMPATIBLE

static void clk_clr_force_on(int id) { return 0; }
static int clk_is_force_on(int id) { return 0; }

#endif // BUILD_WORKAROUND_6589_COMPATIBLE

#if CLKMGR_6572

EXTERN int md_power_on(subsys_id id);
EXTERN int md_power_off(subsys_id id, unsigned int timeout);
EXTERN int conn_power_on(subsys_id id);
EXTERN int conn_power_off(subsys_id id, unsigned int timeout);

#endif // CLKMGR_6572

EXTERN const char *grp_get_name(cg_grp_id id);
EXTERN int grp_dump_regs(cg_grp_id id, unsigned int *ptr);
EXTERN const char *pll_get_name(pll_id id);
EXTERN int pll_dump_regs(pll_id id, unsigned int *ptr);
EXTERN const char *subsys_get_name(subsys_id id);
EXTERN int subsys_dump_regs(subsys_id id, unsigned int *ptr);

#if CLKMGR_6572

EXTERN unsigned int mt_get_emi_freq(void);
EXTERN unsigned int mt_get_bus_freq(void);
EXTERN unsigned int mt_get_cpu_freq(void);

#endif // CLKMGR_6572

EXTERN int snapshot_golden_setting(const char *func, const unsigned int line);

EXTERN void print_mtcmos_trace_info_for_met(void);

#if CLKMGR_6572
EXTERN bool is_ddr3(void);
#endif // CLKMGR_6572

EXTERN bool clkmgr_idle_can_enter(unsigned int *condition_mask, unsigned int *block_mask);

EXTERN cg_grp_id clk_id_to_grp_id(cg_clk_id id);
EXTERN unsigned int clk_id_to_mask(cg_clk_id id);

EXTERN int clkmgr_is_locked(void);

EXTERN int mt_clkmgr_init(void);


#undef EXTERN

#ifdef __cplusplus
// } // TODO: disable temp
#endif

#if CLKMGR_6572

// alps/kernel/mediatek/platform/mt6572/kernel/core/mt_freqhopping.c
#define MSDCPLL_CON0            (APMIXED_BASE + 0x0250) // TODO: remove it latter for build pass
#define MSDCPLL_CON1            (APMIXED_BASE + 0x0254) // TODO: remove it latter for build pass
#define TVDPLL_CON0             (APMIXED_BASE + 0x026C) // TODO: remove it latter for build pass
#define TVDPLL_CON1             (APMIXED_BASE + 0x0270) // TODO: remove it latter for build pass
#define LVDSPLL_CON0            (APMIXED_BASE + 0x0288) // TODO: remove it latter for build pass
#define LVDSPLL_CON1            (APMIXED_BASE + 0x028C) // TODO: remove it latter for build pass

#endif // CLKMGR_6572

#if BUILD_WORKAROUND_6589_COMPATIBLE

#define ISPPLL_CON0                 0
#define MT_CG_PERI0_UART0           0
#define MT_CG_PERI0_UART1           0
#define MT_CG_PERI0_UART2           0
#define MT_CG_PERI0_UART3           0
#define MT_CG_IMAGE_CAM_SMI         0
#define MT_CG_IMAGE_CAM_CAM         0
#define MT_CG_IMAGE_SEN_TG          0
#define MT_CG_IMAGE_SEN_CAM         0
#define MT_CG_IMAGE_LARB3_SMI       0
#define MT_CG_IMAGE_LARB4_SMI       0
#define MT_CG_IMAGE_COMMN_SMI       0
#define MT_CG_PERI1_FHCTL           0
#define MT_CG_PERI0_USB0            0
#define MT_CG_SYS_26M               0
#define MT_CG_DISP0_ROT_ENGINE      0
#define MT_CG_DISP0_ROT_SMI         0
#define MT_CG_DISP0_SCL             0
#define MT_CG_DISP0_WDMA0_ENGINE    0
#define MT_CG_DISP0_WDMA0_SMI       0
#define MT_CG_DISP0_2DSHP           0
#define MT_CG_DISP0_G2D_ENGINE      0
#define MT_CG_DISP0_G2D_SMI         0
#define MT_CG_DISP0_LARB2_SMI       0
#define MT_CG_DISP0_WDMA1_ENGINE    0
#define MT_CG_DISP0_WDMA1_SMI       0
#define MT_CG_DISP0_RDMA1_ENGINE    0
#define MT_CG_DISP0_RDMA1_SMI       0
#define MT_CG_DISP0_RDMA1_OUTPUT    0
#define MT_CG_DISP0_GAMMA_ENGINE    0
#define MT_CG_DISP0_GAMMA_PIXEL     0
#define MT_CG_DISP1_DBI_ENGINE      0
#define MT_CG_PERI0_I2C0            0
#define MT_CG_PERI0_I2C1            0
#define MT_CG_PERI0_I2C2            0
#define MT_CG_PERI0_I2C3            0
#define MT_CG_PERI0_I2C4            0
#define MT_CG_PERI0_I2C5            0
#define MT_CG_PERI1_I2C6            0
#define MT_CG_INFRA_M4U             0
#define MT_CG_INFRA_SMI             0
#define MT_CG_IMAGE_JPGD_SMI        0
#define MT_CG_IMAGE_JPGD_JPG        0
#define MT_CG_IMAGE_JPGE_SMI        0
#define MT_CG_IMAGE_JPGE_JPG        0
#define MT_CG_DISP0_BLS             0
#define MT_CG_PERI0_PWM1            0
#define MT_CG_PERI0_PWM2            0
#define MT_CG_PERI0_PWM3            0
#define MT_CG_PERI0_PWM4            0
#define MT_CG_PERI0_PWM5            0
#define MT_CG_PERI0_PWM6            0
#define MT_CG_PERI0_PWM7            0
#define MT_CG_PERI0_PWM             0
#define MT_CG_DISP0_OVL_ENGINE      0
#define MT_CG_DISP0_OVL_SMI         0
#define MT_CG_DISP0_COLOR           0
#define MT_CG_DISP0_RDMA0_ENGINE    0
#define MT_CG_DISP0_RDMA0_SMI       0
#define MT_CG_DISP0_RDMA0_OUTPUT    0
#define MT_CG_PERI1_SPI1            0
#define MT_CG_AUDIO_AFE             0
#define MT_CG_AUDIO_I2S             0
#define MT_CG_PERI0_APDMA           0
#define MT_CG_VENC_VEN              0
#define MT_CG_VDEC0_VDE             0
#define MT_CG_VDEC1_SMI             0
#define MT_CG_DISP1_DPI1            0
#define MT_CG_DISP1_SLCD            0
#define MT_CG_DISP1_LCD             0
#define MT_CG_DISP1_DBI_SMI         0
#define MT_CG_DISP1_DBI_OUTPUT      0
#define MT_CG_DISP1_DPI0            0
#define MT_CG_DISP1_DSI_ENGINE      0
#define MT_CG_DISP1_DSI_DIGITAL     0
#define MT_CG_DISP1_DSI_DIGITAL_LANE    0

#endif // BUILD_WORKAROUND_6589_COMPATIBLE

#endif // __MT_CLKMGR_H__
