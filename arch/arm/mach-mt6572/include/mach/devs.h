#ifndef __DEVS_H__
#define __DEVS_H__

#include <board-custom.h>
#include <mach/board.h>

#define CFG_DEV_UART1
#define CFG_DEV_UART2

/*
 * Define constants.
 */

#define MTK_UART_SIZE 0x100

/*
 * Define function prototype.
 */
#define CONFIG_LGE_HANDLE_PANIC 1

#if defined(CONFIG_LGE_HANDLE_PANIC) || defined(CONFIG_LGE_HIDDEN_RESET)
	// RESERVED_ADDR_END	:
	// HIDDEN_RESET 			:
	// CTX					:
	// CRASH_LOG			:
	// RAM_CONSOLE			:
	// RESERVED_MEM 			:

	#define LGE_BSP_MEM_BASE_ADDR			0x80000000
#if defined(TARGET_S4)
#if 1 // ONE_BIN_MEMORY
	extern unsigned long LGE_BSP_MEM_MAX_PHY_ADDR;
#else
	#define LGE_BSP_MEM_MAX_PHY_ADDR		(LGE_BSP_MEM_BASE_ADDR+(1024 * SZ_1M))
#endif
#else
	#define LGE_BSP_MEM_MAX_PHY_ADDR		(LGE_BSP_MEM_BASE_ADDR+(512 * SZ_1M))
#endif

	#define LGE_BSP_RESERVED_MEM_SIZE		(1 * SZ_1M)
	#define LGE_BSP_RESERVED_MEM_PHY_ADDR	(LGE_BSP_MEM_MAX_PHY_ADDR - LGE_BSP_RESERVED_MEM_SIZE)

	#define LGE_BSP_RAM_CONSOLE_PHY_ADDR	(LGE_BSP_RESERVED_MEM_PHY_ADDR)
	#define LGE_BSP_RAM_CONSOLE_SIZE    	(124 * 2 * SZ_1K)

	#define LGE_BSP_CRASH_LOG_PHY_ADDR		(LGE_BSP_RAM_CONSOLE_PHY_ADDR + LGE_BSP_RAM_CONSOLE_SIZE)
	#define LGE_BSP_CRASH_LOG_SIZE      	(4 * SZ_1K)

	#define LGE_CRASH_CTX_BUF_PHY_ADDR 		(LGE_BSP_CRASH_LOG_PHY_ADDR + LGE_BSP_CRASH_LOG_SIZE)
	#define	LGE_CRASH_CTX_BUF_SIZE			(1 * SZ_1K)

	#define LGE_BSP_HIDDEN_RESET_PHY_ADDR	(LGE_CRASH_CTX_BUF_PHY_ADDR + LGE_CRASH_CTX_BUF_SIZE)
	#define LGE_BSP_HIDDEN_RESET_SIZE      	(LGE_BSP_MEM_MAX_PHY_ADDR - LGE_BSP_HIDDEN_RESET_PHY_ADDR)


		// these are the standard values. used in lge_save_boot_reason(), lge_get_boot_reason()
		// use these values if need in other files
	#define LGE_BOOT_REASON_MAGIC_CODE		0x1234ABCD
		// boot reason value
		//	prefix, postfix 1 bytes (0xff) so, 0xFFxxxxFF
	#define LGE_BOOT_KERNEL_CRASH 				0xFF0001FF
	#define LGE_BOOT_HIDDEN_RESET_REBOOT		0xFF0002FF
	#define LGE_BOOT_BNR_RECOVERY_MODE_REBOOT	0x77665555
	/* LGE_CHANGE_S: [2012-11-16] duvallee.lee@lge.com	: add magic number for normal power-off */
    #define LGE_BOOT_NORMAL_POWER_OFF           0xFF0003FF
	/* LGE_CHANGE_E: [2012-11-16] duvallee.lee@lge.com	: add magic number for normal power-off */

		int lge_save_boot_reason(unsigned long reason, unsigned long extra1, unsigned long extra2);
		unsigned long lge_get_boot_reason(unsigned long *pExtra1, unsigned long * pExtra2);

#endif

	/*
	 * Define function prototype.
	 */

	 /* LGE_CHANGE: lge crash handler
	  * 2012-07-11, kyeongdon.kim@lge.com  */
#if defined(CONFIG_LGE_HANDLE_PANIC)
	void __init lge_add_panic_handler_devices(void);
#endif

extern int mt_board_init(void);

//extern unsigned int *get_modem_size_list(void);
//extern unsigned int get_nr_modem(void);

#endif 

