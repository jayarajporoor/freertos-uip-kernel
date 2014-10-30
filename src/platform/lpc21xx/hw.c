#include <FreeRTOS.h>
#include <hw.h>
#include <board_config.h>


#define VAL_PLL_MUL			(configCPU_CLOCK_MHZ/BOARD_CONFIG_FOSC_MHZ - 1) 
/*Fcco = cclk * 2 * PLL_DIV. Fcc0 MUST BE from 156MHz to 320MHz. See PLL Sec. in LPC UM*/
#define VAL_FCCO_MIN_MHZ	156
#define VAL_FCCO_MAX_MHZ	320 

#if configCPU_CLOCK_MHZ*1 >= VAL_FCCO_MIN_MHZ
#define VAL_PLL_DIV			0
#elif configCPU_CLOCK_MHZ*2 >= VAL_FCCO_MIN_MHZ
#define VAL_PLL_DIV			1
#elif configCPU_CLOCK_MHZ*4 >= VAL_FCCO_MIN_MHZ
#define VAL_PLL_DIV			2
#elif configCPU_CLOCK_MHZ*8 >= VAL_FCCO_MIN_MHZ
#define VAL_PLL_DIV			3
#else
#error "No valid PLL Divider value exists for selected CCLK/FOSC combination!"
#endif


/* Constants to setup I/O. */
#define VAL_TX_ENABLE	( ( unsigned portLONG ) 0x0001 )
#define VAL_RX_ENABLE	( ( unsigned portLONG ) 0x0004 )
#define VAL_P0_14		( ( unsigned portLONG ) 0x4000 )

/* Constants to setup the PLL. */
#define VAL_PLL_DISABLE		( ( unsigned portCHAR ) 0x0000 )
#define VAL_PLL_ENABLE		( ( unsigned portCHAR ) 0x0001 )
#define VAL_PLL_CONNECT		( ( unsigned portCHAR ) 0x0003 )
#define VAL_PLL_FEED_BYTE1	( ( unsigned portCHAR ) 0xaa )
#define VAL_PLL_FEED_BYTE2	( ( unsigned portCHAR ) 0x55 )
#define VAL_PLL_LOCK		( ( unsigned portLONG ) 0x0400 )

/* Constants to setup the MAM. */
#define VAL_MAM_TIM_3		( ( unsigned portCHAR ) 0x03 )
#define VAL_MAM_TIM_4		( ( unsigned portCHAR ) 0x04 )
#define VAL_MAM_MODE_FULL	( ( unsigned portCHAR ) 0x02 )
#define VAL_MAM_MODE_NONE	( ( unsigned portCHAR ) 0x00 )

/* Constants to setup the peripheral bus. */
#define VAL_BUS_CLK_FULL	( ( unsigned portCHAR ) 0x01 )
#define VAL_BUS_CLK_HALF    ( ( unsigned portCHAR ) 0x02 )
#define VAL_BUS_CLK_QUART   ( ( unsigned portCHAR ) 0x00 )

#define VAL_ENABLE_FIO 		( ( unsigned portLONG ) 0x03 ) 

void (*hw_app_fiq_handler)(void);


int hw_init(int run_from_ram)
{		
	if(run_from_ram)
	{
		/* Remap the interrupt vectors to RAM if we are are running from RAM. */
		SCB_MEMMAP = 2;
	}	

	POWER_ON(PCONP_TIM0);//Timer0 for multitasking 
	POWER_ON(PCONP_USB); //we're using USB memory 
	
	//SCS = VAL_ENABLE_FIO;
	SCS = 0;//no fast GPIO

	
	/* Disable PLL first. */
	/*SCB_PLLCON = VAL_PLL_DISABLE;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE1;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE2;*/

	/* Setup the PLL to multiply the XTAL input. */
	SCB_PLLCFG = (unsigned portCHAR)( VAL_PLL_MUL | (VAL_PLL_DIV<<5) );
	SCB_PLLFEED = VAL_PLL_FEED_BYTE1;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE2;

	/* Activate the PLL by turning it on then feeding the correct sequence of
	bytes. */
	SCB_PLLCON = VAL_PLL_ENABLE;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE1;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE2;
	
	/* Wait for the PLL to lock... */
	while( !( SCB_PLLSTAT & VAL_PLL_LOCK ) );
	
	/* ...before connecting it using the feed sequence again. */
	SCB_PLLCON = VAL_PLL_CONNECT;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE1;
	SCB_PLLFEED = VAL_PLL_FEED_BYTE2;

	/* Setup and turn on the MAM.  Three cycle access is used due to the fast
	PLL used.  It is possible faster overall performance could be obtained by
	tuning the MAM and PLL settings. */
	MAM_CR = VAL_MAM_MODE_NONE;
	MAM_TIM = VAL_MAM_TIM_4;
	MAM_CR = VAL_MAM_MODE_FULL;

	/* Setup the peripheral bus to be the same as the PLL output. */
	SCB_VPBDIV = VAL_BUS_CLK_FULL;
	
	board_config();
		
	return 0;	
}


void hw_fiq_handler()
{
	if(hw_app_fiq_handler != 0UL)
	{
		(*hw_app_fiq_handler)();
	}
	
	asm volatile("SUBS PC, LR, #4	\n\t");
}
