/***************************************************************************
Module: ssp_drvr.c
Description: driver file for the ssp port for LED Display board
Version     Date           Author    Description
0.1       30Jun2007        Sudharsan Driver file for display board 
****************************************************************************/

#include <lpc21xx.h>
#include <FreeRTOS.h>                     
#include <task.h>
#include <queue.h>
#include <errno.h>
#include <board_config.h>
#include <res.h>
#include <typedefs.h>
#include <ssp.h>

// Set Match Reg value = 600 for counting 96 bit times(6 frames) on SSP Clk (PCLK/8) + 4 PClk
#define SSP_SSEL_LOW_PERIOD  600
 
#define SSP_FREE 0
#define SSP_OPEN 1
#define SSP_BUSY 2

#define SSP_DSS         (15UL << 0)
#define SSP_FRF         ( 0UL << 4)
#define SSP_CKPOL       ( 0UL << 6)
#define SSP_CKPHA       ( 0UL << 7)
#define SSP_CKRATE      ( 3UL << 8)
/* SSP Serial Clock Rate i.e the bit frequency is calculated as PCLK/(CPSDVSR * [SCR + 1]) */
/* Here we are setting the clock rate to be 3 and the CPSDVSR as 2 so the Serial Clock Rate will be PCLK / (2 * 4) */

#define SSP_NORMAL_MODE (0UL << 0)
#define SSP_ENABLE      (1UL << 1)
#define SSP_MASTER_MODE (0UL << 2)
#define SSP_SLV_OP_DIS  (1UL << 3)


#define SSP_CPSDVSR    2

#define SSP_ST_TFE    (1UL << 3)
#define SSP_ST_TNF    (1UL << 4)
#define SSP_ST_RNE    (1UL << 5)
#define SSP_ST_RFF    (1UL << 6)
#define SSP_ST_BSY    (1UL << 7)


/* Board Config Defines */
#define BOARD_CONFIG_PORT0_SPI1_SSEL   (1UL << 8) 

static int8 state;
static uint32 vector; 
static wait_struct_t *curr_ws;

static void ssp_tmr1_isr( void ) __attribute__ ((naked));

void ssp_ssel_high();
void ssp_ssel_low();

int32 ssp_open()
{
    int32 status=0;

    if(SSP_OPEN == state)
    {
         return -EBUSY;
    }

    portENTER_CRITICAL();
    {
        if(SSP_BUSY == state)
        {
            status = -EBUSY;
        }else
        {
            state = SSP_BUSY;
        }
    }
    portEXIT_CRITICAL();
    
    if(status < 0)
    {
        return status;
    }
    portENTER_CRITICAL();
    {    
    	if(res_port0_acquire_(__FILE__, 
    					BIT(PIN_SSP_SCK)|
    					BIT(PIN_SSP_MISO)|
    					BIT(PIN_SSP_MOSI)|
    					BIT(BOARD_CONFIG_PORT0_SSP_SSEL)
    					) )
    	{
        	if(!(vector = res_vic_vect_acquire_any_(__FILE__)))
        	{
            	res_port0_release_( __FILE__, 
            						BIT(PIN_SSP_SCK)|
    								BIT(PIN_SSP_MISO)|
    								BIT(PIN_SSP_MOSI)|
    								BIT(BOARD_CONFIG_PORT0_SSP_SSEL)
            						);
            	status = -EBUSY;
        	}
    	}else
    	{
        	status = -EBUSY;
    	}
    }
    portEXIT_CRITICAL();
    
    if(status < 0)
    {
        return status;
    }
        
    POWER_ON(PCONP_SPI1);
       
	PINSEL_HI(PIN_SSP_SCK, PIN_FN2); 
	PINSEL_HI(PIN_SSP_MISO, PIN_FN2); 
	PINSEL_HI(PIN_SSP_MOSI, PIN_FN2); 
	PINSEL_HI(PIN_SSP_SSEL, PIN_FN1);//timer-compare 
    
	// Setting VIC interrupt Vector 

	VIC_VECT_ENABLE(vector, VIC_CH_TIM1,ssp_tmr1_isr);
	VIC_CH_IRQ_ENABLE(VIC_CH_TIM1);
  
    // setup SPI interface :
    // Master mode, MSB first
   
    SSP_CR0 = SSP_DSS|SSP_FRF|SSP_CKPOL|SSP_CKPHA|SSP_CKRATE;
    SSP_CPSR = SSP_CPSDVSR; 
    SSP_CR1 = SSP_NORMAL_MODE|SSP_ENABLE|SSP_MASTER_MODE|SSP_SLV_OP_DIS;  
    
//    res_timer1_acquire_(); 

    ssp_ssel_high(); 
    state = SSP_OPEN;
    
    return status;
    
}

void ssp_ssel_high(void)
{
	unsigned int value;
	//this is called once during initialization
	//not from a critical section
	portENTER_CRITICAL();
	{
		T1_TCR = (0x2); // Disable and Reset Timer1
		T1_PR = (1UL << 0); // Set Prescaler Register to 1 
		T1_MR3 = (1UL << 4);  // Set Match Register value as 16 for matching with timer fast 
		T1_MCR = (3UL << 10); // No Interrupt, Turn on Reset, Disable Timer on match with timer  
		T1_EMR = (1UL << 11); // Drive Matchpin M1.3 high on match with timer  
		T1_TCR = 0x01 ; // Enable Timer1
		value = (T1_EMR & 0x08);
	}
	portEXIT_CRITICAL();
	while (value == 0)  // Wait for Match1.3 bit to get set 
	{
		value = (T1_EMR & 0x08);
	} 
}
   
void ssp_ssel_low(void)
{
	//this is called from critical section
	//inside ssp_write_word 
	T1_CTCR = (0UL << 0); //Set in Counter in Timer Mode  
	T1_TCR = (0x2); // Disable and Reset Timer1
	T1_PR = (1UL << 0); // Set Prescaler Register to 1 
	T1_MR3 = 8;  // Set Match Register value as 2 for matching with timer fast 
	T1_EMR = (1UL << 10); // Drive low Matchpin M1.3 on match with timer  
	T1_MCR = (1UL<<11); // Set MCR values for no interrupt, no timer reset and timer disable on match
	T1_TCR = 0x01 ; // Enable Timer1
	T1_MR3 = SSP_SSEL_LOW_PERIOD;   
	T1_EMR = (1UL << 11); // Drive high Matchpin M1.3 on match with timer  
	T1_MCR = (7UL << 9); // Set MCR values for Interrupt,no timer reset and no timer disable on match
	T1_TCR = 0x01 ; // Enable Timer1
}
  
static int32 ssp_op_status;

int32 ssp_write_word(uint16* text, int32 num_of_words)
{
	uint16 i;
	
	if( (num_of_words < 0) || (num_of_words >= SSP_MAX_WORDS))
	{
		return -EINVAL;
	} 
 	
 	curr_ws = CURR_WAIT_STRUCT();
 	
 	portENTER_CRITICAL();
 	{
		ssp_ssel_low();
		SSP_DR = *(text);
		for (i=1;i<num_of_words;i++)
		{
 			SSP_DR = *(text+i); 
		}
 	}
 	portEXIT_CRITICAL();
 	
 	TASK_WAIT(curr_ws, &ssp_op_status);
 	return num_of_words;
} 

static void ssp_tmr1_isr(void)
{
  	portENTER_SWITCHING_ISR();
   
	T1_IR |= (1UL << 3); 
	
	TASK_NOTIFY_FROM_ISR(curr_ws,&ssp_op_status);		

	VIC_CLEAR_IRQ();
	portEXIT_SWITCHING_ISR(1);
}
