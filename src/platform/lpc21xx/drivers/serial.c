
/*! \file serial.c
    \brief Serial driver for lpc21xx.
*/   
/* Standard includes. */
#include <stdlib.h>
#include <typedefs.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo application includes. */
#include "serial.h"
#include <board_config.h>
#include <errno.h>
#include <res.h>

#define DEFAULT_BAUD 9600

typedef struct
{
	int32 baud;
	uint8 dlm;
	uint8 dll;	
	uint8 mul;
	uint8 divadd;
} serial_baud_t;

#if configCPU_CLOCK_MHZ != 60
#error "Serial baud rate values computed only for 60MHz pclk"
#endif
/*
 * These values are computed assuming pclk = 60 Mhz
 */
static const serial_baud_t bauds[] = {
	{	9600,	1, 23,  5, 2  },
	{  19200,	0, 93, 10, 11 },
	{  57600,	0, 31, 10, 11 }
};

#define NUM_BAUDS (sizeof(bauds)/sizeof(serial_baud_t))
			
			
/* Constants to setup and access the UART. */
#define SER_DLAB						( ( uint8 ) 0x80 )
#define SER_ENABLE_INTERRUPTS			( ( uint8 ) 0x03 )
#define SER_NO_PARITY					( ( uint8 ) 0x00 )
#define SER_1STOP_BIT					( ( uint8 ) 0x00 )
#define SER_8BIT_CHARS					( ( uint8 ) 0x03 )
#define SER_FIFO_ON						( ( uint8 ) 0x01 )
#define SER_CLEAR_FIFO					( ( uint8 ) 0x06 )
#define SER_WANTED_CLOCK_SCALING		( ( uint32 ) 16 )

#define SER_NO_BLOCK					( ( portTickType ) 0 )

#define SER_UART0						0
#define SER_UART1						1

/* Queues used to hold received characters, and characters waiting to be
transmitted. */
static xQueueHandle rx_q[BOARD_CONFIG_NUART]; 
static xQueueHandle tx_q[BOARD_CONFIG_NUART];

#define UART_FREE		0
#define UART_OPEN		1
#define UART_BUSY		2
 
static uint8		uart_state[BOARD_CONFIG_NUART];
static uint32		uart_vectors[BOARD_CONFIG_NUART];

/* Communication flag between the interrupt service routine and serial API. */
static volatile int32 thre_empty[BOARD_CONFIG_NUART];

/* Constants to determine the ISR source. */
#define SER_SOURCE_THRE					( ( uint8 ) 0x02 )
#define SER_SOURCE_RX_TIMEOUT			( ( uint8 ) 0x0c )
#define SER_SOURCE_ERROR				( ( uint8 ) 0x06 )
#define SER_SOURCE_RX					( ( uint8 ) 0x04 )
#define SER_INTERRUPT_SOURCE_MASK		( ( uint8 ) 0x0f )


/* 
 * create xmit/receive queues  
 */
void serial_isr_create_queues(int port, int32 queue_len);

/* UART interrupt service routines.  This can cause a context switch so MUST
be declared "naked". */
static void uart0_isr( void ) __attribute__ ((naked));
static void uart1_isr( void ) __attribute__ ((naked));

int serial_baud_get_vals(int32 baud, uint8 *dll, uint8 *dlm, uint8 *mul, uint8 *divadd);

int32 serial_open (uint8 port, uint32 mode)
{
	int32				status; 
	uint32 				vect;	
	uint8 				dll_val;
	uint8				dlm_val;
	uint8				mul_val;
	uint8				divadd_val;
	volatile uint32		*lcr;
	volatile uint32		*dll;
	volatile uint32		*dlm;
	volatile uint32		*fdr;
	volatile uint32		*fcr;
	volatile uint32		*ier;
	
	status = 1;
	
	
	if(port >= BOARD_CONFIG_NUART)
	{
		return -EINVAL;
	}
	
	if(UART_FREE != uart_state[port])
	{
		return -EBUSY;
	}
	
	portENTER_CRITICAL();
	{
		if(UART_BUSY == uart_state[port])
		{
			status = -EBUSY;
		}else
		{
			uart_state[port] = UART_BUSY;
		}
	}
	portEXIT_CRITICAL();
	
	if(status < 0)
	{
		return status;
	}

	/* Create rx/tx queues if not already done */
	if((xQueueHandle)0 == rx_q[port])
	{
		serial_isr_create_queues( port, SER_MAX_BUF);
	}
	
	if( ( rx_q[port] != (xQueueHandle)0 ) && 
		( tx_q[port] != (xQueueHandle)0 ) 
	   )
	{
		portENTER_CRITICAL();
		{
			status = -EBUSY;
			if(0==port)
			{
				if(res_port0_acquire_(__FILE__, PIN_UART0_TX|PIN_UART0_RX))
				{
					if((vect=res_vic_vect_acquire_any_(__FILE__)))
					{
						POWER_ON(PCONP_UART0);
						PINSEL(PIN_UART0_TX, PIN_FN1);
						PINSEL(PIN_UART0_RX, PIN_FN1);
						uart_vectors[0] = vect;
						status = 0;		
					}else
					{
						res_port0_release_(__FILE__, PIN_UART0_TX|PIN_UART0_RX);
					}
				}
			}else
			{
				if(res_port0_acquire_(__FILE__, PIN_UART1_TX|PIN_UART1_RX))
				{
					if((vect=res_vic_vect_acquire_any_(__FILE__)))
					{
						POWER_ON(PCONP_UART1);
						PINSEL(PIN_UART1_TX, PIN_FN1);
						PINSEL(PIN_UART1_RX, PIN_FN1);
						uart_vectors[1] = vect;
						status = 0;									
					}else
					{
						res_port0_release_(__FILE__, PIN_UART1_TX|PIN_UART1_RX);
					}
				}				
			}
			
		}
		portEXIT_CRITICAL();
	}
	else
	{
		status = -ENOBUFS;
	}
	if(status< 0)
	{
		return status;
	}

	/* Setup the baud rate*/
	
	serial_baud_get_vals(DEFAULT_BAUD, &dll_val, &dlm_val, 
							&mul_val, &divadd_val);
					
	if(0==port)
	{
		lcr = &(UART0_LCR);
		dll = &(UART0_DLL);
		dlm = &(UART0_DLM);
		fdr = &(UART0_FDR);
		fcr = &(UART0_FCR);
		ier = &(UART0_IER);
	}else
	{
		lcr = &(UART1_LCR);
		dll = &(UART1_DLL);
		dlm = &(UART1_DLM);
		fdr = &(UART1_FDR);
		fcr = &(UART1_FCR);
		ier = &(UART1_IER);					
	}
				
	/* Set the DLAB bit so we can access the divisor. */
	*lcr |= SER_DLAB;
	/* Setup the divisor. */
	*dll = dll_val;
	*dlm = dlm_val;
			
	*fdr = ((mul_val<<4)|divadd_val);
	/* Turn on the FIFO's and clear the buffers. */
	*fcr = ( SER_FIFO_ON | SER_CLEAR_FIFO );

	/* Setup transmission format. */
	*lcr = SER_NO_PARITY | SER_1STOP_BIT | SER_8BIT_CHARS;
	/* Setup the VIC for the UART. */
	if(0==port)
	{
		VIC_VECT_ENABLE(vect, VIC_CH_UART0, uart0_isr);
		VIC_CH_IRQ_ENABLE(VIC_CH_UART0);
	}else
	{
		VIC_VECT_ENABLE(vect, VIC_CH_UART1, uart1_isr);
		VIC_CH_IRQ_ENABLE(VIC_CH_UART1);
	}
				
	/* Enable UART0 interrupts. */
	*ier |= SER_ENABLE_INTERRUPTS;
	
	uart_state[port] = UART_OPEN;
	
	return status;
}

int serial_baud_get_vals(int32 baud, uint8 *dll, uint8 *dlm, uint8 *mul, uint8 *divadd)
{
	int i;
	*dll = *dlm = *mul = *divadd=0;
	for(i=0;i<NUM_BAUDS;i++)
	{
		if(baud == bauds[i].baud)
		{
			*dll = bauds[i].dll;
			*dlm = bauds[i].dlm;
			*mul = bauds[i].mul;
			*divadd = bauds[i].divadd;
			return 0;
		}		
	}
	return -1;
}

int32 serial_read(int32 port, int8 *buf, int32 nread)
{
	int32 n=0;

	if(port >= BOARD_CONFIG_NUART)
	{
		return -EINVAL;
	}
	
	if(!uart_state[port]&UART_OPEN)
	{
		return -EBADFD;
	}

	if(uart_state[port]&UART_BUSY)
	{
		return -EBUSY;
	}
	
	while(n < nread)
	{
		if(!xQueueReceive( rx_q[port], (signed portCHAR *) buf, portMAX_DELAY ))
		{
			break;
		}
		n++;
		buf++;
	}
	return n;
}

int32 serial_timed_read(int32 port, int8 *buf, int32 nread, int32 msecs)
{
	int32 n=0;
	int32 nticks = msecs/portTICK_RATE_MS;

	if(port >= BOARD_CONFIG_NUART)
	{
		return -EINVAL;
	}
	
	if(!uart_state[port]&UART_OPEN)
	{
		return -EBADFD;
	}

	if(uart_state[port]&UART_BUSY)
	{
		return -EBUSY;
	}
	
	while(n < nread)
	{
		if(!xQueueReceive( rx_q[port], (signed portCHAR *) buf,  nticks))
		{
			break;
		}
		n++;
		buf++;
	}
	return n;
}


int32 serial_write(int32  port, const int8 *buf, int32 nwrite)
{
	int32 n=0;
	int ok;
	int8 c;
	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	if(port >= BOARD_CONFIG_NUART)
	{
		return -EINVAL;
	}
	
	if(!uart_state[port]&UART_OPEN)
	{
		return -EBADFD;
	}

	if(uart_state[port]&UART_BUSY)
	{
		return -EBUSY;
	}

	while(n < nwrite)
	{
		ok=0;
		portENTER_CRITICAL();
		{
			/* Is there space to write directly to the UART? */
			if(thre_empty[port])
			{
				/* We wrote the character directly to the UART, so was 
				successful. */
				thre_empty[port] = 0;
				if(0==port)
				{
					UART0_THR = *buf;
				}else
				{
					UART1_THR = *buf;
				}
				ok=1;
			}
			else 
			{
				/* We cannot write directly to the UART, so queue the character.
				Block for a maximum of xBlockTime if there is no space in the
				queue. */
				ok = xQueueSend( tx_q[port], buf, portMAX_DELAY );

				/* Depending on queue sizing and task prioritisation:  While we 
				were blocked waiting to post interrupts were not disabled.  It is 
				possible that the serial ISR has emptied the Tx queue, in which
				case we need to start the Tx off again. */
				if( (thre_empty[port]) && ok )
				{
					xQueueReceive( tx_q[port], &c, 0 );
					thre_empty[port] = 0;
					if(0==port)
					{
						UART0_THR = c;
					}else
					{
						UART1_THR = c;
					}
				}
			}
		}
		portEXIT_CRITICAL();
		
		if(!ok)
		{
			break;
		}
		n++;
		buf++;
	}
	return n;
}

int32 serial_close(int32 port)
{
	uint32 n;
	uint8 c;
	
	if(port >= BOARD_CONFIG_NUART)
	{
		return -EINVAL;
	}
	
	if(!uart_state[port]&UART_OPEN)
	{
		return -EBADFD;
	}

	if(uart_state[port]&UART_BUSY)
	{
		return -EBUSY;
	}
		
	portENTER_CRITICAL();
	{
		if(0==port)
		{
			UART0_IER &= ~SER_ENABLE_INTERRUPTS;
			VIC_CH_IRQ_DISABLE(VIC_CH_UART0);			
			POWER_OFF(PCONP_UART0);
			res_port0_release_(__FILE__, PIN_UART0_TX|PIN_UART0_RX);			
		}else
		{
			UART1_IER &= ~SER_ENABLE_INTERRUPTS;
			VIC_CH_IRQ_DISABLE(VIC_CH_UART1);
			POWER_OFF(PCONP_UART1);
			res_port0_release_(__FILE__, PIN_UART1_TX|PIN_UART1_RX);
		}		
		VIC_VECT_DISABLE(uart_vectors[port]);	
		res_vic_vect_release_(__FILE__, uart_vectors[port]);
		uart_vectors[port]=0UL;		
	}
	portEXIT_CRITICAL();
	
	//flush rx/tx queues so that when the uart is reopened
	//queues are empty
	n = uxQueueMessagesWaiting(rx_q[port]);
	while(n > 0)
	{
		xQueueReceive( rx_q[port], &c, 0UL);;		
		n--;
	}
	n = uxQueueMessagesWaiting(tx_q[port]);
	while(n > 0)
	{
		xQueueReceive( tx_q[port], &c, 0UL);;		
		n--;
	}
	
	thre_empty[port] = 1L;	
	uart_state[port] = UART_FREE;
	
	return 0;
}

void serial_isr_create_queues(int port, int32 queue_len)
{
	/* Create the queues used to hold Rx and Tx characters. */
	rx_q[port] = xQueueCreate(queue_len, ( uint32 ) sizeof( int8 ) );
	tx_q[port] = xQueueCreate( queue_len + 1, ( uint32 ) sizeof( int8 ) );

	/* Initialise the THRE empty flag */
	thre_empty[port] = 1L;

}

static void uart0_isr( void )
{
	/* This ISR can cause a context switch, so the first statement must be a
	call to the portENTER_SWITCHING_ISR() macro.  
	This must be BEFORE any local variable declarations. */
	portENTER_SWITCHING_ISR();
	
/*
 * Static variable work for ISRs because interrupts are not
 * nested
 * We're avoid local variable because portENTER_SWITCHING_ISR()
 * switches the stack frame pointer (R11) from ISR stack 
 * to task system mode stack - but SP (R13) is left unchanged
 * (for good reason). 
 * This confuses the debugger.
 */
	static uint8 isr_data;
	static int32 task_woken_by_isr;
	
	task_woken_by_isr=0;
	/* What caused the interrupt? */
	switch( UART0_IIR & SER_INTERRUPT_SOURCE_MASK )
	{
		case SER_SOURCE_ERROR:/* Not handling this, but clear the interrupt. */
			isr_data = UART0_LSR;
			break;

		case SER_SOURCE_THRE:	
			/* The THRE is empty.  If there is another
				character in the Tx queue, send it now. */
			if( xQueueReceiveFromISR( tx_q[SER_UART0], &isr_data, &task_woken_by_isr ) )
			{
				UART0_THR = isr_data;
			}
			else
			{
				/* There are no further characters 
					queued to send so we can indicate 
						that the THRE is available. */
				thre_empty[SER_UART0] = 1;
			}
			break;

		case SER_SOURCE_RX_TIMEOUT:
		case SER_SOURCE_RX:	
			/* A character was received.  Place it in 
				the queue of received characters. */
			isr_data = UART0_RBR;
			if( xQueueSendFromISR( rx_q[SER_UART0], &isr_data, 0) ) 
			{
				task_woken_by_isr = 1;
			}
			break;

		default:	
			/* There is nothing to do, leave the ISR. */
			break;
	}

	/* Clear the ISR in the VIC. 
	 * Note that until we exit ISR IRQs are not enabled by
	 * the processor*/
	VIC_CLEAR_IRQ();

	/* Exit the ISR.  If a task was woken by either a character being received
	or transmitted then a context switch will occur. */
	portEXIT_SWITCHING_ISR( task_woken_by_isr );
}

static void uart1_isr( void )
{
	/* This ISR can cause a context switch, so the first statement must be a
	call to the portENTER_SWITCHING_ISR() macro.  
	This must be BEFORE any	local variable declarations. */
	portENTER_SWITCHING_ISR();
	
/*
 * Static variable work for ISRs because interrupts are not
 * nested
 * We're avoid local variable because portENTER_SWITCHING_ISR()
 * switches the stack frame pointer (R11) from ISR stack 
 * to task system mode stack - but SP (R13) is left unchanged
 * (for good reason). 
 * This confuses the debugger.
 */
	static uint8 isr_data;
	static int32 task_woken_by_isr;
	
	task_woken_by_isr=0;

	/* What caused the interrupt? */
	switch( UART1_IIR & SER_INTERRUPT_SOURCE_MASK )
	{
		case SER_SOURCE_ERROR:/* Not handling this, but clear the interrupt. */
			isr_data = UART1_LSR;
			break;

		case SER_SOURCE_THRE:	
			/* The THRE is empty.  If there is another
				character in the Tx queue, send it now. */
			if( xQueueReceiveFromISR( tx_q[SER_UART1], &isr_data, &task_woken_by_isr ) )
			{
				UART1_THR = isr_data;
			}
			else
			{
				/* There are no further characters 
					queued to send so we can indicate 
						that the THRE is available. */
				thre_empty[SER_UART1] = 1;
			}
			break;

		case SER_SOURCE_RX_TIMEOUT:
		case SER_SOURCE_RX:	
			/* A character was received.  Place it in 
				the queue of received characters. */
			isr_data = UART1_RBR;
			if( xQueueSendFromISR( rx_q[SER_UART1], &isr_data, 0) ) 
			{
				task_woken_by_isr = 1;
			}
			break;

		default:	
			/* There is nothing to do, leave the ISR. */
			break;
	}

	/* Clear the ISR in the VIC. */
	VIC_CLEAR_IRQ();

	/* Exit the ISR.  If a task was woken by either a character being received
	or transmitted then a context switch will occur. */
	portEXIT_SWITCHING_ISR( task_woken_by_isr );
}
