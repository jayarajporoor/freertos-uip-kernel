
#include <lpc21xx.h>
#include <FreeRTOS.h>                     
#include <task.h>
#include <queue.h>
#include <errno.h>
#include <board_config.h>
#include <res.h>
#include <typedefs.h>
#include <spi.h>

#define SPI_FREE 0
#define SPI_OPEN 1
#define SPI_BUSY 2

#define SPI_MASTER_MODE		(1UL << 5)
#define SPI_LSB_FIRST		(1UL << 6)
#define SPI_INT_ENABLE		(1UL << 7)

#define SPI_CLK_DIV			100

static int8 state;
static wait_struct_t *curr_ws;
static void spi_isr( void ) __attribute__ ((naked));

static int8 curr_op;
static int8 is_cmd;
static int32 count_remaining;
static uint8 *curr_buf;
static int8 vector;

int32 spi_open()
{
	int32 status=1;
	
	
	if(SPI_OPEN == state)
	{
		 return -EBUSY;
	}

	taskENTER_CRITICAL();
	{
		if(SPI_BUSY == state)
		{
			status = -EBUSY;
		}else
		{
			state = SPI_BUSY;
		}
	}
	taskEXIT_CRITICAL();
	
	if(status < 0)
	{
		return status;
	}

	taskENTER_CRITICAL();
	{	
		if(res_port0_acquire_(__FILE__
				, BIT(PIN_SPI0_SCK)|
				  BIT(PIN_SPI0_MISO)|
				  BIT(PIN_SPI0_MOSI)|
				  BIT(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN)
				  ) )
		{
		        if(!(vector = res_vic_vect_acquire_any_(__FILE__)))
        		{
            		res_port0_release_( __FILE__,
            							BIT(PIN_SPI0_SCK)|
				  						BIT(PIN_SPI0_MISO)|
				  						BIT(PIN_SPI0_MOSI)|
				  						BIT(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN) 
            							);
            		status = -EBUSY;
        		}
		}else
		{
			status = -EBUSY;
		}
	}
	taskEXIT_CRITICAL();
	
	if(status < 0)
	{
		return status;
	}
		
	POWER_ON(PCONP_SPI0);
	    	
	PINSEL(PIN_SPI0_SCK, PIN_FN1);
	PINSEL(PIN_SPI0_MISO, PIN_FN1);
	PINSEL(PIN_SPI0_MOSI, PIN_FN1); 	
  	PINSEL(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN, PIN_FN0);

  	PORT0_DIR_OUTPUT(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN);
  	PORT0_SET(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN);

	VIC_VECT_ENABLE(vector, VIC_CH_SPI0,spi_isr);
	VIC_CH_IRQ_ENABLE(VIC_CH_SPI0);

  	// setup SPI interface :
  	// Master mode, MSB first
 	   
 	SPI_SPCR = SPI_MASTER_MODE|SPI_INT_ENABLE;
  
  	SPI_SPSR = 0UL;//clear status reg

  	SPI_SPCCR = SPI_CLK_DIV;
  	
	state = SPI_OPEN;
	
	return status;
	
}

static int32 spi_op_status;

int32 spi_op(uint8 *buf, int32 len, int8 op, uint8 cmd)
{

	if(SPI_FREE == state)
	{
		return -EBADFD;
	}

	if(len <= 0)
	{
		return -EINVAL;
	}
	
	state = SPI_BUSY;
		
	curr_buf = buf;
	count_remaining = len;
	curr_op = op;
	curr_ws = CURR_WAIT_STRUCT();
	
	portENTER_CRITICAL();
	{ 	 	  	
  		PORT0_CLR(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN);
	  	
		is_cmd=0;
	
  		if(curr_op&SPI_OP_HAS_CMD)
  		{
  			count_remaining++;
  			is_cmd=1;
  			SPI_SPDR=cmd;
  		}else
  		{
  			if(curr_op&SPI_OP_WRITE)
  			{
  				SPI_SPDR=*curr_buf;
  			}else
  			{
  				SPI_SPDR=0;
  			}
  		}
	}
	portEXIT_CRITICAL();
  			 	
  	TASK_WAIT(curr_ws, &spi_op_status);
  	
	state = SPI_OPEN;
	
	return (len - count_remaining); 
}


static void spi_isr(void)
{
	portENTER_SWITCHING_ISR();
	
	SPI_SPINT = 1;
	
	spi_op_status = SPI_SPSR;
	
	int32 task_woken=0;

	count_remaining--;
		
	if(0 == count_remaining)
	{
		PORT0_SET(BOARD_CONFIG_PORT0_SPI0_SSEL_PIN);

		if(curr_op&SPI_OP_READ)
		{
			*curr_buf = SPI_SPDR;
		}
		spi_op_status=0;
		TASK_NOTIFY_FROM_ISR(curr_ws,&spi_op_status);		
		task_woken=1;
	}else
	{
		if(!is_cmd)
		{
			if(curr_op&SPI_OP_READ)
			{
				*curr_buf = SPI_SPDR;
			}
		
			curr_buf++;
		}else
		{
			is_cmd=0;
		}
		
		if(curr_op&SPI_OP_WRITE)
		{
			SPI_SPDR = *curr_buf;
		}else
		{
			SPI_SPDR = 0;
		}
	}
	VIC_CLEAR_IRQ();
	portEXIT_SWITCHING_ISR(task_woken);
}
