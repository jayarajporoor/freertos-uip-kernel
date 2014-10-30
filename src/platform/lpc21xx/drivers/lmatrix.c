#include <typedefs.h>
#include <FreeRTOS.h>
#include <lpc21xx.h>
#include <lmatrix.h>
#include <ssp.h>
#include <errno.h>
#include <sys.h>

static void lmatrix_write_reg(uint16 config_val);

static int ssp_is_open = 0;
 
int32 lmatrix_open(uint8 intensity)
{
	if(!ssp_is_open)
	{
		ssp_open();
		ssp_is_open=1;
		lmatrix_test();//do this only first time
	}

	// Write to the configuration register for normal operation with reqd functionality
	lmatrix_write_reg(0x0489);
			
	// Write to the configuration register for max intensity
	lmatrix_write_reg(0x0100 | intensity);

	lmatrix_write_reg(0x0200 | intensity);

	return 0;
}

void lmatrix_test()
{
	lmatrix_write_reg(0x0701);
	sys_sleep(1);
	lmatrix_write_reg(0x0700);	
}

static void lmatrix_write_reg(uint16 config_val)
{
    uint16 m;
    uint16 regbuf[LMATRIX_NUM_OF_BOARDS]; 
	
    for (m=0;m<LMATRIX_NUM_OF_BOARDS;m++) 
    {
    	regbuf[m] = config_val;
    }
    ssp_write_word(regbuf, LMATRIX_NUM_OF_BOARDS); 	
}
     
#define LMATRIX_MODULE_ADDR_MAX 0x63

int32 lmatrix_write(char *buf, int len)
{
	uint16 val,addr;
    uint16 i,j,k,index;
    uint16 display[LMATRIX_NUM_OF_LEDS];
    uint16 write_disp[LMATRIX_NUM_OF_BOARDS];
    uint8 num_of_frames;


    for(i=0;i<LMATRIX_NUM_OF_LEDS;i++)
    {
    	j = i%4; 
		val = (i < len) ? *(buf + i) : ' ';
      	if (val < ' ')
      	{
       	 	val = ' ';
      	} 
      	index = LMATRIX_NUM_OF_LEDS - (i+1);
      	addr = LMATRIX_MODULE_ADDR_MAX - j; 
      	val = (addr << 8) + val;
      	display[index] = val;
   	}

	for (k=1;k<5;k++)
    {
    	for (i=0;i<LMATRIX_NUM_OF_BOARDS;i++)
        {
        	j=LMATRIX_NUM_OF_LEDS-((4*i) + k);  
         	write_disp[i] = display[j]; 
      	}
      	num_of_frames = LMATRIX_NUM_OF_BOARDS; 
      	ssp_write_word(write_disp, num_of_frames);       
	}
	return len;     
}

int32 lmatrix_close()
{
	return -EINVAL;
}
