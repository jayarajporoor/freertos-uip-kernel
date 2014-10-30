
#include <lpc21xx.h>
#include <FreeRTOS.h>

#include <stdio.h>
#include <typedefs.h>
#include <board_config.h>
#include <enc28j60.h>
#include <spi.h>
#include <uip.h>
#include <uipopt.h>
#include <klog.h>
#include <sys.h>

//#define SPI_SELECT_ETHERNET SPI_SLAVE_SELECT
//#define SPI_UNSELECT_ETHERNET SPI_SLAVE_UNSELECT

//#define ETHERNET_DEBUG                  /* Define if trying to debug the driver */


/* RX & TX memory organization */
#define MAXFRAMESIZE 1518
#define RAMSIZE	0x2000		
#define TXSTART	0x0000
#define TXEND   0x0fff
#define RXSTART 0x1000
#define RXEND   0x1FFF
#define RXSIZE  (RXSTOP - RXSTART + 1)

/* Chose one or the other */
//#define HALF_DUPLEX
#define FULL_DUPLEX

#ifdef HALF_DUPLEX
#ifdef FULL_DUPLEX
DUPLEX==> CHOOSE ONE OR THE OTHER
#endif
#endif

/* Local Prototypes */
static uint8 encReadEthReg (uint8 address);
static uint8 encReadMacReg (uint8 address);
static void encWriteReg (uint8 address, uint8 data);
static void encWriteReg16(uint8 address, uint16 data);
static void encBFSReg(uint8 address, uint8 data);
static void encBFCReg(uint8 address, uint8 data);
static void SendSystemCommand (void);
static void encBankSelect(uint8 bank);
static void encWritePHYReg(uint8 address, uint16 data);
static uint16 encReadPHYReg(uint8 address);
static void encMACwrite(uint8 data);
static void encMACwriteBulk(uint8 *buffer, uint16 length);
static uint8 encMACread(void);
static void encMACreadBulk(uint8 *buffer, uint16 length);

/* Global variable */
uint16 ethRxPointer;                         // Point to the next available frame

extern volatile uint16 uip_slen;
extern uint8 uip_buf[];
          
extern struct uip_eth_addr uip_ethaddr;

STATUS enc28j60_open()
{
	int32 status;
	uint8 value;
  	uint16 value16;

  	if((status=spi_open()) < 0)
  	{
  		klog_error("%s:%d> SPI open failed. Error code: %l\n", 
  			__FILE__, __LINE__, status);
  		return status;
  	}
  
	/* Make sure the part is out of reset.
  	Monitor CLKRDY bit until set. */
	do 
	{
		value = encReadEthReg(ESTAT);
	} while (!(value & ESTAT_CLKRDY));

	/* Send a Soft Reset to the chip */
  	SendSystemCommand();
  	
  	sys_msleep(2);	

    /* Start in Bank 0 to set the pointers */
	encBankSelect(BANK0);

    /* Initialize the local RX pointer and configure the base address */
	ethRxPointer = RXSTART;
	encWriteReg16(ERXSTL, RXSTART);
  	/* Read pointer is also at the base address for now */
  	encWriteReg16(ERXRDPTL, RXSTART);
  	/* Configure the H/W with the end of memory location */
  	encWriteReg16(ERXNDL, RXEND);

  	/* Configure the H/W with the TX base address */
	encWriteReg16(ETXSTL, TXSTART);

  	/* Go to Bank 1 */
  	encBankSelect(BANK1);
	encWriteReg16(ERXFCON, ERXFCON_UCEN | ERXFCON_CRCEN | ERXFCON_BCEN);

   	/* Go to Bank 2 */
  	encBankSelect(BANK2);
 	/* Remove all reset conditions */
  	encWriteReg(MACON2, 0);

	#ifdef HALF_DUPLEX
  	/* Enable RX */
  	encWriteReg(MACON1, MACON1_MARXEN);
	/* Automatic padding, CRC generation */
  	encWriteReg(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN);
  	#else
  	/* Enable TX pause, RX pause and RX */
  	encWriteReg(MACON1, MACON1_TXPAUS | MACON1_RXPAUS | MACON1_MARXEN);
  	/* Automatic padding, CRC generation, Full-Duplex */
  	encWriteReg(MACON3, MACON3_PADCFG0 | MACON3_TXCRCEN | MACON3_FULDPX);
	#endif

 	/* Set the maximum packet size which the controller will accept */
	encWriteReg16(MAMXFLL, MAXFRAMESIZE);
  
  	/* Set inter-packet gap to 9.6us */
	encWriteReg16(MAIPGL, 0x0C12);

  	/* Set back to back inter-packet gap */
	encWriteReg(MABBIPG, 0x12);

  	/* Go to Bank 3 */
  	encBankSelect(BANK3);
  	encWriteReg(MAADR0, uip_ethaddr.addr[5]);
  	encWriteReg(MAADR1, uip_ethaddr.addr[4]);
  	encWriteReg(MAADR2, uip_ethaddr.addr[3]);
  	encWriteReg(MAADR3, uip_ethaddr.addr[2]);
  	encWriteReg(MAADR4, uip_ethaddr.addr[1]);
  	encWriteReg(MAADR5, uip_ethaddr.addr[0]);

  	#ifdef HALF_DUPLEX
  	encWritePHYReg(PHCON1, 0);           
  	#else
  	encWritePHYReg(PHCON1, PHCON1_PDPXMD);
  	#endif           
  
  	/* Disable half duplex loopback in PHY. */
  	encWritePHYReg(PHCON2, PHCON2_HDLDIS);           

	/* We use the default setting of the LED */

  	/* Read a PHY register (so the compiler does not complain !) */
  	value16 = encReadPHYReg(PHID1);
	/* Enable packet reception */
  	encBFSReg(ECON1, ECON1_RXEN);
  	return 0;
}

void enc28j60_write()
{
	uint16 length;
  	uint16 value;

  	/* Save length for later */
  	length = uip_len;
  	/* Clear the end TX flag */
  	encBFCReg(EIR, EIR_TXIF);

  	/* Go to Bank 0 */
  	encBankSelect(BANK0);
  	/* Configure the H/W with the TX base address */
	encWriteReg16(ETXSTL, TXSTART);
  	/* Configure the H/W with the TX Start and End addresses */
	encWriteReg16(EWRPTL, TXSTART);
  	/* encWriteReg16(ETXNDL, TXSTART + uip_len + 1); */
  	/* Write the per packet option byte
  	we use the default values set in chip earlier */
	encMACwrite(0x00);

  	/* Send 40+14=54 bytes of header */
  	encMACwriteBulk(&uip_buf[0], 54);

  	#ifdef ETHERNETDEBUG
  	for (value=0; value< 54; value++) {
    	printf ("%x ",uip_buf[value]);
  	}
  	printf("\n");
  	#endif

  	if (uip_len > 54) 
  	{
    	/* Send the rest of the packet, the application data */
    	uip_len -= 54;
    	encMACwriteBulk((uint8*)uip_appdata, uip_len);
  	}
  	/* Configure the H/W with the TX Start and End addresses */
  	encWriteReg16(ETXNDL, (TXSTART + length));
  	value = encReadEthReg(ETXNDL);
  	value += (encReadEthReg(ETXNDH)<<8);
	/* Enable packet Transmission */
  	encBFSReg(ECON1, ECON1_TXRTS);
}

uint16 enc28j60_read()
{
	uint16 len, u;

	/* Check if at least one packet has been received and is waiting */
  	if ( (encReadEthReg(EIR) & EIR_PKTIF) == 0) 
  	{
    	return 0;
  	}
  	/* Set read Pointer */
  	encBankSelect(BANK0);
  	encWriteReg16(ERDPTL, ethRxPointer);
	
  	ethRxPointer = encMACread();
  	ethRxPointer += encMACread() << 8;

  	/* Error detection... */
  	if (ethRxPointer > RXEND) 
  	{
    	enc28j60_open();
    	return 0;
  	}

  	len = encMACread();
  	len += encMACread() << 8;

  	/* Read 2 more statuses that we ignore */
  	encMACread();
  	encMACread();

  	/* If the frame is too big to handle, throw it away */
  	if (len > UIP_BUFSIZE) 
  	{
    	for (u=0; u<len; u++) 
    	{
      		encMACread(); /* Read a byte and throw it away */
    	} 
    	return 0;
 	}
  
  	/* Read the whole frame */
  	encMACreadBulk(&uip_buf[0], len);

  	/* Clean up for next packet 
  	Set the read pointer to the start of RX packet to free up
  	space used by the current frame */
  	/* Go to Bank 0 */
  	encBankSelect(BANK0);
  	encWriteReg16(ERXRDPTL, ethRxPointer);

  	/* Decrement EPKTCNT */
  	encBFSReg(ECON2, ECON2_PKTDEC);

  	/* Return the length - the 4 bytes of CRC */
  	return (len-4);  
}


/******************************************************************************
 * Function:        void SendSystemCommand(void)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    All values are lost
 *
 * Overview:        Send the SC (Reset) command to the device 
 * Note:            None
 *****************************************************************************/
/*static void SendSystemCommand(void)
{
  // Select the chip and send the System Command (Reset) 
	SPI_SELECT_ETHERNET;
	spi_put(SC);	
	SPI_UNSELECT_ETHERNET;
}*/

static void SendSystemCommand(void)
{	
  // Select the chip and send the System Command (Reset)
  	uint8 c = SC;
  	spi_write(&c, 1);
}

/******************************************************************************
 * Function:        uint8 encReadEthReg(uint8 address)
 * PreCondition:    None
 * Input:           address to read
 * Output:          value read from the register
 * Side Effects:    None
 *
 * Overview:        read the value at the address over the SPI bus
 * Note:            None
 *****************************************************************************/
/*static uint8 encReadEthReg(uint8 address)
{
	uint8 value;

 	SPI_SELECT_ETHERNET;
	spi_put(RCR | address);	
    value = spi_put(0x00);							
	SPI_UNSELECT_ETHERNET;
	return value;
}*/

static uint8 encReadEthReg(uint8 address)
{
	uint8 v;	
	spi_cmd_read(&v, 1, RCR|address);
	return v;
}

/******************************************************************************
 * Function:        uint8 encReadMacReg(uint8 address)
 * PreCondition:    None
 * Input:           address to read
 * Output:          value read from the register
 * Side Effects:    None
 *
 * Overview:        read the value at the address over the SPI bus
 * Note:            None
 *****************************************************************************/
/*static uint8 encReadMacReg(uint8 address)
{
	uint8 value;

	SPI_SELECT_ETHERNET;
	spi_put(RCR | address);	
	spi_put(0x00);             // Send a dummy byte 
	value = spi_put(0x00);							
	SPI_UNSELECT_ETHERNET;
	return value;
}*/

static uint8 encReadMacReg(uint8 address)
{
	uint8 buf[2];
	
	spi_cmd_read(buf, 2, RCR|address);
	
	return buf[1];
}

/******************************************************************************
 * Function:        void encWriteReg(uint8 address, uint8 data)
 * PreCondition:    Bank must point ot the right bank
 * Input:           address and data to write
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Send value at the address over the SPI bus
 * Note:            None
 *****************************************************************************/
/*static void encWriteReg(uint8 address, uint8 data)
{
	SPI_SELECT_ETHERNET;
	spi_put(WCR | address);
	spi_put(data);
	SPI_UNSELECT_ETHERNET;
}*/

static void encWriteReg(uint8 address, uint8 data)
{
	spi_cmd_write(&data, 1, WCR|address);	
}

/******************************************************************************
 * Function:        void encBFSReg(uint8 address, uint8 data)
 * PreCondition:    Bank must point ot the right bank
 * Input:           address and data field
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Set the data field in the address register
 * Note:            None
 *****************************************************************************/
/*static void encBFSReg(uint8 address, uint8 data)
{
	SPI_SELECT_ETHERNET;
	spi_put(BFS | address);
	spi_put(data);
	SPI_UNSELECT_ETHERNET;
}*/

static void encBFSReg(uint8 address, uint8 data)
{
	spi_cmd_write(&data, 1, BFS|address);	
}


/******************************************************************************
 * Function:        void encBFCReg(uint8 address, uint8 data)
 * PreCondition:    Bank must point ot the right bank
 * Input:           address and data fiels
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Clear the data field in the address register
 * Note:            None
 *****************************************************************************/
/*static void encBFCReg(uint8 address, uint8 data)
{
	SPI_SELECT_ETHERNET;
	spi_put(BFC | address);
	spi_put(data);
	SPI_UNSELECT_ETHERNET;
}*/

static void encBFCReg(uint8 address, uint8 data)
{
	spi_cmd_write(&data, 1, BFC|address);		
}

/******************************************************************************
 * Function:        void encWriteReg16(uint8 address, uint16 data)
 * PreCondition:    Bank must point ot the right bank
 * Input:           address and data to write
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Send value at the address over the SPI bus
 * Note:            None
 *****************************************************************************/
static void encWriteReg16(uint8 address, uint16 data)
{
	encWriteReg(address,   (uint8) (data & 0xff));
 	encWriteReg(address+1, (uint8) (data >> 8));
}

/******************************************************************************
 * Function:        void encWritePHYReg(uint8 address, uint16 data)
 * PreCondition:    None
 * Input:           address and data to write
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Send value at the address over the SPI bus
 * Note:            None
 *****************************************************************************/
static void encWritePHYReg(uint8 address, uint16 data)
{
	/* Switch to Bank 2 */
	encBankSelect(BANK2);
  /* Write the PHY address to write */
	encWriteReg(MIREGADR, address);
	
	/* Write the data */
  encWriteReg16(MIWRL, data);

	/* Switch to Bank 3 */
	encBankSelect(BANK3);
	while( (encReadMacReg(MISTAT) & MISTAT_BUSY) );
}

/******************************************************************************
 * Function:        uint16 encReadPHYReg(uint8 address)
 * PreCondition:    None
 * Input:           address to read from
 * Output:          data from a PHY register
 * Side Effects:    None
 *
 * Overview:        Read a value from a PHY register
 * Note:            None
 *****************************************************************************/
static uint16 encReadPHYReg(uint8 address)
{
  uint16 value;

	/* Switch to Bank 2 */
	encBankSelect(BANK2);
  /* Write the PHY address to read */
	encWriteReg(MIREGADR, address);
  /* Send the read command */
  encWriteReg(MICMD, MICMD_MIIRD);

	/* Wait end of command */
	/* Switch to Bank 3 */
	encBankSelect(BANK3);
  while( (( encReadMacReg(MISTAT)) & MISTAT_BUSY) );

  /* Clear the read command */
	/* Switch to Bank 2 */
	encBankSelect(BANK2);
  encWriteReg(MICMD, 0);

  value = encReadMacReg(MIRDH) << 8;
  value += encReadMacReg(MIRDL);
  return value;
}

/******************************************************************************
 * Function:        void encBankSelect(uint8 bank)
 * PreCondition:    None
 * Input:           bank (0, 1, 2 or 3)
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Set the bank select 
 * Note:            None
 *****************************************************************************/
static void  encBankSelect(uint8 bank)
{
	/* Start by clearing the bank select bits */
	encBFCReg(ECON1, ECON1_BSEL1 | ECON1_BSEL0);
	encBFSReg(ECON1, bank);
}

/******************************************************************************
 * Function:        void encMACwrite(uint8 data)
 * PreCondition:    EWRPT must point to the location to be written to
 * Input:           data to write
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Write a byte in the MAC memory with the intention
 *                  of sending a frame later 
 * Note:            None
 *****************************************************************************/
/*static void encMACwrite(uint8 data)
{
	SPI_SELECT_ETHERNET;
	spi_put(WBM);              	
	spi_put(data);
	SPI_UNSELECT_ETHERNET;
}*/

static void encMACwrite(uint8 data)
{
	spi_cmd_write(&data, 1, WBM); 	
}
/******************************************************************************
 * Function:        void encMACwriteBulk(uint8 *buffer, uint16 length)
 * PreCondition:    EWRPT must point to the location to be written to
 * Input:           data to write and length of data
 * Output:          None
 * Side Effects:    None
 *
 * Overview:        Multi Write in the MAC memory with the intention
 *                  of sending a frame later 
 * Note:            None
 *****************************************************************************/
/*static void encMACwriteBulk(uint8 *buffer, uint16 length)
{
	spi_cmd_write(buffer, length, WBM);	
	
	SPI_SELECT_ETHERNET;
	spi_put(WBM);              	
	while (length--)
	{
		spi_put(*buffer++);
	}
	SPI_UNSELECT_ETHERNET;
}*/

static void encMACwriteBulk(uint8 *buffer, uint16 length)
{
	spi_cmd_write(buffer, length, WBM);		
}

/******************************************************************************
 * Function:        uint8 data encMACread(void)
 * PreCondition:    ERDPT must point to the location to read from
 * Input:           None
 * Output:          Data read
 * Side Effects:    None
 *
 * Overview:        Read a byte from the MAC memory
 * Note:            None
 *****************************************************************************/
/*static uint8 encMACread(void)
{
	uint8 value;

	SPI_SELECT_ETHERNET;
	spi_put(RBM);
	value = spi_put(0x00);							
	SPI_UNSELECT_ETHERNET;
	return value;
}*/

static uint8 encMACread(void)
{
	uint8 v;
  
	spi_cmd_read(&v, 1, RBM);	
	return v;
}

/******************************************************************************
 * Function:        encMACreadBulk(uint8 *buffer, uint16 length)
 * PreCondition:    ERDPT must point to the location to read from
 * Input:           Buffer to put the data and length of data to read
 * Output:          Data read in the buffer
 * Side Effects:    None
 *
 * Overview:        Read multiple bytes from the MAC memory
 * Note:            None
 *****************************************************************************/

/*static void encMACreadBulk(uint8 *buffer, uint16 length)
{
 	SPI_SELECT_ETHERNET;
  	spi_put(RBM);
  	while (length--)
  	{
   		*buffer++ = spi_put(0x00);
 	}
	SPI_UNSELECT_ETHERNET;
}*/

static void encMACreadBulk(uint8 *buffer, uint16 length)
{
	spi_cmd_read(buffer, length, RBM);	
}
