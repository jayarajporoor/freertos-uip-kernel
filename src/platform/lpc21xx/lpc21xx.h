#ifndef lpc21xx_h__
#define lpc21xx_h__

#include <typedefs.h>

#define REG8  (volatile uint8*)
#define REG16 (volatile uint16*)
#define REG32 (volatile uint32*)


/*##############################################################################
## MISC
##############################################################################*/

        /* Constants for data to put in IRQ/FIQ Exception Vectors */
#define VECTDATA_IRQ  0xE51FFFF0  /* LDR PC,[PC,#-0xFF0] */
#define VECTDATA_FIQ  /* __TODO */


/*##############################################################################
## VECTORED INTERRUPT CONTROLLER
##############################################################################*/

#define VICIRQStatus    (*(REG32 (0xFFFFF000)))
#define VICFIQStatus    (*(REG32 (0xFFFFF004)))
#define VICRawIntr      (*(REG32 (0xFFFFF008)))
#define VICIntSelect    (*(REG32 (0xFFFFF00C)))
#define VICIntEnable    (*(REG32 (0xFFFFF010)))
#define VICIntEnClear   (*(REG32 (0xFFFFF014)))
#define VICSoftInt      (*(REG32 (0xFFFFF018)))
#define VICSoftIntClear (*(REG32 (0xFFFFF01C)))
#define VICProtection   (*(REG32 (0xFFFFF020)))
#define VICVectAddr     (*(REG32 (0xFFFFF030)))
#define VICDefVectAddr  (*(REG32 (0xFFFFF034)))

#define VICVectAddrBase 0xFFFFF100
#define VICVectAddr0    (*(REG32 (0xFFFFF100)))
#define VICVectAddr1    (*(REG32 (0xFFFFF104)))
#define VICVectAddr2    (*(REG32 (0xFFFFF108)))
#define VICVectAddr3    (*(REG32 (0xFFFFF10C)))
#define VICVectAddr4    (*(REG32 (0xFFFFF110)))
#define VICVectAddr5    (*(REG32 (0xFFFFF114)))
#define VICVectAddr6    (*(REG32 (0xFFFFF118)))
#define VICVectAddr7    (*(REG32 (0xFFFFF11C)))
#define VICVectAddr8    (*(REG32 (0xFFFFF120)))
#define VICVectAddr9    (*(REG32 (0xFFFFF124)))
#define VICVectAddr10   (*(REG32 (0xFFFFF128)))
#define VICVectAddr11   (*(REG32 (0xFFFFF12C)))
#define VICVectAddr12   (*(REG32 (0xFFFFF130)))
#define VICVectAddr13   (*(REG32 (0xFFFFF134)))
#define VICVectAddr14   (*(REG32 (0xFFFFF138)))
#define VICVectAddr15   (*(REG32 (0xFFFFF13C)))

#define VICVectCntlBase 0xFFFFF200

#define VICVectCntl0    (*(REG32 (0xFFFFF200)))
#define VICVectCntl1    (*(REG32 (0xFFFFF204)))
#define VICVectCntl2    (*(REG32 (0xFFFFF208)))
#define VICVectCntl3    (*(REG32 (0xFFFFF20C)))
#define VICVectCntl4    (*(REG32 (0xFFFFF210)))
#define VICVectCntl5    (*(REG32 (0xFFFFF214)))
#define VICVectCntl6    (*(REG32 (0xFFFFF218)))
#define VICVectCntl7    (*(REG32 (0xFFFFF21C)))
#define VICVectCntl8    (*(REG32 (0xFFFFF220)))
#define VICVectCntl9    (*(REG32 (0xFFFFF224)))
#define VICVectCntl10   (*(REG32 (0xFFFFF228)))
#define VICVectCntl11   (*(REG32 (0xFFFFF22C)))
#define VICVectCntl12   (*(REG32 (0xFFFFF230)))
#define VICVectCntl13   (*(REG32 (0xFFFFF234)))
#define VICVectCntl14   (*(REG32 (0xFFFFF238)))
#define VICVectCntl15   (*(REG32 (0xFFFFF23C)))

#define VICITCR         (*(REG32 (0xFFFFF300)))
#define VICITIP1        (*(REG32 (0xFFFFF304)))
#define VICITIP2        (*(REG32 (0xFFFFF308)))
#define VICITOP1        (*(REG32 (0xFFFFF30C)))
#define VICITOP2        (*(REG32 (0xFFFFF310)))
#define VICPeriphID0    (*(REG32 (0xFFFFFFE0)))
#define VICPeriphID1    (*(REG32 (0xFFFFFFE4)))
#define VICPeriphID2    (*(REG32 (0xFFFFFFE8)))
#define VICPeriphID3    (*(REG32 (0xFFFFFFEC)))

#define VICIntEnClr     VICIntEnClear
#define VICSoftIntClr   VICSoftIntClear


/*##############################################################################
## PCB - Pin Connect Block
##############################################################################*/

#define PCB_PINSEL0     (*(REG32 (0xE002C000)))
#define PCB_PINSEL1     (*(REG32 (0xE002C004)))


/*##############################################################################
## GPIO - General Purpose I/O
##############################################################################*/

#define GPIO_IOPIN      (*(REG32 (0xE0028000))) /* ALTERNATE NAME GPIO = GPIO0 */
#define GPIO_IOSET      (*(REG32 (0xE0028004)))
#define GPIO_IODIR      (*(REG32 (0xE0028008)))
#define GPIO_IOCLR      (*(REG32 (0xE002800C)))

#define GPIO0_IOPIN     (*(REG32 (0xE0028000))) /* ALTERNATE NAME GPIO = GPIO0 */
#define GPIO0_IOSET     (*(REG32 (0xE0028004)))
#define GPIO0_IODIR     (*(REG32 (0xE0028008)))
#define GPIO0_IOCLR     (*(REG32 (0xE002800C)))

#define SCS				(*(REG32 (0xE01FC1A0)))

#define GPIO_FIO0DIR	(*(REG32 (0x3FFFC000)))
#define GPIO_FIO0MASK	(*(REG32 (0x3FFFC010)))
#define GPIO_FIO0PIN	(*(REG32 (0x3FFFC014)))
#define GPIO_FIO0SET	(*(REG32 (0x3FFFC018)))
#define GPIO_FIO0CLR	(*(REG32 (0x3FFFC01C)))

#define GPIO_FIO1DIR	(*(REG32 (0x3FFFC020)))
#define GPIO_FIO1MASK	(*(REG32 (0x3FFFC030)))
#define GPIO_FIO1PIN	(*(REG32 (0x3FFFC034)))
#define GPIO_FIO1SET	(*(REG32 (0x3FFFC038)))
#define GPIO_FIO1CLR	(*(REG32 (0x3FFFC03C)))

/*##############################################################################
## UART0 / UART1
##############################################################################*/

/* ---- UART 0 --------------------------------------------- */
#define UART0_RBR       (*(REG32 (0xE000C000)))
#define UART0_THR       (*(REG32 (0xE000C000)))
#define UART0_IER       (*(REG32 (0xE000C004)))
#define UART0_IIR       (*(REG32 (0xE000C008)))
#define UART0_FCR       (*(REG32 (0xE000C008)))
#define UART0_LCR       (*(REG32 (0xE000C00C)))
#define UART0_LSR       (*(REG32 (0xE000C014)))
#define UART0_SCR       (*(REG32 (0xE000C01C)))
#define UART0_DLL       (*(REG32 (0xE000C000)))
#define UART0_DLM       (*(REG32 (0xE000C004)))
#define UART0_ACR       (*(REG32 (0xE000C020)))
#define UART0_FDR       (*(REG32 (0xE000C028)))
#define UART0_TER       (*(REG32 (0xE000C030)))

/* ---- UART 1 --------------------------------------------- */
#define UART1_RBR       (*(REG32 (0xE0010000)))
#define UART1_THR       (*(REG32 (0xE0010000)))
#define UART1_IER       (*(REG32 (0xE0010004)))
#define UART1_IIR       (*(REG32 (0xE0010008)))
#define UART1_FCR       (*(REG32 (0xE0010008)))
#define UART1_LCR       (*(REG32 (0xE001000C)))
#define UART1_LSR       (*(REG32 (0xE0010014)))
#define UART1_SCR       (*(REG32 (0xE001001C)))
#define UART1_DLL       (*(REG32 (0xE0010000)))
#define UART1_DLM       (*(REG32 (0xE0010004)))
#define UART1_MCR       (*(REG32 (0xE0010010)))
#define UART1_MSR       (*(REG32 (0xE0010018)))
#define UART1_ACR       (*(REG32 (0xE001C020)))
#define UART1_FDR       (*(REG32 (0xE0010028)))
#define UART1_TER       (*(REG32 (0xE001C030)))

/*##############################################################################
## I2C
##############################################################################*/

#define I2C_I2CONSET    (*(REG32 (0xE001C000)))
#define I2C_I2STAT      (*(REG32 (0xE001C004)))
#define I2C_I2DAT       (*(REG32 (0xE001C008)))
#define I2C_I2ADR       (*(REG32 (0xE001C00C)))
#define I2C_I2SCLH      (*(REG32 (0xE001C010)))
#define I2C_I2SCLL      (*(REG32 (0xE001C014)))
#define I2C_I2CONCLR    (*(REG32 (0xE001C018)))


/*##############################################################################
## SPI - Serial Peripheral Interface
##############################################################################*/

#define SPI_SPCR        (*(REG32 (0xE0020000)))
#define SPI_SPSR        (*(REG32 (0xE0020004)))
#define SPI_SPDR        (*(REG32 (0xE0020008)))
#define SPI_SPCCR       (*(REG32 (0xE002000C)))
#define SPI_SPTCR       (*(REG32 (0xE0020010)))
#define SPI_SPTSR       (*(REG32 (0xE0020014)))
#define SPI_SPTOR       (*(REG32 (0xE0020018)))
#define SPI_SPINT       (*(REG32 (0xE002001C)))

/*##############################################################################
## SSP (SPI1) - Serial Peripheral Interface 1
################################################################################*/
#define SSP_CR0             (*(REG32 (0xE0068000)))
#define SSP_CR1             (*(REG32 (0xE0068004)))
#define SSP_DR              (*(REG32 (0xE0068008)))
#define SSP_SR              (*(REG32 (0xE006800C)))
#define SSP_CPSR            (*(REG32 (0xE0068010)))
#define SSP_IMSC            (*(REG32 (0xE0068014)))
#define SSP_RIS             (*(REG32 (0xE0068018)))
#define SSP_MIS             (*(REG32 (0xE006801C)))
#define SSP_ICR             (*(REG32 (0xE0068020)))
/**************************************************/

/*##############################################################################
## Timer 0 and Timer 1
##############################################################################*/

/* ---- Timer 0 -------------------------------------------- */
#define T0_IR           (*(REG32 (0xE0004000)))
#define T0_TCR          (*(REG32 (0xE0004004)))
#define T0_TC           (*(REG32 (0xE0004008)))
#define T0_PR           (*(REG32 (0xE000400C)))
#define T0_PC           (*(REG32 (0xE0004010)))
#define T0_MCR          (*(REG32 (0xE0004014)))
#define T0_MR0          (*(REG32 (0xE0004018)))
#define T0_MR1          (*(REG32 (0xE000401C)))
#define T0_MR2          (*(REG32 (0xE0004020)))
#define T0_MR3          (*(REG32 (0xE0004024)))
#define T0_CCR          (*(REG32 (0xE0004028)))
#define T0_CR0          (*(REG32 (0xE000402C)))
#define T0_CR1          (*(REG32 (0xE0004030)))
#define T0_CR2          (*(REG32 (0xE0004034)))
#define T0_CR3          (*(REG32 (0xE0004038)))
#define T0_EMR          (*(REG32 (0xE000403C)))
#define T0_CTCR         (*(REG32 (0xE0004070)))


/* ---- Timer 1 -------------------------------------------- */
#define T1_IR           (*(REG32 (0xE0008000)))
#define T1_TCR          (*(REG32 (0xE0008004)))
#define T1_TC           (*(REG32 (0xE0008008)))
#define T1_PR           (*(REG32 (0xE000800C)))
#define T1_PC           (*(REG32 (0xE0008010)))
#define T1_MCR          (*(REG32 (0xE0008014)))
#define T1_MR0          (*(REG32 (0xE0008018)))
#define T1_MR1          (*(REG32 (0xE000801C)))
#define T1_MR2          (*(REG32 (0xE0008020)))
#define T1_MR3          (*(REG32 (0xE0008024)))
#define T1_CCR          (*(REG32 (0xE0008028)))
#define T1_CR0          (*(REG32 (0xE000802C)))
#define T1_CR1          (*(REG32 (0xE0008030)))
#define T1_CR2          (*(REG32 (0xE0008034)))
#define T1_CR3          (*(REG32 (0xE0008038)))
#define T1_EMR          (*(REG32 (0xE000803C)))
#define T1_CTCR         (*(REG32 (0xE0004070)))


/*##############################################################################
## PWM
##############################################################################*/

#define PWM_IR          (*(REG32 (0xE0014000)))
#define PWM_TCR         (*(REG32 (0xE0014004)))
#define PWM_TC          (*(REG32 (0xE0014008)))
#define PWM_PR          (*(REG32 (0xE001400C)))
#define PWM_PC          (*(REG32 (0xE0014010)))
#define PWM_MCR         (*(REG32 (0xE0014014)))
#define PWM_MR0         (*(REG32 (0xE0014018)))
#define PWM_MR1         (*(REG32 (0xE001401C)))
#define PWM_MR2         (*(REG32 (0xE0014020)))
#define PWM_MR3         (*(REG32 (0xE0014024)))
#define PWM_MR4         (*(REG32 (0xE0014040)))
#define PWM_MR5         (*(REG32 (0xE0014044)))
#define PWM_MR6         (*(REG32 (0xE0014048)))
#define PWM_EMR         (*(REG32 (0xE001403C)))
#define PWM_PCR         (*(REG32 (0xE001404C)))
#define PWM_LER         (*(REG32 (0xE0014050)))
#define PWM_CCR         (*(REG32 (0xE0014028)))
#define PWM_CR0         (*(REG32 (0xE001402C)))
#define PWM_CR1         (*(REG32 (0xE0014030)))
#define PWM_CR2         (*(REG32 (0xE0014034)))
#define PWM_CR3         (*(REG32 (0xE0014038)))

/*##############################################################################
## RTC
##############################################################################*/

/* ---- RTC: Miscellaneous Register Group ------------------ */
#define RTC_ILR         (*(REG32 (0xE0024000)))
#define RTC_CTC         (*(REG32 (0xE0024004)))
#define RTC_CCR         (*(REG32 (0xE0024008)))  
#define RTC_CIIR        (*(REG32 (0xE002400C)))
#define RTC_AMR         (*(REG32 (0xE0024010)))
#define RTC_CTIME0      (*(REG32 (0xE0024014)))
#define RTC_CTIME1      (*(REG32 (0xE0024018)))
#define RTC_CTIME2      (*(REG32 (0xE002401C)))

/* ---- RTC: Timer Control Group --------------------------- */
#define RTC_SEC         (*(REG32 (0xE0024020)))
#define RTC_MIN         (*(REG32 (0xE0024024)))
#define RTC_HOUR        (*(REG32 (0xE0024028)))
#define RTC_DOM         (*(REG32 (0xE002402C)))
#define RTC_DOW         (*(REG32 (0xE0024030)))
#define RTC_DOY         (*(REG32 (0xE0024034)))
#define RTC_MONTH       (*(REG32 (0xE0024038)))
#define RTC_YEAR        (*(REG32 (0xE002403C)))

/* ---- RTC: Alarm Control Group --------------------------- */
#define RTC_ALSEC       (*(REG32 (0xE0024060)))
#define RTC_ALMIN       (*(REG32 (0xE0024064)))
#define RTC_ALHOUR      (*(REG32 (0xE0024068)))
#define RTC_ALDOM       (*(REG32 (0xE002406C)))
#define RTC_ALDOW       (*(REG32 (0xE0024070)))
#define RTC_ALDOY       (*(REG32 (0xE0024074)))
#define RTC_ALMON       (*(REG32 (0xE0024078)))
#define RTC_ALYEAR      (*(REG32 (0xE002407C)))

/* ---- RTC: Reference Clock Divider Group ----------------- */
#define RTC_PREINT      (*(REG32 (0xE0024080)))
#define RTC_PREFRAC     (*(REG32 (0xE0024084)))


/*##############################################################################
## WD - Watchdog
##############################################################################*/

#define WD_WDMOD        (*(REG32 (0xE0000000)))
#define WD_WDTC         (*(REG32 (0xE0000004)))
#define WD_WDFEED       (*(REG32 (0xE0000008)))
#define WD_WDTV         (*(REG32 (0xE000000C)))


/*##############################################################################
## System Control Block
##############################################################################*/

#define SCB_EXTINT      (*(REG32 (0xE01FC140)))
#define SCB_EXTWAKE     (*(REG32 (0xE01FC144)))
#define SCB_MEMMAP      (*(REG32 (0xE01FC040)))
#define SCB_PLLCON      (*(REG32 (0xE01FC080)))
#define SCB_PLLCFG      (*(REG32 (0xE01FC084)))
#define SCB_PLLSTAT     (*(REG32 (0xE01FC088)))
#define SCB_PLLFEED     (*(REG32 (0xE01FC08C)))
#define SCB_PCON        (*(REG32 (0xE01FC0C0)))
#define SCB_PCONP       (*(REG32 (0xE01FC0C4)))
#define SCB_VPBDIV      (*(REG32 (0xE01FC100)))


#define PINSEL0		    (*(REG32 (0xE002C000)))
#define PINSEL1		    (*(REG32 (0xE002C004)))
#define PINSEL2		    (*(REG32 (0xE002C014)))

/*##############################################################################
## Memory Accelerator Module (MAM)
##############################################################################*/

#define MAM_TIM			(*(REG32 (0xE01FC004)))
#define MAM_CR			(*(REG32 (0xE01FC000)))

#define PCONP			(*(REG32 (0xE01FC0C4)))

//Note: PCONP bit0 is undefined
#define PCONP_TIM0		( 1UL<< 1 )
#define PCONP_TIM1      ( 1UL<< 2 )
#define PCONP_UART0		( 1UL<< 3 )
#define PCONP_UART1		( 1UL<< 4 )
#define PCONP_PWM0		( 1UL<< 5 )
#define PCONP_I2C0		( 1UL<< 7 )
#define PCONP_SPI0		( 1UL<< 8 )
#define PCONP_RTC		( 1UL<< 9 )
#define PCONP_SPI1		( 1UL<<10 )
#define PCONP_AD0		( 1UL<<12 )
#define PCONP_I2C1		( 1UL<<19 )
#define PCONP_AD1		( 1UL<<20 )
#define PCONP_USB		( 1UL<<31 )

#define PIN_UART0_TX	0
#define PIN_PWM1		0

#define PIN_UART0_RX	1 
#define PIN_PWM3		1
#define PIN_ENT0		1

#define PIN_I2C0_SCL	2 
#define PIN_TIM0_CAP0	2

#define PIN_I2C0_SDA	3
#define PIN_TIM0_MATCH0	3
#define PIN_ENT1		3

#define PIN_SPI0_SCK	4
#define PIN_TIM0_CAP1	4
#define PIN_AD0_6		4

#define PIN_SPI0_MISO	5
#define PIN_TIM0_MATCH1	5
#define PIN_AD0_7		5

#define PIN_SPI0_MOSI	6 
#define PIN_TIM0_CAP2	6 
#define PIN_AD1_0		6 

#define PIN_PWM2		7 
#define PIN_EINT2		7 
#define PIN_SPI0_SSEL	7 

#define PIN_UART1_TX	8 
#define PIN_PWM4		8 
#define PIN_AD1_1		8

#define PIN_UART1_RX    9

#define PIN_SSP_SCK     17 

#define PIN_SSP_MISO	18

#define PIN_SSP_MOSI	19

#define PIN_SSP_SSEL	20


#define PIN_FN0			0
#define PIN_FN1			1
#define PIN_FN2			2
#define PIN_FN3			3


#define VIC_CH_WDT		( 1 )
#define VIC_CH_ARMCORE0 ( 2 )
#define VIC_CH_ARMCORE1 ( 3  )
#define VIC_CH_TIM0		( 4  )
#define VIC_CH_TIM1		( 5  )
#define VIC_CH_UART0	( 6  )
#define VIC_CH_UART1	( 7  )
#define VIC_CH_PWM0		( 8  )
#define VIC_CH_I2C0		( 9  )
#define VIC_CH_SPI0		( 10  )
#define VIC_CH_SPI1		( 11  )
#define VIC_CH_SSP		VIC_CH_SPI1
#define VIC_CH_PLL		( 12  )
#define VIC_CH_RTC		( 13  )
#define VIC_CH_EINT0	( 14  )
#define VIC_CH_EINT1	( 15  )
#define VIC_CH_EINT2	( 16  )
#define VIC_CH_EINT3	( 17  )
#define VIC_CH_AD0		( 18  )
#define VIC_CH_I2C1		( 19  )
#define VIC_CH_BOD		( 20  )
#define VIC_CH_AD1		( 21  )
#define VIC_CH_USB		( 22  )

#define VIC_CLEAR_IRQ()	( VICVectAddr = 0UL)

/*
 * For drivers to use interrupts, follow this sequence:
 * 
 * portENTER_CRITICAL();
 * 
 *  .....
 * 
 * (1) Acquire irq vector or fiq resource:
 * 
 * vect = res_vic_vect_acquire_any_(__FILE__) OR
 * vect = res_vic_vect_acquire(__FILE__, required_vector) OR
 * fiq = res_fiq_acquire()
 * 
 * (2) Enable the IRQ VIC Vector or FIQ handler
 * 
 * VIC_VECT_ENABLE(vect, VIC_CH_XXX, xxx_isr); OR  
 * FIQ_HANDLER_ENABLE(xxx_isr)
 *
 * (3) Enable the IRQ or FIQ VIC Channel
 * 
 * VIC_CH_IRQ_ENABLE(VIC_CH_XXX) OR
 * VIC_CH_FIQ_ENABLE(VIC_CH_XXX) 
 * 
 * (4) Now enable the device interrupt by writing to the device CSR
 * 
 * ....
 * 
 * portEXIT_CRITICAL();
 * 
 * To disable use of interrupts (while closing a driver etc):
 * 
 * portENTER_CRITICAL();
 * 
 * ....
 * 
 * (1)  Disable device interrupt
 
 * (2) Disable IRQ/FIQ channel
 
 * VIC_CH_IRQ_DISABLE(VIC_CH_XXX) OR
 * VIC_CH_FIQ_DISABLE(VIC_CH_XXX) 
 
 * (3)Disable VIC vector of fiq handler:
 * 
 * VIC_VECT_DISABLE(vect); OR  
 * FIQ_HANDLER_DISABLE();
 *
 * (4) Release resources:
 * res_vic_vect_release(vect); OR
 * res_fiq_release()
 *   
 */

#define VIC_CH_IRQ_ENABLE(ch)	VICIntSelect &= ~(1UL<<(ch)), VICIntEnable = (1UL<<(ch))
#define VIC_CH_FIQ_ENABLE(ch)	VICIntSelect |=  (1UL<<(ch)), VICIntEnable = (1UL<<(ch))

#define VIC_CH_IRQ_DISABLE(ch)	VICIntEnClear |= (1UL<<(ch))
#define VIC_CH_FIQ_DISABLE(ch)	VICIntSelect &= ~(1UL<<(ch)), VICIntEnClear = (1UL<<(ch))

#define VIC_SLOT_EN		(1 << 5 )
#define VIC_VECT_ENABLE(v, ch, addr)\
				*(REG32(VICVectAddrBase+4*(v)))=(unsigned long)(addr);\
			    *(REG32(VICVectCntlBase+4*(v)))=((ch)|VIC_SLOT_EN) 

#define VIC_VECT_DISABLE(v)\
				*(REG32 (VICVectAddr0+4*(v)))=(0UL);\
			    *(REG32 (VICVectCntl0+4*(v)))&=(~VIC_SLOT_EN) 

typedef void (*hw_app_fiq_handler_t)(void);
extern void (*hw_app_fiq_handler)(void);

#define FIQ_HANDLER_ENABLE(h) hw_app_fiq_handler = (h)

#define FIQ_HANDLER_DISABLE() hw_app_fiq_handler = (hw_app_handler_t)0UL;

#define PORT0_DIR_OUTPUT(pin)	GPIO0_IODIR|=(1UL<<(pin))
#define PORT0_DIR_INPUT(pin)	GPIO0_IODIR&=(~(1UL<<(pin)))

#define PORT1_DIR_OUTPUT(pin)	GPIO1_IODIR|=(1UL<<(pin))
#define PORT1_DIR_INPUT(pin)	GPIO1_IODIR&=(~(1UL<<(pin)))

#define PORT0_SET(pin) 			GPIO0_IOSET =(1UL<<(pin))
#define PORT0_CLR(pin)			GPIO0_IOCLR =(1UL<<(pin))

#define PORT1_SET(pin)			GPIO1_IOSET =(1UL<<(pin))
#define PORT1_CLR(pin)			GPIO1_IOCLR =(1UL<<(pin))

#define POWER_ON(d)				PCONP|=(d)
#define POWER_OFF(d)			PCONP&=~(d)

#define PINSEL_LO(pin, function)\
   PINSEL0&=~(0x3<<((pin)<<1));\
   PINSEL0|=((function)<<((pin)<<1));\

#define PINSEL_HI(pin, function)\
   PINSEL1&=~(0x3<<(((pin)&0xf)<<1));\
   PINSEL1|=((function)<<(((pin)&0xf)<<1));\


#define PINSEL(pin, function)\
	if((pin) <= 15)\
	{\
		PINSEL_LO((pin)&0xf, (function));\
	}else\
	{\
		PINSEL_HI((pin), (function));\
	}

#define BIT(p) (1UL<<(p))
	
#endif /* lpc21xx_h */

