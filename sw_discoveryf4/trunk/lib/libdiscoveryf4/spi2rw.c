/*****************************************************************************
* File Name          : spi2rw.c
* Date First Issued  : 06/12/2011
* Board              : Discovery F4
* Description        : SPI2 routine for simultaneous read/write
*******************************************************************************/

#include "nvicdirect.h"
#include "libopencm3/stm32/nvic.h"
#include "libopencm3/stm32/f4/rcc.h"
#include "libopencm3/stm32/spi.h"
#include "DISCpinconfig.h"	// Pin configuration for STM32 Discovery board
#include "spi2rw.h"

/* Pointers and counter used during the transfer */
static char* 	spi2_outptr;	// Pointer to outgoing array
static char* 	spi2_inptr;	// Pointer to incoming array
static int 	spi2_cnt;	// SPI1 byte counter

void 	(*spi2_readdoneptr)(void);	// Address of function to call upon completion of read

/* Pin configuration to use pin as gp output.  For the options see--
'../svn_discoveryf4/sw_discoveryf4/trunk/lib/libopencm3/stm32/f4/gpio.h' */

/* Pin configurations for SPI */

// Chip select line for shift registers
static const struct PINCONFIG	outputcs = { \
	GPIO_MODE_OUTPUT,	// mode: output 
	GPIO_OTYPE_PP, 		// output type: push-pull 		
	GPIO_OSPEED_100MHZ, 	// speed: highest drive level
	GPIO_PUPD_NONE, 	// pull up/down: none
	0 };			// Alternate function code: not applicable

//  SPI input pin configuration (MISO)  */
static const struct PINCONFIG	inputaf = { \
	GPIO_MODE_AF,		// mode: Input alternate function 
	0, 			// output type: not applicable 		
	0, 			// speed: not applicable
	GPIO_PUPD_PULLUP, 	// pull up/down: pullup 
	GPIO_AF5 };		// AFRLy & AFRHy selection

//  SPI output pin configuration (SCK, MOSI) */
static const struct PINCONFIG	outputaf = { \
	GPIO_MODE_AF, 		// mode: Output alternate function
	GPIO_OTYPE_PP, 		// output type: push pull	
	GPIO_OSPEED_100MHZ, 	// speed: fastest 
	GPIO_PUPD_NONE, 	// pull up/down: none
	GPIO_AF5 };		// AFRLy & AFRHy selection

/******************************************************************************
 * void spi2rw_init(void);
 *  @brief Initialize SPI 
*******************************************************************************/ 
void spi2rw_init(void)
{
	/* Enable bus clocking for SPI2 */
	RCC_APB1ENR |= (1<<14);	// Enable SPI2 clocking

	/* Set up pins for SPI2 use. */
	f4gpiopins_Config ((volatile u32*)GPIOB, 12, (struct PINCONFIG*)&outputcs);	// CS
	f4gpiopins_Config ((volatile u32*)GPIOB, 13, (struct PINCONFIG*)&outputaf);	// SCK
	f4gpiopins_Config ((volatile u32*)GPIOB, 14, (struct PINCONFIG*)&inputaf);	// MISO
	f4gpiopins_Config ((volatile u32*)GPIOB, 15, (struct PINCONFIG*)&outputaf);	// MOSI
	f4gpiopins_Config ((volatile u32*)GPIOE,  6, (struct PINCONFIG*)&outputcs);	// PWM_LED
	GPIO_BSRR(GPIOE) = (1 << (6 + 16));		// Set LCD_PWM low initially, later connect to PWM
	GPIO_BSRR(GPIOB)  = (1 << 12);			// Set /CS high

//	temporary activate beeper 
	f4gpiopins_Config ((volatile u32*)GPIOA,  8, (struct PINCONFIG*)&outputcs);	// Beeper
	GPIO_BSRR(GPIOA) = (1 << (8 + 16));		


	// Set divisor to max.  If APB1 is 42 Mhz, then divide by 256 = 164062.5 Hz, 48 us per byte
/* NOTE: The following line is where the "phase" is set for the clock and polarity */
	//          (SSM SSI)  |enable peripheral | baud divisor | master select | CK 1 when idle | phase  | lsb first
//	for lsb first
//	SPI2_CR1 =  (0x3 << 8) |   (1 << 6)       | (0x7 << 3)   |   (1 << 2)    |    (1 << 1)    |  0x01  | (1 << 7)  ;
//	for msb first	
	SPI2_CR1 =  (0x3 << 8) |   (1 << 6)       | (0x7 << 3)   |   (1 << 2)    |    (1 << 1)    |  0x01 ;
	
	/* SPI-CR2 use default, no interrupt masks enabled at this point */

	/* Set and enable interrupt controller for SPI2 */
	NVICIPR (NVIC_SPI2_IRQ, SPI2_PRIORITY );	// Set interrupt priority
	NVICISER(NVIC_SPI2_IRQ);			// Enable interrupt controller for SPI2

	return;
}
/******************************************************************************
 * unsigned short spi2_busy(void);
 * @brief	: Test if spi2 is busy
 * @return	: 0 = /CS line is low; not-zero (busy) = /CS line is high (not busy)
*******************************************************************************/
unsigned short spi2_busy(void)
{
/* The /CS line is used to show if the SPI transfer is in progress */
	return (GPIOB_ODR & (1<<12));	// Return /CS bit
}

/******************************************************************************
 * void spi2_rw (char *pout, char * pin, int count);
 * @brief	: Initiate a transfer for: Write and read bytes
 * @param	: char *pout = pointer to byte array with bytes to output
 * @param	: char *pin  = pointer to byte array to receive bytes coming in
 * @param	: int count  = byte count of number of write/read cycles
*******************************************************************************/
void spi2_rw (char * pout, char * pin, int count)
{
	/* The following should not be necessary, but it is here JIC. */
	while ( spi2_busy() == 0 );	// Loop until a prior spi communication is complete

	GPIO_BSRR(GPIOB)  = (1<<(12+16));	// Set /CS low (show busy)
	spi2_outptr = pout;		// Set pointer for interrupt handler to store outgoing data
	spi2_inptr = pin;		// Set pointer for interrupt handler to store incoming data
	spi2_cnt = count;		// Set byte count for interrupt handler
	SPI2_DR = *spi2_outptr++;	// Load outgoing byte to start spi cycle
	SPI2_CR2 |= (SPI_CR2_RXNEIE);	// Enable receive buffer loaded (not empty) interrupt
	return;
}
/*#######################################################################################
 * ISR routine
 *####################################################################################### */
int spidebug1;

void SPI2_IRQHandler(void)
{      
/* 
The read-side (read buffer not empty, i.e. loaded) causes the interrupt.  The byte is read
from the register which resets the interrupt flag.  If there are more bytes to transfer the
tx data register is loaded with the next outgoing byte.  

When the count of bytes to transfer has been exhausted the read interrupt enable is turned
off (not really necessary), and the i/o pin with the CS line is brought back high. 
*/

	__attribute__ ((unused)) unsigned int dummy;

	if ( (SPI2_SR & SPI_SR_RXNE) != 0)  // Check for bogus interrupt
	{ /* Here, valid receive interrupt. */
		*spi2_inptr++ = SPI2_DR;	// Get byte that was read
		spi2_cnt -= 1;	 		// Decrement byte count
		if (spi2_cnt <= 0)		// Have we exhausted the count?
		{ /* Here, yes, this byte is the reponse from the last byte transmitted */ 
			SPI2_CR2 &= ~SPI_CR2_RXNEIE;	// Turn off RXE interrupt enable
			GPIO_BSRR(GPIOB)  = (1<<12);// Set /CS high (show not busy)
			if (spi2_readdoneptr != 0) // Skip if pointer is NULL
				(*spi2_readdoneptr)();	// In case we want to do something else
			return;
		}
		else
		{
spidebug1 +=1;
			SPI2_DR = *spi2_outptr++;	// Load outgoing byte to start next spi cycle
		}
	}
	dummy = SPI2_SR; // Prevent tail-chaining.
	return;
}

