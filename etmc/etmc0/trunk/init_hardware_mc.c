/******************************************************************************
* File Name          : init_hardware_mc.c
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Basic hardware initialization for master controller
*******************************************************************************/
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>



#include <malloc.h>

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "xprintf.h"
#include "init_hardware_mc.h"
#include "bsp_uart.h"
#include "4x20lcd.h"
#include "libopencm3/stm32/systick.h"	//	!!!	May not be required
#include "spi2rw.h"
#include "canwinch_ldr.h"
#include "mc_msgs.h"
#include "systick1.h"
#include "clockspecifysetup.h"
#include "libopencm3/stm32/f4/gpio.h"
#include "DISCpinconfig.h"	// Pin configuration for STM32 Discovery board
#include "adc_mc.h"
#include "beep_n_lcd.h"

//	output discrete lines for functions like chip select
static const struct PINCONFIG	outputcs = { \
	GPIO_MODE_OUTPUT,	// mode: output 
	GPIO_OTYPE_PP, 		// output type: push-pull 		
	GPIO_OSPEED_100MHZ, // speed: highest drive level
	GPIO_PUPD_NONE, 	// pull up/down: none
	0 };				// Alternate function code: not applicable

// input discrete with pull up
static const struct PINCONFIG inputpu = { \
	GPIO_MODE_INPUT, 	// mode: Input
	0,                  // output type: not applicable
	0,                  // speed: not applicable
	GPIO_PUPD_PULLUP,   // pull up/down:
	0 };                // alternate function: not used


/* The following values provide --
External 8 MHz xtal
sysclk  =  168 MHz
PLL48CK =   48 MHz
PLLCLK  =  168 MHz
AHB     =  168 MHz
APB1    =   42 MHz
APB2    =   84 MHz

NOTE: PLL48CK must be 48 MHz for the USB
*/

/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */
/* ------------- Each node on the CAN bus gets a unit number -------------------------- */
#define IAMUNITNUMBER	CAN_UNITID_GATE2	// PC<->CAN bus gateway
/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */





const struct CLOCKS clocks = { \
HSOSELECT_HSE_XTAL,	/* Select high speed osc: 0 = internal 16 MHz rc; 1 = external xtal controlled; 2 = ext input; 3 ext remapped xtal; 4 ext input */ \
1,			/* Source for main PLL & audio PLLI2S: 0 = HSI, 1 = HSE selected */ \
APBX_4,			/* APB1 clock = SYSCLK divided by 0,2,4,8,16; freq <= 42 MHz */ \
APBX_2,			/* APB2 prescalar code = SYSCLK divided by 0,2,4,8,16; freq <= 84 MHz */ \
AHB_1,			/* AHB prescalar code: SYSCLK/[2,4,8,16,32,64,128,256,512] (drives APB1,2 and HCLK) */ \
8000000,		/* External Oscillator source frequency, e.g. 8000000 for an 8 MHz xtal on the external osc. */ \
7,			/* Q (PLL) divider: USB OTG FS, SDIO, random number gen. USB OTG FS clock freq = VCO freq / PLLQ with 2 ≤ PLLQ ≤ 15 */ \
PLLP_2,			/* P Main PLL divider: PLL output clock frequency = VCO frequency / PLLP with PLLP = 2, 4, 6, or 8 */ \
84,			/* N Main PLL multiplier: VCO output frequency = VCO input frequency × PLLN with 64 ≤ PLLN ≤ 432	 */ \
2			/* M VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63 */
};


/* Parameters for setting up CAN */

// Default: based on 72 MHz clock|36 MHz AHB freqs--500,000 bps, normal, port B
//const struct CAN_PARAMS can_params = CAN_PARAMS_DEFAULT;	// See 'canwinch_pod.h'

// Experimental CAN params: Based on 64 MHz clock|32 MHz AHB freqs
const struct CAN_PARAMS can_params = { \
	IAMUNITNUMBER,	// CAN ID for this unit
	CANBAUDRATE,	// baudrate (in common_all/trunk/common_can.h)
	3,		// port: port: 0 = PA 11|12; 2 = PB; 3 = PD 0|1;  (1 = not valid; >3 not valid) 
	0,		// silm: CAN_BTR[31] Silent mode (0 or non-zero)
	0,		// lbkm: CAN_BTR[30] Loopback mode (0 = normal, non-zero = loopback)
	4,		// sjw:  CAN_BTR[24:25] Resynchronization jump width
	4,		// tbs2: CAN_BTR[22:20] Time segment 2 (e.g. 5)
	11,		// tbs1: CAN_BTR[19:16] Time segment 1 (e.g. 12)
	1,		// dbf:  CAN_MCR[16] Debug Freeze; 0 = normal; non-zero = Debug Freeze
	0,		// ttcm: CAN_MCR[7] Time triggered communication mode
	1,		// abom: CAN_MCR[6] Automatic bus-off management
	0,		// awum: CAN_MCR[5] Auto WakeUp Mode
	0		// nart: CAN_MCR[4] No Automatic ReTry (0 = retry; non-zero = transmit once)
};


/* file descriptor */
int fd;


/* ***********************************************************************************************************
 * void init_hardware_mc (void);
 * @brief	:Setup & initialization functions 
 ************************************************************************************************************* */
void init_hardware_mc (void) 
{
	int init_ret = -4;
	/* --------------------- Begin setting things up -------------------------------------------------- */ 
		clockspecifysetup((struct CLOCKS*) & clocks);		// Get the system clock and bus clocks running
	/* ---------------------- Set up pins ------------------------------------------------------------- */
		/* Configure pins */
		DISCgpiopins_Config();	// Configure pins
	/* ---------------------- Set usb ----------------------------------------------------------------- */
		// usb1_init();	// Initialization for USB (STM32F4_USB_CDC demo package)
		//setbuf(stdout, NULL);
	/* --------------------- Initialize UARTs ---------------------------------------------------- */
	    //  bsp_uart_int_init_number(u32 uartnumber, u32 baud,u32 rxbuffsize, u32 txbuffsize, u32 dma_tx_int_priority);
		bsp_uart_int_init_number(UARTGATE, 230400, 1024, 1024, 0x40);	// UART used for the Gateway
		bsp_uart_int_init_number(UXPRT,    115200, 256, 256, 0xB0);	// UART used for debugging		
		
		lcd_init(UARTLCD); 						// UART used for the LCD screen
										
	/* Setup STDOUT, STDIN (a shameful sequence until we sort out 'newlib' and 'fopen'.)  The following 'open' sets up 
	   the USART/UART that will be used as STDOUT_FILENO, and STDIN_FILENO.  Don't call 'open' again!  */
		fd = open("tty2", 0,0); // This sets up the uart control block pointer versus file descriptor ('fd')
	
	/* ---------------------- Let the Op know it is alive ------------------------------------ */
	/* Announce who we are. ('xprintf' uses uart number to deliver the output.) */		
	xprintf(UXPRT,  " \n\rDISCOVERY F4 MASTER CONTROLLER: 09/04/2014  v0\n\r");
	/* Make sure we have the correct bus frequencies */
	xprintf (UXPRT, "   hclk_freq (MHz) : %9u...............................\n\r",  hclk_freq/1000000);	
	xprintf (UXPRT, "  pclk1_freq (MHz) : %9u...............................\n\r", pclk1_freq/1000000);	
	xprintf (UXPRT, "  pclk2_freq (MHz) : %9u...............................\n\r", pclk2_freq/1000000);	
	xprintf (UXPRT, " sysclk_freq (MHz) : %9u...............................\n\r",sysclk_freq/1000000);

	/* --------------------- Initialize SPI2 ------------------------------------------------------------------------------- */
	spi2rw_init();
//	xprintf (UXPRT, "   SPI Init Complete\n\r");
	/* --------------------- ADC initialization ---------------------------------------------------------------------------- */
	int i = adc_mc_init_sequence();
	if (i < 0)
	{
		show_op_the_error("ADC init failed", init_ret, 39);  // Show error and pause for 3.9 secs
	}
	/* --------------------- CAN setup ------------------------------------------------------------------- */
		/*  Pin usage for CAN--
		PD00 CAN1  Rx LQFP 81 Header P2|36 BLU
		PD01 CAN1  Tx LQFP 82 Header P2|33 WHT
		PC04 GPIIO RS LQFP 33 Header P1|20 GRN
		*/
		/* Configure CAN driver RS pin: PC4 LQFP 33, Header P1|20, fo hi speed. */
		can_nxp_setRS_ldr(0,(volatile u32 *)GPIOC, 4); // (1st arg) 0 = high speed mode; not-zero = standby modeshow_op_the_error("CAN init failed", init_ret, 40);

		/* Setup CAN registers and initialize routine */
		init_ret = can_init_pod_ldr((struct CAN_PARAMS*)&can_params); // 'struct' that holds all the parameters

		/* Check if initialization was successful, or timed out. */
		if (init_ret <= 0)
		{ // Here the init returned an error code
			show_op_the_error("CAN init failed", init_ret, 40);  // Show error and pause for 4.0 secs
		}
		// xprintf (UXPRT, "\n\rcan ret ct: %d..............................................\n\r",init_ret); // Just a check for how long "exit initialization" took
		/* Set filters to respond "this" unit number and time sync broadcasts */
		can_filter_unitid_ldr(can_params.iamunitnumber);	// Setup msg filter banks

		// xprintf (UXPRT, " IAMUNITNUMBER %0x %0x.....................................\n\r",(unsigned int)IAMUNITNUMBER,(unsigned int)CAN_UNITID_SE1 >> CAN_UNITID_SHIFT); 

		/* Since this is a gateway set the filter for the hardware to accept all msgs. */
		int can_ret = can_filtermask16_add_ldr( 0 );	// Allow all msgs
		/* Check if filter initialization was successful, or timed out. */
		if (can_ret < 0)
		{
			show_op_the_error("CAN filter mask init", init_ret, 25);  // Show error and pause for 2.5 secs
		}
		xprintf (UXPRT,"CAN initialization completed\n\r");

		//	configure beeper discrete I/O
		f4gpiopins_Config ((volatile u32*)GPIOA,  8, (struct PINCONFIG*)&outputcs);	// Beeper
		GPIO_BSRR(GPIOA) = (1 << (8 + 16));	

		//	configure glass control panel detection pin
		f4gpiopins_Config ((volatile u32*)GPIOB, 1, (struct PINCONFIG*)&inputpu);

		//	is delay needed here to let pull up work?


	/* --------------------- Hardware is ready, so do program-specific startup ---------------------------- */


	/* --------------------- Decoding chars from USB-serial port ------------------------------------------ */
		mc_msg_init();
		xprintf (UXPRT,"HARDWARE INITIALIZATION COMPLETED\n\r");
	return;
}
