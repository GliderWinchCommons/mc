/******************************************************************************
* File Name          : etmc0.c
* Date               : 8/9/2014
* Board              : F4-Discovery
* Description        : Electric Thing Prototype Master Controller
*******************************************************************************/
/* 

8/9/2014 This code was started as a compilation of gatef.c, adctest, and spi2 tests.

*/
#include <fcntl.h>
#include <unistd.h>


#include <math.h>
#include <string.h>
#include <stdio.h>
#include "xprintf.h"
#include <malloc.h>

//	Added from spi2test.c
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>

#include "libopencm3/stm32/f4/adc.h"
#include "adc_mc.h"						

//#include "libopencm3/stm32/f4/rcc.h"	//	!!!	May not be required
#include "libopencm3/stm32/f4/gpio.h"
#include "libopencm3/stm32/f4/scb.h"

#include "systick1.h"
#include "clockspecifysetup.h"

#include "DISCpinconfig.h"	// Pin configuration for STM32 Discovery board

#include "canwinch_ldr.h"
#include "common_can.h"
#include "panic_leds.h"
#include "PC_gateway_comm.h"
#include "USB_PC_gateway.h"
#include "CAN_gateway.h"
#include "bsp_uart.h"
#include "4x20lcd.h"
#include "libopencm3/stm32/systick.h"	//	!!!	May not be required
#include "CAN_test_msgs.h"
#include "CAN_error_msgs.h"
#include "spi2rw.h"

#include "mccp.h"

#include "common_canid_et.h"

/* The following includes code a gateway. */
// NOTE: "USB" refers to the serial port for a gateway
#define GATEWAYLOCAL	


void delay_tenth_sec(unsigned int t);
static void canmcbuf_add(struct CANRCVBUF* p);
void calib_control_lever(void);
static void msg_out_mc(struct CANRCVBUF* p);
static void msg_out_usb(struct CANRCVBUF* p);


/* USART|UART assignment for xprintf and read/write */
#define UARTLCD	3 //6	// Uart number for LCD messages
#define UARTGPS	6 //3	// Uart number for GPS or debug
#define UARTGATE 2  // UART number for Gateway (possibly Host later)

#define UXPRT UARTGPS	//	Debugging port

#define DTWTIME	(*(volatile unsigned int *)0xE0001004)	// Read DTW 32b system tick counter

/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */
/* ------------- Each node on the CAN bus gets a unit number -------------------------- */
#define IAMUNITNUMBER	CAN_UNITID_GATE2	// PC<->CAN bus gateway
/* &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& */


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

static struct PCTOGATEWAY pctogateway; // CAN->PC
static struct PCTOGATEWAY gatewayToPC; // PC->CAN




/* Circular buffer for incoming CAN + USB -> MC msgs */
#define CANMCBUFSIZE	8			// Number of incoming CAN msgs to buffer
static struct CANRCVBUF canmcbuf[CANMCBUFSIZE];
static int canmcidxi = 0;			// Incoming index into canbuf
static int canmcidxm = 0;			// Outgoing index into canbuf

/* Advance circular pointer macro */
static int incIdx(int x, int y){x += 1; if (x >= y) x = 0; return x;} 

static struct CANRCVBUF* 	pfifo0;		// Pointer to CAN driver buffer for incoming CAN msgs, low priority
static struct CANRCVTIMBUF*	pfifo1;		// Pointer to CAN driver buffer for incoming CAN msgs, high priority

/* Put sequence number on incoming CAN messages that will be sent to the PC */
u8 canmsgctr = 0;	// Count incoming CAN msgs


// State machine
int currentState = 0;
int nextState = 0;

/* file descriptor */
int fd;
char vv[128];	// sprintf buf

// led timer
u32	t_led;
#define FLASHCOUNT (sysclk_freq/2);	// Orange LED flash increment

// 64th second counter
u32 t_timeKeeper;
#define SIXTYFOURTH (sysclk_freq/64);
u8 count64 = 0;
u32 currentTime = 0;
u8 timerMsgFlag = 0;

// lcd
u32 t_lcd;
#define LCDPACE (sysclk_freq/2); // LCD pacing increment



// SPI globals
char spi_ledout[SPI2SIZE] = {0xAA,0x55};	// Initial outgoing pattern
char spi_swin[SPI2SIZE];
u32	t_spi;

// tension and speed
float desiredTension = 0.0;
float desiredSpeed = 0.0;
float outputTorque = 0.0;



/* --------------- For debugging...----------------------------------- */
int Default_HandlerCode = 999;
u32 DH08;
void Default_Handler08(void) {DH08 += 1; return;}

void OTG_FS_IRQHandler(void);
void Default_Handler76(void) {	OTG_FS_IRQHandler(); return; }

void Default_Handler00(void) { Default_HandlerCode =  0; panic_leds(5); }
void Default_Handler01(void) { Default_HandlerCode =  1; panic_leds(5); }
void Default_Handler02(void) { Default_HandlerCode =  2; panic_leds(5); }
void Default_Handler03(void) { Default_HandlerCode =  3; panic_leds(5); }
void Default_Handler04(void) { Default_HandlerCode =  4; panic_leds(5); }
void Default_Handler05(void) { Default_HandlerCode =  5; panic_leds(5); }
void Default_Handler06(void) { Default_HandlerCode =  6; panic_leds(5); }
void Default_Handler07(void) { Default_HandlerCode =  7; panic_leds(5); }
//void Default_Handler08(void) { Default_HandlerCode =  8; panic_leds(5); }
void Default_Handler09(void) { Default_HandlerCode =  9; panic_leds(5); }
void Default_Handler10(void) { Default_HandlerCode = 10; panic_leds(5); }
void Default_Handler11(void) { Default_HandlerCode = 11; panic_leds(5); }
void Default_Handler12(void) { Default_HandlerCode = 12; panic_leds(5); }
void Default_Handler13(void) { Default_HandlerCode = 13; panic_leds(5); }
void Default_Handler14(void) { Default_HandlerCode = 14; panic_leds(5); }
void Default_Handler15(void) { Default_HandlerCode = 15; panic_leds(5); }
void Default_Handler16(void) { Default_HandlerCode = 16; panic_leds(5); }
void Default_Handler17(void) { Default_HandlerCode = 17; panic_leds(5); }
void Default_Handler18(void) { Default_HandlerCode = 18; panic_leds(5); }
void Default_Handler19(void) { Default_HandlerCode = 19; panic_leds(5); }
void Default_Handler20(void) { Default_HandlerCode = 20; panic_leds(5); }
void Default_Handler21(void) { Default_HandlerCode = 21; panic_leds(5); }
void Default_Handler22(void) { Default_HandlerCode = 22; panic_leds(5); }
void Default_Handler23(void) { Default_HandlerCode = 23; panic_leds(5); }
void Default_Handler24(void) { Default_HandlerCode = 24; panic_leds(5); }
void Default_Handler25(void) { Default_HandlerCode = 25; panic_leds(5); }
void Default_Handler26(void) { Default_HandlerCode = 26; panic_leds(5); }
void Default_Handler27(void) { Default_HandlerCode = 27; panic_leds(5); }
void Default_Handler28(void) { Default_HandlerCode = 28; panic_leds(5); }
void Default_Handler29(void) { Default_HandlerCode = 29; panic_leds(5); }
void Default_Handler30(void) { Default_HandlerCode = 30; panic_leds(5); }
void Default_Handler31(void) { Default_HandlerCode = 31; panic_leds(5); }
void Default_Handler32(void) { Default_HandlerCode = 32; panic_leds(5); }
void Default_Handler33(void) { Default_HandlerCode = 33; panic_leds(5); }
void Default_Handler34(void) { Default_HandlerCode = 34; panic_leds(5); }
void Default_Handler35(void) { Default_HandlerCode = 35; panic_leds(5); }
void Default_Handler36(void) { Default_HandlerCode = 36; panic_leds(5); }
void Default_Handler37(void) { Default_HandlerCode = 37; panic_leds(5); }
void Default_Handler38(void) { Default_HandlerCode = 38; panic_leds(5); }
void Default_Handler39(void) { Default_HandlerCode = 39; panic_leds(5); }
void Default_Handler40(void) { Default_HandlerCode = 40; panic_leds(5); }
void Default_Handler41(void) { Default_HandlerCode = 41; panic_leds(5); }
void Default_Handler42(void) { Default_HandlerCode = 42; panic_leds(5); }
void Default_Handler43(void) { Default_HandlerCode = 43; panic_leds(5); }
void Default_Handler44(void) { Default_HandlerCode = 44; panic_leds(5); }
void Default_Handler45(void) { Default_HandlerCode = 45; panic_leds(5); }
void Default_Handler46(void) { Default_HandlerCode = 46; panic_leds(5); }
void Default_Handler47(void) { Default_HandlerCode = 47; panic_leds(5); }
void Default_Handler48(void) { Default_HandlerCode = 48; panic_leds(5); }
void Default_Handler49(void) { Default_HandlerCode = 49; panic_leds(5); }
void Default_Handler50(void) { Default_HandlerCode = 50; panic_leds(5); }
void Default_Handler51(void) { Default_HandlerCode = 51; panic_leds(5); }
void Default_Handler52(void) { Default_HandlerCode = 52; panic_leds(5); }
void Default_Handler53(void) { Default_HandlerCode = 53; panic_leds(5); }
void Default_Handler54(void) { Default_HandlerCode = 54; panic_leds(5); }
void Default_Handler55(void) { Default_HandlerCode = 55; panic_leds(5); }
void Default_Handler56(void) { Default_HandlerCode = 56; panic_leds(5); }
void Default_Handler57(void) { Default_HandlerCode = 57; panic_leds(5); }
void Default_Handler58(void) { Default_HandlerCode = 58; panic_leds(5); }
void Default_Handler59(void) { Default_HandlerCode = 59; panic_leds(5); }
void Default_Handler60(void) { Default_HandlerCode = 60; panic_leds(5); }
void Default_Handler61(void) { Default_HandlerCode = 61; panic_leds(5); }
void Default_Handler62(void) { Default_HandlerCode = 62; panic_leds(5); }
void Default_Handler63(void) { Default_HandlerCode = 63; panic_leds(5); }
void Default_Handler64(void) { Default_HandlerCode = 64; panic_leds(5); }
void Default_Handler65(void) { Default_HandlerCode = 65; panic_leds(5); }
void Default_Handler66(void) { Default_HandlerCode = 66; panic_leds(5); }
void Default_Handler67(void) { Default_HandlerCode = 67; panic_leds(5); }
void Default_Handler68(void) { Default_HandlerCode = 68; panic_leds(5); }
void Default_Handler69(void) { Default_HandlerCode = 69; panic_leds(5); }
void Default_Handler70(void) { Default_HandlerCode = 70; panic_leds(5); }
void Default_Handler71(void) { Default_HandlerCode = 71; panic_leds(5); }
void Default_Handler72(void) { Default_HandlerCode = 72; panic_leds(5); }
void Default_Handler73(void) { Default_HandlerCode = 73; panic_leds(5); }
void Default_Handler74(void) { Default_HandlerCode = 74; panic_leds(5); }
void Default_Handler75(void) { Default_HandlerCode = 75; panic_leds(5); }
//void Default_Handler76(void) { Default_HandlerCode = 76; panic_leds(5); }
void Default_Handler77(void) { Default_HandlerCode = 77; panic_leds(5); }
void Default_Handler78(void) { Default_HandlerCode = 78; panic_leds(5); }
void Default_Handler79(void) { Default_HandlerCode = 79; panic_leds(5); }
void Default_Handler80(void) { Default_HandlerCode = 80; panic_leds(5); }
void Default_Handler81(void) { Default_HandlerCode = 81; panic_leds(5); }
void Default_Handler82(void) { Default_HandlerCode = 82; panic_leds(5); }
void Default_Handler83(void) { Default_HandlerCode = 83; panic_leds(5); }
void Default_Handler84(void) { Default_HandlerCode = 84; panic_leds(5); }
void Default_Handler85(void) { Default_HandlerCode = 85; panic_leds(5); }
void Default_Handler86(void) { Default_HandlerCode = 86; panic_leds(5); }
void Default_Handler87(void) { Default_HandlerCode = 87; panic_leds(5); }
void Default_Handler88(void) { Default_HandlerCode = 88; panic_leds(5); }
void Default_Handler89(void) { Default_HandlerCode = 89; panic_leds(5); }
void Default_Handler90(void) { Default_HandlerCode = 90; panic_leds(5); }


/* LED identification
Discovery F4 LEDs: PD 12, 13, 14, 15

12 green   
13 orange
14 red
15 blue
*/

/* ************************************************************
 Beeper
***************************************************************/
void single_beep(void)
{
	GPIO_BSRR(GPIOA) = 1 << 8;	// Turn on beeper
	delay_tenth_sec(1);		// 1/10th sec ON
	GPIO_BSRR(GPIOA) = 1 << (8 + 16);// Turn off beeper
	delay_tenth_sec(2);		// 1/10th sec OFF
	return;	
}
void double_beep(void)
{
	single_beep();
	single_beep();
}
void triple_beep(void)
{
	single_beep();
	single_beep();
	single_beep();
}
void delay_tenth_sec(unsigned int t)
{
	int i;
	unsigned int tp = sysclk_freq/10;	// Increment for 1/10th sec
	unsigned int t0 = DTWTIME + tp;
	for (i = 0; i < t; i++)
	{
		while (((int)(DTWTIME - t0)) < 0) // Has the time expired?
		t0 += tp;
	}
	return;
}
/* ************************************************************
 * void show_op_the_error(char* p, int e, unsigned int t);
 * @brief	: Show Op the error, beep, and pause
 * @param	: p = pointer to string that goes on line 0 of LCD
 * @param	: e = error code
 * @param	: t = number of 1/10th secs to pause before continuing.
***************************************************************/
void show_op_the_error(char* p, int e, unsigned int t)
{
	lcd_printToLine(UARTLCD,0, p);	// Line 0 tells "who"
	sprintf(vv, "er code: %d",e); 	
	lcd_printToLine(UARTLCD, 1, vv);	// Line 1 shows error code
	xprintf(UXPRT,"ERROR: %s er code: %s\n\r",p, vv); // Output to debugging serial port
	triple_beep();
	delay_tenth_sec(t);
	return;
}
/* ************************************************************
Turn the LEDs on in sequence, then turn them back off 
***************************************************************/
static int lednum = 12;	// Lowest port bit numbered LED
void toggle_4leds (void)
{
	if ((GPIO_ODR(GPIOD) & (1<<lednum)) == 0)
	{ // Here, LED bit was off
		GPIO_BSRR(GPIOD) = (1<<lednum);	// Set bit
	}
	else
	{ // Here, LED bit was on
		GPIO_BSRR(GPIOD) = (1<<(lednum+16));	// Reset bit
	}
	lednum += 1;		// Step through all four LEDs
	if (lednum > 15) lednum = 12;

}


/* ************************************************************
Turn the LEDs on in sequence, then turn them back off 
***************************************************************/
void toggle_led (int lednum)
{
	if ((GPIO_ODR(GPIOD) & (1<<lednum)) == 0)
	{ // Here, LED bit was off
		GPIO_BSRR(GPIOD) = (1<<lednum);	// Set bit
	}
	else
	{ // Here, LED bit was on
		GPIO_BSRR(GPIOD) = (1<<(lednum+16));	// Reset bit
	}

}



/* ***********************************************************************************************************
 * void initMasterController(void)
 * @brief	:Setup & initialization functions 
 ************************************************************************************************************* */
void initMasterController (void) {
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
		bsp_uart_int_init_number(UARTGATE, 230400, 256, 256, 0x40);	// UART used for the Gateway
		bsp_uart_int_init_number(UXPRT,    115200, 256, 256, 0xB0);	// UART used for debugging		
		lcd_init(UARTLCD); 						// UART used for the LCD screen
										
	/* Setup STDOUT, STDIN (a shameful sequence until we sort out 'newlib' and 'fopen'.)  The following 'open' sets up 
	   the USART/UART that will be used as STDOUT_FILENO, and STDIN_FILENO.  Don't call 'open' again!  */
		fd = open("tty2", 0,0); // This sets up the uart control block pointer versus file descriptor ('fd')
	
	/* ---------------------- DTW sys counter -------------------------------------------------------- */
		// Use DTW_CYCCNT counter (driven by sysclk) for polling type timing 
		// CYCCNT counter is in the Cortex-M-series core.  See the following for details 
		// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337g/BABJFFGJ.html 
		*(volatile unsigned int*)0xE000EDFC |= 0x01000000; // SCB_DEMCR = 0x01000000;
		*(volatile unsigned int*)0xE0001000 |= 0x1;	// Enable DTW_CYCCNT (Data Watch cycle counter)

	/* ---------------------- Let the Op know it is alive ------------------------------------ */
	/* Announce who we are. ('xprintf' uses uart number to deliver the output.) */		
	xprintf(UXPRT,  " \n\rDISCOVERY F4 MASTER CONTROLLER: 08/30/2014  v0\n\r");
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
	/* --------------------- Hardware is ready, so do program-specific startup ---------------------------- */

	/* --------------------- Decoding chars from USB-serial port ------------------------------------------ */
#ifdef GATEWAYLOCAL
		PC_msg_initg(&pctogateway);	// Initialize struct for CAN message from PC
		PC_msg_initg(&gatewayToPC);	// Initialize struct for CAN message from PC

		/* Set modes for USB-serial port routines that receive and send CAN msgs */
		pctogateway.mode_link = MODE_LINK;
		gatewayToPC.mode_link = MODE_LINK;
#endif
	/* --------------------- Control lever calibration ---------------------------------------------------------------- */
//$$$		calib_control_lever();

	/* --------------------- Initial times ---------------------------------------------------------------------------- */
		t_led = DTWTIME + FLASHCOUNT; 
		t_lcd = DTWTIME + LCDPACE;

		xprintf (UXPRT,"ALL INITIALIZATION COMPLETED\n\r");
	return;
}
/* ***********************************************************************************************************
 * void calib_control_lever(void);
 * @brief	:Setup & initialization functions 
 ************************************************************************************************************* */

//	code for calibrating scale and offset for the control lever
//	make function later
#define FSCL	((1 << 12) - 1)	// full scale control lever (CL) output
#define CLREST (1 << 11) 	// SPI bit position for CL rest position switch
#define CLFS  (1 << 8) 		// SPI bit position for CL full scale position
#define CL_ADC_CHANNEL 	0

int cal_cl;			// calibrated control lever output
int cloffset = 0, clmax = 0;	// Min and maximum values observed for control lever
int clscale = 0;		// scale value for generating calibrated output

void calib_control_lever(void)
{
	int clcalstate = 0;		// state for control lever intial calibration
	int sw = 0;			// binary for holding switch values
	int adc_tmp;

	t_led = DTWTIME + FLASHCOUNT;	//	initial t_led

	GPIO_BSRR(GPIOA) = 1 << 8;	// Turn on beeper
	while(clcalstate < 6)
	{
		if (((int)(DTWTIME - t_led)) > 0) // Has the time expired?
		{ //	Time expired
			xprintf(UXPRT, "%5u %8x \n\r", clcalstate, sw);
			//	read filtered control lever adc last value and update min and max values
			adc_tmp = adc_last_filtered[CL_ADC_CHANNEL];
			cloffset = (cloffset < adc_tmp) ? cloffset : adc_tmp;
			clmax = (clmax > adc_tmp) ? clmax : adc_tmp;
			//	Read SPI switches
			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // SPI completed  
				spi2_rw(spi_ledout, spi_swin, SPI2SIZE); // Send/rcv SPI2SIZE bytes
				//	convert to a binary word for comparisons (not general)
				sw = (((int) spi_swin[0]) << 8) | (int) spi_swin[1];				
				switch(clcalstate)
				{				
					case 0:	//	entry state
					{
						sprintf(vv, "Cycle control lever");
						lcd_printToLine(UARTLCD, 0, vv);
						sprintf(vv, "twice:");
						lcd_printToLine(UARTLCD, 1, vv);
						double_beep();
						clcalstate = 1;
					}
					case 1:	// waiting for CL to rest position	
					{
						if (sw & CLREST) break;
						clcalstate = 2;
						cloffset = clmax = adc_tmp;	//	reset min and max values
						sprintf(vv, "twice: 0");
						lcd_printToLine(UARTLCD, 1, vv);
						break;
					}
					case 2:	//	waiting for full scale position first time
					{
						if (!(sw & CLFS)) clcalstate = 3;					
						break;
					}
					case 3:	//	wating for return to rest first time
					{
						if (sw & CLREST) break; 
							{
								clcalstate = 4;
								sprintf(vv, "twice: 1");
								lcd_printToLine(UARTLCD, 1, vv);
								single_beep();
								break;
							}
					}
					case 4:	//	waiting for full scale second time
					{
						if (!(sw & CLFS)) clcalstate = 5;					
						break;
					}
					case 5:	//	waiting for return to rest second time
					{
						if (sw & CLREST) break;
						clscale = (FSCL  << 16) / (clmax - cloffset);
						lcd_clear(UARTLCD);
						single_beep();
						clcalstate = 6; 
					}
				}			
			}
			toggle_4leds(); 	// Advance some LED pattern
			t_led += FLASHCOUNT; 	// Set next toggle time		
		}
	}	
	GPIO_BSRR(GPIOA) = 1 << (8 + 16);	// Turn off beeper
	xprintf(UXPRT, "  cloffset: %10d clmax: %10d clscale: %10d \n\r", cloffset, clmax, clscale);
	xprintf 	(UXPRT, "   Control Lever Initial Calibration Complete\n\r");
	return;
}
/* ============================= main loop functions ============================================================ */
/* **************************************************************************************
 * static void canmcbuf_add(struct CANRCVBUF* p);
 * @brief	: Add msg to buffer holding structs of CAN format msg
 * @param	: p = Pointer to CAN msg
 * ************************************************************************************** */
static void canmcbuf_add(struct CANRCVBUF* p)
{
	int temp;
	canmcbuf[canmcidxi] = *p;		// Copy struct
	temp = incIdx(canmcidxi,CANMCBUFSIZE);	// Increment the index for incoming msgs.
	if (canmcidxm == temp)  		// Did this last fill the last one?
	{ // Yes, we have filled the buffer.  This CAN msg might be dropped (by not advancing the index)
		Errors_misc(-1);		// Add to buffer overrun counter
	}
	else
	{ // Here, there is room in the buffer and we are good to go.
		canmcidxi = temp;		// Update the index to next buffer position.
	}	
	return;
}

/* **************************************************************************************
 * void ledHeartbeat(void);
 * @brief	: Flash the red LED to amuse the hapless Op or signal the wizard programmer that the loop is running.
 * ************************************************************************************** */
	void ledHeartbeat (void) {
		if (((int)(DTWTIME - t_led)) > 0) // Has the time expired?
		{ // Here, yes.
			t_led += FLASHCOUNT; 	// Set next toggle time
			toggle_led(14); 	// Advance some LED pattern
		}
	}

/* **************************************************************************************
 * void void timeKeeper(void);
 * @brief	: function to find the 64th second beats
 * ************************************************************************************** */
	void timeKeeper (void) 
	{
		struct CANRCVBUF can;
		if (((int)(DTWTIME - t_timeKeeper)) > 0) // Has the time expired?
		{ // Here, yes.
			t_timeKeeper += SIXTYFOURTH; 	// Set next toggle time
			count64++;
			can.id       = CANID_TIME; // time id
			if(count64 == 64) {
				currentTime++;
				count64 = 0;
				can.dlc      = 0x00000004;
				can.cd.us[0] = currentTime;
xprintf(UXPRT,"T %d\n\r",currentTime);

			} else {
				can.dlc      = 0x00000001;
				can.cd.us[0] = count64;
			}
			msg_out_mc(&can); // Output to CAN+USB
		}
	}
/* **************************************************************************************
 * void spiInOut(void);
 * @brief	: SPI send/rcv & pacing 
 * ************************************************************************************** */
	void spiInOut (void) {
		if (((int)(DTWTIME - t_spi)) > 0) {
			t_spi += SPIPACE;	// (200 per sec)

			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // Here, no.
				spi2_rw(spi_ledout, spi_swin, SPI2SIZE); // Send/rcv three bytes
			}
		}
	}
/* **************************************************************************************
 * void lcdOut(void);
 * @brief	: LCD output and pacing 
 * ************************************************************************************** */
	void lcdOut (void) {
		char lcdLine[LCDLINESIZE + 1];
		if (((int)(DTWTIME - t_lcd)) > 0) {
			t_lcd += LCDPACE;

			snprintf(lcdLine, 20, "State %4d", currentState); 		lcd_printToLine(UARTLCD, 0, lcdLine);
			snprintf(lcdLine, 20, "Torq  %0.2f", outputTorque);		lcd_printToLine(UARTLCD, 1, lcdLine);
			snprintf(lcdLine, 20, "Time: %d", (int) currentTime);		lcd_printToLine(UARTLCD, 2, lcdLine);
int outputTorquei = outputTorque * 10;
xprintf(UXPRT,"State %4d: Torq  %0.2f: Time: %d\n\r",currentState,outputTorquei,(int) currentTime);
		}
	}
/* **************************************************************************************
 * void stateZero(void);
 * @brief	: State Zero 
 * ************************************************************************************** */
	void stateZero (void) {
		// TODO: check for conditions that progress the state
		if (currentTime > 60) {
			nextState = 1;
		}
	}
/* **************************************************************************************
 * void stateOne(void);
 * @brief	: State One 
 * ************************************************************************************** */
	void stateOne (void) {

	}
/* **************************************************************************************
 * void stateMachine(void);
 * @brief	: State Machine
 * ************************************************************************************** */
	void stateMachine (void) {
		currentState = nextState;

		switch(currentState) {
			case 0:
				stateZero();
				break;
			case 1:
				stateOne();
				break;
		}
	}
/* **************************************************************************************
 * static void desiredTensionSpeed(void);
 * @brief	: compute tension speed
 * ************************************************************************************** */
	void desiredTensionSpeed (void) {
		desiredSpeed = 1;
		desiredTension = 2;
	}
/* **************************************************************************************
 * static void controlLaw(void);
 * @brief	: compute output torque
 * ************************************************************************************** */
	void controlLaw (void) {
		if (currentState == 0) {
			outputTorque = 0.5 * desiredSpeed;
		} else {
			outputTorque = 0.5 * desiredTension;
		}
	}
/* **************************************************************************************
 * static void msg_out_can(struct CANRCVBUF* p);
 * @brief	: Output msg from MC to CAN and USB
 * ************************************************************************************** */
static void msg_out_can(struct CANRCVBUF* p)
{
	CAN_gateway_send(p);	// Add to xmit buffer (if OK)
	return;
}

/* **************************************************************************************
 * static void msg_out_mc(struct CANRCVBUF* p);
 * @brief	: Output msg from MC to CAN and USB
 * ************************************************************************************** */
static void msg_out_mc(struct CANRCVBUF* p)
{
#ifdef GATEWAYLOCAL
	msg_out_usb(p);
#endif
	msg_out_can(p);
	return;
}
/* **************************************************************************************
 * static void msg_out_usb(struct CANRCVBUF* p);
 * @param	: p = pointer to struct with msg to be sent
 * @brief	: Output msg to USB-serial port
 * ************************************************************************************** */
static void msg_out_usb(struct CANRCVBUF* p)
{
	pctogateway.cmprs.seq = canmsgctr++;		// Add sequence number (for PC checking for missing msgs)
	USB_toPC_msg_mode(STDOUT_FILENO, &pctogateway, p); 	// Send to PC via STDOUT
	return;
}
/* **************************************************************************************
 * static int msg_get_can(void);
 * @brief	: Check and get incoming msg from CAN bus
 * @return	: Pointer to a struct CANRCVBUF if there is a msg, otherwise return NULL
 * ************************************************************************************** */
static struct CANRCVBUF* msg_get_can(void)
 {
	if ( (pfifo1 = canrcvtim_get_ldr()) != 0)	// Did we receive a HIGH PRIORITY CAN BUS msg?
	{ // Here yes.
		return &pfifo1->R;	// Return pointer to CANRCVBUF struct
	}
	if ( (pfifo0 = canrcv_get_ldr()) != 0)		// Did we receive a LESS-THAN-HIGH-PRIORITY CAN BUS msg?
	{ // Here yes.
		return pfifo0;	// Add msg to buffer
	}
	return NULL;
	}
/* **************************************************************************************
 * static struct CANRCVBUF* msg_get_usb(void);
 * @brief	: Check and get incoming msg from CAN bus
 * @return	: Pointer to a struct CANRCVBUF if there is a msg, otherwise return NULL
 * ************************************************************************************** */
u32 msg_get_usb_err1 = 0;	// Running error count
static struct CANRCVBUF canrcvbuf;

static struct CANRCVBUF* msg_get_usb(void)
{
	int tmp;
	int temp;
	struct CANRCVBUF* p = 0;  // default to NULL

	/* Each time 'USB_PC_get_msg_mode' is called it adds any buffered incoming ASCII chars */
	temp=USB_PC_get_msg_mode(STDIN_FILENO, &gatewayToPC, &canrcvbuf);	// Check if msg is ready
	if (temp != 0)	// Do we have completion of a msg?
	{ // Here, yes.  We have a msg, but it might not be valid.
		if ( temp == 1 ) // Was valid?
		{ // Here, yes.
			tmp = temp >> 16; // Isolate compression error codes
			if (tmp < 0)	// Did the compression detect some anomolies?
			{ // Here, something wrong with the msg--
				msg_get_usb_err1 += 1;	// Count errors
			}
			else
			{ // Here, msg is OK msg from the PC
				p = &canrcvbuf;	// Return pointer to struct with msg
			}
		}
		else
		{ // Something wrong with the msg.  Count the various types of error returns from 'USB_PC_msg_getASCII'
			Errors_USB_PC_get_msg_mode(temp);
		} // Note: 'pctogateway' gets re-intialized in 'PC_msg_initg' when there are errors.

		/* Initialize struct for next msg from PC to gateway */
		PC_msg_initg(&pctogateway);	
	}
	return p;
}
/* **************************************************************************************
 * static struct CANRCVBUF* msg_get(void);
 * @brief	: Check and add incoming msgs to MC circular buff
 * @return	: pointer to msg, or NULL if no msgs buffered.
 * ************************************************************************************** */
static u32 U;
static struct CANRCVBUF* msg_get(void)
{
	struct CANRCVBUF* p;
	int tmp;

	/* Add all incoming msgs to MC circular buffer */

#ifdef GATEWAYLOCAL
	/* USB -> MC+CAN */
	while ( (p=msg_get_usb()) != 0)
	{
		canmcbuf_add(p); // Add to MC buffer
		tmp = CAN_gateway_send(&canrcvbuf); // Add to CAN xmit buffer
	}
#endif

	/* CAN -> MC+USB */
	while ( (p=msg_get_can()) != 0) // CAN msg ready?
	{ // Here, yes.
#ifdef GATEWAYLOCAL
		msg_out_usb(p);	// Send incoming CAN msg to USB
#endif	
		canmcbuf_add(p);
	}

	/* Get next buffered msg */
	if (canmcidxm == canmcidxi) return NULL; // Return NULL if MC buffer empty
	p = &canmcbuf[canmcidxm];		// Pointer to msg
	canmcidxm = incIdx(canmcidxm,CANMCBUFSIZE); // Advance index
	return p;
}

/*#################################################################################################
And now for the main routine 
  #################################################################################################*/
int main(void)
{
	struct CANRCVBUF* pmc;

	// initialize
	initMasterController();
volatile int tt;
for (tt = 0; tt < 1000000; tt++);

/* --------------------- Endless Polling Loop ----------------------------------------------- */
	while (1==1)
	{
		ledHeartbeat();	// Flash Orange LED to show loop running

		/* Set 'timerMsgFlag' to 1 for 1/64th tick, to 2 for even second tick */
		timeKeeper();	// 

		/* Check for incoming CAN format msgs.  */
		while ((pmc = msg_get()) != NULL)	// Any buffered?
		{ // Here yes.  pmc points to msg struct
			// TODO select msgs of interest here 
xprintf(UXPRT,"U %d %08X\n\r",U++, pmc->id );
		}

		/* Run state machine */
		stateMachine();
		desiredTensionSpeed();
		controlLaw();

		/* Update SPI - led output & switch input */
		spiInOut();

		/* Output data to LCD */
		lcdOut();	// Output LCD (paced)
	}
	return 0;	
}


