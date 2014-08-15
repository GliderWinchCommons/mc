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
#include "libopencm3/stm32/f4/usart.h"	//	!!! May not be required
#include "default_irq_handler.h"

#include "libopencm3/stm32/f4/adc.h"
#include "adc_mc.h"						//	!!! May not be required

#include "libopencm3/stm32/f4/rcc.h"	//	!!!	May not be required
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


static void canbuf_add(struct CANRCVBUF* p);

void sprintbits(char* c, char* p);

/* USART|UART assignment for xprintf and read/write */
#define UARTLCD	6	// Uart number for LCD messages
#define UARTGPS	3	// Uart number for GPS or debug
#define UARTGATE 2  // UART number for Gateway (possibly Host later)

#define UXPRT UARTGPS	//	Debugging port

/*	LCD Line Size  */
#define LCDLINESIZE 20

// Number of bytes in the SPI transfer	
#define SPI2SIZE 2 	


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
static struct CANRCVBUF canrcvbuf;


/* Sequence number checking for incoming msgs from the PC */
//static u32 seqnum;
//static u32 seqnum_old = 0;



/* Circular buffer for passing CAN BUS msgs to PC */
#define CANBUSBUFSIZE	64			// Number of incoming CAN msgs to buffer
static struct CANRCVBUF canbuf[CANBUSBUFSIZE];
static u32 canmsgct[CANBUSBUFSIZE]; // Msg seq number for CAN-to-PC.
static int canbufidxi = 0;			// Incoming index into canbuf
static int canbufidxm = 0;			// Outgoing index into canbuf

static int incIdx(int x){x += 1; if (x >= CANBUSBUFSIZE) x = 0; return x;} 

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
#define FLASHCOUNT (168000000/8);	// LED flash

// 64th second counter
u32 t_timeKeeper;
#define SIXTYFOURTH (168000000/64);
u8 count64 = 0;
u32 currentTime = 0;
u8 timerMsgFlag = 0;

// lcd
u32 t_lcd;
#define LCDPACE (168000000/10);

char lcdLine0[LCDLINESIZE + 1];
char lcdLine1[LCDLINESIZE + 1];
char lcdLine2[LCDLINESIZE + 1];
char lcdLine3[LCDLINESIZE + 1];

// SPI globals
#define SPIPACE (168000000/200);	// Pace the output loop
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
	{ // HEre, LED bit was on
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


/* Setup & initialization functions */ 
void initMasterController () {
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
		bsp_uart_int_init_number(UARTGATE, 115200, 256, 256, 0x40);	// UART used for the Gateway
		bsp_uart_int_init_number(UXPRT, 115200, 256, 256, 0x40);	// UART used for debugging		
		lcd_init(UARTLCD); 											// UART used for the LCD screen
										
	
	/* ---------------------- DTW sys counter -------------------------------------------------------- */
		// Use DTW_CYCCNT counter (driven by sysclk) for polling type timing 
		// CYCCNT counter is in the Cortex-M-series core.  See the following for details 
		// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337g/BABJFFGJ.html 
		*(volatile unsigned int*)0xE000EDFC |= 0x01000000; // SCB_DEMCR = 0x01000000;
		*(volatile unsigned int*)0xE0001000 |= 0x1;	// Enable DTW_CYCCNT (Data Watch cycle counter)

	/* ---------------------- Let the Op know it is alive ------------------------------------ */
	/* Announce who we are. ('xprintf' uses uart number to deliver the output.) */		
	xprintf(UXPRT,  " \n\rDISCOVERY F4 Consolidated Tests: 08/10/2014  v0\n\r");
	/* Make sure we have the correct bus frequencies */
	xprintf (UXPRT, "   hclk_freq (MHz) : %9u...............................\n\r",  hclk_freq/1000000);	
	xprintf (UXPRT, "  pclk1_freq (MHz) : %9u...............................\n\r", pclk1_freq/1000000);	
	xprintf (UXPRT, "  pclk2_freq (MHz) : %9u...............................\n\r", pclk2_freq/1000000);	
	xprintf (UXPRT, " sysclk_freq (MHz) : %9u...............................\n\r",sysclk_freq/1000000);

	/* --------------------- Initialize SPI2 ------------------------------------------------------------------------------- */
	char bout[SPI2SIZE] = {0x55,0xAA};	// Initial outgoing pattern {0x00, 0x00}
	char bin[SPI2SIZE];
	spi2rw_init();
	xprintf (UXPRT, "   SPI Init Complete\n\r");

	/* --------------------- ADC initialization ---------------------------------------------------------------------------- */
	int i = adc_mc_init_sequence();
	if (i < 0)
	{
		xprintf (UXPRT, "ADC init failed with code: %i\n\r", i);	
	}
	xprintf (UXPRT, "   ADC Init Complete\n\r");

	

	/* Setup STDOUT, STDIN (a shameful sequence until we sort out 'newlib' and 'fopen'.)  The following 'open' sets up 
	   the USART/UART that will be used as STDOUT_FILENO, and STDIN_FILENO.  Don't call 'open' again!  */
	fd = open("tty2", 0,0); // This sets up the uart control block pointer versus file descriptor ('fd')


//	code for calibrating scale and offset for the control lever
//	make function later

void single_beep(void)
{
//	Place holder for single CP beep
}

void double_beep(void)
{
//	Place holder for double CP beep
}

#define FSCL	((1 << 12) - 1)	//	full scale control lever (CL) output
#define CLREST (1 << 12) 		//	SPI bit position for CL rest position switch
#define CLFS  (1 << 15) 		//	SPI bit position for CL full scale position
#define CL_ADC_CHANNEL 	0

int cal_cl;						//	calibrated control lever output
int cloffset = 0, clmax = 0;	//	Min and maximum values observed for control lever
int clscale = 0;					// scale value for generating calibrated output
int clcalstate = 0;				//	state for control lever intial calibration
int adc_tmp;
int sw = 0;						//	binary for holding switch values

t_led = *(volatile unsigned int *)0xE0001004 + FLASHCOUNT;	//	initial t_led

while(clcalstate < 6)
{
	if (((int)(*(volatile unsigned int *)0xE0001004 - t_led)) > 0) // Has the time expired?
	{ //	Time expired
		xprintf(UXPRT, "%5u %8x \n\r", clcalstate, sw);
		//	read filtered control lever adc last value and update min and max values
		adc_tmp = adc_last_filtered[CL_ADC_CHANNEL];
		cloffset = (cloffset < adc_tmp) ? cloffset : adc_tmp;
		clmax = (clmax > adc_tmp) ? clmax : adc_tmp;
		//	Read SPI switches
		if (spi2_busy() != 0) // Is SPI2 busy?
		{ // SPI completed  
			spi2_rw(bout, bin, SPI2SIZE); // Send/rcv SPI2SIZE bytes
			sw = (((int) bin[0]) << 8) | (int) bin[1];				
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
		toggle_4leds(); 		// Advance some LED pattern
		t_led += FLASHCOUNT; 	// Set next toggle time		
	}
}
xprintf (UXPRT, "   Control Lever Initial Calibration Complete\n\r");


xprintf(UXPRT, "%10d %10d %10d \n\r", cloffset, clmax, clscale);
//while (1 == 1)
//{}


/*	Temporary code for testing UARTs, LCD, ADCs, SPI2*/
int count = 0;

extern u32 cic_debug0;	// counter in adc_mc.c
u32 cic_debug0_prev = 0; // Used to take difference between new and "previous" counts



void printbits(char* p);


extern int spidebug1;

char cin[21] = "Sw: ";

while (1 == 1) 
{if (((int)(*(volatile unsigned int *)0xE0001004 - t_led)) > 0) // Has the time expired?
		{ //	Time exprired

			sprintf(vv, "Test Count: %6d", count);
			lcd_printToLine(UARTLCD, 0, vv);
			printf("%s\n\r", vv);

			//	ADC
			cal_cl = ((adc_last_filtered[CL_ADC_CHANNEL] - cloffset) * clscale) >> 16;
			sprintf(vv, "CL: %5d  %5d", (int) adc_last_filtered[CL_ADC_CHANNEL], cal_cl);
			lcd_printToLine(UARTLCD, 1, vv);
			xprintf(UXPRT, "%5d  %5d: ", (cic_debug0 - cic_debug0_prev), count); // Sequence number, number of filtered readings between xprintf's
			cic_debug0_prev = cic_debug0;

			for (i = 0; i < NUMBERADCCHANNELS_MC; i++)	// Loop through the three ADC channels
				xprintf(UXPRT,"%5d ", adc_last_filtered[i]);	// ADC filtered and scaled reading

			//	SPI 
			
			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // SPI completed  
				spi2_rw(bout, bin, SPI2SIZE); // Send/rcv SPI2SIZE bytes
				bout[0] ^=  0xff;
				bout[1] ^=  0xff;
				sprintbits(&cin[4], bin);

			}
			lcd_printToLine(UARTLCD, 2, cin);

			xprintf(UXPRT,"%5u ", spidebug1);
			printbits(bin); // Print the bits			

			sprintf(vv, "Ln 4; Count: %5d", count);
			lcd_printToLine(UARTLCD, 3, vv);	

			count++;

			toggle_4leds(); 	// Advance some LED pattern
			t_led += FLASHCOUNT; 	// Set next toggle time
		}

}

/*	!!!	Should this be moved to init function once it is needed?  Looks like several things below here are already
done 	*/

	/* --------------------- CAN setup ------------------------------------------------------------------- */
		/*  Pin usage for CAN--
		PD00 CAN1  Rx LQFP 81 Header P2|36 BLU
		PD01 CAN1  Tx LQFP 82 Header P2|33 WHT
		PC04 GPIIO RS LQFP 33 Header P1|20 GRN
		*/
		/* Configure CAN driver RS pin: PC4 LQFP 33, Header P1|20, fo hi speed. */
		can_nxp_setRS_ldr(0,(volatile u32 *)GPIOC, 4); // (1st arg) 0 = high speed mode; not-zero = standby mode

		/* Setup CAN registers and initialize routine */
		init_ret = can_init_pod_ldr((struct CAN_PARAMS*)&can_params); // 'struct' that holds all the parameters

		/* Check if initialization was successful, or timed out. */
		if (init_ret <= 0)
		{ // Here the init returned an error code
			// xprintf(UARTLCD, "###### can init failed: code = %d\n\r",init_ret); 
			panic_leds(6);	while (1==1);	// Flash panic display with code 6
		}
		// xprintf (UARTLCD, "\n\rcan ret ct: %d..............................................\n\r",init_ret); // Just a check for how long "exit initialization" took

		/* Set filters to respond "this" unit number and time sync broadcasts */
		can_filter_unitid_ldr(can_params.iamunitnumber);	// Setup msg filter banks

		// xprintf (UARTLCD, " IAMUNITNUMBER %0x %0x.....................................\n\r",(unsigned int)IAMUNITNUMBER,(unsigned int)CAN_UNITID_SE1 >> CAN_UNITID_SHIFT); 

		/* Since this is a gateway set the filter for the hardware to accept all msgs. */
		int can_ret = can_filtermask16_add_ldr( 0 );	// Allow all msgs
		/* Check if filter initialization was successful, or timed out. */
		if (can_ret < 0)
		{
			// xprintf(UARTLCD, "###### can_filtermask16_add failed: code = %d\n\r",can_ret);
			panic_leds(7);	while (1==1);	// Flash panic display with code 7
		}
	/* --------------------- Hardware is ready, so do program-specific startup ---------------------------- */
		t_led = *(volatile unsigned int *)0xE0001004 + FLASHCOUNT; // Set initial time

		PC_msg_initg(&pctogateway);	// Initialize struct for CAN message from PC
		PC_msg_initg(&gatewayToPC);	// Initialize struct for CAN message from PC

		/* Set modes for routines that receive and send CAN msgs */
		pctogateway.mode_link = MODE_LINK;
		gatewayToPC.mode_link = MODE_LINK;
	
	/* --------------------- LCD ---------------------------------------------------------------------------- */
		t_lcd = *(volatile unsigned int *)0xE0001004 + LCDPACE;
}

// main loop functions
	/* Flash the red LED to amuse the hapless Op or signal the wizard programmer that the loop is running. */
	void ledHeartbeat () {
		if (((int)(*(volatile unsigned int *)0xE0001004 - t_led)) > 0) // Has the time expired?
		{ // Here, yes.
			t_led += FLASHCOUNT; 	// Set next toggle time

			toggle_led(14); 	// Advance some LED pattern
		}
	}

	/* function to find the 64th second beats */
	void timeKeeper () {
		if (((int)(*(volatile unsigned int *)0xE0001004 - t_timeKeeper)) > 0) // Has the time expired?
		{ // Here, yes.
			t_timeKeeper += SIXTYFOURTH; 	// Set next toggle time

			count64++;

			if(count64 == 64) {
				currentTime++;
				count64 = 0;
				timerMsgFlag = 2; // send 1 sec message
			} else {
				timerMsgFlag = 1; // send 1/64th sec message
			}
		}
	}

	/* spi i/o */
	void spiInOut () {
		if (((int)(*(volatile unsigned int *)0xE0001004 - t_spi)) > 0) {
			t_spi += SPIPACE;

			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // Here, no.
				spi2_rw(spi_ledout, spi_swin, SPI2SIZE); // Send/rcv three bytes
			}
		}
	}

	/* LCD output routine */
	void lcdOut () {
		if (((int)(*(volatile unsigned int *)0xE0001004 - t_lcd)) > 0) {
			t_lcd += LCDPACE;

			snprintf(lcdLine0, 20, "%16s%4d", "Current State:", currentState);
			snprintf(lcdLine1, 20, "outputTorque: %20f", outputTorque);
			snprintf(lcdLine2, 20, "Time: %20d", (int) currentTime);

			// padString(' ', lcdLine0, 20);
			// padString(' ', lcdLine1, 20);
			// padString(' ', lcdLine2, 20);
			// padString(' ', lcdLine3, 20);

			// display a char on the lcd
			lcd_printToLine(UARTLCD, 0, lcdLine0);
			// lcd_printToLine(UARTLCD, 1, lcdLine1);
			lcd_printToLine(UARTLCD, 2, lcdLine2);
			// lcd_printToLine(UARTLCD, 3, lcdLine3);
		}
	}

	void stateZero () {
		// TODO: check for conditions that progress the state
		if (currentTime > 60) {
			nextState = 1;
		}
	}

	void stateOne () {

	}

	/* State Machine */
	void stateMachine () {
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

	void desiredTensionSpeed () {
		desiredSpeed = 1;
		desiredTension = 2;
	}

	void controlLaw () {
		if (currentState == 0) {
			outputTorque = 0.5 * desiredSpeed;
		} else {
			outputTorque = 0.5 * desiredTension;
		}
	}

	/* ================ PC --> CAN ================================================================= */
	// messages here are from the pc. We need to parse and forward to the CANbus
	void usbInput () {
		int tmp;
		int temp;

		temp=USB_PC_get_msg_mode(STDIN_FILENO, &gatewayToPC, &canrcvbuf);	// Check if msg is ready
		if (temp != 0)	// Do we have completion of a msg?
		{ // Here, yes.  We have a msg, but it might not be valid.
			if ( temp == 1 ) // Was valid?
			{ // Here, yes.
				tmp = temp >> 16; // Isolate compression error codes
				if (tmp < 0)	// Did the compression detect some anomolies?
				{ // Here, something wrong with the msg--

				}
				else
				{ // Here, msg is OK msg from the PC

					// TODO: filter message id's that require the MC to do something

					// Take the message and send it over CAN
					tmp = CAN_gateway_send(&canrcvbuf);	// Add to xmit buffer (if OK)
					Errors_CAN_gateway_send(tmp);		// Count any error returns					
					PC_msg_initg(&gatewayToPC);	// Initialize struct for next msg from PC to gateway
				}
			}
			else
			{ // Something wrong with the msg.  Count the various types of error returns from 'USB_PC_msg_getASCII'
				Errors_USB_PC_get_msg_mode(temp);
			} // Note: 'pctogateway' gets re-intialized in 'PC_msg_initg' when there are errors.
		}
	}

	/* ================= CAN --> PC ================================================================= */
	// messages here are from the CANbus. We need to parse and forward to the PC
	void canInput () {
		while ( (pfifo1 = canrcvtim_get_ldr()) != 0)	// Did we receive a HIGH PRIORITY CAN BUS msg?
		{ // Here yes.  Retrieve it from the CAN buffer and save it in our vast mainline storage buffer ;)
			canbuf_add(&pfifo1->R);	// Add msg to buffer

			// TODO: filter message id's that require the MC to do something
		}

		while ( (pfifo0 = canrcv_get_ldr()) != 0)		// Did we receive a LESS-THAN-HIGH-PRIORITY CAN BUS msg?
		{ // Here yes.  Retrieve it from the CAN buffer and save it in our vast mainline storage buffer.
			canbuf_add(pfifo0);	// Add msg to buffer

			// TODO: filter message id's that require the MC to do something
		}
	}

	void usbOutput () {
		/* Send buffered msgs to PC */
		while (canbufidxi != canbufidxm)	// Set up all the buffered msgs until we are caught up.				
		{ // Here, yes.  Set up a buffered msg from the CAN bus to go to the PC.
			pctogateway.cmprs.seq = canmsgct[canbufidxm];		// Add sequence number (for PC checking for missing msgs)
			USB_toPC_msg_mode(STDOUT_FILENO, &pctogateway, &canbuf[canbufidxm]); 	// Send to PC via STDOUT
			canbufidxm = incIdx(canbufidxm);			// Advance outgoing buffer index.
		}
	}

	// check flags and send out appropriate messages
	void canOutput () {
		struct CANRCVBUF can;
		int tmp;

		if (timerMsgFlag == 1) { // every 1/64 second
			can.id       = 0x20000000; // time id
			can.dlc      = 0x00000001;
			can.cd.us[0] = count64;

			tmp = CAN_gateway_send(&can);
			canbuf_add(&can);

			can.id       = 0x21400000; // torque
			can.dlc      = 0x00000002;
			can.cd.us[0] = 0x0001;

			tmp = CAN_gateway_send(&can);
			canbuf_add(&can);

			timerMsgFlag = 0;
		} else if (timerMsgFlag == 2) { // every second
			can.id       = 0x20000000;
			can.dlc      = 0x00000004;
			can.cd.us[0] = currentTime;

			tmp = CAN_gateway_send(&can);
			canbuf_add(&can);

			timerMsgFlag = 0;
		}
	}



/*#################################################################################################
And now for the main routine 
  #################################################################################################*/
int main(void)
{
	// initialize
	initMasterController();

/* --------------------- Endless Polling Loop ----------------------------------------------- */
	while (1==1)
	{
		ledHeartbeat();
		timeKeeper();
		
		// Gateway functionality (also catches messages for the MC)
		usbInput();
		canInput();

		stateMachine();
		desiredTensionSpeed();
		controlLaw();

		// Send any messages the MC needs over CAN
		canOutput();
		usbOutput();

		// SPI - led output & switch input
		spiInOut();
		lcdOut();
	}
	return 0;	
}




/* **************************************************************************************
 * static void canbuf_add(struct CANRCVBUF* p);
 * @brief	: Add msg to buffer
 * @param	: p = Pointer to CAN msg
 * ************************************************************************************** */
static void canbuf_add(struct CANRCVBUF* p)
{
	int temp;
	canbuf[canbufidxi] = *p;		// Copy struct
	canmsgct[canbufidxi] = canmsgctr;	// Save sequence count that goes with this msg
	canmsgctr += 1;				// Count incoming CAN msgs
	temp = incIdx(canbufidxi);		// Increment the index for incoming msgs.
	if (canbufidxm == temp)  		// Did this last fill the last one?
	{ // Yes, we have filled the buffer.  This CAN msg might be dropped (by not advancing the index)
		Errors_misc(-1);		// Add to buffer overrun counter
	}
	else
	{ // Here, there is room in the buffer and we are good to go.
		canbufidxi = temp;		// Update the index to next buffer position.
	}	
	return;
}

/******************************************************************************
 * Print out the bits for the array
 ******************************************************************************/
void printbits(char* p)
{
	//	Kludged function diplay switches on LCD
	int i,j;
	char c;

	/* Print out the bits for the incoming bytes */
	for (j = 0; j < SPI2SIZE; j++) // For each byte in the cycle
	{
		for (i = 0; i < 8; i++) // For each bit within a byte
		{
			if ( (*p & (1<<i)) == 0) 
				c = '.';	// Symbol for "zero"
			else
				c = '1';	// Symbol for "one"
			xprintf (UXPRT,"%c",c);
		}
		p++;
	}

	xprintf (UXPRT,"\n\r");
	return;
}

void sprintbits(char* c, char* p)
{
	int i, j;
	int k = 0;
	

	/* Print out the bits for the incoming bytes */
	for (j = 0; j < SPI2SIZE; j++) // For each byte in the cycle
	{
		for (i = 0; i < 8; i++) // For each bit within a byte
		{
			if ( (*p & (1<<i)) == 0) 
				c[k] = '.';	// Symbol for "zero"
			else
				c[k] = '1';	// Symbol for "one"
			k++;
		}
		p++;
		c[k] = 0;
	}
	return;
}
