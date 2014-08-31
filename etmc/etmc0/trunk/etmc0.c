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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>

#include "xprintf.h"
#include "libopencm3/stm32/f4/adc.h"
#include "adc_mc.h"						

//#include "libopencm3/stm32/f4/rcc.h"	//	!!!	May not be required
#include "libopencm3/stm32/f4/gpio.h"
#include "libopencm3/stm32/f4/scb.h"



#include "canwinch_ldr.h"
#include "common_can.h"
#include "panic_leds.h"
#include "PC_gateway_comm.h"
#include "USB_PC_gateway.h"
#include "CAN_gateway.h"
#include "bsp_uart.h"
#include "4x20lcd.h"
#include "libopencm3/stm32/systick.h"	//	!!!	May not be required
#include "clockspecifysetup.h"
#include "spi2rw.h"
#include "mccp.h"
#include "common_canid_et.h"
#include "mc_msgs.h"
#include "init_hardware_mc.h"
#include "calib_control_lever.h"
#include "beep_n_lcd.h"



void delay_tenth_sec(unsigned int t);
void calib_control_lever(void);




// State machine
int currentState = 0;
int nextState = 0;

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




/* ============================= main loop functions ============================================================ */

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
 * void timeKeeper(void);
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
xprintf(UXPRT,"State: %4d Torq: %d Time: %d\n\r",currentState,outputTorquei,(int) currentTime);
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

/*#################################################################################################
And now for the main routine 
  #################################################################################################*/

unsigned int U; // debug

int main(void)
{
	struct CANRCVBUF* pmc;

	// initialize
	init_hardware_mc();
//	init_control_lever();
	/* --------------------- Initial times ---------------------------------------------------------------------------- */
		t_led        = DTWTIME + FLASHCOUNT; 
		t_lcd        = DTWTIME + LCDPACE;
		t_timeKeeper = DTWTIME + SIXTYFOURTH;
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


