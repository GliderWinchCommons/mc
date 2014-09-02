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


int main(void)
{
// Debug stuff
//unsigned int U; // debug
unsigned int t_loop0;
unsigned int t_loop9;
unsigned int t_diff;
unsigned int t_max = 0;

	struct CANRCVBUF* pmc;	// Incoming msg pointer

	// initialize
	init_hardware_mc();	
//	calib_control_lever();
	/* --------------------- Initial times ---------------------------------------------------------------------------- */
		t_led        = DTWTIME + FLASHCOUNT; 
		t_lcd        = DTWTIME + LCDPACE;
		t_timeKeeper = DTWTIME + SIXTYFOURTH;

t_loop0 = DTWTIME;
t_loop9 = DTWTIME + 168000000; // 1 sec time
/* --------------------- Endless Polling Loop ----------------------------------------------- */
	while (1==1)
	{
// Debug - check loop time
t_diff = DTWTIME - t_loop0;
if (t_diff > t_max) t_max = t_diff; // Save max

if (((int)(DTWTIME - t_loop9)) > 0 ) // Periodic display
{
	xprintf(UXPRT,"%d\n\r",t_max);
//	t_max = 0;	// Reset max
	t_loop9 = DTWTIME + 168000000; // Once per sec
}
t_loop0 = DTWTIME;

		ledHeartbeat();	// Flash Orange LED to show loop running

		/* Set 'timerMsgFlag' to 1 for 1/64th tick, to 2 for even second tick */
		timeKeeper();	// 

		/* Check for incoming CAN format msgs.  */
		while ((pmc = msg_get()) != NULL)	// Any buffered?
		{ // Here yes.  pmc points to msg struct
			// TODO select msgs of interest here 
//xprintf(UXPRT,"U %d %08X\n\r",U++, pmc->id );
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


