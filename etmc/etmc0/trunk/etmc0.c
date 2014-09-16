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
#include "init_hardware_mc.h"
#include "calib_control_lever.h"
#include "beep_n_lcd.h"
#include "mc_state.h"
#include "mc_msgs.h"
#include "etmc0.h"
#include "CAN_error_msgs.h"


// State machine
int currentState = 0;
int nextState = 0;

// led timer
u32	t_led;
#define FLASHCOUNT (sysclk_freq/8);	// LED flash increment

// 64th second counter
u32 t_timeKeeper;
u8 count64;
#define SIXTYFOURTH (sysclk_freq/64);
static struct ETMCVAR etmcvar;

u32	t_spi;

/* ============================= main loop functions ============================================================ */

/* **************************************************************************************
 * void ledHeartbeat(void);
 * @brief	: Flash the red LED to amuse the hapless Op or signal the wizard programmer that the loop is running.
 * ************************************************************************************** */
	void ledHeartbeat (struct ETMCVAR* petmcvar) 
	{
		if (((int)(DTWTIME - t_led)) > 0) // Has the time expired?
		{ // Here, yes.
			t_led += FLASHCOUNT; 	// Set next toggle time
			petmcvar->ledBlink ^= 0xffff;
			toggle_4leds(); 	// Advance some LED pattern
		}
	}

/* **************************************************************************************
 * void timeKeeper(void);
 * @brief	: function to find the 64th second beats
 * ************************************************************************************** */
/*
void timeKeeper (void)
	{
		struct CANRCVBUF can;
		if (((int)(DTWTIME - t_timeKeeper)) > 0) // Has the time expired?
		{ // Here, yes.
			t_timeKeeper += SIXTYFOURTH; 	// Set next toggle time
			etmcvar.count64++;
			can.id       = CANID_TIME; // time id
			if(etmcvar.count64 == 64) {
				etmcvar.unixtime++;
				etmcvar.count64 = 0;
				can.dlc      = 4;
				can.cd.us[0] = etmcvar.unixtime;
//xprintf(UXPRT,"T %d\n\r",etmcvar.unixtime);

			} else {
				can.dlc      = 1;
				can.cd.us[0] = count64;
			}
			msg_out_mc(&can); // Output to CAN+USB
			etmcvar.timeCount = +1;
		}
	}
*/
/* **************************************************************************************
 * void spiInOut(void);
 * @brief	: SPI send/rcv & pacing 
 * ************************************************************************************** */
	void spiInOut (void) {
		if (((int)(DTWTIME - t_spi)) > 0) {
			t_spi += SPIPACE;	// (200 per sec)

			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // Here, no.
				spi2_rw(etmcvar.spi_ledout, etmcvar.spi_swin, SPI2SIZE); // Send/rcv three bytes
			}
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
	unsigned int t_ctr = 0;

	struct CANRCVBUF* pmc;	// Incoming msg pointer

	// initialize
	init_hardware_mc();
	
	/* Set a fixed unix time for now. */
	etmcvar.unixtime = 1409768561; // GMT: Wed, 03 Sep 2014 18:22:41 GMT	

	
	#if George
	calib_control_lever(&etmcvar);
	#endif

	mc_state_init(&etmcvar);

	/* --------------------- Initial times ---------------------------------------------------------------------------- */
	t_led        = DTWTIME + FLASHCOUNT; 
	t_timeKeeper = DTWTIME + SIXTYFOURTH;
	t_spi = DTWTIME + SPIPACE;
	
	t_loop0 = DTWTIME;
	t_loop9 = DTWTIME + sysclk_freq; 	// 1 sec time
/* --------------------- Endless Polling Loop ----------------------------------------------- */
	while (1==1)
	{
		// Debug - check loop time
		t_diff = DTWTIME - t_loop0;
		if (t_diff > t_max) t_max = t_diff; // Save max

		if (((int)(DTWTIME - t_loop9)) > 0 ) // Periodic display
		{
			xprintf(UXPRT,"%d\n\r", t_max);
			if (++t_ctr > 4) {
				t_ctr = 0;
				t_max = 0;	// Reset max
			}
			t_loop9 = DTWTIME + 168000000; // Once per sec
		}
		t_loop0 = DTWTIME;

		ledHeartbeat(&etmcvar);	// LED pattern to show processor is alive

		//	time keeping is temporarily being performed in mc_state.c
		//	timeKeeper();	// 

		/* Check for incoming CAN format msgs.  */
		if ((pmc = msg_get()) != NULL)	// Any buffered?
		{ 	// Here yes.  pmc points to msg struct
			mc_state_msg_select(pmc);	// Select msgs needed for MC
//xprintf(UXPRT,"U %d %08X\n\r",U++, pmc->id );
		}

		/* Run state machine */
		stateMachine(&etmcvar);

		/* Update SPI - led output & switch input */
		spiInOut();

		/* Output data to LCD periodically */
		mc_state_lcd_poll(&etmcvar);	// Output LCD (paced)

		/* Output msg error counter periodically */
		CAN_error_msg_poll(UXPRT);

	}
	return 0;	
}


