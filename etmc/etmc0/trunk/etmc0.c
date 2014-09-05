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

#include "etmc0.h"
#include "mc_state.h"

/* --------------- For debugging...(usb) ------------------------------ */
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



// State machine
int currentState = 0;
int nextState = 0;

// led timer
u32	t_led;
#define FLASHCOUNT (sysclk_freq/2);	// Orange LED flash increment

// 64th second counter
u32 t_timeKeeper;
#define SIXTYFOURTH (sysclk_freq/64);
u8 count64 = 0;

struct ETMCVAR etmcvar;

u32	t_spi;

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
				etmcvar.unixtime++;
				count64 = 0;
				can.dlc      = 4;
				can.cd.us[0] = etmcvar.unixtime;
//xprintf(UXPRT,"T %d\n\r",etmcvar.unixtime);

			} else {
				can.dlc      = 1;
				can.cd.us[0] = count64;
			}
			msg_out_mc(&can); // Output to CAN+USB
			etmcvar.timeFlag = 1;
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

	mc_state_init(&etmcvar);
//	calib_control_lever();
	/* --------------------- Initial times ---------------------------------------------------------------------------- */
		t_led        = DTWTIME + FLASHCOUNT; 
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
	if (++t_ctr > 4) {
		t_ctr = 0;
		t_max = 0;	// Reset max
	}
	t_loop9 = DTWTIME + 168000000; // Once per sec
}
t_loop0 = DTWTIME;

		ledHeartbeat();	// Flash Orange LED to show loop running

		timeKeeper();	// 

		/* Check for incoming CAN format msgs.  */
		while ((pmc = msg_get()) != NULL)	// Any buffered?
		{ // Here yes.  pmc points to msg struct
			mc_state_msg_select(pmc);	// Select msgs needed for MC
//xprintf(UXPRT,"U %d %08X\n\r",U++, pmc->id );
		}

		/* Run state machine */
		stateMachine(&etmcvar);

		/* Update SPI - led output & switch input */
		spiInOut();

		/* Output data to LCD periodically */
		mc_state_lcd_poll(&etmcvar);	// Output LCD (paced)
	}
	return 0;	
}


