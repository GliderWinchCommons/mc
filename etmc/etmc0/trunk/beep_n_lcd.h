/******************************************************************************
* File Name          : beep_n_lcd.h
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Master Controller: simple beeping, lcd alert routines
*******************************************************************************/

#ifndef __BEEP_N_LCD
#define __BEEP_N_LCD

#include "common_misc.h"
#include "common_can.h"

/* *********************************************************** */
void delay_tenth_sec(unsigned int t);
/* @brief	: Looping delay using DTW counter
 * @param	: t = number of 1/10th secs to delay
************************************************************** */
void single_beep(void);	// One beep
void double_beep(void);	// Two beeps
void triple_beep(void);	// Three beeps
/* *********************************************************** */
void show_op_the_error(char* p, int e, unsigned int t);
/* @brief	: Show Op the error, beep, and pause
 * @param	: p = pointer to string that goes on line 0 of LCD
 * @param	: e = error code
 * @param	: t = number of 1/10th secs to pause before continuing.
************************************************************** */
void toggle_4leds (void);
/* @brief	: Turn the LEDs on in sequence, then turn them back off 
************************************************************** */
void toggle_led (int lnum);
/* @brief	: Toggle one LED
  * @param	: lnum = led number (pin number)
************************************************************** */




#endif 

