/******************************************************************************
* File Name          : calib_control_lever.c
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Master Controller: Control Lever calibration
*******************************************************************************/

#include "calib_control_lever.h"
#include "etmc0.h"
#include "mc_msgs.h"
#include "adc_mc.h"
#include "libopencm3/stm32/f4/gpio.h"
#include "bsp_uart.h"
#include "init_hardware_mc.h"
#include "clockspecifysetup.h"
#include "xprintf.h"
#include <string.h>
#include <stdio.h>
#include "spi2rw.h"
#include "4x20lcd.h"
#include "beep_n_lcd.h"
#include "etmc0.h"



/* ***********************************************************************************************************
 * void calib_control_lever(void);
 * @brief	:Setup & initialization functions 
 ************************************************************************************************************* */

//	code for calibrating scale and offset for the control lever
//	make function later
//#define FSCL	((1 << 12) - 1)	// full scale control lever (CL) output
#define CLREST (1 << 11) 	// SPI bit position for CL rest position switch
#define CLFS  (1 << 8) 		// SPI bit position for CL full scale position
#define CL_ADC_CHANNEL 	0
#define FLASHCOUNT (sysclk_freq/8);	// Orange LED flash increment

// Min and maximum values observed for control lever
static int cloffset = 0; 
static int clmax = 0;	
static float fpclscale;		//	CL conversion scale factor 

void calib_control_lever(struct ETMCVAR* petmcvar)
{
	int clcalstate = 0;		// state for control lever intial calibration
	int sw = 0;			// binary for holding switch values
	int adc_tmp;
	unsigned int t_led = DTWTIME + FLASHCOUNT;	//	initial t_led
	char vv[128];

	xprintf (UXPRT,"\nBegin control lever calibration\n\r");
return;
	// dummy read of SPI switches to deal with false 0000 initially returned
	spi2_rw(petmcvar->spi_ledout, petmcvar->spi_swin, SPI2SIZE);
	while(clcalstate < 6)
	{
		if (((int)(DTWTIME - t_led)) > 0) // Has the time expired?
		{ //	Time expired
			//	read filtered control lever adc last value and update min and max values
			adc_tmp = adc_last_filtered[CL_ADC_CHANNEL];
			cloffset = (cloffset < adc_tmp) ? cloffset : adc_tmp;
			clmax = (clmax > adc_tmp) ? clmax : adc_tmp;
			//	Read SPI switches
			//	Not sure why 
			if (spi2_busy() != 0) // Is SPI2 busy?
			{ // SPI completed  
				spi2_rw(petmcvar->spi_ledout, petmcvar->spi_swin, SPI2SIZE); // Send/rcv SPI2SIZE bytes
				//	convert to a binary word for comparisons (not general for different SPI2SIZE)
				sw = (((int) petmcvar->spi_swin[0]) << 8) | (int) petmcvar->spi_swin[1];
				xprintf(UXPRT, "%5u %8x \n\r", clcalstate, sw);				
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
						if (sw & CLFS) break ;
						clcalstate = 3;
						sprintf(vv, "twice: 0.5");
						lcd_printToLine(UARTLCD, 1, vv);
						break;						
					}
					case 3:	//	wating for return to rest first time
					{
						if (sw & CLREST) break; 
						// clcalstate = 4;
						clcalstate = 6;		//	only requires 1 cycle
						sprintf(vv, "twice: 1  ");
						lcd_printToLine(UARTLCD, 1, vv);
						single_beep();
						break;
					}
					case 4:	//	waiting for full scale second time
					{
						if (sw & CLFS) break;
						clcalstate = 5;
						sprintf(vv, "twice: 1.5");
						lcd_printToLine(UARTLCD, 1, vv);
						break;					
					}
					case 5:	//	waiting for return to rest second time
					{
						if (sw & CLREST) break;
						fpclscale = 1.0 / (clmax - cloffset);
						single_beep();
						clcalstate = 6; 
					}
				}			
			}
			toggle_4leds(); 	// Advance some LED pattern
			t_led += FLASHCOUNT; 	// Set next toggle time		
		}
	}	
	lcd_clear(UARTLCD);
	xprintf(UXPRT, "  cloffset: %10d clmax: %10d \n\r", cloffset, clmax);
	xprintf 	(UXPRT, "   Control Lever Initial Calibration Complete\n\r");
	return;
}
/* ***********************************************************************************************************
 * int calib_control_lever_get(void);
 * @brief	:
 * @return	: Calibrated Control lever: 0 - 4095 (but could be slightly negative) -> 0 - 100%
 ************************************************************************************************************* */
float calib_control_lever_get(void)
{
	float x;
	x = (adc_last_filtered[CL_ADC_CHANNEL] - cloffset) * fpclscale;
	/* for now do not limit
	x = x > 1.0 ? 1.0 : x;
	x = x < 0.0 ? 0.0 : x;
	*/
	return x;
}


