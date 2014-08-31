/******************************************************************************
* File Name          : calib_control_lever.c
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Master Controller: Control Lever calibration
*******************************************************************************/

#include "calib_control_lever.h"
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

extern char spi_ledout[SPI2SIZE];
extern char spi_swin[SPI2SIZE];

/* ************************************************************
Turn the LEDs on in sequence, then turn them back off 
***************************************************************/
static int lednum = 12;	// Lowest port bit numbered LED
static void toggle_4leds (void)
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
#define FLASHCOUNT (sysclk_freq/2);	// Orange LED flash increment

int cal_cl;			// calibrated control lever output
int cloffset = 0, clmax = 0;	// Min and maximum values observed for control lever
int clscale = 0;		// scale value for generating calibrated output

void calib_control_lever(void)
{
	int clcalstate = 0;		// state for control lever intial calibration
	int sw = 0;			// binary for holding switch values
	int adc_tmp;
	unsigned int t_led = DTWTIME + FLASHCOUNT;	//	initial t_led
	char vv[128];

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
