/******************************************************************************
* File Name          : etmc0.h
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : CAN format msgs for master controller
*******************************************************************************/

#ifndef __ETMC0
#define __ETMC0

#include "common_misc.h"
#include "common_can.h"

/* The following includes code a gateway. */
// NOTE: "USB" refers to the serial port for a gateway
#define GATEWAYLOCAL

#define SPI2SIZE	2

// SPI_LED assignements (these should become mc system parameters in rewrite)
#define LED_SAFE        0x8000
#define LED_PREP        0x4000
#define LED_ARM         0x2000
#define LED_GNDRLRTN    0x1000
#define LED_RAMP        0x0800
#define LED_CLIMB       0x0400
#define LED_RECOVERY    0x0200
#define LED_RETRIEVE    0x0100
#define LED_STOP        0x0080
#define LED_ABORT       0x0040

#define LED_PREP_PB     0x0002
#define LED_ARM_PB      0x0001


//	control panel switch mapping
#define SW_SAFE   1 << 15	//	active low
#define SW_ACTIVE 1 << 14	//	active low
#define PB_ARM    1 << 13	//	active low
#define PB_PREP   1 << 12	//	active low
#define CL_RST_N0 1 << 11	//	low at rest
#define CL_FS_ NO 1 << 8	// 	low at full scale


struct ETMCVAR
{
	char spi_ledout[SPI2SIZE];
	u32 cp_ledout;				// concatenated spi_ledout
	char spi_swin[SPI2SIZE];
	u32 cp_swin;				//	concatenated spi_swin
	float cp_cl;				//	working control lever value
	u8 fracTime; 	
	u32 unixtime;
	int ledBlink;
	//	beep variables
	int beep_count;
	int beep_state;
	unsigned int beep_time;
	//	not used but referenced in timekeeper
	u8 count64;	
	int timeCount;
};
	

#endif 

