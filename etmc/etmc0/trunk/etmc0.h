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

struct ETMCVAR
{
	char spi_ledout[SPI2SIZE];
	char spi_swin[SPI2SIZE];
	u8 fracTime; 	
	u32 unixtime;
	//	not used but referenced in timekeeper
	u8 count64;	
	int timeCount;
};
	

#endif 

