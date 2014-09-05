/******************************************************************************
* File Name          : init_hardware_mc.h
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Basic hardware initialization for master controller
*******************************************************************************/

#ifndef __INIT_HW_MC
#define __INIT_HW_MC

#define George 1

#include "common_misc.h"
#include "common_can.h"

/* USART|UART assignment for xprintf and read/write */
#define UARTGATE 2  // UART number for Gateway (possibly Host later)
#if George
	#define UARTLCD	6	// Uart number for LCD messages
	#define UARTGPS	3	// Uart number for GPS or debug
#else
	#define UARTLCD	3 //6	// Uart number for LCD messages
	#define UARTGPS	6 //3	// Uart number for GPS or debug
#endif

#define UXPRT UARTGPS	//	Debugging port

/* *********************************************************************************************************** */
void init_hardware_mc (void);
/* @brief	:Setup & initialization functions 
 ************************************************************************************************************* */

#endif 

