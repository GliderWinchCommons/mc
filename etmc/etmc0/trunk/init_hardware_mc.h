/******************************************************************************
* File Name          : init_hardware_mc.h
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : Basic hardware initialization for master controller
*******************************************************************************/

#ifndef __INIT_HW_MC
#define __INIT_HW_MC

#include "common_misc.h"
#include "common_can.h"

/* USART|UART assignment for xprintf and read/write */
#define UARTLCD	3 //6	// Uart number for LCD messages
#define UARTGPS	6 //3	// Uart number for GPS or debug
#define UARTGATE 2  // UART number for Gateway (possibly Host later)

#define UXPRT UARTGPS	//	Debugging port

#define DTWTIME	(*(volatile unsigned int *)0xE0001004)	// Read DTW 32b system tick counter

/* *********************************************************************************************************** */
void init_hardware_mc (void);
/* @brief	:Setup & initialization functions 
 ************************************************************************************************************* */

#endif 

