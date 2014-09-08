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
	u32 elapsedTics;	
	u32 unixtime;
	//	not used but referenced in timekeeper
	u8 count64;	
	int timeCount;
};
	

/* ************************************************************************************** */
void mc_msg_init(void);
/* @brief	: Initialization for msg handling
 * ************************************************************************************** */
struct CANRCVBUF* msg_get(void);
/* @brief	: Check and add incoming msgs to MC circular buff
 * @return	: pointer to msg, or NULL if no msgs buffered.
 * ************************************************************************************** */
void msg_out_mc(struct CANRCVBUF* p);
/* @brief	: Output msg from MC to CAN and USB
 * ************************************************************************************** */
void mc_state_lcd_poll(struct ETMCVAR* petmcvar);
/* @brief	: Output LCD data
 * @param	: petmcvar = pointer to variables in etmc0
 * ************************************************************************************** */

#endif 

