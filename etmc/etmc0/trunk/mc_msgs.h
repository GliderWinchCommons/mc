/******************************************************************************
* File Name          : mc_msgs.h
* Date First Issued  : 08/31/2014
* Board              : DiscoveryF4
* Description        : CAN format msgs for master controller
*******************************************************************************/

#ifndef __CAN_MC_MSGS
#define __CAN_MC_MSGS

#include "common_misc.h"
#include "common_can.h"

/* The following includes code a gateway. */
// NOTE: "USB" refers to the serial port for a gateway
#define GATEWAYLOCAL	

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


#endif 

