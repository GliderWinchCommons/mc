/******************************************************************************
* File Name          : mc_state.h
* Date First Issued  : 09/03/2014
* Board              : 
* Description        : ET state machine for etmc0.c
*******************************************************************************/
#ifndef __MC_STATE
#define __MC_STATE

#include "common_misc.h"
#include "common_can.h"

/* ************************************************************************************** */
void mc_state_init(struct ETMCVAR* petmcvar);
/* @brief	: Initialize structs for state machine
 * @param	: petmcvar = pointer to struct with variables passed from etmc0.c
 * ************************************************************************************** */
void mc_state_msg_select(struct CANRCVBUF* pcan, struct ETMCVAR* petmcvar);
/* @brief	: Select msgs of interest for MC out of incoming stream of msgs
 * @param	: pcan = pointer to can msg
 * ************************************************************************************** */
void stateMachine(struct ETMCVAR* petmcvar);
/* @brief	: Run state machine
 * @param	: petmcvar = pointer vars passed from etmc0
 * ************************************************************************************** */

#endif 

