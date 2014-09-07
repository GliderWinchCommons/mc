/******************************************************************************
* File Name          : common_canid_et.h
* Date First Issued  : 08/30/2014
* Board              : 
* Description        : CAN ID assignments for ET
*******************************************************************************/


#ifndef __COMMON_CANID_ET
#define __COMMON_CANID_ET

#define CANIDSHIFT_21
#ifdef CANIDSHIFT_21

#define CANID_TIME		0x20000000	// 256 
#define CANID_STATE		0x26000000	// 304 
#define CANID_TORQUE		0x25800000	// 300 
#define CANID_MOTOR_SPEED	0x25000000	// 296 
#define CANID_CONTROL_LEVER	0x50200000	// 641 
#define CANID_TENSION		0x38000000	// 448 
#define CANID_CABLE_ANGLE	0x3A000000	// 464 
#define CANID_LAUNCH_PARAM	0x28E00000	// 327 
#define CANID_PARAM_REQUEST	0x27000000	// 312 

#else

/* CAN ID's shifted 20 */
#define CANID_TIME		0x10000000	// 256 
#define CANID_STATE		0x13000000	// 304 
#define CANID_TORQUE		0x12C00000	// 300 
#define CANID_MOTOR_SPEED	0x12800000	// 296 
#define CANID_CONTROL_LEVER	0x28100000	// 641 
#define CANID_TENSION		0x1C000000	// 448 
#define CANID_CABLE_ANGLE	0x1D000000	// 464 
#define CANID_LAUNCH_PARAM	0x14700000	// 327 
#define CANID_PARAM_REQUEST	0x13800000	// 312 

#endif

#endif 

