/******************************************************************************
* File Name          : common_canid_et.h
* Date First Issued  : 08/30/2014
* Board              : 
* Description        : CAN ID assignments for ET
*******************************************************************************/


#ifndef __COMMON_CANID_ET
#define __COMMON_CANID_ET


#define CANID_TIME				0x20000000	// 256 
//#define CANID_MOTOR_SPEED		0x25000000	// 296	inconsistent with Excel 
#define CANID_MOTOR_SPEED		0x24800000	// 292	
#define CANID_TORQUE			0x25800000	// 300 
#define CANID_STATE				0x26000000	// 304 
#define CANID_PARAM_REQUEST		0x27000000	// 312
//#define CANID_LAUNCH_PARAM		0x28E00000	// 327 	inconsistent with Excel
#define CANID_LAUNCH_PARAM		0x28000000	// 320
#define CANID_CP_CL_RMT			0x29000000	// 328	
#define CANID_CP_CL_LCL			0x29200000	// 329		 
#define CANID_CP_INPUTS_RMT		0x29400000	// 330
#define CANID_CP_INPUTS_LCL		0x29600000	// 331
#define CANID_CP_OUTPUTS		0x2A000000	// 336
#define CANID_CP_LCD			0x2A200000	// 337	 

#define CANID_TENSION			0x38000000	// 448 	
#define CANID_CABLE_ANGLE		0x3A000000	// 464 

#endif 

