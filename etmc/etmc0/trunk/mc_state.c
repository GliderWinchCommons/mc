/******************************************************************************
* File Name          : mc_state.c
* Date First Issued  : 09/03/2014
* Board              : 
* Description        : ET state machine for etmc0.c
*******************************************************************************/
#include "libopencm3/stm32/f4/gpio.h"
#include <stdint.h>

#include "etmc0.h"
#include "mc_msgs.h"
#include "common_canid_et.h"

#include "xprintf.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "init_hardware_mc.h"
#include "calib_control_lever.h"
#include "clockspecifysetup.h"
#include "4x20lcd.h"
#include "beep_n_lcd.h"
#include "pay_flt_cnv.h"

#define PI 3.14159265358979323

/* DEBUG */
int debug_mc_state1; // frac time
u32 debug_mc_state2; // delay time
int debug_mc_state3; // max delay

// lcd
u32 t_lcd;
#define LCDPACE (sysclk_freq/4) // LCD pacing increment

u32 t_timeKeeper;

struct MCSTATEPARAM
{
    //  these need to be segregated into launch parameters and winch parameters
    float TICSPERSECOND;
    float REALTIMEFACTOR;
    float STEPTIME;
    u32 STEPTIMECLOCKS;
    
    float GRAVITY_ACCELERATION;
    float ZERO_CABLE_SPEED_TOLERANCE;
    //  DRIVE PARAMETERS
    float TORQUE_TO_TENSION;
    float TENSION_TO_TORQUE;

    // LAUNCH PARAMS
    float GROUND_TENSION_FACTOR;
    float CLIMB_TENSION_FACTOR;

    float GLIDER_MASS;
    float GLIDER_WEIGHT;

    //  for soft stat taper up
    float SOFT_START_TIME;
    float K1;

    //  for rotation taper down 
    float PROFILE_TRIGGER_CABLE_SPEED;
    float MAX_GROUND_CABLE_SPEED;
    float K2;

    //  for transition to ramp
    float PEAK_CABLE_SPEED_DROP;

    //  for ramp taper up 
    float RAMP_TIME;
    float K3;

//  for end of climb taper down
    float TAPERANGLETRIG;
    float TAPERTIME;
    float K4;

    float RELEASEDELTA;

//    for parachute tension and taper
    float MAX_PARACHUTE_TENSION;
    float PARACHUTE_TAPER_SPEED;
    float MAX_PARACHUTE_CABLE_SPEED;
    float K5;

//  for control panel 
    u8 cp_cl_heartbeat;
    float cp_cl_delta;
    u8 cp_inputs_heartbeat;
    u8 cp_outputs_heartbeat;
    u8 cp_lcd_heartbeat;
};

struct MCSCALEOFFSET
{   //  these are more associated with CANbus payload definitions
    float torqueScale;
    float tensionScale;
    float tensionOffset;
    float cableAngleScale;
    float cableAngleOffset;
    float drumRadius;
    float motorToDrum;
    float motorSpeedScale;
};

struct MCSIMULATIONVAR
{
    #define LONGTIME 0
    #if LONGTIME 
    u64 nextStepTime;
    u64 longTime; 
    u32 tmpTime;
    u32 oldTime;
    #else
    u32 nextStepTime;
    #endif  
};

struct MCMEASUREMENTS
{
    float lastTension;
    float lastMotorSpeed;
    float lastCableSpeed;
    float lastCableAngle;
};

struct MCSTATEVAR
{
    // state machine stuff
    int state;
    int speedMessageFlag;
    int tensionMessageFlag;
    int paramReceivedFlag;
    int parametersRequestedFlag;
    int launchResetFlag;
    u32 elapsedTics;
    int startProfileTics;
    int startRampTics;
    float startRampTension;
    float peakCableSpeed;
    int taperFlag;
    float taperTics;
    float minCableSpeed;
	float setptTension;
	float setptTorque;
    float filt_torque;
};

struct MCRCVDCANMSG
{
	struct CANRCVBUF can;	// Msg
	unsigned int dtw;	    // DTW count  
	int frac;		        

};

struct MCMSGSUSED
{
	struct MCRCVDCANMSG lastrcvdTension; 		// CANID_TENSION
	struct MCRCVDCANMSG lastrcvdMotorSpeed; 	// CANID_MOTOR_SPEED
	struct MCRCVDCANMSG lastrcvdCableAngle;		// CABLE_ANGLE_MESSAGE_ID
	struct MCRCVDCANMSG lastrcvdLaunchParam;	// CANID_LAUNCH_PARAM
};

static struct MCSTATEPARAM stateparam;
static struct MCSCALEOFFSET scaleoffset;
static struct MCSIMULATIONVAR simulationvar;
static struct MCMEASUREMENTS measurements;
static struct MCSTATEVAR statevar;
static struct MCMSGSUSED msgsused;

/* **************************************************************************************
 * static void mc_state_msgs_init(struct MCMSGSUSED* msgsused-> );
 * @brief	: Reset struct of msgs use by state machine to not received (dlc = NOTRCVD)
 * @param	: msgsused-> = pointer to struct
 * ************************************************************************************** */
static void mc_state_msgs_init(struct MCMSGSUSED* pmsgsused)
{
	pmsgsused->lastrcvdTension.frac = 0; 	// CANID_TENSION
	pmsgsused->lastrcvdMotorSpeed.frac = 0; 	// CANID_MOTOR_SPEED
	pmsgsused->lastrcvdCableAngle.frac = 0;	// CABLE_ANGLE_MESSAGE_ID
	pmsgsused->lastrcvdLaunchParam.frac = 0;	// CANID_LAUNCH_PARAM
	return;
}
/* **************************************************************************************
 * void mc_state_launch_init(void);
 * @brief	: Initialize structs for state machine
 * ************************************************************************************** */
void mc_state_launch_init(void)
{
    //  this can be simpified for relanch re-initializing only those variables in state
	t_lcd = DTWTIME + LCDPACE;

// struct MCMEASUREMENTS
    measurements.lastTension = 0;
    measurements.lastMotorSpeed = 0;
    measurements.lastCableSpeed = 0;
    measurements.lastCableAngle = 0;

    // struct MCSTATESTUF
    statevar.state = 0;
    statevar.speedMessageFlag = 0;
    statevar.tensionMessageFlag = 0;
    statevar.paramReceivedFlag = 0;
    statevar.parametersRequestedFlag = 0;
    statevar.launchResetFlag = 0;
    statevar.startProfileTics = 0;
    statevar.startRampTics = 0;
    statevar.startRampTension = 0;
    statevar.peakCableSpeed = 0;
    statevar.taperFlag = 1;     //  needed because PS produces a spurious cable angle initially
    statevar.taperTics = 0;
    statevar.minCableSpeed = 0;
    statevar.setptTension = 0;
    statevar.setptTorque = 0;
    statevar.filt_torque = 0;

// struct MCRCVDCANMSGS
	mc_state_msgs_init(&msgsused);
	
	return;
}

/* **************************************************************************************
 * void mc_state_init(struct ETMCVAR* petmcvar);
 * @brief	: Initialize structs for state machine
 * @param	: petmcvar = pointer to struct with variables passed from etmc0.c
 * ************************************************************************************** */
void mc_state_init(struct ETMCVAR* petmcvar)
{
// struct MCSTATEPARAM
    //  these should not need re-initialization

    stateparam.TICSPERSECOND = 16;
    stateparam.STEPTIME = ((float) 1.0) / stateparam.TICSPERSECOND;
    stateparam.REALTIMEFACTOR = ((float) 1.0);

    stateparam.STEPTIMECLOCKS = (u32) (sysclk_freq * stateparam.STEPTIME / stateparam.REALTIMEFACTOR);
    stateparam.GRAVITY_ACCELERATION = (float) 9.81;
    stateparam.ZERO_CABLE_SPEED_TOLERANCE = (float) 0.1;
    //  DRIVE PARAMETERS
    stateparam.TORQUE_TO_TENSION = 20;
    stateparam.TENSION_TO_TORQUE = 1 / stateparam.TORQUE_TO_TENSION;

    // LAUNCH PARAMS
    stateparam.GROUND_TENSION_FACTOR = (float) 1.0;
    stateparam.CLIMB_TENSION_FACTOR = (float) 1.3;

    stateparam.GLIDER_MASS = (float) 600;
    stateparam.GLIDER_WEIGHT = stateparam.GLIDER_MASS * stateparam.GRAVITY_ACCELERATION;

    //  for soft stat taper up
    stateparam.SOFT_START_TIME = (float) 1.0;
    stateparam.K1 = (float) PI / (stateparam.SOFT_START_TIME * stateparam.TICSPERSECOND);

    //  for rotation taper down 
    stateparam.PROFILE_TRIGGER_CABLE_SPEED = (float) 20.578; // 40 knots
    stateparam.MAX_GROUND_CABLE_SPEED = (float) 35;
    stateparam.K2 = (float) (PI / (2 * stateparam.MAX_GROUND_CABLE_SPEED - stateparam.PROFILE_TRIGGER_CABLE_SPEED));

    //  for transition to ramp
    stateparam.PEAK_CABLE_SPEED_DROP = (float) 0.97;

    //  for ramp taper up 
    stateparam.RAMP_TIME = 6;
    stateparam.K3 = (float) (PI / (2 * stateparam.RAMP_TIME * stateparam.TICSPERSECOND));

//  for end of climb taper down
    stateparam.TAPERANGLETRIG = (float) 50;  //  Angle to start taper
    stateparam.TAPERTIME = (float) 3;  //  End of climb taper time
    stateparam.K4 = (float) (PI / (2 * stateparam.TAPERTIME * stateparam.TICSPERSECOND));

    stateparam.RELEASEDELTA = (float) 5;    //  for detection of release

//    for parachute tension and taper
    stateparam.MAX_PARACHUTE_TENSION = (float) 3000;    //  newtons
    stateparam.PARACHUTE_TAPER_SPEED = (float) 25;      //  m/s
    stateparam.MAX_PARACHUTE_CABLE_SPEED = (float) 35;  //  m/s
    stateparam.K5 = (float) (PI / (2 * stateparam.MAX_PARACHUTE_CABLE_SPEED - stateparam.PARACHUTE_TAPER_SPEED));

// struct MCSCALEOFFSET
    scaleoffset.torqueScale = (float) (1.0 / 32.0);
    //scaleoffset.tensionScale = (float) 0.25;
    //scaleoffset.tensionOffset = (float) 1024.0;
    //scaleoffset.cableAngleScale = (float) 0.5;
    //scaleoffset.cableAngleOffset = (float) 40.0;
    scaleoffset.drumRadius = (float) 0.4;
    scaleoffset.motorToDrum = (float) 7.0;    //  speed reduction motor to drum
    //scaleoffset.motorSpeedScale = (float) (1.0 / 128.0);

//  control panel update parameters
    stateparam.cp_cl_heartbeat = 63;
    stateparam.cp_cl_delta = 0.005;
    stateparam.cp_inputs_heartbeat = 63;
    stateparam.cp_outputs_heartbeat = 63;
    stateparam.cp_lcd_heartbeat = 63;


// struct MCSIMULATIONVAR    
//  simulationvar.startTime = (passed from etmc0.c to us)

    #if LONGTIME
    simulationvar.longTime = simulationvar.nextStepTime = simulationvar.oldTime = DTWTIME;
    #else
    simulationvar.nextStepTime = DTWTIME;
    #endif
    petmcvar->fracTime = stateparam.TICSPERSECOND - 1;
    petmcvar->ledBlink = 0xffff;
    statevar.elapsedTics = -1;
	
    mc_state_launch_init();
	
	return;
}
/* **************************************************************************************
 * static void sendStateMessage(int newState);
 * @brief	: Send state machine msg
 * @param	: newState = state.  Payload byte = newState + 1
 * ************************************************************************************ */
static void sendStateMessage(int newState)
{
	struct CANRCVBUF stateMessage;
    stateMessage.id = CANID_STATE;
	stateMessage.dlc = 1;
	stateMessage.cd.uc[0] = newState;
	msg_out_mc(&stateMessage);
	return;
}
/* **************************************************************************************
 * void mc_state_msg_select(struct CANRCVBUF* pcan);
 * @brief	: Select msgs of interest for MC out of incoming stream of msgs
 * @param	: pcan = pointer to can msg
 * ************************************************************************************** */
unsigned int msgrcvlist = 0;	// Accumulate msgs received after sending a time msg
/* Bits within msgrcvlist word */
#define RCV_CANID_TENSION       0x1
#define RCV_CANID_CABLE_ANGLE	0x2
#define RCV_CANID_MOTOR		    0x4
#define RCV_CANID_LAUNCH_PARAM	0x8

void mc_state_msg_select(struct CANRCVBUF* pcan, struct ETMCVAR* petmcvar)
{
	switch (pcan->id)
	{
	case CANID_TENSION:		 // 0x38000000	 448 
if ( (int)(DTWTIME - debug_mc_state2) > debug_mc_state3) debug_mc_state3 = (DTWTIME - debug_mc_state2);
xprintf(UXPRT,"T %d %d\n\r",(DTWTIME - debug_mc_state2),debug_mc_state3 );

		msgsused.lastrcvdTension.can = *pcan;
msgsused.lastrcvdTension.frac = debug_mc_state1;
		//measurements.lastTension = ((float)pcan->cd.us[0] - scaleoffset.tensionOffset) * scaleoffset.tensionScale;
        measurements.lastTension = payhalffptofloat((uint8_t *) &(pcan->cd.uc[0]));
		statevar.tensionMessageFlag = 1;
   	     	msgrcvlist |= RCV_CANID_TENSION;
xprintf(UXPRT,"R%08X %03d %03d\n\r", pcan->id, debug_mc_state1,pcan->cd.uc[3]);

		break;
	case CANID_CABLE_ANGLE:		// 0x3A000000	 464 
		msgsused.lastrcvdCableAngle.can = *pcan;
                //measurements.lastCableAngle = ((float)pcan->cd.uc[0] - scaleoffset.cableAngleOffset) * scaleoffset.cableAngleScale;
                measurements.lastCableAngle = payhalffptofloat((uint8_t *) &(pcan->cd.uc[0]));
                if (statevar.taperFlag == 0 && measurements.lastCableAngle > stateparam.TAPERANGLETRIG)
                {
                	statevar.taperFlag = 1;
                	statevar.taperTics = statevar.elapsedTics;
                }
msgsused.lastrcvdCableAngle.frac = debug_mc_state1;
		msgrcvlist |= RCV_CANID_CABLE_ANGLE;
xprintf(UXPRT,"R%08X %03d %03d\n\r", pcan->id, debug_mc_state1,pcan->cd.uc[2]);
		break;
	case CANID_MOTOR_SPEED:		// 0x25000000	 296 
		msgsused.lastrcvdMotorSpeed.can = *pcan;
		//measurements.lastMotorSpeed = (float)pcan->cd.us[0] * scaleoffset.motorSpeedScale;
        measurements.lastMotorSpeed = payhalffptofloat((uint8_t *) &(pcan->cd.uc[0]));
		measurements.lastCableSpeed = (float) (2 * 3.14159 * scaleoffset.drumRadius * measurements.lastMotorSpeed / scaleoffset.motorToDrum);
msgsused.lastrcvdMotorSpeed.frac = debug_mc_state1;
       		statevar.speedMessageFlag = 1;
		msgrcvlist |= RCV_CANID_MOTOR;
xprintf(UXPRT,"R%08X %03d %03d\n\r", pcan->id, debug_mc_state1,pcan->cd.uc[6]);
		break;
	case CANID_LAUNCH_PARAM:	// 0x28E00000	 327 
		msgsused.lastrcvdLaunchParam.can = *pcan;
msgsused.lastrcvdLaunchParam.frac = debug_mc_state1;
        	statevar.paramReceivedFlag = 1;
		msgrcvlist |= CANID_LAUNCH_PARAM;
xprintf(UXPRT,"R%08X %03d %03d\n\r", pcan->id, debug_mc_state1, pcan->cd.uc[0]);
		break;
    case CANID_CP_INPUTS_RMT: 
        petmcvar->cp_inputs = pcan->cd.us[0];    
        break;
    case CANID_CP_CL_RMT: 
        petmcvar->cp_cl = payhalffptofloat((uint8_t *) &(pcan->cd.uc[0]));    
        break;        
	}
	return;
}

/* --------------------------------------------------------------------------------------
void mc_debug_print(void);
----------------------------------------------------------------------------------------- */
void mc_debug_print(void)
{
	xprintf(UXPRT,"Going to state: %d\n\r", statevar.state);
	return;
}

/* **************************************************************************************
 * void stateMachine(struct ETMCVAR* petmcvar);
 * @brief	: Run state machine
 * @param	: petmcvar = pointer vars passed from etmc0
 * ************************************************************************************** */
void stateMachine(struct ETMCVAR* petmcvar)
{ 
	struct CANRCVBUF can;
    
    //petmcvar->cp_cl = calib_control_lever_get();

    if (statevar.launchResetFlag == 1)   // init variables for launch
     {
        //  reset variables for next launch
         statevar.state = 10;
         statevar.tensionMessageFlag = statevar.speedMessageFlag = 0;
         statevar.parametersRequestedFlag = 0;
         statevar.paramReceivedFlag = 0;
         statevar.launchResetFlag = 0;
         statevar.filt_torque = 0;  //  This should be set to 0 on entry to
                                    //  Prep from Safe in real system
     }
     
     #if LONGTIME     
     
     // update the long times
     simulationvar.tmpTime = DTWTIME;
     // check msbs for overflow toggle
     if (((simulationvar.tmpTime & 0x80000000) == 0) && ((simulationvar.oldTime & 0x80000000) == 1))
     {  //  DTWTIME has overflowed
        simulationvar.longTime += 0x0000000100000000;
     }
     simulationvar.longTime &= 0xffffffff00000000;
     simulationvar.longTime |= (u64) simulationvar.tmpTime;
     simulationvar.oldTime = simulationvar.tmpTime;     

     if (((long)(simulationvar.longTime - simulationvar.nextStepTime) >= 0))
        //&& (statevar.tensionMessageFlag == 1) 
        //&& (statevar.speedMessageFlag == 1))
        {
            (statevar.elapsedTics)++;
            can.id = CANID_TIME;    // time id
            if (++(petmcvar->fracTime) != stateparam.TICSPERSECOND)
            {
                can.dlc = 1;
                can.cd.uc[0] = petmcvar->fracTime;
            }
            else
            {                
                can.dlc = 5;
                can.cd.ui[0] = (petmcvar->unixtime)++;
                can.cd.uc[4] = (u8) 0;    //  status proxy
                petmcvar->fracTime = 0;
            }            
            msg_out_mc(&can); // output to CAN+USB
debug_mc_state1 = petmcvar->fracTime;
            // next on time Time message time                   
            simulationvar.nextStepTime  += stateparam.STEPTIMECLOCKS;
        }
        #else
        if (((int)(DTWTIME - simulationvar.nextStepTime) > 0))
            //&& (statevar.tensionMessageFlag == 1) 
            //&& (statevar.speedMessageFlag == 1))
        {
            (statevar.elapsedTics)++;
            can.id = CANID_TIME;    // time id
            if (++(petmcvar->fracTime) != stateparam.TICSPERSECOND)
            {
                can.dlc = 1;
                can.cd.uc[0] = petmcvar->fracTime;
            }
            else
            {                
                can.dlc = 5;
                can.cd.ui[0] = (petmcvar->unixtime)++;
                can.cd.uc[4] = (u8) 0;    //  status proxy
                petmcvar->fracTime = 0;
            }            
debug_mc_state1 = petmcvar->fracTime;
            msg_out_mc(&can); // output to CAN+USB
debug_mc_state2 = DTWTIME; // Time round trip to PS
            //  Control Panel Output Messages
            if (0 && GPIOB_IDR & (1 << 1)) //    test for local or glass CP
            {

                float clDel = petmcvar->cp_cl - petmcvar->cp_cl_old;
                clDel = (clDel >=0) ? clDel : -clDel;
                if ((clDel > CP_CL_DELTA) || (petmcvar->cp_cl_count-- <= 0))
                {   //  output local control lever message
                    petmcvar->cp_cl_count = petmcvar->cp_cl_count <= 0 ? CP_CL_HB_COUNT : petmcvar->cp_cl_count;
                    petmcvar->cp_cl_old = petmcvar->cp_cl;
                    can.id = CANID_CP_CL_LCL;
                    can.dlc = 3;
                    floattopayhalffp(&can.cd.uc[0], petmcvar->cp_cl);
                    can.cd.uc[2] = 0;
                    msg_out_mc(&can);
                }              
                if ((petmcvar->cp_inputs ^ petmcvar->cp_inputs_old) || (petmcvar->cp_inputs_count-- <= 0))
                {   //  output local control panel inputs message
                    petmcvar->cp_inputs_count = (petmcvar->cp_inputs_count <= 0) ? CP_INPUTS_HB_COUNT : petmcvar->cp_inputs_count;
                    petmcvar->cp_inputs_old = petmcvar->cp_inputs;
                    can.id = CANID_CP_INPUTS_LCL;
                    can.dlc = 2;
                    can.cd.us[0] = petmcvar->cp_inputs;
                    msg_out_mc(&can);    
                }                
            }
            if ((petmcvar->cp_outputs ^ petmcvar->cp_outputs_old) || (petmcvar->cp_outputs_count-- <= 0))
            {   //  output led output message
                petmcvar->cp_outputs_count = (petmcvar->cp_outputs_count <= 0) ? CP_OUTPUTS_HB_COUNT : petmcvar->cp_outputs_count;
                petmcvar->cp_outputs_old = petmcvar->cp_outputs;
                can.id = CANID_CP_OUTPUTS;
                can.dlc = 2;
                can.cd.us[0] = petmcvar->cp_outputs;
                msg_out_mc(&can);
            }
            

            // next on time Time message time
            simulationvar.nextStepTime  += stateparam.STEPTIMECLOCKS;
	} 
        #endif
    
    switch (statevar.state)
    {
        case 0: //  safe
        petmcvar->cp_outputs = LED_SAFE;
        if((petmcvar->cp_inputs & SW_ACTIVE) == 0)
        {
            statevar.state = 10; // going to prep state
            sendStateMessage(1);
            beep_n(1, petmcvar);   //  single beep
            mc_debug_print();
        }
        break;

        case 10: // prep                        
                        
            petmcvar->cp_outputs = (LED_PREP & petmcvar->ledBlink) | LED_ARM_PB | LED_PREP_PB;
            if((petmcvar->cp_inputs & SW_SAFE) == 0)
            {
                statevar.state = 0; // going to safe state
                sendStateMessage(0);
                beep_n(1, petmcvar);   //  single beep
                mc_debug_print();
                break;
            }
            if((petmcvar->cp_inputs & PB_ARM) == 0)
            { 
                statevar.state = 20; // going to armed state
                sendStateMessage(2);
                beep_n(1, petmcvar);   //  single beep
                mc_debug_print();
            }
            break;

        case 20: // armed            
            petmcvar->cp_outputs = LED_ARM_PB | LED_PREP_PB | (LED_ARM & petmcvar->ledBlink);
            if((petmcvar->cp_inputs & SW_SAFE) == 0)
            {
                statevar.state = 0; // going to safe state
                sendStateMessage(0);
                beep_n(1, petmcvar);   //  single beep
                mc_debug_print();
                break;
            }
            if((petmcvar->cp_inputs & PB_PREP) == 0)
            {
                statevar.state = 10; // going to prep state
                sendStateMessage(1);
                beep_n(1, petmcvar);   //  single beep
                mc_debug_print();
                break;
            }
            if ((statevar.parametersRequestedFlag == 0) 
                    && (petmcvar->cp_cl > (float) 0.95))
            {
                // request launch parameters
            	can.id = CANID_PARAM_REQUEST;
can.cd.uc[0] = debug_mc_state1;    //  for debug
            	can.dlc = 0 + 1;
            	msg_out_mc(&can);
                statevar.parametersRequestedFlag = 1;
            }
            if ((statevar.parametersRequestedFlag == 1) 
                    && (petmcvar->cp_cl < (float) 0.05))
            {
                // reset and wait for control lever again                
                statevar.parametersRequestedFlag = 0;
            }
            // when we get the response, start the simulation
            if (statevar.paramReceivedFlag == 1)
            {
            //    simulationvar.startTime = (double) DTWTIME; // not used?
                statevar.state = 30;    // going to profile state
                beep_n(1, petmcvar);   //  single beep
                statevar.startProfileTics = statevar.elapsedTics;
                sendStateMessage(3);
                mc_debug_print();
            }
            break;

        case 30: // profile 0   soft start
            petmcvar->cp_outputs = LED_GNDRLRTN;  // & petmcvar->ledBlink;
            if ((statevar.elapsedTics - statevar.startProfileTics) 
                >= (stateparam.SOFT_START_TIME * stateparam.TICSPERSECOND))
            {
                statevar.state = 31;
                statevar.peakCableSpeed = measurements.lastCableSpeed;
                mc_debug_print();
            }
            break;

        case 31: // profile 1   constant tension ground roll
            petmcvar->cp_outputs = LED_GNDRLRTN;    // & petmcvar->ledBlink;
            statevar.peakCableSpeed = measurements.lastCableSpeed > statevar.peakCableSpeed
                    ? measurements.lastCableSpeed : statevar.peakCableSpeed;
            if (measurements.lastCableSpeed < (statevar.peakCableSpeed * stateparam.PEAK_CABLE_SPEED_DROP))
            {
                statevar.state = 40;
                beep_n(1, petmcvar);   //  single beep
                sendStateMessage(4);
                statevar.startRampTics = statevar.elapsedTics;
                statevar.startRampTension = measurements.lastTension;               
                mc_debug_print();
            }
            break;

        case 40: // ramp
            petmcvar->cp_outputs = LED_RAMP;    // & petmcvar->ledBlink;
            if (statevar.elapsedTics - statevar.startRampTics > stateparam.RAMP_TIME * stateparam.TICSPERSECOND)
            {
                statevar.state = 50;
                beep_n(1, petmcvar);   //  single beep
                sendStateMessage(5);
                statevar.minCableSpeed = measurements.lastCableSpeed;
                statevar.taperFlag = 0;
                mc_debug_print();                
            }
            break;

        case 50: // climb
            //  xprintf(UXPRT,"%6d\n\r", (double) measurements.lastCableSpeed);
            petmcvar->cp_outputs = LED_CLIMB;   // & petmcvar->ledBlink;
            if (measurements.lastCableSpeed < statevar.minCableSpeed)
            {
                statevar.minCableSpeed = measurements.lastCableSpeed;
            }
            if (measurements.lastCableSpeed > statevar.minCableSpeed + stateparam.RELEASEDELTA)
            {
                beep_n(1, petmcvar);   //  single beep
                statevar.state = 60;
                sendStateMessage(6);
                mc_debug_print();
            }
            break;

        case 60: // recovery
             //xprintf(UXPRT,"%6d\n\r",measurements.lastCableSpeed);
            petmcvar->cp_outputs = LED_RECOVERY;    // & petmcvar->ledBlink;
            if (measurements.lastCableSpeed < stateparam.ZERO_CABLE_SPEED_TOLERANCE)
            {
                statevar.state = 10;
                beep_n(2, petmcvar);   //  single beep
                petmcvar->cp_outputs = LED_PREP;
                sendStateMessage(1);                
                statevar.launchResetFlag = 1; 
                mc_debug_print();                           
            }
            break;
    }   // end of switch (statevar.state)
    
    //  Template for Desired Tension and Control Law
    if ((statevar.tensionMessageFlag == 1) && (statevar.speedMessageFlag == 1))   
    {
        switch (statevar.state)
        {
            case 0:
                statevar.setptTension = 0;
                break;

            case 10: // prep
                statevar.setptTension = 0;
                break;

            case 20: // armed
                statevar.setptTension = 0;
                break;
            case 30: // profile 0   soft start
                statevar.setptTension = (float) (stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT
                * 0.5  * (1 - cosf(stateparam.K1 * (statevar.elapsedTics - statevar.startProfileTics))));
                break;

            case 31: // profile 1   constant tension ground roll with taper
                // System.out.println(measurements.lastCableSpeed +  stateparam.PROFILE_TRIGGER_CABLE_SPEED);
                if (measurements.lastCableSpeed < stateparam.PROFILE_TRIGGER_CABLE_SPEED)
                {
                    statevar.setptTension = stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT;
                    //xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	//  System.out.println(tension);
                } 
	             else
                {
                    statevar.setptTension = (float) (stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT * cosf(stateparam.K2 * (measurements.lastCableSpeed - stateparam.PROFILE_TRIGGER_CABLE_SPEED)));
                    //xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	// System.out.println(tension);
                }
                break;
            case 40: // ramp
                statevar.setptTension = (float) ((statevar.startRampTension
                        + (stateparam.CLIMB_TENSION_FACTOR * stateparam.GLIDER_WEIGHT
                        - statevar.startRampTension)
                        * sinf(stateparam.K3 * (statevar.elapsedTics - statevar.startRampTics))));
                //xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	//  System.out.println(tension);
                break;
            case 50: // constant
                statevar.setptTension = (float) (stateparam.CLIMB_TENSION_FACTOR * stateparam.GLIDER_WEIGHT);
                if (statevar.taperFlag == 1)
                {
                    statevar.setptTension *= 0.4 + 0.6 * 0.5
                            * (1 + cosf(stateparam.K4 * (statevar.elapsedTics - statevar.taperTics)));
                }
                break;
            case 60: // recovery
                statevar.setptTension = stateparam.MAX_PARACHUTE_TENSION;
                if (measurements.lastCableSpeed > stateparam.PROFILE_TRIGGER_CABLE_SPEED)
                {
                    statevar.setptTension *= cosf(stateparam.K5 * (measurements.lastCableSpeed - stateparam.PARACHUTE_TAPER_SPEED));
                    //xprintf(UXPRT,"%6d\n\r",(double) statevar.setptTension);	// System.out.println(tension);
                }
                break;            
        }
        //  scale by control lever
        statevar.setptTension *= petmcvar->cp_cl;   //  comment out to not have to hold handle

        //  filter the torque with about 1 Hz bandwidth
        statevar.setptTorque = statevar.setptTension * stateparam.TENSION_TO_TORQUE; 
        statevar.filt_torque += (statevar.setptTorque - statevar.filt_torque) 
        * ((float) 1.0 / ((8 * stateparam.TICSPERSECOND) / 64));

        // torqueMessage.set_short((short) (statevar.filt_torque / scaleoffset.torqueScale), 0); // torque
     	can.id = CANID_TORQUE;
        can.dlc = 2 + 1;
can.cd.uc[2] = debug_mc_state1;    //  for debug
        can.cd.us[0] = (short) (statevar.filt_torque / scaleoffset.torqueScale);
        msg_out_mc(&can);
        statevar.tensionMessageFlag = statevar.speedMessageFlag = 0;
    }            
}

/* **************************************************************************************
 * void mc_state_lcd_poll(struct ETMCVAR* petmcvar);
 * @brief	: Output LCD data
 * @param	: petmcvar = pointer to variables in etmc0
 * ************************************************************************************** */
void mc_state_lcd_poll(struct ETMCVAR* petmcvar)
{
	char lcdLine[LCDLINESIZE + 1];
	if (((int) (DTWTIME - t_lcd)) > 0) 
    {
		t_lcd += LCDPACE;
		snprintf(lcdLine, 21, "State %4d", statevar.state); 
        lcd_printToLine(UARTLCD, 0, lcdLine);
        xprintf(UXPRT,"%s ",lcdLine);
        snprintf(lcdLine, 21, "Dsrd Tension: %5.2f", (double) statevar.setptTension);	
        lcd_printToLine(UARTLCD, 1, lcdLine);
        xprintf(UXPRT,"%s ",lcdLine);
        snprintf(lcdLine, 21, "Time: %d", (int) petmcvar->unixtime);      
        lcd_printToLine(UARTLCD, 2, lcdLine);
        xprintf(UXPRT,"%s ",lcdLine);
		//snprintf(lcdLine, 21, "Control Lvr: %7.3f", (double) petmcvar->cp_cl);		
        //lcd_printToLine(UARTLCD, 3, lcdLine);
        //xprintf(UXPRT,"%s \n\r",lcdLine);
snprintf(lcdLine, 21, "Sw: %4x Cl: %4.2f", (int) petmcvar->cp_inputs, (double) petmcvar->cp_cl);        
lcd_printToLine(UARTLCD, 3, lcdLine);
xprintf(UXPRT,"%s \n\r",lcdLine);        
	}
	return;
}

