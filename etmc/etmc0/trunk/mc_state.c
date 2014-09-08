/******************************************************************************
* File Name          : mc_state.c
* Date First Issued  : 09/03/2014
* Board              : 
* Description        : ET state machine for etmc0.c
*******************************************************************************/
#include "etmc0.h"
#include "common_canid_et.h"

#include "xprintf.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "init_hardware_mc.h"
#include "calib_control_lever.h"
#include "clockspecifysetup.h"
#include "4x20lcd.h"

#define PI 3.14159265358979323

// lcd
u32 t_lcd;
#define LCDPACE (sysclk_freq/8) // LCD pacing increment

u32 t_timeKeeper;

struct MCSTATEPARAM
{
    //  these need to be segregated into launch parameters and winch parameters
    float TICSPERSECOND;
    float REALTIMEFACTOR;
    float STEPTIME;
    
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
{   //  TimeMillis variables need to be just in seconds now tht we have good fp
    unsigned char fracTime;
//	unsigned int startTime;  // not used?
    int elapsedTics;
    //double simulationStrtTime;    //  not used?
    double timeMillis;
    u32 nextStepTime;
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
	int flag;		        

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
	pmsgsused->lastrcvdTension.flag = 0; 	// CANID_TENSION
	pmsgsused->lastrcvdMotorSpeed.flag = 0; 	// CANID_MOTOR_SPEED
	pmsgsused->lastrcvdCableAngle.flag = 0;	// CABLE_ANGLE_MESSAGE_ID
	pmsgsused->lastrcvdLaunchParam.flag = 0;	// CANID_LAUNCH_PARAM

	return;
}
/* **************************************************************************************
 * void mc_state_launch_init(void);
 * @brief	: Initialize structs for state machine
 * ************************************************************************************** */
void mc_state_launch_init(void)
{
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
    statevar.launchResetFlag = 1;
    statevar.startProfileTics = 0;
    statevar.startRampTics = 0;
    statevar.startRampTension = 0;
    statevar.peakCableSpeed = 0;
    statevar.taperFlag = 1;
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
    //  Wwy do these need to be reinitialized?

    stateparam.TICSPERSECOND = 64;
    stateparam.STEPTIME = ((float) 1.0) / stateparam.TICSPERSECOND;
    stateparam.REALTIMEFACTOR = ((float) 1.0);

    stateparam.STEPTIMECLOCKS = sysclk_freq * stateparam.STEPTIME / stateparam.REALTIMEFACTOR;
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
    scaleoffset.tensionScale = (float) 0.25;
    scaleoffset.tensionOffset = (float) 1024.0;
    scaleoffset.cableAngleScale = (float) 0.5;
    scaleoffset.cableAngleOffset = (float) 40.0;
    scaleoffset.drumRadius = (float) 0.4;
    scaleoffset.motorToDrum = (float) 7.0;    //  speed reduction motor to drum
    scaleoffset.motorSpeedScale = (float) (1.0 / 128.0);

// struct MCSIMULATIONVAR
    simulationvar.fracTime = 0;
    simulationvar.elapsedTics = -1;
//  simulationvar.startTime = (passed from etmc0.c to us)
    simulationvar.timeMillis = 0;
    simulationvar.nextStepTime = 0;


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
	stateMessage.cd.uc[0] = newState + 1;
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

void mc_state_msg_select(struct CANRCVBUF* pcan)
{
	switch (pcan->id)
	{	
	case CANID_TENSION:		 // 0x38000000	 448 
		msgsused.lastrcvdTension.can = *pcan;
		msgsused.lastrcvdTension.flag += 1;
		measurements.lastTension = ((float)pcan->cd.us[0] - scaleoffset.tensionOffset) * scaleoffset.tensionScale;
		statevar.tensionMessageFlag = 1;
        msgrcvlist |= RCV_CANID_TENSION;
		break;
	case CANID_CABLE_ANGLE:	// 0x3A000000	 464 
		msgsused.lastrcvdCableAngle.can = *pcan;
                measurements.lastCableAngle = ((float)pcan->cd.uc[0] - scaleoffset.cableAngleOffset) * scaleoffset.cableAngleScale;
                if (statevar.taperFlag == 0 && measurements.lastCableAngle > stateparam.TAPERANGLETRIG)
                {
                	statevar.taperFlag = 1;
                	statevar.taperTics = simulationvar.elapsedTics;
                }
		msgsused.lastrcvdCableAngle.flag += 1;
		msgrcvlist |= RCV_CANID_CABLE_ANGLE;
		break;
	case CANID_MOTOR_SPEED:		// 0x25000000	 296 
		msgsused.lastrcvdMotorSpeed.can = *pcan;
		measurements.lastMotorSpeed = (float)pcan->cd.us[0] * scaleoffset.motorSpeedScale;
		measurements.lastCableSpeed = (float) (2 * 3.14159 * scaleoffset.drumRadius * measurements.lastMotorSpeed / scaleoffset.motorToDrum);
		msgsused.lastrcvdMotorSpeed.flag += 1;
        statevar.speedMessageFlag = 1;
		msgrcvlist |= RCV_CANID_MOTOR;
		break;
	case CANID_LAUNCH_PARAM:	// 0x28E00000	 327 
		msgsused.lastrcvdLaunchParam.can = *pcan;
		msgsused.lastrcvdLaunchParam.flag += 1;
        statevar.paramReceivedFlag = 1;
		msgrcvlist |= CANID_LAUNCH_PARAM;
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


    if (statevar.launchResetFlag == 1)   // init variables for launch
     {
        //  reset variables for next launch
         statevar.state = 0;
         statevar.tensionMessageFlag = statevar.speedMessageFlag = 0;
         statevar.parametersRequestedFlag = 0;
         statevar.paramReceivedFlag = 0;
         statevar.launchResetFlag = 0;
         statevar.filt_torque = 0;  //  This should be set to 0 on entry to
                                    //  Prep from Safe in real system
     }

    //  timekeeping temporarily holding for tension/motor messages
    if ((DTWTIME >= simulationvar.nextStepTime )  
        && (statevar.tensionMessageFlag == 1) 
        && (statevar.speedMessageFlag == 1))
        {
            simulationvar.elapsedTics++;
            petmcvar->fracTime = (char) (simulationvar.elapsedTics % stateparam.TICSPERSECOND);
            can.id = CANID_TIME; // time id
            can.dlc = 1;
            can.cd.us[0] = simulationvar.fracTime;
            if (simulationvar.fracTime == 0)
            {
                can.dlc      = 5;
                can.cd.us[1] = (petmcvar->unixtime)++;
            }
            msg_out_mc(&can); // output to CAN+USB
            // next on time Time message time                   
            simulationvar.nextStepTime  += stateparam.STEPTIMECLOCKS;
        } 

    switch (statevar.state)
    {
        case 0: // prep                        
            //  This will be replaced with detection of pushing the ARM button
            if (calib_control_lever_get() < (float) 0.05)
            { 
                statevar.state = 1; // going to armed state
                // setStateled(1);	// ??? LED
                sendStateMessage(1);
                mc_debug_print();
            }
            break;
        case 1: // armed
            
            if ((statevar.parametersRequestedFlag == 0) 
                    && (calib_control_lever_get() > (float) 0.95))
            {
                // request launch parameters
            	can.id = CANID_PARAM_REQUEST;
            	can.dlc = 0;
            	msg_out_mc(&can);
                statevar.parametersRequestedFlag = 1;
            }
            if ((statevar.parametersRequestedFlag == 1) 
                    && (calib_control_lever_get() < (float) 0.05))
            {
                // reset and wait for control lever again                
                statevar.parametersRequestedFlag = 0;
            }
            // when we get the response, start the simulation
            if (statevar.paramReceivedFlag == 1)
            {
            //    simulationvar.startTime = (double) DTWTIME; // not used?
                
                statevar.state = 2;
                // // setStateled(2); 	// LED ???
                statevar.startProfileTics = simulationvar.elapsedTics;
                sendStateMessage(2);
                mc_debug_print();
            }
            break;
        case 2: // profile 1
            if ((simulationvar.elapsedTics - statevar.startProfileTics) 
                >= (stateparam.SOFT_START_TIME * stateparam.TICSPERSECOND))
            {
                statevar.state = 3;
                statevar.peakCableSpeed = measurements.lastCableSpeed;
                // setStateled(3);
                mc_debug_print();
            }
            break;
        case 3: // profile 2
            statevar.peakCableSpeed = measurements.lastCableSpeed > statevar.peakCableSpeed
                    ? measurements.lastCableSpeed : statevar.peakCableSpeed;
            if (measurements.lastCableSpeed < (statevar.peakCableSpeed * stateparam.PEAK_CABLE_SPEED_DROP))
            {
                statevar.state = 4;
                statevar.startRampTics = simulationvar.elapsedTics;
                statevar.startRampTension = measurements.lastTension;
                sendStateMessage(4);
                // setStateled(4);
                mc_debug_print();
            }
            break;
        case 4: // ramp
            if (simulationvar.elapsedTics - statevar.startRampTics > stateparam.RAMP_TIME * stateparam.TICSPERSECOND)
            {
                statevar.state = 5;
                // setStateled(5);
                sendStateMessage(5);
                statevar.minCableSpeed = measurements.lastCableSpeed;
                mc_debug_print();
                statevar.taperFlag = 0;
            }
            break;
        case 5: // constant
            xprintf(UXPRT,"%6d\n\r", (double) measurements.lastCableSpeed);
            if (measurements.lastCableSpeed < statevar.minCableSpeed)
            {
                statevar.minCableSpeed = measurements.lastCableSpeed;
            }
            if (measurements.lastCableSpeed > statevar.minCableSpeed + stateparam.RELEASEDELTA)
            {
                statevar.state = 6;
                // setStateled(6);
                sendStateMessage(6);
                mc_debug_print();
            }
            break;
        case 6: // recovery
             // xprintf(UXPRT,"%6d\n\r",measurements.lastCableSpeed);
            if (measurements.lastCableSpeed < stateparam.ZERO_CABLE_SPEED_TOLERANCE)
            {
                statevar.state = 0;
                // setStateled(0);
                sendStateMessage(0);
                mc_debug_print();
                statevar.launchResetFlag = 1;                            
            }
            break;
    }   // end of switch (statevar.state)
    
    //  Template for Desired Tension and Control Law
    if ((statevar.tensionMessageFlag == 1) && (statevar.speedMessageFlag == 1))   
    {
        switch (statevar.state)
        {
            case 0: // prep
                statevar.setptTension = 0;
                break;

            case 1: // armed
                statevar.setptTension = 0;
                break;
            case 2: // profile 1
                statevar.setptTension = (float) (stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT
                * 0.5  * (1 - cosf(stateparam.K1 * (simulationvar.elapsedTics - statevar.startProfileTics))));
                break;

            case 3: // profile 2
                // System.out.println(measurements.lastCableSpeed +  stateparam.PROFILE_TRIGGER_CABLE_SPEED);
                if (measurements.lastCableSpeed < stateparam.PROFILE_TRIGGER_CABLE_SPEED)
                {
                    statevar.setptTension = stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT;
                    xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	//  System.out.println(tension);
                } 
	             else
                {
                    statevar.setptTension = (float) (stateparam.GROUND_TENSION_FACTOR * stateparam.GLIDER_WEIGHT * cosf(stateparam.K2 * (measurements.lastCableSpeed - stateparam.PROFILE_TRIGGER_CABLE_SPEED)));
                    xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	// System.out.println(tension);
                }
                break;
            case 4: // ramp
                statevar.setptTension = (float) ((statevar.startRampTension
                        + (stateparam.CLIMB_TENSION_FACTOR * stateparam.GLIDER_WEIGHT
                        - statevar.startRampTension)
                        * sinf(stateparam.K3 * (simulationvar.elapsedTics - statevar.startRampTics))));
                xprintf(UXPRT,"%6d\n\r", (double) statevar.setptTension);	//  System.out.println(tension);
                break;
            case 5: // constant
                statevar.setptTension = (float) (stateparam.CLIMB_TENSION_FACTOR * stateparam.GLIDER_WEIGHT);
                if (statevar.taperFlag == 1)
                {
                    statevar.setptTension *= 0.4 + 0.6 * 0.5
                            * (1 + cosf(stateparam.K4 * (simulationvar.elapsedTics - statevar.taperTics)));
                }
                break;
            case 6: // recovery
                statevar.setptTension = stateparam.MAX_PARACHUTE_TENSION;
                if (measurements.lastCableSpeed > stateparam.PROFILE_TRIGGER_CABLE_SPEED)
                {
                    statevar.setptTension *= cosf(stateparam.K5 * (measurements.lastCableSpeed - stateparam.PARACHUTE_TAPER_SPEED));
                    xprintf(UXPRT,"%6d\n\r",(double) statevar.setptTension);	// System.out.println(tension);
                }
                break;            
        }
        
        statevar.setptTension *= (float) calib_control_lever_get(); 

        //  filter the torque with about 1 Hz bandwidth
        statevar.setptTorque = statevar.setptTension * stateparam.TENSION_TO_TORQUE; 
        statevar.filt_torque += (statevar.setptTorque - statevar.filt_torque) * ((float) 1.0 / 8);

        // torqueMessage.set_short((short) (statevar.filt_torque / scaleoffset.torqueScale), 0); // torque
     	can.id = CANID_TORQUE;
        can.dlc = 2;
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
        snprintf(lcdLine, 21, "Time: %d", petmcvar->unixtime);      
        lcd_printToLine(UARTLCD, 2, lcdLine);
        xprintf(UXPRT,"%s ",lcdLine);
		snprintf(lcdLine, 21, "Control Lvr: %7.3f", (double) calib_control_lever_get());		
        lcd_printToLine(UARTLCD, 3, lcdLine);
        xprintf(UXPRT,"%s \n\r",lcdLine);
	}
	return;
}

