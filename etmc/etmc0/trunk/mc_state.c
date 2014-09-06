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

struct MCSTATEVAR
{

    float TICSPERSECOND;
    float SIMULATIONSTEPTIME;
    float REALTIMEFACTOR;

    float STEPTIMEMILLIS;
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
{
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
        unsigned char fracTime;
	unsigned int simulationStartTime;
        int elapsedTics;
        double simulationStrtTime;      // milliseconds since Unix Epoc
        double timeMillis;
        float nextStepTime;
        double remainingTimeMillis;
};

struct MCMEASUREMENTS
{
        float calibTension;
        float calibMotorSpeed;
        float calibCableSpeed;
        float calibCableAngle;
};


struct MCSTATESTUFF
{
       // state machine stuff
        int state;
        int speedMessageFlag;
        int tensionMessageReceivedFlag;
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
	unsigned int	dtw;	// DTW count
	int flag;		// 

};

struct MCMSGSUSED
{
	struct MCRCVDCANMSG lastrcvdTension; 		// CANID_TENSION
	struct MCRCVDCANMSG lastrcvdMotorSpeed; 	// CANID_MOTOR_SPEED
	struct MCRCVDCANMSG lastrcvdCableAngle;		// CABLE_ANGLE_MESSAGE_ID
	struct MCRCVDCANMSG lastrcvdLaunchParam;	// CANID_LAUNCH_PARAM
};


static struct MCSTATEVAR statevar;
static struct MCSCALEOFFSET scaleoffset;
static struct MCSIMULATIONVAR simulationvar;
static struct MCMEASUREMENTS measurements;
static struct MCSTATESTUFF statestuff;
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
	t_lcd        = DTWTIME + LCDPACE;

// struct MCMEASUREMENTS
        measurements.calibTension = 0;
        measurements.calibMotorSpeed = 0;
        measurements.calibCableSpeed = 0;
        measurements.calibCableAngle = 0;

// struct MCSTATESTUF
        statestuff.state = 0;
        statestuff.speedMessageFlag = 0;
        statestuff.tensionMessageReceivedFlag = 0;
        statestuff.paramReceivedFlag = 0;
        statestuff.parametersRequestedFlag = 0;
        statestuff.launchResetFlag = 1;
        statestuff.startProfileTics = 0;
        statestuff.startRampTics = 0;
        statestuff.startRampTension = 0;
        statestuff.peakCableSpeed = 0;
        statestuff.taperFlag = 1;
        statestuff.taperTics = 0;
        statestuff.minCableSpeed = 0;
        statestuff.setptTension = 0;
        statestuff.setptTorque = 0;
        statestuff.filt_torque = 0;

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
// struct MCSTATEVAR

    statevar.TICSPERSECOND = 64;
    statevar.SIMULATIONSTEPTIME = ((float) 1.0) / statevar.TICSPERSECOND;
    statevar.REALTIMEFACTOR = ((float) 1.0);

    statevar.STEPTIMEMILLIS = 1000 * statevar.SIMULATIONSTEPTIME / statevar.REALTIMEFACTOR;
    statevar.GRAVITY_ACCELERATION = (float) 9.81;
    statevar.ZERO_CABLE_SPEED_TOLERANCE = (float) 0.1;
    //  DRIVE PARAMETERS
    statevar.TORQUE_TO_TENSION = 20;
    statevar.TENSION_TO_TORQUE = 1 / statevar.TORQUE_TO_TENSION;

    // LAUNCH PARAMS
    statevar.GROUND_TENSION_FACTOR = (float) 1.0;
    statevar.CLIMB_TENSION_FACTOR = (float) 1.3;

    statevar.GLIDER_MASS = (float) 600;
    statevar.GLIDER_WEIGHT = statevar.GLIDER_MASS * statevar.GRAVITY_ACCELERATION;

    //  for soft stat taper up
    statevar.SOFT_START_TIME = (float) 1.0;
    statevar.K1 = (float) PI / (statevar.SOFT_START_TIME * statevar.TICSPERSECOND);

    //  for rotation taper down 
    statevar.PROFILE_TRIGGER_CABLE_SPEED = (float) 20.578; // 40 knots
    statevar.MAX_GROUND_CABLE_SPEED = (float) 35;
    statevar.K2 = (float) (PI / (2 * statevar.MAX_GROUND_CABLE_SPEED - statevar.PROFILE_TRIGGER_CABLE_SPEED));

    //  for transition to ramp
    statevar.PEAK_CABLE_SPEED_DROP = (float) 0.97;

    //  for ramp taper up 
    statevar.RAMP_TIME = 6;
    statevar.K3 = (float) (PI / (2 * statevar.RAMP_TIME * statevar.TICSPERSECOND));

//  for end of climb taper down
    statevar.TAPERANGLETRIG = (float) 50;  //  Angle to start taper
    statevar.TAPERTIME = (float) 3;  //  End of climb taper time
    statevar.K4 = (float) (PI / (2 * statevar.TAPERTIME * statevar.TICSPERSECOND));

    statevar.RELEASEDELTA = (float) 5;    //  for detection of release

//    for parachute tension and taper
    statevar.MAX_PARACHUTE_TENSION = (float) 3000;    //  newtons
    statevar.PARACHUTE_TAPER_SPEED = (float) 25;      //  m/s
    statevar.MAX_PARACHUTE_CABLE_SPEED = (float) 35;  //  m/s
    statevar.K5 = (float) (PI / (2 * statevar.MAX_PARACHUTE_CABLE_SPEED - statevar.PARACHUTE_TAPER_SPEED));

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
//        simulationvar.simulationStartTime = (passed from etmc0.c to us)
        simulationvar.timeMillis = 0;
        simulationvar.nextStepTime = 0;
        simulationvar.remainingTimeMillis = 0;


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
#define RCV_CANID_TENSION	0x1
#define RCV_CANID_CABLE_ANGLE	0x2
#define RCV_CANID_MOTOR		0x4
#define RCV_CANID_LAUNCH_PARAM	0x8

#define RCV_ALLNEEDEDMSGS (RCV_CANID_TENSION | RCV_CANID_MOTOR)

void mc_state_msg_select(struct CANRCVBUF* pcan)
{
	switch (pcan->id)
	{	
	case CANID_TENSION:		 // 0x38000000	 448 
		msgsused.lastrcvdTension.can = *pcan;
		msgsused.lastrcvdTension.flag += 1;
		measurements.calibTension = ((float)pcan->cd.us[0] - scaleoffset.tensionOffset) * scaleoffset.tensionScale;
		msgrcvlist |= RCV_CANID_TENSION;
		break;
	case CANID_CABLE_ANGLE:		// 0x3A000000	 464 
		msgsused.lastrcvdCableAngle.can = *pcan;
                measurements.calibCableAngle = ((float)pcan->cd.uc[0] - scaleoffset.cableAngleOffset) * scaleoffset.cableAngleScale;
                if (statestuff.taperFlag == 0 && measurements.calibCableAngle > statevar.TAPERANGLETRIG)
                {
                	statestuff.taperFlag = 1;
                	statestuff.taperTics = simulationvar.elapsedTics;
                }
		msgsused.lastrcvdCableAngle.flag += 1;
		msgrcvlist |= RCV_CANID_CABLE_ANGLE;
		break;
	case CANID_MOTOR_SPEED:		// 0x25000000	 296 
		msgsused.lastrcvdMotorSpeed.can = *pcan;
		measurements.calibMotorSpeed = (float)pcan->cd.us[0] * scaleoffset.motorSpeedScale;
		measurements.calibCableSpeed = (float) (2 * 3.14159 * scaleoffset.drumRadius * measurements.calibMotorSpeed / scaleoffset.motorToDrum);
		msgsused.lastrcvdMotorSpeed.flag += 1;
		msgrcvlist |= RCV_CANID_MOTOR;
		break;
	case CANID_LAUNCH_PARAM:	// 0x28E00000	 327 
		msgsused.lastrcvdLaunchParam.can = *pcan;
		msgsused.lastrcvdLaunchParam.flag += 1;
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
	xprintf(UXPRT,"Going to state: %d\n\r", statestuff.state);
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

	if (petmcvar->timeFlag > 0)
	{ // Here a time msg has been sent
		petmcvar->timeFlag = 0;	// Reset flag
		msgrcvlist = 0;	// Show that no needed msgs have been received
	}

                //  next Time message time                   
                simulationvar.nextStepTime += statevar.STEPTIMEMILLIS;

                //  read messages until tension and motor messages are received
                //  note: if there were other messages after the last tension 
                //  and motor messages (e.g., cable angle, they will be read 
                //  after the next time message.  This will be remidied by 
                //  threading the read operations in the future
                // System.out.println("System Time before reads (ms): "
                //         + (long) System.currentTimeMillis());

// ????	if ((msgrcvlist & RCV_ALLNEEDEDMSGS) == RCV_ALLNEEDEDMSGS )


                switch (statestuff.state)
                {
                    case 0: // prep                        
			if ( calib_control_lever_get() < (4095/10) )
                        { 
                            statestuff.state = 1; // going to armed state
                        //    // setStateled(1);	// ??? LED
                            sendStateMessage(1);
			    mc_debug_print();
                        }
                        break;
                    case 1: // armed
                        
                        if ((statestuff.parametersRequestedFlag == 0) 
                                && (calib_control_lever_get() > (4095/90)))
                        {
                            // request launch parameters
				can.id = CANID_PARAM_REQUEST;
				can.dlc = 0;
				msg_out_mc(&can);
                            	statestuff.parametersRequestedFlag = 1;
                        }
                        // when we get the response, start the simulation
                        if ((msgrcvlist & CANID_LAUNCH_PARAM) != 0 )
                        {
                            simulationvar.simulationStartTime = (double) DTWTIME;
                            
                            statestuff.state = 2;
                            // // setStateled(2); 	// LED ???
                            statestuff.startProfileTics = simulationvar.elapsedTics;
                            sendStateMessage(2);
                            mc_debug_print();
                        }
                        break;
                    case 2: // profile 1
                        if (   (simulationvar.elapsedTics - statestuff.startProfileTics) >=
                               (statevar.SOFT_START_TIME * statevar.TICSPERSECOND) )
                        {
                            statestuff.state = 3;
                            statestuff.peakCableSpeed = measurements.calibCableSpeed;
                            // setStateled(3);
                            mc_debug_print();
                        }
                        break;
                    case 3: // profile 2

                        statestuff.peakCableSpeed = measurements.calibCableSpeed > statestuff.peakCableSpeed
                                ? measurements.calibCableSpeed : statestuff.peakCableSpeed;
                        //if (measurements.calibCableSpeed > statestuff.peakCableSpeed) {
                        //    statestuff.peakCableSpeed = measurements.calibCableSpeed;
                        //}

                        if (measurements.calibCableSpeed < (statestuff.peakCableSpeed * statevar.PEAK_CABLE_SPEED_DROP))
                        {
                            statestuff.state = 4;
                            statestuff.startRampTics = simulationvar.elapsedTics;
                            statestuff.startRampTension = measurements.calibTension;
                            sendStateMessage(4);
                            // setStateled(4);
                            mc_debug_print();
                        }
                        break;
                    case 4: // ramp
                        if (simulationvar.elapsedTics - statestuff.startRampTics > statevar.RAMP_TIME * statevar.TICSPERSECOND)
                        {
                            statestuff.state = 5;
                            // setStateled(5);
                            sendStateMessage(5);
                            statestuff.minCableSpeed = measurements.calibCableSpeed;
                            mc_debug_print();
                            statestuff.taperFlag = 0;
                        }
                        break;
                    case 5: // constant
                        xprintf(UXPRT,"%6d\n\r",measurements.calibCableSpeed);
                        if (measurements.calibCableSpeed < statestuff.minCableSpeed)
                        {
                            statestuff.minCableSpeed = measurements.calibCableSpeed;
                        }
                        if (measurements.calibCableSpeed > statestuff.minCableSpeed + statevar.RELEASEDELTA)
                        {
                            statestuff.state = 6;
                            // setStateled(6);
                            sendStateMessage(6);
                            mc_debug_print();
                        }
                        break;
                    case 6: // recovery
			xprintf(UXPRT,"%6d\n\r",measurements.calibCableSpeed);
                        if (measurements.calibCableSpeed < statevar.ZERO_CABLE_SPEED_TOLERANCE)
                        {
                            statestuff.state = 0;
                            // setStateled(0);
                            sendStateMessage(0);
                            mc_debug_print();
                            statestuff.launchResetFlag = 1;                            
                        }
                        break;
                }
                //  Template for Desired Tension and Control Law        
                switch (statestuff.state)
                {
                    case 0: // prep
                        statestuff.setptTension = 0;
                        break;

                    case 1: // armed
                        statestuff.setptTension = 0;
                        break;
                    case 2: // profile 1
                        statestuff.setptTension = (float) (statevar.GROUND_TENSION_FACTOR * statevar.GLIDER_WEIGHT * 0.5  * (1 - cosf(statevar.K1 * (simulationvar.elapsedTics - statestuff.startProfileTics))));
                        break;

                    case 3: // profile 2
                        // System.out.println(measurements.calibCableSpeed +  statevar.PROFILE_TRIGGER_CABLE_SPEED);
                        if (measurements.calibCableSpeed < statevar.PROFILE_TRIGGER_CABLE_SPEED)
                        {
                            statestuff.setptTension = statevar.GROUND_TENSION_FACTOR * statevar.GLIDER_WEIGHT;
                            xprintf(UXPRT,"%6d\n\r",statestuff.setptTension);	//  System.out.println(tension);
                        } 
			else
                        {
                            statestuff.setptTension = (float) (statevar.GROUND_TENSION_FACTOR * statevar.GLIDER_WEIGHT * cosf(statevar.K2 * (measurements.calibCableSpeed - statevar.PROFILE_TRIGGER_CABLE_SPEED)));
                            xprintf(UXPRT,"%6d\n\r",statestuff.setptTension);	// System.out.println(tension);
                        }
                        break;
                    case 4: // ramp
                        statestuff.setptTension = (float) ((statestuff.startRampTension
                                + (statevar.CLIMB_TENSION_FACTOR * statevar.GLIDER_WEIGHT
                                - statestuff.startRampTension)
                                * sinf(statevar.K3 * (simulationvar.elapsedTics - statestuff.startRampTics))));
                        xprintf(UXPRT,"%6d\n\r",statestuff.setptTension);	//  System.out.println(tension);
                        break;
                    case 5: // constant
                        statestuff.setptTension = (float) (statevar.CLIMB_TENSION_FACTOR * statevar.GLIDER_WEIGHT);
                        if (statestuff.taperFlag == 1)
                        {
                            statestuff.setptTension *= 0.4 + 0.6 * 0.5
                                    * (1 + cosf(statevar.K4 * (simulationvar.elapsedTics - statestuff.taperTics)));
                        }
                        break;
                    case 6: // recovery
                        statestuff.setptTension = statevar.MAX_PARACHUTE_TENSION;
                        if (measurements.calibCableSpeed > statevar.PROFILE_TRIGGER_CABLE_SPEED)
                        {
                            statestuff.setptTension *= cosf(statevar.K5 * (measurements.calibCableSpeed - statevar.PARACHUTE_TAPER_SPEED));
                            xprintf(UXPRT,"%6d\n\r",statestuff.setptTension);	// System.out.println(tension);
                        }
                        break;
                    
                }
                statestuff.setptTension *= (float)calib_control_lever_get() / 4095.0 ; // scale by slider 

               //  filter the torque with about 1 Hz bandwidth
                statestuff.setptTorque = (statestuff.setptTension * statevar.TENSION_TO_TORQUE); 
                statestuff.filt_torque += ((statestuff.setptTorque - statestuff.filt_torque) / 8);

		// torqueMessage.set_short((short) (statestuff.filt_torque / scaleoffset.torqueScale), 0); // torque
         	can.id = CANID_TORQUE;
		can.dlc = 2;
		can.cd.us[0] = (short)(statestuff.filt_torque / scaleoffset.torqueScale);
		msg_out_mc(&can);
            
}

/* **************************************************************************************
 * void mc_state_lcd_poll(struct ETMCVAR* petmcvar);
 * @brief	: Output LCD data
 * @param	: petmcvar = pointer to variables in etmc0
 * ************************************************************************************** */
void mc_state_lcd_poll(struct ETMCVAR* petmcvar)
{
	char lcdLine[LCDLINESIZE + 1];
	if (((int)(DTWTIME - t_lcd)) > 0) {
		t_lcd += LCDPACE;
		snprintf(lcdLine, 20, "State %4d", statestuff.state); 		lcd_printToLine(UARTLCD, 0, lcdLine);
xprintf(UXPRT,"%s ",lcdLine);
statestuff.setptTorque = calib_control_lever_get();
		snprintf(lcdLine, 20, "Torq: %10.3f", statestuff.setptTorque);	lcd_printToLine(UARTLCD, 1, lcdLine);
xprintf(UXPRT,"%s ",lcdLine);
		snprintf(lcdLine, 20, "Time: %d", petmcvar->unixtime);		lcd_printToLine(UARTLCD, 2, lcdLine);
xprintf(UXPRT,"%s \n\r",lcdLine);
	}
	return;
}

