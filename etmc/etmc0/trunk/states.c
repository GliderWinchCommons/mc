// constants / system parameters
float ZERO_CABLE_SPEED_TOLERANCE = 0.1;
float ZERO_DRUM_SPEED_TOLERANCE = 0.1;
float PERCENT_PEAK_CABLE_SPEED = 0.95;
int PREP_ERROR_TIME = 186000000 * 3;
int SAFE_ERROR_TIME = 186000000 * 3;
int ARMED_CONTROL_TIME = 186000000 * 3;
int LAUNCH_PARAMETER_TIME = 186000000;
int LEVER_WAIT_TIME = 186000000 * 3;
int PROFILE_RAMP_TIME = 186000000; // 1 second
int RAMP_TIME = 186000000 * 6; // 6 seconds

// variables needed
int prepSubState = 0;
int rampSubState = 0;
int safeSubState = 0;
int stopSubState = 0;
int armedSubState = 0;
int abortSubState = 0;
int retrieveSubState = 0;
int profileSubState = 0;
int recoverySubState = 0;
int constantSubState = 0;

int brakeStatus = 0;
int guillotineStatus = 0;
int tensiometerStatus = 0;

int calibrationMode = 0;
float peakCableSpeed = 0;
float startingRampTension = 0;

int prepErrorTimer = 0;
int safeErrorTimer = 0;
int armedControlTimer = 0;
int launchParameterTimer = 0;
int leverWaitTimer = 0;
int profileRampTimer = 0;
int rampTimer = 0;

int commandedTorque = 0;
float leverPosition = 0.0; // percentage forward that the lever is

// Launch Parameters
float Fg = 1.0; // ground run tension factor
float Fc = 1.3; // climb tension factor
float G = 9.81; // Gravity?
float gliderMass = 600; // kg
float gliderWeight = gliderMass * G;
int k = 2; // tension/torque conversion
float ki = 1 / k;
int k1 = 0; // based on time of ramp up
float k2 = 0.074; // sc in matlab
float k3 = (3.14159 / (2 * RAMP_TIME));
float k4 = 3.14159 / (2 * 1); // constant for quick stop on retrieve
int MAX_CABLE_SPEED = 35;
float PROFILE_TRIGGER_CABLE_SPEED = 20.578; // 40 knots
float retrieveTension = 50; // N



// PREP STATE
void prepState () {
	switch(prepSubState) {
		case 0:
			if (leverMicroSwitchBack == 1) {
				closeContactor();
				prepSubState = 1;
			} else {
				errorBeep();
				lcdLine3 = "lever advanced";
			}
			break;
		case 1:
			if (prepButton == 1) {
				nextState = 'retreive';
				prepSubState = 0;
			} else if (armButton == 1) {
				prepSubState = 2;
			}
			break;
		case 2:
			if (leverMicroSwitchBack == 1) {
				nextState = 'armedState';
				prepSubState = 0;
			} else {
				startPrepErrorTimer();
				errorBeep();
				lcdLine3 = "lever advanced";
				prepSubState = 3;
			}
			break;
		case 3:
			if (leverMicroSwitchBack == 1) {
				nextState = 'armedState';
				prepSubState = 0;
			} else if (((int)(*(volatile unsigned int *)0xE0001004 - prepErrorTimer)) > PREP_ERROR_TIME) {
				errorBeep();
				lcdLine3 = "Prep lever timeout";
				prepSubState = 1;
			}
			break;
	}
}

// retrieve state
void retrieveState () {
	switch (retrieveSubState) {
		case 0:
			if (abs(cableSpeed) < ZERO_CABLE_SPEED_TOLERANCE) {
				retrieveSubState = 1;
			}
			break;
		case 1:
			if (prepButton == 1) {
				nextState = "prep";
				retrieveSubState = 0;
			} else if (safeSwitch == 0) {
				nextState = "safe";
				retrieveSubState = 0;
			} else if (calibrationMode == 1) {
				nextState = "calibration";
				retrieveSubState = 0;
			}
			break;
	}
}

// safe state
void safeState () {
	switch (safeSubState) {
		case 0:
			openContactor();
			releaseBrakes();
			if (safeSwitch == 1) {
				lcdLine3 = "Launch:Switch Active";
				safeSubState = 1;
			} else {
				lcdLine3 = "Turn switch to safe";
				errorBeep();
				startSafeErrorTimer();
				safeSubState = 2;
			}
			break;
		case 1:
			if (safeSwitch == 0) {
				transitionBeep();
				nextState = "prep";
				safeSubState = 0;
			}
			break;
		case 2:
			if (safeSwitch == 1) {
				safeSubState = 1;
			} else if (((int)(*(volatile unsigned int *)0xE0001004 - safeErrorTimer)) > SAFE_ERROR_TIME) {
				errorBeep();
				startSafeErrorTimer();
			}
			break;
	}
}

// abort state
void abortState () {
	switch (abortSubState) {
		case 0:
			// enter abort tension control
			if (prepButton == 1) {
				nextState = "prep";
				abortSubState = 0;
			} else {
				if (safeSwitch == 1) {
					abortSubState = 1;
				}
			}
			break;
		case 1:
			if (abs(drumSpeed) < ZERO_DRUM_SPEED_TOLERANCE) {
				nextState = "safe";
				abortSubState = 0;
			} else {
				errorBeep();
				lcdLine3 = "Drum speed not 0";
				if (prepButton == 1) {
					nextState = "prep";
					abortSubState = 0;
				}
			}
			break;
	}
}

// constant state
void constantState () {
	switch (constantSubState) {
		case 0:
			if (armButton == 1) {
				nextState = "abort";
				profileSubState = 0;
			}
			if (brakeButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (guillotineButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (emergencyStopButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (leverMicroSwitchBack == 1) {
				nextState = "abort";
				profileSubState = 0;
			}

			if (((int)(*(volatile unsigned int *)0xE0001004 - cableSpeed)) >= MAX_CABLE_SPEED) {
				nextState = "recovery";
				constantSubState = 0;
			}
			break;
	}
}

// recovery state
void recoveryState () {
	switch (recoverySubState) {
		case 0:
			transitionBeep();
			lcdLine3 = "unknown message";
			recoverySubState = 1;
			break;
		case 1:
			if (abs(cableSpeed) < ZERO_CABLE_SPEED_TOLERANCE) { // waiting for speed to be 0
				recoverySubState = 2;
			}
			break;
		case 2:
			if (prepButton == 1) {
				nextState = "prep";
				recoverySubState = 0;
			}
			break;
	}
}

// ramp state
void rampState () {
	switch (rampSubState) {
		case 0:
			startingRampTension = tension;
			startRampTimer();
			rampSubState = 1;
			break;
		case 1:
			if (armButton == 1) {
				nextState = "abort";
				profileSubState = 0;
			}
			if (brakeButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (guillotineButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (emergencyStopButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (leverMicroSwitchBack == 1) {
				nextState = "abort";
				profileSubState = 0;
			}
			if (((int)(*(volatile unsigned int *)0xE0001004 - rampTimer)) > RAMP_TIME) {
				nextState = 'constant';
				rampSubState = 0;
			}
			break;
	}
}

// stop state
void stopState () {
	switch (stopSubState) {
		case 0:
			if (guillotineButton == 1) {
				fireGuillotine();
			}
			openContactor();
			applyBrakes();
			if (abs(drumSpeed) < ZERO_DRUM_SPEED_TOLERANCE) {
				stopSubState = 1;
			}
			break;
		case 1:
			releaseBrakes();
			if (safeSwitch == 0) {
				nextState = "prep";
				stopSubState = 0;
			} else {
				nextState = "safe";
				stopSubState = 0;
			}
			break;
	}
}

// profile state
void profileState () {
	switch (profileSubState) {
		case 0:
			startProfileRampTimer();
			profileSubState = 1;
			break;
		case 1:
			if (armButton == 1) {
				nextState = "abort";
				profileSubState = 0;
			}
			if (brakeButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (guillotineButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (emergencyStopButton == 1) {
				nextState = "stop";
				profileSubState = 0;
			}
			if (leverMicroSwitchBack == 1) {
				nextState = "abort";
				profileSubState = 0;
			}
			if (((int)(*(volatile unsigned int *)0xE0001004 - profileRampTimer)) > PROFILE_RAMP_TIME) {
				profileSubState = 2;
			}
			break;
		case 2:
			// find the max cable speed
			if (cableSpeed > peakCableSpeed) {
				peakCableSpeed = cableSpeed;
			}
			if (cableSpeed < peakCableSpeed * PERCENT_PEAK_CABLE_SPEED) {
				nextState = 'ramp';
				profileSubState = 0;
			}
			break;
	}
}

// armed state
void armedState () {
	switch (armedSubState) {
		case 0: // check to make sure everything is working ok
			applyBrakes();
			if (brakeStatus == 1) { // not normal
				errorBeep();
				lcdLine3 = "Brake Error!";
				armedSubState = 1;
			} else if (guillotineStatus == 1) { // not normal
				errorBeep();
				lcdLine3 = "Guillotine Error!";
				armedSubState = 1;
			} else if (tensiometerStatus == 1) { // not normal
				errorBeep();
				lcdLine3 = "Tensiometer Error!";
				armedSubState = 1;
			} else if (abs(drumSpeed) < ZERO_DRUM_SPEED_TOLERANCE) {
				startArmedControlTimer();
				armedSubState = 2;
			}
			break;
		case 1:
			if (abs(drumSpeed) < ZERO_DRUM_SPEED_TOLERANCE) {
				nextState = "safe";
				armedSubState = 0;
			} else {
				applyBrakes();
			}
			break;
		case 2:
			if (safeSwitch == 1) { // in safe position
				nextState = "safe";
				armedSubState = 0;
			} else if (prepButton == 1) {
				errorBeep();
				nextState = "prep";
				armedSubState = 0;
			} else if (armButton == 1) {
				errorBeep();
				nextState = "prep";
				armedSubState = 0;
			} else if (leverMicroSwitchFront == 1) {
				requestLaunchParameters();
				startLaunchParameterTimer();
				armedSubState = 3;
			} else if (((int)(*(volatile unsigned int *)0xE0001004 - armedControlTimer)) > ARMED_CONTROL_TIME) {
				errorBeep();
				lcdLine3 = "Armed time error";
				nextState = "prep";
				armedSubState = 0;
			}
			break;
		case 3:
			if (launchParametersComplete()) {
				armedSubState = 4;
			} else if (((int)(*(volatile unsigned int *)0xE0001004 - launchParameterTimer)) > LAUNCH_PARAMETER_TIME) {
				errorBeep();
				lcdLine3 = "No host response";
				nextState = "safe";
				armedSubState = 0;
			}
			break;
		case 4:
			if (leverMicroSwitchFront == 1) {
				computeLaunchProfile();
				nextState = "profile";
				armedSubState = 0;
			} else {
				errorBeep();
				lcdLine3 = "Advance Ctrl Lever";
				startLeverWaitTimer();
				armedSubState = 5;
			}
			break;
		case 5:
			if (leverMicroSwitchFront == 1) {
				computeLaunchProfile();
				nextState = "profile";
				armedSubState = 0;
			} else if (((int)(*(volatile unsigned int *)0xE0001004 - leverWaitTimer)) > LEVER_WAIT_TIME) {
				errorBeep();
				lcdLine3 = "Lever wait time over";
				nextState = "prep";
				armedSubState = 0;
			}
			break;
	}
}

void controlLaw () {
	if (currentState == 'retrieve') {
		if (cableSpeed <= 0) {
			commandedTorque = retrieveTension * ki;
		} else {
			commandedTorque = ki * retrieveTension * cos(k4 * cableSpeed);
		}

	} else if (currentState == 'profile') {
		if (profileSubState == 1) { // ramping
			commandedTorque = leverPosition * ( (Fg * gliderWeight) / (2 * k) ) * (1 - cos(2 * k1 * profileRampTimer));
		} else if (profileSubState == 2) {
			if (cableSpeed < PROFILE_TRIGGER_CABLE_SPEED) {
				commandedTorque = leverPosition * Fg * gliderWeight * ki;
			} else {
				commandedTorque = leverPosition * Fg * gliderWeight * ki * cos(k2 * (cableSpeed - PROFILE_TRIGGER_CABLE_SPEED));
			}
		}

	} else if (currentState = 'ramp') {
		commandedTorque = startingRampTension + (((Fc * gliderWeight * G) - startingRampTension) * sin(k3 * rampTimer));
		commandedTorque = commandedTorque * leverPosition * ki; // scale by lever position

	} else if (currentState = 'constant') {
		commandedTorque = leverPosition * Fc * gliderWeight * G * ki;

	} else if (currentState = 'recovery') {
		commandedTorque = leverPosition * Fg * gliderWeight * ki * cos(k2 * (cableSpeed - PROFILE_TRIGGER_CABLE_SPEED));

	} else if (currentState = 'stop') {
		commandedTorque = -10000 * ki;
	}
}

// Other functions

void errorBeep () { // double beep
}

void transitionBeep () { // single beep
}

void closeContactor () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x23000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0xFF; // close it

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

void openContactor () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x23000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0x00; // open it

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

void releaseBrakes () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x21000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0x00; // release

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

void applyBrakes () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x21000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0xFF; // apply

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

void fireGuillotine () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x22000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0xFF; // fire

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

void requestLaunchParameters () {
	struct CANRCVBUF can;
	int tmp;

	can.id       = 0x27000000;
	can.dlc      = 0x00000001;
	can.cd.us[0] = 0x00; // nothing

	tmp = CAN_gateway_send(&can);
	canbuf_add(&can);
}

bool launchParametersComplete () {
	// Placeholder, eventually this will check to see if all launch parameters
	// have been recieved from the host controller.

	// For now, just return true.
	return true;
}

void computeLaunchProfile () {
	// assign launch parameters from messages
	// nothing in our system
}

// timer functions

void startLeverWaitTimer () {
	leverWaitTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startLaunchParameterTimer () {
	launchParameterTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startArmedControlTimer () {
	armedControlTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startSafeErrorTimer () {
	safeErrorTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startPrepErrorTimer () {
	prepErrorTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startProfileRampTimer () {
	profileRampTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}

void startRampTimer () {
	rampTimer = ((int)(*(volatile unsigned int *)0xE0001004));
}