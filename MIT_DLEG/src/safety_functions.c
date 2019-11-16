/*
 * safety_functions.c
 *
 *  Created on: Jan 8, 2019
 *      Author: matt
 */


//****************************************************************************
// Include(s)
//****************************************************************************
#include "safety_functions.h"
#include "global-config.h"

//****************************************************************************
// Definitions
//****************************************************************************
//bitshift macros TODO: test
#define IS_FIELD_HIGH(i, map) ( (map) & (1 << ((i)%16)) )
#define SET_MAP_HIGH(i, map) ( (map) |= (1 << ((i)%16)) )
#define SET_MAP_LOW(i, map) ( (map) &= (~(1 << ((i)%16))) )

//****************************************************************************
// Variable(s)
//****************************************************************************
static int8_t errorConditions[ERROR_ARRAY_SIZE]; //massage extern until it works
static int16_t safetyFlags = 0; //bitmap of all errors. Also serves as boolean for error existence
static int8_t motorMode;
static const int16_t stm32ID[] = STM32ID;

struct staticsig_s staticsig_motVolt;	// check motor voltage for e-stop safety

//****************************************************************************
// Method(s)
//****************************************************************************

//nn is the number of function calls that x must be unchanged for it to return 1
void init_staticsig(struct staticsig_s * ss, int32_t nn)
{
	ss->x = 0;
	ss->n = nn;
	ss->nx = 0;
}

//x is the new signal value
uint8_t update_staticsig(int32_t x, struct staticsig_s * ss)
{
	if(ss->x == x)
	{
		ss->nx++;
	}
	else
	{
		ss->nx = 0;
	}
	ss->x = x;

	if(ss->nx >= ss->n)
	{
		return 1;
	}
	return 0;
}



/*
 *  check connected/disconnect status
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *  TODO: find ways of actually checking these
 */
static void checkLoadcell(Act_s *actx) {
	static uint16_t previousStrainValue = 0;
	static int8_t tripped = 0;
	static uint32_t delayTimer; //TODO: possible to disconnect load cell at t = 0 upon rollover

	//check to see if loadcell is disconnected
	if (abs((int32_t)rigid1.ex.strain - (int32_t)previousStrainValue) >= LOADCELL_DISCONNECT_STRAIN_DIFFERENCE
			&& !tripped && delayTimer >= LOADCELL_DISCONNECT_COUNT_THRESHOLD) {
		errorConditions[ERROR_LDC] = SENSOR_DISCONNECT;
		tripped = 1;
	} else if (!tripped) {
		errorConditions[ERROR_LDC] = SENSOR_NOMINAL;
	}

	previousStrainValue = rigid1.ex.strain;
	delayTimer++;
}

/*
 *  Description
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkJointEncoder(Act_s *actx) {
	static uint16_t disconnectCounter = 0;
	static int16_t previousJointValue = 0;

	if (abs(*rigid1.ex.joint_ang - previousJointValue) <= JOINT_ANGLE_DIFF_VALUE) {
		disconnectCounter++;
	} else {
		disconnectCounter = 0;
	}

	if (disconnectCounter >= JOINT_ANGLE_COUNT_THRESHOLD) {
		errorConditions[ERROR_JOINT_ENCODER] = SENSOR_DISCONNECT;
	} else {
		errorConditions[ERROR_JOINT_ENCODER] = SENSOR_NOMINAL;
	}

	previousJointValue = *rigid1.ex.joint_ang;
}

/*
 *  Description
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkMotorEncoder(Act_s *actx) {
	static uint16_t disconnectCounter = 0;
//	static int32_t previousMotorValue = 0;
//	static float previousJointValue = 0;

	if (abs(rigid1.ex.mot_current) >= MOTOR_CURRENT_DISCONNECT_THRESHOLD &&
			fabs(actx->axialForce) <= MOTOR_ENCODER_DISCONNECT_AXIAL_FORCE_THRESHOLD_N) {
		disconnectCounter++;
	} else {
		disconnectCounter = 0;
	}

	if (disconnectCounter >= MOTOR_ANGLE_COUNT_THRESHOLD ||
			abs(*rigid1.ex.enc_ang) > MOTOR_ENCODER_DISCONNECT_BOUND) {
		errorConditions[ERROR_MOTOR_ENCODER] = SENSOR_DISCONNECT;
	} else if (errorConditions[ERROR_MOTOR_ENCODER] != SENSOR_DISCONNECT){
		errorConditions[ERROR_MOTOR_ENCODER] = SENSOR_NOMINAL;
	}

}

/*
 *  Description
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkMotorThermo(Act_s *actx) {
	errorConditions[ERROR_MOTOR_THERMO] = SENSOR_NOMINAL;
}

/*
 *  Description
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkPCBThermo(Act_s *actx) {
	errorConditions[ERROR_PCB_THERMO] = SENSOR_NOMINAL;
}

/*
 *  Description
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkEMG(Act_s *actx) {
	errorConditions[ERROR_EMG] = SENSOR_NOMINAL;
}

//check values against limits
/*
 *  Checks to see if the voltage is within the bounds of the battery
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkBatteryBounds(Act_s *actx) {

	if (rigid1.re.vb <= UVLO_NOTIFY && rigid1.re.vb >= UV_USB_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_BELOW;
	} else if (rigid1.re.vb >= UVHI_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_ABOVE;
	} else {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_NOMINAL;
	}
}

/*
 *  Checks to see if the measured torque is within the bounds of the device
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkTorqueMeasuredBounds(Act_s *actx) {
	//if sensors are invalid, torque value is invalid
	if (errorConditions[ERROR_LDC] || errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[WARNING_TORQUE_MEASURED] = SENSOR_INVALID;
	} else {
		if (actx->jointTorque >= ABS_TORQUE_LIMIT_INIT) {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_ABOVE;
		} else if (actx->jointTorque <= -ABS_TORQUE_LIMIT_INIT) {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_BELOW;
		} else {
			errorConditions[WARNING_TORQUE_MEASURED] = VALUE_NOMINAL;
		}
	}
}

/*
 *  Checks to see if the measured current is within the bounds of the device
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkCurrentMeasuredBounds(Act_s *actx) {
	if (abs(actx->motCurr) >= CURRENT_LIMIT_INIT) {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_ABOVE;
	} else {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_NOMINAL;
	}
}

/*
 *  Checks to see if the measured temperature is within the bounds of the device
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkTemperatureBounds(Act_s *actx) {
	if (actx->regTemp >= PCB_TEMP_LIMIT_INIT){
		errorConditions[ERROR_PCB_THERMO] = VALUE_ABOVE;
	}else{
		errorConditions[ERROR_PCB_THERMO] = VALUE_NOMINAL;
	}
}

/*
 *  Checks to see if the joint angle is within the bounds of the device
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkJointAngleBounds(Act_s *actx) {
	if (errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[WARNING_JOINTANGLE_SOFT] = SENSOR_INVALID;
	} else {
		//soft angle check
		if (actx->jointAngleDegrees <= JOINT_MIN_SOFT_DEGREES) {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_BELOW;
		} else if (actx->jointAngleDegrees >= JOINT_MAX_SOFT_DEGREES) {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_ABOVE;
		} else {
			errorConditions[WARNING_JOINTANGLE_SOFT] = VALUE_NOMINAL;
		}

		//hard angle check
		if (actx->jointAngleDegrees <= JOINT_MIN_HARD_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_BELOW;
		} else if (actx->jointAngleDegrees >= JOINT_MAX_HARD_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_ABOVE;
		} else {
			errorConditions[ERROR_JOINTANGLE_HARD] = VALUE_NOMINAL;
		}
	}
}

/*
 *  Definition
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
static void checkPersistentError(Act_s *actx) {
	//time limit for errors
}

/*
 * Check if E-stop has been pressed. This sets motor drive to CTRL=NULL
 * CAREFUL!!! todo: clear all static variables before reseting the drive control.
 *
 * Return 1 if pressed
 * return 0 if not pressed
 */
static void checkEmergencyStop(Act_s *actx)
{
	uint16_t eStopPressed = rigid1.ex.status & 0x80;

	uint16_t eStopWindow =0;

	eStopWindow = checkEmergencyStopWindow(eStopPressed);	// store the last ESTOP_WINDOW number of samples. if not cleared then don't clear estop

	if(eStopPressed)
	{
		errorConditions[ERROR_EMERGENCY_SAFETY_STOP] = SENSOR_DISCONNECT;
		actx->resetStaticVariables = 1;		// flag to tell functions to reset static variables
		actx->eStop = 1;


	}
	else if(eStopWindow == 0)
	{
		errorConditions[ERROR_EMERGENCY_SAFETY_STOP] = SENSOR_NOMINAL;
		actx->resetStaticVariables = 0;
		actx->eStop = 0;
	}
}

/*
 * Window the eStop button, we use this to store the last N samples
 */

int16_t checkEmergencyStopWindow(uint16_t inputVal)
{
	int16_t result = 0;
	int16_t i = 0;

	static uint32_t circStopCounter;
	static int32_t counterIndex;
	static uint32_t stopWindow[ESTOP_WINDOW_RELEASED];

	++circStopCounter;	//increment circular counter
	if(circStopCounter == ESTOP_WINDOW_RELEASED)
	{
		circStopCounter = 0;	//reset counter
	}

	stopWindow[circStopCounter] = inputVal;		// load new value into bottom of index
	counterIndex = circStopCounter;


	// using the accumulator,
	for (i = 0; i < ESTOP_WINDOW_RELEASED;  i++)
	{
		result +=  stopWindow[counterIndex];
		--counterIndex;
		if(counterIndex == -1)
		{
			counterIndex = ESTOP_WINDOW_RELEASED-1;
		}
	}

	return result;
}

//****************************************************************************
// Public Function(s)
//****************************************************************************


/*
 * Reduce the current limit in order to reduce heating in the motor or drive electronics.
 * This should then reset current as necessary
 * Param: actx(Act_s) - Actuator structure to track sensor values
 */
static void throttleCurrent(Act_s *actx) {
	actx->currentOpLimit = actx->currentOpLimit - 2;
	if (actx->currentOpLimit < CURRENT_LIMIT_MIN) {
		actx->currentOpLimit = CURRENT_LIMIT_MIN;
	}
}

/*
 * Ramp the current limit if below temp threshold.
 * This should then reset current as necessary
 * Param: actx(Act_s) - Actuator structure to track sensor values
 */
static void rampCurrent(Act_s *actx) {
	actx->currentOpLimit += 2;
	if (actx->currentOpLimit > CURRENT_LIMIT_INIT) {
		actx->currentOpLimit = CURRENT_LIMIT_INIT;
	}
}


/*
 * Disable the motor because of some safety flag
 */
static void disableMotor(void) {
	//CTRL_NONE gives desired damping behavior
	setControlMode(CTRL_NONE, DEVICE_CHANNEL);
}

/*
 *  Sets LEDS 1,2, and 3 to new values
 *  Param: l1Status(unint8_t) - desired integer value representing the status of LED1
 *  Param: l2Status(unint8_t) - desired integer value representing the status of LED2
 *  Param: l3Status(unint8_t) - desired integer value representing the status of LED3
 *
 */
void setLEDStatus(uint8_t l0Status, uint8_t l1Status, uint8_t l2Status) {
	l0 |= l0Status;
	l1 |= l1Status;
	l2 |= l2Status;
}

/*
 *  Sets LEDS 1,2, and 3 to 0
 *
 */
void clearLEDStatus(void) {
	l0 = 0;
	l1 = 0;
	l2 = 0;
}

/*
 *  sets LED RGB values to new boolean system 1(on) or 0(off)
 *  Param: r(unint8_t) - red value of the LED
 *  Param: g(unint8_t) - green value of the LED
 *  Param: b(unint8_t) - blue value of the LED
 *
 */
void overrideLED(uint8_t r, uint8_t g, uint8_t b) {
	if (r >= 1) {
		LEDR(1);
	} else {
		LEDR(0);
	}

	if (g >= 1) {
		LEDG(1);
	} else {
		LEDG(0);
	}

	if (b >= 1) {
		LEDB(1);
	} else {
		LEDB(0);
	}
}

/*
 *  Gets the mode that the motor is in
 *  Return: motorMode(int8_t) - integer value representing the mode that the motor is in
 *
 */
int8_t getMotorMode(void){
	return motorMode;
}

void setMotorMode(int8_t mode){
	motorMode = mode;
}

/*
 *  Gets the current safety conditions for the system
 *  Return: errorConditions(int8_t) - integer value representing the current safety conditions for the system
 *
 */
int8_t* getSafetyConditions(void) {
	return errorConditions;
}

/*
 *  Gets the current safety flags for the system
 *  Return: safetyFlags(int16_t) - integer value representing the current safety flags for the system
 *
 */
int16_t getSafetyFlags(void) {
	return safetyFlags;
}


/*
 *  check for general errors
 */
int actuatorIsCorrect() {
	int16_t* devID16 = getDeviceId16();

	// If we are testing devices don't cancel all actions, probably need to be doing tests.
	#if defined(NO_DEVICE) || defined(NO_ACTUATOR)
	#else
		if (safetyFlags) {
			return 0;
		}
	#endif

	//check all six values of stm32ID match
	for (int i = 0; i < 6; i++) {
		if (*(devID16 + i) != stm32ID[i]) {
			return 0;
		}
	}

	return 1;
}

/*
 * Output device ID so we can record it for new boards.
 * Step through each ID, hold for a delay, then increment.
 * Output of -1 defines end of the ID and a restart of ID.
 */
int16_t getDeviceIdIncrementing(void) {
	static int16_t internalTimer = 0;
	static int8_t i = 0;
	int16_t* devID16 = getDeviceId16();
	int16_t output = 0;

	internalTimer++;

	if (internalTimer < 1500)
	{
		if (i < 6)
		{
			output =  (*(devID16 + i));

		} else
		{
			i = -1;
			output = -1;	// notification of restarting output
		}
	} else
	{
		internalTimer = 0; //reset timer
		i++;
	}
	return output;
}



/*
 *  Makes all safety checks
 *  Param: actx(Act_s) - Actuator structure to track sensor values
 *
 */
void checkSafeties(Act_s *actx)
{
	safetyFlags = 0; //reset this upon entering a check

	checkLoadcell(actx);
	checkJointEncoder(actx);
	checkMotorEncoder(actx);
	checkMotorThermo(actx);
	checkPCBThermo(actx);
	checkEMG(actx);

	checkBatteryBounds(actx);
	checkTorqueMeasuredBounds(actx);
	checkCurrentMeasuredBounds(actx);
	checkJointAngleBounds(actx); //hard and soft limits
	checkTemperatureBounds(actx);

	checkPersistentError(actx);

	checkEmergencyStop(actx);

	//set our safety bitmap for streaming and checking purposes
	//TODO: consider optimizing if there are future processing constraints
	for (int i = 0; i < ERROR_ARRAY_SIZE; i++)
	{
		if (errorConditions[i])
		{
			SET_MAP_HIGH(i, safetyFlags);
		}
	}
}


/*
 * Check for safety flags, and act on them.
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 * todo: this is very DANGEROUS difficult to know state when we turn motor on again.
 */
void handleSafetyConditions(Act_s *actx) {

	static int8_t lastMotorMode = MODE_ENABLED;

#if defined(NO_DEVICE) || defined(NO_ACTUATOR)
#else
	//TODO figure out if MODE_DISABLED should be blocking/ how to do it
	if (errorConditions[ERROR_MOTOR_ENCODER] != SENSOR_NOMINAL)
		motorMode = MODE_DISABLED;
	else if (errorConditions[ERROR_JOINT_ENCODER] ||
			errorConditions[ERROR_LDC])
		motorMode = MODE_PASSIVE;
	else if (errorConditions[ERROR_PCB_THERMO] == VALUE_ABOVE ||
			errorConditions[ERROR_MOTOR_THERMO] != VALUE_NOMINAL ||
			actx->currentOpLimit < CURRENT_LIMIT_INIT)
		motorMode = MODE_OVERTEMP;
	else {
		motorMode = MODE_ENABLED;
	}

	//TODO: get LEDS working
	if (errorConditions[WARNING_TORQUE_MEASURED] != VALUE_NOMINAL){
		setLEDStatus(1,0,0); //flashing yellow
	}

	if (errorConditions[WARNING_JOINTANGLE_SOFT] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow
	}

	if (errorConditions[WARNING_BATTERY_VOLTAGE] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow (TODO LED function is not working)
	}

	if (errorConditions[ERROR_PCB_THERMO] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow
	}

	switch (motorMode){
		case MODE_DISABLED:
			// todo: DEBUG was causing issues, based on joint Encoder most likely. Need to work with Dephy to get comm bus checking for error handling

			break;
		case MODE_PASSIVE:
			// todo: DEBUG was causing issues, based on joint Encoder most likely. Need to work with Dephy to get comm bus checking for error handling
			break;
		case MODE_OVERTEMP:
			if (errorConditions[ERROR_PCB_THERMO] == VALUE_ABOVE ||
				errorConditions[ERROR_MOTOR_THERMO] != VALUE_NOMINAL) {
				throttleCurrent(actx);
			} else {
				rampCurrent(actx);
			}
			break;
		case MODE_ENABLED: // VERY DANGEROUS todo: need graceful way to turn back on, this can cause unexpected behavior. Turned off for now. Maybe best to latch into failed mode.
			if (lastMotorMode != MODE_ENABLED)	// turn motor mode back on.
			{

			}
			break;
	}
	lastMotorMode = motorMode;

#endif // NO_DEVICE || NO_ACTUATOR
}

/*
 * Check for safety flags, and act on them.
 * Param: actx(Act_s) - Actuator structure to track sensor values
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
void handleSafetyConditionsMinimal(Act_s *actx) {

	static int8_t lastMotorMode = MODE_ENABLED;

#if defined(NO_DEVICE) || defined(NO_ACTUATOR)
#else

	//TODO: get LEDS working
	if (errorConditions[WARNING_TORQUE_MEASURED] != VALUE_NOMINAL){
		setLEDStatus(1,0,0); //flashing yellow
	}

	if (errorConditions[WARNING_JOINTANGLE_SOFT] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow
	}

	if (errorConditions[WARNING_BATTERY_VOLTAGE] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow (TODO LED function is not working)
	}

	if (errorConditions[ERROR_PCB_THERMO] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow
	}

	if (errorConditions[ERROR_JOINTANGLE_HARD] != VALUE_NOMINAL){
		setLEDStatus(1,0,0);//flashing yellow
		setMotorMode(MODE_DISABLED);
	}


	switch (motorMode){
		case MODE_DISABLED:
		{
			// todo: DEBUG was causing issues, based on joint Encoder most likely. Need to work with Dephy to get comm bus checking for error handling

			break;
		}
		case MODE_ENABLED:
		{
			if (lastMotorMode != MODE_ENABLED)	// turn motor mode back on. todo: this is totally unsafe, need more careful way to clear errors and turn on again.
			{

			}
			break;
		}
	}
	lastMotorMode = motorMode;

#endif // NO_DEVICE || NO_ACTUATOR
}

