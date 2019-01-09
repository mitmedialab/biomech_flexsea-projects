/*
 * safety_functions.c
 *
 *  Created on: Jan 8, 2019
 *      Author: matt
 */

#include "safety_functions.h"

//bitshift macros TODO: fix
//#define IS_FIELD_HIGH(i, map) ( (map) & (1 << ((i)%16)) )
//#define SET_MAP_HIGH(i, map) ( (map) |= (1 << ((i)%16)) )
//#define SET_MAP_LOW(i, map) ( (map) &= (~(1 << ((i)%16))) )

//variables
static int8_t errorConditions[ERROR_ARRAY_SIZE]; //massage extern until it works
static int16_t safetyFlags; //bitmap of all errors. Also serves as boolean for error existence
static int8_t motorMode;



//check connected/disconnect status
//TODO: find ways of actually checking these
static void checkLoadcell(Act_s *actx) {
	errorConditions[ERROR_LDC] = SENSOR_NOMINAL;

	//add isSafe flag setting when we can actually check sensors
}

static void checkJointEncoder(Act_s *actx) {
	errorConditions[ERROR_JOINT_ENCODER] = SENSOR_NOMINAL;
}

static void checkMotorEncoder(Act_s *actx) {
	errorConditions[ERROR_MOTOR_ENCODER] = SENSOR_NOMINAL;
}

static void checkMotorThermo(Act_s *actx) {
	errorConditions[ERROR_MOTOR_THERMO] = SENSOR_NOMINAL;
}

static void checkPCBThermo(Act_s *actx) {
	errorConditions[ERROR_PCB_THERMO] = SENSOR_NOMINAL;
}

static void checkEMG(Act_s *actx) {
	errorConditions[ERROR_EMG] = SENSOR_NOMINAL;
}

//check values against limits
static void checkBatteryBounds(Act_s *actx) {
	if (rigid1.re.vb <= UVLO_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_BELOW;
	} else if (rigid1.re.vb >= UVHI_BIOMECH) {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_ABOVE;
	} else {
		errorConditions[WARNING_BATTERY_VOLTAGE] = VALUE_NOMINAL;
	}
}

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

static void checkCurrentMeasuredBounds(Act_s *actx) {
	if (abs(actx->motCurr) >= CURRENT_LIMIT_INIT) {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_ABOVE;
	} else {
		errorConditions[ERROR_CURRENT_MEASURED] = VALUE_NOMINAL;
	}
}

static void checkTemperatureBounds(Act_s *actx) {
	if (actx->regTemp >= PCB_TEMP_LIMIT_INIT){
		errorConditions[ERROR_PCB_THERMO] = VALUE_ABOVE;
	}else{
		errorConditions[ERROR_PCB_THERMO] = VALUE_NOMINAL;
	}
}


static void checkJointAngleBounds(Act_s *actx) {
	if (errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[WARNING_JOINTANGLE_SOFT] = SENSOR_INVALID;
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

static void checkPersistentError(Act_s *actx) {
	//time limit for errors
}
/*
 * PUBLIC FUNCTIONS
 */

int8_t getMotorMode(void){
	return motorMode;
}

int8_t* getSafetyConditions(void) {
	return errorConditions;
}

//check for general errors
int actuatorIsCorrect() {
	return (getDeviceId() == STM32_DEVICE_ID && safetyFlags == 0);
}



void checkSafeties(Act_s *actx) {
	safetyFlags = 0; //reset this upon entering a check
	//TODO:

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

//	//set our safety bitmap
//	//TODO: consider optimizing if there are future processing constraints
//	for (int i = 0; i++; i < ERROR_ARRAY_SIZE) {
//		if (errorConditions[i]) {
//			SET_MAP_HIGH(i, safetyFlags);
//		}
//	}
}

/*
 * Check for safety flags, and act on them.
 * todo: come up with correct strategies to deal with flags, include thermal limits also
 */
int8_t handleSafetyConditions(void) {
	int8_t isBlocking = 1;

	//TODO figure out if MODE_DISABLED should be blocking/ how to do it
	if (errorConditions[ERROR_MOTOR_ENCODER] != SENSOR_NOMINAL)
		motorMode = MODE_DISABLED;
	else if (errorConditions[ERROR_JOINT_ENCODER] ||
			errorConditions[ERROR_LDC])
		motorMode = MODE_PASSIVE;
	else if (errorConditions[ERROR_PCB_THERMO] == VALUE_ABOVE ||
			errorConditions[ERROR_MOTOR_THERMO] != VALUE_NOMINAL)
		motorMode = MODE_THROTTLED;
	else
		motorMode = MODE_ENABLED;

	if (errorConditions[WARNING_TORQUE_MEASURED] != VALUE_NOMINAL){
		//led0
	}

	if (errorConditions[WARNING_JOINTANGLE_SOFT] != VALUE_NOMINAL){
		//led1
	}

	if (errorConditions[WARNING_BATTERY_VOLTAGE] != VALUE_NOMINAL){
		//led2
	}

	if (errorConditions[ERROR_PCB_THERMO] != VALUE_NOMINAL){
		//led3
	}

	switch (motorMode){
		case MODE_DISABLED:
			disable_motor();
			break;
		case MODE_PASSIVE:
			actuate_passive_mode(); //position control to neutral angle
			break;
		case MODE_THROTTLED:
			throttle_current();
			isBlocking = 0;
			break;
		case MODE_ENABLED:
			isBlocking = 0;
			break;
		default:
			break;
	}

	return isBlocking;

}

