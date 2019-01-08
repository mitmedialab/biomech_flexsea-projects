/*
 * safety_functions.c
 *
 *  Created on: Jan 8, 2019
 *      Author: matt
 */

#include "safety_functions.h"

static int8_t errorConditions[ERROR_ARRAY_SIZE]; //massage extern until it works
static int16_t safetyFlags; //bitmap of all errors. Also serves as boolean for error existence

int8_t* getSafetyConditions(void) {
	return errorConditions;
}

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
		errorConditions[ERROR_BATTERY_VOLTAGE] = VALUE_BELOW;
	} else if (rigid1.re.vb >= UVHI_BIOMECH) {
		errorConditions[ERROR_BATTERY_VOLTAGE] = VALUE_ABOVE;
	} else {
		errorConditions[ERROR_BATTERY_VOLTAGE] = VALUE_NOMINAL;
	}
}

static void checkTorqueMeasuredBounds(Act_s *actx) {
	//if sensors are invalid, torque value is invalid
	if (errorConditions[ERROR_LDC] || errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[ERROR_TORQUE_MEASURED] = SENSOR_INVALID;
	} else {
		if (actx->jointTorque >= ABS_TORQUE_LIMIT_INIT) {
			errorConditions[ERROR_TORQUE_MEASURED] = VALUE_ABOVE;
		} else if (actx->jointTorque <= -ABS_TORQUE_LIMIT_INIT) {
			errorConditions[ERROR_TORQUE_MEASURED] = VALUE_BELOW;
		} else {
			errorConditions[ERROR_TORQUE_MEASURED] = VALUE_NOMINAL;
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

static void checkJointAngleBounds(Act_s *actx) {
	if (errorConditions[ERROR_JOINT_ENCODER]) {
		errorConditions[ERROR_JOINTANGLE_SOFT] = SENSOR_INVALID;
		errorConditions[ERROR_JOINTANGLE_HARD] = SENSOR_INVALID;
	} else {
		//soft angle check
		if (actx->jointAngleDegrees <= JOINT_MIN_SOFT_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_SOFT] = VALUE_BELOW;
		} else if (actx->jointAngleDegrees >= JOINT_MAX_SOFT_DEGREES) {
			errorConditions[ERROR_JOINTANGLE_SOFT] = VALUE_ABOVE;
		} else {

			errorConditions[ERROR_JOINTANGLE_SOFT] = VALUE_NOMINAL;
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

//check for general errors
int checkActuator() {
	return (getDeviceId() == STM32_DEVICE_ID && safetyFlags == 0);
}

static void checkPersistentError(Act_s *actx) {
	//time limit for errors
}

void checkSafeties(Act_s *actx) {
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

	checkPersistentError(actx);
}

