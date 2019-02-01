/*
 * safety_functions.h
 *
 *  Created on: Jan 8, 2019
 *      Author: matt
 */
#ifndef FLEXSEA_PROJECTS_MIT_DLEG_INC_SAFETY_FUNCTIONS_H_
#define FLEXSEA_PROJECTS_MIT_DLEG_INC_SAFETY_FUNCTIONS_H_

#include <stdint.h>
#include "main.h"
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "mn-MotorControl.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
#include "flexsea_user_structs.h"
#include <math.h>
#include "flexsea_board.h"
#include "misc.h"
#include "user-mn-MIT-DLeg.h"

#include "ui.h"

#if (ACTIVE_PROJECT == PROJECT_MIT_DLEG)//FIXME: Why is this guard not working?


//methods
int8_t getMotorMode(void);
int8_t* getSafetyConditions(void);
int actuatorIsCorrect();
void checkSafeties(Act_s *actx);
void handleSafetyConditions(Act_s *actx); //renamed from safetyFailure(void)
int16_t getSafetyFlags(void);

void setLEDStatus(uint8_t l1_status, uint8_t l2_status, uint8_t l3_status);
void clearLEDStatus(void);
void overrideLED(uint8_t r, uint8_t g, uint8_t b);


//enums
enum MOTOR_MODES{
	MODE_DISABLED = 0,
	MODE_PASSIVE = 1,
	MODE_OVERTEMP = 2,
	MODE_ENABLED = 3,
};

enum VALUE_STATUS{
	VALUE_NOMINAL = 0,
	SENSOR_NOMINAL = 0,
	VALUE_BELOW = -1,
	VALUE_ABOVE = 1,
	SENSOR_INVALID = 2,
	SENSOR_DISCONNECT = 3,
};

//TODO discuss if warnings and errors should be in separate enums
enum ERROR_TYPES{
	//sensor disconnect errors
	ERROR_LDC				= 0, //load cell
	ERROR_JOINT_ENCODER 	= 1,
	ERROR_MOTOR_ENCODER 	= 2, //TODO: need to revisit this because difficult to identify failure
	ERROR_MOTOR_THERMO		= 3,
	ERROR_PCB_THERMO		= 4,
	ERROR_EMG				= 5, //TODO: figure out how to handle EMG disconnect. not doing anything with this now.

	//limit errors
	WARNING_BATTERY_VOLTAGE	= 6,
	WARNING_TORQUE_MEASURED	= 7,
	ERROR_CURRENT_MEASURED	= 8, //not doing anything with this now
	ERROR_TORQUE_REQUESTED	= 9, //set and handled in setMotorTorque
	ERROR_CURRENT_REQUESTED	= 10, //set and handled in setMotorTorque
	WARNING_JOINTANGLE_SOFT	= 11,
	ERROR_JOINTANGLE_HARD	= 12,

	//general errors
	ERROR_WRONG_ACTUATOR	= 13,
	ERROR_PERSISTENT		= 14,
};

#define ERROR_ARRAY_SIZE	15

//sensor disconnect check values
#define LOADCELL_DISCONNECT_STRAIN_DIFFERENCE 5000
#define LOADCELL_DISCONNECT_COUNT_THRESHOLD	 1000

#define JOINT_ANGLE_DIFF_VALUE				 0
#define JOINT_ANGLE_COUNT_THRESHOLD			 100

#define MOTOR_ANGLE_DIFF_VALUE				 0
#define MOTOR_ANGLE_COUNT_THRESHOLD		 	 100
#define MOTOR_ENCODER_DISCONNECT_AXIAL_FORCE_THRESHOLD_N 10
#define MOTOR_ENCODER_DISCONNECT_BOUND 250000
#define MOTOR_CURRENT_DISCONNECT_THRESHOLD		10000

#define SAFE_MODE_MOTOR_POSITION_SETPOINT_RAD JOINT_ZERO

//shared LED codes used in main_fsm
extern uint8_t l0, l1, l2;	//FIXME: Values used in src/main_fsm.c

#endif //#if (ACTIVE_PROJECT == PROJECT_MIT_DLEG)


#endif /* FLEXSEA_PROJECTS_MIT_DLEG_INC_SAFETY_FUNCTIONS_H_ */
