/****************************************************************************
	PRIVATE FUNCTIONS
*****************************************************************************
	[Lead developers] Matt Carney, mcarney at mit dot edu
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Actuator Specific functions

	These are lower level functions that should not be messed with.

****************************************************************************/


//****************************************************************************
// Include(s)
//****************************************************************************
#include <stdint.h>
#include "main.h"
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "mn-MotorControl.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
#include "flexsea_user_structs.h"
#include "user-mn-MIT-DLeg.h"

#include <math.h>
#include <stdbool.h>

#include "walking_state_machine.h"
#include "state_variables.h"
//DEBUG
#include "software_filter.h"


// Initialization
int8_t 	findPoles(void);
void   	mitInitCurrentController(void);
void	mitInitOpenController(void);


// Sensor values
void  updateSensorValues(struct act_s *actx);

//Control outputs
float biomCalcImpedance(Act_s *actx, float k1, float b, float theta_set); 	// returns a desired joint torque, then use setMotorTorque() to get the motor to do its magic
void  setMotorTorque(struct act_s *actx, float tor_d);
void  setMotorTorqueOpenLoop(struct act_s *actx, float tau_des);
void  setMotorTorqueOpenLoopVolts(struct act_s *actx, float tau_des);
float torqueSystemIDFrequencySweep(float omega, float t, float amplitude);
float torqueSystemIDPRBS(void);
bool integralAntiWindup(float tau_err, float tau_C_total, float tau_C_output); // integral term anti-windup clamp check
float actuateAngleLimits(Act_s *actx);	// apply virtual spring/damper on angle limits
float noLoadCurrent(float desCurr);

#define WINDOW_SIZE 10
