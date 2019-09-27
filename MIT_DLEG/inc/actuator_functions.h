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

#include "state_variables.h"
#include "safety_functions.h"
//DEBUG
#include "software_filter.h"


// Initialization
int8_t 	findPoles(void);
void   	mitInitCurrentController(void);
void	mitInitOpenController(void);


// Sensor values
void  updateSensorValues(struct act_s *actx);
void setMotorNeutralPosition(struct act_s *actx);

//Control outputs
float biomCalcImpedance(Act_s *actx, float k1, float b, float theta_set); 	// returns a desired joint torque, then use setMotorTorque() to get the motor to do its magic
float getImpedanceTorque(Act_s *actx, float k1, float b, float thetaSet);	// Returns a Torque Value
float getImpedanceTorqueQuadratic(Act_s *actx, float k1, float b, float thetaSet, float k2);	// Returns a Torque Value
void  setMotorTorque(struct act_s *actx, float tor_d);
void  setMotorTorqueOpenLoop(struct act_s *actx, float tau_des, int8_t motorControl);
float getCompensatorPIDOutput(float refTorque, float sensedTorque);

float getFeedForwardTerm(float refTorque);
float getReferenceLPF(float refTorque);
float getNotchFilter(float refTorque);
float getCompensatorCustomOutput(float refTorque, float sensedTorque);									// calculate compensator output value
//float getCompensatorCustomOutput2(Act_s *actx, float tauMeas, float tauRef);									// calculate compensator output value
float getDOB(float refTorque, float measTorque);
float getDobLpf(float refTorque);
float getDoBInv(float refTorque);


void  setMotorTorqueOpenLoopVolts(struct act_s *actx, float tau_des);
float torqueSystemIDFrequencySweep(float omega, uint32_t signalTimer, float amplitude, float dcBias, float noiseAmp, int16_t begin);
//float torqueSystemIDFrequencySweepChirp(float initOmega,  float finalOmega, float T, float amplitude, float dcBias, float noiseAmp, int8_t chirpType, int8_t begin);
float torqueSystemIDFrequencySweepChirp(float initOmega,  float finalOmega, float testLength, float amplitude, float dcBias, float noiseAmp, int16_t chirpType, int16_t running);
float torqueSystemIDPRBS(void);
bool integralAntiWindup(float tau_err, float tau_C_total, float tau_C_output); // integral term anti-windup clamp check
float actuateAngleLimits(Act_s *actx);	// apply virtual spring/damper on angle limits
int32_t noLoadCurrent(float desCurr);



#define WINDOW_SIZE 10
