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

#include <math.h>


#include "walking_state_machine.h"
#include "state_variables.h"




// Safety features
int8_t safetyShutoff(void); //renamed from safetyFailure(void)
void   clampCurrent(float* pcurrentDes);

// Initialization
int8_t findPoles(void);
void   mit_init_current_controller(void);

// Mechanical transformations
void   	getJointAngleKinematic(struct act_s *act_x);
float   getJointAngularVelocity(void);
float   getAxialForce(void);
float   getLinkageMomentArm(float);
float   getJointTorque(struct act_s *actx);
float 	getJointTorqueRate(struct act_s *actx);
float 	calcRestoringCurrent(struct act_s *actx, float N);
int16_t getMotorTempSensor(void);
void    updateSensorValues(struct act_s *actx);
float 	signalFilterSlope(float value, float a, float limit);
void 	updateJointTorqueRate(struct act_s *actx);

//Control outputs
float biomCalcImpedance(float k1, float b, float theta_set); 	// returns a desired joint torque, then use setMotorTorque() to get the motor to do its magic
void  setMotorTorque(struct act_s *actx, float tor_d);
void  packRigidVars(struct act_s *actx);

void  setMotorTorqueOpenLoop(struct act_s *actx, float tau_des);
void setMotorTorqueOpenLoopVolts(struct act_s *actx, float tau_des);
float frequencySweep(float omega, float t);


//Smoothing
float windowSmoothJoint(int32_t val);
float windowSmoothAxial(float val);
float windowAveraging(float currentVal);

#define WINDOW_SIZE 5
