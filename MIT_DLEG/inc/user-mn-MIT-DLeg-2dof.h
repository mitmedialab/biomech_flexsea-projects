/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developers] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Matthew Carney, mcarney at mit dot edu, Tony Shu, tonyshu at mit dot edu
*****************************************************************************
	[This file] MIT DARPA Leg Main FSM
****************************************************************************/

#if defined INCLUDE_UPROJ_MIT_DLEG || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_MIT_DLEG
#define INC_MIT_DLEG

//****************************************************************************
// Include(s)
//****************************************************************************
#include <stdint.h>
#include "main.h"


//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern struct act_s act1;	//define actuator structure shared
extern int8_t isEnabledUpdateSensors;

//****************************************************************************
// Structure(s)
//****************************************************************************

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_MIT_DLeg(void);
void MIT_DLeg_fsm_1(void);
void MIT_DLeg_fsm_2(void);

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

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
float biomCalcImpedance(float k1, float k2, float b, float theta_set); 	// returns a desired joint torque, then use setMotorTorque() to get the motor to do its magic
void  setMotorTorque(struct act_s *actx, float tor_d);
void  packRigidVars(struct act_s *actx);

//Smoothing
float windowSmoothJoint(int32_t val);
float windowSmoothAxial(float val);

//Main FSMs
void openSpeedFSM(void);
void twoPositionFSM(void);
void oneTorqueFSM(struct act_s *actx);
void twoTorqueFSM(struct act_s *actx);
void torqueSweepTest(struct act_s *actx);
void sineDemo(float phaseDelay, float frequency, float amplitude, float thetaOffset);


//****************************************************************************
// Definition(s):
//****************************************************************************

//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees

//1. Select joint type
#define IS_ANKLE
//#define IS_KNEE

//2. Select device
//#define DEVICE_TF08_A01			// Define specific actuator configuration. Ankle 01
//#define DEVICE_TF08_A02		// Define specific actuator configuration. Ankle 02
#define DEVICE_TF08_A03		// Define specific actuator configuration. Knee 01
//#define DEVICE_TF08_K02		// Define specific actuator configuration. Knee 02


//Begin device specific configurations

//Transmission
#ifdef IS_ANKLE					// Might UPDATE THIS WITH NEW SCREWs ankle = 0.002
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif
#ifdef IS_KNEE
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#endif

#ifdef DEVICE_TF08_A01

//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_HS_MIN		( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		9636		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		4020		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
#define TORQ_CALIB_M		1.0978	// y=Mx+b, from collected data set, applied load
#define TORQ_CALIB_B		0.0656	// y=Mx+b, from collected data set, applied load

//Torque Control PID gains
#define TORQ_KP_INIT			1.2 // good for step response, for zero torque 3 is good
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			2. // good for step response, for zero torque 15 is good

// Motor Parameters
#define MOT_KT 			0.055	// Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH
#define MOT_J			0		//0.000322951	// rotor inertia, [kgm^2]
#define MOT_B			0.0		// damping term for motor and screw combined, drag from rolling elements
#define MOT_TRANS		0		// lumped mass inertia todo: consider MotorMass on Spring inertia contribution.
#define MOT_STIC_POS	1400	// stiction current, 1800
#define MOT_STIC_NEG	1600	// stiction current, 1800

// Current Control Parameters  -- Test these on a motor test stand first
#define ACTRL_I_KP_INIT		15
#define ACTRL_I_KI_INIT		15
#define ACTRL_I_KD_INIT		0

#endif //DEFINED DEVICE_TF08_A01


#ifdef DEVICE_TF08_A02
// copy from above and update, when ready.
//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_HS_MIN		( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		14570		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		8999		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
#define TORQ_CALIB_M		1.0978	// y=Mx+b, from collected data set, applied load
#define TORQ_CALIB_B		0.0656	// y=Mx+b, from collected data set, applied load

//Torque Control PID gains
#define TORQ_KP_INIT			1.2 //10.
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			2. //5, 2. 1 is also a good number


// Motor Parameters
#define MOT_KT 			0.055	// Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH
#define MOT_J			0//0.000120 //0.000322951		//0.000322951	// rotor inertia, [kgm^2]
#define MOT_B			0//0.000200 //0.000131		// damping term for motor and screw combined, drag from rolling elements
#define MOT_TRANS		0		// lumped mass inertia todo: consider MotorMass on Spring inertia contribution.
#define MOT_STIC_POS	1400	// stiction current, 1800
#define MOT_STIC_NEG	1600	// stiction current, 1800

// Current Control Parameters  -- Test these on a motor test stand first
#define ACTRL_I_KP_INIT		15
#define ACTRL_I_KI_INIT		15
#define ACTRL_I_KD_INIT		0

//Transmission
#ifdef IS_ANKLE					//UPDATE THIS WITH NEW SCREWs ankle = 0.002
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif
#ifdef IS_KNEE
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#endif


#endif // DEFINED DEVICE_TF08_A02


#ifdef DEVICE_TF08_A03
// copy from above and update, when ready.
//Encoder
#define JOINT_ZERO_OFFSET 	0		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_HS_MIN		( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		16475		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		10829		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
#define TORQ_CALIB_M		1.0978	// y=Mx+b, from collected data set, applied load
#define TORQ_CALIB_B		0.0656	// y=Mx+b, from collected data set, applied load

//Torque Control PID gains
#define TORQ_KP_INIT			1.2 //10.
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			2. //5, 2. 1 is also a good number


// Motor Parameters
#define MOT_KT 			0.055	// Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH
#define MOT_J			0//0.000120 //0.000322951		//0.000322951	// rotor inertia, [kgm^2]
#define MOT_B			0//0.000200 //0.000131		// damping term for motor and screw combined, drag from rolling elements
#define MOT_TRANS		0		// lumped mass inertia todo: consider MotorMass on Spring inertia contribution.
#define MOT_STIC_POS	1400	// stiction current, 1800
#define MOT_STIC_NEG	1600	// stiction current, 1800

// Current Control Parameters  -- Test these on a motor test stand first
#define ACTRL_I_KP_INIT		15
#define ACTRL_I_KI_INIT		15
#define ACTRL_I_KD_INIT		0

//Transmission
#ifdef IS_ANKLE					//UPDATE THIS WITH NEW SCREWs ankle = 0.002
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif
#ifdef IS_KNEE
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif

#endif // DEFINED DEVICE_TF08_A03


#ifdef DEVICE_TF08_A04
// copy from above and update, when ready.
//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_HS_MIN		( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		14569		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		9007		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPR/360 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
#define TORQ_CALIB_M		1.0978	// y=Mx+b, from collected data set, applied load
#define TORQ_CALIB_B		0.0656	// y=Mx+b, from collected data set, applied load

//Torque Control PID gains
#define TORQ_KP_INIT			1.2 //10.
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			2. //5, 2. 1 is also a good number


// Motor Parameters
#define MOT_KT 			0.055	// Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH
#define MOT_J			0//0.000120 //0.000322951		//0.000322951	// rotor inertia, [kgm^2]
#define MOT_B			0//0.000200 //0.000131		// damping term for motor and screw combined, drag from rolling elements
#define MOT_TRANS		0		// lumped mass inertia todo: consider MotorMass on Spring inertia contribution.
#define MOT_STIC_POS	1400	// stiction current, 1800
#define MOT_STIC_NEG	1600	// stiction current, 1800

// Current Control Parameters  -- Test these on a motor test stand first
#define ACTRL_I_KP_INIT		15
#define ACTRL_I_KI_INIT		15
#define ACTRL_I_KD_INIT		0

//Transmission
#ifdef IS_ANKLE					//UPDATE THIS WITH NEW SCREWs ankle = 0.002
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif
#ifdef IS_KNEE
#define N_SCREW			(2*M_PI/0.005)	// Ballscrew ratio
#endif


#endif // DEFINED DEVICE_TF08_A04


//Joint software limits [Degrees]
#ifdef IS_ANKLE
#define JOINT_MIN_SOFT		-25	* (ANG_UNIT)/360	// [deg] Actuator physical limit min = -30deg dorsiflexion
#define JOINT_MAX_SOFT		55	* (ANG_UNIT)/360	// [deg] Actuator physical limit  max = 90deg plantarflex
#endif

#ifdef IS_KNEE
#define JOINT_MIN_SOFT		-20	* (ANG_UNIT)/360	// [deg] Actuator physical limit min = -30deg extension
#define JOINT_MAX_SOFT		20	* (ANG_UNIT)/360	// [deg] Actuator physical limit max = +90deg flexion
#endif

//Safety limits
#define PCB_TEMP_LIMIT_INIT		70
#define MOTOR_TEMP_LIMIT_INIT	70
#define ABS_TORQUE_LIMIT_INIT	50	    // Joint torque [Nm]
#define CURRENT_LIMIT_INIT		20000		// [mA] useful in this form, 40000 max

// Motor Temp Sensor
#define V25_TICKS		943		//760mV/3.3V * 4096 = 943
#define VSENSE_SLOPE	400		//1/2.5mV
#define TICK_TO_V		1241	//ticks/V

enum {
	SAFETY_OK			=	0,
	SAFETY_ANGLE		=	1,
	SAFETY_TORQUE		=	2,
	SAFETY_FLEX_TEMP	=	3,  //unused
	SAFETY_TEMP			=	4,
};

// System constants
#define SECONDS					1000		// Scale seconds to ms
#define CURRENT_SCALAR_INIT		1000		// Scale Amps to mAmps
#define ANG_UNIT				2*M_PI 		// Use Radians 2*M_PI
#define DEG_PER_RAD 			57.2957795 // degree to rad conversion

// EMG channels
#define JIM_LG					emg_data[5]
#define JIM_TA					emg_data[3]


#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG || BOARD_TYPE_FLEXSEA_MANAGE
