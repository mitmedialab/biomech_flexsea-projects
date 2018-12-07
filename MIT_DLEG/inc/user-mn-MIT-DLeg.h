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
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "flexsea_cmd_calibration.h"
//#include "flexsea_cmd_biomech.h"
#include "flexsea_user_structs.h"

#include <math.h>

//#include "user-mn-MIT-EMG.h"
#include "actuator_functions.h"
#include "walking_state_machine.h"
#include "state_variables.h"


#include <flexsea_comm.h>



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


//Main FSMs
//void openSpeedFSM(void);
//void twoPositionFSM(void);
//void oneTorqueFSM(struct act_s *actx);
//void twoTorqueFSM(struct act_s *actx);
//void torqueSweepTest(struct act_s *actx);

//User writes
void updateUserWrites(Act_s *actx, WalkParams *wParams);
void initializeUserWrites(Act_s *actx, WalkParams *wParams);


//****************************************************************************
// Definition(s):
//****************************************************************************

//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees

//1. Select joint type
#define IS_ANKLE
//#define IS_KNEE

//2. Select device
#define DEVICE_TF08_A01			// Define specific actuator configuration. Ankle 01
//#define DEVICE_TF08_A02		// Define specific actuator configuration. Ankle 02
//#define DEVICE_TF08_A03		// Define specific actuator configuration. Knee 01
//#define DEVICE_TF08_A04		// Define specific actuator configuration. Knee 02


//Begin device specific configurations

//Transmission
#ifdef IS_ANKLE					// Might UPDATE THIS WITH NEW SCREWs ankle = 0.002
#define N_SCREW			1256.63706144	// == (2*M_PI/0.005)	// Ballscrew ratio
#define N_ETA			0.9		// Transmission efficiency
#endif
#ifdef IS_KNEE
#define N_SCREW			1257 	// == (2*M_PI/0.005)	// Ballscrew ratio
#endif

#ifdef DEVICE_TF08_A01

//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_CPDEG			45.5111	// Counts per degree (JOINT_CPR/360)
#define JOINT_HS_MIN		1365	// == ( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		4096	// ==( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		11644	// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		6033	// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPDEG 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		59322.282	// == ( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		6212.717999 // == ( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		0.167502787	// == ( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
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
#define JOINT_CPDEG			45.5111	// Counts per degree (JOINT_CPR/360)
#define JOINT_HS_MIN		1365	// == ( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		4096	// ==( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		14550 //15020		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		8995 //9441		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPDEG 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		59322.282	// == ( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		6212.717999 // == ( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		0.167502787	// == ( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
#define TORQ_CALIB_M		1.0978	// y=Mx+b, from collected data set, applied load
#define TORQ_CALIB_B		0.0656	// y=Mx+b, from collected data set, applied load

//Torque Control PID gains
#define TORQ_KP_INIT			1.2 //10.
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			1. //5, 2. 1 is also a good number


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

#endif // DEFINED DEVICE_TF08_A02


#ifdef DEVICE_TF08_A03
// copy from above and update, when ready.
//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_CPDEG			45.5111	// Counts per degree (JOINT_CPR/360)
#define JOINT_HS_MIN		1365	// == ( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		4096	// ==( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		16475		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		10829		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPDEG 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		59322.282	// == ( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		6212.717999 // == ( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		0.167502787	// == ( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
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

#endif // DEFINED DEVICE_TF08_A03

#ifdef DEVICE_TF08_A04
// copy from above and update, when ready.
//Encoder
#define JOINT_ZERO_OFFSET 	20		// [deg] Joint Angle offset, CW rotation, based on Setup, ie. TEDtalk flexfoot angle = 20, fittings, etc.
#define JOINT_ENC_DIR 		-1		// Encoder orientation. CW = 1 (knee orientation), CCW = -1
#define JOINT_ANGLE_DIR 	1		// Joint angle direction. RHR convention is Ankle: Dorsiflexion (-), Plantarflexion (+) with value == 1
#define JOINT_CPR 			16384	// Counts per revolution (todo: is it (2^14 - 1)?)
#define JOINT_CPDEG			45.5111	// Counts per degree (JOINT_CPR/360)
#define JOINT_HS_MIN		1365	// == ( 30 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in dorsiflexion)
#define JOINT_HS_MAX		4096	// ==( 90 * JOINT_CPR/360 )		// Joint hard stop angle [deg] in plantarflexion)
#define JOINT_MIN_ABS		14569		// Absolute encoder at MIN (Max dorsiflexion, 30Deg)
#define JOINT_MAX_ABS		9007		// Absolute encoder reading at MAX (Max Plantarflexion, 90Deg)
#define JOINT_ZERO_ABS		JOINT_MIN_ABS + JOINT_ENC_DIR * JOINT_HS_MIN 	// Absolute reading of Actuator Zero as designed in CAD
#define JOINT_ZERO 			JOINT_ZERO_ABS + JOINT_ENC_DIR * JOINT_ZERO_OFFSET *JOINT_CPDEG 	// counts for actual angle.

//Force Sensor
#define FORCE_DIR			-1		// Direction of positive force, Dorsiflexion (-), Plantarflex (+) with value == -1
#define FORCE_STRAIN_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define FORCE_STRAIN_BIAS	2.5		// Strain measurement Bias
#define FORCE_EXCIT			5		// Excitation Voltage
#define FORCE_RATED_OUTPUT	0.002	// 2mV/V, Rated Output
#define FORCE_MAX			4448	// Newtons, for LCM300 load cell
#define FORCE_MAX_TICKS		59322.282	// == ( (FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT + FORCE_STRAIN_BIAS)/5 * 65535 )	// max ticks expected
#define FORCE_MIN_TICKS		6212.717999 // == ( (FORCE_STRAIN_BIAS - FORCE_STRAIN_GAIN * FORCE_EXCIT * FORCE_RATED_OUTPUT)/5 * 65535 )	// min ticks expected
#define FORCE_PER_TICK		0.167502787	// == ( ( 2 * FORCE_MAX  ) / (FORCE_MAX_TICKS - FORCE_MIN_TICKS)	)	// Newtons/Tick
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

#endif // DEFINED DEVICE_TF08_A04


//Joint software limits [Degrees]
#ifdef IS_ANKLE
#define JOINT_MIN_SOFT		-25	* RAD_PER_DEG	// [deg] Actuator physical limit min = -30deg dorsiflexion
#define JOINT_MAX_SOFT		45	* RAD_PER_DEG	// [deg] Actuator physical limit  max = 90deg plantarflex
#endif

#ifdef IS_KNEE
#define JOINT_MIN_SOFT		-20	* RAD_PER_DEG	// [deg] Actuator physical limit min = -30deg extension
#define JOINT_MAX_SOFT		20	* RAD_PER_DEG	// [deg] Actuator physical limit max = +90deg flexion
#endif

//Safety limits
#define PCB_TEMP_LIMIT_INIT		70		// DEG C
#define MOTOR_TEMP_LIMIT_INIT	70		// DEG C
#define ABS_TORQUE_LIMIT_INIT	150.0	    // Joint torque [Nm]
#define CURRENT_LIMIT_INIT		50000.0		// [mA] useful in this form, 40000 max

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
#define ANG_UNIT				6.28318531 	// == 2*M_PI 		// Use Radians 2*M_PI
#define RAD_PER_DEG				0.01745329	// == 2PI/360, 0.017453292519943
#define DEG_PER_RAD 			57.2957795 	// degree to rad conversion

// Constants for moment arm calculation
// see matlab script and matt's notes
// todo: how many digits are actually usable?  should this be single precision 6-9 sig digits? using 8 digits for now
#define MA_TF 			1.46228042	// original value 1.462280429971271
#define MA_A2B2			23654
#define MA_TWOAB		12686.6236	// 1.268662366431668e+04
#define MA_A2MINUSB2	19964
#define MA_B			42.95346318	// 42.953463189829060

// EMG channels
#define JIM_LG					emg_data[5]
#define JIM_TA					emg_data[3]




#endif	//INC_MIT_DLEG

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE || BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_DLEG || BOARD_TYPE_FLEXSEA_MANAGE
