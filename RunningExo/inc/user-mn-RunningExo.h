/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-mn-RunningExo: Running Exoskeleton Project
****************************************************************************/

#if defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
#if defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef INC_RUNNINGEXO_MN_H
#define INC_RUNNINGEXO_MN_H

//****************************************************************************
// Include(s)
//****************************************************************************
#include "main.h"
#include "math.h"

//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void init_runningExo(void);
void RunningExo_fsm_1(void);
void RunningExo_fsm_2(void);
void stateTransition(void);

//****************************************************************************
// Accessor(s)
//****************************************************************************


//****************************************************************************
// Constant Definition(s):
//****************************************************************************
#define CONTROL_STRATEGY 1
/* Controlling Strategy Options
 * 0: No action
 * 1: Torque Tracking
 *
 */
#define TORQUE_TRACKING 1

//Angle Limit
//TODO:
#define ENC_POS_MAX 16384
#define ENC_POS_MIN 0

//Velocity Limit
//TODO:
#define ENC_VEL_MAX 1000
#define ENC_VEL_MIN -1000

//Human parameters
#define LEFT_ANKLE 0
#define Right_ANKLE 1
#define DEFAULT_BODY_WEIGHT 75 // kg, subject's body weight
#define	MAX_UNIT_RUNNING_TORQUE	2 // N.m/kg, we use 2 here, can be changed according to the applied torque profile
#define ACTIVE_LEG 0
#define DISABLED_STEPS 5

//Walking and Running modes
#define RUNNING_TORQUE_TRACKING
#define WALKING_TORQUE_TRACKING

//Foot switch
#define FOOTSWITCH_TOE 0                	// 0th analog channel
#define FOOTSWITCH_LATERAL 1            	// 1st analog channel
#define FOOTSWITCH_HEEL 2               	// 2nd analog channel
#define FOOTSWITCH_MEDIAL 3             	// 3rd analog channel

#if ACTIVE_LEG == LEFT_ANKLE

//#define FOOTSWITCH_THRESHOLD 500			//threshold for heel strike
#define FOOT_FLAT_THRESHOLD 500
#define TOE_OFF_THRESHOLD 30		    	//threshold for toe off detection
#define dTOE_OFF_THRESHOLD -30		    	//differential threshold for toe off detection
#define HEEL_STRIKE_THRESHOLD 1500			//threshold for heel strike detection
#define dHEEL_STRIKE_THRESHOLD 30	    	//differential threshold for heel strike detection

//Gait sanity checks
#define MIN_GAIT_PERIOD 200  	//msec, floor time for a normal gait cycle period
#define MAX_GAIT_PERIOD 2000  //msec, ceiling time for  a normal gait cycle period
#define MAX_STANCE_PERIOD 2000  	//msec, floor time for a normal swing period of a gait cycle

//#elif ACTIVE_LEG == RIGHT_ANKLE
//
//#define FOOTSWITCH_THRESHOLD 500			//tick, threshold for heel strike
//#define TOE_OFF_THRESHOLD 30		    	//tick, threshold for toe off detection
//#define dTOE_OFF_THRESHOLD -30		    	//tick, differential threshold for toe off detection
//#define HEEL_STRIKE_THRESHOLD 1500			//tick,threshold for heel strike detection
//#define dHEEL_STRIKE_THRESHOLD 30	    	//tick, differential threshold for heel strike detection
//#define MIN_GAIT_PERIOD 200  	//msec, floor time for a normal gait cycle period
//#define MAX_GAIT_PERIOD 2000  //msec, ceiling time for  a normal gait cycle period
//#define MIN_STANCE_PERIOD 30  	//msec, floor time for a normal swing period of a gait cycle

#endif //ACTIVE_LEG == LEFT_ANKLE

//Lookup Table
#if (CONTROL_STRATEGY == TORQUE_TRACKING)
	#define TABLE_SIZE 1001
	#define DEFAULT_TORQUE_PROFILE_GAIN 0.1    	//percentage of biological torque applied to the subject
	//State Definition
	#define DEACTIVATED 0
	#define HEEL_STRIKE 1						//between heel strike and foot flat
	#define FOOT_FLAT 2
	#define SWING_PHASE 3

#endif //(CONTROL_STRATEGY == TORQUE_TRACKING)


//RunningExo_fsm_1 state variable
#define STATE_IDLE	-2
#define	STATE_INIT	-1
#define STATE_ENABLE_SENSOR_UPDATE	0
#define STATE_TORQUE_TRACKING	1
//****************************************************************************
// Structure(s)
//****************************************************************************


//****************************************************************************
// Shared variable(s)
//****************************************************************************



//****************************************************************************
// Force sensor parameter(s)
//****************************************************************************
#if ACTIVE_LEG == LEFT_ANKLE

#define LOAD_MAX_IN_POUND				500		// lb, maximum mass the force sensor can measure
#define KILOGRAM_PER_POUNUD				0.4535924
#define LOAD_MAX_IN_KILOGRAM			(LOAD_MAX_IN_POUND*KILOGRAM_PER_POUNUD)		// 226.7962 kg, maximum mass the force sensor can measure
#define GRAVITY_ACC						9.8
#define FORCE_MAX						(LOAD_MAX_IN_KILOGRAM*GRAVITY_ACC)		// 2222.602613 Newton,  for LCM200 load cell maximum force the force sensor can measure
#define FORCE_STRAIN_SENSITIVITY		2.1938 	// mV/V, force sensor sensitivity
#define FORCE_EXCIT						5		// V, Excitation Voltage
#define FULL_RANGE_OUTPUT				(FORCE_STRAIN_SENSITIVITY*FORCE_EXCIT)	// 10.969 mV, strain gauge output at full range point 500 pound
#define FORCE_STRAIN_AMPLIFIER_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define MAX_LOAD_VOLTAGE_TO_ADC			(ZERO_LOAD_VOLTAGE_TO_ADC+FULL_RANGE_OUTPUT*FORCE_STRAIN_AMPLIFIER_GAIN) //2.5V+2222.3194 mV = 2.5+2.2223194 V=4.7223194 V, stain gauge full output voltage augmented by the amplifer
#define ADC_RESOLUTION_BIT				16		//ADC resolution in bits.
#define ADC_RESOLUTION					((uint16)pow(2,ADC_RESOLUTION_BIT)) // 65535, ADC resolution in decimal.
#define ADC_MAX_TICKS					((uint16)pow(2,ADC_RESOLUTION_BIT)) // 65535, ADC maximum output, corresponding to 5 V voltage input to ADC under this circumstance
#define ADC_MAX_TICKS_VOLTAGE			5	//V, the ADC can collect voltage up to 5 V, corresponding to 65535 ticks output output from ADC under this circumstance
#define ZERO_LOAD_TICKS					33048  // ticks when no load applied on the load cell, less than 33054 when compressed, greater than 33054 when tensioned
#define ZERO_LOAD_VOLTAGE_TO_ADC		2.5		// V, stain gauge voltage converted to ADC after amplifier augmentation
#define MAX_LOAD_TICKS					((uint16)((ADC_MAX_TICKS-ZERO_LOAD_TICKS)*(MAX_LOAD_VOLTAGE_TO_ADC-ZERO_LOAD_VOLTAGE_TO_ADC)/(ADC_MAX_TICKS_VOLTAGE-ZERO_LOAD_VOLTAGE_TO_ADC)))
										// 28879 ticks, corresponding to 500 lb load
#define FORCE_PER_TICK					(FORCE_MAX/MAX_LOAD_TICKS) // newton/tick, 0.0769785825165379

#define FORCE_CALIB_M					0.074306 // Force = M*tick + B, from collected data set, applied load
#define FORCE_CALIB_B					-2455.6817859 	// Force = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

//****************************************************************************
// Running exoskeleton parameter(s)
//****************************************************************************
#define MOMENT_ARM_ON_FOOT 				0.25 //m, cable tension force's applied arm
#define ANKLE_TORQUE_PER_TICK			(FORCE_PER_TICK*MOMENT_ARM_ON_FOOT) // N.m/tick, 0.01924465
#define ANKLE_TORQUE_CALIB_M			0.0185766 // Torque = M*tick + B, from collected data set, applied load
#define ANKLE_TORQUE_CALIB_B			-613.9204465 	// Torque = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

#elif ACTIVE_LEG == RIGHT_ANKLE

#define LOAD_MAX_IN_POUND				500		// lb, maximum mass the force sensor can measure
#define KILOGRAM_PER_POUNUD				0.4535924
#define LOAD_MAX_IN_KILOGRAM			(LOAD_MAX_IN_POUND*KILOGRAM_PER_POUNUD)		// 226.7962 kg, maximum mass the force sensor can measure
#define GRAVITY_ACC						9.8
#define FORCE_MAX						(LOAD_MAX_IN_KILOGRAM*GRAVITY_ACC)		// 2222.602613 Newton,  for LCM200 load cell maximum force the force sensor can measure
#define FORCE_STRAIN_SENSITIVITY		2.1938 	// mV/V, force sensor sensitivity
#define FORCE_EXCIT						5		// V, Excitation Voltage
#define FULL_RANGE_OUTPUT				(FORCE_STRAIN_SENSITIVITY*FORCE_EXCIT)	// 10.969 mV, strain gauge output at full range point 500 pound
#define FORCE_STRAIN_AMPLIFIER_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define MAX_LOAD_VOLTAGE_TO_ADC			(ZERO_LOAD_VOLTAGE_TO_ADC+FULL_RANGE_OUTPUT*FORCE_STRAIN_AMPLIFIER_GAIN) //2.5V+2222.3194 mV = 2.5+2.2223194 V=4.7223194 V, stain gauge full output voltage augmented by the amplifer
#define ADC_RESOLUTION_BIT				16		//ADC resolution in bits.
#define ADC_RESOLUTION					((uint16)pow(2,ADC_RESOLUTION_BIT)) // 65535, ADC resolution in decimal.
#define ADC_MAX_TICKS					((uint16)pow(2,ADC_RESOLUTION_BIT)) // 65535, ADC maximum output, corresponding to 5 V voltage input to ADC under this circumstance
#define ADC_MAX_TICKS_VOLTAGE			5	//V, the ADC can collect voltage up to 5 V, corresponding to 65535 ticks output output from ADC under this circumstance
#define ZERO_LOAD_TICKS					33048  // ticks when no load applied on the load cell, less than 33054 when compressed, greater than 33054 when tensioned
#define ZERO_LOAD_VOLTAGE_TO_ADC		2.5		// V, stain gauge voltage converted to ADC after amplifier augmentation
#define MAX_LOAD_TICKS					((uint16)((ADC_MAX_TICKS-ZERO_LOAD_TICKS)*(MAX_LOAD_VOLTAGE_TO_ADC-ZERO_LOAD_VOLTAGE_TO_ADC)/(ADC_MAX_TICKS_VOLTAGE-ZERO_LOAD_VOLTAGE_TO_ADC)))
										// 28879 ticks, corresponding to 500 lb load
#define FORCE_PER_TICK					(FORCE_MAX/MAX_LOAD_TICKS) // newton/tick, 0.0769785825165379

#define FORCE_CALIB_M					0.074306 // Force = M*tick + B, from collected data set, applied load
#define FORCE_CALIB_B					-2455.6817859 	// Force = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

//****************************************************************************
// Running exoskeleton parameter(s)
//****************************************************************************
#define MOMENT_ARM_ON_FOOT 				0.25 //m, cable tension force's applied arm
#define ANKLE_TORQUE_PER_TICK			(FORCE_PER_TICK*MOMENT_ARM_ON_FOOT) // N.m/tick, 0.01924465
#define ANKLE_TORQUE_CALIB_M			0.0185766 // Torque = M*tick + B, from collected data set, applied load
#define ANKLE_TORQUE_CALIB_B			-613.9204465 	// Torque = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

#endif //ACTIVE_LEG == LEFT_ANKLE


// Motor Parameters
#define MOT_KV			90		//(round/min)/V, motor rpm constant
#define MOT_KT 			((1/((2*M_PI*MOT_KV)/60))/sqrt(3))	// 0.06126 N.m/A, torque constant, Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH, not yet be measured, refer to Matt's data
#define MOT_J			0 // 0.000322951 kgm^2, rotor inertia, not yet be measured, refer to Matt's data
#define MOT_B			0 //0.000131, damping term for motor, not yet be measured, refer to Matt's data
#define MOT_TRANS		0 // lumped mass inertia todo: consider MotorMass on Spring inertia contribution. refer to Matt's data
#define N_ETA			0.75		// Transmission efficiency

// Limitation and safety parameters
#define MAX_BOARD_TEMP		70						//centidegree, avoid the FlexSEA board overheating
#define MAX_ANKLE_TORQUE	(DEFAULT_TORQUE_PROFILE_GAIN*DEFAULT_BODY_WEIGHT*MAX_UNIT_RUNNING_TORQUE) // N.m, applied torque on the human ankle
#define	MAX_CABLE_TENSION_FORCE	(MAX_ANKLE_TORQUE/MOMENT_ARM_ON_FOOT) //Newton
#define MOT_OUTPUT_SHAFT_DIAMETER	0.0065			//m, motor output shaft
#define ROPE_DIAMETER		0.001	//m, diameter of ropes
#define MOTOR_TORQUE_MARGIN_FACTOR	1.2
#define MAX_MOTOR_TORQUE_REQUIRED	(MOTOR_TORQUE_MARGIN_FACTOR*MAX_CABLE_TENSION_FORCE*(MOT_OUTPUT_SHAFT_DIAMETER/2))
#define MAX_MOTOR_CURRENT	(1000*MAX_MOTOR_TORQUE_REQUIRED/MOT_KT)		//mA, 33.95 A--2.08 N.m
#define	MOTOR_CURRENT_LIMIT		18000 //mA
#define B_ANGLE_LIMIT			MOTOR_CURRENT_LIMIT/1000.
#define ENCODER_RESOLUTION_BIT		14		//AS5047P encoder resolution in bits.
#define ENCODER_RESOLUTION			((uint16_t)pow(2,ENCODER_RESOLUTION_BIT)) // 16384, AS5047P encoder resolution in decimal.
#define	ENCODER_CPR		ENCODER_RESOLUTION //16384, Counts per revolution
#define MAX_FOOT_PULL_HEIGHT	0.3	//m, cable's maximum retraction length
#define	MOT_OUTPUT_SHAFT_PERIMETER	(M_PI*MOT_OUTPUT_SHAFT_DIAMETER)	//0.02042 m, Motor output shaft perimeter
#define MAX_MOTOR_ENC_REVOLUTION	(MAX_FOOT_PULL_HEIGHT/MOT_OUTPUT_SHAFT_PERIMETER) // 14.69 (rounds) *16384 ticks, read the encoder every time before operating the motor, and then compare the current position and the initial position to shut off the motor if arriving at the position limitation
#define MIN_MOTOR_REVOLUTION	-(MAX_FOOT_PULL_HEIGHT/MOT_OUTPUT_SHAFT_PERIMETER) // -14.69 (rounds) *16384 ticks, read the encoder every time before operating the motor, and then compare the current position and the initial position to shut off the motor if arriving at the position limitation
#define	GAIT_CYCLE_PERIOD	0.41 //s, 0.41 s  when running at speed of 3.9 m/s
#define STANCE_PERCENTAGE	0.4 // percentage of stance phase occupying the whole gait cycle period
#define MOTOR_SPEED_MARGIN_FACTOR	0.6
#define	MAX_MOTOR_SPEED		(MOTOR_SPEED_MARGIN_FACTOR*(MAX_FOOT_PULL_HEIGHT/MOT_OUTPUT_SHAFT_PERIMETER/(GAIT_CYCLE_PERIOD*STANCE_PERCENTAGE))*2*M_PI) // rad/sec

enum {
	SAFETY_OK				=	0,
	SAFETY_TEMPERATURE		=	1,
	SAFETY_MOTOR_CURRENT	=	2,
	SAFETY_MOTOR_VELOCITY	=	3,
	SAFETY_MOTOR_POSITION	=	4,
	SAFETY_CABLE_TENSION		=	5,
	};


//running state structure
struct runningExoSystemState
{
	//System state
	int8_t state;
	uint32_t timer;
	uint32_t pedometer;
	//Time stamps
	int32_t heelStrikeTime;
	int32_t footFlatTime;
	int32_t toeOffTime;
	uint32_t prevStanceDuration;
	uint32_t prevGaitDuration;
	_Bool running;
	_Bool enableOutput;
	uint32_t disabledPedometer;					//number of disabled steps AFTER all issues are cleared
	#if (CONTROL_STRATEGY == TORQUE_TRACKING)
	//add controller specific stuff here
	#endif //CONTROL_STRATEGY == TORQUE_TRACKING
};



// parameters to track sensor values, actuate the motors
struct actuation_parameters
	{
	//exoskeleton parameter
	float ankleTorqueMeasured;		//N.m
	float ankleTorqueDesired;		//N.m
	float ankleHeight;				//m
	float ankleVel;					//m/s
	float ankleAcc;					//m/s/s
	float cableTensionForce;		//N
	//motor parameters
	float motorTorqueMeasured;		//N.m
	float motorTorqueDesired;		//N.m
	int32_t motorCurrentMeasured;	//mA
	int32_t motorCurrentDesired;	//mA
	int32_t initialMotorEncPosition;		//rad
	int32_t currentMotorEncPosition;	//motor position [rad]
	float motorRelativeEncRevolution; //motor revolutions relative to the the initial position after motion
	float motorRotationAngle;		 //rad, motor rotation angle relative to the initial position
	float motorAngularVel;		//motor angular velocity [rad/s]
	float motorAngularAcc;		// motor angular acceleration [rad/s/s]

	//control related parameters
	//float tauMeasuredAnkle;          // N.m, feedback ankle torque
	//float tauDesiredAnkle;           // N.m, desired ankle torque
	//float tauErrorMotor;				//N.m, reflected to motor, tauMeasured - tauDesired

	//safety parameters
	int16_t boardTemperature;	//centidegree, get from temperature sensor on the FlexSEA board
	int8_t safetyFlag;		//identify various safety problems
	int32_t currentOpLimit; // mA, current throttling limit
	};


//impedance control structure
typedef struct{

	float m;
	float b;
	float k;
	float thetaDes;

} GainParams;


//Torque Control PID gains
#define TORQ_KP_INIT			6 //10.
#define TORQ_KI_INIT			0.
#define TORQ_KD_INIT			5 //2.
#define MAX_TORQ_I_ERROR				0.05	//prevent wind-up
// Current Control Parameters  -- Test these on a motor test stand first
#define ACTRL_I_KP_INIT		48
#define ACTRL_I_KI_INIT		32
#define ACTRL_I_KD_INIT		0




// extern variables
extern struct actuation_parameters act_para;




//****************************************************************************
// Torque command trajectory lookup table(s)
//****************************************************************************


#define CURRENT_SCALAR_INIT		1000	// Scale Amps to mAmps
#define TIMESTEPS_PER_SECOND			1000
#define FORCE_OFFSET_CLOSED	0 //N, cable preload of close loop
#define FORCE_OFFSET_OPEN	50 //N, cable preload of open loop. Todo, currently, roughly use this, should program to initialize the cable preload according to the force sensor reader


#endif	//INC_RUNNINGEXO_MN_H

#endif //defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
