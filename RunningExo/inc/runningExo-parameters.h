/*
 * runningExo-parameters.h
 *
 *  Created on: May 25, 2018
 *      Author: Albert Wu
 */

#ifndef RUNNING_EXO_PARAMETERS
#define RUNNING_EXO_PARAMETERS
#include "user-mn.h"
#include "user-mn-ActPack.h"
#include "flexsea_sys_def.h"
#include "flexsea_user_structs.h"
#include "flexsea_global_structs.h"
#include <flexsea_system.h>
#include "flexsea_cmd_calibration.h"
#include <flexsea_comm.h>
#include "flexsea_board.h"



//Turn off safety check for testing purposes
//#define DISABLE_SAFETY
//****************************************************************************
// Constant Definition(s):
//****************************************************************************
/* Controlling Strategy Options
 * 0: No action
 * 1: Torque Tracking
 * 2: User torque command from GUI
 */
#define GAIT_TORQUE_TRACKING 1
#define USER_TORQUE_COMMAND 2
#define TRAJECTORY_TORQUE_TRACKING 3

//Active control strategy
#define CONTROL_STRATEGY USER_TORQUE_COMMAND
//#define PD_TUNING
#define MOTOR_FEEDFOWARD_TUNING

#if CONTROL_STRATEGY == TRAJECTORY_TORQUE_TRACKING
	#define TRACK_PERIOD 3
#endif

//Angle Limit
//#define ENC_POS_MAX 1000000		//basically no limit, not used.
//#define ENC_POS_MIN -28000			//not used.
#define FEEDBACK_POS_MIN 10000		//TODO: this seems to change on every start up
#define FEEDBACK_MIN_VOLTAGE -8.
//Human parameters
#define LEFT_ANKLE 0
#define Right_ANKLE 1
#define DEFAULT_BODY_WEIGHT 68 // kg, subject's body weight
#define	MAX_UNIT_RUNNING_TORQUE	2 // N.m/kg, we use 2 here, can be changed according to the applied torque profile
#define ACTIVE_LEG 0
#define DISABLED_STEPS 5

//Walking and Running modes
#define RUNNING_TORQUE_TRACKING
#define WALKING_TORQUE_TRACKING

//Foot switch
//#define FOOTSWITCH_TOE 0                	// 0th analog channel
//#define FOOTSWITCH_LATERAL 1            	// 1st analog channel
//#define FOOTSWITCH_HEEL 2               	// 2nd analog channel
//#define FOOTSWITCH_MEDIAL 3             	// 3rd analog channel

//Right leg
#define FOOTSWITCH_TOE 0                	// 0th analog channel
#define FOOTSWITCH_LATERAL 3            	// 1st analog channel
#define FOOTSWITCH_HEEL 1               	// 2nd analog channel
#define FOOTSWITCH_MEDIAL 2             	// 3rd analog channel

#if ACTIVE_LEG == LEFT_ANKLE

//#define FOOTSWITCH_THRESHOLD 500			//threshold for heel strike
#define FOOT_FLAT_THRESHOLD 500
#define TOE_OFF_THRESHOLD 30		    	//threshold for toe off detection
#define dTOE_OFF_THRESHOLD -30		    	//differential threshold for toe off detection
#define HEEL_STRIKE_THRESHOLD 500			//threshold for heel strike detection
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
//#if (CONTROL_STRATEGY == GAIT_TORQUE_TRACKING)
#define TABLE_SIZE 1001
#define DEFAULT_TORQUE_PROFILE_GAIN 0.2    	//percentage of biological torque applied to the subject
#define VARIABLE_BIO_TORQUE_GAIN
//State Definition
#define DEACTIVATED 0
#define HEEL_STRIKE 1						//between heel strike and foot flat
#define FOOT_FLAT 2
#define SWING_PHASE 3

//#endif //(CONTROL_STRATEGY == TORQUE_TRACKING)


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
#define MIN_FEEDBACK_ANKLE_TORQUE		4.5//Nm minimum command to activate feedback system
#define DERIVATIVE_WEIGHTING_FACTOR		0.01
//****************************************************************************
// Running exoskeleton parameter(s)
//****************************************************************************
#define MOMENT_ARM_ON_FOOT 				0.25 //m, cable tension force's applied arm
#define ANKLE_TORQUE_PER_TICK			2.*(FORCE_PER_TICK*MOMENT_ARM_ON_FOOT) // N.m/tick, 0.01924465
#define ANKLE_TORQUE_CALIB_M			0.0185766 // Torque = M*tick + B, from collected data set, applied load
#define ANKLE_TORQUE_CALIB_B			-613.9204465 	// Torque = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752
#define SLACK_ANKLE_TORQUE				4. //prevent string slack
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
//#define MOT_KT 			((1/((2*M_PI*MOT_KV)/60))/sqrt(3))	// 0.06126 N.m/A, torque constant, Phase Kt value = linearKt/(3^0.5)
#define MOT_L			0.068	// mH, not yet be measured, refer to Matt's data
#define MOT_J			0 // 0.000322951 kgm^2, rotor inertia, not yet be measured, refer to Matt's data
#define MOT_B			0 //0.000131, damping term for motor, not yet be measured, refer to Matt's data
#define MOT_TRANS		0 // lumped mass inertia todo: consider MotorMass on Spring inertia contribution. refer to Matt's data
#define N_ETA			0.75		// Transmission efficiency
#define MOT_R			0.0948		//Measured line-to-line resistance
//#define MOT_KT			0.1350		//Measured phase Kt
#define MOT_KT			0.1		//Measured phase Kt
#define TORQUE_EPSILON 0.01
//Hand Tuned Feedforward Parameters
//Values for motor 1
// V = tau_desired*K1+omega*K2

//Values for motor 1
#define K1 0.78//Motor Only 0.493
#define K2 0.1125//Motor only: 0.1135
#define DEADBAND 0.489


//Values for motor 3

#define OMEGA_THRESHOLD 0.2		//Prevent noise
//#define DEADBAND 0.523
#define EXO_ANKLE_DEADBAND 12.
#define FRICTION_COMPENSATION 1

//Values for motor 2
//#define K1 0.051
//#define K2 0.03
//#define OMEGA_THRESHOLD 0.2		//Prevent noise
//#define DEADBAND 0.49

// Limitation and safety parameters
#define MAX_BOARD_TEMP		70						//centidegree, avoid the FlexSEA board overheating
//#define MAX_ANKLE_TORQUE	(DEFAULT_TORQUE_PROFILE_GAIN*DEFAULT_BODY_WEIGHT*MAX_UNIT_RUNNING_TORQUE) // N.m, applied torque on the human ankle
#define	MAX_CABLE_TENSION_FORCE	(MAX_ANKLE_TORQUE/MOMENT_ARM_ON_FOOT) //Newton
#define MOT_OUTPUT_SHAFT_DIAMETER	0.0065			//m, motor output shaft
#define MOT_OUTPUT_SHAFT_RADIUS MOT_OUTPUT_SHAFT_DIAMETER/2
#define ROPE_DIAMETER		0.001	//m, diameter of ropes
#define MOTOR_TORQUE_MARGIN_FACTOR	1.2
#define MAX_MOTOR_TORQUE_REQUIRED	(MOTOR_TORQUE_MARGIN_FACTOR*MAX_CABLE_TENSION_FORCE*(MOT_OUTPUT_SHAFT_DIAMETER/2))
#define MAX_MOTOR_CURRENT	(1000*MAX_MOTOR_TORQUE_REQUIRED/MOT_KT)		//mA, 33.95 A--2.08 N.m
#define	MOTOR_CURRENT_LIMIT		28000 //mA
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
//#define	MAX_MOTOR_SPEED		(MOTOR_SPEED_MARGIN_FACTOR*(MAX_FOOT_PULL_HEIGHT/MOT_OUTPUT_SHAFT_PERIMETER/(GAIT_CYCLE_PERIOD*STANCE_PERCENTAGE))*2*M_PI) // rad/sec
#define	MAX_MOTOR_SPEED 100	//debug


//Torque Control PID gains
//For motor 1
#define TORQUE_KP			300e-3 //10.
#define TORQUE_KD			-100e-6 //don't know why it's negative but it works well

//Averager Filtering
#define AVERAGE_FILTER_SAMPLES 80

//Digital LPF
#define LPF_SAMPLE_COUNT 62
static const float LPF_COEFFICIENTS[LPF_SAMPLE_COUNT] = {0.00526686499700382,0.0333076591118252,
							  0.107859852808001,0.227534642759739,0.333326852544359,0.329499070916401,
							  0.172699406644114,-0.0517901034056601,-0.176913727961088,
							  -0.112713620698725,0.0494147803722348,0.127266858406567,
							  0.0490093786773033,-0.072383748526244,-0.0868594850269479,
							  0.00775203978426433,0.0779184127860659,0.0378254421386039,
							  -0.0453799557082701,-0.0567458282956024,0.00887033828750875,
							  0.0531191789265961,0.0193707435453973,-0.0357115063583919,
							  -0.0340690494637236,0.0139490687776598,0.0357622392982401,
							  0.00526921542994905,-0.0276627188872557,-0.0172094978439014,
							  0.0155900051997027,0.0221193976849347,-0.00227626115728514,
							  -0.0187825326334373,-0.00558146647581938,0.0149258003318076,
							  0.0140472946656259,-0.00297288051186792,-0.00925309141243743,
							  0.00474873519148799,0.0215576639985494,0.0225225414349096,
							  0.00921890841391762,-0.00369968175420303,-0.00734184373464438,
							  -0.00441495051624024,-0.00115984206007953};
// extern variables
extern struct actuation_parameters act_para;

//****************************************************************************
// Torque command trajectory lookup table(s)
//****************************************************************************

#define CURRENT_SCALAR_INIT		1000	// Scale Amps to mAmps
#define TIMESTEPS_PER_SECOND			1000.
#define TIMESTEP_SIZE 1/TIMESTEPS_PER_SECOND //timestep size for doing 1st order differentiation
#define FORCE_OFFSET_CLOSED	0 //N, cable preload of close loop
#define FORCE_OFFSET_OPEN	30 //N, cable preload of open loop. Todo, currently, roughly use this, should program to initialize the cable preload according to the force sensor reader

#endif /* RUNNING_EXO_PARAMETERS */
