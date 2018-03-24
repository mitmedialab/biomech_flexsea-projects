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
#define ACTIVE_LEG 0

//Walking and Running modes
#define RUNNING_TORQUE_TRACKING
#define WALKING_TORQUE_TRACKING

//Foot switch
#define FOOTSWITCH_TOE 0                	// 0th analog channel
#define FOOTSWITCH_LATERAL 1            	// 1st analog channel
#define FOOTSWITCH_HEEL 2               	// 2nd analog channel
#define FOOTSWITCH_MEDIAL 3             	// 3rd analog channel

#if ACTIVE_LEG == LEFT_ANKLE

#define FOOTSWITCH_THRESHOLD 500			//threshold for heel strike
#define TOE_OFF_THRESHOLD 30		    	//threshold for toe off detection
#define dTOE_OFF_THRESHOLD -30		    	//differential threshold for toe off detection
#define HEEL_STRIKE_THRESHOLD 1500			//threshold for heel strike detection
#define dHEEL_STRIKE_THRESHOLD 30	    	//differential threshold for heel strike detection
#define GAIT_PERIOD_THRESHOLD_FLOOR 200  	//msec, floor time for a normal gait cycle period
#define GAIT_PERIOD_THRESHOLD_CEILING 2000  //msec, ceiling time for  a normal gait cycle period
#define SWING_PERIOD_THRESHOLD_FLOOR 30  	//msec, floor time for a normal swing period of a gait cycle

#elif ACTIVE_LEG == RIGHT_ANKLE

#define FOOTSWITCH_THRESHOLD 500			//tick, threshold for heel strike
#define TOE_OFF_THRESHOLD 30		    	//tick, threshold for toe off detection
#define dTOE_OFF_THRESHOLD -30		    	//tick, differential threshold for toe off detection
#define HEEL_STRIKE_THRESHOLD 1500			//tick,threshold for heel strike detection
#define dHEEL_STRIKE_THRESHOLD 30	    	//tick, differential threshold for heel strike detection
#define GAIT_PERIOD_THRESHOLD_FLOOR 200  	//msec, floor time for a normal gait cycle period
#define GAIT_PERIOD_THRESHOLD_CEILING 2000  //msec, ceiling time for  a normal gait cycle period
#define SWING_PERIOD_THRESHOLD_FLOOR 30  	//msec, floor time for a normal swing period of a gait cycle

#endif //ACTIVE_LEG == LEFT_ANKLE

//Lookup Table
#if (CONTROL_STRATEGY == TORQUE_TRACKING)
#define TABLE_SIZE 1001
#define DEFAULT_TORQUE_PROFILE_GAIN 0.1    //percentage of biological torque applied to the subject
#endif //(CONTROL_STRATEGY == TORQUE_TRACKING)

//State Definition
#define DEACTIVATED 0
#define STANCE_PHASE 1
#define SWING_PHASE 2

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
#define LOAD_MAX_IN_KILOGRAM			LOAD_MAX_IN_POUND*KILOGRAM_PER_POUNUD		// 226.7962 kg, maximum mass the force sensor can measure
#define GRAVITY_ACC						9.8
#define FORCE_MAX						LOAD_MAX_IN_KILOGRAM*GRAVITY_ACC		// 2222.602613 Newton,  for LCM200 load cell maximum force the force sensor can measure
#define FORCE_STRAIN_SENSITIVITY		2.1938 	// mV/V, force sensor sensitivity
#define FORCE_EXCIT						5		// V, Excitation Voltage
#define FULL_RANGE_OUTPUT				FORCE_STRAIN_SENSITIVITY*FORCE_EXCIT	// 10.969 mV, strain gauge output at full range point 500 pound
#define FORCE_STRAIN_AMPLIFIER_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define MAX_LOAD_VOLTAGE_TO_ADC			ZERO_LOAD_VOLTAGE_TO_ADC+FULL_RANGE_OUTPUT*FORCE_STRAIN_AMPLIFIER_GAIN //2.5V+2222.3194 mV = 2.5+2.2223194 V=4.7223194 V, stain gauge full output voltage augmented by the amplifer
#define ADC_RESOLUTION_BIT				16		//ADC resolution in bits.
#define ADC_RESOLUTION					(uint16)pow(2,ADC_RESOLUTION_BIT) // 65535, ADC resolution in decimal.
#define ADC_MAX_TICKS					(uint16)pow(2,ADC_RESOLUTION_BIT) // 65535, ADC maximum output, corresponding to 5 V voltage input to ADC under this circumstance
#define ADC_MAX_TICKS_VOLTAGE			5	//V, the ADC can collect voltage up to 5 V, corresponding to 65535 ticks output output from ADC under this circumstance
#define ZERO_LOAD_TICKS					33048  // ticks when no load applied on the load cell, less than 33054 when compressed, greater than 33054 when tensioned
#define ZERO_LOAD_VOLTAGE_TO_ADC		2.5		// V, stain gauge voltage converted to ADC after amplifier augmentation
#define MAX_LOAD_TICKS					(uint16)((ADC_MAX_TICKS-ZERO_LOAD_TICKS)*(MAX_LOAD_VOLTAGE_TO_ADC-ZERO_LOAD_VOLTAGE_TO_ADC)/(ADC_MAX_TICKS_VOLTAGE-ZERO_LOAD_VOLTAGE_TO_ADC))
										// 28879 ticks, corresponding to 500 lb load
#define FORCE_PER_TICK					FORCE_MAX/MAX_LOAD_TICKS // newton/tick, 0.0769785825165379

#define FORCE_CALIB_M					0.074306 // Force = M*tick + B, from collected data set, applied load
#define FORCE_CALIB_B					-2455.6817859 	// Force = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

#elif ACTIVE_LEG == RIGHT_ANKLE

#define LOAD_MAX_IN_POUND				500		// lb, maximum mass the force sensor can measure
#define KILOGRAM_PER_POUNUD				0.4535924
#define LOAD_MAX_IN_KILOGRAM			LOAD_MAX_IN_POUND*KILOGRAM_PER_POUNUD		// 226.7962 kg, maximum mass the force sensor can measure
#define GRAVITY_ACC						9.8
#define FORCE_MAX						LOAD_MAX_IN_KILOGRAM*GRAVITY_ACC		// 2222.602613 Newton,  for LCM200 load cell maximum force the force sensor can measure
#define FORCE_STRAIN_SENSITIVITY		2.1938 	// mV/V, force sensor sensitivity
#define FORCE_EXCIT						5		// V, Excitation Voltage
#define FULL_RANGE_OUTPUT				FORCE_STRAIN_SENSITIVITY*FORCE_EXCIT	// 10.969 mV, strain gauge output at full range point 500 pound
#define FORCE_STRAIN_AMPLIFIER_GAIN 	202.6	// Defined by R23 on Execute, better would be G=250 to max range of ADC
#define MAX_LOAD_VOLTAGE_TO_ADC			ZERO_LOAD_VOLTAGE_TO_ADC+FULL_RANGE_OUTPUT*FORCE_STRAIN_AMPLIFIER_GAIN //2.5V+2222.3194 mV = 2.5+2.2223194 V=4.7223194 V, stain gauge full output voltage augmented by the amplifer
#define ADC_RESOLUTION_BIT				16		//ADC resolution in bits.
#define ADC_RESOLUTION					(uint16)pow(2,ADC_RESOLUTION_BIT) // 65535, ADC resolution in decimal.
#define ADC_MAX_TICKS					(uint16)pow(2,ADC_RESOLUTION_BIT) // 65535, ADC maximum output, corresponding to 5 V voltage input to ADC under this circumstance
#define ADC_MAX_TICKS_VOLTAGE			5	//V, the ADC can collect voltage up to 5 V, corresponding to 65535 ticks output output from ADC under this circumstance
#define ZERO_LOAD_TICKS					33048  // ticks when no load applied on the load cell, less than 33054 when compressed, greater than 33054 when tensioned
#define ZERO_LOAD_VOLTAGE_TO_ADC		2.5		// V, stain gauge voltage converted to ADC after amplifier augmentation
#define MAX_LOAD_TICKS					(uint16)((ADC_MAX_TICKS-ZERO_LOAD_TICKS)*(MAX_LOAD_VOLTAGE_TO_ADC-ZERO_LOAD_VOLTAGE_TO_ADC)/(ADC_MAX_TICKS_VOLTAGE-ZERO_LOAD_VOLTAGE_TO_ADC))
										// 28879 ticks, corresponding to 500 lb load
#define FORCE_PER_TICK					FORCE_MAX/MAX_LOAD_TICKS // newton/tick, 0.0769785825165379

#define FORCE_CALIB_M					0.074306 // Force = M*tick + B, from collected data set, applied load
#define FORCE_CALIB_B					-2455.6817859 	// Force = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752

#endif //ACTIVE_LEG == LEFT_ANKLE



//****************************************************************************
// Running exoskeleton parameter(s)
//****************************************************************************
#define MOMENT_ARM_ON_FOOT 				0.25 //m, cable tension force's applied arm
#define ANKLE_TORQUE_PER_TICK			FORCE_PER_TICK*MOMENT_ARM_ON_FOOT // N.m/tick, 0.01924465
#define ANKLE_TORQUE_CALIB_M			0.0185766 // Torque = M*tick + B, from collected data set, applied load
#define ANKLE_TORQUE_CALIB_B			-613.9204465 	// Torque = M*tick + B, , from collected data set, applied load
										// r^2 = 0.999954358260752



//****************************************************************************
// Torque command trajectory lookup table(s)
//****************************************************************************




#endif	//INC_RUNNINGEXO_MN_H

#endif //defined BOARD_TYPE_FLEXSEA_MANAGE || defined BOARD_TYPE_FLEXSEA_PLAN
#endif //defined INCLUDE_UPROJ_RUNNINGEXO || defined BOARD_TYPE_FLEXSEA_PLAN
