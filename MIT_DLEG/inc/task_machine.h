#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>



#define UPPER_ACCNORM_THRESH_SQ 106.0900f
#define LOWER_ACCNORM_THRESH_SQ 86.49000f
#define MAX_SAMPLES_FROM_MAX_TQ 100
#define MIN_TQ_FOR_FOOT_ON 12.0000f
#define N_MIN_TQ_FOR_FOOT_ON -12.0000f
#define MIN_TQ_FOR_FOOT_STATIC 40.0000f
#define MIN_SWING_SAMPLES 200
#define MIN_STANCE_SAMPLES 200
#define DEFAULT_STANCE_RESET_SAMPLES 100
#define MIN_TQDOT_FOR_FOOT_STATIC -0.1f
#define STANCE_RESET_EXPIRY_SAMPLES 1000

// #define RAMAGYZ 0.0498214190885808f

#define SAMPLE_RATE_HZ 1000.0
#define SAMPLE_PERIOD 1.0/SAMPLE_RATE_HZ

#define PREDICTION_CUTOFF_SAMPLES 300
#define BEGINNING_CUTOFF_SAMPLES 44
#define ENDING_START_SAMPLES PREDICTION_CUTOFF_SAMPLES - BEGINNING_CUTOFF_SAMPLES





struct taskmachine_s
{
	
	int mode;
	int task;
	int state;

	int latestFootStaticSamples;
	int elapsedSamples;
	int latestFootOffSamples;

	int foundFirstZvupAfterFootOn;
	int taskPredicted;
	int inSwing;
	int achievedMinTorqueForStance;


	int resetTrigger;

	float tqDot;
	float tq;
	float maxTq;
	int maxTqInd;

};

struct taskmachine_s* get_task_machine();
void task_machine_demux(struct rigid_s* rigid);


enum Task_Machine_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	INIT_KINEMATICS,
	RUN_TASK_MACHINE
};

enum Walking_States {
    STATE_IDLE =  0,
    STATE_RELAXED_OBSOLETE = 1,          /// 1  DEPRECATED as of 1.4
    STATE_EARLY_SWING = 2,               /// 2
    STATE_LATE_SWING = 3,                /// 3
    STATE_EARLY_STANCE = 4,              /// 4
    STATE_LATE_STANCE = 5,               /// 5
    STATE_LATE_STANCE_POWER = 6, ///6
    STATE_LATE_SWING_UR = 10,                /// 10
    STATE_EARLY_STANCE_UR = 11,              /// 11
    STATE_LATE_STANCE_UR = 12,               /// 12
    STATE_LATE_SWING_DR = 13,                /// 10
    STATE_EARLY_STANCE_DR = 14,              /// 11
    STATE_LATE_STANCE_DR = 15,               /// 12
    STATE_LATE_SWING_US = 16,
    STATE_EARLY_STANCE_US = 17,
    STATE_LATE_STANCE_US = 18,
    STATE_LATE_SWING_DS = 19,
    STATE_EARLY_STANCE_DS = 20,
    STATE_LATE_STANCE_DS = 21,

    STATE_MOTOR_ERROR          = 8,  /// 8 DO NOT CHANGE: TUNING APP DEPENDENCY: States 8-19 Are 'Sad' (Error) states in Tuning App V1.2.1 and greater.
    STATE_WALKING_ENTRY_STATE = 9,       /// 9 DO NOT CHANGE: TUNING APP DEPENDENCY: States 8-19 Are 'Sad' (Error) states in Tuning App V1.2.1 and greater.

    STATE_BEEP_MOTOR_OBSOLETE  = 14, /// 14 DEPRECATED as of 1.4
    STATE_BENCHTEST            = 15, /// 15 DO NOT CHANGE: TUNING APP DEPENDENCY: States 8-19 Are 'Sad' (Error) states in Tuning App V1.2.1 and greater.
    STATE_DO_NOTHING           = 17, /// 17 DO NOT CHANGE: TUNING APP DEPENDENCY: States 8-19 Are 'Sad' (Error) states in Tuning App V1.2.1 and greater.

    STATE_IMPEDANCE_PREP       = 27,  /// 27   

};

enum Walking_Tasks {
    TASK_FL = 1,
    TASK_UR = 2,
    TASK_DR = 3,
    TASK_US = 4,
    TASK_DS = 5,
};

enum Walking_Modes {
    MODE_FL = 1,
    MODE_UR = 2,
    MODE_DR = 3,
    MODE_US = 4,
    MODE_DS = 5,
    MODE_PR = 6,
    MODE_SW = 7
};


#endif