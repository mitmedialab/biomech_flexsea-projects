#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>


//Swing/stance thresholds
#define MAX_SAMPLES_FROM_MAX_TQ 100
#define MIN_TQ_FOR_FOOT_ON 12.0000f
#define N_MIN_TQ_FOR_FOOT_ON -12.0000f
#define MIN_SWING_SAMPLES 200
#define MIN_STANCE_SAMPLES 200
#define MIN_STRIDE_SAMPLES 500

//Foot static thresholds
#define UPPER_ACCNORM_THRESH_SQ 106.0900f
#define LOWER_ACCNORM_THRESH_SQ 86.49000f
#define MIN_TQ_FOR_FOOT_STATIC 40.0000f
#define DEFAULT_STANCE_RESET_SAMPLES 100
#define MIN_TQDOT_FOR_FOOT_STATIC -0.1f
#define STANCE_RESET_EXPIRY_SAMPLES 1000

//Prediction thresholds
#define PREDICTION_CUTOFF_SAMPLES 200

//Timing constants
#define SAMPLE_RATE_HZ 1000.0
#define SAMPLE_PERIOD 1.0/SAMPLE_RATE_HZ

//Back estimation constants
#define US_Z_THRESH 0.303f
#define US_MAX_SAMPLES_TO_Z_THRESH 1400

#define UR_Z_THRESH 0.05f
#define UR_MEAN_THETA_THRESH -8.0f

#define DS_Z_THRESH -0.13f
#define DS_TQ_THRESH 50.0

#define DR_Z_THRESH -0.05f
#define DR_MEAN_THETA_THRESH 1.6f




struct taskmachine_s
{
	
	int mode;
	int task;

	int latest_foot_static_samples;
	int elapsed_samples;
	int latest_foot_off_samples;

	int foundFirstZvupAfterFootOn;
	int taskPredicted;
	int inSwing;
	int achievedMinTorqueForStance;


	int resetTrigger;

	float tqDot;
	float tq;
	float theta;

	//Back estimation variables
	float mean_stance_theta;
	float max_stance_tq;
	float max_stance_tq_ind;
	float prev_max_stance_tq;
	float min_swing_z;
	float max_swing_z;
	float max_swing_z_samples;
	float prev_stance_samples;
	int k_est;
	int do_learning;

};

struct taskmachine_s* get_task_machine();
void task_machine_demux(struct rigid_s* rigid);


enum Task_Machine_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	INIT_KINEMATICS,
	RUN_TASK_MACHINE
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