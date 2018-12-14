#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>


//TODO: These constants need to be checked!!!!!

//Swing/stance thresholds
#define MIN_TQ_FOR_FOOT_ON 12.0000f
#define MIN_STANCE_SAMPLES 200

//Foot static thresholds
#define UPPER_ACCNORM_THRESH_SQ 102.01f
#define LOWER_ACCNORM_THRESH_SQ 90.25f
#define MIN_TQ_FOR_FOOT_STATIC_NM 15.0000f
#define DEFAULT_STANCE_RESET_SAMPLES 50
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

	int found_optimal_foot_static;
	int in_swing;
	int reached_classification_time;
	int stride_classified;
	int do_learning_for_stride;

	int translation_reset_trigger;
	int learning_reset_trigger;

	float tq;
	float theta;

	

};

struct back_estimator_s 
{
	float mean_stance_theta;
	float max_stance_tq;
	float max_stance_tq_ind;
	float prev_max_stance_tq;
	float min_swing_z;
	float max_swing_z;
	float max_swing_z_samples;
};

struct taskmachine_s* get_task_machine();
void task_machine_demux(struct rigid_s* rigid);


enum Task_Machine_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	INIT_KINEMATICS,
	RUN_TASK_MACHINE,
};

enum Walking_Tasks {
    TASK_FL = 0,
    TASK_UR = 1,
    TASK_DR = 2,
    TASK_US = 3,
    TASK_DS = 4,
};

enum Walking_Modes {
    MODE_FL = 0,
    MODE_UR = 1,
    MODE_DR = 2,
    MODE_US = 3,
    MODE_DS = 4,
    MODE_PR = 5,
    MODE_SW = 6,
};


#endif