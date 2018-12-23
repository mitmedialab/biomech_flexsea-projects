#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "back_estimation_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>
#include <math.h>

//Copied from matlab pil simulation
 //Gait event thresholds
#define MIN_TQ_FOR_FOOT_ON 1.0
#define MIN_LOW_TQ_SAMPLES_FOR_SWING_TRANSITION 50
#define MIN_TQ_FOR_FOOT_STATIC 5.0
#define AA_DOT_AOMEGA_ERROR_THRESH 0.6
#define PREDICTION_CUTOFF_SAMPLES 250.0

 //Timing constants
#define SAMPLE_RATE_HZ 1000.0
#define SAMPLE_PERIOD 1.0/SAMPLE_RATE_HZ

 //Default filter coefficients
#define FILTA 0.95
#define FILTB 0.05
#define FILTC 0.999
#define FILTD 0.001

 //Gait events
#define GAIT_EVENT_FOOT_ON 1
#define GAIT_EVENT_FOOT_STATIC 2
#define GAIT_EVENT_FOOT_OFF 3
#define GAIT_EVENT_WINDOW_CLOSE 4

 

//Copied from matlab pil simulation
struct taskmachine_s
{
	
	int latest_foot_static_samples;
    int elapsed_samples;
    int latest_foot_off_samples;
    uint8_t in_swing;
    uint8_t stride_classified;
    uint8_t do_learning_for_stride;

    uint8_t gait_event_trigger;  
    uint8_t reset_back_estimator_trigger;

    int low_torque_counter;

    float tq;
    float tq_dot;
    float aa;
    float aa_dot;
    float aa_dot_aOmegaX_error;

	int demux_state;

    float tq_prev;
    float aa_prev;
    float aa_dot_aOmegaX_error_prev ;

    float torque_raw; //testing purposes only
    float angle_raw; //testing purposes only

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