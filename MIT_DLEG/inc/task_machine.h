#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "back_estimation_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>
#include <math.h>
#include "terrain_state_machine.h"

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

//Other important values
#define PI 3.14159
#define RAD_PER_DEG PI/180.0
#define RAD_PER_DEG_X_SAMPLE_RATE_HZ RAD_PER_DEG * SAMPLE_RATE_HZ
#define N_CLASSES 5

//Ideal controller values
#define FL_IDEAL_NET_WORK_J_PER_KG 0.0244 //Sinitski 2012
#define US_IDEAL_NET_WORK_J_PER_KG 0.386 //Sinitski 2012
#define DS_IDEAL_NET_WORK_J_PER_KG -0.559 //Sinitski 2012
#define UR_IDEAL_NET_WORK_J_PER_KG 0.390 //calculated from digitized McIntosh 2006 ankle power
#define DR_IDEAL_NET_WORK_J_PER_KG -0.391 //calculated from digitized McIntosh 2006 ankle power
#define FL_IDEAL_ROM_RAD 0.51 //Sinitski 2012
#define US_IDEAL_ROM_RAD 0.74 //Sinitski 2012
#define DS_IDEAL_ROM_RAD 1.06 //Sinitski 2012
#define UR_IDEAL_ROM_RAD 0.642 //calculated from digitized McIntosh 2006 ankle angle
#define DR_IDEAL_ROM_RAD 0.398 //calculated from digitized McIntosh 2006 ankle angle
#define FL_IDEAL_HEELSTRIKE_ANGLE_RAD 0.03 //Sinitski 2012
#define US_IDEAL_HEELSTRIKE_ANGLE_RAD 0.28 //Sinitski 2012
#define DS_IDEAL_HEELSTRIKE_ANGLE_RAD -0.51 //Sinitski 2012
#define UR_IDEAL_HEELSTRIKE_ANGLE_RAD 0.14 //calculated from digitized McIntosh 2006 ankle angle
#define DR_IDEAL_HEELSTRIKE_ANGLE_RAD 0.03 //calculated from digitized McIntosh 2006 ankle angle

//Copied from matlab pil simulation
struct taskmachine_s
{
	uint8_t terrain_mode;
	uint8_t do_update_learner;

	
	float latest_foot_static_samples;
    float elapsed_samples;
    float latest_foot_off_samples;
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

    float tq_prev;
    float aa_prev;
    float aa_dot_aOmegaX_error_prev ;

    float net_work_j_p_kg;
    float stance_rom_rad;
    float heelstrike_angle_rad;

    float* net_work_error_j_p_kg;
    float* stance_rom_error_rad;
    float* heelstrike_angle_error_rad;

    float torque_raw; //testing purposes only
    float angle_raw; //testing purposes only

    //subject specific params
    float weight_kg;

};


struct taskmachine_s* get_task_machine();
void task_machine_demux(struct rigid_s* rigid);


enum Task_Machine_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	INIT_KINEMATICS,
	RUN_TASK_MACHINE,
};

enum Terrain_Modes {
	MODE_FLAT,
	MODE_URAMP,
	MODE_DRAMP,
	MODE_USTAIRS,
	MODE_DSTAIRS,
	MODE_NOMINAL,
	MODE_PREDICT,
};

enum Terrains {
    K_FLAT = 0,
	K_URAMP = 1,
	K_DRAMP = 2,
	K_USTAIRS = 3,
	K_DSTAIRS = 4,
	K_NOMINAL = 5,
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
