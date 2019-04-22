#ifndef __TASKMACHINE_H__
#define __TASKMACHINE_H__

#include "flexsea_user_structs.h"
#include "kinematics_methods.h"
#include "machine_learning_methods.h"
#include "back_estimation_methods.h"
#include "user-mn-MIT-DLeg.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include "terrain_state_machine.h"
#include "state_variables.h"

 //Gait event thresholds
#define MIN_TQ_FOR_FOOT_ON 5.0
#define MIN_LOW_TQ_SAMPLES_FOR_SWING_TRANSITION 50
#define PREDICTION_CUTOFF_SAMPLES 250.0

 //Timing constants
#define SAMPLE_RATE_HZ 1000.0
#define SAMPLE_PERIOD 1.0/SAMPLE_RATE_HZ

 //Default filter coefficients
#define FILTA 0.95
#define FILTB 0.05
#define FILTC 0.999
#define FILTD 0.001



//Other important values
#define PI 3.14159
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
#define FL_IDEAL_FOOTSTRIKE_ANGLE_RAD 0.03 //Sinitski 2012
#define US_IDEAL_FOOTSTRIKE_ANGLE_RAD 0.28 //Sinitski 2012
#define DS_IDEAL_FOOTSTRIKE_ANGLE_RAD -0.51 //Sinitski 2012
#define UR_IDEAL_FOOTSTRIKE_ANGLE_RAD 0.14 //calculated from digitized McIntosh 2006 ankle angle
#define DR_IDEAL_FOOTSTRIKE_ANGLE_RAD 0.03 //calculated from digitized McIntosh 2006 ankle angle

//Copied from matlab pil simulation
struct taskmachine_s
{
    uint8_t initialized;
    
	int control_mode;
	uint8_t do_update_learner;

	
    float elapsed_samples;
    float latest_foot_off_samples;
    uint8_t in_swing;
    uint8_t stride_classified;
    uint8_t do_learning_for_stride;

    uint8_t gait_event_trigger;  
    uint8_t reset_back_estimator_trigger;

    int low_torque_counter;

    float tq;
    float max_tq;
    float tq_dot;
    float aa;
    float aa_dot;


    float tq_prev;
    float aa_prev;


    float net_work_j;
    float max_power_w;
    float min_power_w;
    float stance_rom_rad;
    float heelstrike_angle_rad;
    float peak_power_timing_percent;

    float* net_work_error_j_p_kg;
    float* stance_rom_error_rad;
    float* heelstrike_angle_error_rad;
    float* peak_power_timing_error_percent;

    bool net_work_within_bounds;
    bool stance_rom_within_bounds;
    bool heelstrike_angle_within_bounds;
    bool peak_power_timing_within_bounds;


    int est_pred_correct; //used as combined variable holding k_est (hundreds), k_pred (tens), and correctness (ones). ex. 441 or 430

};


struct taskmachine_s* get_task_machine();
void task_machine_demux(struct rigid_s* rigid, Act_s* act);


enum Task_Machine_States {
	INIT_TASK_MACHINE,
	INIT_LEARNING,
	INIT_KINEMATICS,
	INIT_TERRAIN_STATE_MACHINE,
	RUN_TASK_MACHINE,
};

enum Control_Modes {
	MODE_FLAT = 0,
	MODE_URAMP = 1,
	MODE_DRAMP = 2,
	MODE_USTAIRS = 3,
	MODE_DSTAIRS = 4,
    MODE_NOMINAL = 5,
    MODE_POSITION = 6,
	MODE_ADAPTIVE = 7,
};

enum Gait_Events {
    GAIT_EVENT_DEFAULT = 0,
    GAIT_EVENT_FOOT_ON = 1,
    GAIT_EVENT_FOOT_STATIC = 2,
    GAIT_EVENT_FOOT_OFF = 3,
    GAIT_EVENT_WINDOW_CLOSE = 4,
};

 enum Terrain_Classes {
     K_FLAT = 0,
 	K_URAMP = 1,
 	K_DRAMP = 2,
 	K_USTAIRS = 3,
 	K_DSTAIRS = 4,
 };

#endif
