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
#include "filtering_methods.h"
#include "state_variables.h"



 //Timing constants
#define SAMPLE_RATE_HZ 1000.0
#define SAMPLE_PERIOD_S 1.0/SAMPLE_RATE_HZ

 //Default filter coefficients
#define FILTA 0.95
#define FILTB 0.05
#define FILTC 0.99999
#define FILTD 0.00001

//Other important values
#define PI 3.14159
#define RAD_PER_DEG_X_SAMPLE_RATE_HZ RAD_PER_DEG * SAMPLE_RATE_HZ
#define N_CLASSES 5

//Copied from matlab pil simulation
struct taskmachine_s
{
    uint8_t initialized;
    
	int control_mode;
	int current_terrain;
	uint8_t do_update_learner;

	
    float elapsed_samples;
    float latest_foot_off_samples;
    uint8_t in_swing;
    uint8_t do_learning_for_curr_stride;
    uint8_t do_learning_for_prev_stride;
    uint8_t passed_min_stance_tq;

    uint8_t gait_event_trigger;  
    uint8_t reset_back_estimator_trigger;

    int low_torque_counter;

    float tq;
    float tq_dot;
    float aa;
    float aa_dot;
    float aa_dot_15hz_filt;
    float tq_prev;
    float aa_prev;
    float mean_swing_tq_nm;


    float net_work_j;
    float power_w;
    float peak_power_w;
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

enum Gait_Modes {
	MODE_FLAT = 0,
	MODE_URAMP = 1,
	MODE_DRAMP = 2,
	MODE_USTAIRS = 3,
	MODE_DSTAIRS = 4,
    MODE_NOMINAL = 5,
    MODE_POSITION = 6,
	MODE_ADAPTIVE_WITH_LEARNING = 7,
	MODE_ADAPTIVE_WITH_HEURISTICS = 8,
};

enum Gait_Events {
    GAIT_EVENT_DEFAULT = 0,
    GAIT_EVENT_FOOT_ON = 1,
    GAIT_EVENT_FOOT_STATIC = 2,
    GAIT_EVENT_FOOT_OFF = 3,
    GAIT_EVENT_WINDOW_CLOSE = 4,
};

 enum Terrain_Classes {
	 K_DEFAULT = -1,
     K_FLAT = 0,
 	K_URAMP = 1,
 	K_DRAMP = 2,
 	K_USTAIRS = 3,
 	K_DSTAIRS = 4,
 };

#endif
