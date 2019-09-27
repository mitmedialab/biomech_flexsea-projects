#ifndef __BACKESTIMATIONMETHODS_H__
#define __BACKESTIMATIONMETHODS_H__


#include <stdio.h>
#include <float.h>
#include "task_machine.h"
#include "kinematics_methods.h"

void back_estimate(struct taskmachine_s* tm, struct kinematics_s* kin, struct statistics_s* stats);
void update_heuristics(struct taskmachine_s* tm, struct kinematics_s* kin);
void init_heuristics();
struct heuristics_s*  get_heuristics();
void set_ur_slope_thresh_rad(float val);
void set_dr_slope_thresh_rad(float val);
void set_us_thresh_m(float yval, float zval);
void set_ur_thresh_m(float yval, float zval);
void set_ds_thresh_m(float yval, float zval);
void set_dr_thresh_m(float yval, float zval);

struct heuristics_s
{

	uint8_t ready_for_prediction;
	uint8_t made_prediction;
	uint8_t slope_thresh_status;
	uint8_t stepping_backwards;
	uint8_t passed_vertical_thresh;
	uint8_t good_trajectory;
	uint8_t prediction;
    float prev_stance_samples;
    float prev_swing_samples;
    float prev_torque_range;
    float torque_range;
    float max_torque;
    float min_torque;

    float ur_z_thresh_m;
    float ur_y_thresh_m;
    float us_z_thresh_m;
    float us_y_thresh_m;
    float dr_z_thresh_m;
    float dr_y_thresh_m;
   	float ds_z_thresh_m;
   	float ds_y_thresh_m;
   	float ur_slope_thresh_rad;
   	float dr_slope_thresh_rad;
   	float ground_slope_bin_size;
   	float ground_slope_offset_rad;
   	float min_stance_aOmegaX;



};


enum Slope_Statuses{
	PASSED_NO_SLOPE_THRESH = 0,
	PASSED_UR_SLOPE_THRESH = 1,
	PASSED_DR_SLOPE_THRESH = 2,
};

#endif
