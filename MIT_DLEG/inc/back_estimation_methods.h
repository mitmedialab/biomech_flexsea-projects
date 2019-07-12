#ifndef __BACKESTIMATIONMETHODS_H__
#define __BACKESTIMATIONMETHODS_H__


#include <stdio.h>
#include <float.h>
#include "task_machine.h"

void back_estimate(struct taskmachine_s* tm, struct statistics_s* stats, struct kinematics_s* kin);
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin);
void init_back_estimator();
struct back_estimator_s*  get_back_estimator();
void set_us_z_thresh_m(float val);
void set_ds_z_thresh_m(float val);
void set_us_z_max_samples(int val);
void set_ds_z_max_samples(int val);
void set_ur_slope_thresh_rad(float val);
void set_dr_slope_thresh_rad(float val);

//Copied from matlab pil simulation
struct back_estimator_s 
{

    uint8_t curr_stride_paz_thresh_status;
    uint8_t prev_stride_paz_thresh_status;

    float us_z_thresh_m;
   	float ds_z_thresh_m;
   	float ds_y_thresh_m;
   	int us_z_max_samples;
   	int ds_z_max_samples;
   	float ur_slope_thresh_rad;
   	float dr_slope_thresh_rad;

   	float prev_torque_range;
   	float max_torque;
   	float min_torque;
   	float prev_stance_samples;



};

enum Paz_Statuses{
   PAZ_PASSED_NO_THRESH = 0,
   PAZ_PASSED_US_THRESH = 1,
   PAZ_PASSED_DS_THRESH = 2,
};

#endif
