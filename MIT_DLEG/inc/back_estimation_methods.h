#ifndef __BACKESTIMATIONMETHODS_H__
#define __BACKESTIMATIONMETHODS_H__


#include <stdio.h>
#include <float.h>
#include "task_machine.h"

void back_estimate(struct taskmachine_s* tm, struct statistics_s* stats);
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin);
void init_back_estimator();
struct back_estimator_s*  get_back_estimator();


//Copied from matlab pil simulation
struct back_estimator_s 
{
	float sum_sin_sq_attack_angle;
    float prev_mean_sin_sq_attack_angle;
    float min_stance_theta;
    float max_stance_theta;
    float prev_min_stance_theta;
    float prev_passed_ds_z_thresh_samples;
    int passed_ds_z_thresh_samples;
    uint8_t passed_us_z_thresh;
    uint8_t passed_ds_z_thresh;
    uint8_t completed_back_estimation;
};

#endif
