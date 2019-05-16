#ifndef __BACKESTIMATIONMETHODS_H__
#define __BACKESTIMATIONMETHODS_H__


#include <stdio.h>
#include <float.h>
#include "task_machine.h"

void back_estimate(struct statistics_s* stats, struct kinematics_s* kin);
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin);
void init_back_estimator();
struct back_estimator_s*  get_back_estimator();


//Copied from matlab pil simulation
struct back_estimator_s 
{
    float min_stance_theta;
    float max_stance_theta;

    uint8_t curr_stride_paz_thresh_status;
    int curr_stride_paz_thresh_pass_samples;
    uint8_t prev_stride_paz_thresh_status;
    int prev_stride_paz_thresh_pass_samples;   
};

enum Paz_Statuses{
   PAZ_PASSED_NO_THRESH = 0,
   PAZ_PASSED_US_THRESH = 1,
   PAZ_PASSED_DS_THRESH = 2,
};

#endif
