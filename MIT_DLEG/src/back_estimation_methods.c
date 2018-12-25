
#include "back_estimation_methods.h"

 //Back estimation constants (copied from matlab pil)
#define US_Z_THRESH 0.1
#define DS_Z_THRESH -0.25
#define DS_Z_SAMPLE_THRESH 500
#define UR_ANKLE_ANGLE_THRESH -20.0
#define DR_ANKLE_ANGLE_THRESH 8.0
#define DS_ANKLE_ANGLE_THRESH -10.0
#define US_SIN_SQ_ATTACK_ANGLE_THRESH 0.2


static struct back_estimator_s be;


//copied from matlab pil
void back_estimate(struct taskmachine_s* tm, struct statistics_s* stats){
  
	 tm->reset_back_estimator_trigger = 1;


	if (be.max_stance_theta > DR_ANKLE_ANGLE_THRESH){
	    stats->k_est = TASK_DR;
	    return;
	}

	if (be.prev_mean_sin_sq_attack_angle > US_SIN_SQ_ATTACK_ANGLE_THRESH && be.passed_us_z_thresh){
	    stats->k_est = TASK_US;
	    return;
	}

	if (be.min_stance_theta < UR_ANKLE_ANGLE_THRESH){
	    stats->k_est = TASK_UR;
	    return;
	}

	if (be.passed_ds_z_thresh && be.prev_passed_ds_z_thresh_samples < DS_Z_SAMPLE_THRESH && be.prev_min_stance_theta < DS_ANKLE_ANGLE_THRESH){
	       stats->k_est = TASK_DS;
	       return;
	}
}

//copied from matlab pil
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin)
{
	if (!tm->in_swing){
	    be.min_stance_theta = MIN(be.min_stance_theta, tm->aa);
	    be.max_stance_theta = MAX(be.max_stance_theta, tm->aa);
	}
	else{
	    if (kin->pAz > US_Z_THRESH && !be.passed_us_z_thresh)
	        be.passed_us_z_thresh = 1;
	    
	    if (kin->pAz < DS_Z_THRESH && !be.passed_ds_z_thresh){
	        be.passed_ds_z_thresh_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        be.passed_ds_z_thresh = 1;
	    }
	    
	    if (tm->do_learning_for_stride == 0)
	        be.sum_sin_sq_attack_angle = be.sum_sin_sq_attack_angle + kin->sinSqAttackAngle;

	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.prev_passed_ds_z_thresh_samples = be.passed_ds_z_thresh_samples;
	    be.prev_mean_sin_sq_attack_angle = be.sum_sin_sq_attack_angle/PREDICTION_CUTOFF_SAMPLES;
	}

	if (tm->reset_back_estimator_trigger){
	    tm->reset_back_estimator_trigger = 0;
	    be.sum_sin_sq_attack_angle = 0;
	    be.passed_us_z_thresh = 0;
	    be.passed_ds_z_thresh = 0;
	    be.max_stance_theta = -FLT_MAX;
	    be.prev_min_stance_theta = be.min_stance_theta;
	    be.min_stance_theta = FLT_MAX;
	}

}

//Copied from matlab pil simulation
void init_back_estimator(){
    be.sum_sin_sq_attack_angle = 0.0;
    be.prev_mean_sin_sq_attack_angle = 0.0;
    be.min_stance_theta = 0.0;
    be.max_stance_theta = 0.0;
    be.prev_min_stance_theta = 0.0;
    be.prev_passed_ds_z_thresh_samples = 0.0;
    be.passed_ds_z_thresh_samples = 0;
    be.passed_us_z_thresh = 0;
    be.passed_ds_z_thresh = 0;
    be.completed_back_estimation = 0;
}

struct back_estimator_s*  get_back_estimator(){
  return &be;
}
