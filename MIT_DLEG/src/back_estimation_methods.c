
#include "back_estimation_methods.h"

 //Back estimation constants (copied from matlab pil)
#define US_Z_THRESH_M  0.28
#define DS_Z_THRESH_M  -0.1
#define US_Z_MAX_SAMPLES  200
#define DS_Z_MAX_SAMPLES  400
#define DS_Z_MIN_SAMPLES  100
#define UR_MAX_ANKLE_ANGLE_DEG  -10.0
#define DR_MIN_ANKLE_ANGLE_DEG  5.0
#define US_MIN_SIN_SQ_ATTACK_ANGLE_RAD  0.2
#define MAX_DORSIFLEXION_ANGLE_DEG  -3.0
#define MIN_DORSIFLEXION_ANGLE_DEG  -10.0
#define LATE_STANCE_TORQUE_THRESHOLD_NM  10.0
#define DR_MIN_MEAN_LATE_STANCE_PITCH_RAD  0.15
#define UR_MIN_DORSIFLEXION_RATIO 0.8
#define UR_MIN_MEAN_LATE_STANCE_PITCH_RAD 0.0


static struct back_estimator_s be;


//copied from matlab pil
void back_estimate(struct taskmachine_s* tm, struct statistics_s* stats){
  
	stats->k_est = K_FLAT;

	tm->reset_back_estimator_trigger = 1;

	be.dorsiflexion_ratio = ((float) be.dorsiflexion_counter)/tm->latest_foot_off_samples;
	if (be.late_stance_pitch_counter > 0)
	    be.mean_late_stance_pitch = be.sum_late_stance_pitch/((float)be.late_stance_pitch_counter);
	else
	    be.mean_late_stance_pitch = 0.0;

	if (be.passed_us_z_thresh &&
	        be.passed_us_z_thresh_samples < US_Z_MAX_SAMPLES &&
	        be.prev_mean_sin_sq_attack_angle > US_MIN_SIN_SQ_ATTACK_ANGLE_RAD){
	        stats->k_est = K_USTAIRS;
	       return;
	}

	if (be.max_stance_theta > DR_MIN_ANKLE_ANGLE_DEG &&
			be.mean_late_stance_pitch > DR_MIN_MEAN_LATE_STANCE_PITCH_RAD){
	    stats->k_est = K_DRAMP;
	    return;
	}

	if (be.passed_ds_z_thresh &&
	         be.passed_ds_z_thresh_samples < DS_Z_MAX_SAMPLES &&
	         be.passed_ds_z_thresh_samples > DS_Z_MIN_SAMPLES){
	        stats->k_est = K_DSTAIRS;
	       return;
	}

	if (be.min_stance_theta < UR_MAX_ANKLE_ANGLE_DEG &&
			(be.dorsiflexion_ratio > UR_MIN_DORSIFLEXION_RATIO || be.mean_late_stance_pitch < UR_MIN_MEAN_LATE_STANCE_PITCH_RAD )){
	    stats->k_est = K_URAMP;
	    return;
	    }
}


//copied from matlab pil
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin)
{
	if (!tm->in_swing){

		if (tm->aa < MAX_DORSIFLEXION_ANGLE_DEG)
			be.dorsiflexion_counter = be.dorsiflexion_counter + 1;

		if (tm->tq > LATE_STANCE_TORQUE_THRESHOLD_NM && tm->aa < MAX_DORSIFLEXION_ANGLE_DEG && tm->aa > MIN_DORSIFLEXION_ANGLE_DEG){
			be.sum_late_stance_pitch = be.sum_late_stance_pitch + kin->rot3;
			be.late_stance_pitch_counter = be.late_stance_pitch_counter + 1;
		}

	    be.min_stance_theta = MIN(be.min_stance_theta, tm->aa);
	    be.max_stance_theta = MAX(be.max_stance_theta, tm->aa);
	}
	else{
	    if (kin->pAz > US_Z_THRESH_M && !be.passed_us_z_thresh){
	    	be.passed_us_z_thresh_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        be.passed_us_z_thresh = 1;
	    }
	    
	    if (kin->pAz < DS_Z_THRESH_M && !be.passed_ds_z_thresh){
	        be.passed_ds_z_thresh_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        be.passed_ds_z_thresh = 1;
	    }
	    
	    if (tm->do_learning_for_stride == 0)
	        be.sum_sin_sq_attack_angle = be.sum_sin_sq_attack_angle + kin->sinSqAttackAngle;

	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.prev_mean_sin_sq_attack_angle = be.sum_sin_sq_attack_angle/PREDICTION_CUTOFF_SAMPLES;

	    be.sum_late_stance_pitch = 0.0;
	    be.late_stance_pitch_counter = 0;
	    be.dorsiflexion_counter = 0;

	    be.min_stance_theta = FLT_MAX;
	    be.max_stance_theta = -FLT_MAX;
	}

	if (tm->reset_back_estimator_trigger){
	    tm->reset_back_estimator_trigger = 0;
	    be.sum_sin_sq_attack_angle = 0;
	    be.passed_us_z_thresh = 0;
	    be.passed_ds_z_thresh = 0;
	}
}


//Copied from matlab pil simulation
void init_back_estimator(){
    be.sum_sin_sq_attack_angle = 0.0;
    be.prev_mean_sin_sq_attack_angle = 0.0;
    be.min_stance_theta = 0.0;
    be.max_stance_theta = 0.0;
    be.prev_min_stance_theta = 0.0;
    be.passed_us_z_thresh_samples = 0;
    be.passed_ds_z_thresh_samples = 0;
    be.passed_us_z_thresh = 0;
    be.passed_ds_z_thresh = 0;
    be.completed_back_estimation = 0;

    be.dorsiflexion_ratio = 0.0;
	be.dorsiflexion_counter = 0;
	be.sum_late_stance_pitch = 0.0;
	be.mean_late_stance_pitch = 0.0;
	be.late_stance_pitch_counter = 0;
}

struct back_estimator_s*  get_back_estimator(){
  return &be;
}
