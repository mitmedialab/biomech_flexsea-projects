
#include "back_estimation_methods.h"

#define US_Z_THRESH_M  0.3
#define DS_Z_THRESH_M  -0.08
#define US_Z_MAX_SAMPLES  300
#define DS_Z_MAX_SAMPLES  550
#define UR_SLOPE_THRESH_RAD  0.16
#define DR_SLOPE_THRESH_RAD  -0.07


static struct back_estimator_s be;


//copied from matlab pil
void back_estimate(struct statistics_s* stats, struct kinematics_s* kin){
  
	stats->k_est = K_FLAT; 

	uint8_t passed_ur_slope_thresh = 0;
	uint8_t passed_dr_slope_thresh = 0;
	if (kin->ground_slope_est < DR_SLOPE_THRESH_RAD){
	    passed_dr_slope_thresh = 1;
	}
	
	if (kin->ground_slope_est > UR_SLOPE_THRESH_RAD){
	    passed_ur_slope_thresh = 1;
	}
	

	if (passed_ur_slope_thresh &&
	    stats->k_est_prev != K_USTAIRS)
	    stats->k_est = K_URAMP;
	else{
	    if (passed_dr_slope_thresh &&
	        stats->k_est_prev != K_DSTAIRS)
	        stats->k_est = K_DRAMP;
	    else{
	        if (be.prev_stride_paz_thresh_status == PAZ_PASSED_US_THRESH &&
	                be.prev_stride_paz_thresh_pass_samples < US_Z_MAX_SAMPLES &&
	                stats->k_est_prev != K_URAMP)
	            stats->k_est = K_USTAIRS;
	        else{
	            if (be.prev_stride_paz_thresh_status == PAZ_PASSED_DS_THRESH &&
	                    be.prev_stride_paz_thresh_pass_samples < DS_Z_MAX_SAMPLES &&
	                stats->k_est_prev != K_DRAMP){
	                stats->k_est = K_DSTAIRS;
	            }else{
	                stats->k_est = K_FLAT;
	            }
	            
	        }
	    }
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
	    if (be.curr_stride_paz_thresh_status == PAZ_PASSED_NO_THRESH){
	        if (kin->pAz > US_Z_THRESH_M){
	            be.curr_stride_paz_thresh_status = PAZ_PASSED_US_THRESH;
	            be.curr_stride_paz_thresh_pass_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        }else if (kin->pAz < DS_Z_THRESH_M){
	            be.curr_stride_paz_thresh_status = PAZ_PASSED_DS_THRESH;
	            be.curr_stride_paz_thresh_pass_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        }
	    }
    }

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.prev_stride_paz_thresh_status =  be.curr_stride_paz_thresh_status;
	    be.prev_stride_paz_thresh_pass_samples = be.curr_stride_paz_thresh_pass_samples;
	        
	    be.min_stance_theta = FLT_MAX;
	    be.max_stance_theta = -FLT_MAX;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
	    be.curr_stride_paz_thresh_status = PAZ_PASSED_NO_THRESH;
	}

}


//Copied from matlab pil simulation
void init_back_estimator(){

    be.min_stance_theta = 0.0;
    be.max_stance_theta = 0.0;

    be.curr_stride_paz_thresh_status = 0;
    be.curr_stride_paz_thresh_pass_samples = 0;
    be.prev_stride_paz_thresh_status = 0;
    be.prev_stride_paz_thresh_pass_samples = 0;
}

struct back_estimator_s*  get_back_estimator(){
  return &be;
}
