
#include "back_estimation_methods.h"

#define DEFAULT_US_Z_THRESH_M  0.3
#define  DEFAULT_DS_Z_THRESH_M  -0.08
#define  DEFAULT_US_Z_MAX_SAMPLES  300
#define  DEFAULT_DS_Z_MAX_SAMPLES  550
#define  DEFAULT_UR_SLOPE_THRESH_RAD  0.16
#define  DEFAULT_DR_SLOPE_THRESH_RAD  -0.07


static struct back_estimator_s be;


//copied from matlab pil
void back_estimate(struct statistics_s* stats, struct kinematics_s* kin){
  
	stats->k_est = K_FLAT; 

	uint8_t passed_ur_slope_thresh = 0;
	uint8_t passed_dr_slope_thresh = 0;
	if (kin->ground_slope_est < be.dr_slope_thresh_rad){
	    passed_dr_slope_thresh = 1;
	}
	
	if (kin->ground_slope_est > be.ur_slope_thresh_rad){
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
	                be.prev_stride_paz_thresh_pass_samples < be.us_z_max_samples &&
	                stats->k_est_prev != K_URAMP)
	            stats->k_est = K_USTAIRS;
	        else{
	            if (be.prev_stride_paz_thresh_status == PAZ_PASSED_DS_THRESH &&
	                    be.prev_stride_paz_thresh_pass_samples < be.ds_z_max_samples &&
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
	        if (kin->pAz > be.us_z_thresh_m){
	            be.curr_stride_paz_thresh_status = PAZ_PASSED_US_THRESH;
	            be.curr_stride_paz_thresh_pass_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
	        }else if (kin->pAz < be.ds_z_thresh_m){
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

    be.us_z_thresh_m =  DEFAULT_US_Z_THRESH_M;
	be.ds_z_thresh_m = DEFAULT_DS_Z_THRESH_M;
	be.us_z_max_samples = DEFAULT_US_Z_MAX_SAMPLES;
	be.ds_z_max_samples = DEFAULT_DS_Z_MAX_SAMPLES;
	be.ur_slope_thresh_rad = DEFAULT_UR_SLOPE_THRESH_RAD;
	be.dr_slope_thresh_rad = DEFAULT_DR_SLOPE_THRESH_RAD;



}

struct back_estimator_s*  get_back_estimator(){
  return &be;
}

void set_us_z_thresh_m(float val){
	be.us_z_thresh_m = val;
}
void set_ds_z_thresh_m(float val){
	be.ds_z_thresh_m = val;
}
void set_us_z_max_samples(int val){
	be.us_z_max_samples = val;
}
void set_ds_z_max_samples(int val){
	be.ds_z_max_samples = val;
}
void set_ur_slope_thresh_rad(float val){
	be.ur_slope_thresh_rad = val;
}
void set_dr_slope_thresh_rad(float val){
	be.dr_slope_thresh_rad = val;
}
