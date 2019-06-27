
#include "back_estimation_methods.h"

//#define DEFAULT_US_Z_THRESH_M  0.3
//#define  DEFAULT_DS_Z_THRESH_M  -0.08
//#define  DEFAULT_US_Z_MAX_SAMPLES  300
//#define  DEFAULT_DS_Z_MAX_SAMPLES  550
//#define  DEFAULT_UR_SLOPE_THRESH_RAD  0.16
//#define  DEFAULT_DR_SLOPE_THRESH_RAD  -0.07


#define US_Z_EARLY_MAX_SAMPLES 200
#define DS_Z_EARLY_MAX_SAMPLES 200
#define US_Z_EARLY_THRESH_M 0.2
#define DS_Z_EARLY_THRESH_M -0.04
#define US_Z_MAX_SAMPLES 300
#define DS_Z_MAX_SAMPLES 500
#define US_Z_THRESH_M 0.3
#define DS_Z_THRESH_M -0.2
#define MAX_STANCE_SAMPLES 1500
#define UR_GROUND_SLOPE_THRESH_RAD -0.05
#define DR_GROUND_SLOPE_THRESH_RAD 0.05
#ifdef POLETEST
#define TORQUE_RANGE_THRESH 10.0
#else
#define TORQUE_RANGE_THRESH 30.0
#endif


static struct back_estimator_s be;


//copied from matlab pil
void back_estimate(struct taskmachine_s* tm, struct statistics_s* stats, struct kinematics_s* kin){
  

	stats->k_est = K_FLAT; 

	if (tm->elapsed_samples > MAX_STANCE_SAMPLES ||
			be.prev_torque_range < TORQUE_RANGE_THRESH){
		return;
	}

	if (be.prev_stride_paz_thresh_status == PAZ_PASSED_US_THRESH &&
			be.prev_stride_paz_thresh_pass_samples < US_Z_EARLY_MAX_SAMPLES){
		stats->k_est = K_USTAIRS;
		return;
	}

	if (be.prev_stride_paz_thresh_status == PAZ_PASSED_DS_THRESH &&
			be.prev_stride_paz_thresh_pass_samples < DS_Z_EARLY_MAX_SAMPLES &&
		kin->prev_ground_slope_est < DR_GROUND_SLOPE_THRESH_RAD){
		stats->k_est = K_DSTAIRS;
		return;
	}

	if (kin->curr_ground_slope_est < UR_GROUND_SLOPE_THRESH_RAD){
		stats->k_est = K_URAMP;
		return;
	}

	if (kin->curr_ground_slope_est > DR_GROUND_SLOPE_THRESH_RAD){
		stats->k_est = K_DRAMP;
	}
}


//copied from matlab pil
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin)
{

	if (be.curr_stride_paz_thresh_status == PAZ_PASSED_NO_THRESH){
		if (kin->pAz > be.us_z_thresh_m){
			be.curr_stride_paz_thresh_status = PAZ_PASSED_US_THRESH;
			be.curr_stride_paz_thresh_pass_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
		}else if (kin->pAz < be.ds_z_thresh_m){
			be.curr_stride_paz_thresh_status = PAZ_PASSED_DS_THRESH;
			be.curr_stride_paz_thresh_pass_samples = tm->elapsed_samples - tm->latest_foot_off_samples;
		}
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.prev_stride_paz_thresh_status =  be.curr_stride_paz_thresh_status;
	    be.prev_stride_paz_thresh_pass_samples = be.curr_stride_paz_thresh_pass_samples;
	    be.prev_stance_samples = tm->latest_foot_off_samples;
	    be.prev_torque_range = be.max_torque - be.min_torque;
	    be.max_torque = -FLT_MAX;
	    be.min_torque = FLT_MAX;
	        
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
	    be.curr_stride_paz_thresh_status = PAZ_PASSED_NO_THRESH;
	}

	if (!tm->in_swing){
		be.max_torque = MAX(be.max_torque, tm->tq);
		be.min_torque = MIN(be.min_torque, tm->tq);
	}

}


//Copied from matlab pil simulation
void init_back_estimator(){

    be.curr_stride_paz_thresh_status = 0;
    be.curr_stride_paz_thresh_pass_samples = 0;
    be.prev_stride_paz_thresh_status = 0;
    be.prev_stride_paz_thresh_pass_samples = 0;

    be.us_z_thresh_m =  US_Z_EARLY_THRESH_M;
	be.ds_z_thresh_m = DS_Z_EARLY_THRESH_M;
	be.us_z_max_samples = US_Z_MAX_SAMPLES;
	be.ds_z_max_samples = DS_Z_MAX_SAMPLES;
	be.ur_slope_thresh_rad = UR_GROUND_SLOPE_THRESH_RAD;
	be.dr_slope_thresh_rad = DR_GROUND_SLOPE_THRESH_RAD;



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
