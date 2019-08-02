
#include "back_estimation_methods.h"

#define UR_Y_EARLY_THRESH_M 0.65
#define UR_Z_EARLY_THRESH_M 0.13
#define US_Y_EARLY_THRESH_M 0.18
#define US_Z_EARLY_THRESH_M 0.2
#define DR_Y_EARLY_THRESH_M 0.65
#define DR_Z_EARLY_THRESH_M -0.05
#define DS_Y_EARLY_THRESH_M 0.45
#define DS_Z_EARLY_THRESH_M -0.08

#define MAX_GAIT_PHASE_SAMPLES 2000
#define GROUND_SLOPE_OFFSET_RAD 0.08
#define GROUND_SLOPE_BIN_SIZE 0.04
#define UR_GROUND_SLOPE_THRESH_RAD (-GROUND_SLOPE_BIN_SIZE + GROUND_SLOPE_OFFSET_RAD)
#define DR_GROUND_SLOPE_THRESH_RAD (GROUND_SLOPE_BIN_SIZE + GROUND_SLOPE_OFFSET_RAD)
#define UR_PAZ_THRESH_M -0.05
#define DR_PAZ_THRESH_M 0.05
#define TORQUE_RANGE_THRESH 20.0



static struct back_estimator_s be;


//copied from matlab pil
void back_estimate( struct taskmachine_s* tm, struct kinematics_s* kin, struct statistics_s* stats){
  

	stats->k_est = K_DEFAULT;

	if (be.prev_stance_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.prev_swing_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.prev_torque_range < TORQUE_RANGE_THRESH ||
			tm->elapsed_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.torque_range < TORQUE_RANGE_THRESH){
		return;
	}
	stats->k_est = K_FLAT;

	if (be.prev_stride_paz_thresh_status == PASSED_US_THRESH){
		stats->k_est = K_USTAIRS;
		return;
	}

	if (be.prev_stride_paz_thresh_status == PASSED_DS_THRESH){
		stats->k_est = K_DSTAIRS;
		return;
	}

	if (kin->curr_ground_slope_est < (-GROUND_SLOPE_BIN_SIZE + be.ground_slope_offset_rad) &&
			kin->end_of_stride_pAz > UR_PAZ_THRESH_M){
		stats->k_est = K_URAMP;
		return;
	}

	if (kin->curr_ground_slope_est > (GROUND_SLOPE_BIN_SIZE + be.ground_slope_offset_rad) &&
			kin->end_of_stride_pAz < DR_PAZ_THRESH_M){
		stats->k_est = K_DRAMP;
		return;
	}
}


//copied from matlab pil
void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin)
{




	if (be.curr_stride_paz_thresh_status == PASSED_NO_THRESH && !be.estimation_completed){
		if (kin->pAz > be.us_z_thresh_m &&
				kin->pAy < be.us_y_thresh_m){
			be.curr_stride_paz_thresh_status = PASSED_US_THRESH;
		}else if (kin->pAz < be.ds_z_thresh_m &&
				kin->pAy < be.ds_y_thresh_m){
			be.curr_stride_paz_thresh_status = PASSED_DS_THRESH;
		}else if (kin->pAz > be.ur_z_thresh_m &&
				kin->pAy > be.ur_y_thresh_m){
			be.curr_stride_paz_thresh_status = PASSED_UR_THRESH;
		}else if (kin->pAz < be.dr_z_thresh_m &&
				kin->pAy < be.dr_y_thresh_m){
			be.curr_stride_paz_thresh_status = PASSED_DR_THRESH;
			}
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.prev_stance_samples = tm->latest_foot_off_samples;
	    be.prev_swing_samples = tm->prev_stride_samples - tm->latest_foot_off_samples;
	    be.max_torque = -FLT_MAX;
	    be.min_torque = FLT_MAX;
	    be.estimation_completed = 0;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
		be.prev_torque_range = be.torque_range;
		be.torque_range = be.max_torque - be.min_torque;
		be.prev_stride_paz_thresh_status =  be.curr_stride_paz_thresh_status;
	    be.curr_stride_paz_thresh_status = PASSED_NO_THRESH;

	    if (be.prev_stance_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.prev_swing_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.prev_torque_range < TORQUE_RANGE_THRESH ||
			tm->elapsed_samples > MAX_GAIT_PHASE_SAMPLES ||
			be.torque_range < TORQUE_RANGE_THRESH){
	    	be.estimation_completed = 1;
	    	return;
	    }
	    if (kin->curr_ground_slope_est < (-GROUND_SLOPE_BIN_SIZE + be.ground_slope_offset_rad) &&
	    			kin->end_of_stride_pAz > UR_PAZ_THRESH_M){
	    	be.curr_stride_paz_thresh_status = PASSED_UR_THRESH;
	    	be.estimation_completed = 1;
	    }

		if (kin->curr_ground_slope_est > (GROUND_SLOPE_BIN_SIZE + be.ground_slope_offset_rad) &&
				kin->end_of_stride_pAz < DR_PAZ_THRESH_M){
			be.curr_stride_paz_thresh_status = PASSED_DR_THRESH;
			be.estimation_completed = 1;
		}
	}

	if (!tm->in_swing){
		be.max_torque = MAX(be.max_torque, tm->tq);
		be.min_torque = MIN(be.min_torque, tm->tq);
	}

}


//Copied from matlab pil simulation
void init_back_estimator(){
	be.estimation_completed = 0;
    be.ur_z_thresh_m =  UR_Z_EARLY_THRESH_M;
    be.ur_y_thresh_m =  UR_Y_EARLY_THRESH_M;
    be.us_z_thresh_m =  US_Z_EARLY_THRESH_M;
    be.us_y_thresh_m =  US_Y_EARLY_THRESH_M;
	be.dr_z_thresh_m = DR_Z_EARLY_THRESH_M;
	be.dr_y_thresh_m = DR_Y_EARLY_THRESH_M;
	be.ds_z_thresh_m = DS_Z_EARLY_THRESH_M;
	be.ds_y_thresh_m = DS_Y_EARLY_THRESH_M;
	be.ur_slope_thresh_rad = UR_GROUND_SLOPE_THRESH_RAD;
	be.dr_slope_thresh_rad = DR_GROUND_SLOPE_THRESH_RAD;
	be.ground_slope_offset_rad = GROUND_SLOPE_OFFSET_RAD;


}

struct back_estimator_s*  get_back_estimator(){
  return &be;
}

void set_us_thresh_m(float yval, float zval){
	be.us_y_thresh_m = yval;
	be.us_z_thresh_m = zval;
}

void set_ur_thresh_m(float yval, float zval){
	be.ur_y_thresh_m = yval;
	be.ur_z_thresh_m = zval;
}

void set_ds_thresh_m(float yval, float zval){
	be.ds_y_thresh_m = yval;
	be.ds_z_thresh_m = zval;
}

void set_dr_thresh_m(float yval, float zval){
	be.dr_y_thresh_m = yval;
	be.dr_z_thresh_m = zval;
}

void set_ur_slope_thresh_rad(float val){
	be.ur_slope_thresh_rad = val;
}
void set_dr_slope_thresh_rad(float val){
	be.dr_slope_thresh_rad = val;
}
void set_dr_slope_thresh_rad(float val){
	be.dr_slope_thresh_rad = val;
}
