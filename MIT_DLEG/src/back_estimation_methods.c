
#include "back_estimation_methods.h"

#define RAMP_Y_THRESH_M 0.7

#define UR_Z_THRESH_M 0.17
#define US_Y_THRESH_M 0.19
#define US_Z_THRESH_M 0.2
#define DR_Z_THRESH_M -0.045
#define DS_Y_THRESH_M 0.4
#define DS_Z_THRESH_M -0.07

#define MAX_STANCE_SAMPLES 3000
#define MAX_SWING_SAMPLES 1000
#define GROUND_SLOPE_OFFSET_RAD 0.0
#define GROUND_SLOPE_BIN_SIZE 0.1
#define UR_GROUND_SLOPE_THRESH_RAD (-GROUND_SLOPE_BIN_SIZE + GROUND_SLOPE_OFFSET_RAD)
#define DR_GROUND_SLOPE_THRESH_RAD (GROUND_SLOPE_BIN_SIZE + GROUND_SLOPE_OFFSET_RAD)
#define UR_PAZ_THRESH_M -0.05
#define DR_PAZ_THRESH_M 0.05
#define TORQUE_THRESH 30.0
#define MAX_DISPLACEMENT_THRESH_M 1.7
#define PAY_GOOD_TRAJECTORY_THRESH_M -0.03
#define PAZ_GOOD_TRAJECTORY_THRESH_M 0.005
#define STANCE_AOMEGAX_THRESH_RPS -2.0
#define FOOT_OFF_PAY_THRESH_M 0.2


static struct back_estimator_s be;

static int allow_flat_to_dr_transition = 1;

void update_back_estimation_features(struct taskmachine_s* tm, struct kinematics_s* kin)
{

	if (kin->pAy < PAY_GOOD_TRAJECTORY_THRESH_M){
		be.stepping_backwards = 1;
	}
	if (kin->pAz > PAZ_GOOD_TRAJECTORY_THRESH_M){
		be.passed_vertical_thresh = 1;
	}

	if (tm->in_swing && !be.stepping_backwards && be.passed_vertical_thresh && !be.made_prediction && kin->foot_off_pAy < FOOT_OFF_PAY_THRESH_M){
		be.ready_for_prediction = 1;
	}else{
		be.ready_for_prediction = 0;
	}

	if (be.ready_for_prediction){

		if (kin->displacement > MAX_DISPLACEMENT_THRESH_M ||
			tm->elapsed_samples - tm->latest_foot_off_samples > MAX_SWING_SAMPLES){
			be.made_prediction = 1;
			return;
		}
		//If we are past horizontal stair ascent thresh, then if ground slope was steep enough or we pass vertical ramp
		//ascent thresh, identify as ramp ascent.
		if (kin->pAz > be.us_z_thresh_m){
			if (kin->pAy < be.us_y_thresh_m){
				be.prediction = K_USTAIRS;
				be.made_prediction = 1;
				return;
			}
		}

		if (kin->pAz < be.ds_z_thresh_m){
			if (kin->pAy < be.ds_y_thresh_m && tm->elapsed_samples - tm->latest_foot_off_samples < 390.0 && be.max_torque > 35.0){
				be.prediction = K_DSTAIRS;
				be.made_prediction = 1;
				return;
			}
		}

		if (be.slope_thresh_status == PASSED_UR_SLOPE_THRESH){
			be.prediction = K_URAMP;
			be.made_prediction = 1;
			return;
		}

		if (be.slope_thresh_status == PASSED_DR_SLOPE_THRESH){
			be.prediction = K_DRAMP;
			be.made_prediction = 1;
			return;
		}

		if (kin->pAy > be.ur_y_thresh_m){
			if (kin->pAz > be.ur_z_thresh_m ){
				be.made_prediction = 1;
				be.prediction = K_URAMP;
				return;
			}
			if (kin->pAz < be.dr_z_thresh_m && allow_flat_to_dr_transition ){
				if (allow_flat_to_dr_transition)
					allow_flat_to_dr_transition = 0;
				be.made_prediction = 1;
				be.prediction = K_DRAMP;
				return;
			}

		}
		return;
	}

	if(tm->elapsed_samples == kin->latest_foot_static_samples){
		 be.passed_vertical_thresh = 0;
		 be.stepping_backwards = 0;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.max_torque = -FLT_MAX;
	    be.min_torque = FLT_MAX;
	    be.max_aa = -FLT_MAX;
	    be.ready_for_prediction = 0;
	}

	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){
		allow_flat_to_dr_transition = 1;
		be.slope_thresh_status = PASSED_NO_SLOPE_THRESH;
		be.torque_range = be.max_torque - be.min_torque;
		be.prediction = K_FLAT;

	    if (tm->elapsed_samples < MAX_STANCE_SAMPLES &&
			(be.max_torque > TORQUE_THRESH || be.min_torque < -TORQUE_THRESH) &&
			kin->latest_foot_static_samples != 0){
	    	be.made_prediction = 0;
	    }else{
	    	return;
	    }

	    if (kin->curr_ground_slope_est < (-be.ground_slope_bin_size + be.ground_slope_offset_rad) &&
	    		kin->foot_off_vA > 0.8){
	    	be.slope_thresh_status = PASSED_UR_SLOPE_THRESH;
	    }

		if (kin->curr_ground_slope_est > (be.ground_slope_bin_size + be.ground_slope_offset_rad) &&
				be.max_aa > 0.08){
			be.slope_thresh_status = PASSED_DR_SLOPE_THRESH;
		}
	}

	if (!tm->in_swing){
		be.max_torque = MAX(be.max_torque, tm->tq);
		be.min_torque = MIN(be.min_torque, tm->tq);
		be.max_aa = MAX(be.max_aa, tm->aa);
	}

}

void init_back_estimator(){
	be.ready_for_prediction = 0;
	be.stepping_backwards = 0;
	be.passed_vertical_thresh = 0;
	be.slope_thresh_status = PASSED_NO_SLOPE_THRESH;
	be.prediction = K_FLAT;
    be.ur_z_thresh_m =  UR_Z_THRESH_M;
    be.ur_y_thresh_m =  RAMP_Y_THRESH_M;
    be.us_z_thresh_m =  US_Z_THRESH_M;
    be.us_y_thresh_m =  US_Y_THRESH_M;
	be.dr_z_thresh_m = DR_Z_THRESH_M;
	be.dr_y_thresh_m = RAMP_Y_THRESH_M;
	be.ds_z_thresh_m = DS_Z_THRESH_M;
	be.ds_y_thresh_m = DS_Y_THRESH_M;
	be.ur_slope_thresh_rad = UR_GROUND_SLOPE_THRESH_RAD;
	be.dr_slope_thresh_rad = DR_GROUND_SLOPE_THRESH_RAD;
	be.ground_slope_bin_size = GROUND_SLOPE_BIN_SIZE;
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
