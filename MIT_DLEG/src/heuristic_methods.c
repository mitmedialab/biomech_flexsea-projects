
#include <heuristic_methods.h>

#define UR_Y_EARLY_THRESH_M 0.65
#define UR_Z_EARLY_THRESH_M 0.13
#define US_Y_EARLY_THRESH_M 0.2
#define US_Z_EARLY_THRESH_M 0.2
#define DR_Y_EARLY_THRESH_M 0.65
#define DR_Z_EARLY_THRESH_M -0.05
#define DS_Y_EARLY_THRESH_M 0.45
#define DS_Z_EARLY_THRESH_M -0.08

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


static struct heuristics_s be;

//Use measurements and associated thresholds to determine what the current walking terrain is.
void update_heuristics(struct taskmachine_s* tm, struct kinematics_s* kin)
{

	if (kin->pAy < PAY_GOOD_TRAJECTORY_THRESH_M){
		be.stepping_backwards = 1;
	}
	if (kin->pAz > PAZ_GOOD_TRAJECTORY_THRESH_M){
		be.passed_vertical_thresh = 1;
	}

	//Only predict a non-level-ground terrain if certain conditions are met.
	if (tm->in_swing && !be.stepping_backwards && be.passed_vertical_thresh && !be.made_prediction){
		be.ready_for_prediction = 1;
	}else{
		be.ready_for_prediction = 0;
	}

	if (be.ready_for_prediction){

		//Conclude level ground if displacement is really big or a lot of time has passed in swing.
		if (kin->displacement > MAX_DISPLACEMENT_THRESH_M ||
			tm->elapsed_samples - tm->latest_foot_off_samples > MAX_SWING_SAMPLES){
			be.made_prediction = 1;
			return;
		}

		//Heuristic rules

		if (kin->pAy < be.us_y_thresh_m){
			if (kin->pAz > be.us_z_thresh_m){
				be.prediction = K_USTAIRS;
				be.made_prediction = 1;
				return;
			}
		}else{
			if (be.slope_thresh_status == PASSED_UR_SLOPE_THRESH){
				be.prediction = K_URAMP;
				be.made_prediction = 1;
				return;
			}
		}

		if (kin->pAy < be.ds_y_thresh_m){
			if (kin->pAz < be.ds_z_thresh_m && tm->elapsed_samples - tm->latest_foot_off_samples < 600){
				be.prediction = K_DSTAIRS;
				be.made_prediction = 1;
				return;
			}
//		}
		}else{
			if (be.slope_thresh_status == PASSED_DR_SLOPE_THRESH){
				be.prediction = K_DRAMP;
				be.made_prediction = 1;
				return;
			}
		}

		//Optional heuristics for identifying the first stride transitioning onto a ramp. Works about half the time.
		if (kin->pAy > be.ur_y_thresh_m){
			be.made_prediction = 1;
			if (be.min_stance_aOmegaX < STANCE_AOMEGAX_THRESH_RPS){
				if (kin->pAz > be.ur_z_thresh_m ){
					be.prediction = K_URAMP;
				}
				else if (kin->pAz < be.dr_z_thresh_m ){
					be.prediction = K_DRAMP;
				}else{
					be.prediction = K_FLAT;
				}
			}else{
				be.prediction = K_FLAT;
			}
			return;
		}
	}


	if(tm->elapsed_samples == kin->latest_foot_static_samples){
		 be.passed_vertical_thresh = 0;
		 be.stepping_backwards = 0;
	}

	//Reset certain max/min values at foot strike.
	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_ON){
	    be.max_torque = -FLT_MAX;
	    be.min_torque = FLT_MAX;
	    be.min_stance_aOmegaX = FLT_MAX;
	    be.ready_for_prediction = 0;
	}


	if (tm->gait_event_trigger == GAIT_EVENT_FOOT_OFF){

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

	    //if passed a certain slope thresh and rolled over the foot with sufficient rotational velocity, identify
	    //as ramp ascent/descent.
	    if (kin->curr_ground_slope_est < (-be.ground_slope_bin_size + be.ground_slope_offset_rad) &&
	    			kin->end_of_stride_pAz > UR_PAZ_THRESH_M && be.min_stance_aOmegaX < STANCE_AOMEGAX_THRESH_RPS){
	    	be.slope_thresh_status = PASSED_UR_SLOPE_THRESH;
	    }

		if (kin->curr_ground_slope_est > (be.ground_slope_bin_size + be.ground_slope_offset_rad) &&
				kin->end_of_stride_pAz < DR_PAZ_THRESH_M && be.min_stance_aOmegaX < STANCE_AOMEGAX_THRESH_RPS){
			be.slope_thresh_status = PASSED_DR_SLOPE_THRESH;
		}
	}

	if (!tm->in_swing){
		be.max_torque = MAX(be.max_torque, tm->tq);
		be.min_torque = MIN(be.min_torque, tm->tq);
		be.min_stance_aOmegaX = MIN(be.min_stance_aOmegaX, kin->aOmegaX);
	}

}

void init_heuristics(){
	be.ready_for_prediction = 0;
	be.stepping_backwards = 0;
	be.passed_vertical_thresh = 0;
	be.slope_thresh_status = PASSED_NO_SLOPE_THRESH;
	be.prediction = K_FLAT;
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
	be.ground_slope_bin_size = GROUND_SLOPE_BIN_SIZE;
	be.ground_slope_offset_rad = GROUND_SLOPE_OFFSET_RAD;


}

struct heuristics_s*  get_heuristics(){
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
