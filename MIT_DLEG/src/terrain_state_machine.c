

#include <terrain_state_machine.h>




static int state_machine_demux_state = STATE_EARLY_SWING;


static struct control_params_s cp;
static struct minimum_jerk_values_s mj;

static void init_adaptive_control_params_s()
{

	cp.adaptive.hard_stop_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.hard_stop_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.adaptive.lsw_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.est_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.est_b_Nm_p_rps = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.lst_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.lst_b_Nm_p_rps = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.lst_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.adaptive.est_lst_min_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));



}

static void init_minimum_jerk_values_s(){
	mj.params = (float*)calloc(6, sizeof(float));
	mj.T = (float*)calloc(6, sizeof(float));
	mj.T[0] = 1.0;
	mj.T[1] = DEFAULT_MINIMUM_JERK_TRAJECTORY_TIME;
	mj.T[2] = mj.T[1]*mj.T[1];
	mj.T[3] = mj.T[2]*mj.T[1];
	mj.T[4] = mj.T[3]*mj.T[1];
	mj.T[5] = mj.T[4]*mj.T[1];
	mj.update_counter = UINT_MAX;
	mj.angle_tol_rad = DEFAULT_MINIMUM_JERK_ANGLE_TOL_RAD;
	mj.trajectory_defined = 0;
	mj.enabled = 1;

}

static float set_minimum_jerk_trajectory_params(Act_s* actx, float theta_target, float theta_dot_target, float theta_ddot_target, float desired_trajectory_period_s){

	float distance_to_target_rads = fabs(theta_target - actx->jointAngle);

	float trajectory_period_s;
	
	if (distance_to_target_rads > MINIMUM_JERK_MAX_MEAN_SPEED_RPS * desired_trajectory_period_s ){
		trajectory_period_s = distance_to_target_rads / MINIMUM_JERK_MAX_MEAN_SPEED_RPS;
	}else{
		trajectory_period_s = desired_trajectory_period_s;
	}

	mj.theta_target  = theta_target;
	mj.theta_dot_target = theta_dot_target;
	mj.theta_ddot_target = theta_ddot_target;
	
	set_minimum_jerk_trajectory_period(trajectory_period_s);

	

	mj.params[0] = actx->jointAngle;
	mj.params[1] = 0.0;
	mj.params[2] = 0.0;

	float b3 = mj.theta_target - actx->jointAngle - mj.T[1]*mj.params[1] - 0.5*mj.T[2]*mj.params[2];
	float b4 = mj.theta_dot_target - mj.params[1] - mj.T[1]*mj.params[2];
	float b5 = mj.theta_ddot_target - mj.params[2];

	mj.params[3] = (10.0/mj.T[3])*b3 + (-4.0/mj.T[2])*b4 + (0.5/mj.T[1])*b5;
	mj.params[4] = (-15.0/mj.T[4])*b3 + (7.0/mj.T[3])*b4 + (-1.0/mj.T[2])*b5;
	mj.params[5] = (6.0/mj.T[5])*b3 + (-3.0/mj.T[4])*b4 + (0.5/mj.T[3])*b5;

	mj.update_counter = 0;
	mj.total_trajectory_updates =(uint)(mj.T[1] * 1000.0); //TODO: put the actual variable for sampling rate here
}

static void set_next_theta_for_minimum_jerk(Act_s* actx){

	// if (fabs(mj.des_theta - actx->jointAngle) < mj.angle_tol_rad)
	mj.update_counter++;

	float t = (float)(mj.update_counter)/1000.0;
	float t2 = t*t;
	float t3 = t2*t;
	float t4 = t3*t;
	float t5 = t4*t;

	mj.des_theta = mj.params[0] + mj.params[1]*t + mj.params[2]*t2 + mj.params[3]*t3 +
		mj.params[4]*t4 + mj.params[5]*t5;
}

static void set_joint_torque(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b) {
	actx->tauDes = k * (des_theta - actx->jointAngle) - b * actx->jointVel ;
	setMotorTorque(actx, actx->tauDes);
}

static void set_joint_torque_with_hardstop(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b, float hs_theta, float hs_k) {
	actx->tauDes = k * (des_theta - tm->aa*RAD_PER_DEG) - b * tm->aa_dot ;
	if (actx->jointAngle < hs_theta){
		actx->tauDes = actx->tauDes - hs_k*(tm->aa*RAD_PER_DEG - hs_theta);
	}
	setMotorTorque(actx, actx->tauDes);
}



static void set_control_params_for_terrain(int terrain){

	cp.active.hard_stop_theta_rad = cp.adaptive.hard_stop_theta_rad[terrain];
	cp.active.hard_stop_k_Nm_p_rad = cp.adaptive.hard_stop_k_Nm_p_rad[terrain];

	cp.active.lsw_theta_rad = cp.adaptive.lsw_theta_rad[terrain];
	cp.active.est_k_Nm_p_rad = cp.adaptive.est_k_Nm_p_rad[terrain];
	cp.active.est_b_Nm_p_rps = cp.adaptive.est_b_Nm_p_rps[terrain];
	cp.active.lst_k_Nm_p_rad = cp.adaptive.lst_k_Nm_p_rad[terrain];
	cp.active.lst_b_Nm_p_rps = cp.adaptive.lst_b_Nm_p_rps[terrain];
	cp.active.lst_theta_rad = cp.adaptive.lst_theta_rad[terrain];

	cp.active.est_lst_min_theta_rad = cp.adaptive.est_lst_min_theta_rad[terrain];
}

void reset_terrain_state_machine_parameters(){
	cp.adaptive.hard_stop_theta_rad[K_FLAT] = DEFAULT_FLAT_HS_THETA_RAD;
	cp.adaptive.hard_stop_theta_rad[K_URAMP] = DEFAULT_URAMP_HS_THETA_RAD;
	cp.adaptive.hard_stop_theta_rad[K_DRAMP] = DEFAULT_DRAMP_HS_THETA_RAD;
	cp.adaptive.hard_stop_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_HS_THETA_RAD;
	cp.adaptive.hard_stop_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_HS_THETA_RAD;

	cp.adaptive.hard_stop_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_HS_K_NM_P_RAD;
	cp.adaptive.hard_stop_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_HS_K_NM_P_RAD;
	cp.adaptive.hard_stop_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_HS_K_NM_P_RAD;
	cp.adaptive.hard_stop_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_HS_K_NM_P_RAD;
	cp.adaptive.hard_stop_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_HS_K_NM_P_RAD;

	cp.adaptive.lsw_theta_rad[K_FLAT] = DEFAULT_FLAT_LSW_THETA_RAD;
	cp.adaptive.lsw_theta_rad[K_URAMP] = DEFAULT_URAMP_LSW_THETA_RAD;
	cp.adaptive.lsw_theta_rad[K_DRAMP] = DEFAULT_DRAMP_LSW_THETA_RAD;
	cp.adaptive.lsw_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_LSW_THETA_RAD;
	cp.adaptive.lsw_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LSW_THETA_RAD;

	cp.adaptive.est_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_EST_K_NM_P_RAD;
	cp.adaptive.est_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_EST_K_NM_P_RAD;
	cp.adaptive.est_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_EST_K_NM_P_RAD;
	cp.adaptive.est_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_EST_K_NM_P_RAD;
	cp.adaptive.est_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_K_NM_P_RAD;

	cp.adaptive.est_b_Nm_p_rps[K_FLAT] = DEFAULT_FLAT_EST_B_NM_P_RPS;
	cp.adaptive.est_b_Nm_p_rps[K_URAMP] = DEFAULT_URAMP_EST_B_NM_P_RPS;
	cp.adaptive.est_b_Nm_p_rps[K_DRAMP] = DEFAULT_DRAMP_EST_B_NM_P_RPS;
	cp.adaptive.est_b_Nm_p_rps[K_USTAIRS] = DEFAULT_USTAIRS_EST_B_NM_P_RPS;
	cp.adaptive.est_b_Nm_p_rps[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_B_NM_P_RPS;

	cp.adaptive.lst_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_K_NM_P_RAD;

	cp.adaptive.lst_b_Nm_p_rps[K_FLAT] = DEFAULT_FLAT_LST_B_NM_P_RPS;
	cp.adaptive.lst_b_Nm_p_rps[K_URAMP] = DEFAULT_URAMP_LST_B_NM_P_RPS;
	cp.adaptive.lst_b_Nm_p_rps[K_DRAMP] = DEFAULT_DRAMP_LST_B_NM_P_RPS;
	cp.adaptive.lst_b_Nm_p_rps[K_USTAIRS] = DEFAULT_USTAIRS_LST_B_NM_P_RPS;
	cp.adaptive.lst_b_Nm_p_rps[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_B_NM_P_RPS;

	cp.adaptive.lst_theta_rad[K_FLAT] = DEFAULT_FLAT_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_URAMP] = DEFAULT_URAMP_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_DRAMP] = DEFAULT_DRAMP_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_THETA_RAD;

	cp.adaptive.est_lst_min_theta_rad[K_FLAT] = DEFAULT_FLAT_EST_LST_MIN_THETA_RAD;
	cp.adaptive.est_lst_min_theta_rad[K_URAMP] = DEFAULT_URAMP_EST_LST_MIN_THETA_RAD;
	cp.adaptive.est_lst_min_theta_rad[K_DRAMP] = DEFAULT_DRAMP_EST_LST_MIN_THETA_RAD;
	cp.adaptive.est_lst_min_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_EST_LST_MIN_THETA_RAD;
	cp.adaptive.est_lst_min_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_LST_MIN_THETA_RAD;

	cp.active.esw_theta_rad = DEFAULT_ESW_THETA_RAD;
	cp.active.sw_k_Nm_p_rad = DEFAULT_SW_K_NM_P_RAD;
	cp.active.sw_b_Nm_p_rps = DEFAULT_SW_B_NM_P_RPS;

	cp.nominal.theta_rad = DEFAULT_NOMINAL_THETA_RAD;
	cp.nominal.k_Nm_p_rad = DEFAULT_NOMINAL_K_NM_P_RAD;
	cp.nominal.b_Nm_p_rps = DEFAULT_NOMINAL_B_NM_P_RPS;
}



void init_terrain_state_machine(){
	init_adaptive_control_params_s();
	reset_terrain_state_machine_parameters();
	init_minimum_jerk_values_s();
}

int get_walking_state(){
	return state_machine_demux_state;
}

struct minimum_jerk_values_s* get_minimum_jerk_values(){
	return &mj;
}
struct control_params_s* get_control_params(){
	return &cp;
}

void set_minimum_jerk_trajectory_period(float T){
	mj.T[1] = T;
	mj.T[2] = mj.T[1]*mj.T[1];
	mj.T[3] = mj.T[2]*mj.T[1];
	mj.T[4] = mj.T[3]*mj.T[1];
	mj.T[5] = mj.T[4]*mj.T[1];
}

void set_minimum_jerk_angle_tol_rad(float angle_tol_rad){
	mj.angle_tol_rad = angle_tol_rad;
}

void enable_minimum_jerk(uint8_t enabled){
	mj.enabled = enabled;
}

void set_esw_theta_rad(float theta_rad){
	cp.active.esw_theta_rad = theta_rad;
}

void set_sw_k_Nm_p_rad(float k_Nm_p_rad){
	cp.active.sw_k_Nm_p_rad = k_Nm_p_rad;
}
void set_sw_b_Nm_p_rps(float b_Nm_p_rps){
	cp.active.sw_b_Nm_p_rps = b_Nm_p_rps;
}
void set_nominal_theta_rad(float theta_rad){
	cp.nominal.theta_rad = theta_rad;
}
void set_nominal_k_Nm_p_rad(float k_Nm_p_rad){
	cp.nominal.k_Nm_p_rad = k_Nm_p_rad;
}
void set_nominal_b_Nm_p_rps(float b_Nm_p_rps){
	cp.nominal.b_Nm_p_rps = b_Nm_p_rps;
}
void set_hard_stop_theta_rad(float hard_stop_theta_rad, int terrain){
	cp.adaptive.hard_stop_theta_rad[terrain] = hard_stop_theta_rad;
}
void set_hard_stop_k_Nm_p_rad(float hard_stop_k_Nm_p_rad, int terrain){
	cp.adaptive.hard_stop_k_Nm_p_rad[terrain] = hard_stop_k_Nm_p_rad;
}
void set_lsw_theta_rad(float lsw_theta_rad, int terrain){
	cp.adaptive.lsw_theta_rad[terrain] = lsw_theta_rad;
}
void set_est_k_Nm_p_rad(float est_k_Nm_p_rad, int terrain){
	cp.adaptive.est_k_Nm_p_rad[terrain] = est_k_Nm_p_rad;
}
void set_est_b_Nm_p_rps(float est_b_Nm_p_rps, int terrain){
	cp.adaptive.est_b_Nm_p_rps[terrain] = est_b_Nm_p_rps;
}
void set_lst_k_Nm_p_rad(float lst_k_Nm_p_rad, int terrain){
	cp.adaptive.lst_k_Nm_p_rad[terrain] = lst_k_Nm_p_rad;
}
void set_lst_b_Nm_p_rps(float lst_b_Nm_p_rps, int terrain){
	cp.adaptive.lst_b_Nm_p_rps[terrain] = lst_b_Nm_p_rps;
}
void set_lst_theta_rad(float lst_theta_rad, int terrain){
	cp.adaptive.lst_theta_rad[terrain] = lst_theta_rad;
}
void set_est_lst_min_theta_rad(float est_lst_min_theta_rad, int terrain){
	cp.adaptive.est_lst_min_theta_rad[terrain] = est_lst_min_theta_rad;
}


void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int terrain_mode){
static int sample_counter = 0;
static int on_entry = 0;
static int prev_state_machine_demux_state = STATE_ESW;
static float stance_entry_theta_rad = 0.0;

if (terrain_mode == MODE_NOMINAL){
	set_joint_torque(actx, tm, cp.nominal.theta_rad, cp.nominal.k_Nm_p_rad, cp.nominal.b_Nm_p_rps);
	return;
}else if (terrain_mode == MODE_POSITION){
	if (mj.enabled){
		if (mj.update_counter < mj.total_trajectory_updates){
				set_next_theta_for_minimum_jerk(actx);
				set_joint_torque(actx, tm, mj.des_theta, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
		}else{
				set_minimum_jerk_trajectory_params(actx, cp.active.esw_theta_rad, 0.0, 0.0, 0.0);
		}
	}else{
		set_joint_torque(actx, tm, cp.active.esw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
	}
	return;
}

set_control_params_for_terrain(terrain_mode);
switch (state_machine_demux_state){
	
	case STATE_ESW:
		if (on_entry)
			sample_counter = 0;
		// if (mj.enabled){
		// 	if (mj.update_counter < mj.total_trajectory_updates){
		// 			set_next_theta_for_minimum_jerk(actx);
		// 			set_joint_torque(actx, tm, mj.des_theta, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
		// 	}else if (!mj.trajectory_defined){
		// 			mj.trajectory_defined = 1;
		// 			set_minimum_jerk_trajectory_params(actx, cp.active.esw_theta_rad, 0.0, 0.0, PREDICTION_CUTOFF_SAMPLES/1000.0);
		// 	}
		// }else{
			set_joint_torque(actx, tm, cp.active.esw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
		// }

        if (tm->gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE){
        	state_machine_demux_state = STATE_LSW;
        }
        
    break;
    case STATE_LSW:
  //   	if (mj.enabled){
		// 	if (mj.update_counter < mj.total_trajectory_updates){
		// 			set_next_theta_for_minimum_jerk(actx);
		// 			set_joint_torque(actx, tm, mj.des_theta, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
		// 	}else{
		// 			set_minimum_jerk_trajectory_params(actx, cp.active.lsw_theta_rad, 0.0, 0.0, 0.001);
		// 	}
		// }else{
			set_joint_torque(actx, tm, cp.active.lsw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
		// }

    	//Transition condition should be, you have a certain velocity of the ankle joint opposing the direction of the torque??
    	//TODO: make terrain specific transition condition here
        if (!tm->in_swing){
        	state_machine_demux_state = STATE_EST;
        	stance_entry_theta_rad = actx->jointAngle;
        }

    break;
    case STATE_EST:
        set_joint_torque_with_hardstop(actx, tm, stance_entry_theta_rad, cp.active.est_k_Nm_p_rad, cp.active.est_b_Nm_p_rps, cp.active.hard_stop_theta_rad, cp.active.hard_stop_k_Nm_p_rad);

       if (actx->jointAngle < cp.active.hard_stop_theta_rad - cp.active.est_lst_min_theta_rad){
       	state_machine_demux_state = STATE_LST;
       }

    break;
    case STATE_LST:
//    	if (on_entry){
//
//    	}
  //   	if (mj.enabled){
		// 	if (mj.update_counter < mj.total_trajectory_updates){
		// 			set_next_theta_for_minimum_jerk(actx);
		// 			set_joint_torque(actx, tm, mj.des_theta, cp.active.lst_k_Nm_p_rad, cp.active.lst_b_Nm_p_rps);
		// 	}else{
		// 			set_minimum_jerk_trajectory_params(actx, cp.active.lst_theta_rad, 0.0, 0.0);
		// 	}
		// }else{
    	set_joint_torque_with_hardstop(actx, tm, cp.active.lst_theta_rad, cp.active.lst_k_Nm_p_rad, cp.active.lst_b_Nm_p_rps, cp.active.hard_stop_theta_rad, cp.active.hard_stop_k_Nm_p_rad);
		// }

   	 	if (tm->in_swing){
        	state_machine_demux_state = STATE_ESW;
        }

    break;
}

if (state_machine_demux_state != prev_state_machine_demux_state)
	on_entry = 1;
else
	on_entry = 0;

prev_state_machine_demux_state = state_machine_demux_state;
sample_counter++;

}
