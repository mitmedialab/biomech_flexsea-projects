

#include <terrain_state_machine.h>




static int state_machine_demux_state = STATE_EARLY_SWING;

static struct control_params_s cp;

static void init_adaptive_control_params_s()
{

	cp.adaptive.hard_stop_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.hard_stop_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.adaptive.lsw_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.st_b_Nm_p_rps = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.est_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.lst_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.adaptive.lst_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.adaptive.lst_engagement_tq_Nm = (float*)calloc(N_CLASSES,sizeof(float));
	cp.adaptive.lst_delay_tics = (float*)calloc(N_CLASSES,sizeof(int));


}

static void set_joint_torque(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b, float scale_factor) {
	actx->thetaDes = des_theta;
	actx->tauDes = scale_factor * (k * (des_theta - tm->aa) - b * tm->aa_dot_15hz_filt );
	setMotorTorque(actx, actx->tauDes);
}

static void set_joint_torque_with_hardstop(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b, float hs_theta, float hs_k, float scale_factor) {
	actx->thetaDes = des_theta;
	actx->tauDes = scale_factor * (k * (des_theta - tm->aa) - b * tm->aa_dot_15hz_filt  );
	if (tm->aa < hs_theta){
		actx->tauDes = actx->tauDes - hs_k*(tm->aa - hs_theta);
	}
	setMotorTorque(actx, actx->tauDes);
}



static void set_control_params_for_terrain(int terrain){

	cp.active.hard_stop_theta_rad = cp.adaptive.hard_stop_theta_rad[terrain];
	cp.active.hard_stop_k_Nm_p_rad = cp.adaptive.hard_stop_k_Nm_p_rad[terrain];

	cp.active.lsw_theta_rad = cp.adaptive.lsw_theta_rad[terrain];
	cp.active.est_k_Nm_p_rad = cp.adaptive.est_k_Nm_p_rad[terrain];
	cp.active.st_b_Nm_p_rps = cp.adaptive.st_b_Nm_p_rps[terrain];
	cp.active.lst_k_Nm_p_rad = cp.adaptive.lst_k_Nm_p_rad[terrain];
	cp.active.lst_theta_rad = cp.adaptive.lst_theta_rad[terrain];
	cp.active.lst_engagement_tq_Nm = cp.adaptive.lst_engagement_tq_Nm[terrain];
	cp.active.lst_delay_tics = cp.adaptive.lst_delay_tics[terrain];
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

	cp.adaptive.st_b_Nm_p_rps[K_FLAT] = DEFAULT_FLAT_ST_B_NM_P_RPS;
	cp.adaptive.st_b_Nm_p_rps[K_URAMP] = DEFAULT_URAMP_ST_B_NM_P_RPS;
	cp.adaptive.st_b_Nm_p_rps[K_DRAMP] = DEFAULT_DRAMP_ST_B_NM_P_RPS;
	cp.adaptive.st_b_Nm_p_rps[K_USTAIRS] = DEFAULT_USTAIRS_ST_B_NM_P_RPS;
	cp.adaptive.st_b_Nm_p_rps[K_DSTAIRS] = DEFAULT_DSTAIRS_ST_B_NM_P_RPS;

	cp.adaptive.lst_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_K_NM_P_RAD;
	cp.adaptive.lst_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_K_NM_P_RAD;

	cp.adaptive.lst_theta_rad[K_FLAT] = DEFAULT_FLAT_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_URAMP] = DEFAULT_URAMP_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_DRAMP] = DEFAULT_DRAMP_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_THETA_RAD;
	cp.adaptive.lst_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_THETA_RAD;

	cp.adaptive.lst_engagement_tq_Nm[K_FLAT] = DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM;
	cp.adaptive.lst_engagement_tq_Nm[K_URAMP] = DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM;
	cp.adaptive.lst_engagement_tq_Nm[K_DRAMP] = DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM;
	cp.adaptive.lst_engagement_tq_Nm[K_USTAIRS] = DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM;
	cp.adaptive.lst_engagement_tq_Nm[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM;

	cp.adaptive.lst_delay_tics[K_FLAT] = DEFAULT_FLAT_LST_DELAY_TICS;
	cp.adaptive.lst_delay_tics[K_URAMP] = DEFAULT_URAMP_LST_DELAY_TICS;
	cp.adaptive.lst_delay_tics[K_DRAMP] = DEFAULT_DRAMP_LST_DELAY_TICS;
	cp.adaptive.lst_delay_tics[K_USTAIRS] = DEFAULT_USTAIRS_LST_DELAY_TICS;
	cp.adaptive.lst_delay_tics[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_DELAY_TICS;

	cp.active.esw_theta_rad = DEFAULT_ESW_THETA_RAD;
	cp.active.sw_k_Nm_p_rad = DEFAULT_SW_K_NM_P_RAD;
	cp.active.sw_b_Nm_p_rps = DEFAULT_SW_B_NM_P_RPS;
	cp.active.sw_delay_tics = DEFAULT_SW_DELAY_TICS;
	cp.active.desired_trajectory_period_s = DEFAULT_DESIRED_TRAJECTORY_PERIOD_S;

	cp.nominal.theta_rad = DEFAULT_NOMINAL_THETA_RAD;
	cp.nominal.k_Nm_p_rad = DEFAULT_NOMINAL_K_NM_P_RAD;
	cp.nominal.b_Nm_p_rps = DEFAULT_NOMINAL_B_NM_P_RPS;
}



void init_terrain_state_machine(){
	init_adaptive_control_params_s();
	reset_terrain_state_machine_parameters();
}

int get_walking_state(){
	return state_machine_demux_state;
}

struct control_params_s* get_control_params(){
	return &cp;
}

void set_esw_theta_rad(float theta_rad){
	cp.active.esw_theta_rad = theta_rad;
}

void set_sw_k_Nm_p_rad(float k_Nm_p_rad){
	cp.active.sw_k_Nm_p_rad = k_Nm_p_rad;
}
void set_sw_b_Nm_p_rps(float b_Nm_p_rps){
	if (b_Nm_p_rps > DEFAULT_MAX_SAFE_B_NM_P_RPS)
		cp.active.sw_b_Nm_p_rps = DEFAULT_MAX_SAFE_B_NM_P_RPS;
	else
		cp.active.sw_b_Nm_p_rps = b_Nm_p_rps;
}

void set_sw_delay_tics(float sw_delay_tics){
	cp.active.sw_delay_tics = sw_delay_tics;
}

void set_nominal_theta_rad(float theta_rad){
	cp.nominal.theta_rad = theta_rad;
}
void set_nominal_k_Nm_p_rad(float k_Nm_p_rad){
	cp.nominal.k_Nm_p_rad = k_Nm_p_rad;
}
void set_nominal_b_Nm_p_rps(float b_Nm_p_rps){
	if (b_Nm_p_rps > DEFAULT_MAX_SAFE_B_NM_P_RPS)
		cp.nominal.b_Nm_p_rps = DEFAULT_MAX_SAFE_B_NM_P_RPS;
	else
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
void set_st_b_Nm_p_rps(float st_b_Nm_p_rps, int terrain){
	if (st_b_Nm_p_rps > DEFAULT_MAX_SAFE_B_NM_P_RPS)
		cp.adaptive.st_b_Nm_p_rps[terrain] = DEFAULT_MAX_SAFE_B_NM_P_RPS;
	else
		cp.adaptive.st_b_Nm_p_rps[terrain] = st_b_Nm_p_rps;
}

void set_lst_k_Nm_p_rad(float lst_k_Nm_p_rad, int terrain){
	cp.adaptive.lst_k_Nm_p_rad[terrain] = lst_k_Nm_p_rad;
}

void set_lst_theta_rad(float lst_theta_rad, int terrain){
	cp.adaptive.lst_theta_rad[terrain] = lst_theta_rad;
}

void set_lst_engagement_tq_Nm(float lst_engagement_tq_Nm, int terrain){
	cp.adaptive.lst_engagement_tq_Nm[terrain] = lst_engagement_tq_Nm;
}

void set_lst_delay_tics(float lst_delay_tics, int terrain){
	cp.adaptive.lst_delay_tics[terrain] = lst_delay_tics;
}




void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int terrain_mode){
static int sample_counter = 0;
static float delay_tics = 0.0;
static int on_entry = 0;
static int prev_state_machine_demux_state = STATE_ESW;
static float stance_entry_theta_rad = 0.0;
static float prev_esw_theta_rad = 0.0;
static float prev_sw_delay_tics = 0.0;

if (terrain_mode == MODE_NOMINAL){
	set_joint_torque(actx, tm, cp.nominal.theta_rad, cp.nominal.k_Nm_p_rad, cp.nominal.b_Nm_p_rps,1.0);
	if (tm->in_swing){
		state_machine_demux_state = STATE_LSW;
	}else{
		state_machine_demux_state = STATE_LST;
	}
	return;
}else if (terrain_mode == MODE_POSITION){
	if (delay_tics < cp.active.sw_delay_tics){
		delay_tics++;
	}
	set_joint_torque(actx, tm, cp.active.esw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps, delay_tics/cp.active.sw_delay_tics);

	if (cp.active.esw_theta_rad != prev_esw_theta_rad ||
			cp.active.sw_delay_tics != prev_sw_delay_tics){
		delay_tics = 0;
	}
	prev_esw_theta_rad = cp.active.esw_theta_rad;
	prev_sw_delay_tics = cp.active.sw_delay_tics;

	return;
}

set_control_params_for_terrain(terrain_mode);
switch (state_machine_demux_state){
	
	case STATE_ESW:
		if (on_entry){
			sample_counter = 0;
			delay_tics = 0;
		}

		if (delay_tics < cp.active.sw_delay_tics){
			delay_tics++;
		}
		set_joint_torque(actx, tm, cp.active.esw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps,delay_tics/cp.active.sw_delay_tics);

        if (tm->gait_event_trigger == GAIT_EVENT_WINDOW_CLOSE){
        	state_machine_demux_state = STATE_LSW;
        }
        
    break;
    case STATE_LSW:
    	if (on_entry){
    		delay_tics = 0;
    	}

    	if (delay_tics < cp.active.sw_delay_tics){
    		delay_tics++;
    	}
    	set_joint_torque(actx, tm, cp.active.lsw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps,delay_tics/cp.active.sw_delay_tics);

    	//Transition condition should be, you have a certain velocity of the ankle joint opposing the direction of the torque??
    	//TODO: make terrain specific transition condition here
        if (!tm->in_swing){
        	state_machine_demux_state = STATE_EST;
        	stance_entry_theta_rad = actx->jointAngle;
        }

    break;
    case STATE_EST:
    	if (on_entry){
    	}

        set_joint_torque_with_hardstop(actx, tm, stance_entry_theta_rad, cp.active.est_k_Nm_p_rad, cp.active.st_b_Nm_p_rps, cp.active.hard_stop_theta_rad, cp.active.hard_stop_k_Nm_p_rad,1.0);

       	if (actx->jointTorque > cp.active.lst_engagement_tq_Nm && actx->jointAngle < cp.active.hard_stop_theta_rad){
       		state_machine_demux_state = STATE_LST;
       	}


       	if (tm->in_swing){
       		state_machine_demux_state = STATE_ESW;
       	}


    break;
    case STATE_LST:
    	if (on_entry) {
    		delay_tics = 0;
    	}

		// This is the scaling factor for ramping into powered pushoff
		if (delay_tics < cp.active.lst_delay_tics){
			delay_tics++;
		}

    	set_joint_torque_with_hardstop(actx, tm, cp.active.lst_theta_rad, cp.active.lst_k_Nm_p_rad, cp.active.st_b_Nm_p_rps, cp.active.hard_stop_theta_rad, cp.active.hard_stop_k_Nm_p_rad, delay_tics/cp.active.lst_delay_tics);

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
