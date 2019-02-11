

#include <terrain_state_machine.h>




static int state_machine_demux_state = STATE_EARLY_SWING;


static struct control_params_s cp;

static void init_nominal_control_params_s()
{
	cp.nominal.theta_rad = DEFAULT_NOMINAL_THETA_RAD;
	cp.nominal.k_Nm_p_rad = DEFAULT_NOMINAL_K_NM_P_RAD;
	cp.nominal.b_Nm_p_rps = DEFAULT_NOMINAL_B_NM_P_RPS;

}

static void init_terrain_dependent_control_params_s()
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


}

static void set_joint_torque(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b) {
	actx->tauDes = k * (des_theta - actx->jointAngle) - b * actx->jointVel ;
	setMotorTorque(actx, actx->tauDes);
}

static void set_joint_torque_with_hardstop(Act_s* actx, struct taskmachine_s* tm, float des_theta, float k, float b, float hs_theta, float hs_k) {
	actx->tauDes = k * (des_theta - actx->jointAngle) - b * actx->jointVel ;
	if (actx->jointAngle < hs_theta){
		actx->tauDes = actx->tauDes - hs_k*(actx->jointAngle - hs_theta);
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

void init_terrain_state_machine(){
	init_nominal_control_params_s();
	init_terrain_dependent_control_params_s();
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


void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int current_terrain){
static int sample_counter = 0;
static int on_entry = 0;
static int prev_state_machine_demux_state = STATE_ESW;
static float stance_entry_theta_rad = 0.0;

if (current_terrain == K_NOMINAL){
	set_joint_torque(actx, tm, cp.nominal.theta_rad, cp.nominal.k_Nm_p_rad, cp.nominal.b_Nm_p_rps);
	return;
}
set_control_params_for_terrain(current_terrain);

switch (state_machine_demux_state){
	case STATE_ESW:
		if (on_entry)
			sample_counter = 0;
		// set_joint_torque(actx, tm, POSITION_CONTROL_GAIN_K_NM_P_RAD, POSITION_CONTROL_GAIN_B_NM_P_RPS, cp.active.esw_theta_rad);
		set_joint_torque(actx, tm, cp.active.esw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);
        if (tm->gait_event_trigger = GAIT_EVENT_WINDOW_CLOSE)
        	state_machine_demux_state = STATE_LSW;
        
    break;
    case STATE_LSW:
    	set_joint_torque(actx, tm, cp.active.lsw_theta_rad, cp.active.sw_k_Nm_p_rad, cp.active.sw_b_Nm_p_rps);

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

    	set_joint_torque_with_hardstop(actx, tm, cp.active.lst_theta_rad, cp.active.lst_k_Nm_p_rad, cp.active.lst_b_Nm_p_rps, cp.active.hard_stop_theta_rad, cp.active.hard_stop_k_Nm_p_rad);
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
