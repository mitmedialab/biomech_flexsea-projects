

#include <terrain_state_machine.h>

#define POSITION_CONTROL_GAIN_K 1.5 
#define POSITION_CONTROL_GAIN_B 0.3

#define DEFAULT_NOMINAL_K_NM_P_RAD 1.5
#define DEFAULT_NOMINAL_B_NM_P_RPS 0.3
#define DEFAULT_NOMINAL_THETA_RAD 0.0

#define DEFAULT_FLAT_HS_THETA_RAD 0.0
#define DEFAULT_FLAT_HS_K_NM_P_RAD 0.0
#define DEFAULT_FLAT_LSW_THETA_RAD 0.0
#define DEFAULT_FLAT_EST_K_NM_P_RAD 0.0
#define DEFAULT_FLAT_EST_B_NM_P_RPS 0.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 0.0
#define DEFAULT_FLAT_LST_B_NM_P_RPS 0.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.0
#define DEFAULT_FLAT_EST_LST_MIN_THETA_RAD 0.0

#define DEFAULT_URAMP_HS_THETA_RAD 0.0
#define DEFAULT_URAMP_HS_K_NM_P_RAD 0.0
#define DEFAULT_URAMP_LSW_THETA_RAD 0.0
#define DEFAULT_URAMP_EST_K_NM_P_RAD 0.0
#define DEFAULT_URAMP_EST_B_NM_P_RPS 0.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 0.0
#define DEFAULT_URAMP_LST_B_NM_P_RPS 0.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.0
#define DEFAULT_URAMP_EST_LST_MIN_THETA_RAD 0.0

#define DEFAULT_DRAMP_HS_THETA_RAD 0.0
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 0.0
#define DEFAULT_DRAMP_LSW_THETA_RAD 0.0
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 0.0
#define DEFAULT_DRAMP_EST_B_NM_P_RPS 0.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 0.0
#define DEFAULT_DRAMP_LST_B_NM_P_RPS 0.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.0
#define DEFAULT_DRAMP_EST_LST_MIN_THETA_RAD 0.0

#define DEFAULT_USTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 0.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD 0.0
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 0.0
#define DEFAULT_USTAIRS_EST_B_NM_P_RPS 0.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_USTAIRS_LST_B_NM_P_RPS 0.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_USTAIRS_EST_LST_MIN_THETA_RAD 0.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_EST_B_NM_P_RPS 0.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_B_NM_P_RPS 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_EST_LST_MIN_THETA_RAD 0.0

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

	cp.terrdep.hard_stop_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.hard_stop_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.terrdep.lsw_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.est_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.est_b_Nm_p_rps = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.lst_k_Nm_p_rad = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.lst_b_Nm_p_rps = (float*)calloc(N_CLASSES, sizeof(float));
	cp.terrdep.lst_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));

	cp.terrdep.est_lst_min_theta_rad = (float*)calloc(N_CLASSES, sizeof(float));


	cp.terrdep.hard_stop_theta_rad[K_FLAT] = DEFAULT_FLAT_HS_THETA_RAD;
	cp.terrdep.hard_stop_theta_rad[K_URAMP] = DEFAULT_URAMP_HS_THETA_RAD;
	cp.terrdep.hard_stop_theta_rad[K_DRAMP] = DEFAULT_DRAMP_HS_THETA_RAD;
	cp.terrdep.hard_stop_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_HS_THETA_RAD;
	cp.terrdep.hard_stop_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_HS_THETA_RAD;

	cp.terrdep.hard_stop_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_HS_K_NM_P_RAD;
	cp.terrdep.hard_stop_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_HS_K_NM_P_RAD;
	cp.terrdep.hard_stop_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_HS_K_NM_P_RAD;
	cp.terrdep.hard_stop_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_HS_K_NM_P_RAD;
	cp.terrdep.hard_stop_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_HS_K_NM_P_RAD;

	cp.terrdep.lsw_theta_rad[K_FLAT] = DEFAULT_FLAT_LSW_THETA_RAD;
	cp.terrdep.lsw_theta_rad[K_URAMP] = DEFAULT_URAMP_LSW_THETA_RAD;
	cp.terrdep.lsw_theta_rad[K_DRAMP] = DEFAULT_DRAMP_LSW_THETA_RAD;
	cp.terrdep.lsw_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_LSW_THETA_RAD;
	cp.terrdep.lsw_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LSW_THETA_RAD;

	cp.terrdep.est_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_EST_K_NM_P_RAD;
	cp.terrdep.est_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_EST_K_NM_P_RAD;
	cp.terrdep.est_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_EST_K_NM_P_RAD;
	cp.terrdep.est_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_EST_K_NM_P_RAD;
	cp.terrdep.est_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_K_NM_P_RAD;

	cp.terrdep.est_b_Nm_p_rps[K_FLAT] = DEFAULT_FLAT_EST_B_NM_P_RPS;
	cp.terrdep.est_b_Nm_p_rps[K_URAMP] = DEFAULT_URAMP_EST_B_NM_P_RPS;
	cp.terrdep.est_b_Nm_p_rps[K_DRAMP] = DEFAULT_DRAMP_EST_B_NM_P_RPS;
	cp.terrdep.est_b_Nm_p_rps[K_USTAIRS] = DEFAULT_USTAIRS_EST_B_NM_P_RPS;
	cp.terrdep.est_b_Nm_p_rps[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_B_NM_P_RPS;

	cp.terrdep.lst_k_Nm_p_rad[K_FLAT] = DEFAULT_FLAT_LST_K_NM_P_RAD;
	cp.terrdep.lst_k_Nm_p_rad[K_URAMP] = DEFAULT_URAMP_LST_K_NM_P_RAD;
	cp.terrdep.lst_k_Nm_p_rad[K_DRAMP] = DEFAULT_DRAMP_LST_K_NM_P_RAD;
	cp.terrdep.lst_k_Nm_p_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_K_NM_P_RAD;
	cp.terrdep.lst_k_Nm_p_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_K_NM_P_RAD;

	cp.terrdep.lst_b_Nm_p_rps[K_FLAT] = DEFAULT_FLAT_LST_B_NM_P_RPS;
	cp.terrdep.lst_b_Nm_p_rps[K_URAMP] = DEFAULT_URAMP_LST_B_NM_P_RPS;
	cp.terrdep.lst_b_Nm_p_rps[K_DRAMP] = DEFAULT_DRAMP_LST_B_NM_P_RPS;
	cp.terrdep.lst_b_Nm_p_rps[K_USTAIRS] = DEFAULT_USTAIRS_LST_B_NM_P_RPS;
	cp.terrdep.lst_b_Nm_p_rps[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_B_NM_P_RPS;

	cp.terrdep.lst_theta_rad[K_FLAT] = DEFAULT_FLAT_LST_THETA_RAD;
	cp.terrdep.lst_theta_rad[K_URAMP] = DEFAULT_URAMP_LST_THETA_RAD;
	cp.terrdep.lst_theta_rad[K_DRAMP] = DEFAULT_DRAMP_LST_THETA_RAD;
	cp.terrdep.lst_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_LST_THETA_RAD;
	cp.terrdep.lst_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_LST_THETA_RAD;

	cp.terrdep.est_lst_min_theta_rad[K_FLAT] = DEFAULT_FLAT_EST_LST_MIN_THETA_RAD;
	cp.terrdep.est_lst_min_theta_rad[K_URAMP] = DEFAULT_URAMP_EST_LST_MIN_THETA_RAD;
	cp.terrdep.est_lst_min_theta_rad[K_DRAMP] = DEFAULT_DRAMP_EST_LST_MIN_THETA_RAD;
	cp.terrdep.est_lst_min_theta_rad[K_USTAIRS] = DEFAULT_USTAIRS_EST_LST_MIN_THETA_RAD;
	cp.terrdep.est_lst_min_theta_rad[K_DSTAIRS] = DEFAULT_DSTAIRS_EST_LST_MIN_THETA_RAD;

}

static void set_joint_torque(Act_s *actx, float theta, float k, float b) {

	actx->tauDes = k * (theta - actx->jointAngle) - b * actx->jointVel ;
}

static void set_control_params_for_terrain(int terrain){
	cp.active.hard_stop_theta_rad = cp.terrdep.hard_stop_theta_rad[terrain];
	cp.active.hard_stop_k_Nm_p_rad = cp.terrdep.hard_stop_k_Nm_p_rad[terrain];

	cp.active.lsw_theta_rad = cp.terrdep.lsw_theta_rad[terrain];
	cp.active.est_k_Nm_p_rad = cp.terrdep.est_k_Nm_p_rad[terrain];
	cp.active.est_b_Nm_p_rps = cp.terrdep.est_b_Nm_p_rps[terrain];
	cp.active.lst_k_Nm_p_rad = cp.terrdep.lst_k_Nm_p_rad[terrain];
	cp.active.lst_b_Nm_p_rps = cp.terrdep.lst_b_Nm_p_rps[terrain];
	cp.active.lst_theta_rad = cp.terrdep.lst_theta_rad[terrain];

	cp.active.est_lst_min_theta_rad = cp.terrdep.est_lst_min_theta_rad[terrain];
}

int get_walking_state(){
	return state_machine_demux_state;
}
struct control_params_s* get_control_params(){
	return &cp;
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
	cp.terrdep.hard_stop_theta_rad[terrain] = hard_stop_theta_rad;
}
void set_hard_stop_k_Nm_p_rad(float hard_stop_k_Nm_p_rad, int terrain){
	cp.terrdep.hard_stop_k_Nm_p_rad[terrain] = hard_stop_k_Nm_p_rad;
}
void set_lsw_theta_rad(float lsw_theta_rad, int terrain){
	cp.terrdep.lsw_theta_rad[terrain] = lsw_theta_rad;
}
void set_est_k_Nm_p_rad(float est_k_Nm_p_rad, int terrain){
	cp.terrdep.est_k_Nm_p_rad[terrain] = est_k_Nm_p_rad;
}
void set_est_b_Nm_p_rps(float est_b_Nm_p_rps, int terrain){
	cp.terrdep.est_b_Nm_p_rps[terrain] = est_b_Nm_p_rps;
}
void set_lst_k_Nm_p_rad(float lst_k_Nm_p_rad, int terrain){
	cp.terrdep.lst_k_Nm_p_rad[terrain] = lst_k_Nm_p_rad;
}
void set_lst_b_Nm_p_rps(float lst_b_Nm_p_rps, int terrain){
	cp.terrdep.lst_b_Nm_p_rps[terrain] = lst_b_Nm_p_rps;
}
void set_lst_theta_rad(float lst_theta_rad, int terrain){
	cp.terrdep.lst_theta_rad[terrain] = lst_theta_rad;
}
void set_est_lst_min_theta_rad(float est_lst_min_theta_rad, int terrain){
	cp.terrdep.est_lst_min_theta_rad[terrain] = est_lst_min_theta_rad;
}









void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int current_terrain){
static int sample_counter = 0;
static int on_entry = 0;
static int prev_state_machine_demux_state = STATE_ESW;
static float stance_entry_theta_rad = 0.0;

if (current_terrain == K_NOMINAL){
	set_joint_torque(actx, cp.nominal.theta_rad, cp.nominal.k_Nm_p_rad, cp.nominal.b_Nm_p_rps);
	return;
}

switch (state_machine_demux_state){
	case STATE_ESW:
		if (on_entry)
			sample_counter = 0;
		set_joint_torque(actx, POSITION_CONTROL_GAIN_K, POSITION_CONTROL_GAIN_B, cp.active.esw_theta_rad);
        if (tm->stride_classified)
        	state_machine_demux_state = STATE_LSW;
        
    break;
    case STATE_LSW:
    	if (on_entry)
    		set_control_params_for_terrain(current_terrain);
    	set_joint_torque(actx, POSITION_CONTROL_GAIN_K, POSITION_CONTROL_GAIN_B, cp.active.lsw_theta_rad);

        if (!tm->in_swing){
        	state_machine_demux_state = STATE_EST;
        	stance_entry_theta_rad = actx->jointAngle;
        }

    break;
    case STATE_EST:
        
        set_joint_torque(actx, cp.active.est_k_Nm_p_rad, cp.active.est_b_Nm_p_rps, stance_entry_theta_rad);

        if (actx->jointAngle < cp.active.est_lst_min_theta_rad){
        	state_machine_demux_state = STATE_LST;
        }

    break;
    case STATE_LST:

    	set_joint_torque(actx, cp.active.lst_k_Nm_p_rad, cp.active.lst_b_Nm_p_rps, cp.active.lst_theta_rad);
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
