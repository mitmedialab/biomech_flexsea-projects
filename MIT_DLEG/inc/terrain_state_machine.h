

#ifndef __WSM_ROMAN_H__
#define __WSM_ROMAN_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"
#include "actuator_functions.h"


//#define DEFAULT_SW_K_NM_P_RAD 1000.0
#define DEFAULT_SW_K_NM_P_RAD 2.5/RAD_PER_DEG
#define DEFAULT_SW_B_NM_P_RPS 0.4/RAD_PER_DEG
#define DEFAULT_LST_DELAY_TICS 70.0

#define DEFAULT_NOMINAL_K_NM_P_RAD 150.0
#define DEFAULT_NOMINAL_B_NM_P_RPS 30.0
#define DEFAULT_NOMINAL_THETA_RAD 0.0
#define DEFAULT_DESIRED_TRAJECTORY_PERIOD_S 0.250

#define DEFAULT_ESW_THETA_RAD 0.0

#define DEFAULT_FLAT_HS_THETA_RAD 0.04
#define DEFAULT_FLAT_HS_K_NM_P_RAD 200.0
#define DEFAULT_FLAT_LSW_THETA_RAD -0.03
#define DEFAULT_FLAT_EST_K_NM_P_RAD 86.0
#define DEFAULT_FLAT_EST_B_NM_P_RPS 17.0
#define DEFAULT_FLAT_LST_K_NM_P_RAD 200.0
#define DEFAULT_FLAT_LST_B_NM_P_RPS 11.0
#define DEFAULT_FLAT_LST_THETA_RAD 0.26
#define DEFAULT_FLAT_EST_LST_MIN_THETA_RAD 0.26
#define DEFAULT_FLAT_LST_ENGAGEMENT_TQ_NM 20.0

#define DEFAULT_URAMP_HS_THETA_RAD -0.19
#define DEFAULT_URAMP_HS_K_NM_P_RAD 500.0
#define DEFAULT_URAMP_LSW_THETA_RAD -0.14
#define DEFAULT_URAMP_EST_K_NM_P_RAD 10.0
#define DEFAULT_URAMP_EST_B_NM_P_RPS 0.0
#define DEFAULT_URAMP_LST_K_NM_P_RAD 100.0
#define DEFAULT_URAMP_LST_B_NM_P_RPS 0.0
#define DEFAULT_URAMP_LST_THETA_RAD 0.3
#define DEFAULT_URAMP_LST_ENGAGEMENT_TQ_NM 20.0

#define DEFAULT_DRAMP_HS_THETA_RAD -0.08
#define DEFAULT_DRAMP_HS_K_NM_P_RAD 500.0
#define DEFAULT_DRAMP_LSW_THETA_RAD -0.03
#define DEFAULT_DRAMP_EST_K_NM_P_RAD 10.0
#define DEFAULT_DRAMP_EST_B_NM_P_RPS 0.0
#define DEFAULT_DRAMP_LST_K_NM_P_RAD 10.0
#define DEFAULT_DRAMP_LST_B_NM_P_RPS 0.0
#define DEFAULT_DRAMP_LST_THETA_RAD 0.0
#define DEFAULT_DRAMP_LST_ENGAGEMENT_TQ_NM 20.0

#define DEFAULT_USTAIRS_HS_THETA_RAD -0.33
#define DEFAULT_USTAIRS_HS_K_NM_P_RAD 500.0
#define DEFAULT_USTAIRS_LSW_THETA_RAD -0.28
#define DEFAULT_USTAIRS_EST_K_NM_P_RAD 10.0
#define DEFAULT_USTAIRS_EST_B_NM_P_RPS 0.0
#define DEFAULT_USTAIRS_LST_K_NM_P_RAD 100.0
#define DEFAULT_USTAIRS_LST_B_NM_P_RPS 0.0
#define DEFAULT_USTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_USTAIRS_LST_ENGAGEMENT_TQ_NM 20.0

#define DEFAULT_DSTAIRS_HS_THETA_RAD 0.04
#define DEFAULT_DSTAIRS_HS_K_NM_P_RAD 100.0
#define DEFAULT_DSTAIRS_LSW_THETA_RAD 0.51
#define DEFAULT_DSTAIRS_EST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_EST_B_NM_P_RPS 20.0
#define DEFAULT_DSTAIRS_LST_K_NM_P_RAD 0.0
#define DEFAULT_DSTAIRS_LST_B_NM_P_RPS 0.0
#define DEFAULT_DSTAIRS_LST_THETA_RAD 0.0
#define DEFAULT_DSTAIRS_LST_ENGAGEMENT_TQ_NM 30.0

#define DEFAULT_MINIMUM_JERK_TRAJECTORY_TIME 0.2
#define DEFAULT_MINIMUM_JERK_ANGLE_TOL_RAD 0.05
#define MINIMUM_JERK_MAX_MEAN_SPEED_RPS 1.5

int get_walking_state();
struct minimum_jerk_values_s* get_minimum_jerk_values();
struct control_params_s* get_control_params();
void set_minimum_jerk_trajectory_period(float T);
void set_minimum_jerk_angle_tol_rad(float angle_tol_rad);
void enable_minimum_jerk(uint8_t enabled);
void set_esw_theta_rad(float theta_rad);
void set_sw_k_Nm_p_rad(float k_Nm_p_rad);
void set_sw_b_Nm_p_rps(float b_Nm_p_rps);
void set_nominal_theta_rad(float theta_rad);
void set_nominal_k_Nm_p_rad(float k_Nm_p_rad);
void set_nominal_b_Nm_p_rps(float b_Nm_p_rps);
void set_hard_stop_theta_rad(float hard_stop_theta_rad, int terrain);
void set_hard_stop_k_Nm_p_rad(float hard_stop_k_Nm_p_rad, int terrain);
void set_lsw_theta_rad(float lsw_theta_rad, int terrain);
void set_est_k_Nm_p_rad(float est_k_Nm_p_rad, int terrain);
void set_est_b_Nm_p_rps(float est_b_Nm_p_rps, int terrain);
void set_lst_k_Nm_p_rad(float lst_k_Nm_p_rad, int terrain);
void set_lst_b_Nm_p_rps(float lst_b_Nm_p_rps, int terrain);
void set_lst_theta_rad(float lst_theta_rad, int terrain);
void set_est_lst_min_theta_rad(float est_lst_min_theta_rad, int terrain);
void set_lst_engagement_tq_Nm(float lst_engagement_tq_Nm, int terrain);
void terrain_state_machine_demux(struct taskmachine_s* tm, struct rigid_s* rigid, Act_s *actx, int current_terrain);
void init_terrain_state_machine();
void reset_terrain_state_machine_parameters();

static struct nominal_control_params_s
{
	float theta_rad;
	float k_Nm_p_rad;
	float b_Nm_p_rps;
};


static struct active_control_params_s
{
	//Terrain independent params
	float esw_theta_rad;
	float sw_k_Nm_p_rad;
	float sw_b_Nm_p_rps;
	float desired_trajectory_period_s;
	float lst_delay_tics;

	//Terrain dependent params
	float hard_stop_theta_rad;
	float hard_stop_k_Nm_p_rad;
	
	float lsw_theta_rad;
	float est_k_Nm_p_rad;
	float est_b_Nm_p_rps;
	float lst_k_Nm_p_rad;
	float lst_b_Nm_p_rps;
	float lst_theta_rad;

	float lst_engagement_tq_Nm;

};


static struct terrain_dependent_control_params_s
{
	float* hard_stop_theta_rad;
	float* hard_stop_k_Nm_p_rad;
	
	float* lsw_theta_rad;
	float* est_k_Nm_p_rad;
	float* est_b_Nm_p_rps;
	float* lst_k_Nm_p_rad;
	float* lst_b_Nm_p_rps;
	float* lst_theta_rad;

	float* est_lst_min_theta_rad;
	float* lst_engagement_tq_Nm;

};

struct control_params_s{
	 struct terrain_dependent_control_params_s adaptive;
 	struct active_control_params_s active;
	struct nominal_control_params_s nominal;
};

struct minimum_jerk_values_s{
	float* T;
	float* params;
	float theta_target;
	float theta_dot_target;
	float theta_ddot_target;
	float des_theta;
	uint update_counter;
	uint total_trajectory_updates;
	float angle_tol_rad;
	uint8_t trajectory_defined;
	uint8_t enabled;
};


enum Walking_States {
   STATE_ESW = 2,               /// 2
   STATE_LSW = 3,                /// 3
   STATE_EST = 4,              /// 4
   STATE_LST = 5,               /// 5
};

// Fl
// Sw
// - go to alpha_fl
// EST
// - provide some damping and virtual hardstop at theta_fl
// LST
// - provide some damping and stiffness and virtual hardstop at theta_fl > alpha_fl
// LSP
// - stiff spring with far away set point

// Ur
// Sw
// - go to alpha_ur (dorsiflexed)
// EST
// - provide some damping and virtual hardstop at theta_ur > alpha_ur
// LSP
// - stiffer spring wirh far away set point

// Dr
// SW
// - go to alpha_dr = alpha_fl
// Est
// - provide damping
// Lst
// - provide damping and stiffness

// Us
// Sw
// - go to alpha_us
// EST
// - provide some damping or stiffness??
// LSP
// - use mildly stiff spring to try to open the ankle, and have much stiffer virtual hardstop ratcheting angle, such that the ankle can provide support independently of angle

// Ds
// SW
// - go to alpha_ds
// Est
// - provide lots of damping and maybe a small stiffness
// Lst
// - provide stiffness to a zero set point 

#endif
