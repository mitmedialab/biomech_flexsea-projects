

#ifndef __WSM_ROMAN_H__
#define __WSM_ROMAN_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"

int get_walking_state();
void state_machine_demux(struct rigid_s* rigid, int k_est);

#define HARDSTOP_STIFFNESS

struct fl_params_s
{
	float foot_off_theta_rad;
	float foot_strike_theta_rad;
	float early_stance_b_Nm_p_rps;
	float early_stance_k_N_p_rad;
	float late_stance_b_Nm_p_rps;
	float late_stance_k_N_p_rad;
	float late_stance_theta_rad;
	float hardstop_theta_rad;
};

struct ur_params_s
{
	float swing_theta_rad;
	float hardstop_theta_rad;
	float late_stance_b_Nm_p_rps;
	float late_stance_k_N_p_rad;
	float late_stance_theta_rad;

}
struct dr_params_s
{
	float swing_theta_rad;
	float early_stance_b_Nm_p_rps;
	float early_stance_k_N_p_rad;
	float late_stance_b_Nm_p_rps;
	float late_stance_k_N_p_rad;
	float late_stance_theta_rad;

}
struct us_params_s
{
	float swing_theta_rad;
	float early_stance_b_Nm_p_rps;
	float early_stance_k_N_p_rad;
	float late_stance_b_Nm_p_rps;
	float late_stance_k_N_p_rad;
	float late_stance_theta_rad;

}
struct ds_params_s
{
	float swing_theta_rad;
	float early_stance_b_Nm_p_rps;
	float early_stance_k_N_p_rad;
	float late_stance_b_Nm_p_rps;	
	float late_stance_k_N_p_rad;
	float late_stance_theta_rad;

}

struct walking_params_s
{
	struct fl_params_s fl_params;
	struct ur_params_s ur_params;
	struct dr_params_s ds_params;
	struct us_params_s us_params;
	struct ds_params_s ds_params;

}

enum Walking_States {
   STATE_SW = 2,               /// 2
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
