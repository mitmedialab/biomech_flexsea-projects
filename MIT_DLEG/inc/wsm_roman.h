

#ifndef __WSM_ROMAN_H__
#define __WSM_ROMAN_H__

#include "imu.h"
#include <math.h>
#include "flexsea_user_structs.h"
#include "task_machine.h"

int get_walking_state();
void state_machine_demux(struct rigid_s* rigid, int k_est);


struct walking_params_s
{
	float foot_off_theta_rad;
	float foot_strike_theta_rad;
	float early_stance_b_Nm_p_rps;
	float early_stance_k_N_p_rad;
	float late_stance_b_Nm_p_rps;
	float late_stance_k_N_p_rad;
	float hardstop_theta_rad;
};

enum Walking_States {
    STATE_EARLY_SWING = 2,               /// 2
    STATE_LATE_SWING = 3,                /// 3
    STATE_EARLY_STANCE = 4,              /// 4
    STATE_LATE_STANCE = 5,               /// 5  
};

#endif
