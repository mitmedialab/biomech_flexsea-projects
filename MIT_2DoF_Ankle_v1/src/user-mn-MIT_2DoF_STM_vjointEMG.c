/*
* user-mn-MIT_2DoF_Ankle_STMjointEMG.c
*
*  Created on: Mar 5, 2019
*      Author: hm
*/


#ifdef INCLUDE_UPROJ_MIT_A2DOF
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT_2DoF_Ankle_v1.h"
#include "cmd-MIT_2DoF_Ankle_v1.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"
#include "user-mn-MIT_2DoF_STM_vjointEMG.h"
#include "user-mn-MIT_2DoF_control_API.h"
//****************************************************************************
// Variable(s)
//****************************************************************************

int32_t dp_stiffness = 0, ie_stiffness = 0;
int32_t dp_stiffness_des = 0, ie_stiffness_des = 0;

int LR_flip = 0;

int cal_angle_DP = 0;
int cal_angle_IE = 0;

int k_lim_rob = 1000;

int32_t DP_torque = 0, IE_torque = 0, torque_set_1 = 0, torque_set_2 = 0, DP_error = 0, IE_error = 0;
int32_t IE_torque_meas = 0, DP_torque_meas = 0, DP_torqueAdd = 0, IE_torqueAdd = 0; // units on these are unknown, but current-based.
int16_t DP_torque_trans = 0, IE_torque_trans = 0;
float IE_add_mult = 0;

// int16_t per_PFDF = 0;
// int16_t per_INEV = 0;
// uint16_t stiff_PFDF = 0;

int16_t EMGwindow[4][100];  // 4muscles, 100 window size. Initialize to baseline. Assuming a sample rate of 1kHz, we will be averaging over 0.1s
int win_front = 0;
int isnot_stim[4] = {1,1,1,1};  //{LG,TP,TA,PL}
int DP_stim_counter = 0, IE_stim_counter = 0;

int32_t EMGavgs[4] = {0,0,0,0}; // Initialize the EMG signals to 0;
float PFDF_state [3] = {0,0,0};
float INEV_state [3] = {0,0,0};

int32_t basecals_tot[4] = {0.0,0,0};
int32_t cals[2][4];
int rest_ctr = 0;
float stepsize = .001; //dt, if running at 1kHz
float stiffness_multiplier_1, stiffness_multiplier_2;

float pfdf_out = 0;
float inev_out = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************
void RK4_SIMPLE(float dtheta, float domega, float* cur_state);

//****************************************************************************
// Public Function(s)
//****************************************************************************
float ankle_2dof_get_PFDF(void)
{
    return pfdf_out;
}
float ankle_2dof_get_INEV(void)
{
    return inev_out;
}
//Call this function once in main.c, just before the while()
uint8_t ankle_2dof_STMjoint_init(void)
{
		int x, y;
		for(x = 0; x < 4; x++)
		{
			for(y = 0; y < 100; y++) EMGwindow[x][y] = 0;
		}

		if (LEFTY)
		{
			LR_flip = 1;
		}
		else
		{
			LR_flip = -1;
		}

		if (AUTOCAL == 1)
		{
			for(y = 0; y < 4; y++) cals[1][y] = 7000;
			for(y = 0; y < 4; y++) cals[0][y] = 0;

            user_data_1.w[9] = 1; //recall calibration process
		}
		else
		{
			cals[0][0] = MANMIN_LG; // LG
			cals[1][0] = MANMAX_LG; // LG

			cals[0][1] = MANMIN_TP; // TP
			cals[1][1] = MANMAX_TP; // TP

			cals[0][2] = MANMIN_TA; // TA
			cals[1][2] = MANMAX_TA; // TA

			cals[0][3] = MANMIN_PL;  //PL
			cals[1][3] = MANMAX_PL;  //PL
		}

    return 1;
}

uint8_t ankle_2dof_STMjoint_calibration_fsm(void)
{
    static int16_t cal_state = 0;
    static int16_t next_cal_state = 1;
    static int16_t state_t = 0;

    uint8_t ret = 0;

    cal_angle_DP = 0;
    cal_angle_IE = 0;

    state_t++;
    switch(cal_state)
    {
        case 0: //REST
            pfdf_out = 0;
            inev_out = 0;
            if (next_cal_state == 1)
            {
                basecals_tot[0] += EMGavgs[0];
                basecals_tot[1] += EMGavgs[1];
                basecals_tot[2] += EMGavgs[2];
                basecals_tot[3] += EMGavgs[3];
                rest_ctr++;
            }

            if (state_t>3000)
            {
                cal_state = next_cal_state;
                state_t = -1;
            }

            if(cal_state > 4) //End of calibration
            {
                cals[0][0] = basecals_tot[0]/rest_ctr;
                cals[0][1] = basecals_tot[1]/rest_ctr;
                cals[0][2] = basecals_tot[2]/rest_ctr;
                cals[0][3] = basecals_tot[3]/rest_ctr;

                cal_state = 0;
                next_cal_state =1;
                ret = 1;
            }

            break;

        case 1: //PLANTARFLEXION
            pfdf_out = -1;
            inev_out = 0;

            cal_angle_DP = -1400;
            if (EMGavgs[0] > cals[1][0]) // LG
            {
                cals[1][0] = EMGavgs[0];
            }
            if (state_t>3000)
            {
                cal_state = 0;
                next_cal_state = 2;
                state_t = -1;
            }
            break;

        case 2: //INVERSION
            pfdf_out = 0;
            inev_out = 1*LR_flip;
            cal_angle_IE = -700;
            if (EMGavgs[1] > cals[1][1]) // TP
            {
                cals[1][1] = EMGavgs[1];
            }
            if (state_t>3000)
            {
                cal_state = 0;
                next_cal_state = 3;
                state_t = -1;
            }
            break;

        case 3: //DORSIFLEXION
            pfdf_out = 1;
            inev_out = 0;

            cal_angle_DP = 800;
            if (EMGavgs[2] > cals[1][2]) // TA
            {
                cals[1][2] = EMGavgs[2];
            }
            if (state_t>3000)
            {
                cal_state = 0;
                next_cal_state = 4;
                state_t = -1;
            }
            break;

        case 4: //EVERSION
            pfdf_out = 0;
            inev_out = -1*LR_flip;

            cal_angle_IE = 700;
            if (EMGavgs[3] > cals[1][3]) // LG
            {
                cals[1][3] = EMGavgs[3];
            }
            if (state_t>3000)
            {
                cal_state = 0;
                next_cal_state = 5;
                state_t = -1;
            }
            break;
    }

    return ret;
}

uint8_t ankle_2dof_STMjoint_EMG_fsm(void)
{
    static uint32_t state_t = 0;
    static int8_t fsm_state = 0;
    static uint8_t state_transition=0;

    state_t++;

    if(user_data_1.w[9] == 1)
    {
        fsm_state =1;
        user_data_1.w[0] = 0;
    }

    get_EMG();
    switch(fsm_state)
    {
    	case 0:
            interpret_EMG(.85, .1, .0025);
            break;

    	case 1:
            state_transition = ankle_2dof_STMjoint_calibration_fsm();
    		break;

        default:
            break;
    }

    if(state_transition ==1)
    {
        state_transition = 0;
        fsm_state=0;
    }

    return 0;
}




void get_EMG(void) //Read the EMG signal, rectify, and integrate. Output an integrated signal.
{
 int16_t EMG_baseline[4] = {2027,2027,2470,2470};
 for( uint16_t musc = 0; musc < 4; musc ++)
 {
 	uint16_t cur_ain_val = get_adc1(musc);
 	int16_t cur_read = abs(cur_ain_val-EMG_baseline[musc]) * isnot_stim[musc] + cals[0][musc]/100 * (1 - isnot_stim[musc]);

 	EMGwindow[musc][win_front] = cur_read;
     if( win_front == 99)
     {
     	EMGavgs[musc] = (EMGavgs[musc] + cur_read - EMGwindow[musc][0]);
     }
     else
     {
     	EMGavgs[musc] = (EMGavgs[musc] + cur_read - EMGwindow[musc][win_front + 1]);
     }
 }
 if( win_front < 99)
 {
 	win_front ++;
 }
 else
 {
 	win_front = 0;
 }
}

void interpret_EMG (float k, float b, float J)
{
	int PF_torque_gain = PFTORQUEGAIN; //Denotes simulated torque at act = 1; Nm per act (0-1)
	int DF_torque_gain = DFTORQUEGAIN;
	int PFDF_stiffness_gain = PFDFSTIFFGAIN; // Denotes stiffness at mean(act1,act2) = 1;

	int IN_torque_gain = INTORQUEGAIN;
	int EV_torque_gain = EVTORQUEGAIN;
	int INEV_stiffness_gain = INEVSTIFFGAIN;

	float onthresh_dp = DPONTHRESH; //
	float onthresh_ie = IEONTHRESH; //

	// Calculate LG activation
	float LGact = 0;
	float LGthresh = cals[0][0] + (onthresh_dp * (cals[1][0]-cals[0][0]));
	if (EMGavgs[0] > LGthresh)
	{
		LGact = (EMGavgs[0] - LGthresh) / (cals[1][0] - LGthresh);
	}

	float TPact = 0;
	float TPthresh = cals[0][1] + (onthresh_ie * (cals[1][1]-cals[0][1]));
	if (EMGavgs[1] > TPthresh)
	{
		TPact = (EMGavgs[1] - TPthresh) / (cals[1][1] - TPthresh);
	}

	float TAact = 0;
	float TAthresh = cals[0][2] + (onthresh_dp * (cals[1][2]-cals[0][2]));
	if (EMGavgs[2] > TAthresh)
	{
		TAact = (EMGavgs[2] - TAthresh) / (cals[1][2] - TAthresh);
	}

	float PLact = 0;
	float PLthresh = cals[0][3] + (onthresh_ie * (cals[1][3]-cals[0][3]));
	if (EMGavgs[3] > PLthresh)
	{
		PLact = (EMGavgs[3] - PLthresh) / (cals[1][3] - PLthresh);
	}

	// PF - DF Calcs
	float TALG_diff = TAact - LGact;
	float T_PFDF;
	if (TALG_diff < 0)
	{
		T_PFDF = TALG_diff * PF_torque_gain;
	}
	else
	{
		T_PFDF = TALG_diff * DF_torque_gain;
	}

	// JOINT MODEL
	//first, we apply extra damping in co-contraction
	float cocon_thresh = COCONTHRESH;
	if ((TAact > cocon_thresh) && (LGact > cocon_thresh))
	{
		float stiffness = (TAact + LGact)/2.0;
		b = b * (erff(10.0*(stiffness-.15-cocon_thresh))+1)*20;
	}

	float dtheta = PFDF_state[1];
	float domega = -k/J * PFDF_state[0] - b/J * PFDF_state[1] + 1/J * T_PFDF; // Some dope dynamic equations, yo.

	// if (VIRTUALFIELD)
	// 		{
	// 			domega = -k/J * PFDF_state[0] - 10*b/J * PFDF_state[1] + 1/J * T_PFDF; // Some dope dynamic equations, yo.
	// 		}

	int max_DFangle = 9;
	int max_PFangle = -13;

	int k_lim = 250; //virtual spring constant opposing motion at virtual joint limit.
	int b_lim = 1; //virtual damping constant opposing motion at virtual joint limit.

/*
	if (PFDF_state[0] > max_DFangle)
	{
		domega = domega - k_lim/J * (PFDF_state[0] - max_DFangle) - b_lim/J * PFDF_state[1]; // Hittin a wall.
	}
	else if (PFDF_state[0] < max_PFangle)
	{
		domega = domega - k_lim/J * (PFDF_state[0] - max_PFangle) - b_lim/J * PFDF_state[1]; // Hittin the other wall.
	}
*/
	RK4_SIMPLE(dtheta, domega, PFDF_state); // Integration station.

	if (PFDF_state[0] > max_DFangle)
	{
		PFDF_state[0] = max_DFangle;
		// domega = domega - k_lim/J * (PFDF_state[0] - max_DFangle) - b_lim/J * PFDF_state[1]; // Hittin a wall.
	}
	else if (PFDF_state[0] < max_PFangle)
	{
		PFDF_state[0] = max_PFangle;
		// domega = domega - k_lim/J * (PFDF_state[0] - max_PFangle) - b_lim/J * PFDF_state[1]; // Hittin the other wall.
	}

	PFDF_state[2] = (TAact + LGact)/2.0 * PFDF_stiffness_gain; // stiffness comes from the common signal

    if(PFDF_state[0] >=0)
    {
        pfdf_out = PFDF_state[0]/max_DFangle;
    }
    else if(PFDF_state[0] <0)
    {
        pfdf_out = PFDF_state[0]/(-max_PFangle);
    }

	// IN - EV Calcs
	float PLTP_diff = PLact - TPact;
	float T_INEV;
	if (PLTP_diff < 0)
	{
		T_INEV = PLTP_diff * IN_torque_gain;
	}
	else
	{
		T_INEV = PLTP_diff * EV_torque_gain;
	}

	int max_EVangle = 5;
	int max_INangle = -5;

	float dtheta_inev = INEV_state[1];
	float domega_inev = -k/J * INEV_state[0] - b/J * INEV_state[1] + 1/J * T_INEV; // Some dope dynamic equations, yo.
	// if (INEV_state[0] > max_EVangle)
	// {
	// 	domega_inev = domega_inev - k_lim/J * (INEV_state[0] - max_EVangle) - b_lim/J * INEV_state[1]; // Hittin a wall.
	// }
	// else if (INEV_state[0] < max_INangle)
	// {
	// 	domega_inev = domega_inev - k_lim/J * (INEV_state[0] - max_INangle) - b_lim/J * INEV_state[1]; // Hittin the other wall.
	// }

	RK4_SIMPLE(dtheta_inev, domega_inev, INEV_state); // Integration station.
	if (INEV_state[0] > max_EVangle)
	{
		INEV_state[0] = max_EVangle;
		// domega = domega - k_lim/J * (PFDF_state[0] - max_DFangle) - b_lim/J * PFDF_state[1]; // Hittin a wall.
	}
	else if (INEV_state[0] < max_EVangle)
	{
		INEV_state[0] = max_EVangle;
		// domega = domega - k_lim/J * (PFDF_state[0] - max_PFangle) - b_lim/J * PFDF_state[1]; // Hittin the other wall.
	}

    //need to look.
    inev_out = INEV_state[0]/(max_EVangle)*LR_flip;

	INEV_state[2] = (PLact + TPact)/2.0 * INEV_stiffness_gain; // stiffness comes from the common signal

}

void RK4_SIMPLE(float d1_dt,float d2_dt, float* cur_state)
{
	float next_state[2];

	float F1 = d1_dt;
	float F2 = d1_dt + .5 * stepsize * F1;
	float F3 = d1_dt + .5 * stepsize * F2;
	float F4 = d1_dt + stepsize * F3;
	next_state[0] = cur_state[0] + (stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);

	F1 = d2_dt;
	F2 = d2_dt + .5 * stepsize * F1;
	F3 = d2_dt + .5 * stepsize * F2;
	F4 = d2_dt + stepsize * F3;
	next_state[1] = cur_state[1] + (stepsize/6) * (F1 + 2*F2 + 2*F3 + F4);

	cur_state[0] = next_state[0];
	cur_state[1] = next_state[1];
}


#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
