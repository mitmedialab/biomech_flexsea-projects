/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User projects
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developer] Luke Mooney, lmooney at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] user-ex-MIT_2DoF_Ankle_v1: User code running on Manage
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-28 | jfduval | New release
	* 2016-11-16 | jfduval | Cleaned code, improved formatting
	* 2019-02-13 | jfduval | Made it work with new stack, new demo
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_A2DOF
#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//To test your code without a walking controller enable the following:
#define A2DOF_INDEPENDENT_DEMO
//This code assumes that the two motors are free to spin - use with care!

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT_2DoF_Ankle_v1.h"
#include "cmd-MIT_2DoF_Ankle_v1.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

int state = -5;
int32_t dp_stiffness = 0, ei_stiffness = 0;

int32_t ank_angs_1[6] = {0,0,0,0,0,0};
int32_t mot_angs_1[6] = {0,0,0,0,0,0};

int32_t ank_angs_2[6] = {0,0,0,0,0,0};
int32_t mot_angs_2[6] = {0,0,0,0,0,0};

int32_t ank_angs_t[6] = {0,0,0,0,0,0};

int32_t ank_angs_dp[6] = {0,0,0,0,0,0};
int32_t ank_angs_ie[6] = {0,0,0,0,0,0};

int32_t ankle_vel_t = 0, ankle_torque_t = 0, last_ankle_torque_t = 0;
int32_t angle_zero = 0, init_angle, last_DP_torque = 0;
int32_t ankle_vel_1, ankle_trans_1 = 0, angle_zero_1=0, init_angle_1;
int32_t mot_vel_1, ankle_torque_1, last_ankle_torque_1, mot_ang_1;
int32_t ankle_vel_2, ankle_trans_2 = 0, angle_zero_2=0, init_angle_2;
int32_t mot_vel_2, ankle_torque_2, last_ankle_torque_2, mot_ang_2;

uint8_t my_control;
int16_t my_pwm[2] = {0,0};
int16_t my_cur[2] = {0,0};

int32_t DP_torque = 0, EI_torque_1 = 0, EI_torque_2 = 0;

int16_t glob_var_1;
int16_t glob_var_2;
int16_t glob_var_3;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void ankle_2dof_refresh_values(void);
static void ankle_2dof_independent_demo(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_ankle_2dof(void)
{
	my_control = CTRL_NONE;
	state = -5;
	// state = 1;
}

//Ankle 2-DoF Finite State Machine.
//Call this function in one of the main while time slots.
void ankle_2dof_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)

	uint8_t info[2] = {PORT_RS485_1, PORT_RS485_1};
	static uint32_t time = 0, state_t = 0;

	//Increment time (1 tick = 1ms)
	time++;
	state_t++;

	//Before going to a state we refresh values:
	ankle_2dof_refresh_values();

	//Demo mode - no walking controller when enabled:
	#ifdef A2DOF_INDEPENDENT_DEMO

		ankle_2dof_independent_demo();
		return;

	#endif

	//State machine:
	switch(state)
	{
		case -5://Wait for 10 seconds to let everything load

			my_control = CTRL_OPEN;
			my_pwm[0] = 0;
			my_pwm[1] = 0;
			init_angle = (*exec1.enc_ang);
			if (state_t>10000)
			{
				state_t = -1;
				state = -4;
			}

			break;

		case -4: //Do not start until the right motor is spun 10 degrees by hand

			my_control = CTRL_OPEN;
			my_pwm[0] = 0;
			my_pwm[1] = 0;

			if ((*exec1.enc_ang) > init_angle+10 || (*exec1.enc_ang) < init_angle-10)
			{
				state = -2;
				state_t = -1;
			}

			break;

		case -3: //

			if (state_t>5000)
			{
				state = -2;
				state_t = -1;
				angle_zero_1 = (*exec1.enc_ang);
				angle_zero_2 = (*exec2.enc_ang);
			}

			break;

		case -2: //move the motors to maximum plantar flexion

			my_control = CTRL_OPEN;
			my_pwm[0] = -70;
			my_pwm[1] = -70;

			if (state_t>8000)
			{
				state = -1;
				state_t = -1;
				angle_zero_1 = (*exec1.enc_ang);
				angle_zero_2 = (*exec2.enc_ang);
			}

			break;

		case -1: //move the motor to 20 degrees from maximum dorsiflexion and stop

			my_control = CTRL_OPEN;

			my_pwm[0] = 70;
			my_pwm[1] = 70;

			if (ank_angs_1[0]<2000)
			{
				my_pwm[0] = 0;
			}
			if (ank_angs_2[0]<2000)
			{
				my_pwm[1] = 0;
			}
			if (ank_angs_1[0]<2000 && ank_angs_2[0]<2000)
			{
				state = 0;
				state_t = -1;

				my_control = CTRL_CURRENT;

				info[0] = PORT_RS485_1;
				tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

				info[0] = PORT_RS485_2;
				tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);

				HAL_Delay(10);

				info[0] = PORT_RS485_1;
				tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 30, 0, 0);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

				info[0] = PORT_RS485_2;
				tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 30, 0, 0);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);

				user_data_1.w[0] = 100;
				user_data_1.w[1] = 100;
			}

			break;

		case 0: //behave like a spring both in PF-DF direction and EV-INV direction until 5 degrees of DF is achieved

			my_control = CTRL_CURRENT;
			my_pwm[0] = 0;
			my_pwm[1] = 0;

			dp_stiffness = 100;
			ei_stiffness = 100; 	// originally 100

			DP_torque = (2000-ank_angs_t[0])*dp_stiffness;
			EI_torque_1 = -(ank_angs_t[0]-ank_angs_1[0])*ei_stiffness;
			EI_torque_2 = -(ank_angs_t[0]-ank_angs_2[0])*ei_stiffness;

			//DP_torque= 0;
			set_ankle_torque_1(-DP_torque+EI_torque_1);
			set_ankle_torque_2(-DP_torque+EI_torque_2);


			if (ank_angs_t[0]<1500)
			{
				state_t = -1;
				state = 1;
			}

			break;

		case 1://still behave like a spring, but try to apply max torque in the PF-DF direction until ankle reached 20 degrees of PF

			DP_torque = (2000-ank_angs_t[0])*dp_stiffness;
			if (DP_torque >last_DP_torque)
			{
				last_DP_torque = DP_torque;
			}

			EI_torque_1 = -(ank_angs_t[0]-ank_angs_1[0])*ei_stiffness;
			EI_torque_2 = -(ank_angs_t[0]-ank_angs_2[0])*ei_stiffness;

			set_ankle_torque_1(-last_DP_torque+EI_torque_1); //pos
			set_ankle_torque_2(-last_DP_torque+EI_torque_2); //neg

			if (ank_angs_t[0]>=2000)
			{
				state_t = -1;
				state = 0;
				last_DP_torque = 0;
			}

			break;

		default:
			//Handle exceptions here
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}

//Second state machine for the Ankle project
//Deals with the communication between Manage and 2x Execute, on different RS-485 busses
//This function is called at 1kHz
void ankle_2dof_fsm_2(void)
{
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)

	static uint8_t ex_refresh_fsm_state = 0;
	static uint32_t timer = 0;
	uint8_t info[2] = {PORT_RS485_1, PORT_RS485_1};

	//This FSM talks to the slaves at 250Hz each
	switch(ex_refresh_fsm_state)
	{
		case 0:		//Power-up

			if(timer < 7000)
			{
				//We wait 7s before sending the first commands
				timer++;
			}
			else
			{
				//Ready to start transmitting
				ex_refresh_fsm_state = 1;
			}

			break;

		case 1:	//Communicating with Execute #1

			info[0] = PORT_RS485_1;
			tx_cmd_ankle2dof_r(TX_N_DEFAULT, 0, my_control, my_cur[0], my_pwm[0]);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

			// ex_refresh_fsm_state++;
			ex_refresh_fsm_state=3;

			break;

		case 2:

			//Skipping one cycle
			ex_refresh_fsm_state++;

			break;

		case 3:	//Communicating with Execute #2

			info[0] = PORT_RS485_2;
			tx_cmd_ankle2dof_r(TX_N_DEFAULT, 1, my_control, my_cur[1], my_pwm[1]);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);
			ex_refresh_fsm_state=1;
			// ex_refresh_fsm_state++;

			break;

		case 4:

			//Skipping one cycle
			ex_refresh_fsm_state = 1;

			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

//
static void ankle_2dof_refresh_values(void)
{
	//motor angle in degrees
	//(*exec1.enc_ang) -> positive in plantarflexion
	//(*exec2.enc_ang) -> positive in dorsiflexion
	mot_ang_1 = ((*exec1.enc_ang)-angle_zero_1)/46-129; //46 should be 45.5
	mot_ang_2 = ((*exec2.enc_ang)-angle_zero_2)/46-129;  //46 should be 45.5

	ankle_trans_1 = get_ankle_trans(mot_ang_1);
	ankle_trans_2 = get_ankle_trans(mot_ang_2);

	//ankle angle in deg x 100
	ank_angs_1[5] = ank_angs_1[4];
	ank_angs_1[4] = ank_angs_1[3];
	ank_angs_1[3] = ank_angs_1[2];
	ank_angs_1[2] = ank_angs_1[1];
	ank_angs_1[1] = ank_angs_1[0];
	ank_angs_1[0] = get_ankle_ang(mot_ang_1);

	ank_angs_2[5] = ank_angs_2[4];
	ank_angs_2[4] = ank_angs_2[3];
	ank_angs_2[3] = ank_angs_2[2];
	ank_angs_2[2] = ank_angs_2[1];
	ank_angs_2[1] = ank_angs_2[0];
	ank_angs_2[0] = get_ankle_ang(mot_ang_2);

	ank_angs_t[0]  = (ank_angs_1[0]+ank_angs_2[0])/2;
	ank_angs_t[1]  = (ank_angs_1[1]+ank_angs_2[1])/2;
	ank_angs_t[2]  = (ank_angs_1[2]+ank_angs_2[2])/2;
	ank_angs_t[3]  = (ank_angs_1[3]+ank_angs_2[3])/2;
	ank_angs_t[4]  = (ank_angs_1[4]+ank_angs_2[4])/2;
	ank_angs_t[5]  = (ank_angs_1[5]+ank_angs_2[5])/2;

	glob_var_1 = ank_angs_1[0];
	glob_var_2 = ank_angs_2[0];
	glob_var_3 = ank_angs_t[0];

	mot_angs_1[5] = mot_angs_1[4];
	mot_angs_1[4] = mot_angs_1[3];
	mot_angs_1[3] = mot_angs_1[2];
	mot_angs_1[2] = mot_angs_1[1];
	mot_angs_1[1] = mot_angs_1[0];
	mot_angs_1[0] = -(*exec1.enc_ang); //motor angle in degrees

	mot_angs_2[5] = mot_angs_2[4];
	mot_angs_2[4] = mot_angs_2[3];
	mot_angs_2[3] = mot_angs_2[2];
	mot_angs_2[2] = mot_angs_2[1];
	mot_angs_2[1] = mot_angs_2[0];
	mot_angs_2[0] = (*exec2.enc_ang); //motor angle in degrees

	//ankle velocity in deg/sec x 100
	ankle_vel_1 = (ank_angs_1[0]-ank_angs_1[5])*250/5;
	ankle_vel_2 = (ank_angs_2[0]-ank_angs_2[5])*250/5;
	ankle_vel_t = (ank_angs_t[0]-ank_angs_t[5])*250/5;

	mot_vel_1 = (mot_angs_1[0]-mot_angs_1[5]);
	mot_vel_2 = (mot_angs_2[0]-mot_angs_2[5]);

	// user_data_1.r[0] = ank_angs_1[0];
	// user_data_1.r[1] = ank_angs_2[0];

	ank_angs_ie[0]  = (ank_angs_1[0]-ank_angs_2[0])/2;
	ank_angs_dp[0]  = (ank_angs_1[0]+ank_angs_2[0])/2;
}

//returns the ankle angle [deg x 100]
//0 at maximum dorsiflexion
int16_t get_ankle_ang(double ma) //mot_ang [deg] where mot_ang = 0 is at maximum plantarflexion
{
	(ma < 0) ? ma = 0 : ma;
	return (int16_t)(A0+A1*cos(ma*W)+B1*sin(ma*W)+A2*cos(ma*W*2.0)+B2*sin(ma*W*2.0));
}

//return 10x the transmission ratio
int16_t get_ankle_trans(double ma) //mot_ang [deg] where mot_ang = 0 is at maximum plantarflexion
{
   double slope = 1.0, transmission = 0.0;
   double tmp1 = 0.0, tmp2 = 0.0;

   (ma < 0) ? ma = 0 : ma;

   tmp1 = ma*W;
   tmp2 = ma*W*2.0;
   slope = (-W*A1*sin(tmp1) + W*B1*cos(tmp1) - 2.0*W*A2*sin(tmp2) + 2.0*W*B2*cos(tmp2));

   transmission = ((double)(-1000.0/slope));

   if(transmission > 3000)
   {
	   return (int16_t)3000;
   }
   else
   {
	   return (int16_t)(transmission);
   }

   return 0;
}

void set_ankle_torque_1(int32_t des_torque) //des_torque in mNm
{
	int32_t motor_torque = des_torque/(ankle_trans_1/10); 	//[mNm] at the motor
	int32_t des_motor_current = motor_torque*10; 			//[mA] at the motor
	my_cur[0] = des_motor_current; 							//[current IU]
	// user_data_1.r[2] = des_motor_current; 					//[current IU]
}

void set_ankle_torque_2(int32_t des_torque) //des_torque in mNm
{
	int32_t motor_torque = des_torque/(ankle_trans_2/10); 	//[mNm] at the motor
	int32_t des_motor_current = motor_torque*10; 			//[mA] at the motor
	my_cur[1] = des_motor_current; 							//[current IU]
	// user_data_1.r[3] = des_motor_current; 					//[current IU]
}

//Demo code to test hardware and software. This is not a walking controller.
void ankle_2dof_independent_demo(void)
{
	static uint8_t spinCycles = 5;
	static uint32_t time = 0, state_t = 0;
	uint8_t info[2] = {PORT_RS485_1, PORT_RS485_1};
	//Increment time (1 tick = 1ms)
	time++;
	state_t++;

	//State machine:
	user_data_1.r[5] = state;

	switch(state)
	{
		case -5:	//Wait for 5 seconds to let everything load

			my_control = CTRL_OPEN;

			my_pwm[0] = 0;
			my_pwm[1] = 0;
			// init_angle = (*exec1.enc_ang);
			if (state_t > 7000)
			{
				state_t = -1;
				state = -4;
			}

			break;


		case -4: //move the motors to maximum plantar flexion

			my_control = CTRL_OPEN;

			my_pwm[0] = -1350; // 1.3V
			my_pwm[1] = -1350;

			if (state_t> 10000)
			{
				state = -3;
				state_t = -1;
				angle_zero_1 = *exec1.enc_ang; 	//exec1.enc_display;
				angle_zero_2 = *exec2.enc_ang;	//exec2.enc_display;
			}

            break;

		case -3: //move the motor to 20 degrees from maximum dorsiflexion and stop

			my_control = CTRL_OPEN;

			my_pwm[0] = 1250;
			my_pwm[1] = 1250;

			if (ank_angs_1[0]<REST_MOTOR_ANGLE)
			{
				my_pwm[0] = 0;

			}
			if (ank_angs_2[0]<REST_MOTOR_ANGLE)
			{
				my_pwm[1] = 0;
			}

			if (ank_angs_1[0]<REST_MOTOR_ANGLE && ank_angs_2[0]<REST_MOTOR_ANGLE)
			{
				// state = -2;
				state = 1;
				state_t = -1;
			}
			// user_data_1.r[2] = REST_MOTOR_ANGLE;
			user_data_1.r[1] = mot_ang_1;//ank_angs_dp[0]*100;
			user_data_1.r[2] =  mot_ang_2;//ank_angs_ie[0]*100;
			user_data_1.r[3] = ank_angs_1[0];//ank_angs_dp[0]*100;
			user_data_1.r[4] = ank_angs_2[0];

			break;

		case -2 :
			if(state_t ==0)
			{
				my_control = CTRL_CURRENT;

				info[0] = PORT_RS485_1;
				tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

				info[0] = PORT_RS485_2;
				tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);
				/* Understanding :
					- change mode of the control to current control for both execute
				*/
			}
			else if(state_t > 10)
			{
				state = -1;
				state_t = -1;
			}

			break;

		case -1 :
			if(state_t ==0)
			{
				info[0] = PORT_RS485_1;
				tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 30, 0, 0);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

				info[0] = PORT_RS485_2;
				tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 30, 0, 0);
				packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);
				/*
					write current control gain of
					kp =30, ki = 0, kd = 0
					Q-JF : for control gain change since then
				*/
			}
			else if(state_t > 100)
			{
				state = 0;
				state_t = -1;
			}

		case 0:
			state = 1;
			state_t = -1;
			break;

		case 1:
			my_control = CTRL_OPEN;

			my_pwm[0] = 0;
			my_pwm[1] = 0;
			// user_data_1.r[3] =  *exec1.enc_ang;//ank_angs_dp[0]*100;
			// user_data_1.r[4] =  *exec2.enc_ang;//ank_angs_ie[0]*100;
			break;
		case 2:
			/*
			my_control = CTRL_CURRENT;
			my_pwm[0] = 0;
			my_pwm[1] = 0;

			DP_error = (PFDF_state[0]*100-(REST_MOTOR_ANGLE-ank_angs_dp[0])); // if error is positive, ankle should try to dorsiflex
			IE_error = (INEV_state[0]*100*LR_flip-ank_angs_ie[0]); // if error is positive, ankle should try to invert


			dp_stiffness = (10 + PFDF_state[2]);
			ie_stiffness = (10 + INEV_state[2]);

			if (ISON_STIM && !VIRTUALFIELD)
			{
				dp_stiffness = dp_stiffness * STIMSTIFFGAIN;
				ie_stiffness = ie_stiffness * STIMSTIFFGAIN;
			}

			DP_torque = DP_error*dp_stiffness; // pos torque is dorsiflexion
			IE_torque = IE_error*ie_stiffness; // pos torque is inversion (for left foot)

			if (VIRTUALFIELD)
			{
				DP_torqueAdd =  DFDAMPING * ank_vel_t;
				IE_add_mult = 1;//-(pow((ank_angs_dp[0]-1000),2))/1000000;
				if (IE_add_mult < 0)
				{
					IE_add_mult = 0;
				}
				if (abs(ank_angs_ie[0]) > SAFEZONE)
				{
					IE_torqueAdd = IESPRING * ank_angs_ie[0] * IE_add_mult;
				}
				else
				{
					IE_torqueAdd = 0;
				}

			}

			torque_set_1 = (DP_torque + DP_torqueAdd) - (IE_torque + IE_torqueAdd);
			torque_set_2 = (DP_torque + DP_torqueAdd) + (IE_torque + IE_torqueAdd);

			if( ank_angs_1[0] > 2400)
			{
				torque_set_1 = torque_set_1 + k_lim_rob*(ank_angs_1[0]-2400);
			}
			else if( ank_angs_1[0] < 200)
			{
				torque_set_1 = torque_set_1 + k_lim_rob*(ank_angs_1[0]-200);
			}

			if( ank_angs_2[0] > 2400)
			{
				torque_set_2 = torque_set_2 + k_lim_rob*(ank_angs_2[0]-2400);
			}
			else if( ank_angs_2[0] < 200)
			{
				torque_set_2 = torque_set_2 + k_lim_rob*(ank_angs_2[0]-200);
			}


			set_ankle_torque_1(torque_set_1);
			set_ankle_torque_2(torque_set_2);

			DP_torque_meas = (exec1.current + exec2.current - DP_torqueAdd);// (exec1.current - exec2.current)/2.0;
			IE_torque_meas = (exec1.current - exec2.current + IE_torqueAdd*4);

			if ((BLANKWINDOW > 0 && STIMSTIFFGAIN>3) || BLANKWINDOW > 2)
				if ((ank_angs_1[0] > 2200 || ank_angs_2[0] > 2200) && DP_torque_meas > 0)
				{
					DP_torque_meas = 0;
				}
				else if ((ank_angs_1[0] < 400 || ank_angs_2[0] < 400) && DP_torque_meas < 0)
				{
					DP_torque_meas = 0;
				}
				else if (ank_angs_ie[0] < -500 && IE_torque_meas < 0)
				{
					IE_torque_meas = 0;
				}
				else if (ank_angs_ie[0] > 500 && IE_torque_meas > 0)
				{
					IE_torque_meas = 0;
				}


			//SET STIM DATA
			if (ISON_STIM)
			{
				if (DP_torque_meas >= DPSTIM_ONTHRESH)
				{
					isnot_stim[0] = 0;
					DP_torque_trans = DP_torque_meas; //DP_torque - DPSTIM_ONTHRESH;
					DP_stim_counter = 1;
				}
				else if (DP_torque_meas <= -DPSTIM_ONTHRESH)
				{
					isnot_stim[2] = 0;
					DP_torque_trans = DP_torque_meas;
					DP_stim_counter = 1;
				}
				else if (DP_stim_counter > 0 && DP_stim_counter < BLANKWINDOW)
				{
					DP_stim_counter += 1;
					DP_torque_trans = 0;
				}
				else
				{
					isnot_stim[0] = 1;
					isnot_stim[2] = 1;
					DP_torque_trans = 0;
					DP_stim_counter = 0;
				}


				if (IE_torque_meas >= IESTIM_ONTHRESH)
				{
					isnot_stim[3] = 0;
					IE_torque_trans = IE_torque_meas;
					IE_stim_counter = 1;
				}
				else if (IE_torque_meas <= -IESTIM_ONTHRESH)
				{
					isnot_stim[1] = 0;
					IE_torque_trans = IE_torque_meas;
					IE_stim_counter = 1;
				}
				else if (IE_stim_counter > 0 && IE_stim_counter < BLANKWINDOW)
				{
					IE_stim_counter += 1;
					IE_torque_trans = 0;
				}
				else
				{
					isnot_stim[1] = 1;
					isnot_stim[3] = 1;
					IE_torque_trans = 0;
					IE_stim_counter = 0;
				}

			}

			// UPDATE THE GUI
			if (ank_angs_dp[0] - 1000 < 0)
			{
				per_PFDF = (REST_MOTOR_ANGLE-ank_angs_dp[0]) *100 / (1000);
			}
			else
			{
				per_PFDF = (REST_MOTOR_ANGLE-ank_angs_dp[0]) *100 / (1500);
			}

			if (per_PFDF > 100)
			{
				per_PFDF = 100;
			}
			else if (per_PFDF < -100)
			{
				per_PFDF = -100;
			}

			per_INEV = -ank_angs_ie[0]*100/500;
			if (per_INEV > 100)
			{
				per_INEV = 100;
			}
			else if (per_INEV < -100)
			{
				per_INEV = -100;
			}

			stiff_PFDF = PFDF_state[2] * 100 / 50;

			 */
			break;

		default:
			//Handle exceptions here
			break;
	}
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
