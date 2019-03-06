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

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-mn-MIT_2DoF_Ankle_v1.h"
#include "cmd-MIT_2DoF_Ankle_v1.h"
#include <flexsea_comm.h>
#include <math.h>
#include "flexsea_sys_def.h"
#include "flexsea_system.h"

#include "user-mn-MIT_2DoF_control_API.h"
#include "user-mn-MIT_2DoF_STM_vjointEMG.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

int32_t ank_angs_1[6] = {0,0,0,0,0,0};
int32_t mot_angs_1[6] = {0,0,0,0,0,0};
int32_t ank_angs_2[6] = {0,0,0,0,0,0};
int32_t mot_angs_2[6] = {0,0,0,0,0,0};
int32_t ank_angs_dp[6] = {0,0,0,0,0,0};
int32_t ank_angs_ie[6] = {0,0,0,0,0,0};

int32_t ank_vel_t = 0, ank_vel_d = 0, ankle_torque_t = 0, last_ankle_torque_t = 0;
int32_t angle_zero = 0, init_angle, last_DP_torque = 0;
int32_t ank_vel_1, ankle_trans_1 = 0, angle_zero_1=0, init_angle_1;
int32_t mot_vel_1, ankle_torque_1, last_ankle_torque_1, mot_ang_1;
int32_t ank_vel_2, ankle_trans_2 = 0, angle_zero_2=0, init_angle_2;
int32_t mot_vel_2, ankle_torque_2, last_ankle_torque_2, mot_ang_2;

uint8_t my_control;
int16_t my_pwm[2] = {0,0};
int16_t my_cur[2] = {0,0};


//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void ankle_2dof_sensor_update(void);
static uint8_t ankle_2dof_init_actuator(void);
static int16_t get_ankle_ang(double);
static int16_t get_ankle_trans(double);
//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function once in main.c, just before the while()
void init_ankle_2dof(void)
{
	return;
}


void set_ankle_torque_1(int32_t des_torque) //des_torque in mNm
{
	int32_t motor_torque = des_torque/(ankle_trans_1)*10; 	//[mNm] at the motor
	int32_t des_motor_current = motor_torque*10; 			//[mA] at the motor
	my_cur[0] = des_motor_current; 							//[current IU]
	// user_data_1.r[2] = des_motor_current; 					//[current IU]
}

void set_ankle_torque_2(int32_t des_torque) //des_torque in mNm
{
	int32_t motor_torque = des_torque/(ankle_trans_2)*10; 	//[mNm] at the motor
	int32_t des_motor_current = motor_torque*10; 			//[mA] at the motor
	my_cur[1] = des_motor_current; 							//[current IU]
	// user_data_1.r[3] = des_motor_current; 					//[current IU]
}

uint8_t ankle_2dof_init_actuator(void)
{
	static uint32_t state_t = 0;
	static int16_t function_state = -5;

	state_t++;

	static int16_t open_motor_voltage1 = 1250;
	static int16_t open_motor_voltage2 = 1450;

	static uint8_t ret=0;

	user_data_1.r[3] = function_state;
	user_data_1.r[4] = ank_angs_1[0];
	user_data_1.r[5] = ank_angs_2[0];

	switch(function_state)
	{
		case -5:	//Wait for 7 seconds to let executes boot up
			my_control = CTRL_OPEN;

			my_pwm[0] = 0;
			my_pwm[1] = 0;

			if (state_t > 7000)
			{
				state_t = -1;
				function_state = -4;
			}
			break;

		case -4:
			//Move the motors to maximum plantar flexion
			//Set reference angle
			my_control = CTRL_OPEN;

			my_pwm[0] = -1250;
			my_pwm[1] = -1650;

			if (state_t> 10000)
			{
				function_state = -3;
				state_t = -1;
				angle_zero_1 = *exec1.enc_ang; 	//exec1.enc_display;
				angle_zero_2 = *exec2.enc_ang;	//exec2.enc_display;
			}

            break;

		case -3: //move the motor to 20 degrees from maximum dorsiflexion and stop
			my_control = CTRL_OPEN;

			my_pwm[0] = 1250;
			my_pwm[1] = 1550;

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
				function_state = -2;
				state_t = -1;
			}

			break;

		case -2:
			if (state_t> 1000)
			{
				function_state = -1;
				state_t = -1;
			}
			break;
		case -1:
			ret = 1;
			break;
	}

	return ret;
}
//Ankle 2-DoF Finite State Machine.
//Call this function in one of the main while time slots.
void ankle_2dof_fsm_1(void)
{
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)

	static uint32_t state_t = 0;
	static uint32_t timer = 0;
	static int16_t main_fsm_state = -2;
	static uint8_t state_transition=0;

	//Increment time (1 tick = 1ms)
	timer++;
	state_t++;
	//Before going to a state we refresh values:
	ankle_2dof_sensor_update();
	// ankle_2dof_control_update();

	user_data_1.r[0] = main_fsm_state;

	static float pfdf_control;
	static float inev_control;


	switch(main_fsm_state)
	{
		case -2:
			state_transition = ankle_2dof_init_actuator();
			break;

		case -1:
			state_transition = ankle_2dof_control_init_position();
			// state_transition = ankle_2dof_control_init_impedance();
		 	break;

		case 0:
			main_fsm_state =50;

			break;

		case 1: //STM paper EMG based actuation
			state_transition = ankle_2dof_STMjoint_init();
			break;

		case 2:
			state_transition = ankle_2dof_STMjoint_EMG_fsm();

			pfdf_control = ankle_2dof_get_PFDF();
			inev_control = ankle_2dof_get_INEV();
			ankle_2dof_control_update_position(pfdf_control,inev_control, 30,0,0);
			break;

		case 50:
			pfdf_control = (float)user_data_1.w[8]*0.001;
			inev_control = (float)user_data_1.w[9]*0.001;

			ankle_2dof_control_update_position(pfdf_control, inev_control, 30,0,0);
			break;

		case 99:
			state_transition = ankle_2dof_control_demo_position_fsm();
			// state_transition =ankle_2dof_control_demo_impedance_fsm();
			break;

		case 100:
			my_control = CTRL_OPEN;
			my_pwm[0] = 0;
			my_pwm[1] = 0;
			break;
	}

	if(state_transition ==1)
	{
		main_fsm_state++;
		state_transition = 0;
		state_t = -1;
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
			ex_refresh_fsm_state=2;

			break;


		case 2:	//Communicating with Execute #2

			info[0] = PORT_RS485_2;
			tx_cmd_ankle2dof_r(TX_N_DEFAULT, 1, my_control, my_cur[1], my_pwm[1]);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);
			ex_refresh_fsm_state=1;
			// ex_refresh_fsm_state++;

			break;

		default:
			break;
	}

	#endif	//ACTIVE_PROJECT == PROJECT_ANKLE_2DOF
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

//Note: 'static' makes them private; they can only called from functions in this
//file. It's safer than making everything global.

void ankle_2dof_sensor_update(void)
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

	ank_angs_dp[0]  = (ank_angs_1[0]+ank_angs_2[0])/2;
	ank_angs_dp[1]  = (ank_angs_1[1]+ank_angs_2[1])/2;
	ank_angs_dp[2]  = (ank_angs_1[2]+ank_angs_2[2])/2;
	ank_angs_dp[3]  = (ank_angs_1[3]+ank_angs_2[3])/2;
	ank_angs_dp[4]  = (ank_angs_1[4]+ank_angs_2[4])/2;
	ank_angs_dp[5]  = (ank_angs_1[5]+ank_angs_2[5])/2;

	ank_angs_ie[0]  = (ank_angs_1[0]-ank_angs_2[0])/2;
	ank_angs_ie[1]  = (ank_angs_1[1]-ank_angs_2[1])/2;
	ank_angs_ie[2]  = (ank_angs_1[2]-ank_angs_2[2])/2;
	ank_angs_ie[3]  = (ank_angs_1[3]-ank_angs_2[3])/2;
	ank_angs_ie[4]  = (ank_angs_1[4]-ank_angs_2[4])/2;
	ank_angs_ie[5]  = (ank_angs_1[5]-ank_angs_2[5])/2;

	mot_angs_1[5] = mot_angs_1[4];
	mot_angs_1[4] = mot_angs_1[3];
	mot_angs_1[3] = mot_angs_1[2];
	mot_angs_1[2] = mot_angs_1[1];
	mot_angs_1[1] = mot_angs_1[0];
	mot_angs_1[0] = mot_ang_1;//(*exec1.enc_ang); //motor angle in degrees

	mot_angs_2[5] = mot_angs_2[4];
	mot_angs_2[4] = mot_angs_2[3];
	mot_angs_2[3] = mot_angs_2[2];
	mot_angs_2[2] = mot_angs_2[1];
	mot_angs_2[1] = mot_angs_2[0];
	mot_angs_2[0] = mot_ang_2;//(*exec2.enc_ang); //motor angle in degrees

	//ankle velocity in deg/sec x 100
	ank_vel_1 = (ank_angs_1[0]-ank_angs_1[5])*250/5;
	ank_vel_2 = (ank_angs_2[0]-ank_angs_2[5])*250/5;
	ank_vel_t = (ank_angs_dp[0]-ank_angs_dp[5])*250/5;
	ank_vel_d = (ank_angs_ie[0]-ank_angs_ie[5])*250/5;

	mot_vel_1 = (mot_angs_1[0]-mot_angs_1[5]);
	mot_vel_2 = (mot_angs_2[0]-mot_angs_2[5]);
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

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
