/*
 * user-mn-MIT_2DoF_control_API.c
 *
 *  Created on: Mar 5, 2019
 *      Author: Seong Ho Yeon_ML
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

#include "user-mn-MIT_2DoF_control_API.h"
#include "user-mn-MIT_2DoF_STM_vjointEMG.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

extern uint8_t my_control;
extern int16_t my_pwm[2];
extern int16_t my_cur[2];
extern int32_t ank_angs_1[6];
extern int32_t ank_angs_2[6];

static float target_ankle_angle[2][5] ={{0,0,0,0,0},{0,0,0,0,0}};

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************


//****************************************************************************
// Private Function(s)
//****************************************************************************
void ankle_2dof_control_update(void)
{
	//ankle angle in deg x 100
	for(int m=0; m<2;m++)
	{
		for(int ch=0;ch<4;ch++)
		{
			target_ankle_angle[m][ch+1] = target_ankle_angle[m][ch];
		}
	}
}

//Demo code to test hardware and software. This is not a walking controller.
uint8_t ankle_2dof_control_init_position(void)
{
	static uint32_t state_t = 0;
	static int8_t function_state = 0;

	static uint8_t ret=0;

	state_t++;

	my_control = CTRL_OPEN;

	if(state_t>100)
		ret =1;

	return ret;
}
uint8_t ankle_2dof_control_init_impedance(void)
{
	static uint32_t state_t = 0;
	static int8_t function_state = 0;

	static uint8_t ret=0;
	uint8_t info[2] = {PORT_RS485_1, PORT_RS485_1};

	state_t++;

	switch(function_state)
	{
		case 0 :
			my_control = CTRL_CURRENT;

			info[0] = PORT_RS485_1;
			tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

			function_state = 1;
			state_t = -1;
			break;

		case 1:
			if(state_t>100)
			{
				function_state = 2;
				state_t =-1;
			}
			break;

		case 2:
			my_control = CTRL_CURRENT;

			info[0] = PORT_RS485_2;
			tx_cmd_ctrl_mode_w(TX_N_DEFAULT, CTRL_CURRENT);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);

			function_state = 3;
			state_t = -1;
			break;

		case 3:
			if(state_t>100)
			{
				function_state = 4;
				state_t =-1;
			}
			break;

		case 4:
			my_control = CTRL_CURRENT;

			info[0] = PORT_RS485_1;
			tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 1000, 2, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_1, info, SEND_TO_SLAVE);

			function_state = 5;
			state_t = -1;
			break;

		case 5:
			if(state_t>100)
			{
				function_state = 6;
				state_t =-1;
			}
			break;


		case 6:
			my_control = CTRL_CURRENT;

			info[0] = PORT_RS485_2;
			tx_cmd_ctrl_i_g_w(TX_N_DEFAULT, 1000, 2, 0);
			packAndSend(P_AND_S_DEFAULT, FLEXSEA_EXECUTE_2, info, SEND_TO_SLAVE);

			function_state = 7;
			state_t = -1;
			break;
		case 7:
			if(state_t>100)
			{
				function_state = 8;
				state_t =-1;
			}
			break;
		case 8:
			ret = 1;
			break;
		default:
			break;
	}

	return ret;
}

void ankle_2dof_control_update_target(float pfdf, float inev)
{
	static const int16_t ANKLE_ANGLE_MIN_DF =250;
	static const int16_t ANKLE_ANGLE_REST = 1000;
	static const int16_t ANKLE_ANGLE_MAX_PF = 2400;

	static const int16_t ANKLE_ANGLE_INEV_CONTRIB = 285;

	static float temp=0;

	if(pfdf >1)
		pfdf = 1;
	else if(pfdf <-1)
		pfdf = -1;

	if(inev >1)
		inev =1;
	else if( inev<-1)
		inev = -1;

	if (pfdf>=0)
	{
		temp = ANKLE_ANGLE_REST - (ANKLE_ANGLE_REST-ANKLE_ANGLE_MIN_DF)*pfdf;
	}
	else
	{
		temp = ANKLE_ANGLE_REST - (ANKLE_ANGLE_MAX_PF-ANKLE_ANGLE_REST)*pfdf; //pfdf<0
	}


	target_ankle_angle[0][0] = temp + ANKLE_ANGLE_INEV_CONTRIB*inev;
	target_ankle_angle[1][0] = temp - ANKLE_ANGLE_INEV_CONTRIB*inev;

	for(int i=0;i<2;i++)
	{
		if(target_ankle_angle[i][0] >ANKLE_ANGLE_MAX_PF )
			target_ankle_angle[i][0] = ANKLE_ANGLE_MAX_PF;
		else if(target_ankle_angle[i][0] <ANKLE_ANGLE_MIN_DF)
			target_ankle_angle[i][0] = ANKLE_ANGLE_MIN_DF;
	}

	return;
}


void ankle_2dof_control_update_position(float pfdf, float inev, float kp, float ki, float kd)
{
		// recommend kp =30;

	ankle_2dof_control_update_target(pfdf, inev);
	my_control = CTRL_OPEN;

	static int32_t temp;
 	const int16_t  MAX_VOLTAGE = 11500;
	const int16_t BASE_VOLTAGE1 = 1525;
	const int16_t BASE_VOLTAGE2 = 1675;


	temp = (int32_t)(-(target_ankle_angle[0][0]- ank_angs_1[0])*kp);

	if(temp > MAX_VOLTAGE)
		my_pwm[0] = MAX_VOLTAGE;
	else if(temp <-MAX_VOLTAGE)
		my_pwm[0] = -MAX_VOLTAGE;
	else
	{
		if(temp>=0 && temp <BASE_VOLTAGE1)
			my_pwm[0] = 0;
		else if(temp<0 && temp >-BASE_VOLTAGE1)
			my_pwm[0] = 0;
		else
			my_pwm[0] = (int16_t)temp;
	}

	temp = (int32_t)(-(target_ankle_angle[1][0]- ank_angs_2[0])*kp*1.1);


	if(temp>MAX_VOLTAGE)
		my_pwm[1] =MAX_VOLTAGE;
	else if(temp <-MAX_VOLTAGE)
		my_pwm[1] = -MAX_VOLTAGE;
	else
	{
		if(temp>0 && temp <BASE_VOLTAGE2)
			my_pwm[1] = 0;
		else if(temp<0 && temp >-BASE_VOLTAGE2)
			my_pwm[1] = 0;
		else
			my_pwm[1] = (int16_t)temp;
	}


	return;
}

void ankle_2dof_control_update_impedance(float pfdf, float inev, float kp, float ki, float kd)
{
	// recommend kp =600;
	// const int16_t ANKLE_DAMPING = 0;
	// const int16_t ANKLE_CURRENT_BASE1 = 3500;
	// const int16_t ANKLE_CURRENT_BASE2 = 8000;

	const int16_t MAX_CURRENT = 32600;
	const int16_t BASE_CURRENT1 = 7500;
	const int16_t BASE_CURRENT2 = 12000;

	 // static int16_t  MAX_CURRENT = 300000;
	 my_control = CTRL_CURRENT;


	ankle_2dof_control_update_target(pfdf, inev);

	int32_t temp;

	temp = (int32_t)(-(target_ankle_angle[0][0]- ank_angs_1[0])*kp);

	if(temp >= 0 && temp<BASE_CURRENT1)
	{
		temp = 0;
	}
	else if(temp<0 && temp>-BASE_CURRENT1)
	{
		temp = 0;
	}

	if(temp>MAX_CURRENT)
		my_cur[0] = MAX_CURRENT;
	else if(temp <-MAX_CURRENT)
		my_cur[0] = -MAX_CURRENT;
	else
	{
		my_cur[0] = (int16_t)temp;
	}


	temp = (int32_t)(-(target_ankle_angle[1][0]- ank_angs_2[0])*kp*2.5);


	if(temp>=0 && temp < BASE_CURRENT2)
	{
		temp = 0;
	}
	else if(temp<0 && temp>-BASE_CURRENT2)
	{
		temp = 0;
	}


	if(temp>MAX_CURRENT)
		my_cur[1] =MAX_CURRENT;
	else if(temp <-MAX_CURRENT)
		my_cur[1] = -MAX_CURRENT;
	else
		my_cur[1] = (int16_t)temp;

	// if(temp>MAX_CURRENT)
	// 	temp =MAX_CURRENT;
	// else if(temp <-MAX_CURRENT)
	// 	temp = -MAX_CURRENT;
	//
	// set_ankle_torque_2(temp*10);

	return;
}

uint8_t ankle_2dof_control_demo_impedance_fsm(void)
{
	static uint32_t state_t = 0;
	static int8_t function_state = 0;

	state_t++;

	static float kp=600;

	switch(function_state)
	{
		case 0:
			ankle_2dof_control_update_impedance(0, 0, kp,0,0);
			break;

		case 1:
			ankle_2dof_control_update_impedance(1, 0, kp,0,0);
			break;

		case 2:
			ankle_2dof_control_update_impedance(-1, 0, kp,0,0);
			break;

		case 3:
			ankle_2dof_control_update_impedance(0, 0, kp,0,0);

			break;

		case 4:
			ankle_2dof_control_update_impedance(0, 1, kp,0,0);
			break;

		case 5:
			ankle_2dof_control_update_impedance(0, -1, kp,0,0);
			break;
	}

	if(state_t >1000)
	{
		function_state++;
		if(function_state >5)
			function_state = 0;

		state_t = -1;
	}

	return 0;
}

uint8_t ankle_2dof_control_demo_position_fsm(void)
{
	static uint32_t state_t = 0;
	static int8_t function_state = 0;

	state_t++;

	static float kp = 30;

	switch(function_state)
	{
		case 0:
		 	ankle_2dof_control_update_position(0, 0, kp,0,0);
			break;

		case 1:
			ankle_2dof_control_update_position(1, 0, kp,0,0);
			break;

		case 2:
			ankle_2dof_control_update_position(-1, 0, kp,0,0);
			break;

		case 3:
			ankle_2dof_control_update_position(0, 0, kp,0,0);
			break;

		case 4:
			ankle_2dof_control_update_position(0, 1, kp,0,0);
			break;

		case 5:
			ankle_2dof_control_update_position(0, -1, kp,0,0);
			break;
	}

	if(state_t >1000)
	{
		function_state++;
		if(function_state >5)
			function_state = 0;

		state_t = -1;
	}

	return 0;
}

#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
#endif //INCLUDE_UPROJ_MIT_A2DOF
