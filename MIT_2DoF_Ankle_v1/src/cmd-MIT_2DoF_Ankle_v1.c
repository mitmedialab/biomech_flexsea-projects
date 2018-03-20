/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/MIT_2DoF_Ankle' MIT Biomechatronics 2-dof Ankle
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
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd-MIT_2DoF_Ankle: Custom commands for this project
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_MIT_A2DOF

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <cmd-MIT_2DoF_Ankle_v1.h>
#include <stdio.h>
#include <stdlib.h>
#include <flexsea.h>
#include <flexsea_system.h>
#include <flexsea_cmd_user.h>

//Execute boards only:
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
#include "main.h"
#include "imu.h"
#include "strain.h"
#include "safety.h"
#include "analog.h"
#include "motor.h"
#include "control.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Function(s)
//****************************************************************************

void tx_cmd_ankle2dof_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_A2DOF;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = slave;

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Structure pointer. Points to exec1 by default.
		struct execute_s *exec_s_ptr = &exec1;

		//Assign data structure based on slave:
		if(slave == 0)
		{
			exec_s_ptr = &exec1;
		}
		else
		{
			exec_s_ptr = &exec2;
		}

		SPLIT_16((uint16_t)exec_s_ptr->gyro.x, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->gyro.y, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->gyro.z, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.x, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.y, shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->accel.z, shBuf, &index);

		SPLIT_16(exec_s_ptr->strain, shBuf, &index);
		SPLIT_16(exec_s_ptr->analog[0], shBuf, &index);
		SPLIT_16(exec_s_ptr->analog[1], shBuf, &index);

		SPLIT_32((uint32_t)(*exec_s_ptr->enc_ang), shBuf, &index);
		SPLIT_16((uint16_t)exec_s_ptr->current, shBuf, &index);

		shBuf[index++] = exec_s_ptr->volt_batt;
		shBuf[index++] = exec_s_ptr->volt_int;
		shBuf[index++] = exec_s_ptr->temp;
		shBuf[index++] = exec_s_ptr->status1;
		shBuf[index++] = exec_s_ptr->status2;

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		#ifdef USE_IMU
			SPLIT_16((uint16_t)imu.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)imu.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)imu.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)imu.accel.z, shBuf, &index);
		#else
			SPLIT_16((uint16_t)0, shBuf, &index);
			SPLIT_16((uint16_t)0, shBuf, &index);
			SPLIT_16((uint16_t)0, shBuf, &index);
			SPLIT_16((uint16_t)0, shBuf, &index);
			SPLIT_16((uint16_t)0, shBuf, &index);
			SPLIT_16((uint16_t)0, shBuf, &index);
		#endif

		SPLIT_16(strain_read(), shBuf, &index);
		SPLIT_16(read_analog(0), shBuf, &index);
		SPLIT_16(read_analog(1), shBuf, &index);

		SPLIT_32((uint32_t)(*exec1.enc_ang), shBuf, &index);
		SPLIT_16((uint16_t)ctrl.current.actual_val, shBuf, &index);

		shBuf[index++] = safety_cop.v_vb;
		shBuf[index++] = safety_cop.v_vg;
		shBuf[index++] = safety_cop.temperature;
		shBuf[index++] = safety_cop.status1;
		shBuf[index++] = safety_cop.status2;

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	//Payload length:
	(*len) = index;
}

void tx_cmd_ankle2dof_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t slave, uint8_t controller, \
							int16_t ctrl_i, int16_t ctrl_o)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_A2DOF;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = slave;
	shBuf[index++] = controller;
	SPLIT_16((uint16_t)ctrl_i, shBuf, &index);
	SPLIT_16((uint16_t)ctrl_o, shBuf, &index);

	//Payload length:
	(*len) = index;
}

void rx_cmd_ankle2dof_rw(uint8_t *buf, uint8_t *info)
{
	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		int16_t tmp_wanted_current = 0, tmp_open_spd = 0;
		uint8_t slave = 0;
		uint16_t index = P_DATA1;
		slave = buf[index++];

		//Update controller:
		control_strategy(buf[index++]);

		//Only change the setpoint if we are in current control mode:
		if(ctrl.active_ctrl == CTRL_CURRENT)
		{
			index = P_DATA1+2;
			tmp_wanted_current = (int16_t) REBUILD_UINT16(buf, &index);
			ctrl.current.setpoint_val = tmp_wanted_current;
		}
		else if(ctrl.active_ctrl == CTRL_OPEN)
		{
			index = P_DATA1+4;
			tmp_open_spd = (int16_t) REBUILD_UINT16(buf, &index);;
			motor_open_speed_1(tmp_open_spd);
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		uint8_t slave = 0;
		slave = buf[P_DATA1];

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Reply:
	tx_cmd_ankle2dof_w(TX_N_DEFAULT, buf[P_DATA1]);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_ankle2dof_rr(uint8_t *buf, uint8_t *info)
{
	(void)info;	//Unused for now

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

		uint8_t slave = 0;
		uint16_t index = 0;
		struct execute_s *exec_s_ptr = &exec1;

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE))

			slave = buf[P_XID];
			//Assign data structure based on slave:
			if(slave == FLEXSEA_EXECUTE_1)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_PLAN))

			slave = buf[P_DATA1];
			//Assign data structure based on slave:
			if(slave == 0)
			{
				exec_s_ptr = &exec1;
			}
			else
			{
				exec_s_ptr = &exec2;
			}

		#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE))

		#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

			index = P_DATA1+1;
			exec_s_ptr->gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->accel.z = (int16_t) REBUILD_UINT16(buf, &index);

			exec_s_ptr->strain = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->analog[0] = (int16_t) REBUILD_UINT16(buf, &index);
			exec_s_ptr->analog[1] = (int16_t) REBUILD_UINT16(buf, &index);
			*(exec_s_ptr->enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			exec_s_ptr->current = (int16_t) REBUILD_UINT16(buf, &index);

			exec_s_ptr->volt_batt = buf[index++];
			exec_s_ptr->volt_int = buf[index++];
			exec_s_ptr->temp = buf[index++];
			exec_s_ptr->status1 = buf[index++];
			exec_s_ptr->status2 = buf[index++];

		#endif	//BOARD_TYPE_FLEXSEA_PLAN

	#else

		(void)buf;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

#ifdef __cplusplus
}
#endif

#endif //INCLUDE_UPROJ_MIT_A2DOF
