/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/RICNU_Knee_v1' RIC/NU Knee
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
	[This file] cmd-RICNU_Knee_v1: Custom commands for this project
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-10-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef INCLUDE_UPROJ_RICNU_KNEE_V1

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <flexsea.h>
#include <flexsea_system.h>
#include "../inc/cmd-RICNU_Knee_v1.h"

//Execute boards only:
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
#include "imu.h"
#include "strain.h"
#include "safety.h"
#include "analog.h"
#include "motor.h"
#include "control.h"
#include "mag_encoders.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#endif	//BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Function(s)
//****************************************************************************

//Command: CMD_RICNU. Type: R/W.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType,uint16_t *len,\
					uint8_t offset, uint8_t controller, \
					int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
					int16_t g2, int16_t g3)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = offset;
	shBuf[index++] = controller;
	SPLIT_32(setpoint, shBuf, &index);
	shBuf[index++] = setGains;
	SPLIT_16(g0, shBuf, &index);
	SPLIT_16(g1, shBuf, &index);
	SPLIT_16(g2, shBuf, &index);
	SPLIT_16(g3, shBuf, &index);

	//Payload length:
	(*len) = index;
}

//Command: CMD_RICNU. Type: R.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = 100 + offset;
	//An offset >= 100 means a pure Read, with no writing (not a RW)
	//Fill the other fields with zeros to avoid problems:
	shBuf[index++] = 0;
	SPLIT_32(0, shBuf, &index);
	shBuf[index++] = 0;
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);

	//Payload length:
	(*len) = index;
}

//Command: CMD_RICNU. Type: W.
//setGains: KEEP/CHANGE
void tx_cmd_ricnu_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_RICNU;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		//Arguments:
		if(offset == 0)
		{
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
			SPLIT_32((uint32_t)(*exec1.enc_ang), shBuf, &index);
			SPLIT_32((uint32_t)as5048b.ang_abs_clks, shBuf, &index);
			SPLIT_16((uint16_t)ctrl.current.actual_val, shBuf, &index);
			SPLIT_16((uint16_t)exec1.sine_commut_pwm, shBuf, &index);
			//(24 bytes)
		}
		else if(offset == 1)
		{
			//Compressed Strain:

			shBuf[index++] = strain1.compressedBytes[0];
			shBuf[index++] = strain1.compressedBytes[1];
			shBuf[index++] = strain1.compressedBytes[2];
			shBuf[index++] = strain1.compressedBytes[3];
			shBuf[index++] = strain1.compressedBytes[4];
			shBuf[index++] = strain1.compressedBytes[5];
			shBuf[index++] = strain1.compressedBytes[6];
			shBuf[index++] = strain1.compressedBytes[7];
			shBuf[index++] = strain1.compressedBytes[8];

			//Padding the Batt board variables with zeros. Not sure it's needed,
			//but won't hurt
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
			shBuf[index++] = 0;
		}
		else if(offset == 2)
		{
			//User variables:
			SPLIT_16((uint16_t)ricnu_1.gen_var[0], shBuf, &index);
			SPLIT_16((uint16_t)ricnu_1.gen_var[1], shBuf, &index);
			SPLIT_16((uint16_t)ricnu_1.gen_var[2], shBuf, &index);
			SPLIT_16((uint16_t)ricnu_1.gen_var[3], shBuf, &index);
			SPLIT_16((uint16_t)ricnu_1.gen_var[4], shBuf, &index);
			SPLIT_16((uint16_t)ricnu_1.gen_var[5], shBuf, &index);
			//(12 bytes, we could add more variables here)
		}
		else
		{
			//Deal with other offsets here...
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE


	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//struct execute_s *ex = &exec1;
		struct ricnu_s *rn = &ricnu_1;

		//Arguments:
		if(offset == 0)
		{
			#ifndef BOARD_SUBTYPE_RIGID
				SPLIT_16((uint16_t)rn->ex->gyro.x, shBuf, &index);
				SPLIT_16((uint16_t)rn->ex->gyro.y, shBuf, &index);
				SPLIT_16((uint16_t)rn->ex->gyro.z, shBuf, &index);
				SPLIT_16((uint16_t)rn->ex->accel.x, shBuf, &index);
				SPLIT_16((uint16_t)rn->ex->accel.y, shBuf, &index);
				SPLIT_16((uint16_t)rn->ex->accel.z, shBuf, &index);
			#else
				//When using Rigid we use Mn's IMU:
				struct rigid_s *ri = &rigid1;
				SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
				SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
				SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
				SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
				SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
				SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
			#endif
		}
		else if(offset == 1)
		{
			//Compressed Strain:
			shBuf[index++] = rn->st->compressedBytes[0];
			shBuf[index++] = rn->st->compressedBytes[1];
			shBuf[index++] = rn->st->compressedBytes[2];
			shBuf[index++] = rn->st->compressedBytes[3];
			shBuf[index++] = rn->st->compressedBytes[4];
			shBuf[index++] = rn->st->compressedBytes[5];
			shBuf[index++] = rn->st->compressedBytes[6];
			shBuf[index++] = rn->st->compressedBytes[7];
			shBuf[index++] = rn->st->compressedBytes[8];

			//Battery board?
			#ifdef USE_BATTBOARD

				shBuf[index++] = batt1.rawBytes[0];
				shBuf[index++] = batt1.rawBytes[1];
				shBuf[index++] = batt1.rawBytes[2];
				shBuf[index++] = batt1.rawBytes[3];
				shBuf[index++] = batt1.rawBytes[4];
				shBuf[index++] = batt1.rawBytes[5];
				shBuf[index++] = batt1.rawBytes[6];
				shBuf[index++] = batt1.rawBytes[7];

			#else

				//Pad with zeros:
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;
				shBuf[index++] = 0;

			#endif
			//(17 bytes)
		}
		else if(offset == 2)
		{
			//User variables:
			SPLIT_16((uint16_t)rn->gen_var[0], shBuf, &index);
			SPLIT_16((uint16_t)rn->gen_var[1], shBuf, &index);
			SPLIT_16((uint16_t)rn->gen_var[2], shBuf, &index);
			SPLIT_16((uint16_t)rn->gen_var[3], shBuf, &index);
			SPLIT_16((uint16_t)rn->gen_var[4], shBuf, &index);
			SPLIT_16((uint16_t)rn->gen_var[5], shBuf, &index);
			//(12 bytes, we could add more variables here)
		}
		else
		{
			//Deal with other offsets here...
		}

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	//Payload length:
	(*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_ricnu_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	(void)info;

	//Temporary variables
	uint8_t offset = 0;
	uint8_t tmpController = 0, tmpSetGains = 0;
	int32_t tmpSetpoint = 0;
	int16_t tmpGain[4] = {0,0,0,0};

	//Decode data received:
	index = P_DATA1;
	offset = buf[index++];
	tmpController = buf[index++];
	tmpSetpoint = (int32_t)REBUILD_UINT32(buf, &index);
	tmpSetGains = buf[index++];
	tmpGain[0] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[1] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[2] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain[3] = (int16_t)REBUILD_UINT16(buf, &index);

	//An offset >= 100 means a pure Read, with no writing (not a RW)
	if(offset < 100)
	{
		#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

			//Act on the decoded data:
			rx_cmd_ricnu_Action1(tmpController, tmpSetpoint, tmpSetGains, tmpGain[0],
									tmpGain[1], tmpGain[2], tmpGain[3]);

		#else

			(void)tmpController;
			(void)tmpSetGains;
			(void)tmpSetpoint;
			(void)tmpGain;

		#endif
	}
	else
	{
		offset -= 100;
	}

	//Reply:
	tx_cmd_ricnu_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_ricnu_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;

	(void)info;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
		(void)buf;
		(void)index;
		(void)offset;
		flexsea_error(SE_CMD_NOT_PROGRAMMED);
		return;
	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#if((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))

		struct execute_s *ex = &exec1;
		struct ricnu_s *rn = &ricnu_1;

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			//Typical Execute variables, + new encoders:
			rn->ex->gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			rn->enc_motor = (int32_t) REBUILD_UINT32(buf, &index);
			rn->enc_joint = (int32_t) REBUILD_UINT32(buf, &index);
			rn->ex->current = (int16_t) REBUILD_UINT16(buf, &index);
			rn->ex->sine_commut_pwm = (int16_t) REBUILD_UINT16(buf, &index);
		}
		else if(offset == 1)
		{
			//Strain amplifier:
			rn->st->compressedBytes[0] = buf[index++];
			rn->st->compressedBytes[1] = buf[index++];
			rn->st->compressedBytes[2] = buf[index++];
			rn->st->compressedBytes[3] = buf[index++];
			rn->st->compressedBytes[4] = buf[index++];
			rn->st->compressedBytes[5] = buf[index++];
			rn->st->compressedBytes[6] = buf[index++];
			rn->st->compressedBytes[7] = buf[index++];
			rn->st->compressedBytes[8] = buf[index++];

			#ifdef BOARD_TYPE_FLEXSEA_PLAN
				//Battery board:
				batt1.rawBytes[0] = buf[index++];
				batt1.rawBytes[1] = buf[index++];
				batt1.rawBytes[2] = buf[index++];
				batt1.rawBytes[3] = buf[index++];
				batt1.rawBytes[4] = buf[index++];
				batt1.rawBytes[5] = buf[index++];
				batt1.rawBytes[6] = buf[index++];
				batt1.rawBytes[7] = buf[index++];
			#endif
		}
		else if(offset == 2)
		{
			//User variables:
			rn->gen_var[0] = (int16_t) REBUILD_UINT16(buf, &index);
			rn->gen_var[1] = (int16_t) REBUILD_UINT16(buf, &index);
			rn->gen_var[2] = (int16_t) REBUILD_UINT16(buf, &index);
			rn->gen_var[3] = (int16_t) REBUILD_UINT16(buf, &index);
			rn->gen_var[4] = (int16_t) REBUILD_UINT16(buf, &index);
			rn->gen_var[5] = (int16_t) REBUILD_UINT16(buf, &index);
		}
		else
		{
			//...
		}

		//Copy RICNU to Exec (that way we refresh the Execute window):
		ex = rn->ex;
		*(ex->enc_ang) = rn->enc_motor;

	#endif	//((defined BOARD_TYPE_FLEXSEA_MANAGE) || (defined BOARD_TYPE_FLEXSEA_PLAN))
}

//Gets called when our Master Writes to us
void rx_cmd_ricnu_w(uint8_t *buf, uint8_t *info)
{
	//Master Write isn't implemented for this command.

	(void)buf;
	(void)info;
	flexsea_error(SE_CMD_NOT_PROGRAMMED);
}

//Command = rx_cmd_ricnu, section = READ
void rx_cmd_ricnu_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3)
{
	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

	//Update controller (if needed):
	control_strategy(controller);

	//Only change the setpoint if we are in current control mode:
	if(ctrl.active_ctrl == CTRL_CURRENT)
	{
		ctrl.current.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.current.gain.g0 = g0;
			ctrl.current.gain.g1 = g1;
		}
	}
	else if(ctrl.active_ctrl == CTRL_OPEN)
	{
		motor_open_speed_1(setpoint);
	}
	else if(ctrl.active_ctrl == CTRL_POSITION)
	{
		ctrl.position.setp = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.position.gain.g0 = g0;
			ctrl.position.gain.g1 = g1;
			ctrl.position.gain.g2 = g2;
		}
	}
	else if (ctrl.active_ctrl == CTRL_IMPEDANCE)
	{
		ctrl.impedance.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.impedance.gain.g0 = g0;
			ctrl.impedance.gain.g1 = g1;
			ctrl.current.gain.g0 = g2;
			ctrl.current.gain.g1 = g3;
		}
	}

	#else

		//Unused - deal with variables to avoid warnings
		(void) controller;
		(void)setpoint;
		(void)setGains;
		(void)g0;
		(void)g1;
		(void)g2;
		(void)g3;

	#endif
}

#ifdef __cplusplus
}
#endif

#endif //INCLUDE_UPROJ_RICNU_KNEE_V1
