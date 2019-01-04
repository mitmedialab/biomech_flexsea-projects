/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-manage' Mid-level computing, and networking
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>

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
	[This file] mn-MotorControl: Wrappers for motor control functions on Mn
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-05-22 | jfduval | Initial GPL-3.0 release
	*
****************************************************************************/

#ifdef BOARD_TYPE_FLEXSEA_MANAGE

//****************************************************************************
// Include(s)
//****************************************************************************

#include "mn-MotorControl.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

struct ctrl_s ctrl[2];
writeEx_s writeEx[2];

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************


//****************************************************************************
// Public Function(s)
//****************************************************************************

void initWriteEx(uint8_t ch)
{
	writeEx[ch].ctrl = CTRL_NONE;
	writeEx[ch].setpoint = 0;
	writeEx[ch].setGains = KEEP;
	writeEx[ch].offset = 0;
	writeEx[ch].g[0] = 0;
	writeEx[ch].g[1] = 0;
	writeEx[ch].g[2] = 0;
	writeEx[ch].g[3] = 0;
}

void init_current_controller(uint8_t ch)
{
	ctrl[ch].active_ctrl = CTRL_CURRENT;
	ctrl[ch].current.gain.g0 = CTRL_I_KP;
	ctrl[ch].current.gain.g1 = CTRL_I_KI;
	ctrl[ch].current.setpoint_val = 0;

	//Prep for comm:
	writeEx[ch].ctrl = CTRL_CURRENT;
	writeEx[ch].g[0] = CTRL_I_KP;
	writeEx[ch].g[1] = CTRL_I_KI;
	writeEx[ch].setpoint = 0;
	writeEx[ch].setGains = CHANGE;
}

void init_position_controller(uint8_t ch)
{
	ctrl[ch].active_ctrl = CTRL_POSITION;
	ctrl[ch].position.gain.g0 = CTRL_P_KP;
	ctrl[ch].position.gain.g1 = 0;
	ctrl[ch].position.gain.g2 = 0;
	ctrl[ch].position.setp = *rigid1.ex.enc_ang;

	//Prep for comm:
	writeEx[ch].ctrl = CTRL_POSITION;
	writeEx[ch].g[0] = CTRL_P_KP;
	writeEx[ch].g[1] = 0;
	writeEx[ch].g[2] = 0;
	writeEx[ch].setpoint = *rigid1.ex.enc_ang;
	writeEx[ch].setGains = CHANGE;
}

void setMotorVoltage(int32_t v, uint8_t ch)
{
	if(writeEx[ch].ctrl == CTRL_OPEN)
	{
		writeEx[ch].setpoint = v;
	}
}

void setMotorCurrent(int32_t i, uint8_t ch)
{
	ctrl[ch].current.setpoint_val =  i;

	//Prep for comm:
	if(writeEx[ch].ctrl == CTRL_CURRENT)
	{
		writeEx[ch].setpoint = i;
	}
}

void setMotorPosition(int32_t i, uint8_t ch)
{
	if(writeEx[ch].ctrl == CTRL_POSITION)
	{
		writeEx[ch].setpoint = i;
		ctrl[ch].position.setp =  i;
	}
	else if(writeEx[ch].ctrl == CTRL_IMPEDANCE)
	{
		writeEx[ch].setpoint = i;
		ctrl[ch].impedance.setpoint_val = i;
	}
}

void setControlMode(uint8_t m, uint8_t ch)
{
	ctrl[ch].active_ctrl = m;
	writeEx[ch].ctrl = m;
}

void setControlGains(int16_t g0, int16_t g1, int16_t g2, int16_t g3, uint8_t ch)
{
	writeEx[ch].g[0] = g0;
	writeEx[ch].g[1] = g1;
	writeEx[ch].g[2] = g2;
	writeEx[ch].g[3] = g3;
	writeEx[ch].setGains = CHANGE;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************



#endif 	//BOARD_TYPE_FLEXSEA_MANAGE
