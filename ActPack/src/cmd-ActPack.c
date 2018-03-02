/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'user/ActPack' Dephy's Actuator Package (ActPack)
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd-ActPack: Custom commands for the Actuator package
****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-09-27 | jfduval | Initial release
	*
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef INCLUDE_UPROJ_ACTPACK

//****************************************************************************
// Include(s)
//****************************************************************************

#include "flexsea.h"
#include "flexsea_comm.h"
#include "flexsea_system.h"
#include "flexsea_cmd_user.h"
#include "flexsea_global_structs.h"
#include "flexsea_user_structs.h"
#include "cmd-ActPack.h"

//Execute boards only:
#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
#include "main.h"
#include "motor.h"
#include "control.h"
#include "safety.h"
#include "strain.h"
#include "analog.h"
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

//Manage boards only:
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn-ActPack.h"
#endif	//BOARD_TYPE_FLEXSEA_MANAGE


//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t newActPackRRpacketAvailableFlag = 0;
uint8_t ActPackSys = 0;

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//Poll to see if new data is available:
uint8_t newActPackRRpacketAvailable(void)
{
	uint8_t retVal = newActPackRRpacketAvailableFlag;
	newActPackRRpacketAvailableFlag = 0;
	return retVal;
}

//Get a copy of the latest Rigid values
//void getLastRigidData(struct rigid_s *r){(*r) = rigid1;}

//****************************************************************************
// Private Function Prototype(s)
//****************************************************************************

#if (defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)
static void rx_cmd_actpack_Action1(uint8_t controller, int32_t setpoint, \
									uint8_t setGains, int16_t g0, int16_t g1, \
									int16_t g2, int16_t g3, uint8_t system);
#endif //BOARD_TYPE_FLEXSEA_EXECUTE

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Command: CMD_ACTPACK. Type: R/W.
//setGains: KEEP/CHANGE
void tx_cmd_actpack_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
						uint16_t *len, uint8_t offset, uint8_t controller, \
						int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
						int16_t g2, int16_t g3, uint8_t system)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_ACTPACK;
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
	shBuf[index++] = system;

	//Payload length:
	(*len) = index;
}

//Pack tx_cmd_actpack_rw()
void ptx_cmd_actpack_rw(uint8_t slaveId, uint16_t *numb, uint8_t *commStr, \
						uint8_t offset, uint8_t controller, \
						int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
						int16_t g2, int16_t g3, uint8_t system)
{
	tx_cmd_actpack_rw(TX_N_DEFAULT, offset, controller, setpoint, setGains, \
						g0, g1,	g2, g3, system);
	pack(P_AND_S_DEFAULT, slaveId, NULL, numb, commStr);
}

//Command: CMD_RICNU. Type: R.
//setGains: KEEP/CHANGE
void tx_cmd_actpack_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_ACTPACK;
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
	shBuf[index++] = 0;

	//Payload length:
	(*len) = index;
}

//Only used for replies
void tx_cmd_actpack_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
							uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_ACTPACK;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		struct rigid_s *ri = &rigid1;

		SPLIT_32((uint32_t)*(ri->ex.enc_ang), shBuf, &index);
		SPLIT_32((uint32_t)(*ri->ex.enc_ang_vel), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex.joint_ang), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex.joint_ang_vel), shBuf, &index);
		SPLIT_32((uint32_t)ri->ex.mot_current, shBuf, &index);
		SPLIT_32((uint32_t)ri->ex.mot_acc, shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
		SPLIT_16((uint16_t)(ctrl.current.setpoint_val >> 3), shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.status, shBuf, &index);
		//(28 bytes)

	#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid1;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//Arguments:
		if(offset == 0)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang), shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
			SPLIT_16((uint16_t)*(encoder), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_current >> 3), shBuf, &index);
			shBuf[index++] = rigid1.re.temp;
			//(27 bytes)
		}
		else if(offset == 1)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang_vel), shBuf, &index);
			SPLIT_16((uint16_t)*(encoderVel), shBuf, &index);
			SPLIT_16(rigid1.re.vb, shBuf, &index);
			SPLIT_16((uint16_t)rigid1.re.current, shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[0]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[1]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[2]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[3]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[4]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[5]), shBuf, &index);
			//(26 bytes)
		}
		else if(offset == 2)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)ri->ex.mot_acc, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.ctrl.current.setpoint_val >> 3), shBuf, &index);
			SPLIT_16(rigid1.re.status, shBuf, &index);
			SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[6]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[7]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[8]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[9]), shBuf, &index);
			//(22 bytes)
		}
		else if(offset == 3)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16(rigid1.re.vg, shBuf, &index);
			SPLIT_16(rigid1.re.v5, shBuf, &index);
			SPLIT_16((ri->mn.analog[0]), shBuf, &index);
			SPLIT_16((ri->mn.analog[1]), shBuf, &index);
			SPLIT_16((ri->mn.analog[2]), shBuf, &index);
			SPLIT_16((ri->mn.analog[3]), shBuf, &index);
			//(16 bytes)
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE
	
	//Payload length:
	(*len) = index;
}

void rx_cmd_actpack_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	(void)info;
	
	//Temporary variables
	uint8_t offset = 0;
	uint8_t tmpController = 0, tmpSetGains = 0, tmpSystem = 0;
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
	tmpSystem = buf[index++];

	//An offset >= 100 means a pure Read, with no writing (not a RW)
	if(offset < 100)
	{
		#if(defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)

			//Act on the decoded data:
			rx_cmd_actpack_Action1(tmpController, tmpSetpoint, tmpSetGains, tmpGain[0],
									tmpGain[1], tmpGain[2], tmpGain[3], tmpSystem);

		#else

			(void)tmpController;
			(void)tmpSetGains;
			(void)tmpSetpoint;
			(void)tmpGain;
			(void)tmpSystem;

		#endif //(defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)
	}
	else
	{
		offset -= 100;
	}

	//Reply:
	tx_cmd_actpack_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

void rx_cmd_actpack_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;
	(void)info;	//Unused for now

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		struct rigid_s *ri = &rigid1;
		(void)index;
		(void)buf;
		(void)offset;

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		struct rigid_s *ri = &rigid1;
		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			*(ri->ex.enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex.enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex.mot_current = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex.mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex.mot_volt = (int32_t) ((int16_t)REBUILD_UINT16(buf, &index) << 3);
			ri->ex.ctrl.current.setpoint_val = (int32_t) ((int16_t)REBUILD_UINT16(buf, &index) << 3);
			ri->ex.strain = REBUILD_UINT16(buf, &index);
			ri->ex.status = REBUILD_UINT16(buf, &index);
		}

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_PLAN

		struct rigid_s *ri = &rigid1;
		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			*(ri->ex.enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			ri->mn.gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex.mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex.mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->re.temp = buf[index++];
			//(27 bytes)
		}
		else if(offset == 1)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			*(ri->ex.enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->re.vb = REBUILD_UINT16(buf, &index);
			ri->re.current = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[0] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[1] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[2] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[3] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[4] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[5] = (int16_t)REBUILD_UINT16(buf, &index);
			//(26 bytes)

			//In some cases genVar contains Strain data:
			strain1.ch[0].strain_filtered = ri->mn.genVar[0];
			strain1.ch[1].strain_filtered = ri->mn.genVar[1];
			strain1.ch[2].strain_filtered = ri->mn.genVar[2];
			strain1.ch[3].strain_filtered = ri->mn.genVar[3];
			strain1.ch[4].strain_filtered = ri->mn.genVar[4];
			strain1.ch[5].strain_filtered = ri->mn.genVar[5];
			strain1.preDecoded = 1;
		}
		else if(offset == 2)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->ex.mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex.ctrl.current.setpoint_val = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->re.status = REBUILD_UINT16(buf, &index);
			ri->ex.strain = REBUILD_UINT16(buf, &index);
			ri->mn.genVar[6] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[7] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[8] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[9] = (int16_t)REBUILD_UINT16(buf, &index);
			//(22 bytes)
		}
		else if(offset == 3)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->re.vg = REBUILD_UINT16(buf, &index);
			ri->re.v5 = REBUILD_UINT16(buf, &index);
			ri->mn.analog[0] = REBUILD_UINT16(buf, &index);
			ri->mn.analog[1] = REBUILD_UINT16(buf, &index);
			ri->mn.analog[2] = REBUILD_UINT16(buf, &index);
			ri->mn.analog[3] = REBUILD_UINT16(buf, &index);
			//(16 bytes)
		}
		else
		{
			//...
		}

	#endif	//BOARD_TYPE_FLEXSEA_PLAN

	newActPackRRpacketAvailableFlag = 1;
	ri->lastOffsetDecoded = offset;
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
//Command = rx_cmd_actpack, section = READ
void rx_cmd_actpack_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3, uint8_t system)
{
	(void) system;

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
		setMotorVoltage(setpoint);
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
}
#endif	//BOARD_TYPE_FLEXSEA_EXECUTE

#ifdef BOARD_TYPE_FLEXSEA_MANAGE
//Command = rx_cmd_actpack, section = READ
void rx_cmd_actpack_Action1(uint8_t controller, int32_t setpoint, uint8_t setGains,
						int16_t g0,	int16_t g1,	int16_t g2, int16_t g3, uint8_t system)
{
	//Update controller (if needed):
	setControlMode(controller);

	//Only change the setpoint if we are in current control mode:
	if(ctrl.active_ctrl == CTRL_CURRENT)
	{
		ctrl.current.setpoint_val = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.current.gain.g0 = g0;
			ctrl.current.gain.g1 = g1;
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3);
		}
		//Copy to writeEx:
		setMotorCurrent(setpoint);
	}
	else if(ctrl.active_ctrl == CTRL_OPEN)
	{
		setMotorVoltage(setpoint);
	}
	else if(ctrl.active_ctrl == CTRL_POSITION)
	{
		ctrl.position.setp = setpoint;
		if (setGains == CHANGE)
		{
			ctrl.position.gain.g0 = g0;
			ctrl.position.gain.g1 = g1;
			ctrl.position.gain.g2 = g2;
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3);
		}
		//Copy to writeEx:
		setMotorPosition(setpoint);
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
			//Copy to writeEx:
			setControlGains(g0, g1, g2, g3);
		}
		//Copy to writeEx:
		setMotorPosition(setpoint);
	}

	ActPackSys = system;
}
#endif	//BOARD_TYPE_FLEXSEA_MANAGE

#ifdef __cplusplus
}
#endif

#endif 	//INCLUDE_UPROJ_ACTPACK
