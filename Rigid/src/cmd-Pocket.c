/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] flexsea-projects' User projects
	Copyright (C) 2018 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developer] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd-Pocket: Pocket Commands
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2018-02-28 | jfduval | New code
****************************************************************************/

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
#include "../inc/cmd-Pocket.h"
#include "flexsea_user_structs.h"
#include "user-mn.h"
#include "cmd-ActPack.h"
#include "flexsea_cmd_user.h"

#ifdef DEPHY
#include "dephy-mn.h"
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t newPocketRRpacketAvailableFlag = 0;
//Pocket1
int32_t cpo_enc_ang = 0, cpo_enc_ang_vel = 0;
int16_t cpo_joint_ang = 0, cpo_joint_ang_vel = 0;
int16_t cpo_ctrl_ank_ang_deg = 0, cpo_ctrl_ank_vel = 0;
int16_t cpo_joint_ang_from_mot = 0, cpo_ctrl_ank_ang_from_mot = 0;
//Pocket2
int32_t cpo_enc_ang2 = 0, cpo_enc_ang_vel2 = 0;
int16_t cpo_joint_ang2 = 0, cpo_joint_ang_vel2 = 0;
int16_t cpo_ctrl_ank_ang_deg2 = 0, cpo_ctrl_ank_vel2 = 0;
int16_t cpo_joint_ang_from_mot2 = 0, cpo_ctrl_ank_ang_from_mot2 = 0;

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//Poll to see if new data is available:
uint8_t newPocketRRpacketAvailable(void)
{
	uint8_t retVal = newPocketRRpacketAvailableFlag;
	newPocketRRpacketAvailableFlag = 0;
	return retVal;
}

//Get a copy of the latest Pocket values
void getLastPocketData(struct pocket_s *r){(*r) = pocket1;}

//Initialize pointers
void init_pocket(void)
{
	//Pocket1
	pocket1.ex[0].enc_ang = &cpo_enc_ang;
	pocket1.ex[0].enc_ang_vel = &cpo_enc_ang_vel;
	pocket1.ex[0].joint_ang = &cpo_joint_ang;
	pocket1.ex[0].joint_ang_vel = &cpo_joint_ang_vel;
	pocket1.ex[0].joint_ang_from_mot = &cpo_joint_ang_from_mot;

	pocket1.ex[1].enc_ang = &cpo_enc_ang2;
	pocket1.ex[1].enc_ang_vel = &cpo_enc_ang_vel2;
	pocket1.ex[1].joint_ang = &cpo_joint_ang2;
	pocket1.ex[1].joint_ang_vel = &cpo_joint_ang_vel2;
	pocket1.ex[1].joint_ang_from_mot = &cpo_joint_ang_from_mot2;
}

//****************************************************************************
// RX/TX Function(s)
//****************************************************************************

void tx_cmd_pocket_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_READ_ALL_POCKET;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = 100 + offset;
	//An offset >= 100 means a pure Read, with no writing (not a RW)

	shBuf[index++] = 0;
	SPLIT_32(0, shBuf, &index);
	shBuf[index++] = 0;
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);
	SPLIT_16(0, shBuf, &index);

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

//Pack tx_cmd_pocket_r()
void ptx_cmd_pocket_r(uint8_t slaveId, uint16_t *numb, uint8_t *commStr, \
							uint8_t offset)
{
	tx_cmd_pocket_r(TX_N_DEFAULT, offset);
	pack(P_AND_S_DEFAULT, slaveId, NULL, numb, commStr);
}

//Command: CMD_POCKET. Type: R/W.
//setGains: KEEP/CHANGE
void tx_cmd_pocket_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
						uint16_t *len, uint8_t offset, uint8_t controller, \
						int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
						int16_t g2, int16_t g3, uint8_t controllerB, \
						int32_t setpointB, uint8_t setGainsB, int16_t g0B, int16_t g1B,\
						int16_t g2B, int16_t g3B, uint8_t system)
{
	//Variable(s) & command:
	uint16_t index = 0;
	(*cmd) = CMD_READ_ALL_POCKET;
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

	shBuf[index++] = controllerB;
	SPLIT_32(setpointB, shBuf, &index);
	shBuf[index++] = setGainsB;
	SPLIT_16(g0B, shBuf, &index);
	SPLIT_16(g1B, shBuf, &index);
	SPLIT_16(g2B, shBuf, &index);
	SPLIT_16(g3B, shBuf, &index);

	shBuf[index++] = system;

	//Payload length:
	(*len) = index;
}

//Pack tx_cmd_pocket_rw()
void ptx_cmd_pocket_rw(uint8_t slaveId, uint16_t *numb, uint8_t *commStr, \
							uint8_t offset, uint8_t controller, \
							int32_t setpoint, uint8_t setGains, int16_t g0, int16_t g1,\
							int16_t g2, int16_t g3, uint8_t controllerB, \
							int32_t setpointB, uint8_t setGainsB, int16_t g0B, int16_t g1B,\
							int16_t g2B, int16_t g3B, uint8_t system)
{
	tx_cmd_pocket_rw(TX_N_DEFAULT, offset, controller, setpoint, setGains, g0, \
			g1, g2, g3, controllerB, setpointB, setGainsB, g0B, g1B, g2B, \
			g3B, system);
	pack(P_AND_S_DEFAULT, slaveId, NULL, numb, commStr);
}

void tx_cmd_pocket_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_READ_ALL_POCKET;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE

		struct pocket_s *ri = &pocket1;
		//pocket1.ex[0] = rigid1.ex;		

		SPLIT_32((uint32_t)*(ri->ex[offset].enc_ang), shBuf, &index);
		SPLIT_32((uint32_t)(*ri->ex[offset].enc_ang_vel), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex[offset].joint_ang), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ex[offset].joint_ang_vel), shBuf, &index);
		SPLIT_32((uint32_t)ri->ex[offset].mot_current, shBuf, &index);
		SPLIT_32((uint32_t)ri->ex[offset].mot_acc, shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex[offset].mot_volt >> 3), shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex[offset].ctrl.current.setpoint_val >> 3), shBuf, &index);
		SPLIT_16((uint16_t)ri->ex[offset].strain, shBuf, &index);
		SPLIT_16((uint16_t)ri->ex[offset].status, shBuf, &index);
		//(28 bytes)

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct pocket_s *ri = &pocket1;
		//Copy from rigid where needed:
		pocket1.mn = rigid1.mn;
		pocket1.re = rigid1.re;

		//Arguments:
		if(offset == 0)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);

			SPLIT_16((ri->mn.analog[0]), shBuf, &index);
			SPLIT_16((ri->mn.analog[1]), shBuf, &index);

			SPLIT_16(pocket1.re.vb, shBuf, &index);
			SPLIT_16(pocket1.re.v5, shBuf, &index);
			SPLIT_16((uint16_t)pocket1.re.current, shBuf, &index);
			SPLIT_16(pocket1.re.status, shBuf, &index);
			shBuf[index++] = pocket1.re.temp;

			//(29 bytes)
		}
		else if(offset == 1)	//Left Execute
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex[0].enc_ang), shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex[0].enc_ang_vel), shBuf, &index);
			SPLIT_32((uint32_t)ri->ex[0].mot_acc, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex[0].mot_current >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex[0].mot_volt >> 3), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ex[0].joint_ang), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ex[0].joint_ang_vel), shBuf, &index);
			SPLIT_16((uint16_t)ri->ex[0].strain, shBuf, &index);

			//(26 bytes)
		}
		else if(offset == 2)	//Right Execute
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex[1].enc_ang), shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex[1].enc_ang_vel), shBuf, &index);
			SPLIT_32((uint32_t)ri->ex[1].mot_acc, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex[1].mot_current >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex[1].mot_volt >> 3), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ex[1].joint_ang), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ex[1].joint_ang_vel), shBuf, &index);
			SPLIT_16((uint16_t)ri->ex[1].strain, shBuf, &index);

			//(26 bytes)
		}
		else if(offset == 3)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[0]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[1]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[2]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[3]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[4]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[5]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[6]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[7]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[8]), shBuf, &index);
			SPLIT_16((uint16_t)(ri->mn.genVar[9]), shBuf, &index);
			//( bytes)
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Payload length:
	(*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_pocket_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	(void)info;

	//Temporary variables
	uint8_t offset = 0;
	uint8_t tmpController = 0, tmpSetGains = 0, tmpSystem = 0;
	int32_t tmpSetpoint = 0;
	int16_t tmpGain[4] = {0,0,0,0};
	uint8_t tmpController2 = 0, tmpSetGains2 = 0;
	int32_t tmpSetpoint2 = 0;
	int16_t tmpGain2[4] = {0,0,0,0};

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
	
	tmpController2 = buf[index++];
	tmpSetpoint2 = (int32_t)REBUILD_UINT32(buf, &index);
	tmpSetGains2 = buf[index++];
	tmpGain2[0] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain2[1] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain2[2] = (int16_t)REBUILD_UINT16(buf, &index);
	tmpGain2[3] = (int16_t)REBUILD_UINT16(buf, &index);
	
	tmpSystem = buf[index++];
	
	if(offset < 100)
	{
		#if (defined BOARD_TYPE_FLEXSEA_EXECUTE || defined BOARD_TYPE_FLEXSEA_MANAGE)
		#if (defined CO_ENABLE_ACTPACK)
		//Act on the decoded data:
		rx_cmd_actpack_Action1(tmpController, tmpSetpoint, tmpSetGains, tmpGain[0],
										tmpGain[1], tmpGain[2], tmpGain[3], tmpSystem, 0);
		rx_cmd_actpack_Action1(tmpController2, tmpSetpoint2, tmpSetGains2, tmpGain2[0],
										tmpGain2[1], tmpGain2[2], tmpGain2[3], tmpSystem, 1);
		#else
		(void)tmpController; (void)tmpSetpoint; (void)tmpSetGains; 
		(void)tmpGain; (void) tmpSystem;
		(void)tmpController2; (void)tmpSetpoint2; (void)tmpSetGains2; 
		(void)tmpGain2;
		#endif
		#endif
	}
	else
	{
		offset -= 100;
	}

	//Reply:
	tx_cmd_pocket_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_pocket_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;
	struct pocket_s *ri = &pocket1;
	(void)info;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
		(void)buf;
	#endif

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0 || offset == 1)
		{
			*(ri->ex[offset].enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex[offset].enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex[offset].joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex[offset].joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex[offset].mot_current = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[offset].mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[offset].mot_volt = (int32_t) ((int16_t)REBUILD_UINT16(buf, &index) << 3);
			ri->ex[offset].ctrl.current.setpoint_val = (int32_t) ((int16_t)REBUILD_UINT16(buf, &index) << 3);
			ri->ex[offset].strain = REBUILD_UINT16(buf, &index);
			ri->ex[offset].status = REBUILD_UINT16(buf, &index);
			//(28 bytes)
		}

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_PLAN

		//pocketPtrXid(&ri, buf[P_XID]);

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->mn.gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.analog[0] = REBUILD_UINT16(buf, &index);
			ri->mn.analog[1] = REBUILD_UINT16(buf, &index);
			ri->re.vb = REBUILD_UINT16(buf, &index);
			ri->re.v5 = REBUILD_UINT16(buf, &index);
			ri->re.current = (int16_t)REBUILD_UINT16(buf, &index);
			ri->re.status = REBUILD_UINT16(buf, &index);
			ri->re.temp = buf[index++];
		}
		else if(offset == 1)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			*(ri->ex[0].enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex[0].enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[0].mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[0].mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex[0].mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			*(ri->ex[0].joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex[0].joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex[0].strain = REBUILD_UINT16(buf, &index);
		}
		else if(offset == 2)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			*(ri->ex[1].enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex[1].enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[1].mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex[1].mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex[1].mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			*(ri->ex[1].joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex[1].joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex[1].strain = REBUILD_UINT16(buf, &index);
		}
		else if(offset == 3)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->mn.genVar[0] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[1] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[2] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[3] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[4] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[5] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[6] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[7] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[8] = (int16_t)REBUILD_UINT16(buf, &index);
			ri->mn.genVar[9] = (int16_t)REBUILD_UINT16(buf, &index);
			//(24 bytes)
		}
		else
		{
			//...
		}

	#endif	//BOARD_TYPE_FLEXSEA_PLAN

	newPocketRRpacketAvailableFlag = 1;
	ri->lastOffsetDecoded = offset;
}

//Gets called when our Master Writes to us
void rx_cmd_pocket_w(uint8_t *buf, uint8_t *info)
{
	//Master Write isn't implemented for this command.

	(void)buf;
	(void)info;
	flexsea_error(SE_CMD_NOT_PROGRAMMED);
}

#ifdef __cplusplus
}
#endif
