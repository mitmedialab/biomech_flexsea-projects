/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-user' User functions
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd-Bilateral: Communication between 2 Rigid
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-10-06 | jfduval | New code
****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*Notes: BILATERAL is a variant of Rigid used to communicate between two Exos.
 In the future it might get merged with Rigid, DpEb, or ActPack... but for
 now it's simpler and faster to create a new command.

 When we write, we always use rigid1. When we receive, we always use rigid2.*/

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <flexsea.h>
#include <flexsea_system.h>
//#include "../inc/cmd-Rigid.h"
#include "cmd-Bilateral.h"
#include "flexsea_user_structs.h"
#include "flexsea_cmd_user.h"
#include "user-mn.h"

#if(ACTIVE_PROJECT == PROJECT_DPEB31)
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn-DpEb31.h"
#endif
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//****************************************************************************
// RX/TX Function(s)
//****************************************************************************

//With Bilateral, it's always bidirectional. We write the content of our rigid
//while requesting the content of theirs
void tx_cmd_bilateral_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_BILATERAL;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = offset;

	//Defaults:
	struct rigid_s *ri = &rigid1;
	int16_t *encoder = ri->ex.joint_ang;
	int16_t *encoderVel = ri->ex.joint_ang_vel;

	//For Rigid we use a different structure with correct signs, and different encoders:
	#if((ACTIVE_PROJECT == PROJECT_DPEB31) && defined BOARD_TYPE_FLEXSEA_MANAGE)
	ri = &dpRigid;
	encoder = ri->ctrl.ank_ang_deg;
	encoderVel = ri->ctrl.ank_vel;
	#endif

	//Arguments:
	if(offset == 0)
	{
		SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
		SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
		SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
		SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
		//SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
		//SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
		//SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
		SPLIT_16((uint16_t)*(encoder), shBuf, &index);
		SPLIT_16((uint16_t)*(encoderVel), shBuf, &index);
		SPLIT_16((uint16_t)*(ri->ctrl.ank_ang_from_mot), shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
		SPLIT_16((uint16_t)(ri->ex.mot_current >> 3), shBuf, &index);
		shBuf[index++] = (uint8_t)ri->ctrl.walkingState;
		shBuf[index++] = (uint8_t)ri->ctrl.gaitState;
		SPLIT_16((uint16_t)ri->ctrl.step_energy, shBuf, &index);
		SPLIT_16((uint16_t)ri->ctrl.contra_hs, shBuf, &index);
		//(x bytes)
	}

	//Payload length:
	(*len) = index;
}

void tx_cmd_bilateral_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_BILATERAL;
	(*cmdType) = CMD_WRITE;

	//Data:
	shBuf[index++] = offset;

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid1;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//For Rigid we use a different structure with correct signs, and different encoders:
		#if(ACTIVE_PROJECT == PROJECT_DPEB31)
		ri = &dpRigid;
		encoder = ri->ctrl.ank_ang_deg;
		encoderVel = ri->ctrl.ank_vel;
		#endif

		//Arguments:
		if(offset == 0)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
			//SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
			//SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
			//SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
			SPLIT_16((uint16_t)*(encoder), shBuf, &index);
			SPLIT_16((uint16_t)*(encoderVel), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ctrl.ank_ang_from_mot), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_current >> 3), shBuf, &index);
			shBuf[index++] = (uint8_t)ri->ctrl.walkingState;
			shBuf[index++] = (uint8_t)ri->ctrl.gaitState;
			SPLIT_16((uint16_t)ri->ctrl.step_energy, shBuf, &index);
			SPLIT_16((uint16_t)ri->ctrl.contra_hs, shBuf, &index);
			//(x bytes)
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Payload length:
	(*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_bilateral_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	(void)info;

	//Temporary variables
	uint8_t offset = 0;

	//Decode data received:
	index = P_DATA1;
	offset = buf[index++];

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid2;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//For Rigid we use a different structure with correct signs, and different encoders:
		#if(ACTIVE_PROJECT == PROJECT_DPEB31)
		//ri = &dpRigid;
		encoder = ri->ctrl.ank_ang_deg;
		encoderVel = ri->ctrl.ank_vel;
		#endif

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->mn.gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			*(encoder) = (int16_t) REBUILD_UINT16(buf, &index);
			*(encoderVel) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ctrl.ank_ang_from_mot) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex.mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex.mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ctrl.walkingState = (int8_t)buf[index++];
			ri->ctrl.gaitState = (int8_t)buf[index++];
			ri->ctrl.step_energy = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ctrl.contra_hs = (int16_t) REBUILD_UINT16(buf, &index);

			//(x bytes)
		}

	#endif //BOARD_TYPE_FLEXSEA_MANAGE

	//Reply:
	tx_cmd_bilateral_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_bilateral_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;
	(void)info;


	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid2;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//For Rigid we use a different structure with correct signs, and different encoders:
		#if(ACTIVE_PROJECT == PROJECT_DPEB31)
		//ri = &dpRigid;
		encoder = ri->ctrl.ank_ang_deg;
		encoderVel = ri->ctrl.ank_vel;
		#endif

		index = P_DATA1;
		offset = buf[index++];

		if(offset == 0)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->mn.gyro.x = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.y = (int16_t) REBUILD_UINT16(buf, &index);
			ri->mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.x = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.y = (int16_t) REBUILD_UINT16(buf, &index);
			//ri->mn.accel.z = (int16_t) REBUILD_UINT16(buf, &index);
			*(encoder) = (int16_t) REBUILD_UINT16(buf, &index);
			*(encoderVel) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ctrl.ank_ang_from_mot) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex.mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex.mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ctrl.walkingState = (int8_t)buf[index++];
			ri->ctrl.gaitState = (int8_t)buf[index++];
			ri->ctrl.step_energy = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ctrl.contra_hs = (int16_t) REBUILD_UINT16(buf, &index);
			//(x bytes)
		}

		ri->lastOffsetDecoded = offset;

	#else

		(void)index;
		(void)offset;

	#endif //BOARD_TYPE_FLEXSEA_MANAGE
}

#ifdef __cplusplus
}
#endif
