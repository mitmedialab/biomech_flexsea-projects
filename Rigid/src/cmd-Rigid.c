/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'plan-gui' Graphical User Interface
	Copyright (C) 2017 Dephy, Inc. <http://dephy.com/>
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] cmd_rigid: Rigid Commands
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-04-18 | jfduval | New code
	* 2017-08-27 | jfduval | Complete rehaul of the command. Not backward
	* compatible - use only with recent versions of the Plan & Ex.
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
#include "../inc/cmd-Rigid.h"
#include "flexsea_user_structs.h"
#include "flexsea_cmd_user.h"
#include "user-mn.h"

#ifdef DEPHY
#include "dephy-mn.h"
#endif

#if(ACTIVE_PROJECT == PROJECT_DPEB31)
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn-DpEb31.h"
#endif
#endif

#if((ACTIVE_PROJECT == PROJECT_DEPHY) && (ACTIVE_DEPHY_PROJECT == PRJ_DEPHY_DPEB42))
#ifdef BOARD_TYPE_FLEXSEA_MANAGE
#include "user-mn-DpEb42.h"
#endif
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t newRigidRRpacketAvailableFlag = 0;
//Rigid1
int32_t cri_enc_ang = 0, cri_enc_ang_vel = 0;
int16_t cri_joint_ang = 0, cri_joint_ang_vel = 0;
int16_t cri_ctrl_ank_ang_deg = 0, cri_ctrl_ank_vel = 0;
int16_t cri_joint_ang_from_mot = 0, cri_ctrl_ank_ang_from_mot = 0;
//Rigid2
int32_t cri_enc_ang2 = 0, cri_enc_ang_vel2 = 0;
int16_t cri_joint_ang2 = 0, cri_joint_ang_vel2 = 0;
int16_t cri_ctrl_ank_ang_deg2 = 0, cri_ctrl_ank_vel2 = 0;
int16_t cri_joint_ang_from_mot2 = 0, cri_ctrl_ank_ang_from_mot2 = 0;

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//Poll to see if new data is available:
uint8_t newRigidRRpacketAvailable(void)
{
	uint8_t retVal = newRigidRRpacketAvailableFlag;
	newRigidRRpacketAvailableFlag = 0;
	return retVal;
}

//Get a copy of the latest Rigid values
void getLastRigidData(struct rigid_s *r){(*r) = rigid1;}

void initializeRigidPointers(struct rigid_s *r)
{
	r->ex.enc_ang = &r->ex._enc_ang_;
	r->ex.enc_ang_vel = &r->ex._enc_ang_vel_;
	r->ex.joint_ang = &r->ex._joint_ang_;
	r->ex.joint_ang_vel = &r->ex._joint_ang_vel_;
	r->ex.joint_ang_from_mot = &r->ex._joint_ang_from_mot_;

	r->ctrl.ank_ang_deg = &r->ctrl._ank_ang_deg_;
	r->ctrl.ank_vel = &r->ctrl._ank_vel_;
	r->ctrl.ank_ang_from_mot = &r->ctrl._ank_ang_from_mot_;
}

//Initialize pointers
void init_rigid(void)
{
	initializeRigidPointers(&rigid1);
	initializeRigidPointers(&rigid2);
}

//****************************************************************************
// RX/TX Function(s)
//****************************************************************************

void tx_cmd_rigid_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_READ_ALL_RIGID;
	(*cmdType) = CMD_READ;

	//Data:
	shBuf[index++] = offset;

	//Payload length:
	(*len) = index;
}

//Pack tx_cmd_rigid_r()
void ptx_cmd_rigid_r(uint8_t slaveId, uint16_t *numb, uint8_t *commStr, \
							uint8_t offset)
{
	tx_cmd_rigid_r(TX_N_DEFAULT, offset);
	pack(P_AND_S_DEFAULT, slaveId, NULL, numb, commStr);
}

void tx_cmd_rigid_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
					uint16_t *len, uint8_t offset)
{
	uint16_t index = 0;

	//Formatting:
	(*cmd) = CMD_READ_ALL_RIGID;
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
		SPLIT_16((uint16_t)(ri->ex.ctrl.current.setpoint_val >> 3), shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
		SPLIT_16((uint16_t)ri->ex.status, shBuf, &index);
		//(28 bytes)

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

		//Defaults:
		struct rigid_s *ri = &rigid1;
		int16_t *encoder = ri->ex.joint_ang;
		int16_t *encoderVel = ri->ex.joint_ang_vel;

		//For Rigid we use a different structure with correct signs, and different encoders:
		#if((ACTIVE_PROJECT == PROJECT_DPEB31) || ((ACTIVE_PROJECT == PROJECT_DEPHY) && (ACTIVE_DEPHY_PROJECT == PRJ_DEPHY_DPEB42)) && RUNTIME_FSM1 == ENABLED)
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
			SPLIT_16((uint16_t)ri->mn.accel.x, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.y, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.accel.z, shBuf, &index);
			SPLIT_16((uint16_t)*(encoder), shBuf, &index);
			SPLIT_16((uint16_t)*(encoderVel), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ctrl.ank_ang_from_mot), shBuf, &index);
			SPLIT_16((uint16_t)ri->ex.strain, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.ctrl.current.setpoint_val >> 3), shBuf, &index);
			//(28 bytes)
		}
		else if(offset == 1)
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang), shBuf, &index);
			SPLIT_32((uint32_t)*(ri->ex.enc_ang_vel), shBuf, &index);
			SPLIT_32((uint32_t)ri->ex.mot_acc, shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_current >> 3), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ex.mot_volt >> 3), shBuf, &index);
			SPLIT_16(rigid1.re.vb, shBuf, &index);
			SPLIT_16((uint16_t)rigid1.re.current, shBuf, &index);
			shBuf[index++] = rigid1.re.temp;
			SPLIT_16(rigid1.re.status, shBuf, &index);
			shBuf[index++] = (uint8_t)ri->ctrl.walkingState;
			shBuf[index++] = (uint8_t)ri->ctrl.gaitState;
			//(29 bytes)
		}
		else if(offset == 2)
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
			//(24 bytes)
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
		else if(offset == 4)	//This is used to tweak and test bilateral controllers
		{
			SPLIT_32(ri->ctrl.timestamp, shBuf, &index);
			SPLIT_16((uint16_t)ri->mn.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ctrl.ank_ang_deg), shBuf, &index);
			SPLIT_16((uint16_t)*(ri->ctrl.ank_ang_from_mot), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ctrl.contra_hs), shBuf, &index);
			SPLIT_16((uint16_t)(ri->ctrl.step_energy), shBuf, &index);
			shBuf[index++] = (uint8_t)ri->ctrl.walkingState;
			shBuf[index++] = (uint8_t)ri->ctrl.gaitState;

			SPLIT_16((uint16_t)rigid2.mn.gyro.z, shBuf, &index);
			SPLIT_16((uint16_t)*(rigid2.ctrl.ank_ang_deg), shBuf, &index);
			SPLIT_16((uint16_t)*(rigid2.ctrl.ank_ang_from_mot), shBuf, &index);
			SPLIT_16((uint16_t)(rigid2.ctrl.contra_hs), shBuf, &index);
			SPLIT_16((uint16_t)(rigid2.ctrl.step_energy), shBuf, &index);
			shBuf[index++] = (uint8_t)rigid2.ctrl.walkingState;
			shBuf[index++] = (uint8_t)rigid2.ctrl.gaitState;
			//(30 bytes)
		}

	#endif	//BOARD_TYPE_FLEXSEA_MANAGE

	//Payload length:
	(*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_rigid_rw(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	(void)info;

	//Temporary variables
	uint8_t offset = 0;

	//Decode data received:
	index = P_DATA1;
	offset = buf[index++];

	//Reply:
	tx_cmd_rigid_w(TX_N_DEFAULT, offset);
	packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER);
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_rigid_rr(uint8_t *buf, uint8_t *info)
{
	uint16_t index = 0;
	uint8_t offset = 0;
	struct rigid_s *ri = &rigid1;
	(void)info;

	#ifdef BOARD_TYPE_FLEXSEA_EXECUTE
		(void)buf;
	#endif

	#ifdef BOARD_TYPE_FLEXSEA_MANAGE

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
			//(28 bytes)
		}

	#endif

	#ifdef BOARD_TYPE_FLEXSEA_PLAN

		rigidPtrXid(&ri, buf[P_XID]);

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
			*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang_from_mot) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ex.strain = REBUILD_UINT16(buf, &index);
			ri->ex.ctrl.current.setpoint_val = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			//(28 bytes)
		}
		else if(offset == 1)
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			*(ri->ex.enc_ang) = (int32_t) REBUILD_UINT32(buf, &index);
			*(ri->ex.enc_ang_vel) = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex.mot_acc = (int32_t) REBUILD_UINT32(buf, &index);
			ri->ex.mot_current = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->ex.mot_volt = (int32_t)(((int16_t)REBUILD_UINT16(buf, &index)) << 3);
			ri->re.vb = REBUILD_UINT16(buf, &index);
			ri->re.current = (int16_t)REBUILD_UINT16(buf, &index);
			ri->re.temp = buf[index++];
			ri->re.status = REBUILD_UINT16(buf, &index);
			ri->ctrl.walkingState = (int8_t)buf[index++];
			ri->ctrl.gaitState = (int8_t)buf[index++];
			//(29 bytes)
		}
		else if(offset == 2)
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

			//In some cases genVar contains Strain data:
			strain1.ch[0].strain_filtered = ri->mn.genVar[0];
			strain1.ch[1].strain_filtered = ri->mn.genVar[1];
			strain1.ch[2].strain_filtered = ri->mn.genVar[2];
			strain1.ch[3].strain_filtered = ri->mn.genVar[3];
			strain1.ch[4].strain_filtered = ri->mn.genVar[4];
			strain1.ch[5].strain_filtered = ri->mn.genVar[5];
			strain1.preDecoded = 1;
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
		else if(offset == 4)	//This is used to tweak and test bilateral controllers
		{
			ri->ctrl.timestamp = REBUILD_UINT32(buf, &index);
			ri->mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(ri->ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ctrl.contra_hs = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ctrl.step_energy = (int16_t) REBUILD_UINT16(buf, &index);
			ri->ctrl.walkingState = (int8_t)buf[index++];
			ri->ctrl.gaitState = (int8_t)buf[index++];


			rigid2.mn.gyro.z = (int16_t) REBUILD_UINT16(buf, &index);
			*(rigid2.ex.joint_ang) = (int16_t) REBUILD_UINT16(buf, &index);
			*(rigid2.ex.joint_ang_vel) = (int16_t) REBUILD_UINT16(buf, &index);
			rigid2.ctrl.contra_hs = (int16_t) REBUILD_UINT16(buf, &index);
			rigid2.ctrl.step_energy = (int16_t) REBUILD_UINT16(buf, &index);
			rigid2.ctrl.walkingState = (int8_t)buf[index++];
			rigid2.ctrl.gaitState = (int8_t)buf[index++];

			//Map to genVar for logging:
			uint8_t genVindex = 2;
			rigid1.mn.genVar[genVindex++] = rigid2.mn.gyro.z;
			rigid1.mn.genVar[genVindex++] = *(rigid2.ex.joint_ang);
			rigid1.mn.genVar[genVindex++] = *(rigid2.ex.joint_ang_vel);
			rigid1.mn.genVar[genVindex++] = rigid2.ctrl.contra_hs;
			rigid1.mn.genVar[genVindex++] = rigid2.ctrl.step_energy;
			rigid1.mn.genVar[genVindex++] = rigid2.ctrl.walkingState;
			rigid1.mn.genVar[genVindex++] = rigid2.ctrl.gaitState;
		}
		else
		{
			//...
		}

	#endif	//BOARD_TYPE_FLEXSEA_PLAN

	newRigidRRpacketAvailableFlag = 1;
	ri->lastOffsetDecoded = offset;
}

//Gets called when our Master Writes to us
void rx_cmd_rigid_w(uint8_t *buf, uint8_t *info)
{
	//Master Write isn't implemented for this command.

	(void)buf;
	(void)info;
	flexsea_error(SE_CMD_NOT_PROGRAMMED);
}

#ifdef __cplusplus
}
#endif
