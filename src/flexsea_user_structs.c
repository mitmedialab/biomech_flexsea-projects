/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-system' System commands & functions
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
	[This file] flexsea_user_structs: contains all the data structures
	used across the user projects
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "flexsea_user_structs.h"

#if(defined BOARD_TYPE_FLEXSEA_EXECUTE)
	#include "mag_encoders.h"
	#include "ext_input.h"
	#if(defined BOARD_SUBTYPE_RIGID || defined BOARD_SUBTYPE_POCKET)
		#include "user-ex-rigid.h"
	#endif
#endif

//****************************************************************************
// Variable(s)
//****************************************************************************

//Data structures:
struct motortb_s motortb;

int16_t globvar[10] = {0,0,0,0,0,0,0,0,0,0};

struct rigid_s rigid1, rigid2;
struct pocket_s pocket1;
struct dual_utt_s utt;

//****************************************************************************
// Function(s)
//****************************************************************************

void initializeUserStructs(void)
{
	#if(defined BOARD_TYPE_FLEXSEA_EXECUTE)
		#ifdef BOARD_SUBTYPE_RIGID

			#if(ENC_COMMUT == ENC_AS5047)
			rigid1.ex.enc_ang = &(as5047.signed_ang);
			rigid1.ex.enc_ang_vel = &(as5047.signed_ang_vel);
			#endif

			#if(ENC_COMMUT == ENC_QUADRATURE)
			rigid1.ex.enc_ang = &(as5047.signed_ang);
			rigid1.ex.enc_ang_vel = &(as5047.signed_ang_vel);
			#endif

			rigid1.ex.joint_ang = &(as5048b.filt.ang_clks_16b);
			rigid1.ex.joint_ang_vel = &(as5048b.filt.vel_cpms_16b);
		#endif
		#ifdef BOARD_SUBTYPE_POCKET
			
			#if(ENC_COMMUT == ENC_QUADRATURE)
			pocket1.ex[0].enc_ang = &encoder.count;
			pocket1.ex[1].enc_ang = &encoder2.count;
			pocket1.ex[0].enc_ang_vel = &(as5047.signed_ang_vel);
			pocket1.ex[1].enc_ang_vel = &(as5047.signed_ang_vel);
			#endif
			
			//rigid1.ex.joint_ang = &(as5048b.filt.ang_clks_16b);
			//rigid1.ex.joint_ang_vel = &(as5048b.filt.vel_cpms_16b);
		#endif
	#endif
}
