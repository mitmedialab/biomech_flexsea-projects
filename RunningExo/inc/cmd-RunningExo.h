/*
 * cmd-RunningExo.h
 *
 *  Created on: Apr 6, 2018
 *      Author: Xingbang
 */

#ifndef INC_FLEXSEA_CMD_RUNNINGEXO_H
#define INC_FLEXSEA_CMD_RUNNINGEXO_H

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>
#include "flexsea_user_structs.h"
#include "user-mn-RunningExo.h"
#include "flexsea_cmd_user.h"

//****************************************************************************
// RX/TX Prototype(s):
//****************************************************************************

void rx_cmd_runexo_rw(uint8_t *buf, uint8_t *info);
void rx_cmd_runexo_rr(uint8_t *buf, uint8_t *info);

void tx_cmd_runexo_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber);
void tx_cmd_runexo_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber);
void tx_cmd_runexo_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber);

//****************************************************************************
// Prototype(s) - simplified functions (DLL):
//****************************************************************************

//****************************************************************************
// Definition(s):
//****************************************************************************

//****************************************************************************
// Structure(s):
//****************************************************************************

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern GainParams* impedanceGains[2];
extern int16_t fsm1StatePlan;
extern float currentScalarPlan;
extern int16_t fsm1State;
extern struct runningExoSystemState runningExoState;
extern float currentScalar;

#ifdef __cplusplus
}
#endif





#endif /* INC_FLEXSEA_CMD_RUNNINGEXO_H */
