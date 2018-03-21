#ifndef INC_FLEXSEA_CMD_DLEG_H
#define INC_FLEXSEA_CMD_DLEG_H

#ifdef __cplusplus
extern "C" {
#endif

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>
#include "flexsea_user_structs.h"
#include "biom_state_machine.h"

//****************************************************************************
// RX/TX Prototype(s):
//****************************************************************************

void rx_cmd_dleg_rw(uint8_t *buf, uint8_t *info);
void rx_cmd_dleg_rr(uint8_t *buf, uint8_t *info);

void tx_cmd_dleg_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber);
void tx_cmd_dleg_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber);
void tx_cmd_dleg_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
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

extern GainParams* stateGains[7];
extern int16_t fsm1StatePlan;
extern float currentScalarPlan;

#ifdef __cplusplus
}
#endif

#endif	//INC_FLEXSEA_CMD_DLEG_H

