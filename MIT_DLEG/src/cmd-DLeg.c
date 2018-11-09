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
#include <flexsea_cmd_user.h>
#include "cmd-DLeg.h"
#include "state_variables.h"
#include "walking_state_machine.h"
#include "user-mn.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//array used for indexing in cmd-DLeg !ORDER MATTERS!
GainParams* stateGains[7] = {&eswGains, &lswGains, &estGains,  &lstPowerGains};
int16_t fsm1StatePlan;
float currentScalarPlan;

//****************************************************************************
// DLL Function(s)
//****************************************************************************

//****************************************************************************
// RX/TX Function(s)
//****************************************************************************

//gainSet specifies which state's gains we are changing
//shBuf contains all packet info
//cmd specifies the class of commands
//cmdType indicates that we are reading (flexSEA will run write function in response)
//len is number of bytes
void tx_cmd_dleg_r(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber)
{
    uint16_t index = 0;

    //Formatting:
    (*cmd) = CMD_DLEG;
    (*cmdType) = CMD_READ;

    //Data:
    shBuf[index++] = setNumber;

    //Payload length:
    (*len) = index;

}

void tx_cmd_dleg_rw(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber)
{
    uint16_t index = 0;

    //Formatting:
    (*cmd) = CMD_DLEG;
    (*cmdType) = CMD_READ;

    //Data:
    shBuf[index++] = setNumber;

    #ifdef BOARD_TYPE_FLEXSEA_PLAN

        //will pack as uint32_t and unpack with:
        //myFloat = *(float*) &myInt;
        //myInt = *(uint32_t*) &myFloat;
        //gain order = {eswGains, lswGains, estGains, lstGains, lstPowerGains}

        //need to pack as uint32_t, not cast
        SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->k1, shBuf, &index);
        SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->k2, shBuf, &index);
        SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->b, shBuf, &index);
        SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->thetaDes, shBuf, &index);
        SPLIT_16((uint16_t) fsm1StatePlan, shBuf, &index);
        SPLIT_32((uint32_t) *(uint32_t*) &currentScalarPlan, shBuf, &index);
        //(22 bytes)

    #endif	//BOARD_TYPE_FLEXSEA_PLAN

    //Payload length:
    (*len) = index;
}

void tx_cmd_dleg_w(uint8_t *shBuf, uint8_t *cmd, uint8_t *cmdType, \
                    uint16_t *len, uint8_t setNumber) {

    uint16_t index = 0;

    //Formatting:
    (*cmd) = CMD_DLEG;
    (*cmdType) = CMD_WRITE;

    //Data:
    shBuf[index++] = setNumber;

    #ifdef BOARD_TYPE_FLEXSEA_MANAGE

    //will pack as uint32_t and unpack with:
    //myFloat = *(float*) &myInt;
    //myInt = *(uint32_t*) &myFloat;
    //gain order = {eswGains, lswGains, estGains, lstGains, lstPowerGains}

    //need to pack as uint32_t, not cast
    SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->k1, shBuf, &index);
    SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->k2, shBuf, &index);
    SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->b, shBuf, &index);
    SPLIT_32((uint32_t) *(uint32_t*) &stateGains[setNumber]->thetaDes, shBuf, &index);
    SPLIT_16((uint16_t) stateMachine.current_state, shBuf, &index);
    SPLIT_16((uint16_t) fsm1State, shBuf, &index);
    SPLIT_32((uint32_t) *(uint32_t*) &currentScalar, shBuf, &index);

    //(24 bytes)

    #endif	//BOARD_TYPE_FLEXSEA_MANAGE

    //Payload length:
    (*len) = index;
}

//Gets called when our Master sends us a Read request
void rx_cmd_dleg_rw(uint8_t *buf, uint8_t *info)
{
    uint16_t index = 0;
    (void)info;

    //Temporary variables
    uint8_t setNumber = 0;

    //Decode data received:
    index = P_DATA1;
    setNumber = buf[index++];

    #ifdef BOARD_TYPE_FLEXSEA_MANAGE

    uint32_t k1 = REBUILD_UINT32(buf, &index);
    uint32_t k2 = REBUILD_UINT32(buf, &index);
    uint32_t b = REBUILD_UINT32(buf, &index);
    uint32_t thetaDes = REBUILD_UINT32(buf, &index);

    stateGains[setNumber]->k1 = *(float*) &k1;
    stateGains[setNumber]->k2 = *(float*) &k2;
    stateGains[setNumber]->b = *(float*) &b;
    stateGains[setNumber]->thetaDes = *(float*) &thetaDes;

    fsm1State = (int16_t) REBUILD_UINT16(buf, &index);

    uint32_t curScale = REBUILD_UINT32(buf, &index);
    currentScalar = *(float*) &curScale;

    //22 bytes
    #endif	//BOARD_TYPE_FLEXSEA_MANAGE

    //Reply:
    tx_cmd_dleg_w(TX_N_DEFAULT, setNumber);
    packAndSend(P_AND_S_DEFAULT, buf[P_XID], info, SEND_TO_MASTER); //3rd arg info unused
}

//Gets called when our Slave sends us a Reply to our Read Request
void rx_cmd_dleg_rr(uint8_t *buf, uint8_t *info)
{
    (void)info;

    #ifdef BOARD_TYPE_FLEXSEA_PLAN

		uint16_t index = 0;
		uint8_t setNumber = 0;

        index = P_DATA1;
        setNumber = buf[index++];

        uint32_t k1 = REBUILD_UINT32(buf, &index);
        uint32_t k2 = REBUILD_UINT32(buf, &index);
        uint32_t b = REBUILD_UINT32(buf, &index);
        uint32_t thetaDes = REBUILD_UINT32(buf, &index);
        uint16_t currentState = REBUILD_UINT16(buf, &index);

        stateGains[setNumber]->k1 = *(float*) &k1;
        stateGains[setNumber]->k2 = *(float*) &k2;
        stateGains[setNumber]->b = *(float*) &b;
        stateGains[setNumber]->thetaDes = *(float*) &thetaDes;
        stateMachine.current_state = currentState;

        fsm1StatePlan = (int16_t) REBUILD_UINT16(buf, &index);

        uint32_t curScale = REBUILD_UINT32(buf, &index);
        currentScalarPlan = *(float*) &curScale;

        //24 bytes

    #endif	//BOARD_TYPE_FLEXSEA_PLAN
}

#ifdef __cplusplus
}
#endif
