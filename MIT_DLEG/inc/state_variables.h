#ifndef INC_STATE_VARIABLES
#define INC_STATE_VARIABLES

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//****************************************************************************
// Structure(s):
//****************************************************************************

// Actuator structure to track sensor values, initially built for the TF08 style actuator
typedef struct act_s
{
    float jointAngle;
    float jointAngleDegrees;
    float jointVel;
    float jointVelDegrees;
    float jointAcc;
    float linkageMomentArm;
    float axialForce;
    float jointTorque;
    float tauMeas;          // torque contribution from series spring
    float tauDes;           // FSM des torque - tauMeas
    float lastJointAngle;
    float lastJointTorque;
    float jointTorqueRate;  // Joint torque rate

    int32_t motorVel;		// motor velocity [rad/s]
    int32_t motorAcc;		// motor acceleration [rad/s/s]
    int16_t regTemp;		// regulate temperature
    int16_t motTemp;		// motor temperature
    int32_t motCurr;		// motor current
    int32_t desiredCurrent; // desired current from getMotorCurrent()
    int32_t currentOpLimit; // current throttling limit
    int8_t safetyFlag;		// todo: consider if necessary
    int8_t motorOnFlag;

} Act_s;


//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern int8_t fsm1State;
extern float currentScalar;


#ifdef __cplusplus
}
#endif

#endif //INC_STATE_VARIABLES
