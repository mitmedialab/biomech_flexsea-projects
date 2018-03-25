#ifndef INC_STATE_VARIABLES
#define INC_STATE_VARIABLES

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//****************************************************************************
// Structure(s):
//****************************************************************************

enum {
	STATE_IDLE = 0,
	STATE_INIT = 1,
	STATE_EARLY_SWING = 2,
	STATE_LATE_SWING = 3,
	STATE_EARLY_STANCE = 4,
    STATE_LATE_STANCE = 5,
    STATE_LATE_STANCE_POWER = 6,
    STATE_EMG_STAND_ON_TOE = 7,
    STATE_LSW_EMG = 8
};

typedef struct{
    int8_t current_state;
    uint16_t on_entry_sm_state;
    uint16_t last_sm_state;

} WalkingStateMachine;

//Gain params
#define NUM_STATES				8
#define NUM_IMPEDANCE_TERMS		4

typedef struct{

	float k1;
	float k2;
	float b;
	float thetaDes;

} GainParams;

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

    float jointTorqueRate;  // Joint torque rate

    int32_t motorVel;		// motor velocity [rad/s]
    int32_t motorAcc;		// motor acceleration [rad/s/s]
    int16_t regTemp;		// regulate temperature
    int16_t motTemp;		// motor temperature
    int32_t motCurr;		// motor current
    int32_t desiredCurrent; // desired current from getMotorCurrent()
    int32_t currentOpLimit; // current throttling limit
    int8_t safetyFlag;		// todo: consider if necessary

    //biom early stance value
    float scaleFactor;
    
    //pff values
    float samplesInLSP;
    float pff_gain;
    float pff_exponent;
    float lsp_entry_tq;
    float pff_lumped_gain;
    float virtual_hardstop_tq;

    //biom late swing to early stance variables (may be optional)
    


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
