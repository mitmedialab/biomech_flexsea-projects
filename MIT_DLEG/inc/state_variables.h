#ifndef INC_STATE_VARIABLES
#define INC_STATE_VARIABLES

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees

//1. Select joint type

#define IS_KNEE        //IS_KNEE is AT02_01 (MOTOR1), for now
//#define IS_ANKLE     //IS_ANKLE is AT02_02 (MOTOR2), for now

//2. Select device
#define DEVICE_AT02_01         // Define specific actuator configuration. Two dof Ankle Motor 1
//#define DEVICE_AT02_02         // Define specific actuator configuration. Two dof Ankle Motor 2

//#define DEVICE_TF08_A01			// Define specific actuator configuration. Ankle 01
//#define DEVICE_TF08_A02		// Define specific actuator configuration. Ankle 02
//#define DEVICE_TF08_A03		// Define specific actuator configuration. Knee 01
//#define DEVICE_TF08_A04		// Define specific actuator configuration. Knee 02
//#define NO_DEVICE

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
    int8_t currentState;
    uint16_t onEntrySmState;
    uint16_t lastSmState;

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

    float jointTorque;
    float tauMeas;          // torque contribution from series spring
    float tauDes;           // FSM des torque - tauMeas
    float lastJointAngle;
    float lastJointVel;
    float lastJointTorque;
    float jointTorqueRate;  // Joint torque rate
    float safetyTorqueScalar;	// Scalar value to reduce overall allowed torque generated.
    float crankAngleDegrees;    	// crank position [degrees]
    float lastCrankAngleDegrees;
    float crankAngleRad;
    float crankVel;          //[deg/s]
    int32_t motorPosRaw;    	// motor position [counts]
    int32_t motorPosNeutral;	// neutral position of motor.
    float motorPos;       		// motor position [rad]
    float motorVel;				// motor velocity [rad/s]
    float motorAcc;				// motor acceleration [rad/s/s]
    int16_t regTemp;		// regulate temperature [C]
    int16_t motTemp;		// motor temperature [C]
    int32_t motCurr;		// motor current [mA]
    int32_t desiredCurrent; // desired current from getMotorCurrent() [mA]
    int32_t currentOpLimit; // current throttling limit [mA]
    int8_t safetyFlag;		// todo: consider if necessary

    //following are multipacket specific
    int8_t motorOnFlag;
	int8_t mapWritten;
	int8_t foundPoles;
	uint16_t commandTimer;

	//following values are sent over multipacket
	int32_t axialForce;
	int16_t intCrankAngleDegrees;
	int16_t intJointAngleDegrees; // all have x100 multiplication when sent over!!
	int16_t intTauDes;
	int32_t intCrankVel;
	int16_t intJointTorque;
	int16_t desiredJointAngleDeg; //multiplier
	float	desiredJointAngleDeg_f;
	uint16_t desiredJointK; //multiplier
	float 	desiredJointK_f;
	uint16_t desiredJointB; //multiplier
	float	desiredJointB_f;
	int16_t volt;

} Act_s;


typedef struct walkParams {

	//Early stance params
    float earlyStanceK0;
    float earlyStanceKF;
    float earlyStanceB;
    float earlyStanceDecayConstant;
    float virtualHardstopK;
    float virtualHardstopEngagementAngle;

    int8_t initializedStateMachineVariables;

    //LSP values
    float lspEngagementTorque;
    float samplesInLSP;
	float pffGain;
	float pffExponent;
	float lspEntryTq;
	float pffLumpedGain;
	float virtualHardstopTq;
	float pffRampTics;
	int16_t transitionId;
	float lstPGDelTics;

	//biom early stance value
	float scaleFactor;

} WalkParams;

typedef struct cubicSpline{
	float xi1; // x initial
	float xInt1; // x intermediate
	float xf1; // x final
	float yi1; // y initial
	float yInt1; // y intermediate
	float yf1; // y final
	float X; // interpolation x coordinate
	float Y; // interpolation Y coordinate
	float thetaSetFsm;
	float resFactor; // resolution factor
	uint32_t timeState;
	float a11; // coefficients for the polynomial
	float a21;
	float b11;
	float b21;
	float xi2; // x initial
	float xInt2; // x intermediate
	float xf2; // x final
	float yi2; // y initial
	float yInt2; // y intermediate
	float yf2; // y final
	float a12; // coefficients for cubic functions
	float a22;
	float b12;
	float b22;
} CubicSpline;

//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern int8_t fsm1State;
extern float currentScalar;


#ifdef __cplusplus
}
#endif

#endif //INC_STATE_VARIABLES
