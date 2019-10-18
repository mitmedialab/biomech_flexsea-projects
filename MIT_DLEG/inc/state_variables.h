#ifndef INC_STATE_VARIABLES
#define INC_STATE_VARIABLES

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


//Joint Type: activate one of these for joint limit angles.
//measured from nominal joint configuration, in degrees

//0. Update ./flexsea-projects/inc/User-mn.h to specify ACTIVE_SUBPROJECT

//1. Select joint type

//#define IS_KNEE	// SUBPROJECT_A <- Don't forget to set this if using Knee
#define IS_ANKLE	// SUBPROJECT_B <- Don't forget to set this if using Knee, ankle is slave


/* //Phasing out these modes now moving to use userWrite[0] to command this.
*	//#define IS_ACTUATOR_TESTING 		// Used when testing actuators, ie manually setting impedance values
*	//#define IS_SWEEP_TEST
*	//#define IS_SWEEP_CHIRP_TEST			// For system ID experiments.
*/


//2. Set Subproject if necessary.
// go to ../inc/user-mn.h
// set SUBPROJECT_A <- Don't forget to set this if using Ankle alone, or set for Knee when using an Ankle, This is Master
// set SUBPROJECT_B <- Don't forget to set this for the ankle if using Knee, ankle is slave

//3. Select device
//#define DEVICE_TF08_A01			// Define specific actuator configuration. Ankle 01
//#define DEVICE_TF08_A02		// Define specific actuator configuration. Knee 01
//#define DEVICE_TF08_A03		// Define specific actuator configuration. Knee 01
#define DEVICE_TF08_A04		// Define specific actuator configuration. Knee 02
//#define DEVICE_M14			// Standalone motor for testbench
//#define DEVICE_M15			// Standalone motor for testbench
//#define DEVICE_M16			// Standalone motor for testbench
//#define DEVICE_M23			// Standalone motor for testbench


// 4. Turn off things if necessary.
//#define NO_DEVICE				// use if not connected to an actuator or any hardware. Note to get Device ID use getDeviceIdIncrementing()
//#define NO_ACTUATOR				// use if testing motors, but not attached to actuator
#define NO_POWER				// testing control signals, do not use setMotorcurrent()

//5. Select peripheral options
//#define USE_EMG

//6. Specify User Walking Parameters (if applicable)
//#define SUBJECT_012
#define SUBJECT_002

//****************************************************************************
// Structure(s):
//****************************************************************************

// torque replay
#define IS_TORQUE_REPLAY

enum walkingStateModes{

	STATE_INIT = -2,
	STATE_IDLE = -1,

	STATE_EARLY_STANCE = 0,
    STATE_MID_STANCE = 1,
	STATE_LATE_STANCE_POWER = 2,
	STATE_EARLY_SWING = 3,
	STATE_LATE_SWING = 4,

	STATE_STANDING = 5,
	STATE_STANDING_UNLOADED = 6,

	STATE_EMG_STAND_ON_TOE = 7,
    STATE_LSW_EMG = 8,
	STATE_TORQUE_REPLAY = 9,
	STATE_NONLINEAR_STIFF = 10
};

enum guiExperimentMode{
	// Define different experiments,
	// these specify which controls to run and what input/outputs
	// used by user_data_1.w[0]
	EXP_RESET_DEVICE				= -99,
	EXP_ACTUATOR_TESTING			= -3,
	EXP_IS_SWEEP_CHIRP_TEST			= -2,
	EXP_IS_SWEEP_TEST				= -1,

	EXP_USER_CODE					= 0,

	EXP_ANKLE_PASSIVE 				= 1,	// ACT AS JUST A SPRING ELEMENT
	EXP_ANKLE_WALKING_FSM 			= 2,	// FINITE-STATE MACHINE CONTROLLER
	EXP_ANKLE_WALKING_BIOM_FSM 		= 3,	// SIMULATE A BIOM WITH NO DORSIFLEXION
	EXP_ANKLE_WALKING_TORQUE_REPLAY = 4,		// REPLAY TORQUE
	EXP_ANKLE_WALKING_QUASIPASSIVE 	= 5,	// QUASI-PASSIVE (TWO SPRING) MODE
	EXP_ANKLE_WALKING_NONLINEAR_K 	= 6,	// NONLINEAR STIFFNESS, BIOINSPIRED REPLAY MODE
	EXP_KNEE_WALKING_FSM			= 7 	// NOT IN USE RIGHT NOW.

};

enum guiWalkingParameterVariableUpdates{
	// Walking Controller User Inputs
	// Change mode for what set of inputs you want to use
	// used by user_data_1.w[1]
	USER_INPUT_ANKLE_NOMINAL		= 0,
	USER_INPUT_ANKLE_ORIGINAL		= 1,
	USER_INPUT_ANKLE_IMPEDANCE		= 2,
	USER_INPUT_ANKLE_STANCE			= 3,
	USER_INPUT_ANKLE_SWING			= 4,
	USER_INPUT_ANKLE_TORQUE_REPLAY  = 5,
	USER_INPUT_ANKLE_NONLINEAR_K	= 6,
	USER_INPUT_ANKLE_NONLINEAR_K1	= 7

};

enum guiActuatorTestingVariableUpdates{
	EXP_ACT_CONTROL_PARAM_MAIN		= 0,
	EXP_ACT_CONTROL_PARAM_SECOND	= 1
};


typedef struct{
    int8_t 	 currentState;
    int8_t 	 slaveCurrentState;
    uint16_t onEntrySmState;
    int8_t 	 onEntrySlaveSmState;
    uint16_t lastSmState;
    uint32_t timeStampFromSlave;

} WalkingStateMachine;

//Gain params
#define NUM_STATES				8
#define NUM_IMPEDANCE_TERMS		4

typedef struct rampParam{
// Store values for ramping torque from one place to the next
	// If you ramp K
	float kCurrent;
	float kInit;
	float kFinal;
	float kNew;

	//If you ramp theta
	float thetaCurrent;
	float thetaInit;
	float thetaFinal;

	float torqueCurrent;

	uint32_t deltaTime;
	uint32_t runningTimer;
} RampParam;

typedef struct gainParams{

	float k1;
	float k2;
	float b;
	float thetaDes;

	RampParam kParam;

} GainParams;


// Actuator structure to track sensor values, initially built for the TF08 style actuator
typedef struct act_s
{
    float jointAngle;		// [rad]
    float jointAngleDegrees;
    float jointVel;			// [rad/s]
    float jointVelDegrees;
    float jointAcc;			// [rad/s/s]
    float jointPower;		// [watts]
    float jointEnergy;		// [J]

    float efficiencyInstant; //

    float linkageMomentArm;
    float axialForce;		// actively used force measurement.
    float axialForceTF;		// calculated or transfer function force measurement
    float axialForceLC;		// load cell measurement
    float axialForceEnc;	// encoder calculated force
    float jointTorque;
    float tauMeas;          // torque contribution from series spring
    float tauDes;           // FSM des torque - tauMeas
    float screwLengthDelta;		// track deflection of spring
    float linkageLengthNonLinearity;	// track difference in calculated and measured length
    float lastJointAngle;
    float lastJointVel;
    float lastJointTorque;
    float jointTorqueRate;  // Joint torque rate
    float safetyTorqueScalar;	// Scalar value to reduce overall allowed torque generated.

    float c;				// calculated screw Length 	[mm]
    float c0; 				// initial calculated screw length	[mm]
    int32_t motorPosRaw;    	// motor position [counts]
    int32_t motorPos0;		// initial position of motor.[counts]
    int32_t motorPosDelta; 	// motor position expected [cnts]
    float motorPos;       		// motor position [rad]
    float motorVel;				// motor velocity [rad/s]
    float motorAcc;				// motor acceleration [rad/s/s]
    int16_t regTemp;		// regulate temperature [C]
    int16_t motTemp;		// motor temperature [C]
    int32_t motCurr;		// motor current [mA]
    int32_t motCurrDt;		// di/dt change in motor current [mA]

    float motorPower;		// [W]
    float motorEnergy;	// [J]

    int32_t desiredCurrent; // desired current from getMotorCurrent() [mA]
    int32_t desiredVoltage; // desired current from getMotorCurrent() [mV]
    int32_t currentOpLimit; // current throttling limit [mA]
    int32_t voltageOpLimit; // maximum allowable voltage.

    int32_t stanceTimer;	// [cycle counts]

    int16_t safetyFlag;		// todo: consider if necessary
    int8_t  initializedSettings;	// True if settings have been set for whatever testing mode

    // Controls Related
    float torqueKp;
    float torqueKi;
    float torqueKd;
    float controlFF;		// used for testing parameter requirements, not used normally
    float controlScaler;	// used for testing parameter requirements, not used normally

    /* MULTIPACKET BELOW */
    //following are multipacket specific
    int8_t motorOnFlag;
	int8_t mapWritten;
	int8_t foundPoles;
	uint16_t commandTimer;

	//following values are sent over multipacket
	int16_t intJointAngleDegrees; // all have x100 multiplication when sent over!!
	int16_t intJointVelDegrees;
	int16_t intJointTorque;
	int16_t desiredJointAngleDeg; //multiplier
	float	desiredJointAngleDeg_f;
	uint16_t desiredJointK; //multiplier
	float 	desiredJointK_f;
	uint16_t desiredJointB; //multiplier
	float	desiredJointB_f;

} Act_s;


typedef struct walkParams {

	//Early stance params
    float earlyStanceK0;
    float earlyStanceKF;
    float earlyStanceB;
    float earlyStanceDecayConstant;
    float virtualHardstopK;
    float virtualHardstopB;
    float virtualHardstopEngagementAngle;

    float passiveVirtualHardstopK;
    float passiveVirtualHardstopB;
    float passiveVirtualHardstopEngagementAngle;

    float biomVirtualHardstopK;
    float biomVirtualHardstopB;
    float biomVirtualHardstopEngagementAngle;
    float biomAnkleGainsThetaDesEsw;
    float biomAnkleGainsThetaDesLsw;

    float neutralPosition;

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

	uint32_t timerInStance;
	uint32_t timerInStanceLast;

	uint32_t timerInSwing;			// track counts in swing
	uint32_t timerInSwingLast;		// track counts in swing from last swing phase
	uint32_t swingTrajectoryTimer;	// how long to ramp into position.

	GainParams ankleGainsEst;
	GainParams ankleGainsMst;
	GainParams ankleGainsLst;
	GainParams ankleGainsEsw;
	GainParams ankleGainsLsw;
	GainParams ankleGainsNonLinear;

	float userMass;

	//biom early stance value
	float scaleFactor;

} WalkParams;



typedef struct kTransitionParams{
// not sure this is in use
	float kInit;
	float kFinal;
	float deltaT;
	float timer;
	float thetaSet;
	float thetaCurr;

}KTransitionParams;

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


typedef struct actTestSettings {
	float freqInput 	;
	float freqRad 		;
	float inputTheta 	;
	float inputK 		;
	float inputB 		;
	float inputTorq 	;
	int8_t currentOrVoltage ;

	int16_t begin;
	float freq	;
	float freqFinal;
	float freqSweepTime	;
	int16_t chirpType;
	float amplitude;
	float dcBias;
	float noiseAmp;

} ActTestSettings;

// torque replay variables
typedef struct TorqueRep{
	float tauDes;
	float tauDesPlot;
	float time_stance;
	float standard_stance_period;
	float previous_stance_period;
	float time_swing;
	float standard_swing_period;
	float previous_swing_period;
	float speedFactor;
	float percent;
	float torqueScalingFactor;	// User defined scaling value
	//float torque_traj_mscaled[TRAJ_SIZE];
	int16_t index;
	int8_t entry_replay;	// Turns off first time torque replay is working.
	int8_t begin;
} TorqueRep;


// nonlinear K control
typedef struct nonLinearK {
	int16_t ascAngleIndex;
	int16_t descAngleIndex;
	float stiffnessCurrentVal;
	int8_t earlyLateFlag;
	float torqueStiff;
	int8_t currentWalkingState;

} NonLinearK;

//****************************************************************************
// Shared variable(s)
//****************************************************************************
extern int8_t fsm1State;
extern float currentScalar;


#ifdef __cplusplus
}
#endif

#endif //INC_STATE_VARIABLES
