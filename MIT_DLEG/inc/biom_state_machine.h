#ifdef __cplusplus
extern "C" {
#endif

#if (defined INCLUDE_UPROJ_MIT_DLEG && defined BOARD_TYPE_FLEXSEA_MANAGE) || defined BOARD_TYPE_FLEXSEA_PLAN

#ifndef STATE_MACHINE
#define STATE_MACHINE

//****************************************************************************
// Include(s)
//****************************************************************************
#include "state_variables.h"

//****************************************************************************
// Definition(s):
//****************************************************************************
#define ESW_TO_LSW_DELAY              			  100    // Transition time: 2->3 in ms (orig 200)
#define EST_TO_ESW_DELAY              			  20000   // FOR TESTING

//      THRESHOLD / LIMIT NAME                    VALUE          	UNITS           BRIEF DESCRIPTION                             TRANSITION(S)
#define JNT_ORIENT								-JOINT_ANGLE_DIR	// 				JOINT_ANGLE_DIR is defined in user-mn-MIT-DLeg-2dof

#define K_VIRTUAL_HARDSTOP_NM_P_DEG				 7.13// Stiffness of virtual hardstop mimicking BiOM physical hardstop
#define HARD_HEELSTRIKE_TORQUE_THRESH             -5 * JNT_ORIENT				// -80 Nm      - Foot-strike detector
#define HARD_HEELSTRIKE_TORQ_RATE_THRESH         -600 * JNT_ORIENT				// Nm/sec      - Foot-strike detector                          3->4
#define HARD_TOESTRIKE_ANGLE_THRESH              140/K_VIRTUAL_HARDSTOP_NM_P_DEG * JNT_ORIENT                           // Degree          - Toe-strike detector
#define	GENTLE_HEALSTRIKE_TORQ_RATE_THRESH       -300  * JNT_ORIENT                          // Nm/sec      - Gentle foot-strike detector
#define LSTPWR_HS_ANGLE_TRIGGER_THRESH               45/K_VIRTUAL_HARDSTOP_NM_P_DEG * JNT_ORIENT	// Nm          - The ONLY entry to Late Stance Power           4->5
#define LSTPWR_HS_TORQ_TRIGGER_THRESH				45 //Nm torque threshold late stance --> PFF
#define ANKLE_UNLOADED_TORQUE_THRESH                3.0            	// Nm          - Foot unloaded threshold                       5->2
#define EST_TO_LST_FOOT_FLAT_TORQ_RATE            0        // Nm/sec
#define EST_TO_LST_FOOT_FLAT_HS_ANGLE_LIMIT        15/K_VIRTUAL_HARDSTOP_NM_P_DEG * JNT_ORIENT       // Nm

#define EARLYSTANCE_DECAY_CONSTANT                0.99   //                                    Decay constant for early stance
#define K_ES_INITIAL_NM_P_RAD                      300 // Nm/rad                              Early stance initial k (stiffness) value
#define K_ES_INITIAL_NM_P_DEG                      K_ES_INITIAL_NM_P_RAD / DEG_PER_RAD // Nm/deg                              Early stance initial k (stiffness) value
#define K_ES_FINAL_NM_P_RAD                          10 // Nm/rad                                Early stance final k value
#define K_ES_FINAL_NM_P_DEG                          K_ES_FINAL_NM_P_RAD / DEG_PER_RAD // Nm/deg                                Early stance final k value
#define DELTA_K                                (K_ES_INITIAL_NM_P_RAD - K_ES_FINAL_NM_P_RAD )  
#define DELTA_K_DEG                                (K_ES_INITIAL_NM_P_DEG - K_ES_FINAL_NM_P_DEG )
#define B_ES_NM_S_P_RAD 							5 // Nm s/rad                             Early stance damping
#define B_ES_NM_S_P_DEG							B_ES_NM_S_P_RAD / DEG_PER_RAD // Nm s/deg                            Early stance damping
#define EARLYSTANCE_DECAY_CONSTANT_STAR	          (EARLYSTANCE_DECAY_CONSTANT-1)

#define ANGLE_VIRTUAL_HARDSTOP_NM_P_DEG         0.0 //Virtual hardstop engagement angle

#define PCI										131.36 //Nm, NOT EXACTLY RIGHT
#define PFF_DELAY_SAMPLES						300
#define PFF_ANGLE_THRESH_DEG					5 //pff angle thresh



//****************************************************************************
// Shared Variable(s):
//****************************************************************************
extern GainParams eswGains;
extern GainParams lswGains;
extern GainParams estGains;
extern GainParams lstGains; //currently unused in simple implementation
extern GainParams lstPowerGains;
extern GainParams emgStandGains; //currently unused
extern GainParams emgFreeGains;
extern WalkingStateMachine stateMachine;
extern Act_s act1;

//****************************************************************************
// Prototype(s):
//****************************************************************************

void runFlatGroundFSM(struct act_s *actx);
static void updateImpedanceParams(struct act_s *actx);
static float updatePffTorque(struct act_s *actx);
static float calcJointTorque(GainParams gainParams, struct act_s *actx);
static void updatePFDFState(struct act_s *actx);
static void updateVirtualHardstopTorque(struct act_s *actx);

#endif //STATE_MACHINE
#endif //(INCLUDE_UPROJ_MIT_DLEG && BOARD_TYPE_FLEXSEA_MANAGE) || BOARD_TYPE_FLEXSEA_PLAN
#ifdef __cplusplus
}
#endif
