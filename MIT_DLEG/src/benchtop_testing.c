#include "benchtop_testing.h"




//control ankle torque by through user_data_1[2] as amplitude and user_data_1[3] as frequency
//void torqueSweepTest(struct act_s *actx, struct user_data_s *user_data_1) {
//		static int32_t timer = 0;
//		static int32_t stepTimer = 0;
//		static float currentFrequency = 0;
//
//		float torqueAmp = user_data_1->w[0]/10.;
//		float frequency = user_data_1->w[1]/10.;
//		float frequencyEnd = user_data_1->w[2]/10.;
//		float numSteps = user_data_1->w[3];
//
//		timer++;
//
//		// if torqueAmp is ever set to 0, reset sweep test params
//		if (torqueAmp == 0) {
//			stepTimer = 0;
//			currentFrequency = 0;
//		}
//
//		// start check to see what type of sweep desired
//		if (frequency > 0) {
//			float torqueDes = 0;
//
//			//just want to sweep at one frequency
//			if (frequencyEnd == 0 || numSteps == 0) {
//
//				torqueDes = torqueAmp * sin(frequency*timer*2*M_PI/1000);
//				setMotorTorque(actx, torqueDes);
//
//			} else {
//
//				// 2 seconds per intermediate frequency step
//				if (stepTimer <= 2*SECONDS) {
//
//					torqueDes = torqueAmp * sin(currentFrequency*stepTimer*2*M_PI/1000);
//					setMotorTorque(actx, torqueDes);
//
//					stepTimer++;
//					user_data_1->r[0] = 1; //1 if testing
//
//				} else {
//
//					stepTimer = 0;
//					currentFrequency += (frequencyEnd-frequency)/numSteps; //increment frequency step
//					//stop test
//					if (currentFrequency > frequencyEnd) {
//
//						torqueAmp = 0;
//						frequency = 0;
//						frequencyEnd = 0;
//						numSteps = 0;
//						user_data_1->r[0] = 0; //0 if not sweep testing
//
//					}
//				}
//			}
//
//			//pass back for plotting purposes
//			user_data_1->r[2] = torqueDes;
//
//		} else if (frequency == 0){
//			timer = 0;
//			setMotorTorque(actx, torqueAmp);
//
//			stepTimer = 0;
//			currentFrequency = 0;
//			user_data_1->r[2] = torqueAmp;
//		} else {
//			timer = 0;
//			setMotorTorque(actx, 0);
//
//			stepTimer = 0;
//			currentFrequency = 0;
//			user_data_1->r[2] = 0;
//		}
//
//		user_data_1->r[3] = frequency;
//
//
//
//}
