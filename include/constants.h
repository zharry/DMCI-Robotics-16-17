/*  Directional Rules:
 * 	Motors:
 * 	 1. Holding the motor screw holes up
 * 	 2. Axle away from you
 * 	 A. Clockwise is positive
 * 	 B. Counter Clockwise is negative
 * 	Cortex:
 * 	 1. Ports side up
 * 	 2. Power facing towards you
 * 	 3. USB port facing away from you
 * 	 A. Forward is pointing in the same direction as the USB port
 *
 * Build Rules:
 *  1. Connect same color to wires
 *  2. On two size ports, the black is on the left
 *  3. On three size ports, the black is on the right
 */
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// Set Teamname string
#define TEAMNAME "Team2505"

// Movement Constant
#define RANGE_MAX 255
#define MOVEK 1
#define LIFTSPEED 64

#define CAP(x,l) (x<-l?-l:(x>l?l:x))
#define CAP1(x) CAP(x,1)

// Movement Cap
//inline int cap(int num, int limit)
//{
//	if(num > limit)
//		return limit;
//	if(num < -limit)
//		return -limit;
//	return num;
//}

#define JOY_RX 1
#define JOY_RY 2
#define JOY_LX 4
#define JOY_LY 3
#define JOY_LBUM 5
#define JOY_RBUM 6
#define JOY_LPAD 7
#define JOY_RPAD 8

// Joystick Channel Definitions
//typedef struct {
//	int R_X, R_Y, L_Y, L_X, L_BUM, R_BUM, L_PAD, R_PAD;
//} JOYSTICK_CHANNEL;
//const JOYSTICK_CHANNEL JC = { 1, 2, 3, 4, 5, 6, 7, 8 };

#define MC_WHEEL_L 2
#define MC_WHEEL_R 9
#define MC_WHEEL_M 1

#define MC_LIFT_BL 3
#define MC_LIFT_ML 4
#define MC_LIFT_TL 5
#define MC_LIFT_BR 6
#define MC_LIFT_MR 7
#define MC_LIFT_TR 8

#define MC_SUPPORT 10

// Cortex Motor Channel Definitions
//typedef struct {
//	// Splitter On:
//	// L_WHEEL R_WHEEL
//	int M_WHEEL, L_WHEEL, BL_LIFT, ML_LIFT, TL_LIFT, BR_LIFT, MR_LIFT, TR_LIFT,
//			R_WHEEL, SUPPORT;
//} MOTOR_CHANNEL;
//	// Power Expander On:
//	// 2, 9 - After splitter
//const MOTOR_CHANNEL MC = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

// Robot Status/State
//typedef struct {
//	int AUTO, ONLINE, ENABLED;
//} ROBOT_STATUS;
//const ROBOT_STATUS RS = { 0, 1, 2 };

#endif // CONSTANTS_H_
