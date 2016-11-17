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

// Movement Cap
int cap(int num, int limit)
{
	if(num > limit)
		return limit;
	if(num < -limit)
		return -limit;
	return num;
}

// Digital Read/Write Constants
#define ON 0
#define OFF 1

// Digital Channel Definitions
typedef struct {
	int a, b, c, d, e, f, g, h, i, j, k, l, SP;
} DIGITAL_CHANNEL;
const DIGITAL_CHANNEL DC = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1 };

// Analog Channel Definitions
typedef struct {
	int a, b, c, d, e, f, g, h;
} ANALOG_CHANNEL;
const ANALOG_CHANNEL AC = { 13, 14, 15, 16, 17, 18, 19, 20 };

// Joystick Channel Definitions
typedef struct {
	int R_X, R_Y, L_Y, L_X, L_BUM, R_BUM, L_PAD, R_PAD;
} JOYSTICK_CHANNEL;
const JOYSTICK_CHANNEL JC = { 1, 2, 3, 4, 5, 6, 7, 8 };

// Cortex Motor Channel Definitions
typedef struct {
	// Splitter On:
	// L_WHEEL R_WHEEL
	int M_WHEEL, L_WHEEL, BL_LIFT, ML_LIFT, TL_LIFT, BR_LIFT, MR_LIFT, TR_LIFT,
			R_WHEEL, SUPPORT;
} MOTOR_CHANNEL;
	// Power Expander On:
	// 2, 9 - After splitter
const MOTOR_CHANNEL MC = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

// Robot Status/State
typedef struct {
	int AUTO, ONLINE, ENABLED;
} ROBOT_STATUS;
const ROBOT_STATUS RS = { 0, 1, 2 };

#endif // CONSTANTS_H_
