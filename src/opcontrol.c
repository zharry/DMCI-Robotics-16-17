/**
 * File for operator control code.
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 ********************************************************************************/

#include "../include/API.h"

/**
 * Runs the user operator control code.
 *
 * This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the operator control mode. If the robot is disabled or communications is lost, the operator control task will be stopped by the kernel. Re-enabling the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will run the operator control task. Be warned that this will also occur if the VEX Cortex is tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */

// --------------------------- CONSTANTS -----------------------------

// Digital Read/Write Constants
#define ON 0
#define OFF 1

// Constants for Digital Channel Definitions
typedef struct {
	int a, b, c, d, e, f, g, h, i, j, k, l, SP;
} DIGITAL_CHANNEL;
const DIGITAL_CHANNEL DC = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1 };

// Constants for Analog Channel Definitions
typedef struct {
	int a, b, c, d, e, f, g, h;
} ANALOG_CHANNEL;
const ANALOG_CHANNEL AC = { 13, 14, 15, 16, 17, 18, 19, 20};

// Constants for Joystick Channel Definitions
typedef struct {
	int R_X, R_Y, L_Y, L_X, L_BUM, R_BUM, L_PAD, R_PAD;
} JOYSTICK_CHANNEL;
const JOYSTICK_CHANNEL JC = { 1, 2, 3, 4, 5, 6, 7, 8 };

// Constants for Cortex Motor Channel Definitions
typedef struct {
	int _null, SW_WHEEL, SE_WHEEL, MI_WHEEL;
} MOTOR_CHANNEL;
const MOTOR_CHANNEL MC = { 0, 3, 2, 0 }; // 1, 2, 8, 9 are recommended

// Constants for Robot Status/State
typedef struct {
	int AUTO, ONLINE, ENABLED;
} ROBOT_STATUS;
const ROBOT_STATUS RS = { 0, 1, 2 };

#define RANGE_MAX (127)

int cap(int num, int max) {
	if(num > max) {
		num = max;
	}
	else if(num < -max) {
		num = -max;
	}
	return num;
}

// ------------------------ CONTROL CODE ------------------------------
void operatorControl() {
	// Update or Set Team Name if not yet done already
	setTeamName(TEAMNAME);

	/* Stores which joysticks are connected
	 * 0 - None
	 * 1 - Only Joystick 1
	 * 2 - Only Joystick 2
	 * 3 - Both Josticks 1 and 2
	 */
	int joystickStatus = 0;
	if (isJoystickConnected(1))
		joystickStatus |= 1;
	if (isJoystickConnected(2))
		joystickStatus |= 2;

	// Update Robot Status
	int robotStatus = RS.AUTO;
	if (!isAutonomous()) {
		if (isOnline())
			robotStatus |= RS.ONLINE;
		if (isEnabled())
			robotStatus |= RS.ENABLED;
	}

	// Robot Control Loop
	// Safe checking to avoid disqualification
	while (robotStatus | RS.ENABLED) {
		// Local Variable Definitions
		int lhsX = 0, lhsY = 0, rhsX = 0;

		const int K = 0;
		int L, R, C; // Output Values corresponding to the motor

		// Get Joystick Values based on Status
		if (joystickStatus == 3) {
			// Both Joysticks 1 and 2 are connected and can be referenced
		} else if (joystickStatus == 0) {
			// No Joysticks connected, for debugging operator controlled
		} else {
			/* One of Josystick 1 or 2 is connected,
			 * arguments requiring a joystick should use {joystickStatus} as the joystick number
			 * as the numbers assigned match up (reads from the specified joystick)
			 */
			// leftX = joystickGetAnalog(joystickStatus, JC.L_X); // Movement
			lhsY = joystickGetAnalog(joystickStatus, JC.L_Y); // Movement
			rhsX = joystickGetAnalog(joystickStatus, JC.R_X); // Rotate
		}

		// Debug
		if (false) {

		// Live
		} else {
			L = cap(lhsY - rhsX, RANGE_MAX);
			R = cap(lhsY + rhsX, RANGE_MAX);
			C = cap(lhsX + rhsX * K, RANGE_MAX);

			motorSet(MC.SW_WHEEL, L);
			motorSet(MC.SE_WHEEL, R);
			motorSet(MC.MI_WHEEL, C);
		}

		// Motors can only be updated once every 20ms
		delay(20);
	}

	// Re-do entire process if it failed to start
	operatorControl();
}
