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
	int a, b, c, d, e, f, g, h, i, DEBUGBOOL, DEBUG1, DEBUG2, SP;
} DIGITAL_CHANNEL;
const DIGITAL_CHANNEL DC = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, -1 };

// Constants for Analog Channel Definitions
typedef struct {
	int a, b, c, d, e, f, g, h;
} DIGITAL_CHANNEL;
const DIGITAL_CHANNEL DC = { 13, 14, 15, 16, 17, 18, 19, 20};

// Constants for Joystick Channel Definitions
typedef struct {
	int L_X, L_Y, R_Y, R_X, L_BUM, R_BUM, L_PAD, R_PAD;
} JOYSTICK_CHANNEL;
const JOYSTICK_CHANNEL JC = { 1, 2, 3, 4, 5, 6, 7, 8 };

// Constants for Cortex Motor Channel Definitions
typedef struct {
	int _null, NW_WHEEL, NE_WHEEL, SE_WHEEL, SW_WHEEL;
} MOTOR_CHANNEL;
const MOTOR_CHANNEL MC = { 1, 2, 3, 4, 5 };

// Constants for Robot Status/State
typedef struct {
	int AUTO, ONLINE, ENABLED;
} ROBOT_STATUS;
const ROBOT_STATUS RS = { 0, 1, 2 };

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
		joystickStatus += 1;
	if (isJoystickConnected(2))
		joystickStatus += 2;

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
		int leftX = 0, leftY = 0, rightX = 0;

		// Get Joystick Values based on Status
		if (joystickStatus == 3) {
			// Both Joysticks 1 and 2 are connected and can be referenced
		} else if (joystickStatus == 0) {
			// No Joysticks connected, for debugging operator controlled
		} else {
			/* One of Josystick 1 or 2 is connected,
			 * arguments requiring a joystick should use {joystickStatus} as the joystick number
			 * as the numbers assigned match up
			 */
			leftX = joystickGetAnalog(joystickStatus, JC.L_X); // Movement
			leftY = joystickGetAnalog(joystickStatus, JC.L_Y); // Movement
			rightX = -joystickGetAnalog(joystickStatus, JC.R_X); // Rotate
		}

		// Read Inputs
		int deb1 = digitalRead(DC.DEBUG1), deb2 = digitalRead(DC.DEBUG2);

		if (deb1 == ON && deb2 == ON) {
			// Turn on Debug Light
			digitalWrite(DC.DEBUGBOOL, ON);
			// Turn on all Digital Outputs
			for (int i = 1; i < 13; i++) {
				digitalWrite(i, ON);
			}
			// Spin all motors at 0.5 speed
			for (int i = 1; i < 11; i++) {
				motorSet(i, 64);
			}
		} else {
			// Calculate Movement
			int wheel1 = -leftY - leftX - rightX;
			int wheel2 = leftY - leftX - rightX;
			int wheel3 = leftY + leftX - rightX;
			int wheel4 = -leftY + leftX - rightX;

			// Spin Motors
			motorSet(MC.NW_WHEEL, wheel1);
			motorSet(MC.NE_WHEEL, wheel2);
			motorSet(MC.SE_WHEEL, wheel3);
			motorSet(MC.SW_WHEEL, wheel4);
		}

		// Motors can only be updated once every 20ms
		delay(20);
	}

	// Re-do entire process if it failed to start
	operatorControl();
}
