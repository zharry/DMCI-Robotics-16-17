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

#include <stdbool.h>

// Custom "main.h" header
#include "main.h"

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

// ------------------------ CONTROL CODE ------------------------------
void operatorControl() {

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

		// Local Variables
		int moveX, moveY, rotate;
		int L, R, C;
		bool liftUp, liftDw, supUp, supDw;

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

			// Driving
			moveX = joystickGetAnalog(joystickStatus, JC.L_X); // Movement
			moveY = joystickGetAnalog(joystickStatus, JC.L_Y); // Movement
			rotate = joystickGetAnalog(joystickStatus, JC.R_X); // Rotate

			// Support
			supUp = joystickGetDigital(joystickStatus, JC.L_BUM, JOY_DOWN);
			supDw = joystickGetDigital(joystickStatus, JC.L_BUM, JOY_UP);

			// Lift
			liftUp = joystickGetDigital(joystickStatus, JC.R_BUM, JOY_DOWN);
			liftDw = joystickGetDigital(joystickStatus, JC.R_BUM, JOY_UP);

		}

		// Driving
		L = cap(moveY - rotate, RANGE_MAX);
		R = cap(moveY + rotate, RANGE_MAX);
		C = cap((moveX + rotate) * MOVEK, RANGE_MAX);
		motorSet(MC.L_WHEEL, L);
		motorSet(MC.R_WHEEL, R);
		motorSet(MC.M_WHEEL, C);

		// Support
		if (supDw)
			motorSet(MC.SUPPORT, -32);
		else if (supUp)
			motorSet(MC.SUPPORT, 32);
		else
			motorSet(MC.SUPPORT, 0);

		// Lift
		if (liftUp) {
			motorSet(MC.BL_LIFT, LIFTSPEED);
			motorSet(MC.ML_LIFT, LIFTSPEED);
			motorSet(MC.TL_LIFT, LIFTSPEED);
			motorSet(MC.BR_LIFT, -LIFTSPEED);
			motorSet(MC.MR_LIFT, -LIFTSPEED);
			motorSet(MC.TR_LIFT, -LIFTSPEED);
		} else if (liftDw) {
			motorSet(MC.BL_LIFT, -LIFTSPEED);
			motorSet(MC.ML_LIFT, -LIFTSPEED);
			motorSet(MC.TL_LIFT, -LIFTSPEED);
			motorSet(MC.BR_LIFT, LIFTSPEED);
			motorSet(MC.MR_LIFT, LIFTSPEED);
			motorSet(MC.TR_LIFT, LIFTSPEED);
		} else {
			motorSet(MC.BL_LIFT, 0);
			motorSet(MC.ML_LIFT, 0);
			motorSet(MC.TL_LIFT, 0);
			motorSet(MC.BR_LIFT, 0);
			motorSet(MC.MR_LIFT, 0);
			motorSet(MC.TR_LIFT, 0);
		}

		// Motors can only be updated once every 20ms
		delay(25);
	}

	// Re-do entire process if it failed to start
	operatorControl();
}
