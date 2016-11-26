/**
* File for autonomous code.
*
* This file should contain the user autonomous() function and any functions related to it.
*
* Copyright (c) 2011-2014, Purdue University ACM SIG BOTS. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of Purdue University ACM SIG BOTS nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be obtained from http://sourceforge.net/projects/freertos/files/ or on request.
********************************************************************************/

// Custom "main.h" header
#include "main.h"

double targetArm = 0;
bool armPidEnabled = false;

int moveX, moveY, rotate;

void update() {
	int L = CAP(moveY + rotate, RANGE_MAX);
	int R = CAP(moveY - rotate, RANGE_MAX);
	int C = CAP(moveX, RANGE_MAX);
	motorSet(MC_WHEEL_L, L);
	motorSet(MC_WHEEL_R, -R);
	motorSet(MC_WHEEL_M, -C);
}

void pidTask() {
	unsigned long prevWakeupTime = millis();

	struct pid_dat arm_pid;

	initPID(&arm_pid, 2, 0.5, 0.02);

	while(1) {
		if(armPidEnabled) {
			int liftSpeed = -MAP_OUTPUT(computePID((targetArm + 1.0) / 2.0 * 0.9 - 0.8, MAP_POT(analogRead(1)), &arm_pid));
	
			motorSet(MC_LIFT_ML, liftSpeed);
			motorSet(MC_LIFT_TL, liftSpeed);
			motorSet(MC_LIFT_BR, -liftSpeed);
			motorSet(MC_LIFT_MR, -liftSpeed);
			motorSet(MC_LIFT_TR, -liftSpeed); 

		}

		//update();

		taskDelayUntil(&prevWakeupTime, 10);
	}
}

/**
* Runs the user autonomous code.
*
* This function will be started in its own task with the default priority and stack size whenever the robot is enabled via the Field Management System or the VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart the task, not re-start it from where it left off.
*
* Code running in the autonomous task cannot access information from the VEX Joystick. However, the autonomous function can be invoked from another task if a VEX Competition Switch is not available, and it can access joystick information if called in this way.
*
* The autonomous task may exit, unlike operatorControl() which should never exit. If it does so, the robot will await a switch to another mode or disable/enable cycle.
*/

void autonomous() {

	unsigned long prevWakeupTime = millis();

	bool team2 = digitalRead(1);

//	armPidEnabled = !team2;

	moveY = 127;
	update();
	delay(1500);
	moveY = 0;
	update();
	motorSet(MC_SUPPORT, -127);
	delay(1000);
	motorSet(MC_SUPPORT, 0);



	//taskCreate(pidTask, TASK_DEFAULT_STACK_SIZE * 4, NULL, TASK_PRIORITY_DEFAULT);
	

//	targetArm = -0.5;
//	motorSet(MC_SUPPORT, 32);
//	delay(1000);

//	Encoder enc1 = encoderInit(5, 6, false);

//	int calib = encoderGet(enc1);
//	while(1) {
//		printf("%d\r\n", encoderGet(enc1));
//		delay(10);
//	}
//	struct pid_dat wheel_pid;
//	initPID(&wheel_pid, 0.6, 0, 0);
//	double setDist = 1000;
//
//	targetArm = 0;
//	
//	for(int i = 0; i < 500; ++i) {
//		moveY = computePID(setDist, encoderGet(enc1) - calib, &wheel_pid);
//		taskDelayUntil(&prevWakeupTime, 10);
//	}
//
//	setDist = -1000;
//	for(int i = 0; i < 500; ++i) {
//		moveY = computePID(setDist, encoderGet(enc1) - calib, &wheel_pid);
//		taskDelayUntil(&prevWakeupTime, 10);
//	}
//
//	moveY = 0;
//
//    while(true);
}
