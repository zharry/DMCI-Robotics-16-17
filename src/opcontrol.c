#include <stdbool.h>

#include "../include/API.h"
#include "../include/constants.h"
#include "../include/main.h"
#include "../include/pid.h"

//void autonomous();

void operatorControl() {

	//autonomous();

	unsigned long prevWakeupTime = millis();

	// PID controller variable
	struct pid_dat arm_pid;
	initPID(&arm_pid, 2, 0.5, 0.02);

	while (1) {

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

		// Local Variables
		int moveX = 0, moveY = 0, rotate = 0, arm = 0, liftSpeed = 0;
		int L, R, C;
		bool liftUp = 0, liftDw = 0, supUp = 0, supDw = 0, liftCenter = 0, liftLower = 0;

		// Subteam Definition
		int team1 = digitalRead(1);

		// Get Joystick Values based on Status
		if (joystickStatus == 3) {
			// Driving
			moveX = joystickGetAnalog(1, JOY_LX); // Movement
			moveY = joystickGetAnalog(1, JOY_LY); // Movement
			rotate = joystickGetAnalog(1, JOY_RX); // Rotate
			arm = joystickGetAnalog(2, JOY_RY);

			// Support
			supUp = joystickGetDigital(1, JOY_LBUM, JOY_DOWN);
			supDw = joystickGetDigital(1, JOY_LBUM, JOY_UP);

			// Both joysticks connected, reference as 1 or 2
			liftUp = joystickGetDigital(1, JOY_RBUM, JOY_DOWN);
			liftDw = joystickGetDigital(1, JOY_RBUM, JOY_UP);

			liftCenter = joystickGetDigital(2, JOY_RBUM, JOY_UP);
			liftLower = joystickGetDigital(2, JOY_RBUM, JOY_DOWN);
		} else {
			// Driving
			moveX = joystickGetAnalog(joystickStatus, JOY_LX); // Movement
			moveY = joystickGetAnalog(joystickStatus, JOY_LY); // Movement
			rotate = joystickGetAnalog(joystickStatus, JOY_RX); // Rotate

			// Support
			supUp = joystickGetDigital(joystickStatus, JOY_LBUM, JOY_DOWN);
			supDw = joystickGetDigital(joystickStatus, JOY_LBUM, JOY_UP);

			// Lift
			liftUp = joystickGetDigital(joystickStatus, JOY_RBUM, JOY_DOWN);
			liftDw = joystickGetDigital(joystickStatus, JOY_RBUM, JOY_UP);
		}

		// Driving
		L = CAP(moveY + rotate, RANGE_MAX);
		R = CAP(moveY - rotate, RANGE_MAX);
		C = CAP(moveX, RANGE_MAX);
		motorSet(MC_WHEEL_L, team1 == ON ? L : L*2/3);
		motorSet(MC_WHEEL_R, team1 == ON ? -R : -R*2/3);
		motorSet(MC_WHEEL_M, team1 == ON ? -C : -C*2/3);

		// Support
		if (supDw)
			motorSet(MC_SUPPORT, team1 == ON ? -32 : 32);
		else if (supUp)
			motorSet(MC_SUPPORT, team1 == ON ? 32 : -32);
		else
			motorSet(MC_SUPPORT, 0);

		if (team1 == ON) {
			double loc = ((MAP_INPUT(arm) + 1.0) / 2.0) * 0.9 - 0.8;
			if(liftCenter) {
				loc = -0.35;
			} else if(liftLower) {
				loc = -0.45;
			}
			liftSpeed = -MAP_OUTPUT(computePID(loc, MAP_POT(analogRead(1)), &arm_pid ));
			
		}
		if (liftUp)
			liftSpeed = LIFTSPEED;
		else if (liftDw)
			liftSpeed = -LIFTSPEED;

		if(team1 != ON) {
			liftSpeed = -liftSpeed;
		}

		// Lift
		motorSet(MC_LIFT_BL, liftSpeed);
		motorSet(MC_LIFT_ML, liftSpeed);
		motorSet(MC_LIFT_TL, liftSpeed);
		motorSet(MC_LIFT_BR, -liftSpeed);
		motorSet(MC_LIFT_MR, -liftSpeed);
		motorSet(MC_LIFT_TR, -liftSpeed);

		// Motors can only be updated once every 20ms, therefore updating at twice the rate for better response time
		taskDelayUntil(&prevWakeupTime, 10);
	}

}
