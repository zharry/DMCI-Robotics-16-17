// PID controller functions for the VEX robot

#ifndef PID_H_
#define PID_H_

struct pid_dat {
	double kp;
	double ki;
	double kd;
	double pid_ni;
	double pid_nd;
	unsigned long prevTime;
};

void initPID(struct pid_dat *p, double kp, double ki, double kd);

double computePID(double set, double feedback, struct pid_dat *p);

#define MAP_POT(x) (x / 2047.5 - 1.0)
#define MAP_INPUT(x) (x / 127.0)
#define MAP_OUTPUT(x) ((int)(x * 127.0))

#endif
