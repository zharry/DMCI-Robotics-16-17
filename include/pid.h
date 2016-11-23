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

#endif
