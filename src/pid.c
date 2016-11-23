// PID Conroller

#include "pid.h"
#include "API.h"

void initPID(struct pid_dat *p, double kp, double ki, double kd) {
	p->kp = kp;
	p->ki = ki;
	p->kd = kd;
	p->pid_ni = 0;
	p->pid_nd = 0;
	p->prevTime = millis();
}

double computePID(double set, double feedback, struct pid_dat *p) {
	double err = set - feedback;
	unsigned long now = millis();
	double dt = (now - p->prevTime) / 1000.0;
	p->pid_ni += err * dt;
	double derr = (err = p->pid_nd) / dt;
	p->pid_nd = err;
	p->prevTime = now;
	return p->kp * err + p->ki * p->pid_ni + p->kd * derr;
}
