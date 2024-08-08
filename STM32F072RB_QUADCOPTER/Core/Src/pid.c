/*
 * pid.c
 *
 *  Created on: Jul 1, 2024
 *      Author: Uzeyir Varli
 */


#include "pid.h"

void PID_Init(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
}

float PID_Compute(PIDController *pid, float setpoint, float input) {
	if (pid->ki<= 0.055 && pid->ki>=0.003 )
	{
		pid->min_integral=-3636;
		pid->max_integral=3636;
	}


	else if (pid->ki<=0.003)
	{
		pid->min_integral=-100000;
		pid->max_integral=100000;
	}
	pid->setpoint = setpoint;

	pid->min_output=-200;

	pid->max_output=200;

    float error = pid->setpoint - input;
    pid->integral += error;
    float derivative = error - pid->previous_error;

    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->previous_error = error;

    if (pid->integral<pid->min_integral) pid->integral=pid->min_integral;
    if (pid->integral>pid->max_integral) pid->integral=pid->max_integral;
    if (pid->output<pid->min_output) pid->output=pid->min_output;
    if (pid->output>pid->max_output) pid->output=pid->max_output;
    return pid->output;
}
