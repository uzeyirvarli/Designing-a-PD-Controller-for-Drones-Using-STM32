/*
 * pid.h
 *
 *  Created on: Jul 1, 2024
 *      Author: Uzeyir Varli
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float previous_error;
    float max_integral;
    float min_integral;
    float max_output;
    float min_output;
    float output;

} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd);
float PID_Compute(PIDController *pid, float setpoint, float input);

#endif /* INC_PID_H_ */
