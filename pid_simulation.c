#define _USE_MATH_DEFINES

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

float
p_ctrl(float setpoint, float actual, float kp)
{
	float error = setpoint - actual;
	return kp * error;
}

float
pi_ctrl(float setpoint, float actual, float kp,
	float ki, float *integral)
{
	float error = setpoint - actual;
	*integral += error;
	return kp * error + ki * *integral;
}
float
pid_ctrl(float setpoint, float actual, float kp,
	float ki, float kd, float *integral,
	float *prev_error)
{
	float error = setpoint - actual;
	*integral += error;
	float derivative = error - *prev_error;
	*prev_error = error;
	return kp * error + ki * *integral + kd * derivative;
}
int main(void) {
	float setpoint, actual, kp;
	printf("setpoint, actual, kp, ki: \n");
	scanf("%f %f %f", &setpoint, &actual, &kp);
	printf("result: %f\n", p_ctrl(setpoint, actual, kp));
}
