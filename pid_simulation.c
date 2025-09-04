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

int
main(void) {
	float setpoint, actual, kp, ki, kd, *integral, *prev_error;
	float result = 0;
	int counter = 0;
	char *option = malloc(sizeof(char) * 8);
	printf("pid simulation\np - p controller \npi - pi controller\n"
			"pid - pid controller\n");
	while(1) {
		printf("option: ");
		fgets(option, sizeof(option), stdin);
		option[strcspn(option, "\n")] = 0;
		if (strcmp(option, "p") == 0)  {
			printf("setpoint: ");
			scanf("%f", &setpoint);
			printf("actual: ");
			scanf("%f", &actual);
			printf("kp: ");
			scanf("%f", &kp);
			result = p_ctrl(setpoint, actual, kp);
			printf("hasilnya: %.2f\n", result);
			while(1) {
				counter++;
				actual += result;
				printf("%d %f %f %f\n", counter, setpoint, actual, kp);
				result = p_ctrl(setpoint, actual, kp);
				if(fabs(setpoint - actual) < 0.01) {
					printf("good job :)\n");
					printf("%d %f %f %f\n", counter, setpoint, actual, result);
					break;
				}
			}
		}
		else if(strcmp(option, "pi") == 0) {
		}
		else if(strcmp(option, "pid") == 0) {
		}
		else {
			printf("goodbye :(\n");
			break;
		}
	}
}
