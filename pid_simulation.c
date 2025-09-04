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
main() {
	/* printf wont show after scanf on musl */
	setvbuf(stdout, NULL, _IONBF, 0);

	float setpoint, actual, kp, ki, kd, integral = 0, prev_error = 0;
	float result = 0;
	int counter = 0;
	char option[6];
	char input[100];

	printf("pid simulation\n"
		"p - p controller \n"
		"pi - pi controller\n"
		"pid - pid controller\n"
		"q - quit\n");
	
	while(1) {
		fprintf(stdout, "option: ");
		fgets(option, sizeof(option), stdin);
		option[strcspn(option, "\n")] = '\0';

		if (strcmp(option, "p") == 0)  {
			
			printf("setpoint: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &setpoint);

			printf("actual: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &actual);


			printf("kp: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &kp);

			result = p_ctrl(setpoint, actual, kp);
			printf("counter - setpoint - actual - kp - error(?)\n");
			printf("%.2f\n", result);
			while(1) {
				counter++;
				actual += result;
				printf("%d %f %f %f %f\n", counter, setpoint, actual, kp, setpoint-actual);
				result = p_ctrl(setpoint, actual, kp);
				if(fabs(setpoint - actual) < 0.01) {
					counter++;
					printf("good job :)\n");
					printf("%d %f %f %f %f\n", counter, setpoint, actual, result, setpoint-actual);
					counter = 0;
					break;
				}
			}
		}

		else if(strcmp(option, "pi") == 0) {

			printf("setpoint: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &setpoint);

			printf("actual: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &actual);

			printf("kp: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &kp);
		
			printf("ki: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &ki);
			
			printf("integral: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &integral);
				
			result = pi_ctrl(setpoint, actual, kp, ki, &integral);
			printf("counter - setpoint - actual - kp - ki - integral\n");
			printf("%.2f", result);
			while(1) {
				counter++;
				actual += result;
				result = pi_ctrl(setpoint, actual, kp, ki, &integral);
				printf("%d %f %f %f %f, %f\n", counter, setpoint, actual, kp,
						ki, integral);
				if(fabs(setpoint - actual) < 0.01) {
					counter++;
					printf("good job\n");
					printf("%d %f %f %f %f, %f\n", counter, setpoint, actual, kp,
							ki, integral);
					counter = 0;
					break;
				}
			}
		}

		else if(strcmp(option, "pid") == 0) {
			printf("setpoint: ");
			scanf("%f", &setpoint);
			printf("actual: ");
			scanf("%f", &actual);
			printf("kp: ");
			scanf("%f", &kp);
			printf("ki: ");
			scanf("%f", &ki);
			printf("integral: ");
			scanf("%f", &integral);

			prev_error = 0;
			result = pid_ctrl(setpoint, actual, kp, ki, kd,
					&integral, &prev_error);
			while(1) {
				counter++;
				actual += result;
				result = pid_ctrl(setpoint, actual, kp, ki, kd,
						&integral, &prev_error);
				printf("%d %f %f %f %f %f %f %f\n", counter, setpoint, actual, kp,
						ki, kd, integral, prev_error);
				if(fabs(setpoint - actual) < 0.01) {
					printf("good job\n");
					counter = 0;
					break;
				}
			}
		}
		else if(strcmp(option, "q") == 0) {
			printf("goodbye :(\n");
			break;
		}

		else {
			printf("wrong option :(\n");
		}
	}
	return 0;
}
