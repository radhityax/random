/*
 * setpoint -> e(t) -> controller: (p,pi,pid) -> u(t) -> plant -> out(y(t))
 *		^					   ^
 *		|					   |
 *		|----------- feedback ---------------------|
 * setpoint : the value that we want
 * e(t) : difference between setpoint and actual value
 * plant : an actual system that we give a control, then produces an output
*/

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define err_tolerant 0.01

/* konstanta waktu(?) */
#define tau 1.0
/* gain sistem (?) */
#define k 2.0
#define dt 0.1
#define little_omega_n 1.0
#define xi 1.0
pthread_t thread1;

/* u(t) = kp * e(t) */
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
	*integral += error * dt;
	return kp * error + ki * *integral;
}

float
pid_ctrl(float setpoint, float actual, float kp,
	float ki, float kd, float *integral,
	float *prev_error)
{
	float error = setpoint - actual;
	*integral += error * dt;
	float derivative = (error - *prev_error) / dt;
	*prev_error = error;
	return kp * error + ki * *integral + kd * derivative;
}

/* transfer function ordo pertama / first order system ?? */
/* DALAM MODEL LAPLANCE */
float first_order(double s) {
	return k / (1 + tau * s);
}

/* transfer function ordo kedua / second order system ?? */
/* DALAM MODEL LAPLANCE */
float second_order(double s) {
	return little_omega_n * little_omega_n / 
	(s * s + 2 * xi * little_omega_n * s + little_omega_n * little_omega_n);
}


void
activity(void *arg) {
}

int
main() {
	/* printf wont show after scanf on musl */
	setvbuf(stdout, NULL, _IONBF, 0);

	float setpoint, actual, kp, ki, kd, integral = 0, prev_error = 0;
	float result = 0;
	unsigned int counter = 0;
	char option[6];
	char input[100];

	fprintf(stdout, "pid simulation\n"
		"p - p controller \n"
		"pi - pi controller\n"
		"pid - pid controller\n"
		"q - quit\n");
	
	while(1) {
		fprintf(stdout, "option: ");
		fgets(option, sizeof(option), stdin);
		option[strcspn(option, "\n")] = '\0';

		if (strcmp(option, "p") == 0)  {
			
			fprintf(stdout, "setpoint: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &setpoint);

			fprintf(stdout, "actual: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &actual);


			fprintf(stdout, "kp: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &kp);

			result = p_ctrl(setpoint, actual, kp);
			fprintf(stdout, "counter - setpoint - actual - kp - error\n");
			fprintf(stdout, "%d %f %f %f %f\n", counter, 
				setpoint, actual, kp, setpoint-actual);
			while(1) {
				counter++;
				result = p_ctrl(setpoint, actual, kp);
				/* ga tau ini nulis apa, harusnya di-diskrit?*/
				actual = actual + (dt/tau) * (-actual + k * result);
				fprintf(stdout, "%d %f %f %f %f\n", 
					counter, setpoint, actual, 
					kp, setpoint-actual);
				if(fabs(setpoint - actual) < err_tolerant) {
					counter++;
					printf("good job :)\n");
					fprintf(stdout, "%d %f %f %f %f\n",
						counter, setpoint, actual, 
						result, setpoint-actual);
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
			/*
			   printf("integral: ");
			   fgets(input, sizeof(input), stdin);
			   input[strcspn(input, "\n")] = '\0';
			   sscanf(input, "%f", &integral);
			   */
			integral = 0;
			result = pi_ctrl(setpoint, actual, kp, ki, &integral);
			fprintf(stdout, "counter - setpoint - actual -" 
				"kp - ki - integral\n");
			printf("%.2f", result);
			while(1) {
				counter++;
				result = pi_ctrl(setpoint, actual, kp, ki,
						 &integral);
				actual = actual + (dt/tau) * (-actual + k * result);
				fprintf(stdout, "%d %f %f %f %f, %f\n", 
						counter, setpoint, actual, kp,
						ki, integral);
				if(fabs(setpoint - actual) < err_tolerant) {
					counter++;
					fprintf(stdout, "good job\n");
					fprintf(stdout, "%d %f %f %f %f, %f\n",
							counter, setpoint, actual, kp,
							ki, integral);
					counter = 0;
					break;
				}
			}
		}

		else if(strcmp(option, "pid") == 0) {
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

			printf("kd: ");
			fgets(input, sizeof(input), stdin);
			input[strcspn(input, "\n")] = '\0';
			sscanf(input, "%f", &kd);

			integral = 0;
			prev_error = 0;
			result = pid_ctrl(setpoint, actual, kp, ki, kd,
					&integral, &prev_error);
			while(1) {
				counter++;
				actual = actual + (dt/tau) * (-actual + k * result);
				result = pid_ctrl(setpoint, actual, kp, ki, kd,
						&integral, &prev_error);
				printf("%d %f %f %f %f %f %f %f\n", counter, setpoint, actual, kp,
						ki, kd, integral, prev_error);
				if(fabs(setpoint - actual) < err_tolerant) {
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
