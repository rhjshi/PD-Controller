#include <stdio.h>
#include <math.h>
//1 -> 4
#define Kp 2

//0.05 -> 0.2
#define Ki 0.05

//0.25 -> 1
#define Kd 0.5

double normalize(double angle){
	if (angle < 0){
		while (angle < 0){
			angle += 360;
		}
	}
	else{
		while (angle > 359){
			angle -= 360;
		}
	}
	return angle;
}

double compute(double *setPoint, double *input, double *lastError, double *errSum){
	//compute all working error variables
	double error = *setPoint - *input;
	*errSum += (error * 1);
	double dErr = (error - *lastError) / 1;

	//set last err as current err
	*lastError = error;

	//compute PID
	return (Kp * error + Ki * (*errSum) + Kd * dErr);
}
/*
int main(void){
	double setPoint = 0;
	double input = M_PI/8;
	double lastError = 0;
	double errSum = 0;

	for(int i = 0; i < 1000; ++i, input -= M_PI/8000){
		if (i % 2 == 1)input *= -1;
		printf("Angle: %f\nOutput: %f\n\n", input*180/M_PI, 180/M_PI*compute(&setPoint, &input, &lastError, &errSum));
		if (i % 2 == 1)input *= -1;
	}
	return 0;
}
*/
