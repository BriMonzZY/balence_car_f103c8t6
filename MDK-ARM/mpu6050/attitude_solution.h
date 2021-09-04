#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "main.h"

typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}eulerianAngles_t;




typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t_mpu6050;

typedef struct{
	float pitch;
	float roll;
	float yaw;
}eulerianAngles_t_mpu6050;
	
	


extern eulerianAngles_t eulerAngle;  /* 欧拉角结构体 */
extern eulerianAngles_t_mpu6050 eulerAngle_mpu6050;

extern float values[10];

void IMU_getValues(float * values);
void IMU_quaterToEulerianAngles(void);
void IMU_quaterToEulerianAngles_mpu6050(void);

#endif
