#ifndef __chassis_H
#define __chassis_H

#include "stm32f4xx_hal.h"
#include "pid.h"

typedef struct{
	float target;
	uint8_t target_changeMode;
	float last_target;
	float temp;
	float angle;
	float angle_speed;
	float angle_last;
}_chassisYaw;

typedef struct{
	float Fb;
	float Lr;
	float Rt;
	float x,y;
	_chassisYaw yaw;
	float observe;
	_pidDouble pid;
	uint8_t pid_flag;
}_chassis;

typedef struct{
	float target_vx;
	float target_vy;
	float x;
	float y;
	float Vx;
	float Vy;
	float Wz;
	float theta;
}_wheel_solve;


#endif

