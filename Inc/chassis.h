#ifndef __chassis_H
#define __chassis_H

#include "stm32f4xx_hal.h"
#include "pid.h"

typedef struct{
	float target;
	float last_target;
	float temp;
	float angle;
	float angle_speed;
	float angle_last;
}_chassisYaw;

typedef struct{
	_chassisYaw yaw;
	float Fb;
	float Lr;
	float Rt;
	float x,y;
	float observe;
	uint8_t pid_flag;
}_chassis;

typedef struct{
	int16_t Speed[4];
	int16_t Postion[4];
}_feedback;

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

typedef struct{
	float target_speed[4];
	int16_t pidOut[4]; 
}_Wheel_cope;

typedef struct{
	_pid_Para k_para;
	_pid_Out pid[4];
}_Wheel_pid;

typedef struct{
	_Wheel_pid pid_para;
	_wheel_solve solved_para;
	_Wheel_cope wheel;
	_feedback feedback;
}_wheel_Info;

extern _wheel_Info Wheel_Para;
extern _chassis chassisPara;

uint8_t ChassisControl(uint8_t);

#endif

