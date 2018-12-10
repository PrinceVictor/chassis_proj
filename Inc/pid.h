#ifndef __pid_H
#define __pid_H

#include "stm32f4xx_hal.h"

typedef struct pid_Para{
	float kp;
	float	ki;
	float kd;
	float ki_limit;
	float outlimit;
	uint8_t modeFlag;
	uint8_t otherflag;
	uint8_t i_flag;
	uint8_t d_flag;
}_pid_Para;

typedef struct pid_Out{
	float target;
	float feedback;
	float p_Out;
	float error;
	float last_error;
	float i_interval;
	float	i_Out;
	float d_Out;
	float Out;
}_pid_Out;

typedef struct pid{
	_pid_Out pid;
	_pid_Para k_para;
}_pid;

typedef	struct pidDouble{
	_pid	shell;
	_pid	core;	
}_pidDouble;

int16_t pidGet(_pid_Para* ,_pid_Out* ,float ,float);
float amplitudeLimiting(uint8_t , float , float);					

#endif

