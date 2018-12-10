#include "pid.h"

#define abs(x) ((x)>0? (x):(-(x)))

int16_t pidGet(_pid_Para* pidPara,
							_pid_Out* pidOut,
							float target,
							float feedback)
{
	float last_target = pidOut->target;
	pidOut->target = target;
	pidOut->feedback = (float)feedback;
	pidOut->error = target - (float)feedback;
	
	if(!pidPara->modeFlag){
		return 0;
	}
	else{
		switch(pidPara->modeFlag){
		case 1: { 
			pidOut->error = pidOut->error * 0.7f;
			if(pidOut->target == 0){
				if(abs(pidOut->error)<100.0f) pidOut->error = 0;
			}
			if((abs(pidOut->error) < 400) && (pidOut->target != 0)) pidPara ->i_flag = 1;
			else {
				pidPara ->i_flag = 0;
				pidOut -> i_interval = 0;
				pidOut ->i_Out = 0;
			}
			break;
		}
		case 2: {
			pidOut->error = pidOut->error * 1.0f;
			if(abs(pidOut->error)<0.5f && (pidPara->otherflag == 0)) pidOut->error = 0;
			if((abs(pidOut->error) > 30) && (pidPara->otherflag != 0)) pidPara ->i_flag = 1;
			else {
				pidPara ->i_flag = 0;
				pidOut -> i_interval = 0;
				pidOut ->i_Out = 0;
			}
			break;
		}		
		default : break;
		}
	}
	pidOut->p_Out = pidPara->kp * pidOut->error;
	pidOut->Out = pidOut->p_Out;

	if(pidPara->i_flag){
		pidOut->i_interval += pidOut->error;
		pidOut->i_Out = pidOut->i_interval * pidPara->ki;
		pidOut->i_Out = amplitudeLimiting(pidPara->i_flag,
											pidOut->i_Out,
											pidPara->ki_limit);
		pidOut->Out = pidOut->Out + pidOut->i_Out;
	}
	if(pidPara->d_flag){
		pidOut->d_Out = (pidOut->error - pidOut->last_error)* pidPara->kd;
		pidOut->last_error = pidOut->error;
		pidOut->Out = pidOut->p_Out + pidOut->d_Out;
	}
	pidOut->Out = amplitudeLimiting(pidPara->modeFlag,
										pidOut->Out,
										pidPara->outlimit);
	return (int16_t)pidOut->Out;
}

float amplitudeLimiting(uint8_t flag , float input , float limit)
{
	if(!flag)	return input;
	if( input > limit)	return limit;
	else if( input < -limit )	return-limit;
	else	return input;
}

