#include "chassis.h"
#include "6050.h"

#define abs(x) ((x)>0? (x):(-(x)))

_chassis chassisPara = {0};

void YawUpdate(uint8_t flag){
	if(flag){
		chassisPara.yaw.angle_speed =  sensor.gyro.radian.z * 57.3f;
		if(abs(chassisPara.yaw.angle_speed) < 3.0f){
			chassisPara.yaw.angle_speed = 0;
		}
		chassisPara.yaw.angle += chassisPara.yaw.angle_speed / 1000.0f;
	}
}