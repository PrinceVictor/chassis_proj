#include "chassis.h"
#include "6050.h"

#define abs(x) ((x)>0? (x):(-(x)))
#define LineSpeed2Rotor  2.19558f //micro meter
#define Rotor2LineSpeed  0.45546f // 1/19*76mm/60*2pi  micrometer
#define Degree2Radian  0.01745f 

typedef enum{
	length= 290,  //mirco meter
	width= 400
}vehicle_size;

_chassis chassisPara = {0};
_wheel_Info Wheel_Para = {
	.pid_para={
		.k_para={
			.kp=15.0f,
			.ki=0.3f,
			.kd=0.0f,
			.i_flag=0.0f,
			.d_flag=0.0f,
			.ki_limit=1000.0f,
			.outlimit=5000,
			.modeFlag=1
		}
	}
};

void WheelSolute(_wheel_Info*  ,_chassis* );
							
void YawUpdate(uint8_t flag){
	if(flag){
		chassisPara.yaw.angle_speed =  sensor.gyro.radian.z * 57.3f;
		if(abs(chassisPara.yaw.angle_speed) < 3.0f){
			chassisPara.yaw.angle_speed = 0;
		}
		chassisPara.yaw.angle += chassisPara.yaw.angle_speed / 1000.0f;
	}
}

uint8_t ChassisControl(uint8_t flag){
	if(flag){
		
		WheelSolute(&Wheel_Para,&chassisPara);
		
		for(int i=0; i<4; i++){
			Wheel_Para.wheel.pidOut[i]=pidGet(&Wheel_Para.pid_para.k_para,
																				&Wheel_Para.pid_para.pid[i],
																				Wheel_Para.wheel.target_speed[i],
																				(float)Wheel_Para.feedback.Speed[i]);
		}
	}
	else{
		for(int i=0; i<4; i++){
			Wheel_Para.wheel.pidOut[i]=0;
		}
	}
	
	return flag;
}

void WheelSolute(_wheel_Info* wheels ,_chassis* chassis){
	float temp[4];
	temp[0] = chassis->Fb-chassis->Lr-(length+width)*chassis->Rt*Degree2Radian/1000.0f;
	temp[1] = -(chassis->Fb+chassis->Lr+(length+width)*chassis->Rt*Degree2Radian/1000.0f);
	temp[2] = -(chassis->Fb-chassis->Lr+(length+width)*chassis->Rt*Degree2Radian/1000.0f);
	temp[3] = chassis->Fb+chassis->Lr-(length+width)*chassis->Rt*Degree2Radian/1000.0f;
	
	for(int i=0; i<4; i++ ){
		wheels->wheel.target_speed[i]=amplitudeLimiting(ENABLE,temp[i]*LineSpeed2Rotor*1000.0f,4000);
	}
}
