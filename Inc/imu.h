#ifndef __imu_H
#define __imu_H

#ifdef __cplusplus
 extern "C" {
#endif
	
#include "stm32f4xx_hal.h"

#define GYRO_FILTER_NUM 10
#define Kp 1.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kii 0.01f                     // integral gain governs rate of convergence of gyroscope biases
#define KALMAN_Q        0.02
#define KALMAN_R        6.0000

float invSqrt(float x);

#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制	

typedef struct{
	float roll;
	float pitch;
	float yaw;
}_angle;

typedef struct{
	float yaw_speed;
	float yaw;
	float yaw_last;
	uint16_t count;
	uint8_t flag;
	uint8_t cycle_calibration;
}_imu_yaw;

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void readIMU(uint8_t);
void imu(int8_t);
extern _imu_yaw imu_yaw;
extern _angle angle;
extern _angle dmp_angle;

#ifdef __cplusplus
}
#endif

#endif
