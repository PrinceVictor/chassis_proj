#ifndef __mpu6050_H
#define __mpu6050_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			  0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define	PWR_MGMT_1		0x6B	 //电源管理，典型值：0x00(正常启用)
#define	PWR_MGMT_2		0x6C	 //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	   //IIC地址寄存器(默认数值0x68，只读)

#define	MPU6050_ADDRESS  0xD0	  //陀螺地址
#define MAG_ADDRESS    0x18       //磁场地址
#define ACCEL_ADDRESS  0xD0 

#define	MPU6050_ADDRESS2  0xD2	 


#define	GYRO_Y		0x45
#define	GYRO_Z		0x47

#define Acc_G 		0.0011963f		//  1/32768/2/9.8     加速度量程为4G		
#define Gyro_G 		0.06103515f	//  1/32768/2000      陀螺仪量程为 +—1000			
#define Gyro_Gr		0.00106519f   //  1/32768/2000/57.3 

typedef struct{
				float X;
				float Y;
				float Z;}FLOAT_XYZ;

struct _float{
	      float x;
				float y;
				float z;};

struct _int16_t{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16_t origin;  //原始值
	   struct _int16_t averag;  //平均值
	   struct _int16_t histor;  //历史值
	   struct _int16_t quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;					
extern struct _sensor sensor2;
							
							
extern uint8_t	mpu6050_buffer[16];					
							
uint8_t InitMPU6050(void);					   
void MPU6050_Read(void);

void Gyro_OFFEST(void);							


#ifdef __cplusplus
}
#endif
#endif
