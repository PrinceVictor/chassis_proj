#include "6050.h"
#include "i2c.h"


struct _sensor sensor;	

uint8_t	mpu6050_buffer[16];					
unsigned char TAddr;

/***************************************************************************************
  * @函数描述：  MPU6050初始化
  * @入口参数：  无.
  * @返回值  :   初始化完成标志，0----完成，!0----未完成.
****************************************************************************************/

void delayms(uint32_t count)
{ 
	uint32_t time_count = count *27190;
   while(time_count--) 
   { 
			__NOP; 
   }  
}

void delayus(uint32_t count)
{ 
	uint32_t time_count = count *27;
   while(time_count--) 
   { 
			__NOP; 
   }  
}

uint8_t InitMPU6050(void)
{
	//unsigned char TAddr;
	uint8_t date;
	do
	{
		date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);  	   //休眠
		delayms(500);
		date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	   //解除休眠状态0x00
		delayms(50);
		
		TAddr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
		                                  
	}while(TAddr!=0x68);
	
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //解除休眠状态0x00
	delayms(10);
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //采样频率（1KHz）
	delayms(10);
	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x02);         //低通滤波0x00
	delayms(10);
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x18);    //陀螺仪量程 
	delayms(10);
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);   //加速度量程 
	delayms(10);
	date += Single_Write(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);   
	
	return date;
}
/***************************************************************************************
  * @函数描述：  读取所有加速度计、陀螺仪
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
	
	
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
	
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
	
	
	
	
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	
	
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);//传感器2的z轴陀螺仪
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);//
	
	//mpu6050_buffer[14]=Single_Read(MPU6050_ADDRESS2, GYRO_ZOUT_H);//传感器2的z轴陀螺仪
	//mpu6050_buffer[15]=Single_Read(MPU6050_ADDRESS2, GYRO_ZOUT_L);//
	
	
	
	
}


/***************************************************************************************
  * @函数描述：  陀螺仪零点校准
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=1000;
	 int cnt = cnt_g;
	 float  tempgx=0,tempgy=0,tempgz;
	
	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	
	 while(cnt_g--)       //循环采集1000次   求平均
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
		 
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
}

