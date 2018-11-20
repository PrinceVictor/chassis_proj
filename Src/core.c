#include "core.h"
#include "communi.h"
#include "chassis.h"
#include "gpio.h"
#include "can.h"
#include "imu.h"
#include "usart.h"
#include "6050.h"
#include "string.h"
#include "tim.h"

typedef enum{

	core_init = 0,
	core_run = 1,
	core_stop = 2

}core_state;

extern DMA_HandleTypeDef hdma_usart2_tx;

extern uint8_t usart2_tx_buffer[usart2_tx_bufferLength]; 
extern uint8_t remote_rx_buffer[18];
extern int16_t IMU_TxInit[4];
extern uint8_t uart2_tx_busyFlag;

extern uint8_t mian_func_start;

void YawUpdate(uint8_t);
uint8_t Usart_Tx(uint8_t* , Tx_Mode , UART_HandleTypeDef *, uint8_t* ,uint16_t );

uint8_t RemoteFeed(uint8_t);
uint32_t Get_Freqz(Freqz*);
uint8_t RemoteControl(uint8_t , _RC_Ctl* , _chassis* );

_sysState sys = { 0 };

Freqz tim4_fs;

uint8_t Core_Task(uint8_t flag, uint32_t* time_count){
	if(flag){
		switch(sys.state){
			case core_init:{
				
				if(imu_yaw.count == 3000){
					can_send_msg(ENABLE, &hcan1, CAN_IMU_TxID, IMU_TxInit);
					//sys.state	= run;
					imu_yaw.count--;
				}
				else if(imu_yaw.count > 0){
					imu_yaw.count --;
				}
				else{
					imu_yaw.count = 3000; // require that complete the reset IMU device in 3 seconds
				}
				if(imu_yaw.flag && mian_func_start) sys.state	= core_run;
				
				break;
			}
			case core_run:{
				
				if(*time_count > 1000){
						//led0_switch;
						*time_count = 0;
				}
				
				if(RemoteFeed(Readremote(remote_rx_buffer))){		
					can_send_msg(ChassisControl(RemoteControl(ENABLE,&remote,&chassisPara)), 
											&hcan2,
											CAN_WHEEL_TxID, 
											Wheel_Para.wheel.pidOut);
				}
				else{
					can_send_msg(ChassisControl(DISABLE), &hcan2, CAN_WHEEL_TxID, Wheel_Para.wheel.pidOut);
				}
				//Get_Freqz(&tim4_fs);
				//YawUpdate(ENABLE);
				Usart_Tx(&uart2_tx_busyFlag, Attitude_dataUpload, &huart2, usart2_tx_buffer, usart2_tx_bufferLength);
				break;
			}
			case core_stop:{
				
				break;
			}
			default: break;
		}
	}
	return flag;
}

uint8_t Usart_Tx(uint8_t* flag, Tx_Mode mode, UART_HandleTypeDef *huart, uint8_t* tx_data,uint16_t size){
	uint8_t data[size];
	if(!(*flag)){
			switch(mode){
				//freqz 1khz 
				case Attitude_dataUpload:{
					data[0] = 0xa0;
					//float IMU data
					data[1] = BYTE0(*(&imu_yaw.yaw));  
					data[2] = BYTE1(*(&imu_yaw.yaw));
					data[3] = BYTE2(*(&imu_yaw.yaw));
					data[4] = BYTE3(*(&imu_yaw.yaw));
					
					
					data[5] = 0xb0;
					//wheels information
					data[6] = Wheel_Para.feedback.Speed[0]>>8;  
					data[7] = Wheel_Para.feedback.Speed[0];
					data[8] = Wheel_Para.feedback.Speed[1]>>8;
					data[9] = Wheel_Para.feedback.Speed[1];
					data[10] = Wheel_Para.feedback.Speed[2]>>8;  
					data[11] = Wheel_Para.feedback.Speed[2];
					data[12] = Wheel_Para.feedback.Speed[3]>>8;
					data[13] = Wheel_Para.feedback.Speed[3];
					
					//time
					uint32_t timcnt = TIM2->CNT;
					data[14] = timcnt >>24;  
					data[15] = timcnt >>16;
					data[16] = timcnt >>8;
					data[17] = timcnt;
					//int16 mpu6050 dmp data 
//					data[6] = BYTE0(*(&dmp_angle.yaw));  
//					data[7] = BYTE1(*(&dmp_angle.yaw));
//					data[8] = BYTE2(*(&dmp_angle.yaw));
//					data[9] = BYTE3(*(&dmp_angle.yaw));
//					data[10] = BYTE0(*(&dmp_angle.pitch));  
//					data[11] = BYTE1(*(&dmp_angle.pitch));
//					data[12] = BYTE2(*(&dmp_angle.pitch));
//					data[13] = BYTE3(*(&dmp_angle.pitch));
//					data[14] = BYTE0(*(&dmp_angle.roll));  
//					data[15] = BYTE1(*(&dmp_angle.roll));
//					data[16] = BYTE2(*(&dmp_angle.roll));
//					data[17] = BYTE3(*(&dmp_angle.roll));
					
					//int16 mpu6050 gyro data
					data[18] = 0xc0;
					data[19] = sensor.gyro.origin.x >> 8;
					data[20] = sensor.gyro.origin.x ;
					data[21] = sensor.gyro.origin.y >> 8;
					data[22] = sensor.gyro.origin.y ;
					data[23] = sensor.gyro.origin.z >> 8;
					data[24] = sensor.gyro.origin.z ;
					//int16 mpu6050 Accelerometer data
					data[25] = sensor.acc.origin.x >> 8;
					data[26] = sensor.acc.origin.x ;
					data[27] = sensor.acc.origin.y >> 8;
					data[28] = sensor.acc.origin.y ;
					data[29] = sensor.acc.origin.z >> 8;
					data[30] = sensor.acc.origin.z ;
					data[31] = 0xd0;
					data[32] = BYTE0(*(&angle.yaw));  
					data[33] = BYTE1(*(&angle.yaw));
					data[34] = BYTE2(*(&angle.yaw));
					data[35] = BYTE3(*(&angle.yaw));
					data[36] = BYTE0(*(&angle.pitch));  
					data[37] = BYTE1(*(&angle.pitch));
					data[38] = BYTE2(*(&angle.pitch));
					data[39] = BYTE3(*(&angle.pitch));
					data[40] = BYTE0(*(&angle.roll));  
					data[41] = BYTE1(*(&angle.roll));
					data[42] = BYTE2(*(&angle.roll));
					data[43] = BYTE3(*(&angle.roll));
					data[44] = 0xe0;
					memcpy(tx_data , data, size);
					HAL_UART_Transmit_DMA(huart,tx_data,size);
					__HAL_DMA_ENABLE(&hdma_usart2_tx);
					break;
				}
				default :break;
			}
			*flag = 1;
	}
	return *flag;
}
