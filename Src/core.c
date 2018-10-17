#include "core.h"
#include "communi.h"
#include "gpio.h"
#include "can.h"
#include "imu.h"
#include "usart.h"
#include "6050.h"
#include "string.h"
#include "tim.h"

#define init 0
#define run	 1
#define stop 2

extern DMA_HandleTypeDef hdma_usart2_tx;

extern uint8_t usart2_tx_buffer[usart2_tx_bufferLength]; 
extern uint8_t remote_rx_buffer[18];
extern int16_t IMU_TxInit[4];
extern uint8_t uart2_tx_busyFlag;
_sysState sys = { 0 };

void YawUpdate(uint8_t);
HAL_StatusTypeDef Usart_Tx(uint8_t* , Tx_Mode , UART_HandleTypeDef *, uint8_t* ,uint16_t );

uint8_t RemoteFeed(uint8_t);
uint32_t Get_Freqz(Freqz*);
uint32_t check_timer =0, last_check_time =0, period =0;
uint32_t check_timer1 =0, last_check_time1 =0, period1 =0;
Freqz tim4_fs;

uint8_t Core_Task(uint8_t flag, uint32_t* time_count){
	if(flag){
		switch(sys.state){
			case init:{
				
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
				if(imu_yaw.flag) sys.state	= run;
				
				break;
			}
			case run:{
				
				if(*time_count > 1000){
						//led0_switch;
						*time_count = 0;
				}
				
				if(RemoteFeed(Readremote(remote_rx_buffer))){
					
				}
				else{
					
				}
				Get_Freqz(&tim4_fs);
				//YawUpdate(ENABLE);
				Usart_Tx(&uart2_tx_busyFlag, Attitude_dataUpload, &huart2, usart2_tx_buffer, usart2_tx_bufferLength);
				break;
			}
			case stop:{
				break;
			}
			default: break;
		}
	}
	return flag;
}


uint8_t RemoteFeed(uint8_t flag){
	static uint16_t feed_count = 25;
	if(flag){
		feed_count = 25;
		return ENABLE;
	}
	else if(feed_count == 0){
		return DISABLE;
	}
	else {
		feed_count--;
	}
	return ENABLE;
}


HAL_StatusTypeDef Usart_Tx(uint8_t* flag, Tx_Mode mode, UART_HandleTypeDef *huart, uint8_t* tx_data,uint16_t size){
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
					//int16 mpu6050 gyro data
					data[5] = 0xb0;
					data[6] = sensor.gyro.origin.x >> 8;
					data[7] = sensor.gyro.origin.x ;
					data[8] = sensor.gyro.origin.y >> 8;
					data[9] = sensor.gyro.origin.y ;
					data[10] = sensor.gyro.origin.z >> 8;
					data[11] = sensor.gyro.origin.z ;
					//int16 mpu6050 Accelerometer data
					data[12] = sensor.acc.origin.x >> 8;
					data[13] = sensor.acc.origin.x ;
					data[14] = sensor.acc.origin.y >> 8;
					data[15] = sensor.acc.origin.y ;
					data[16] = sensor.acc.origin.z >> 8;
					data[17] = sensor.acc.origin.z ;
					data[18] = 0xc0;
					memcpy(tx_data , data, size);
					break;
				}
				default :break;
			}
			*flag = 1;
			return HAL_UART_Transmit_DMA(huart,tx_data,size);
	}
	return NULL;
}
