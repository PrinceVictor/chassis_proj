#include "core.h"
#include "communi.h"
#include "gpio.h"
#include "can.h"
#include "imu.h"

#define init 0
#define run	 1
#define stop 2

extern uint8_t remote_rx_buffer[18];
extern int16_t IMU_TxInit[4];
_sysState sys = { 0 };

void YawUpdate(uint8_t);
uint8_t RemoteFeed(uint8_t);

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
					imu_yaw.count = 3000;
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
				
				YawUpdate(ENABLE);
				
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
