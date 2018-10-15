#include "core.h"
#include "communi.h"
#include "gpio.h"

#define init 0
#define run	 1
#define stop 2

extern uint8_t remote_rx_buffer[18];
_sysState sys = { 0 };

uint8_t Core_Task(uint8_t flag, uint32_t* time_count){
	if(flag){
		switch(sys.state){
			case init:{
				sys.state = run;
				break;
			}
			case run:{
				
				if(*time_count > 1000){
						//led0_switch;
						*time_count = 0;
				}
				
				Readremote(remote_rx_buffer);
				
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

