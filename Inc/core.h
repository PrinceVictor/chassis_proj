#ifndef __core_H
#define __core_H

#include "stm32f4xx_hal.h"

typedef struct{

	int8_t state;							// 0 for init, more details see as above
	int8_t manualOrauto;			// 0 for manual, 1 for auto 
	int8_t remoteOrkeyboard;  // 0 for remote, 1 for keyboard and mouse

}_sysState;


#endif

