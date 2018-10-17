#ifndef __core_H
#define __core_H

#include "stm32f4xx_hal.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

typedef struct{

	int8_t state;							// 0 for init, more details see as above
	int8_t manualOrauto;			// 0 for manual, 1 for auto 
	int8_t remoteOrkeyboard;  // 0 for remote, 1 for keyboard and mouse

}_sysState;

typedef enum{
	Attitude_dataUpload = 0x401
}Tx_Mode;

#endif

