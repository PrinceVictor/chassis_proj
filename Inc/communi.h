#ifndef __communi_H
#define __communi_H

#include "stm32f4xx_hal.h"

typedef struct RC_Ctl
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}_RC_Ctl;


uint8_t Readremote(const uint8_t*);
#endif

