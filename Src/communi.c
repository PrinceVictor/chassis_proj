#include "communi.h"

_RC_Ctl remote = {0};
uint8_t remoteData_receiveFlag = 0;

uint8_t Readremote(const uint8_t* buffer){
	if(remoteData_receiveFlag){
		remote.rc.ch0 = (buffer[0]| (buffer[1] << 8)) & 0x07ff; //!< Channel 0
		remote.rc.ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07ff; //!< Channel 1
		remote.rc.ch2 = ((buffer[2] >> 6) | (buffer[3] << 2) |(buffer[4] << 10)) & 0x07ff; //!< Channel 2
		remote.rc.ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07ff; //!< Channel 3
		remote.rc.s1 = ((buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		remote.rc.s2 = ((buffer[5] >> 4)& 0x0003); //!< Switch right
		remote.mouse.x = -(buffer[6] | (buffer[7] << 8)); //!< Mouse X axis
		remote.mouse.y = buffer[8] | (buffer[9] << 8); //!< Mouse Y axis
		remote.mouse.z = buffer[10] | (buffer[11] << 8); //!< Mouse Z axis
		remote.mouse.press_l = buffer[12]; //!< Mouse Left Is Press 
		remote.mouse.press_r = buffer[13]; //!< Mouse Right Is Press 
		remote.key.v = buffer[14] | (buffer[15] << 8); //!< KeyBoard value
		
		//clear remote receive complete flag	;
		remoteData_receiveFlag = 0;
		return 1;
	}
	return remoteData_receiveFlag;
}
