// Functions relating to the LMS telegrams.

#ifndef	TELEGRAM_H
#define TELEGRAM_H

#include <sys/types.h>

//
//	CreateCRC  --  compute cyclic redundancy check
//
//	Per SICK Data Telegram manual, p. 14.
//
inline u_short mkshort(u_short a, u_short b) {	return(a | (b<<8));	}
//
//	createCRC  --  compute cyclic redundancy check
//	Per SICK Data Telegram manual, p. 14.
//
inline u_short createCRC(const u_char data[], size_t len)
{	const u_short CRC16_GEN_POL = 0x8005;
	u_short ucrc16 = 0;
	u_char abData[2] = {0,0};
	for (unsigned int i=0; i<len; i++)
	{	abData[1] = abData[0];
		abData[0] = data[i];
		if (ucrc16 & 0x8000)
		{	ucrc16 = (ucrc16 & 0x7fff) << 1;
			ucrc16 ^= CRC16_GEN_POL;
		} else { ucrc16 <<= 1; }
		ucrc16 ^= mkshort(abData[0],abData[1]);
	}
	return(ucrc16);
}

#endif