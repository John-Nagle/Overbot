#ifdef OBSOLETE

#include "telegram.h"

#include <stdio.h>


int main (void) {
	unsigned char temp[16];
	
	temp[0] = 0x02;
	temp[1] = 0x00;
	temp[2] = 0x03;
	temp[3] = 0x00;
	temp[4] = 0x30;
	temp[5] = 0x00;
	temp[6] = 0x01;
	temp[7] = 0x71;
	temp[8] = 0x38;
	
	u_short calcdCRC = createCRC(temp, 7);
	
	printf("calcdCRC: %04x  orig: %04x\n", calcdCRC, *(unsigned short *)(temp+7));

return 0;
}
#endif // OBSOLETE