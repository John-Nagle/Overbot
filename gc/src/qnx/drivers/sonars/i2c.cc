/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Nov 4 03
 * Overbot
 *
 * All functions relating to i2c, this is dependent on the rs232 to i2c converter
 */

#include "i2c.h"

Serial * serial_p =  NULL;

/**
 * read the serial port
 * @return true if we get back the replyMsg (ignore case)
 */
bool read_response(char * replyMsg)
{
	char response[128];
	
	int len = strlen(replyMsg);
	for (int i = 0 ; i < len ; i++) {
		serial_p->ReadBuf(response + i, 1);
	}
	//printf("replyMsg: %s\n\n", replyMsg);
	//printf("response: %s\n\n", response);
	//compare and ignore case
	return strncasecmp(replyMsg, response, len) == 0;
}

/**
 * given a char in ascii like 'A', return 10
 */
unsigned char hex_ascii_2_num(char ascii)
{
	//printf("hex_ascii_2_num: %c\n", ascii);
	
	if (ascii >= '0' && ascii <= '9') {
		return ascii - '0';
	}
	if (ascii == 'A' || ascii == 'a') {
		return 10;
	}
	if (ascii == 'B' || ascii == 'b') {
		return 11;
	}
	if (ascii == 'C' || ascii == 'c') {
		return 12;
	}
	if (ascii == 'D' || ascii == 'd') {
		return 13;
	}
	if (ascii == 'E' || ascii == 'e') {
		return 14;	
	}
	if (ascii = 'F' || ascii == 'f') {
		return 15;
	}
	return 15;
}


/**
 * try to read a hex number from the serial port that looks like "0xXX\r"
 */
unsigned char read_hex_num()
{
	char response[8];
	
	int len = 5;
	
	for (int i = 0 ; i < len ; i++) {
		serial_p->ReadBuf(response + i, 1);
	}
	
	unsigned char data = hex_ascii_2_num(response[2]) * 16 + hex_ascii_2_num(response[3]);		
	
	//printf("response: %s\n", response);
	//printf("data: %d\n", data);	
	return data;
}

/**
 * clears the read buffer in the serial port
 */
void clear_response()
{
	char response[1024];
	if (serial_p->ReadBuf(&response, 1024) < 0) {
		cerr << "Failed to clear response" << endl;
		return;
	}
}

/**
 * all initialization needed for our i2c implementation
 * includes opening of serial port
 */
void i2c_init(char * dev)
{
	serial_p = new Serial(dev, 9600, "8N1", 100000);
	if (serial_p->Open() < 0) {
		cerr << "Failed to Open " << dev << endl;
		abort();
	}
	clear_response();
}

/**
 * waits for ms number of milliseconds
 */
void wait(unsigned int ms)
{
	if (ms > 999) { cerr << " wait takes in less than 999 ms " << endl; }
	//wait for a while
	struct timeval t;
	t.tv_usec = ms * 1000;
	t.tv_sec = 0;
	select (0, NULL, NULL,NULL, &t);
	
}	

/**
 * send out the start signal
 */
void i2c_start(void)
{
	char byte = 'S';
	if (serial_p->WriteBuf(&byte, 1) != 1) {
		cerr << "i2c_start: Failed to write S" << endl;
		return;
	}

	char expected[32] = "S!";
	if (!read_response(expected)) {
		cerr << "i2c_stop: Failed to read back S!" << endl;	
	}	
	
}

void i2c_stop(void)
{
	
	char byte = 'T';
	if (serial_p->WriteBuf(&byte, 1) != 1) {
		cerr << "i2c_stop: Failed to write T" << endl;
		return;
	}
	
	char expected[32] = "T!";
	if (!read_response(expected)) {
		cerr << "i2c_stop: Failed to read back T!" << endl;	
	}	

}

//obsolete calls that we can use for reference
/*
void i2c_set_dest(unsigned char dest)
{
	char buf[16];
	sprintf(buf, "D0x%x\n", dest >> 1);
	if ((unsigned int)serial_p->WriteBuf(buf, strlen(buf)) != strlen(buf)) {
		cerr << "Error i2c_set_dest " << endl;
	}
}
*/
/*
void i2c_write(unsigned char data0, unsigned char data1)
{
	char buf[32];
	sprintf(buf, "w0x%x,0x%x\r", data0, data1);
	if ((unsigned int)serial_p->WriteBuf(buf, strlen(buf)) != strlen(buf)) {
		cerr << "Error i2c_write " << endl;
	}
	//TODO fix this hackery
	wait(100);

}
*/

/**
 * transmit the data using the A command
 */
void i2c_transmit(unsigned char data)
{
	char send_buf[16];
	char expected_recv_buf[32];
	
	//we really want 0x0 to be sent as "0x00"
	if (data < 0x10) {
		sprintf(send_buf, "A0x0%x\r", data);
		sprintf(expected_recv_buf, "A0x0%x\r0x0%x\r",data,data);
	}
	else {
		sprintf(send_buf,"A0x%x\r", data);
		sprintf(expected_recv_buf, "A0x%x\r0x%x\r",data,data);
	}
	
	if ((unsigned int)serial_p->WriteBuf(send_buf, strlen(send_buf)) != strlen(send_buf)) {
		cerr << "i2c_transmit: Failed to transmit properly" << endl;
		return;
	}

	if (!read_response(expected_recv_buf)) {
		cerr << "i2c_transmit: failed to get expected response" << endl;
	}
}

/**
 * reads a value off the i2c bus
 * need to have 
 * if we are using ack, we use the B command
 * if we are not doing ack, we use the C command
 */
unsigned char i2c_receive(bool ack)
{
	char byte[2];
	//terminate the str
	byte[1] = '\0';
	if (ack) {
		byte[0] = 'B';
	}
	else {
		byte[0] = 'C';
	}
	
	if (serial_p->WriteBuf(byte, 1) < 1) {
		cerr << "Failed to write in receive " << endl;
		return 0xFF;
	}

	read_response(byte);
	return read_hex_num();
}
