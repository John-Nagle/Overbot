/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Nov 4 03
 * Overbot
 *
 * All functions relating to talking to srf08 
 */ 

#include <srf08.h>

static unsigned char address=SRF08_UNIT_0;

void srf08_init(char * dev)
{
	i2c_init(dev);
}


//can be modified slightly to use
/*
void srf08_set_gain(unsigned char gain)
{
	if (gain > 31) gain = 31;
	I2C_START_TX(address);
	i2c_transmit(SRF08_GAIN);
	i2c_transmit(gain);
	i2c_stop();
	
}

void srf08_set_range(unsigned int millimeters)
{
    millimeters= (millimeters/43);
    if(millimeters > 0xff ) { millimeters=0xff; }
    I2C_START_TX(address);
    i2c_transmit(2);
    i2c_transmit(millimeters);
    i2c_stop();

}
*/

//for reference, might come in useful
/*
unsigned int srf08_read_register(unsigned char srf08_register)
{
	union i2c_union {
                   unsigned int  rx_word;
                   unsigned char rx_byte[2];
                } i2c;


    I2C_START_TX(address);
    i2c_transmit(srf08_register);
    I2C_START_RX(address);

    // get high byte msb first 
    if(srf08_register>=2) { i2c.rx_byte[1]=i2c_receive(I2C_CONTINUE); }


    //get low byte msb first
    i2c.rx_byte[0]=i2c_receive(I2C_QUIT);

    i2c_stop();

	return(i2c.rx_word);
}
*/

/**
 * send a ping to address, should call srf08_ping_recv on the same address to get
 * back the dist
 */
void srf08_ping_send(unsigned char address)
{
	i2c_start();
	i2c_transmit(address);
	i2c_transmit(SRF08_COMMAND);
	i2c_transmit(SRF08_CENTIMETERS);
	i2c_stop();
	
}

/**
 * must have first called srf08_ping_send previously
 * then call this to get the dist
 */
unsigned int srf08_ping_recv(unsigned char address)
{
	i2c_start();
	i2c_transmit(address);
	i2c_transmit(SRF08_ECHO_1);
	
	union i2c_union {
                   unsigned int  rx_word;
                   unsigned char rx_byte[2];
                } i2c;
	
	i2c_start();
	//to perform reading
	i2c_transmit(address|0x01); 
	//receive with ack
	i2c.rx_byte[1] = i2c_receive(true);
	//receive with no ack
	i2c.rx_byte[0] = i2c_receive(false);

	return i2c.rx_word;
}

unsigned int srf08_ping(unsigned char address)
{
	srf08_ping_send(address);
	//here we should wait a while for the echo's to be done
	//wait time as recommended by the devantech sonar controller documentations
	wait(65);
	return srf08_ping_recv(address);

}


/**
 * here we really are changing address by doing broadcasting
 * remember to do this with only one sonar controller on the bus
 * that sonar controller will change its address to new_i2c_address
 */	
void srf08_change_i2c_address(unsigned char new_i2c_address) {
	
    // Start the I2C address changing procedure
    i2c_start();
    i2c_transmit(address);
    i2c_transmit(SRF08_COMMAND);
    i2c_transmit(SRF08_CHANGE_ADDRESS_BYTE0);
    i2c_stop();
    
    i2c_start();
    i2c_transmit(address);
    i2c_transmit(SRF08_COMMAND);
    i2c_transmit(SRF08_CHANGE_ADDRESS_BYTE1);
    i2c_stop();
    
    i2c_start();
    i2c_transmit(address);
    i2c_transmit(SRF08_COMMAND);
    i2c_transmit(SRF08_CHANGE_ADDRESS_BYTE2);
    i2c_stop();
    
    i2c_start();
    i2c_transmit(address);
    i2c_transmit(SRF08_COMMAND);
    i2c_transmit(new_i2c_address);
    i2c_stop();
    
}


/**
 * the above version tries to use the A command but does not yet work
 * here is a shorter version that uses the W command
 */
 /*
void srf08_change_i2c_address(unsigned char new_i2c_address)
{

	//here we really are changing address by doing broadcasting
	//remember to do this with only one sonar controller on the bus
	
    // Start the I2C address changing procedure
    i2c_write(0x00,0xa0);
    i2c_write(0x00,0xaa);
    i2c_write(0x00,0xa5);
    i2c_write(0x00,new_i2c_address);

    // Make the new i2c address the active one.
    address=new_i2c_address;
}
*/

/**
 * changes the internal active address selection
 */

void srf08_select_unit(unsigned char srf08_address)
{
    // New address validity check

    if( (srf08_address<0xE0 || srf08_address>0XFE) && srf08_address != 0 )  { 
    	return; 
   	}
    if(srf08_address%2) { 
    	return; 
    }

    // Make the new i2c address the active one.
    address=srf08_address;
}
