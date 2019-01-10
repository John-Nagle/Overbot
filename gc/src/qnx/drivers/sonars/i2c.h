#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <serial.h>
#include <strings.h>
using namespace std;

#define I2C_READ        1
#define I2C_WRITE       0

#define I2C_QUIT        0
#define I2C_CONTINUE    1

extern  void          i2c_init(char * dev);
extern  void          i2c_start(void);
extern  void          i2c_stop(void);
extern  void 		  i2c_transmit(unsigned char data);
extern  unsigned char i2c_receive(bool ack);
extern void			  i2c_set_dest(unsigned char dest);
extern void			  i2c_write(unsigned char data0, unsigned char data1);
extern void		      wait(unsigned int ms);
extern void			  i2c_ping();
extern void			  clear_response();
extern unsigned int   i2c_read_ping();

/* Macro definitions */

#define I2C_START(ADDRESS)     { i2c_start(); i2c_transmit(ADDRESS); }
#define I2C_START_TX(ADDRESS)  I2C_START(ADDRESS)
#define I2C_START_RX(ADDRESS)  I2C_START(ADDRESS | I2C_READ)


#endif //I2C_H