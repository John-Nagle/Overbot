#ifndef SRF08_H
#define SRF08_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "i2c.h"
using namespace std;


#define SRF08_UNIT_0   0xE0  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_1   0xE2  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_2   0xE4  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_3   0xE6  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_4   0xE8  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_5   0xEA  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_6   0xEC  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_7   0xEE  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_8   0xF0  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_9   0xF2  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_10  0xF4  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_11  0xF6  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_12  0xF8  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_13  0xFA  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_14  0xFC  /* the SRF08 MODULE I2C address */
#define SRF08_UNIT_15  0xFE  /* the SRF08 MODULE I2C address */

#define SRF08_I2C_BROADCAST_ADDRESS   0X00

#define SRF08_MIN_GAIN        0      /* sets gain to 94   */
#define SRF08_MAX_GAIN        31     /* sets gain to 1025 */
#define SRF08_MIN_RANGE       0      /* in millimeters    */
#define SRF08_MAX_RANGE       11008  /* in millimeters    */

#define SRF08_INCHES          0X50
#define SRF08_CENTIMETERS     0X51
#define SRF08_MICROSECONDS    0X52

/* register positions */
#define SRF08_COMMAND         0
#define SRF08_LIGHT           1
#define SRF08_GAIN            1
#define SRF08_ECHO_1          2
#define SRF08_ECHO_2          4
#define SRF08_ECHO_3          6
#define SRF08_ECHO_4          8
#define SRF08_ECHO_5          10
#define SRF08_ECHO_6          12
#define SRF08_ECHO_7          14
#define SRF08_ECHO_8          16
#define SRF08_ECHO_9          18
#define SRF08_ECHO_10         20
#define SRF08_ECHO_11         22
#define SRF08_ECHO_12         24
#define SRF08_ECHO_13         26
#define SRF08_ECHO_14         28
#define SRF08_ECHO_15         30
#define SRF08_ECHO_16         32
#define SRF08_ECHO_17         34

#define SRF08_CHANGE_ADDRESS_BYTE0	0xA0 
#define SRF08_CHANGE_ADDRESS_BYTE1	0XAA
#define SRF08_CHANGE_ADDRESS_BYTE2	0XA5

extern void          srf08_select_unit(unsigned char srf08_address);
extern void          srf08_init(char * dev);

extern void          srf08_set_gain(unsigned char gain);
extern void          srf08_set_range(unsigned int millimeters);

extern void			 srf08_ping_send(unsigned char address);
extern unsigned int  srf08_ping_recv(unsigned char address);
extern unsigned int  srf08_ping(unsigned char address);
extern unsigned int  srf08_read_register(unsigned char srf08_register);

extern void          srf08_change_i2c_address(unsigned char new_i2c_address);

#endif //SRF08_H