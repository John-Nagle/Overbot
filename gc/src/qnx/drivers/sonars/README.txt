README SONARS server
Khian Hao Lim 02/07/04

This server will control the firing of sonars and store up their readings. 
It will also be able to respond to queries by other clients about the readings

i2c.cc and i2c.h contains the i2c level code which in this case (since we are
using a I2C-to-RS232 converter) are low level serial writing and reading
logic

srf08.cc and srf08.h uses the i2c level logic in i2c.cc to communicate with
each srf08 unit and presents a level of abstraction for main.cc to use

main.cc uses high level functions like srf08_ping to obtain distance estimates
and srf08_change_i2c_address to change the addresses of a srf08 unit.
