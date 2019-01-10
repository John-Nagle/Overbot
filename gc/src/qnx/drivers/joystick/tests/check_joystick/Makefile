#========================================================================
# Logitech Wingman Extreme 3D Digital HID Joystick Interface    
#
# Author           : Vilas Kumar Chitrakaran <cvilas@ces.clemson.edu>     
# Date             : March 2, 2004
# Compiler         : GNU GCC 2.95.3qnx-nto
# Operating System : QNX Momentics 6.2.1                                
# ----------------------------------------------------------------------  
# File: makefile
#========================================================================  

LDFLAGS = g++ -Wall -O3 -o
INCLUDEHEADERS = -I ../ -I ../../
INCLUDELIB = -L ../ -L ../../ -ljoystick -lhiddi
TARGETS = check_joystick

targets:	$(TARGETS)
CLEAN = rm -rf $(TARGETS) *.dat *.o


# ----- Wingman3D -----
check_joystick : check_joystick.cpp
	$(LDFLAGS) $@ $? $(INCLUDEHEADERS) $(INCLUDELIB)

clean:
	$(CLEAN)

