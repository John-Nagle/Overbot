GPSINS README by Khian Hao Lim for Overbot Project 10/06/03
--------------------------------------------------
Overview:

The GPSINS code is based off the kalman filtering software developed
at autopilot.sourceforge.net which was targetted for state estimation
of an autonomous RC-modified helicopter.

Hardware and services:
See /doc/nav for data sheets.

Crossbow AHRS400CB
Crossbow FOG Gyro
Novatel ProPak-LB
Novatel 600LB antenna
Omnistar Subscription

--------------------------------------------------
Refer to /src/qnx/common/include/gpsins_messaging.h for the list of
cmds that the GPSINS server supports

--------------------------------------------------
Do run in watchdog, add line
ID=GPSINS path-to-gpsins-binary


