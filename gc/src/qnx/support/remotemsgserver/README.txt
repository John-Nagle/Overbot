Remote message server

John Nagle
Team Overbot
November, 2004.

This is a support utility used to allow remote access to running servers over the net.
It is run only in testing.

It is invoked in start files, with, for example

	ID=TESTSPEED remotemsgserver SPEED 
	
This allows a remote process not started by the watchdog to use QNX messaging
to talk to the server SPEED, by opening a connection with our 
"name_open_remote" function to "TESTSPEED".  

Multiple copies may be run under different names if needed.

NOTE: Running this creates a security hole allowing remote message
access to live real-time processes. It is only for test use.