Telnet client and library, for talking to serial-to-async converter devices.
John Nagle
December, 2003

Notes:

	1.	This is a rather limited implementation of the Telnet com port protocol, per RFC 2277.
		It does appear to do all the essential functions.
		
	2.	The Sealevel unit doesn't quite seem to do Telnet option negotiation right, but
		we live with that. It sends suboptions for the com port option before negotiating
		that we understand that option. But we do, so that's OK.
		
	3.	Setting the baud rate sends the number you set to the serial port converter.
		You may get back a slightly different number if you ask for the baud rate.
		The Sealevel unit with the 16Mhz crystal does 500000 baud exactly, and
		9600 baud approximately. The unmodified Sealevel unit is the other way
		round.
		
	4.	You can turn DTR on and off, but can't read its state. 
	
	5.	The Sealevel unit sometimes generates about 1K of zeros when started up.
	
26 JAN 2004

We're not actually using this.  We set the SICK unit to 500000 baud and left it
there, and ran the Sealevel unit in simple socket mode.