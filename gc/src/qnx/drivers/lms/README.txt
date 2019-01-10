Development notes, "lms".

J. Nagle
December 25, 2003

1.		"lms -m -d", followed by "i" command, produces a segmentation fault.
		Code assumed pointer was valid and nonzero. Fixed.
		Check that all pointers in objects are initialized in the constructor.

2.		The "special protocol for talking to serial ports" for the Sealevel unit
		is just Telnet protocol. The device talks Telnet, and understands
		the "raw mode" and "set baud rate" Telnet options. You can do that too,
		although you have to implement a handler for Telnet options and the
		Telnet option escape character.  We may or may not have to do that,
		but that's how it works.  If we can ever get this thing working reliably
		at 500000 baud, we can leave it there.
		
3.		The SICK LMS test program does those high baud rates in a wierd way.
		The "PCI/ISA" and "PCMCIA" options for 500000 baud don't really
		specify 500000 baud. They assume you have one of SICK's wierd
		serial boards, with a nonstandard clock NOT compensated for
		in its software. See the "LMS serial communications manual".
		They're setting other magic values their boards expect.
		The Sealevel boxes take real baud rates over the
		wire; the new one with the new crystal also has the firmware
		changed to do it right. That's why the SICK LMS program for
		Windows won't switch to 500000 baud properly.
		The "set baud rate" option over the network, incidentally,
		takes a real baud rate as a 32-bit number; it's not an index.
		
December 6, 2004.

	Current SICK configuration: Set to 500000 baud.

	Will work on Quatech port (COM5) using SICK LMS application
	set to COM5, 500000 baud, "ISA/PCI" mode.  But it may take
	several tries before it works, since the SICK unit's reception
	at 500000 baud is flakey.  

	(It can't really receive and scan at the same time, and it
	only has a one-byte serial input buffer and an 8-bit CPU,
	so when it's scanning at full speed and listening to the serial port,
	its serial input is flakey.)