Interface board PC layout files.

This was an outsourced job, done by Tom Smith
(tsmith@velocity11.com)

Thomas Smith
1138 Myrna Way
Rodeo, CA 94572

(510) 366-6972 (C)
(650) 846-6651 (W)

---
2004-10-9
Changes needed for board revision:

1. Bypass caps were limiting frequency response, and board
works without them. 

Delete C1, C5
Delete C11, C12, C13, C14, C15
Delete C16, C17, C18, C19, C20

2. Component change on U7, U8: use PS2506-4.  Same pinout.

3. Polysilicon fuse lead spacing is 0.10in.

4, Delete R13, R14 zero ohm resistors. 
   Replace R14 with a trace, delete R13

5. Delete R15, R16 zero ohm resistors.
   Replace R16 with a trace, delete R15.

6. Connectors J2, J3, J11, J12, J13, J14:
	pin 9 to +12V

7. connectors J13, J14: 
	disconnect pin 6 from +12V
	disconnect pin 2 from +5A, +5B
	J13: pin 5 to +5A
	J14: pin 5 to +5B.

This consistently puts power on pins 5 and 9, and
makes it safe to plug any connector into any socket
without shorting anything.

8. Output of TACH-1 is ANA 1-1, not ANA 2.1
   Output of TACH-2 is ANA 2-1, not ANA 2-2.

10. Tachometer component value changes (do not affect blank board)

	R3	270K
	C2	1mfd
	C3	0.047mfd

	R7	100K
	C4	0.22mfd
	C6	0.0033mfd

11.  Attempt to reduce component height. Bend TO-220 regulators
     flat to board if possiible and allow space for standard
     TO-220 heat sink.  Replace electrolytic caps with lower profile
     units if possible.  


=====

2004-11-12

Errors in new board, and configuration thereof:

1.   For a board WITH tachometers, a jumper wire is required between

	 ANA_1-1 at pin 5 of J5
	 ANA_1-1 at pin 5 of U1

2.   For a board WITHOUT tachometers, omit U1, U2, R1, and R5, to isolate the
     analog inputs from the unused tachometer circuitry.
