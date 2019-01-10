////////////////////////////////////////////////////////////////////////////////
//
//    File:
//      moveserver.cc
//
//    Usage:
//
//	In the watchdog startup file (startfile.txt):
//		ID=DIR moveserver
//
//	For a much more detailed description of the Move server,
//	see note #41 on the website, or gc/doc/nav/moveserver.pdf
//
//    Description:
//
//	Takes high-level move commands from mapserver and implements them
//	by translating them into low-level commands to the vehicle actuator
//	servers (steering, throttle, brake, trasmission).
//       
//      Performs sanity checks on the move commands.  It contains a simple
//      dynamics model and will refuse to move in any manner which threatens
//      vehicle safety.  If necessary it will reduce the specified speed until
//      it is deemed safe.
//      
//      Also checks for potential collisions by querying the vorad server;
//	if a collision appears imminent, it will halt the vehicle in spite
//	of any move commands.
//
//	Keeps track of predicted position vs. actual position error so the
//	algorithm can adapt and adjust for it.
//      
//      
//      Simulation modes are provided for testing purposes
//		(see moveservermenu.h)
//
//	The Move Server consists of the following threads:
//          	- a main thread, which accepts messages
//     		- a menu thread
// 
//    Messages received:
//
//	MoveServerMSG::MsgMOVE
//	
//    Messages sent:
//
//
//    Written By:
//
//      Achut Reddy
//      Team Overbot
//      January 2003
//
////////////////////////////////////////////////////////////////////////////////
