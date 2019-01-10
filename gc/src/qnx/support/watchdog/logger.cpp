//
//	Watchdog  -- top-level watchdog program for QNX real-time applications
//
//	File logger.cpp -- reads standard output of all children, prepends time and program ID, sends to log.
//
//	J.	Nagle
//	August, 2002
//
#include <stdio.h>
#include "messaging.h"
#include "watchdog.h"
