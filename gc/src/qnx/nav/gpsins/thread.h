/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Used to share the var between threads
 */

#ifndef THREAD_H
#define THREAD_H

using namespace std;

//clear the include paths later
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <iostream>

//TODO
#include "../../common/include/gpsins_messaging.h"
#include "../../common/include/messaging.h"
#include "../../common/include/mutexlock.h"
#include "../../common/include/Kalman.h"
#include "../../common/include/Vector_Rotate.h"
#include "../../common/include/Frames.h"
#include "../../common/include/Quat.h"

#include "Kalman_Filter.h"

#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>

extern int verbose;

void * server_thread(void * args_v);

void * meter_thread(void * args_v);

inline
double getNumSecPassed(struct timeval * before, struct timeval * now)
{
      double secs = 0;
      secs += now->tv_sec - before->tv_sec;
      secs += (now->tv_usec - before->tv_usec) / 1000000.0;
      return secs;

}

#endif //THREAD_H
