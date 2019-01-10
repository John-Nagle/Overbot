/* -*- indent-tabs-mode:T; c-basic-offset:8; tab-width:8; -*- vi: set ts=8:
 *
 * Khian Hao Lim
 * Oct 03
 * Overbot
 *
 * Object that encapsulates all the data from the METER server
 */
#ifndef METER_H
#define METER_H
class Kalman_Filter;

class METER {
public:
    METER();
   ~METER();
   
   Kalman_Filter * mKF;
private:
};

#endif
