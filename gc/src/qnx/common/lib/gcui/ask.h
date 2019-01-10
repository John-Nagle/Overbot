/////////////////////////////////////////////////////////////////////////////
//
//    File: ask.h
//
//    Usage:
//        #include "ask.h"
//
//    Description:
//        Header file for ask.cc.
//
//    Written By:
//        Celia Oakley
//        Team Overbot
//        July, 2003
//
/////////////////////////////////////////////////////////////////////////////

#ifndef ASK_H
#define ASK_H

#include <string>
#include <iostream>
#include <cstdio>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

class Ask {
public:
	// all static member functions
    static int Ask::Int(const char *prompt, const int def);
    static int Ask::Int(const char *prompt, const int def,
                        const int min, const int max);

    static int Ask::Hex(const char *prompt, const int def);
    static int Ask::Hex(const char *prompt, const int def,
                        const int min, const int max);

    static float Ask::Float(const char *prompt, const float def);
    static float Ask::Float(const char *prompt, const float def,
                            const float min, const float max);

    static double Ask::Double(const char *prompt, const double def);
    static double Ask::Double(const char *prompt, const double def,
                            const double min, const double max);

	static char Ask::Char(const char *prompt, const char def);
	static char Ask::Char(const char *prompt, const char def,
	                      const char *charString);

    static void Ask::String(const char *prompt, const char *def,
                            char *result, size_t resultLen);

    static bool Ask::YesNo(const char *prompt, const bool def);

    static void Ask::Pause(const char *prompt);
    static void Ask::Pause();

	static bool Ask::CharReady();
	static void Ask::CharFlush();
};

#endif // ASK_H