//========================================================================
// Wingman3D.t.cpp  : Example program for Logitech Wingman Extreme 3D
//                    Digital HID Joystick Interface
//
// Author           : Vilas Kumar Chitrakaran <cvilas@ces.clemson.edu>
// Date             : March 2, 2004
// Compiler         : GNU GCC 2.95.3qnx-nto
// Operating System : QNX Momentics 6.2.1
//========================================================================

#include "joystick.hpp"
#include <stdlib.h>
#include <iostream.h>
#include <unistd.h>

int main()
{
    Joystick joystick;
    if( !joystick.isStatusOk() )
    {
        cout << joystick.getStatusMessage() << endl;
        return EXIT_FAILURE;
    }
    sleep(1);
    joystick.printDeviceInfo(3);
    sleep(1);

    while(1)
    {
        cout << "(X, Y, Twist)   : " << joystick.getX() << " " << joystick.getY()
        << " " << joystick.getTwist() << endl;

        cout << "Buttons pressed : ";
        for(int i = 1; i < 8; i++)
        {
            if( joystick.isButtonPressed(i) )
            {
                cout << i << " ";
            }
        }
        cout << endl;

        cout << "Slider          : " << joystick.getSliderValue() << endl;
        cout << "Hat Swtiches    : " << joystick.getHatSwitchStatus() << endl;
        cout << "Status          : " << joystick.getStatusMessage() << endl;

        if( !joystick.isStatusOk() )
            return EXIT_FAILURE;

        system("clear");
    }

    return EXIT_SUCCESS;
}
