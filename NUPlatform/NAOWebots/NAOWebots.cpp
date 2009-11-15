/*! @file NAOWebots.cpp
    @brief Implementation of NAOWebots : NUbot (Robot) class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NAOWebots.h"
#include <iostream>
using namespace std;

/*! @brief Constructor for NAO in Webots robotic platform
 
    Webots passes the controller a command line argument specifying the URBI port.
    This port number is then used to determine which robot the controller is running on.
 
    @param argc the number of command line arguements
    @param argv[] the command line arguements
 */
NAOWebots::NAOWebots(int argc, const char *argv[])
{
    // find URBI port number (e.g. -p 54001) in controllerArgs
    int port = -1;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
            port = atoi(argv[i + 1]);
            break;
        }
    }
    
    if (port == -1) {
        cout << "Error: could not find port number in controllerArgs" << endl;
    }
    
    int m_number = (port % 10) + 1;
    cout << "NAOWebots::NAOWebots" << endl;
    cout << "NAOWebots::NAOWebots. this: " << this << endl;
    setNameFromNumber();
    //sensors = new Sensors();
    //actuators = new Actuators();
}

NAOWebots::~NAOWebots()
{
}


/* A function which generates a Robot's name based on a number
 @param number the robot's number
 @param name the string to be updated with the robot's name
 */
void NAOWebots::setNameFromNumber()
{
    switch (m_number) {
        case 1:
            m_name = string("Susannah");
            break;
        case 2:
            m_name = string("Daisy");
            break;
        case 3:
            m_name = string("Lisbeth");
            break;
        case 4:
            m_name = string("Poppy");
            break;
        default:
            m_name = string("Valerie");
            break;
    }
}

