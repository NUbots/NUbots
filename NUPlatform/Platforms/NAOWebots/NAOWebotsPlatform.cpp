/*! @file NAOWebotsPlatform.cpp
    @brief Implementation of NAOWebotsPlatform : NUbot (Robot) class

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

#include "NAOWebotsPlatform.h"
#include "NAOWebotsCamera.h"
#include "NAOWebotsSensors.h"
#include "NAOWebotsActionators.h"
#include "NAOWebotsSystem.h"
#include "NAOWebotsIO.h"
#include "debug.h"
#include "debugverbositynuplatform.h"

#include <string.h>
#include <iostream>
#include <stdlib.h>
using namespace std;

/*! @brief Constructor for NAO in Webots robotic platform
 
    Webots passes the controller a command line argument specifying the URBI port.
    This port number is then used to determine which robot the controller is running on.
 
    @param argc the number of command line arguements
    @param argv[] the command line arguements
 */
NAOWebotsPlatform::NAOWebotsPlatform(int argc, const char *argv[])
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOWebotsPlatform::NAOWebotsPlatform" << endl;
#endif
    if (argc < 3)
        errorlog << "NAOWebotsPlatform::NAOWebotsPlatform(). Could not find team id and player id in controllerArgs" << endl;
    
    m_player_number = atoi(argv[1]) + 1;
    m_team_number = atoi(argv[2]);
    
    setNameFromNumber();
    
    system = new NAOWebotsSystem(this);                 // the system needs to be created first because it provides times for the other modules!
    camera = new NAOWebotsCamera(this);
    sensors = new NAOWebotsSensors(this);
    actionators = new NAOWebotsActionators(this);
}

NAOWebotsPlatform::~NAOWebotsPlatform()
{
}


/*! @brief A function which generates a Robot's name based on a number
    @param number the robot's number
    @param name the string to be updated with the robot's name
 */
void NAOWebotsPlatform::setNameFromNumber()
{
    switch (m_player_number) {
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

