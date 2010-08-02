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
#include "NAOWebotsIO.h"

#include "debug.h"
#include "debugverbositynuplatform.h"

#include <string.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
using namespace std;

/*! @brief Constructor for NAO in Webots robotic platform
 */
NAOWebotsPlatform::NAOWebotsPlatform(int argc, const char *argv[]) : m_argc(argc), m_argv(argv)
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 1
        debug << "NAOWebotsPlatform::NAOWebotsPlatform()" << endl;
    #endif
    init();
    
    m_camera = new NAOWebotsCamera(this);
    m_sensors = new NAOWebotsSensors(this);
    m_actionators = new NAOWebotsActionators(this);
}

NAOWebotsPlatform::~NAOWebotsPlatform()
{
}

/*! @brief Initialises the NUPlatform's name
    Sets m_name.
 */
void NAOWebotsPlatform::initName()
{
    // In webots the player number is a command line arguement. We use this as the robot number.
    // We construct the name as nubot${playernumber}
    int player_number = 0;
    if (m_argc >= 2)
        player_number = atoi(m_argv[1]) + 1;
    
    stringstream ss;
    ss << "nubot" << player_number;
    m_name = ss.str();
}

/*! @brief Intialise the NUPlatform's robot number 
    Sets m_robot_number
 */
void NAOWebotsPlatform::initNumber()
{
    // In webots the player number is a command line arguement. We use this as the robot number.
    if (m_argc >= 2)
        m_robot_number = atoi(m_argv[1]) + 1;
}

/*! @brief Intialise the NUPlatform's team number 
    Sets m_team_number
 */
void NAOWebotsPlatform::initTeam()
{
    // In webots the team number is a command line arguement.
    if (m_argc >= 3)
        m_team_number = atoi(m_argv[2]);
}

/*! @brief Intialise the NUPlatform's mac address 
    Sets m_mac_address
 */
void NAOWebotsPlatform::initMAC()
{
    // In webots we manufacture a mac addresss that is all zero except for the last digit, which is the robot number
    stringstream ss;
    ss << "00-00-00-00-0" << m_robot_number;
    m_mac_address = ss.str();
}


