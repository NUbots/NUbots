/*! @file NAOPlatform.cpp
    @brief Implementation of NAOPlatform

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

#include "NAOPlatform.h"
#include "NAOCamera.h"
#include "NAOSensors.h"
#include "NAOActionators.h"
#include "NAOSystem.h"
#include "NAOIO.h"
#include "Motion/Tools/MotionFileTools.h"

#include <unistd.h>

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotdataconfig.h"

/*! @brief Constructor for NAO robotic platform
 */
NAOPlatform::NAOPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::NAOPlatform()" << endl;
#endif
    char hostname[255];                                         // we store the name in /etc/hostname so the network name and robot name are the same
    gethostname(hostname, 255);
    m_name = string(hostname);  
    m_player_number = atoi(m_name.substr(5).c_str());           // the name will be nubotXX where XX is the player number
    
    ifstream teamfile((string(CONFIG_DIR) + string("Team.cfg")).c_str());      // the team number is stored in a file
    if (teamfile.is_open())
        m_team_number = MotionFileTools::toFloat(teamfile);
    else
    {
        errorlog << "NAOPlatform::NAOPlatform(). Unable to load Team.cfg" << endl;
        m_team_number = 0;
    }
    system = new NAOSystem();                 // the system needs to be created first because it provides times for the other modules! 
    camera = new NAOCamera();
    sensors = new NAOSensors();
    actionators = new NAOActionators();
	
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::NAOPlatform(). Completed." << endl;
#endif
}

NAOPlatform::~NAOPlatform()
{
#if DEBUG_NUPLATFORM_VERBOSITY > 4
    debug << "NAOPlatform::~NAOPlatform()" << endl;
#endif
}

