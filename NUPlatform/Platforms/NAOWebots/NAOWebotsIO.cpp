/*! @file NAOIO.cpp
    @brief Implementation of NAOIO input/output class

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

#include "NAOWebotsIO.h"
#include "NAOWebotsPlatform.h"
#include "NAOWebotsNetworkThread.h"

#include "NUPlatform/NUIO/NetworkPortNumbers.h"
#include "NUPlatform/NUIO/JobPort.h"
#include "NUPlatform/NUIO/TcpPort.h"

#include "debug.h"
#include "debugverbositynetwork.h"
#include "ioconfig.h"

using namespace std;

#include "NUbot.h"

/*! @brief Construct a NAOWebotsIO object
    @param nubot a pointer to the NUbot, we need this to gain access to the public store
 */
NAOWebotsIO::NAOWebotsIO(NUbot* nubot, NAOWebotsPlatform* platform)
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NAOWebotsIO::NAOWebotsIO()" << endl;
#endif
    m_nubot = nubot;
    #if defined(USE_NETWORK_GAMECONTROLLER) or defined(USE_NETWORK_TEAMINFO)
        m_network = new NAOWebotsNetworkThread(nubot->GameInfo, nubot->TeamInfo, platform, 250);
        m_network->start();
    #endif
    
    #ifdef USE_NETWORK_JOBS
        m_jobs_port = new JobPort(nubot->Jobs);
    #endif
    #ifdef USE_NETWORK_DEBUGSTREAM
        m_vision_port = new TcpPort(VISION_PORT);
        m_localisation_port = new TcpPort(LOCWM_PORT);
    #endif
}

NAOWebotsIO::~NAOWebotsIO()
{
#if DEBUG_NETWORK_VERBOSITY > 4
    debug << "NAOWebotsIO::~NAOWebotsIO()" << endl;
#endif
    delete m_network;
}

