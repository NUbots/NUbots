/*! @file TeamTransmissionThread.h
    @brief Implementation of the team packet transmission thread.

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "TeamTransmissionThread.h"
#include "TeamPort.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include <string>


#include "debug.h"
#include "debugverbositynetwork.h"

/*! @brief Constructs the team packet transmission thread
 */

TeamTransmissionThread::TeamTransmissionThread(TeamPort* port, int period) : PeriodicThread(std::string("TeamTransmissionThread"), period, 0)
{
    #if DEBUG_VERBOSITY > 0
        debug << "TeamTransmissionThread::TeamTransmissionThread(" << period << ") with priority " << static_cast<int>(m_priority) << std::endl;
    #endif
    m_port = port;
}

TeamTransmissionThread::~TeamTransmissionThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "TeamTransmissionThread::~TeamTransmissionThread()" << std::endl;
    #endif
}

void TeamTransmissionThread::periodicFunction()
{
    if (m_port->m_team_information->getPlayerNumber() > 0)
    {
        std::stringstream buffer;
        buffer << m_port->m_team_information->generateTeamTransmissionPacket();
        m_port->sendData(buffer);
    }
}
