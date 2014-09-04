/*! @file TeamPort.cpp
    @brief Implementation of TeamPort class.

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
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

#include "TeamPort.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "TeamTransmissionThread.h"

#include "debug.h"
#include "debugverbositynetwork.h"

#include <string>

/*! @brief Constructs a TeamPort
    @param nubotteaminformation the public nubot team information class
    @param portnumber the port number to broadcast on, and listen on
 */
TeamPort::TeamPort(TeamInformation* nubotteaminformation, int portnumber, bool ignoreself): UdpPort(std::string("TeamPort"), portnumber, ignoreself)
{
    m_team_information = nubotteaminformation;
    m_team_transmission_thread = new TeamTransmissionThread(this, 250);
    
    m_team_transmission_thread->start();
}

/*! @brief Closes the job port
 */
TeamPort::~TeamPort()
{
    m_team_transmission_thread->stop();
    delete m_team_transmission_thread;
}

/*! @brief Copies the received data into the public nubot joblist
    @param buffer containing the joblist
*/
void TeamPort::handleNewData(std::stringstream& buffer)
{
    #if DEBUG_NETWORK_VERBOSITY > 0
        debug << "TeamPort::handleNewData()." << std::endl;
    #endif
    std::string s_buffer = buffer.str();
    if (s_buffer.size() == sizeof(TeamPacket))
    {   // discard team packets that are the wrong size
        TeamPacket temp;
        buffer >> temp;
        m_team_information->addReceivedTeamPacket(temp);
    }
    else
        debug << "TeamPort::handleNewData(). The received packet does not have the correct length: " << s_buffer.size() << " instead of " << sizeof(TeamPacket) << std::endl;
}

