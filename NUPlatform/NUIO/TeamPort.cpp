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
#include "RoboCupGameControlData.h"
#include "GameController/GameInformation.h"

#include "debug.h"
#include "debugverbositynetwork.h"

#include <string>

/*! @brief Constructs a TeamPort
    @param nubotjobs the public nubot job list
 */
TeamPort::TeamPort(TeamInformation* nubotteaminformation, int portnumber): UdpPort(std::string("TeamPort"), portnumber)
{
    m_team_information = nubotteaminformation;
}

/*! @brief Closes the job port
 */
TeamPort::~TeamPort()
{
}

/*! @brief Sends this robot's team packet over the network
 */
TeamPort& operator<<(TeamPort& port, TeamInformation& teaminfo)
{
    // tp = m_game_information.getTeamPacket()
    // buffer << tp;
    // port.sendData(buffer)
    return port;
}

/*! @brief Sends this robot's team packet over the network
 */
TeamPort& operator<<(TeamPort& port, TeamInformation* teaminfo)
{
    // tp = m_game_information->getTeamPacket()
    // buffer << tp;
    // port.sendData(buffer)
    return port;
}

/*! @brief Copies the received data into the public nubot joblist
    @param buffer containing the joblist
*/
void TeamPort::handleNewData(std::stringstream& buffer)
{
    //m_game_information->newTeamPacket(buffer);
}
