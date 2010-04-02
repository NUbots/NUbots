/*! @file GameControllerPort.cpp
    @brief Implementation of GameControllerPort class.

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

#include "GameControllerPort.h"
#include "RoboCupGameControlData.h"
#include "GameController/GameInformation.h"

#include "debug.h"
#include "debugverbositynetwork.h"

#include <string>

/*! @brief Constructs a GameControllerPort
    @param nubotjobs the public nubot job list
 */
GameControllerPort::GameControllerPort(GameInformation* nubotgameinformation): UdpPort(std::string("GameControllerPort"), GAMECONTROLLER_PORT)
{
    m_game_information = nubotgameinformation;
}

/*! @brief Closes the job port
 */
GameControllerPort::~GameControllerPort()
{
}

/*! @brief Copies the received data into the public nubot joblist
    @param buffer containing the joblist
*/
void GameControllerPort::handleNewData(std::stringstream& buffer)
{
    RoboCupGameControlData* gcpacket = (RoboCupGameControlData*) buffer.str().c_str();
    //m_game_information->updateNetworkData(*gcpacket);
}
