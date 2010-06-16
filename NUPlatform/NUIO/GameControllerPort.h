/*! @file GameControllerPort.h
    @brief Declaration of GameControllerPort class.

    @class GameControllerPort
    @brief GameControllerPort a network port for listening

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
#ifndef GAMECONTROLLERPORT_H
#define GAMECONTROLLERPORT_H

#include "UdpPort.h"
class GameInformation;
struct RoboCupGameControlReturnData;

class GameControllerPort : public UdpPort
{
public:
    GameControllerPort(GameInformation* nubotgameinformation);
    ~GameControllerPort();
    
    void sendReturnPacket(RoboCupGameControlReturnData* data);
private:
    void handleNewData(std::stringstream& buffer);
public:
private:
    GameInformation* m_game_information;
};

#endif
