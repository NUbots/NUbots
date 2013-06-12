/*! @file DarwinWebotsNetworkThread.h
    @brief Declaration of a special thread to use webot's simulated network.

    @class DarwinWebotsNetworkThread
    @brief A thread to send and receive packets using webot's simulated network.
 
    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jason Kulk, Jed Rietveld
 
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

#ifndef TEAM_THREAD_H
#define TEAM_THREAD_H

#include "Tools/Threading/PeriodicThread.h"
class TeamInformation;
class GameInformation;
class RoboCupGameControlData;
class RoboCupGameControlDataWebots;
#include "DarwinWebotsPlatform.h"

class DarwinWebotsNetworkThread : public PeriodicThread
{
public:
    DarwinWebotsNetworkThread(GameInformation* gameinfo, TeamInformation* teaminfo, DarwinWebotsPlatform* webots, int period = 500);
    ~DarwinWebotsNetworkThread();
private:
    void periodicFunction();
    void convertToGamePacket(const RoboCupGameControlDataWebots* data);
    
    RoboCupGameControlData* m_game_packet;
    
    webots::Receiver* m_receiver;
    webots::Emitter* m_emitter;
    
    GameInformation* m_game_info;
    TeamInformation* m_team_info;
};

#endif

