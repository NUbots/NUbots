/*! @file DarwinWebotsNetworkThread.h
    @brief Implementation of a special thread to use webot's simulated network.

    @author Jason Kulk, Jed Rietveld
 
 Copyright (c) 2010 Jason Kulk, Jed Rietveld
 
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

#include "DarwinWebotsNetworkThread.h"

#include "Infrastructure/GameInformation/GameInformation.h"
#include "NUPlatform/NUIO/RoboCupGameControlData.h"
#include "RoboCupGameControlDataWebots.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include <string>
#include <sstream>
using namespace std;

#include "debug.h"

/*! @brief Constructs the team packet transmission thread
 */

DarwinWebotsNetworkThread::DarwinWebotsNetworkThread(GameInformation* gameinfo, TeamInformation* teaminfo, DarwinWebotsPlatform* webots, int period) : PeriodicThread(string("DarwinWebotsNetworkThread"), period, 0)
{
    m_game_info = gameinfo;
    m_game_packet = new RoboCupGameControlData();
    m_team_info = teaminfo;

    m_receiver = webots->getReceiver("receiver");
    m_receiver->enable(period);
    m_emitter = webots->getEmitter("emitter");
}

DarwinWebotsNetworkThread::~DarwinWebotsNetworkThread()
{
    delete m_game_packet;
}

void DarwinWebotsNetworkThread::periodicFunction()
{
    // Do receiving
    while (m_receiver->getQueueLength() > 0)
    {   // while there are packets to be processed...
        const char* data = reinterpret_cast<const char*>(m_receiver->getData());
        if (memcmp(data, GAMECONTROLLER_STRUCT_HEADER, sizeof(GAMECONTROLLER_STRUCT_HEADER)-1) == 0 and m_receiver->getDataSize() == sizeof(RoboCupGameControlDataWebots))
        {   // if it is a gamecontroller packet
            convertToGamePacket((RoboCupGameControlDataWebots*)data);
            (*m_game_info) << m_game_packet;
        }
        else if (memcmp(data, TEAM_PACKET_STRUCT_HEADER, sizeof(TEAM_PACKET_STRUCT_HEADER)-1) == 0 and m_receiver->getDataSize() == sizeof(TeamPacket))
        {   // if it is a team packet
            stringstream ss;
            ss.write((char*) data, sizeof(TeamPacket));
            TeamPacket temp;
            ss >> temp;
            m_team_info->addReceivedTeamPacket(temp);
        }
        else
            std::cout << "Received " << m_receiver->getDataSize() << " unknown bytes. Want " << sizeof(RoboCupGameControlDataWebots) << " or " << sizeof(TeamPacket) << std::endl;
        m_receiver->nextPacket();
    };
    
    // Do transmitting
    stringstream ss;
    ss << m_team_info->generateTeamTransmissionPacket();
    m_emitter->send(ss.str().c_str(), ss.str().size());
}

void DarwinWebotsNetworkThread::convertToGamePacket(const RoboCupGameControlDataWebots* data)
{
    memcpy(m_game_packet->header, data->header, sizeof(m_game_packet->header));
    m_game_packet->version = GAMECONTROLLER_STRUCT_VERSION;
    m_game_packet->playersPerTeam = data->playersPerTeam;
    m_game_packet->state = data->state;
    m_game_packet->firstHalf = data->firstHalf;
    m_game_packet->kickOffTeam = data->kickOffTeam;
    m_game_packet->secondaryState = data->secondaryState;
    m_game_packet->dropInTeam = data->dropInTeam;
    m_game_packet->dropInTime = data->dropInTime;
    m_game_packet->secsRemaining = data->secsRemaining;
    
    m_game_packet->teams[0].teamNumber = data->teams[0].teamNumber;
    m_game_packet->teams[0].teamColour = data->teams[0].teamColour;
    m_game_packet->teams[0].goalColour = 0;
    m_game_packet->teams[0].score = data->teams[0].score;    
    memcpy(m_game_packet->teams[0].players, data->teams[0].players, sizeof(data->teams[0].players));
    
    m_game_packet->teams[1].teamNumber = data->teams[1].teamNumber;
    m_game_packet->teams[1].teamColour = data->teams[1].teamColour;
    m_game_packet->teams[1].goalColour = 0;
    m_game_packet->teams[1].score = data->teams[1].score; 
    memcpy(m_game_packet->teams[1].players, data->teams[1].players, sizeof(data->teams[1].players));
}
