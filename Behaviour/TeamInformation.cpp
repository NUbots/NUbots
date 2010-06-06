/*! @file TeamInformation.cpp
    @brief Implementation of TeamInformation class

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

#include "TeamInformation.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUSystem.h"

#include <memory.h>

#include "debug.h"

TeamInformation::TeamInformation(int playernum, int teamnum, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects) : m_TIMEOUT(2000)
{
    m_player_number = playernum;
    m_team_number = teamnum;
    m_data = data;
    m_actions = actions;
    m_objects = fieldobjects;
    
    initTeamPacket();
    m_received_packets = vector<boost::circular_buffer<TeamPacket> >(13, boost::circular_buffer<TeamPacket>(3));
}


TeamInformation::~TeamInformation()
{
}

bool TeamInformation::amIClosestToBall()
{
    for (size_t i=0; i<m_received_packets.size(); i++)
    {
        if (not m_received_packets[i].empty())
        {
            if ((m_data->CurrentTime - m_received_packets[i].back().ReceivedTime < m_TIMEOUT) and (m_packet.TimeToBall > m_received_packets[i].back().TimeToBall))
                return false;
        }
    }
    return true;
}

/*! @brief Returns all of the shared balls in the TeamInformation
 */
vector<TeamPacket::SharedBall> TeamInformation::getSharedBalls()
{
    vector<TeamPacket::SharedBall> sharedballs;
    sharedballs.reserve(m_received_packets.size());
    for (size_t i=0; i<m_received_packets.size(); i++)
    {
        if (not m_received_packets[i].empty() and (m_data->CurrentTime - m_received_packets[i].back().ReceivedTime < m_TIMEOUT))
        {   // if there is a received packet that is not too old grab the shared ball
            sharedballs.push_back(m_received_packets[i].back().Ball);
        }
    }
    return sharedballs;
}

/*! @brief Initialises my team packet to send to my team mates
 */
void TeamInformation::initTeamPacket()
{   
    memcpy(m_packet.Header, TEAM_PACKET_STRUCT_HEADER, sizeof(m_packet.Header));
    m_packet.ID = 0;
    // we initialise everything that never changes here
    m_packet.PlayerNumber = static_cast<char>(m_player_number);
    m_packet.TeamNumber = static_cast<char>(m_team_number);
}

/*! @brief Updates my team packet with the latest information
 */
void TeamInformation::updateTeamPacket()
{
    if (m_data == NULL or m_objects == NULL)
        return;
    m_packet.ID = m_packet.ID + 1;
    m_packet.SentTime = m_data->CurrentTime;
    m_packet.TimeToBall = getTimeToBall();
    
    // ------------------------------ Update shared localisation information
    // update shared ball
    MobileObject& ball = m_objects->mobileFieldObjects[FieldObjects::FO_BALL];
    m_packet.Ball.TimeSinceLastSeen = ball.TimeSinceLastSeen();
    m_packet.Ball.X = ball.X();
    m_packet.Ball.Y = ball.Y();
    m_packet.Ball.SRXX = ball.srXX();
    m_packet.Ball.SRXY = ball.srXY();
    m_packet.Ball.SRYY = ball.srYY();
    
    // update self
    Self& self = m_objects->self;
    m_packet.Self.X = self.wmX();
    m_packet.Self.Y = self.wmY();
    m_packet.Self.Heading = self.Heading();
    m_packet.Self.SDX = self.sdX();
    m_packet.Self.SDY = self.sdY();
    m_packet.Self.SDHeading = self.sdHeading();
}

float TeamInformation::getTimeToBall()
{
    float time = 600;
    if (m_objects->mobileFieldObjects[FieldObjects::FO_BALL].TimeSeen() > 0)
    {
        float headyaw, headpitch;
        m_data->getJointPosition(NUSensorsData::HeadPitch,headpitch);
        m_data->getJointPosition(NUSensorsData::HeadYaw, headyaw);
        float measureddistance = m_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredDistance();
		float balldistance = measureddistance * cos(m_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredElevation());
        float ballbearing = m_objects->mobileFieldObjects[FieldObjects::FO_BALL].measuredBearing();
        time = balldistance/10.0 + fabs(ballbearing)/0.5;
    }
    return time;
}

void TeamPacket::summaryTo(ostream& output)
{
    output << "ID: " << ID;
    output << " Player: " << (int)PlayerNumber;
    output << " TimeToBall: " << TimeToBall;
    output << endl;
}


ostream& operator<< (ostream& output, const TeamPacket& packet)
{
    output.write((char*) &packet, sizeof(packet));
    return output;
}

istream& operator>> (istream& input, TeamPacket& packet)
{
    TeamPacket packetBuffer;
    input.read(reinterpret_cast<char*>(&packetBuffer), sizeof(packetBuffer));
    packet = packetBuffer;
    
    return input;
}

ostream& operator<< (ostream& output, TeamInformation& info)
{
    info.updateTeamPacket();
    output << info.m_packet;
    nusystem->displayTeamPacketSent(info.m_actions);
    return output;
}

ostream& operator<< (ostream& output, TeamInformation* info)
{
    if (info != NULL)
        output << (*info);
    return output;
}

istream& operator>> (istream& input, TeamInformation& info)
{
    TeamPacket temp;
    input >> temp;
    double timenow;
    if (info.m_data != NULL)
        timenow = info.m_data->CurrentTime;
    else
        timenow = nusystem->getTime();
    temp.ReceivedTime = timenow;
    
    if (temp.PlayerNumber > 0 and (unsigned) temp.PlayerNumber < info.m_received_packets.size() and temp.PlayerNumber != info.m_player_number and temp.TeamNumber == info.m_team_number)
    {   // only accept packets from valid player numbers
        if (info.m_received_packets[temp.PlayerNumber].empty())
        {   // if there have been no previous packets from this player always accept the packet
            info.m_received_packets[temp.PlayerNumber].push_back(temp);
            nusystem->displayTeamPacketReceived(info.m_actions);
        }
        else
        {
            TeamPacket lastpacket = info.m_received_packets[temp.PlayerNumber].back();
            if (timenow - lastpacket.ReceivedTime > 2000)
            {   // if there have been no packets recently from this player always accept the packet
                info.m_received_packets[temp.PlayerNumber].push_back(temp);
                nusystem->displayTeamPacketReceived(info.m_actions);
            }
            else if (temp.ID > lastpacket.ID)
            {   // avoid out of order packets by only adding recent packets that have a higher ID
                info.m_received_packets[temp.PlayerNumber].push_back(temp);
                nusystem->displayTeamPacketReceived(info.m_actions);
            }
        }
    }
    return input;
}

istream& operator>> (istream& input, TeamInformation* info)
{
    if (info != NULL)
        input >> (*info);
    return input;
}
