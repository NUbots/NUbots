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
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUPlatform.h"
#include <sstream>
#include "Tools/FileFormats/FileFormatException.h"

#include <memory.h>

#include "debug.h"
#include "debugverbositynetwork.h"

TeamInformation::TeamInformation(int playernum, int teamnum) : TimestampedData(), m_TIMEOUT(2000)
{
    m_player_number = playernum;
    m_team_number = teamnum;
    m_data = Blackboard->Sensors;
    m_actions = Blackboard->Actions;
    m_objects = Blackboard->Objects;
    
    initTeamPacket();
    m_received_packets = PacketBufferArray(13, boost::circular_buffer<TeamPacket>(3));
    
    m_led_red = vector<float>(3,0);
    m_led_red[0] = 1;
    m_led_green = vector<float>(3,0);
    m_led_green[1] = 1;
}


TeamInformation::~TeamInformation()
{
}

bool TeamInformation::amIClosestToBall()
{

    for (size_t i=0; i<m_received_packets.size(); i++)
    {
        Self& self = m_objects->self;
        MobileObject& ball = m_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (not m_received_packets[i].empty())
        { //check if I'm clsoest to the ball *I can see*
            if ((m_data->CurrentTime - m_received_packets[i].back().ReceivedTime < m_TIMEOUT) and (m_packet.TimeToBall > m_received_packets[i].back().TimeToBall))
                return false;
        }
    }
    return true;
}

/*! @brief Returns all of the shared balls in the TeamInformation
 */
vector<TeamPacket::SharedBall> TeamInformation::getSharedBalls() const
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
    m_timestamp = m_data->CurrentTime;
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
    Self& self = m_objects->self;
    MobileObject& ball = m_objects->mobileFieldObjects[FieldObjects::FO_BALL];
    
    //avoidance for our own penalty box
    bool inPenaltyBox = false;
    float distance_from_centre_to_goal = fabs(m_objects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_LEFT].X());
    float ballDirection = -1.f;
    if (Blackboard->GameInfo->getTeamColour() == GameInformation::BlueTeam) {
        ballDirection = 1.f;
    }
    if (fabs(ball.Y()) < fabs(m_objects->stationaryFieldObjects[FieldObjects::FO_CORNER_YELLOW_PEN_LEFT].Y()) and ball.X()*ballDirection > distance_from_centre_to_goal) {
        inPenaltyBox = true;
        if (m_player_number == 1) {
            return time/10.f;
        } else {
            return time*2.f;
        }
    }
    
    float balldistance = ball.estimatedDistance();
    float ballbearing = ball.estimatedBearing();
    if (m_data->isIncapacitated() || (Blackboard->GameInfo->getCurrentState() != GameInformation::PlayingState))         // if we are incapacitated then we can't chase a ball
        return time;
    else if (m_player_number == 1 and balldistance > 150)            // goal keeper is a special case, don't chase balls too far away
        return time;
    else if (m_objects->mobileFieldObjects[FieldObjects::FO_BALL].lost() == false)
    {   // if neither the ball or self are lost or if we can see the ball then we can chase.
        vector<float> walkspeed, maxspeed;
        bool walk_speed_good = m_data->get(NUSensorsData::MotionWalkSpeed, walkspeed);
        bool max_speed_good = m_data->get(NUSensorsData::MotionWalkMaxSpeed, maxspeed);

        // If walk speed is not available, then we may not be able to get to the ball.
        if(not (walk_speed_good and max_speed_good))
        {
            debug << "Warning: Walk values not available - walk_speed_good: " << walk_speed_good << " max_speed_good: " <<  max_speed_good << std::endl;
            return time;
        }

        // Add time for the movement to the ball
        time = balldistance/maxspeed[0] + fabs(ballbearing)/maxspeed[2];
        
        if (balldistance > 30)
        {   // Add time for the 'acceleration' from the current speed to the speed required to the ball
            time += 1.5*fabs(cos(ballbearing) - walkspeed[0]/maxspeed[0]) + 1.5*fabs(sin(ballbearing) - walkspeed[1]/maxspeed[1]) + 1.5*fabs(ballbearing - walkspeed[2]/maxspeed[2]);
        }
        if (self.lost())
            time += 3;
    }
    return time;
}

ostream& TeamPacket::toFile(ostream& output) const
{
    output.write(reinterpret_cast<const char*>(Header), 4);
    int temp_id = ID;
    output.write(reinterpret_cast<const char*>(&temp_id), sizeof(temp_id));
    output.write(reinterpret_cast<const char*>(&SentTime), sizeof(SentTime));
    output.write(reinterpret_cast<const char*>(&ReceivedTime), sizeof(ReceivedTime));
    output.write(reinterpret_cast<const char*>(&PlayerNumber), sizeof(PlayerNumber));
    output.write(reinterpret_cast<const char*>(&TeamNumber), sizeof(TeamNumber));
    output.write(reinterpret_cast<const char*>(&TimeToBall), sizeof(TimeToBall));
    output.write(reinterpret_cast<const char*>(&Ball), sizeof(Ball));
    output.write(reinterpret_cast<const char*>(&Self), sizeof(Self));
    return output;
}

//#include <QDebug>
istream& TeamPacket::fromFile(istream& input)
{
//    //qDebug() << "Size is " << sizeof(ID);
    input.read(reinterpret_cast<char*>(Header), 4);
    int temp_id = 0;
    input.read(reinterpret_cast<char*>(&temp_id), sizeof(temp_id));
    ID = temp_id;
    input.read(reinterpret_cast<char*>(&SentTime), sizeof(SentTime));
    input.read(reinterpret_cast<char*>(&ReceivedTime), sizeof(ReceivedTime));
    input.read(reinterpret_cast<char*>(&PlayerNumber), sizeof(PlayerNumber));
    input.read(reinterpret_cast<char*>(&TeamNumber), sizeof(TeamNumber));
    input.read(reinterpret_cast<char*>(&TimeToBall), sizeof(TimeToBall));
    input.read(reinterpret_cast<char*>(&Ball), sizeof(Ball));
    input.read(reinterpret_cast<char*>(&Self), sizeof(Self));
    return input;
}

std::string TeamPacket::toString() const
{
    std::stringstream result;
    summaryTo(result);
    return result.str();
}

void TeamPacket::summaryTo(ostream& output) const
{
    output << "ID: " << ID;
    output << " Player: " << (int)PlayerNumber;
    output << " Team: " << (int)TeamNumber;
    output << " TimeToBall: " << TimeToBall;
    output << endl;
}

TeamPacket TeamInformation::generateTeamTransmissionPacket()
{
    updateTeamPacket();
    return m_packet;
}

void TeamInformation::addReceivedTeamPacket(TeamPacket& receivedPacket)
{
    double timenow;
    if (m_data != NULL)
        timenow = m_data->CurrentTime;
    else
        timenow = Platform->getTime();
    receivedPacket.ReceivedTime = timenow;

    if (receivedPacket.PlayerNumber > 0 and (unsigned) receivedPacket.PlayerNumber < m_received_packets.size() and receivedPacket.PlayerNumber != m_player_number and receivedPacket.TeamNumber == m_team_number)
    {   // only accept packets from valid player numbers
        // System->displayTeamPacketReceived(info.m_actions);
        debug << "team packet received from " << (int)receivedPacket.PlayerNumber << "." << std::endl;
        if (m_received_packets[receivedPacket.PlayerNumber].empty())
        {   // if there have been no previous packets from this player always accept the packet
            m_received_packets[receivedPacket.PlayerNumber].push_back(receivedPacket);
        }
        else
        {
            TeamPacket lastpacket = m_received_packets[receivedPacket.PlayerNumber].back();
            if (timenow - lastpacket.ReceivedTime > 2000)
            {   // if there have been no packets recently from this player always accept the packet
                m_received_packets[receivedPacket.PlayerNumber].push_back(receivedPacket);
            }
            else if (receivedPacket.ID > lastpacket.ID)
            {   // avoid out of order packets by only adding recent packets that have a higher ID
                m_received_packets[receivedPacket.PlayerNumber].push_back(receivedPacket);
            }
        }
    }
    else
    {
        #if DEBUG_NETWORK_VERBOSITY > 0
            debug << ">>TeamInformation. Rejected team packet:";
            receivedPacket.summaryTo(debug);
            debug << endl;
        #endif
    }
    return;
}

std::string TeamInformation::toString() const
{
    std::stringstream result;
    result << "Team Information - Timestamp: " << GetTimestamp() << std::endl;
    result << "Player Number: " << m_player_number << std::endl;
    result << "Team Number: " << m_team_number << std::endl;
    result << "Current Packet (self): " << std::endl;
    result << m_packet.toString() << std::endl;

    result << "Received packets:" << std::endl;
    int total_packets_displayed = 0;
    PacketBufferArray::const_iterator buffer_iterator = m_received_packets.begin();
    int bufferNumber = 0;
    while(buffer_iterator != m_received_packets.end())
    {
        if(buffer_iterator->size() > 0)
        {
            PacketBuffer::const_iterator packet_iterator = buffer_iterator->begin();
            int packetNumber = 0;
            while(packet_iterator != buffer_iterator->end())
            {
                result << packet_iterator->toString();
                ++total_packets_displayed;
                ++packet_iterator;
                ++packetNumber;
            }
        }
        ++buffer_iterator;
        ++bufferNumber;
    }
    if(total_packets_displayed <= 0) result << "None." << std::endl;
    return result.str();
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

ostream& operator<< (ostream& output, const TeamInformation& info)
{
    output.write(reinterpret_cast<const char*>(&info.m_timestamp), sizeof(info.m_timestamp));
    output.write(reinterpret_cast<const char*>(&info.m_player_number), sizeof(info.m_player_number));
    output.write(reinterpret_cast<const char*>(&info.m_team_number), sizeof(info.m_team_number));
    info.m_packet.toFile(output);

    int num_buffers = info.m_received_packets.size();
    output.write(reinterpret_cast<const char*>(&num_buffers), sizeof(num_buffers));

    int numEntries;
    TeamInformation::PacketBufferArray::const_iterator buffer_iterator = info.m_received_packets.begin();
    while(buffer_iterator != info.m_received_packets.end())
    {
        numEntries = buffer_iterator->size();
        output.write(reinterpret_cast<const char*>(&numEntries), sizeof(numEntries));

        TeamInformation::PacketBuffer::const_iterator packet_iterator = buffer_iterator->begin();
        while(packet_iterator != buffer_iterator->end())
        {
            packet_iterator->toFile(output);
            ++packet_iterator;
        }
        ++buffer_iterator;
    }
    return output;
}

istream& operator>> (istream& input, TeamInformation& info)
{
    input.read(reinterpret_cast<char*>(&info.m_timestamp), sizeof(info.m_timestamp));
    input.read(reinterpret_cast<char*>(&info.m_player_number), sizeof(info.m_player_number));
    input.read(reinterpret_cast<char*>(&info.m_team_number), sizeof(info.m_team_number));
    info.m_packet.fromFile(input);

    int num_buffers = 0;
    int num_entries = 0;
    input.read(reinterpret_cast<char*>(&num_buffers), sizeof(num_buffers));

    if(input.bad() || input.eof())
    {
        std::stringstream error_msg;
        error_msg << "Error reading number of buffers " << num_buffers << " - end of file reached." << std::endl;
        throw FileFormatException(error_msg.str());
    }
    info.m_received_packets.resize(num_buffers);
    for (int b=0; b < num_buffers; b++)
    {
        input.read(reinterpret_cast<char*>(&num_entries), sizeof(num_entries));
        if(input.bad() || input.eof())
        {
            std::stringstream error_msg;
            error_msg << "Error reading packet buffer " << b << " of " << num_buffers << " - end of file reached." << std::endl;
            throw FileFormatException(error_msg.str());
        }
        info.m_received_packets[b].resize(num_entries);

        for(int e=0; e < num_entries; e++)
        {
            if(input.bad() || input.eof())
            {
                std::stringstream error_msg;
                error_msg << "Error reading received packet " << e << " of " << num_entries << " for buffer " << b << " of " << num_buffers << " - end of file reached." << std::endl;
                throw FileFormatException(error_msg.str());
            }
            info.m_received_packets[b][e].fromFile(input);
        }
    }
    return input;
}

/*
ostream& operator<< (ostream& output, TeamInformation& info)
{
    info.updateTeamPacket();
    output << info.m_packet;
    Platform->toggle(NUPlatform::Led3, Blackboard->Actions->CurrentTime, info.m_led_green);
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
        timenow = Platform->getTime();
    temp.ReceivedTime = timenow;
    
    if (temp.PlayerNumber > 0 and (unsigned) temp.PlayerNumber < info.m_received_packets.size() and temp.PlayerNumber != info.m_player_number and temp.TeamNumber == info.m_team_number)
    {   // only accept packets from valid player numbers
        Platform->toggle(NUPlatform::Led1, Blackboard->Actions->CurrentTime, info.m_led_green);;
        if (info.m_received_packets[temp.PlayerNumber].empty())
        {   // if there have been no previous packets from this player always accept the packet
            info.m_received_packets[temp.PlayerNumber].push_back(temp);
        }
        else
        {
            TeamPacket lastpacket = info.m_received_packets[temp.PlayerNumber].back();
            if (timenow - lastpacket.ReceivedTime > 2000)
            {   // if there have been no packets recently from this player always accept the packet
                info.m_received_packets[temp.PlayerNumber].push_back(temp);
            }
            else if (temp.ID > lastpacket.ID)
            {   // avoid out of order packets by only adding recent packets that have a higher ID
                info.m_received_packets[temp.PlayerNumber].push_back(temp);
            }
        }
    }
    else
    {
        #if DEBUG_NETWORK_VERBOSITY > 0
            debug << ">>TeamInformation. Rejected team packet:";
            temp.summaryTo(debug);
            debug << endl;
        #endif
    }
    return input;
}

istream& operator>> (istream& input, TeamInformation* info)
{
    if (info != NULL)
        input >> (*info);
    return input;
}
*/
