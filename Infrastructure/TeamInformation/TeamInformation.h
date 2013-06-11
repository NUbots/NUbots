/*! @file TeamInformation.h
    @brief Declaration of TeamInformation
 
    @class TeamInformation
    @brief A class to hold all of the team information

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

#ifndef TEAMINFORMATION_H
#define TEAMINFORMATION_H

class NUSensorsData;
class NUActionatorsData;
class FieldObjects;

#include <boost/circular_buffer.hpp>
#include <vector>
#include <iostream>
#include "Tools/FileFormats/TimestampedData.h"


#define TEAM_PACKET_STRUCT_HEADER "NUtm"

class TeamPacket
{
public:
    struct SharedBall 
    {
        float TimeSinceLastSeen;
        float X;
        float Y;
        float SRXX;
        float SRXY;
        float SRYY;
    };
    struct SharedSelf
    {
        float X;
        float Y;
        float Heading;
        float SDX;
        float SDY;
        float SDHeading;
    };
    char Header[4];
    unsigned long ID;
    double SentTime;
    double ReceivedTime;
    char PlayerNumber;
    char TeamNumber;
    float TimeToBall;
    SharedBall Ball;
    SharedSelf Self;

    std::string toString() const;
    std::ostream& toFile(std::ostream& output) const;
    std::istream& fromFile(std::istream& input);
    void summaryTo(std::ostream& output) const;
    friend std::ostream& operator<< (std::ostream& output, const TeamPacket& packet);
    friend std::istream& operator>> (std::istream& input, TeamPacket& packet);
};

class TeamInformation: public TimestampedData
{
public:
    typedef boost::circular_buffer<TeamPacket> PacketBuffer;
    typedef std::vector<PacketBuffer> PacketBufferArray;

    TeamInformation(int playernum=0, int teamnum=0);
    ~TeamInformation();
    
    int getPlayerNumber() {return m_player_number;};
    void setPlayerNumber(int num) {m_player_number = num;};
    int getTeamNumber() {return m_team_number;};
    bool amIClosestToBall();
    
    std::vector<TeamPacket::SharedBall> getSharedBalls() const;
    
    void UpdateTime(double newTime) {m_timestamp=newTime;};
    double GetTimestamp() const{return m_timestamp;};
    std::string toString() const;

    friend std::ostream& operator<< (std::ostream& output, const TeamInformation& info);
    //friend std::ostream& operator<< (std::ostream& output, TeamInformation* info);
    friend std::istream& operator>> (std::istream& input, TeamInformation& info);
    //friend std::istream& operator>> (std::istream& input, TeamInformation* info);

    TeamPacket generateTeamTransmissionPacket();
    void addReceivedTeamPacket(TeamPacket& receivedPacket);
private:
    void initTeamPacket();
    void updateTeamPacket();
    float getTimeToBall();
private:
    const float m_TIMEOUT;
    int m_player_number;
    int m_team_number;
    double m_timestamp;

    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    FieldObjects* m_objects;
    
    TeamPacket m_packet;                                                //!< team packet to send

    PacketBufferArray m_received_packets;     //!< team packets received from other robots
    
    std::vector<float> m_led_green;
    std::vector<float> m_led_red;
};


#endif

