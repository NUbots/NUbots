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
using namespace std;

class TeamPacket
{
public:
    unsigned long ID;
    double SentTime;
    double ReceivedTime;
    char PlayerNumber;
    char TeamNumber;
    float TimeToBall;
    
    void summaryTo(ostream& output);
    friend ostream& operator<< (ostream& output, const TeamPacket& packet);
    friend istream& operator>> (istream& input, TeamPacket& packet);
};

class TeamInformation
{
public:
    TeamInformation(int playernum, int teamnum, NUSensorsData* data, NUActionatorsData* actions, FieldObjects* fieldobjects);
    ~TeamInformation();
    
    int getPlayerNumber() {return m_player_number;};
    int getTeamNumber() {return m_team_number;};
    bool amIClosestToBall();
    
    friend ostream& operator<< (ostream& output, TeamInformation& info);
    friend ostream& operator<< (ostream& output, TeamInformation* info);
    friend istream& operator>> (istream& input, TeamInformation& info);
    friend istream& operator>> (istream& input, TeamInformation* info);
private:
    void initTeamPacket();
    void updateTeamPacket();
    float getTimeToBall();
private:
    int m_player_number;
    int m_team_number;
    
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    FieldObjects* m_objects;
    
    TeamPacket m_packet;                                                //!< team packet to send
    vector<boost::circular_buffer<TeamPacket> > m_received_packets;     //!< team packets received from other robots
};


#endif

