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
#include "NUPlatform/NUSystem.h"

#include "debug.h"

TeamInformation::TeamInformation(int playernum, int teamnum, NUSensorsData* data, NUActionatorsData* actions)
{
    m_player_number = playernum;
    m_team_number = teamnum;
    m_data = data;
    m_actions = actions;
}


TeamInformation::~TeamInformation()
{
}

ostream& operator<< (ostream& output, const TeamInformation& info)
{
    output.write((char*) &info.m_player_number, sizeof(info.m_player_number));
    output.write((char*) &info.m_team_number, sizeof(info.m_team_number));
    nusystem->displayTeamPacketSent(info.m_actions);
    return output;
}

ostream& operator<< (ostream& output, const TeamInformation* info)
{
    if (info != NULL)
        output << (*info);
    return output;
}

istream& operator>> (istream& input, TeamInformation& info)
{
    nusystem->displayTeamPacketReceived(info.m_actions);
    return input;
}

istream& operator>> (istream& input, TeamInformation* info)
{
    if (info != NULL)
        input >> (*info);
    return input;
}
