/*! @file TeamPort.h
    @brief Declaration of TeamPort class.

    @class TeamPort
    @brief TeamPort a network port for sending jobs wirelessly to/from robots

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
#ifndef TEAMPORT_H
#define TEAMPORT_H

#include "UdpPort.h"

class TeamInformation;
class TeamTransmissionThread;

class TeamPort : public UdpPort
{
public:
    TeamPort(TeamInformation* nubotteaminformation, int portnumber, bool ignoreself = true);
    ~TeamPort();
    
private:
    void handleNewData(std::stringstream& buffer);
public:
private:
    TeamInformation* m_team_information;
    friend class TeamTransmissionThread;
    TeamTransmissionThread* m_team_transmission_thread;
};

#endif

