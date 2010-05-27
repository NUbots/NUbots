/*! @file NUIO.h
    @brief Declaration of nuio class.
 
    @class NUIO
    @brief NUIO class for input and output to streams, files and networks

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

#ifndef NUIO_H
#define NUIO_H

#include <string>

class NUbot;

class GameControllerPort;
class TeamPort;
class JobPort;
class TcpPort;

class JobList;
class GameInformation;
class TeamInformation;
class NUimage;

class NUIO
{
// Functions:
public:
    NUIO() {};
    NUIO(NUbot* nubot);
    NUIO(GameInformation* gameinfo, TeamInformation* teaminfo, JobList* jobs);
    virtual ~NUIO();
    
    // JobList streaming
    friend NUIO& operator<<(NUIO& io, JobList& jobs);
    friend NUIO& operator<<(NUIO& io, JobList* jobs);
    void setJobPortTargetAddress(std::string ipaddress);
    void setJobPortToBroadcast();
    

    // Raw Image streaming 
    friend NUIO& operator<<(NUIO& io, NUbot& p_nubot);
    friend NUIO& operator<<(NUIO& io, NUbot* p_nubot);
    
protected:
    NUbot* m_nubot;
    GameControllerPort* m_gamecontroller_port;
    TeamPort* m_team_port;
    TcpPort* m_vision_port;
    JobPort* m_jobs_port;
};

#endif

