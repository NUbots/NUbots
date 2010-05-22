/*! @file TeamTransmissionThread.h
    @brief Declaration of team packet transmission thread class.

    @class TeamTransmissionThread
    @brief The team packet transmission thread periodically transmits team information
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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
class TeamPort;

class TeamTransmissionThread : public PeriodicThread
{
public:
    TeamTransmissionThread(TeamPort* port, int period = 500);
    ~TeamTransmissionThread();
private:
    void periodicFunction();
    
private:
    TeamPort* m_port;
};

#endif

