/*! @file JobPort.h
    @brief Declaration of JobPort class.

    @class JobPort
    @brief JobPort a network port for sending jobs wirelessly to/from robots

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
#ifndef JOBPORT_H
#define JOBPORT_H

#include "UdpPort.h"
#include <string>

class JobList;

class JobPort : public UdpPort
{
public:
    JobPort(JobList* nubotjobs);
    ~JobPort();
    
    void setTargetAddress(std::string ipaddress);
    void setBroadcast();
    
    friend JobPort& operator<<(JobPort& port, JobList& jobs);
    friend JobPort& operator<<(JobPort& port, JobList* jobs);
private:
    void handleNewData(std::stringstream& buffer);
public:
private:
    JobList* m_jobs;
};

#endif

