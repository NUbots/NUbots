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

#include "ioconfig.h"

class NUbot;

class UdpPort;
class TcpPort;

class JobList;
class NUimage;

class NUIO
{
// Functions:
public:
    NUIO(int robotnumber, NUbot* nubot);
    virtual ~NUIO();
    
    // JobList streaming
    friend NUIO& operator<<(NUIO& io, JobList& jobs);
    friend NUIO& operator<<(NUIO& io, JobList* jobs);
    friend NUIO& operator>>(NUIO& io, JobList& jobs);
    friend NUIO& operator>>(NUIO& io, JobList* jobs);
    

    // Raw Image streaming 
    friend NUIO& operator<<(NUIO& io, NUimage& p_image);
    friend NUIO& operator<<(NUIO& io, NUimage* p_image);
    
protected:
private:
    
// Members:
public:
protected:
private:
    NUbot* m_nubot;
    
    UdpPort* m_gamecontroller_port;
    UdpPort* m_team_port;
    TcpPort* m_vision_port;
    UdpPort* m_jobs_port;
};

#endif

