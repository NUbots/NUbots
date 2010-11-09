/*! @file SSLVisionPort.h
    @brief Declaration of SSLVisionPort class.

    @class SSLVisionPort
    @brief SSLVisionPort a network port for receiving ssl vision position data

    @author Steven Nicklin
 
 Copyright (c) 2010 Steven Nicklin
 
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
#ifndef SSLVISIONPORT_H
#define SSLVISIONPORT_H

#include "UdpPort.h"

class NUSensorsData;
class SSLVisionPacket;
class TeamInformation;

class SSLVisionPort : public UdpPort
{
public:
    SSLVisionPort(NUSensorsData *nubotsensors, TeamInformation* teamInfo, int portnumber, bool ignoreself = true);
    ~SSLVisionPort();
    
private:
    void handleNewData(std::stringstream& buffer);
    void writePacketToSensors(SSLVisionPacket* packet, NUSensorsData* sensors);
public:
private:
    TeamInformation* m_team_information;
    NUSensorsData* m_sensor_data;
    SSLVisionPacket* m_packet;
};

#endif

