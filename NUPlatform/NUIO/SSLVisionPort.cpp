/*! @file TeamPort.cpp
    @brief Implementation of TeamPort class.

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

#include "SSLVisionPort.h"
#include "debug.h"
#include "debugverbositynetwork.h"
#include "SSLVisionPacket.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "Behaviour/TeamInformation.h"

#include <string>

/*! @brief Constructs a TeamPort
    @param nubotteaminformation the public nubot team information class
    @param portnumber the port number to broadcast on, and listen on
 */
SSLVisionPort::SSLVisionPort(NUSensorsData *nubotsensors, TeamInformation* teamInfo, int portnumber, bool ignoreself): UdpPort(std::string("SSLVisionPort"), portnumber, ignoreself)
{
    m_sensor_data = nubotsensors;
    m_team_information = teamInfo;
    m_packet = new SSLVisionPacket();
}

/*! @brief Closes the job port
 */
SSLVisionPort::~SSLVisionPort()
{
    if(m_packet != NULL) delete m_packet;
}

/*! @brief Copies the received data into the public nubot joblist
    @param buffer containing the joblist
*/
void SSLVisionPort::handleNewData(std::stringstream& buffer)
{
    #if DEBUG_NETWORK_VERBOSITY > 0
    debug << "SSLVisionPort::handleNewData()." << std::endl;
    #endif
    string s_buffer = buffer.str();
//    if (s_buffer.size() == sizeof(SSLVisionPacket))
//    {   // discard team packets that are the wrong size
        buffer >> (*m_packet);
        writePacketToSensors(m_packet, m_sensor_data);
//    }
//    else
//        debug << "SSLVisionPort::handleNewData(). The received packet does not have the correct length: " << s_buffer.size() << " instead of " << sizeof(TeamPacket) << std::endl;
}
void SSLVisionPort::writePacketToSensors(SSLVisionPacket* packet, NUSensorsData* sensors)
{
    if(packet->robots.size() > 0)
    {
        for(unsigned int i = 0; i < packet->robots.size(); i++)
        {
            if(packet->robots[i].id == m_team_information->getPlayerNumber())
            {
                double currTime = sensors->CurrentTime;
                // Position Data
                std::vector<float> gpsData(2,0.0f);
                gpsData[0] = packet->robots[i].location.x;
                gpsData[1] = packet->robots[i].location.y;
                sensors->setGPSValues(currTime,gpsData,false);

                // Orientation Data
                std::vector<float> compassData(1,0.0f);
                compassData[0] = packet->robots[i].heading;
                sensors->setCompassValues(currTime,compassData,false);
                debug << "SSLVisionPort. Position data received - gps: (" << gpsData[0] << "," << gpsData[1];
                debug << ") Heading: " << compassData[0] << std::endl;
            }
            else
            {
                debug << "SSLVisionPort. Data For Player " << packet->robots[i].id << " received." << endl;
            }
        }
    }
}
