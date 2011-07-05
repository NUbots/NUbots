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
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "nubotdataconfig.h"
#include "Motion/Tools/MotionFileTools.h"
#include "Tools/Math/General.h"
#include "NUPlatform/NUPlatform.h"

#include <string>

/*! @brief Constructs a TeamPort
    @param nubotteaminformation the public nubot team information class
    @param portnumber the port number to broadcast on, and listen on
 */
SSLVisionPort::SSLVisionPort(NUSensorsData *nubotsensors, TeamInformation* teaminfo, int portnumber, bool ignoreself): UdpPort(std::string("SSLVisionPort"), portnumber, ignoreself)
{
    m_sensor_data = nubotsensors;
    m_team_info = teaminfo;
    m_packet = new SSLVisionPacket();
    ifstream idfile((string(CONFIG_DIR) + string("SslId.cfg")).c_str());      // the ssl id is stored in a config file
    if (idfile.is_open())                                                     // Use that if it exists, otherwise use the player number
    {
        m_ssl_id = static_cast<int>(MotionFileTools::toFloat(idfile));
        debug << "SslId.cfg found. Using (" << m_ssl_id << ")" << endl;
    }
    else
    {
        m_ssl_id = m_team_info->getPlayerNumber();
        debug << "SslId.cfg not found. Using player number instead (" << m_ssl_id << ")" << endl;
    }
    idfile.close();
	storage.open("/var/volatile/storage.log");	
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
    buffer >> (*m_packet);
    writePacketToSensors(m_packet, m_sensor_data);
    vector<float> led_red(3,0);
    led_red[0] = 1;
    Platform->toggle(NUPlatform::Led0, m_sensor_data->CurrentTime, led_red);

}

/*! @brief Writes the position information in the packet into the gps and compass sensors in NUSensorsData
    @param packet the sslvisionpacket just received
    @param sensors a pointer to the NUSensorsData
 */
void SSLVisionPort::writePacketToSensors(SSLVisionPacket* packet, NUSensorsData* sensors)
{
    if(packet->robots.size() > 0)
    {
        for(unsigned int i = 0; i < packet->robots.size(); i++)
        {
            if(packet->robots[i].id == m_ssl_id)
            {
                double currTime = sensors->CurrentTime;
                // Position Data
                std::vector<float> gpsData(2,0.0f);
                gpsData[0] = packet->robots[i].location.x;
                gpsData[1] = packet->robots[i].location.y;
                sensors->set(NUSensorsData::Gps, currTime, gpsData);

                // Orientation Data
                float compassData;
                float headyaw;
                sensors->getPosition(NUSensorsData::HeadYaw, headyaw);
                compassData = mathGeneral::normaliseAngle(packet->robots[i].heading - headyaw);           // as the marker is attached to the head, subtract the head yaw position
                sensors->set(NUSensorsData::Compass, currTime, compassData);
				
				// Record data into file
				storage<<"\n"<<gpsData[0]<<", "<<gpsData[1]<<", "<<compassData<<", "<<headyaw<<", "<<currTime;
				
                #if DEBUG_NETWORK_VERBOSITY > 0
                    debug << "SSLVisionPort. Position data received - gps: (" << gpsData[0] << "," << gpsData[1];
                    debug << ") Heading: " << compassData << std::endl;
                #endif
            }
            else
            {
                #if DEBUG_NETWORK_VERBOSITY > 0
                    debug << "SSLVisionPort. Data For Player " << packet->robots[i].id << " received." << endl;
                #endif
            }
        }
    }
}
