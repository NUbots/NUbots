/*! @file NUIO.cpp
    @brief Implementation of  NUIO input/output class

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

#include "NUIO.h"
#include "NUPlatform/NUPlatform.h"
#include "Behaviour/Jobs.h"
#include "NUIO/RoboCupGameControlData.h"
#include "NUIO/NetworkPorts.h"
#include "Tools/Image/NUimage.h"
#include <sstream>
using namespace std;

#include "debug.h"

/*! @brief Create a new NUIO interface to network and log files
    @param robotnumber the unique number of the robot (this is used to select a port offset)
 */
NUIO::NUIO(int robotnumber)
{
#if DEBUG_NUSYSTEM_VERBOSITY > 4
    debug << "NUIO::NUIO(" << robotnumber << ")" << endl;
#endif
    
    m_gamecontroller_port = new UdpPort(GAMECONTROLLER_PORT);
    //m_team_port; The implementation of the team network is going to be platform specific!
    m_camera_port = new UdpPort(CAMERA_PORT + robotnumber);
    m_sensors_port = new UdpPort(SENSORS_PORT + robotnumber);
    m_actionators_port = new UdpPort(ACTIONATORS_PORT + robotnumber);
    m_vision_port = new TcpPort(VISION_PORT + robotnumber);
    m_wm_port = new UdpPort(WM_PORT + robotnumber);
    m_behaviour_port = new UdpPort(BEHAVIOUR_PORT + robotnumber);
    m_motion_port = new UdpPort(MOTION_PORT + robotnumber);
    m_jobs_port = new UdpPort(JOBS_PORT + robotnumber);
}

NUIO::~NUIO()
{
#if DEBUG_NUSYSTEM_VERBOSITY > 4
    debug << "NUIO::~NUIO()" << endl;
#endif
    if (m_gamecontroller_port != NULL)
        delete m_gamecontroller_port;
    if (m_team_port != NULL)
        delete m_team_port;
    if (m_camera_port != NULL)
        delete m_camera_port;
    if (m_sensors_port != NULL)
        delete m_sensors_port;
    if (m_actionators_port != NULL)
        delete m_actionators_port;
    if (m_vision_port != NULL)
        delete m_vision_port;
    if (m_wm_port != NULL)
        delete m_wm_port;
    if (m_behaviour_port != NULL)
        delete m_behaviour_port;
    if (m_motion_port != NULL)
        delete m_motion_port;
    if (m_jobs_port != NULL)
        delete m_jobs_port;
}

/*! @brief Stream insertion operator for NUSensorsData
    @param io the nuio stream object
    @param sensors the sensor data to stream
 */
NUIO& operator<<(NUIO& io, const NUSensorsData& sensors)
{
    stringstream buffer;
    buffer << sensors;
    
    io.m_sensors_port->sendData(buffer);
    return io;
}

/*! @brief Stream insertion operator for a pointer to NUSensorsData
    @param io the nuio stream object
    @param sensors the pointer to the sensor data to put in the stream
 */
NUIO& operator<<(NUIO& io, const NUSensorsData* sensors)
{
    io << *sensors;
    return io;
}

/*! @brief Stream insertion operator for a JobList
    @param io the nuio stream object
    @param jobs the job list to put in the stream
 */
NUIO& operator<<(NUIO& io, JobList& jobs)
{
    stringstream buffer;
    buffer << jobs;
    
    io.m_jobs_port->sendData(buffer);
    return io;
}

/*! @brief Stream insertion operator for a pointer to a JobList
    @param io the nuio stream object
    @param jobs the pointer to the job list to put in the stream
 */
NUIO& operator<<(NUIO& io, JobList* jobs)
{
    io << *jobs;
    return io;
}

/*! @brief Stream extraction operator for a JobList
    @param io the nuio stream object to extract the new jobs from
    @param jobs the job list to add the extracted jobs
 */
NUIO& operator>>(NUIO& io, JobList& jobs)
{
#if DEBUG_NUSYSTEM_VERBOSITY > 4
    debug << "NUIO >> JobList" << endl;
#endif
    network_data_t netdata = io.m_jobs_port->receiveData();
    if (netdata.size > 0)
    {
        #if DEBUG_NUSYSTEM_VERBOSITY > 3
            debug << "NUIO >> JobList received: " << netdata.size << endl;
        #endif
        #if DEBUG_NUSYSTEM_VERBOSITY > 4
            debug << "NUIO >> JobList received: ";
            for (int i=0; i<netdata.size; i++)
                debug << netdata.data[i];
            debug << endl;
        #endif
        stringstream buffer;
        buffer.write(reinterpret_cast<char*>(netdata.data), netdata.size);
        buffer >> jobs;
    }
    return io;
}

/*! @brief Stream extraction operator for a JobList
    @param io the nuio stream object to extract the new jobs from
    @param jobts the pointer to the job list to add the extracted jobs
 */
NUIO& operator>>(NUIO& io, JobList* jobs)
{
    io >> *jobs;
    return io;
}
/*! @brief Stream insertion operator for NUimage
    @param io the nuio stream object
    @param sensors the NUimage data to stream
 */
NUIO& operator<<(NUIO& io, NUimage& p_image)
{
    network_data_t netdata = io.m_vision_port->receiveData();
    if(netdata.size > 0)
    {
        stringstream buffer;
        buffer << p_image;    
        io.m_vision_port->sendData(buffer);
    }
    return io;
}

/*! @brief Stream insertion operator for a pointer to NUimage
    @param io the nuio stream object
    @param sensors the pointer to the NUimage data to put in the stream
 */
NUIO& operator<<(NUIO& io, NUimage* p_image)
{
    io << *p_image;
    return io;
}

