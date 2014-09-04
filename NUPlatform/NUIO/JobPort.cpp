/*! @file JobPort.cpp
    @brief Implementation of JobPort class.

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

#include "JobPort.h"
#include "NetworkPortNumbers.h"
#include "Infrastructure/Jobs/JobList.h"

#include "debug.h"
#include "debugverbositynetwork.h"

/*! @brief Constructs a JobPort
    @param nubotjobs the public nubot job std::list
 */
JobPort::JobPort(JobList* nubotjobs): UdpPort(std::string("JobPort"), JOBS_PORT, true)
{
    #if DEBUG_NETWORK_VERBOSITY > 0
        debug << "JobPort::JobPort(" << nubotjobs << ")" << std::endl;
    #endif
    m_jobs = nubotjobs;
}

/*! @brief Closes the job port
 */
JobPort::~JobPort()
{
#if DEBUG_NETWORK_VERBOSITY > 0
    debug << "JobPort::~JobPort()" << std::endl;
#endif
}

/*! @brief Sets the target ip address to that which is specified
    @param ipaddress the ip address to send jobs, eg. 192.168.1.5
    
    Note that this function will not work with bonjour addresses, it needs to be a proper ip address.
    If your ip address is invalid the address 255.255.255.255 will be used!
 */
void JobPort::setTargetAddress(std::string ipaddress)
{
    m_target_address.sin_addr.s_addr = inet_addr(ipaddress.c_str());
    #if DEBUG_NETWORK_VERBOSITY > 0
        debug << "JobPort::setTargetAddress() " << inet_ntoa(m_target_address.sin_addr) << std::endl;
    #endif
}

/*! @brief Sets the job port to broadcast, this means all robots on the network will execute the job :D
 */
void JobPort::setBroadcast()
{
    m_target_address = m_broadcast_address;
    #if DEBUG_NETWORK_VERBOSITY > 0
        debug << "JobPort::setTargetAddress() " << inet_ntoa(m_target_address.sin_addr) << std::endl;
    #endif
}

/*! @brief Send the jobs over the network
    @param port the jobport
    @param jobs the job std::list to send
 */
JobPort& operator<<(JobPort& port, JobList& jobs)
{   
    std::stringstream buffer;
    buffer << jobs;
    port.sendData(buffer);
    return port;
}

/*! @brief Send the jobs over the network
    @param port the jobport
    @param jobs the job std::list to send
 */
JobPort& operator<<(JobPort& port, JobList* jobs)
{
    std::stringstream buffer;
    buffer << *jobs;
    port.sendData(buffer);
    return port;
}

/*! @brief Copies the received data into the public nubot joblist
    @param buffer containing the joblist
*/
void JobPort::handleNewData(std::stringstream& buffer)
{
    #if DEBUG_NETWORK_VERBOSITY > 0
        debug << "JobPort::handleNewData()" << std::endl;
    #endif
    buffer >> *m_jobs;
    #if DEBUG_NETWORK_VERBOSITY > 0
        m_jobs->summaryTo(debug);
    #endif
}
