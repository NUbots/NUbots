/*! @file UdpPort.h
    @brief Declaration of UdpPort class.

    @class UdpPort
    @brief UdpPort class encapsulating a udp socket

    @author Aaron Wong, Jason Kulk
 
 Copyright (c) 2009, 2010 Aaron Wong
 
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
#ifndef UDPPORT_H
#define UDPPORT_H

#ifdef WIN32
    #include <winsock.h>
    #define socklen_t int
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
#endif

#include "Tools/Threading/Thread.h"

#include <sstream>
#include <string>

class UdpPort : public Thread
{
public:
    UdpPort(std::string name, int portnumber, bool ignoreself = false);
    virtual ~UdpPort();
protected:
    void sendData(const std::stringstream& stream);
    virtual void handleNewData(std::stringstream& buffer) = 0;
private:
    void run();
    
protected:
    std::string m_port_name;            //!< the name of this port
    std::string m_host_name;            //!< the name of the host machine
    sockaddr_in m_local_address;        //!< the machine's local address
    sockaddr_in m_target_address;       //!< the socket target address
    
    double m_time_last_receive;         //!< the time in milliseconds the last packet was received
private:
    int m_sockfd;                       //!< the socket
    int m_port_number;                  //!< the port number of the socket
    bool m_ignore_self;                 //!< true if you want to ignore your own transmissions
    
    sockaddr_in m_address;              //!< the socket address

    pthread_mutex_t m_socket_mutex;     //!< lock to prevent simultaneous reading and writing on the same port

};

#endif

