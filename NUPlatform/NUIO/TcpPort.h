/*! @file UdpPort.h
    @brief Declaration of UdpPort class.

    @class UdpPort
    @brief UdpPort class encapsulating a udp socket

    @author Aaron Wong
 
 Copyright (c) 2009 Aaron Wong
 
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

#ifdef WIN32

#include <winsock.h>
#define socklen_t int
#endif

#ifndef WIN32
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "nubotconfig.h"
#include "Tools/Threading/Thread.h"
class NUImage;
class NUSensorsData;
class SelfLocalisation;
class FieldObjects;

#include <sstream>
using namespace std;

typedef unsigned char byte;
#ifndef NETDATA
#define NETDATA
struct network_data_t
{
    int size;
    char* data;
};
#endif

class TcpPort : public Thread
{
public:
    TcpPort(int portnumber);
    virtual ~TcpPort();
    void sendData(network_data_t netData);
    void sendData(const NUImage& p_image, const NUSensorsData& p_sensors);
    #if defined(USE_LOCALISATION)
        void sendData(const SelfLocalisation& p_locwm, const FieldObjects& p_objects);
    #endif
    network_data_t receiveData();
private:
    void run();
public:
private:
    int m_sockfd;                       //!< the socket
    int m_port_number;                  //!< the port number of the socket
    sockaddr_in m_address;              //!< the socket address
    sockaddr_in m_broadcast_address;    //!< the socket broadcast address
    double m_time_last_receive;         //!< the time in milliseconds the last packet was received
    
    char m_data[10*1024];               //!< the data buffer for received data
    int m_message_size;                 //!< the number of bytes received (ie the number of valid bytes in m_data)
    bool m_has_data;                    //!< flag indicating whether new data has been received

    pthread_mutex_t m_socket_mutex;     //!< lock to prevent simultaneous reading and writing on the same port

    int m_clientSockfd;                 //!< Connected Clients socket

};

