/*
 * NatNet.h is part of NatNetLinux, and is Copyright 2013-2014,
 * Philip G. Lee <rocketman768@gmail.com>
 *
 * NatNetLinux is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NatNetLinux is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NatNetLinux.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NatNet.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>

#include <unistd.h>
#include <arpa/inet.h>

struct sockaddr_in NatNet::createAddress(uint32_t inAddr, uint16_t port) {
    struct sockaddr_in ret;
    memset(&ret, 0, sizeof(ret));
    ret.sin_family = AF_INET;
    ret.sin_port = htons(port);
    ret.sin_addr.s_addr = inAddr;

    return ret;
}

int NatNet::createCommandSocket(uint32_t inAddr, uint16_t port) {
    // Asking for a buffer of 1MB = 2^20 bytes. This is what NP does, but this
    // seems far too large on Linux systems where the max is usually something
    // like 256 kB.
    const int rcvBufSize = 0x100000;
    int sd;
    int tmp=0;
    socklen_t len=0;
    struct sockaddr_in sockAddr = createAddress(inAddr, port);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sd < 0) {
        std::cerr << "Could not open socket. Error: " << errno << std::endl;
        exit(1);
    }

    // Bind socket to the address.
    tmp = bind( sd, (struct sockaddr*)&sockAddr, sizeof(sockAddr) );
    if(tmp < 0) {
        std::cerr << "Could not bind socket. Error: " << errno << std::endl;
        close(sd);
        exit(1);
    }

    int value = 1;
    tmp = setsockopt( sd, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(value) );
    if(tmp < 0) {
        std::cerr << "Could not set socket to broadcast mode. Error: " << errno << std::endl;
        close(sd);
        exit(1);
    }

    setsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBufSize, sizeof(rcvBufSize));
    getsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&tmp, &len);
    if(tmp != rcvBufSize) {
        std::cerr << "WARNING: Could not set receive buffer size. Asked for "
                  << rcvBufSize << "B got " << tmp << "B" << std::endl;
    }

    return sd;
}

int NatNet::createDataSocket(uint32_t inAddr, uint16_t port, uint32_t multicastAddr) {
    int sd;
    int value;
    int tmp;
    struct ip_mreq group;
    struct sockaddr_in localSock = createAddress(INADDR_ANY, port);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    value = 1;
    tmp = setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
    if(tmp < 0) {
       std::cerr << "ERROR: Could not set socket option." << std::endl;
       close(sd);
       return -1;
    }

    // Bind the socket to a port.
    bind(sd, (struct sockaddr*)&localSock, sizeof(localSock));

    // Connect a local interface address to the multicast interface address.
    group.imr_multiaddr.s_addr = multicastAddr;
    group.imr_interface.s_addr = inAddr;
    tmp = setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&group, sizeof(group));
    if(tmp < 0) {
       std::cerr << "ERROR: Could not add the interface to the multicast group." << std::endl;
       close(sd);
       return -1;
    }

    return sd;
}

int NatNet::createDataSocket(uint32_t inAddr, uint16_t port) {

    return createDataSocket(std::forward<uint32_t>(inAddr), std::forward<uint16_t>(port), inet_addr("239.255.42.99"));
}
