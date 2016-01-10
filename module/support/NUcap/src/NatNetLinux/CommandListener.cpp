/*
 * CommandListener.cpp is part of NatNetLinux, and is Copyright 2013-2014,
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

#include "CommandListener.h"

CommandListener::CommandListener(int sd)
  : _run(false)
  , _thread(0)
  , _sd(sd)
  , _nnMajor(0)
  , _nnMinor(0) {
    _nnVersionMutex.lock();
}

CommandListener::~CommandListener() {
    if(running()) {
        stop();
    }

    delete _thread;
}

void CommandListener::start() {
    _run = true;
    _thread = new std::thread(&CommandListener::_work, this, _sd);
}

void CommandListener::stop() {
    _run = false;
}

bool CommandListener::running() {
    return _run;
}

void CommandListener::join() {
    if(_thread) {
        _thread->join();
    }
}

void CommandListener::getNatNetVersion(unsigned char& major, unsigned char& minor) {
    _nnVersionMutex.lock();
    major = _nnMajor;
    minor = _nnMinor;
    _nnVersionMutex.unlock();
}

void CommandListener::_work(int sd) {

    const char* response;
    ssize_t len;
    NatNetPacket nnp;
    struct sockaddr_in senderAddress;
    socklen_t senderAddressLength = sizeof(senderAddress);
    NatNetSender sender;

    fd_set rfds;
    struct timeval timeout;

    while(_run) {

        // Wait for at most 1 second until the socket has data (recvfrom()
        // will not block). Otherwise, continue. This gives outside threads
        // a chance to kill this thread every second.
        timeout.tv_sec = 1; timeout.tv_usec = 0;
        FD_ZERO(&rfds); FD_SET(sd, &rfds);
        if(!select(sd+1, &rfds, 0, 0, &timeout)) {
            continue;
        }

        // blocking
        len = recvfrom(sd,
                       nnp.rawPtr(),
                       nnp.maxLength(),
                       0,
                       reinterpret_cast<struct sockaddr*>(&senderAddress),
                       &senderAddressLength);

        if(len <= 0) {
            continue;
        }

        switch(nnp.iMessage()) {
            case NatNetPacket::NAT_MODELDEF:
                //Unpack(nnp.rawPtr());
                break;
            case NatNetPacket::NAT_FRAMEOFDATA:
                //Unpack(nnp.rawPtr());
                break;
            case NatNetPacket::NAT_PINGRESPONSE:
                sender.unpack(nnp.read<char>(0));
                _nnMajor = sender.natNetVersion()[0];
                _nnMinor = sender.natNetVersion()[1];
                _nnVersionMutex.unlock();
                std::cout << "[Client] Server Software: " << sender.name() << std::endl;
                printf("[Client] NatNetVersion: %d.%d\n",sender.natNetVersion()[0],sender.natNetVersion()[1]);
                printf("[Client] ServerVersion: %d.%d\n",sender.version()[0],sender.version()[1]);
                break;
            case NatNetPacket::NAT_RESPONSE:
                response = nnp.read<char>(0);
                printf("Response : %s", response);
                break;
            case NatNetPacket::NAT_UNRECOGNIZED_REQUEST:
                printf("[Client] received 'unrecognized request'\n");
                break;
            case NatNetPacket::NAT_MESSAGESTRING:
                response = nnp.read<char>(0);
                printf("[Client] Received message: %s\n", response);
                break;
            default:
                break;
        } // end switch(nnp.iMessage)
    }
}
