/*
 * NatNetPacket.h is part of NatNetLinux, and is Copyright 2013-2014,
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

#include "NatNetPacket.h"

#include <cstdint>
#include <cstring>

NatNetPacket::NatNetPacket()
  : _data(new char[MAX_PACKETSIZE + 4])
  , _dataLen(MAX_PACKETSIZE + 4) {
}

NatNetPacket::NatNetPacket( const NatNetPacket& other )
  : _data(new char[other._dataLen])
  , _dataLen(other._dataLen) {

    memcpy( _data, other._data, other._dataLen );
}

NatNetPacket::~NatNetPacket() {
    delete[] _data;
}

NatNetPacket& NatNetPacket::operator=(const NatNetPacket& other) {
    // Careful with self-assignment
    if(_dataLen < other._dataLen) {
        delete[] _data;
        _dataLen = other._dataLen;
        _data = new char[_dataLen];
        // Can do memcpy, because we know the other isn't us.
        memcpy(_data, other._data, _dataLen);
    }
    else {
        memmove(_data, other._data, _dataLen);
    }

    return *this;
}

//! \brief Construct a "ping" packet.
NatNetPacket NatNetPacket::pingPacket() {
    NatNetPacket packet;

    uint16_t m = NAT_PING;
    uint16_t len = 0;

    *reinterpret_cast<uint16_t*>(packet._data)   = m;
    *reinterpret_cast<uint16_t*>(packet._data+2) = len;

    return packet;
}

int NatNetPacket::send(int sd) const {

    // Have to prepend '::' to avoid conflicting with NatNetPacket::send().
    return ::send(sd, _data, 4 + nDataBytes(), 0);
}

int NatNetPacket::send(int sd, struct sockaddr_in destAddr) const {

    return sendto(sd, _data, 4 + nDataBytes(), 0, (sockaddr*)&destAddr, sizeof(destAddr));
}

char* NatNetPacket::rawPtr() {
    return _data;
}

const char* NatNetPacket::rawPtr() const {
    return _data;
}

const char* NatNetPacket::rawPayloadPtr() const {
    return _data+4;
}

size_t NatNetPacket::maxLength() const {
    return _dataLen;
}

//! \brief Get the message type.
NatNetPacket::NatNetMessageID NatNetPacket::iMessage() const {

    unsigned short m = *reinterpret_cast<unsigned short*>(_data);
    return static_cast<NatNetMessageID>(m);
}

//! \brief Get the number of bytes in the payload.
unsigned short NatNetPacket::nDataBytes() const {
    return *reinterpret_cast<unsigned short*>(_data+2);
}
