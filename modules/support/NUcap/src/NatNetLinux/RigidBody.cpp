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

#include "RigidBody.h"

#include <cstring>

RigidBody::RigidBody()
  : _id(-1)
  , _loc()
  , _ori()
  , _markers()
  , _mId()
  , _mSize()
  , _mErr()
  , _trackingValid(true) {
}

RigidBody::RigidBody(const RigidBody & other)
  : _id(other._id)
  , _loc(other._loc)
  , _ori(other._ori)
  , _markers(other._markers)
  , _mId(other._mId)
  , _mSize(other._mSize)
  , _mErr(other._mErr)
  , _trackingValid(other._trackingValid) {
}

RigidBody::~RigidBody(){}

RigidBody& RigidBody::operator=(const RigidBody& other) {
    _id = other._id;
    _loc = other._loc;
    _ori = other._ori;
    _markers = other._markers;
    _mId = other._mId;
    _mSize = other._mSize;
    _mErr = other._mErr;
    _trackingValid = other._trackingValid;

    return *this;
}

int RigidBody::id() const {
    return _id;
}

Point3f RigidBody::location() const {
    return _loc;
}

Quaternion4f RigidBody::orientation() const {
    return _ori;
}

const std::vector<Point3f>& RigidBody::markers() const {
    return _markers;
}

bool RigidBody::trackingValid() const {
    return _trackingValid;
}

const char* RigidBody::unpack(const char* data, char nnMajor, char nnMinor) {
    int i;
    float x,y,z;

    // Rigid body ID
    memcpy(&_id,data,4); data += 4;

    // Location and orientation.
    memcpy(&_loc.x, data, 4); data += 4;
    memcpy(&_loc.y, data, 4); data += 4;
    memcpy(&_loc.z, data, 4); data += 4;
    memcpy(&_ori.qx, data, 4); data += 4;
    memcpy(&_ori.qy, data, 4); data += 4;
    memcpy(&_ori.qz, data, 4); data += 4;
    memcpy(&_ori.qw, data, 4); data += 4;

    // Associated markers
    int nMarkers = 0;
    memcpy(&nMarkers, data, 4); data += 4;
    for(i = 0; i < nMarkers; ++i) {
        memcpy(&x, data, 4); data += 4;
        memcpy(&y, data, 4); data += 4;
        memcpy(&z, data, 4); data += 4;
        _markers.push_back(Point3f{x, y, z});
    }

    if(nnMajor >= 2) {
        // Marker IDs
        uint32_t id = 0;
        for(i = 0; i < nMarkers; ++i) {
            memcpy(&id,data,4); data += 4;
            _mId.push_back(id);
        }

        // Marker sizes
        float size;
        for(i = 0; i < nMarkers; ++i) {
            memcpy(&size,data,4); data += 4;
            _mSize.push_back(size);
        }

        if(((nnMajor==2) && (nnMinor >= 6)) || (nnMajor > 2) || (nnMajor == 0)) {
            uint16_t tmp;
            memcpy(&tmp, data, 2); data += 2;
            _trackingValid = tmp & 0x01;
        }

        // Mean marker error
        memcpy(&_mErr,data,4); data += 4;
    }

    return data;
}

std::ostream& operator<<(std::ostream& s, const RigidBody& body) {
    s
    << "    Rigid Body: " << body.id() << std::endl
    << "      loc: " << body.location()
    << "      ori: " << body.orientation() << std::endl;

    return s;
}
