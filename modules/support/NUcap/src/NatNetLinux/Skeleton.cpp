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

#include "Skeleton.h"

#include <cstring>

Skeleton::Skeleton()
  : _id(0)
  , _rBodies() {}

Skeleton::Skeleton(const Skeleton & other)
  : _id(other._id)
  , _rBodies(other._rBodies) {}

Skeleton::~Skeleton() {}

int Skeleton::id() const {
    return _id;
}

const std::vector<RigidBody>& Skeleton::rigidBodies() const {
    return _rBodies;
}

const char* Skeleton::unpack(const char* data, char nnMajor, char nnMinor) {
    int i;
    int numRigid = 0;

    memcpy(&_id,data,4); data += 4;
    memcpy(&numRigid,data,4); data += 4;

    for(i = 0; i < numRigid; ++i) {
        RigidBody b;
        data = b.unpack( data, nnMajor, nnMinor );
        _rBodies.push_back(b);
    }

    return data;
}
