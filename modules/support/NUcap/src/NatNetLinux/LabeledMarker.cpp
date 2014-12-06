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

#include "LabeledMarker.h"

#include <cstring>

LabeledMarker::LabeledMarker()
  : _id(0)
  , _p()
  , _size(0.f) {}

LabeledMarker::~LabeledMarker() {}

LabeledMarker::LabeledMarker(const LabeledMarker& other)
  : _id(other._id)
  , _p(other._p)
  , _size(other._size) {}

LabeledMarker& LabeledMarker::operator=(const LabeledMarker& other) {
    _id = other._id;
    _p = other._p;
    _size = other._size;
    return *this;
}

int LabeledMarker::id() const {
    return _id;
}

Point3f LabeledMarker::location() const {
    return _p;
}

float LabeledMarker::size() const {
    return _size;
}

const char* LabeledMarker::unpack( const char* data ) {
    memcpy(&_id, data, 4); data += 4;
    memcpy(&_p.x, data, 4); data += 4;
    memcpy(&_p.y, data, 4); data += 4;
    memcpy(&_p.z, data, 4); data += 4;
    memcpy(&_size, data, 4); data += 4;

    return data;
}
