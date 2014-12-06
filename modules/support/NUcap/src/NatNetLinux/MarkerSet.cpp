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

#include "MarkerSet.h"

#include <cstring>

MarkerSet::MarkerSet()
  : _name()
  , _markers() {}

MarkerSet::~MarkerSet(){}

MarkerSet::MarkerSet(const MarkerSet & other)
  : _name(other._name)
  , _markers(other._markers) {}

MarkerSet& MarkerSet::operator=(const MarkerSet& other) {
    _name = other._name;
    _markers = other._markers;
    return *this;
}

const std::string& MarkerSet::name() const {
    return _name;
}

const std::vector<Point3f>& MarkerSet::markers() const {
    return _markers;
}

const char* MarkerSet::unpack(const char* data) {
    char n[256];
    n[255] = '\0';

    int numMarkers;
    int i;
    float x, y, z;

    strncpy(n, data, sizeof(n)-1);
    _name = n;
    data += strlen(n) + 1;

    memcpy(&numMarkers, data, 4); data += 4;
    for(i = 0; i < numMarkers; ++i) {
        memcpy(&x, data, 4); data += 4;
        memcpy(&y, data, 4); data += 4;
        memcpy(&z, data, 4); data += 4;
        _markers.push_back(Point3f{x, y, z});
    }

    return data;
}

//! \brief Output operator to print human-readable text describin a MarkerSet
std::ostream& operator<<(std::ostream& s, const MarkerSet& set) {
    size_t i, size;

    s
    << "    MarkerSet: '" << set.name() << "'" << std::endl;

    const std::vector<Point3f>& markers = set.markers();
    size = markers.size();

    for(i = 0; i < size; ++i) {
        s << "      " << markers[i];
    }

    s << std::endl;

    return s;
}
