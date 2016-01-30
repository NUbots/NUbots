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

#include "Point3f.h"

#include <iomanip>

std::ostream& operator<<(std::ostream& s, const Point3f& point) {
    std::ios::fmtflags f(s.flags());

    s
    << std::fixed
    << "( "
    << std::setprecision(3) << point.x << ", "
    << std::setprecision(3) << point.y << ", "
    << std::setprecision(3) << point.z << " )" << std::endl;

    s.flags(f);

    return s;
}
