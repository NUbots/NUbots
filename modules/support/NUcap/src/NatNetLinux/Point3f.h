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

#ifndef POINT3F_H
#define POINT3F_H

#include <iostream>

/*!
 * \brief Simple 3D point
 * \author Philip G. Lee
 */
struct Point3f {
    float x;
    float y;
    float z;
};

//! \brief Output operator for Point3f.
std::ostream& operator<<(std::ostream& s, const Point3f& point);

#endif /*POINT3F_H*/
