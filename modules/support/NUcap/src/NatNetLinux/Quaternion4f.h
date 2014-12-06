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

#ifndef QUATERNION4F_H
#define QUATERNION4F_H

#include <iostream>
#include <vector>

#include "Point3f.h"

/*!
 * \brief Quaternion for 3D rotations and orientation
 * \author Philip G. Lee
 *
 * For more information, please see [Quaternions on Wikipedia](http://en.wikipedia.org/wiki/Quaternion).
 * A quaternion is a non-commutative group which describes 3D rotations, and whose group
 * operation (multiplication) represents rotation composition (A*B means rotate
 * by B, then by A). Since it is a proper group, there are no uninvertible
 * rotations (like with Euler angles). It is also parameterized by the smallest
 * number of parameters (4), unlike a 3x3 matrix (9).
 */
class Quaternion4f {
public:
    float qx;
    float qy;
    float qz;
    float qw;

    //! \brief Default constructor. Without parameters, returns the identity (no rotation).
    Quaternion4f(float qx = 0.f, float qy = 0.f, float qz = 0.f, float qw = 1.f);

    ~Quaternion4f();

    //! \brief Copy constructor
    Quaternion4f(const Quaternion4f& other);

    //! \brief Assignment operator
    Quaternion4f& operator=(const Quaternion4f& other);

    //! \brief Quaternion multiplication/assignment
    Quaternion4f& operator*=(const Quaternion4f& rhs);

    //! \brief Quaternion multiplication (rotation composition)
    Quaternion4f operator*(const Quaternion4f& rhs) const;

    //! \brief Quaternion division/assignment
    Quaternion4f& operator/=(const Quaternion4f& rhs);

    //! \brief Quaternion division
    Quaternion4f operator/(const Quaternion4f& rhs) const;

    //! \brief Rotate a point using the quaternion.
    Point3f rotate(const Point3f& p) const;

private:

    // If the magnitude of the quaternion exceeds a tolerance, renormalize it
    // to have magnitude of 1.
    void renormalize();
};

//! \brief Output operator for Quaternion4f.
std::ostream& operator<<(std::ostream& s, const Quaternion4f& q);

#endif /*QUATERNION4F_H*/
