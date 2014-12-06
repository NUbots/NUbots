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

#include "Quaternion4f.h"

#include <cmath>
#include <iomanip>

Quaternion4f::Quaternion4f(float qx, float qy, float qz, float qw)
  : qx(qx)
  , qy(qy)
  , qz(qz)
  , qw(qw) {

    renormalize();
}

Quaternion4f::Quaternion4f(const Quaternion4f& other)
  : qx(other.qx)
  , qy(other.qy)
  , qz(other.qz)
  , qw(other.qw) {}

Quaternion4f::~Quaternion4f(){}

Quaternion4f& Quaternion4f::operator=(const Quaternion4f& other) {
    qx = other.qx;
    qy = other.qy;
    qz = other.qz;
    qw = other.qw;

    return *this;
}

Quaternion4f& Quaternion4f::operator*=(const Quaternion4f& rhs) {
    float x,y,z,w;

    x = qw*rhs.qw - qx*rhs.qx - qy*rhs.qy - qz*rhs.qz;
    y = qw*rhs.qx + qx*rhs.qw + qy*rhs.qz - qz*rhs.qy;
    z = qw*rhs.qy - qx*rhs.qz + qy*rhs.qw + qz*rhs.qx;
    w = qw*rhs.qz + qx*rhs.qy - qy*rhs.qx + qz*rhs.qw;

    qx = x;
    qy = y;
    qz = z;
    qw = w;

    renormalize();

    return *this;
}

Quaternion4f Quaternion4f::operator*(const Quaternion4f& rhs) const {
    Quaternion4f ret(*this);

    ret *= rhs;

    return ret;
}

Quaternion4f& Quaternion4f::operator/=(const Quaternion4f& rhs) {
    // Create the conjugate and multiply.
    Quaternion4f rhsConj(-rhs.qx, -rhs.qy, -rhs.qy, rhs.qw);
    *this *= rhsConj;

    return *this;
}

Quaternion4f Quaternion4f::operator/(const Quaternion4f& rhs) const {
    Quaternion4f ret(*this);
    ret /= rhs;
    return ret;
}

Point3f Quaternion4f::rotate(const Point3f& p) const {
    Point3f pout;

    pout.x = (1.f-2.f*qy*qy-2.f*qz*qz)*p.x + (2.f*qx*qy-2.f*qw*qz)*p.y + (2.f*qx*qz+2.f*qw*qy)*p.z;
    pout.y = (2.f*qx*qy+2.f*qw*qz)*p.x + (1.f-2.f*qx*qx-2.f*qz*qz)*p.y + (2.f*qy*qz+2.f*qw*qx)*p.z;
    pout.z = (2.f*qx*qz-2.f*qw*qy)*p.x + (2.f*qy*qz-2.f*qw*qx)*p.y + (1.f-2.f*qx*qx-2.f*qy*qy)*p.z;

    return pout;
}

void Quaternion4f::renormalize() {
    static const float tolHigh = 1.f+1e-6;
    static const float tolLow  = 1.f-1e-6;

    float mag = qx*qx + qy*qy + qw*qw + qz*qz;

    if( mag < tolLow || mag > tolHigh )
    {
        mag = sqrtf( mag );
        qx /= mag;
        qy /= mag;
        qz /= mag;
        qw /= mag;
    }
}

std::ostream& operator<<(std::ostream& s, const Quaternion4f& q) {

    std::ios::fmtflags f(s.flags());

    s
    << std::fixed
    << "(qx,qy,qz,qw) = ( "
    << std::setprecision(3) << q.qx << ", "
    << std::setprecision(3) << q.qy << ", "
    << std::setprecision(3) << q.qz << ", "
    << std::setprecision(3) << q.qw << " )" << std::endl;

    s.flags(f);

    return s;
}
