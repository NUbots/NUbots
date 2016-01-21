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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <cstdint>
#include <iostream>
#include <vector>

#include "Quaternion4f.h"

/*!
 * \brief Rigid body
 * \author Philip G. Lee
 *
 * This class is a composition of markers that describe a rigid body. The basic
 * traits of the rigid body are its 3D location() and orientation(). Rigid
 * bodies can be created in Optitrack's Motive:Tracker software.
 */
class RigidBody {
public:

    //! \brief Default constructor
    RigidBody();

    //! \brief Copy constructor
    RigidBody(const RigidBody& other);

    ~RigidBody();

    //! \brief Assignment operator
    RigidBody& operator=(const RigidBody& other);

    //! \brief ID of this RigidBody
    int id() const;
    //! \brief Location of this RigidBody
    Point3f location() const;
    //! \brief Orientation of this RigidBody
    Quaternion4f orientation() const;
    //! \brief Vector of markers that make up this RigidBody
    const std::vector<Point3f>& markers() const;
    //! \brief True if the tracking is valid. Used in NatNet version >= 2.6.
    bool trackingValid() const;

    /*!
     * \brief Unpack rigid body data from raw packed data.
     *
     * \param data pointer to packed data representing a RigidBody
     * \param nnMajor major version of NatNet used to construct the packed data
     * \param nnMinor Minor version of NatNet packets used to read this frame
     * \returns pointer to data immediately following the RigidBody data
     */
    const char* unpack(const char* data, char nnMajor, char nnMinor);

private:
    int _id;
    Point3f _loc;
    Quaternion4f _ori;
    // List of [x,y,z] positions of each marker.
    std::vector<Point3f> _markers;

    // NOTE: If NatNet.major >= 2
    // List of marker IDs (each uint32_t)
    std::vector<uint32_t> _mId;
    // List of marker sizes (each float)
    std::vector<float> _mSize;
    // Mean marker error
    float _mErr;

    // NOTE: If NatNet version >= 2.6
    bool _trackingValid;
};

//! \brief Output operator for RigidBody.
std::ostream& operator<<(std::ostream& s, const RigidBody& body);

#endif /*RIGIDBODY_H*/
