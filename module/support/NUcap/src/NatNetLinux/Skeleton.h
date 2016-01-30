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

#ifndef SKELETON_H
#define SKELETON_H

#include <vector>

#include "RigidBody.h"

/*!
 * \brief A composition of rigid bodies
 * \author Philip G. Lee
 *
 * A skeleton is simply a collection of RigidBody elements.
 */
class Skeleton {
public:

    Skeleton();

    Skeleton(const Skeleton& other);

    ~Skeleton();

    //! \brief ID of this skeleton.
    int id() const;
    //! \brief Vector of rigid bodies in this skeleton.
    const std::vector<RigidBody>& rigidBodies() const;

    /*!
     * \brief Unpack skeleton data from raw packed data.
     *
     * \param data pointer to packed data representing a Skeleton
     * \param nnMajor major version of NatNet used to construct the packed data
     * \param nnMinor Minor version of NatNet packets used to read this frame
     * \returns pointer to data immediately following the Skeleton data
     */
    const char* unpack(const char* data, char nnMajor, char nnMinor);

private:
    int _id;
    std::vector<RigidBody> _rBodies;
};

#endif /*SKELETON_H*/
