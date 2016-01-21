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

#ifndef LABELEDMARKER_H
#define LABELEDMARKER_H

#include "Point3f.h"

/*!
 * \brief A labeled marker.
 * \author Philip G. Lee
 */
class LabeledMarker {
public:

    //! \brief Default constructor.
    LabeledMarker();

    ~LabeledMarker();

    //! \brief Copy constructor.
    LabeledMarker(const LabeledMarker& other);

    //! \brief Assignment operator.
    LabeledMarker& operator=(const LabeledMarker& other);

    //! \brief ID of this marker.
    int id() const;
    //! \brief Location of this marker.
    Point3f location() const;
    //! \brief Size of this marker.
    float size() const;

    /*!
     * \brief Unpack the marker from packed data.
     *
     * \param data pointer to packed data representing a labeled marker
     * \returns pointer to data immediately following the labeled marker data
     */
    const char* unpack(const char* data);

private:
    int _id;
    Point3f _p;
    float _size;
};

#endif /*LABELEDMARKER_H*/