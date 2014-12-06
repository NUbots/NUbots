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

#ifndef MARKERSET_H
#define MARKERSET_H

#include <iostream>
#include <vector>

#include "Point3f.h"

/*!
 * \brief A set of markers
 * \author Philip G. Lee
 */
class MarkerSet {
public:
    //! \brief Default constructor
    MarkerSet();

    ~MarkerSet();

    //! \brief Copy constructor
    MarkerSet(const MarkerSet& other);

    //! \brief Assignment operator
    MarkerSet& operator=(const MarkerSet& other);

    //! \brief The name of the set
    const std::string& name() const;
    //! \brief Vector of markers making up the set
    const std::vector<Point3f>& markers() const;

    /*!
     * \brief Unpack the set from raw packed data
     *
     * \param data pointer to packed data representing the MarkerSet
     * \returns pointer to data immediately following the MarkerSet data
     */
    const char* unpack(const char* data);

private:

    std::string _name;
    std::vector<Point3f> _markers;
};

//! \brief Output operator to print human-readable text describin a MarkerSet
std::ostream& operator<<(std::ostream& s, const MarkerSet& set);

#endif /*MARKERSET_H*/
