/*! @file ActionatorPoint.cpp
    @brief Implementation of a container for a single actionator point
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ActionatorPoint.h"

#include "Tools/Math/StlVector.h"

#include "debug.h"
#include "debugverbositynuactionators.h"
using namespace boost;

/*! @brief Constructs an ActionatorPoint that holds a single data value
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const float& data)
{
    Time = time;
    FloatData = boost::shared_ptr<float>(new float(data));
}

/*! @brief Constructs an ActionatorPoint that holds a std::vector of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const std::vector<float>& data)
{
    Time = time;
    VectorData = boost::shared_ptr<std::vector<float> >(new std::vector<float>(data));
}

/*! @brief Constructs an ActionatorPoint that holds a matrix of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const std::vector<std::vector<float> >& data)
{
    Time = time;
    MatrixData = boost::shared_ptr<std::vector<std::vector<float> > >(new std::vector<std::vector<float> >(data));
}

/*! @brief Constructs an ActionatorPoint that holds a three dimensional matrix of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const std::vector<std::vector<std::vector<float> > >& data)
{
    Time = time;
    ThreeDimData = boost::shared_ptr<std::vector<std::vector<std::vector<float> > > >(new std::vector<std::vector<std::vector<float> > >(data));
}

/*! @brief Constructs an ActionatorPoint that holds a std::string
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const std::string& data)
{
    Time = time;
    StringData = boost::shared_ptr<std::string>(new std::string(data));
}

/*! @brief Constructs an ActionatorPoint that holds a std::vector of std::strings
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const double& time, const std::vector<std::string>& data)
{
    Time = time;
    VectorStringData = boost::shared_ptr<std::vector<std::string> >(new std::vector<std::string>(data));
}

/*! @brief Copy constructor for an ActionatorPoint. 
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(const ActionatorPoint& original)
{
    Time = original.Time;
    FloatData = original.FloatData;
    VectorData = original.VectorData;
    MatrixData = original.MatrixData;
    ThreeDimData = original.ThreeDimData;
    StringData = original.StringData;
    VectorStringData = original.VectorStringData;
}

/*! @brief Destroy the ActionatorPoint */
ActionatorPoint::~ActionatorPoint()
{
}

/*! @brief operator< for comparing two points */
bool ActionatorPoint::operator< (const ActionatorPoint& other) const
{
    return Time < other.Time;
}

/*! @brief operator<< for outputing the contents of an actionator_point */
std::ostream& operator<< (std::ostream& output, const ActionatorPoint& p)
{
    output << p.Time << ": ";
    if (p.FloatData)
        output << *p.FloatData;
    if (p.VectorData)
        output << *p.VectorData;
    if (p.MatrixData)
        output << p.MatrixData;
    if (p.ThreeDimData)
        output << p.ThreeDimData;
    if (p.StringData)
        output << p.StringData;
    if (p.VectorStringData)
        output << p.VectorStringData;
    return output;
}


