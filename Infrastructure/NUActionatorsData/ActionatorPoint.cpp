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

#include "debug.h"
#include "debugverbositynuactionators.h"

/*! @brief Constructs an ActionatorPoint that holds a single data value
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(double time, float data)
{
    Time = time;
    FloatData = new float(data);
    VectorData = 0;
    MatrixData = 0;
    ThreeDimData = 0;
    StringData = 0;
}

/*! @brief Constructs an ActionatorPoint that holds a vector of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(double time, const vector<float>& data)
{
    Time = time;
    FloatData = 0;
    VectorData = new vector<float>(data);
    MatrixData = 0;
    ThreeDimData = 0;
    StringData = 0;
}

/*! @brief Constructs an ActionatorPoint that holds a matrix of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(double time, const vector<vector<float> >& data)
{
    Time = time;
    FloatData = 0;
    VectorData = 0;
    MatrixData = new vector<vector<float> >(data);
    ThreeDimData = 0;
    StringData = 0;
}

/*! @brief Constructs an ActionatorPoint that holds a three dimensional matrix of data values
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(double time, const vector<vector<vector<float> > >& data)
{
    Time = time;
    FloatData = 0;
    VectorData = 0;
    MatrixData = 0;
    ThreeDimData = new vector<vector<vector<float> > >(data);
    StringData = 0;
}

/*! @brief Constructs an ActionatorPoint that holds a string
    @param time the time the data will be applied
    @param data the actionator data
 */
ActionatorPoint::ActionatorPoint(double time, const string& data)
{
    Time = time;
    FloatData = 0;
    VectorData = 0;
    MatrixData = 0;
    ThreeDimData = 0;
    StringData = new string(data);
}

/*! @brief Destroy the ActionatorPoint */
ActionatorPoint::~ActionatorPoint()
{
    delete FloatData;
    FloatData = 0;
    delete VectorData;
    VectorData = 0;
    delete MatrixData;
    MatrixData = 0;
    delete ThreeDimData;
    ThreeDimData = 0;
    delete StringData;
    StringData = 0;
}

/*! @brief operator< for comparing two points */
bool ActionatorPoint::operator< (const ActionatorPoint& other) const
{
    return Time < other.Time;
}

/*! @brief operator<< for outputing the contents of an actionator_point */
ostream& operator<< (ostream& output, const ActionatorPoint& p)
{
    output << p.Time << ": ";
    /*if (p.FloatData)
        output << p.FloatData;
    if (p.VectorData)
        output << p.VectorData;
    if (p.MatrixData)
        output << p.MatrixData;
    if (p.ThreeDimData)
        output << p.ThreeDimData;
    if (p.String)
        output << p.String;*/
    return output;
}


