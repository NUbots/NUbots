/*! @file ActionatorPoint.h
    @brief Declaration of an actionator point
    @author Jason Kulk
 
    @class ActionatorPoint
    @brief A container for an actionator point

    ActionatorPoint can handle several different types of data; floats, vectors, vector<vector>s vector<vector<vector>> and strings.
 
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

#ifndef ACTIONATOR_POINT_H
#define ACTIONATOR_POINT_H

#include <vector>
#include <string>
using namespace std;

class ActionatorPoint 
{
public:
    ActionatorPoint(double time, const float data);
    ActionatorPoint(double time, const vector<float>& data);
    ActionatorPoint(double time, const vector<vector<float> >& data);
    ActionatorPoint(double time, const vector<vector<vector<float> > >& data);
    ActionatorPoint(double time, const string& data);
    ~ActionatorPoint();
    
    bool operator< (const ActionatorPoint& other) const;
    friend ostream& operator<< (ostream& output, const ActionatorPoint& p);
public:
    double Time;                                    //!< the time the actionator point will be completed in milliseconds since epoch or program start
    float* FloatData;                               //!< a pointer to the float data associated with the actionator point
    vector<float>* VectorData;                      //!< a pointer to the vector data associated with the actionator point
    vector<vector<float> >* MatrixData;             //!< a pointer to the matrix data associated with the actionator point
    vector<vector<vector<float> > >* ThreeDimData;  //!< a pointer to the three dimensional matrix associated with the actionator point
    string* StringData;                             //!< a pointer to the string assocaiated with the actionator point
};

#endif

