/*! @file EndEffectorTouch.h
    @brief Declaration of a class to handle pressure data for end effectors
    @author Jason Kulk
 
    @class EndEffectorTouch
    @brief A class to handle pressure data of end effectors to produce soft sensor data.
 
 	In particular, this class can calculate the following:
 		- Total force on the end effector
 		- Whether the end effector is in contact with something
 		- Whether the end effector is supporting the weight of the robot
 		- The time of the last impact with an object
 		- The centre of pressure
 
 	The related kinematic class EndEffector calculates the end effector's position and rotation.
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010, 2011 Jason Kulk
 
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

#ifndef ENDEFFECTOR_TOUCH_H
#define ENDEFFECTOR_TOUCH_H

#include "Infrastructure/NUData.h"

#include <vector>


class EndEffectorTouch
{
public:
    EndEffectorTouch();
    ~EndEffectorTouch();
    
    void calculate();

private:
    void calculate(const NUData::id_t& endeffector);
    void invalidate(const NUData::id_t& endeffector);
    
    void calculateForce(const NUData::id_t& endeffector);
    void calculateContact(const NUData::id_t& endeffector);
    void calculateCentreOfPressure(const NUData::id_t& endeffector);
    void calculateSupport(const NUData::id_t &endeffector);
    
    void getHull(const NUData::id_t& endeffector, std::vector<std::vector<float> >& hull);
private:
    std::vector<float> m_touch_data;								//!< the current touch data we are working with
    
    int m_id_offset;
    std::vector<float> m_min_forces;								//!< the min forces on each of the end effectors
    std::vector<float> m_max_forces;								//!< the max forces on each of the end effectors
    
    std::vector<std::vector<std::vector<float> > > m_hulls;				//!< the convex hulls for each of the end effectors [[[x,y],...,[x,y]],...,]
    
    float m_Nan;											//!< the value used to represent invalid data
    std::vector<float> m_Nan_all;								//!< the std::vector to invalidate all of the touch sensor
};

#endif


