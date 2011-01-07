/*! @file BearSensors.h
    @brief Declaration of Bear sensors class

    @author Jason Kulk
 
    @class BearSensors
    @brief A Bear sensors
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#ifndef BEARSENSORS_H
#define BEARSENSORS_H

#include "NUPlatform/NUSensors.h"
#include "Infrastructure/NUData.h"
class Motors;

#include <vector>
#include <string>


class BearSensors : public NUSensors
{
public:
    BearSensors(Motors* motors);
    ~BearSensors();
    
    void copyFromHardwareCommunications();
    void copyFromJoints();
    
private:
    static vector<string> m_servo_names;                //!< a vector of the names of each available servo
    Motors* m_motors;                                   //!< a pointer to the underlying Motors class which communicates with the hardware
    vector<NUData::id_t*> m_joint_ids;                  //!< a vector containing pointers to all of the joint id_t. This is used to loop through all of the joints quickly
    vector<float> m_previous_positions;
    vector<float> m_previous_velocities;
};

#endif

