/*! @file DarwinSensors.h
    @brief Declaration of Darwin sensors class
 
    @class DarwinSensors
    @brief A Darwin sensors
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef DARWINSENSORS_H
#define DARWINSENSORS_H

#include <vector>
#include "NUPlatform/NUSensors.h"
#include "Infrastructure/NUData.h"

class DarwinJointMapping;
class DarwinPlatform;
namespace Robot
{
    class CM730;
}

class DarwinSensors : public NUSensors
{
public:
    DarwinSensors(DarwinPlatform*, Robot::CM730*);
    ~DarwinSensors();
    
    void copyFromHardwareCommunications();
    void copyFromJoints();
    void copyFromAccelerometerAndGyro();
    void copyFromFeet();
    void copyFromButtons();
    void copyFromBattery();
    
protected:
    vector<NUData::id_t*> m_joint_ids;    	//!< a vector containing pointers to all of the joint id_t. This is used to loop through all of the joints quickly
    vector<float> m_previous_positions;
    vector<float> m_previous_velocities;
    DarwinPlatform* platform;
    Robot::CM730* cm730;
    DarwinJointMapping* m_joint_mapping;

    vector<vector<int> > error_fields;      //! A vector of motor id/error field pairs
    bool motor_error;                       //! A flag to indicate a motor indicated an error
    std::string error2Description(unsigned int errorValue);

private:
    static const unsigned int NUM_MOTORS=20;
};

#endif

