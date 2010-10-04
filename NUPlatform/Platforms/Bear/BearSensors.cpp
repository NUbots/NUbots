/*! @file BearSensors.cpp
    @brief Implementation of Bear sensor class

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

#include "BearSensors.h"
#include "Serial/Motors.h"

#include "debug.h"
#include "debugverbositynusensors.h"

// init m_servo_names:
static string temp_servo_names[] = {string("HeadPitch"), string("HeadYaw"), \
                                    string("NeckPitch"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowPitch"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowPitch"), \
                                    string("TorsoRoll"), string("TorsoYaw"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
vector<string> BearSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

/*! @brief Constructs a nubot sensor class with Bear backend
 */
BearSensors::BearSensors(Motors* motors)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "BearSensors::BearSensors()" << endl;
    #endif
    m_motors = motors;
    m_data->setAvailableJoints(m_servo_names);
}

/*! @brief Destructor for BearSensors
 */
BearSensors::~BearSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "BearSensors::~BearSensors()" << endl;
    #endif
}

void BearSensors::copyFromHardwareCommunications()
{
    vector<float> positions (JointPositions, JointPositions + sizeof(JointPositions)/sizeof(float));
    vector<float> torques (JointLoads, JointLoads + sizeof(JointLoads)/sizeof(float));
    m_data->setJointPositions(m_current_time, positions);
    m_data->setJointTorques(m_current_time, torques);
}


