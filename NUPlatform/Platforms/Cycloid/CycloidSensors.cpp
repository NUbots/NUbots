/*! @file CycloidSensors.cpp
    @brief Implementation of Cycloid sensor class

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

#include "CycloidSensors.h"
#include "../Robotis/Motors.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include <limits>
using namespace std;

// init m_servo_names:
static string temp_servo_names[] = {string("HeadYaw"), \
                                    string("LShoulderRoll"), string("LShoulderPitch"), string("LElbowRoll"), string("LElbowYaw"), \
                                    string("RShoulderRoll"), string("RShoulderPitch"), string("RElbowRoll"), string("RElbowYaw"), \
                                    string("TorsoRoll"), string("TorsoPitch"), \
                                    string("LHipRoll"),  string("LHipPitch"), string("LHipYaw"), string("LKneePitch"), string("LAnkleRoll"), string("LAnklePitch"), \
                                    string("RHipRoll"),  string("RHipPitch"), string("RHipYaw"), string("RKneePitch"), string("RAnkleRoll"), string("RAnklePitch")};
vector<string> CycloidSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

/*! @brief Constructs a nubot sensor class with Bear backend
 */
CycloidSensors::CycloidSensors(Motors* motors)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "CycloidSensors::CycloidSensors()" << endl;
    #endif
    m_motors = motors;
    m_data->addSensors(m_servo_names);
    m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
    m_previous_positions = vector<float>(m_joint_ids.size(), 0);
    m_previous_velocities = vector<float>(m_joint_ids.size(), 0);
}

/*! @brief Destructor for CycloidSensors
 */
CycloidSensors::~CycloidSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "CycloidSensors::~CycloidSensors()" << endl;
    #endif
}

/*! @brief Copys the sensors data from the hardware communication module to the NUSensorsData container
 */
void CycloidSensors::copyFromHardwareCommunications()
{
    copyFromJoints();
}

/*! @brief Copys the joint sensor data from the Motors class to the NUSensorsData container.
 
    The Motors class is very old and uses two globals; JointPositions and JointLoads to store sensor data.
    Consequently, we need only copy from these old arrays into the new fancy NUSensorsData structure.
 */
void CycloidSensors::copyFromJoints()
{
    static const float NaN = numeric_limits<float>::quiet_NaN();
    
    vector<float> targets, stiffnesses;
    m_motors->getTargets(targets);
    m_motors->getStiffnesses(stiffnesses);
    
    vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = 1000*(m_current_time - m_previous_time);
    for (size_t i=0; i<m_joint_ids.size(); i++)
    {
        joint[NUSensorsData::PositionId] = Motors::MotorSigns[i]*(JointPositions[i] - Motors::DefaultPositions[i])/195.379;         // I know, its a horrible way of converting from motor units to radians
        joint[NUSensorsData::VelocityId] = (joint[NUSensorsData::PositionId] - m_previous_positions[i])/delta_t;    
        joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
        joint[NUSensorsData::TargetId] = Motors::MotorSigns[i]*(targets[i] - Motors::DefaultPositions[i])/195.379;;
        joint[NUSensorsData::StiffnessId] = stiffnesses[i];
        joint[NUSensorsData::TorqueId] = Motors::MotorSigns[i]*JointLoads[i]*1.6432e-3;             // This torque conversion factor was measured for a DX-117, I don't know how well it applies to other motors
        m_data->set(*m_joint_ids[i], m_current_time, joint);
        
        m_previous_positions[i] = joint[NUSensorsData::PositionId];
        m_previous_velocities[i] = joint[NUSensorsData::VelocityId];
    }
}

