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
#include "../Robotis/Motors.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "debug.h"
#include "debugverbositynusensors.h"

#include <limits>


// init m_servo_names:
static std::string temp_servo_names[] = {std::string("HeadPitch"), std::string("HeadYaw"), \
                                    std::string("NeckPitch"), \
                                    std::string("LShoulderRoll"), std::string("LShoulderPitch"), std::string("LElbowPitch"), \
                                    std::string("RShoulderRoll"), std::string("RShoulderPitch"), std::string("RElbowPitch"), \
                                    std::string("TorsoRoll"), std::string("TorsoYaw"), \
                                    std::string("LHipRoll"),  std::string("LHipPitch"), std::string("LKneePitch"), std::string("LAnkleRoll"), std::string("LAnklePitch"), \
                                    std::string("RHipRoll"),  std::string("RHipPitch"), std::string("RKneePitch"), std::string("RAnkleRoll"), std::string("RAnklePitch")};
std::vector<std::string> BearSensors::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

/*! @brief Constructs a nubot sensor class with Bear backend
 */
BearSensors::BearSensors(Motors* motors)
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "BearSensors::BearSensors()" << std::endl;
    #endif
    m_motors = motors;
    m_data->addSensors(m_servo_names);
    m_joint_ids = m_data->mapIdToIds(NUSensorsData::All);
    m_previous_positions = std::vector<float>(m_joint_ids.size(), 0);
    m_previous_velocities = std::vector<float>(m_joint_ids.size(), 0);
}

/*! @brief Destructor for BearSensors
 */
BearSensors::~BearSensors()
{
    #if DEBUG_NUSENSORS_VERBOSITY > 0
        debug << "BearSensors::~BearSensors()" << std::endl;
    #endif
}

/*! @brief Copys the sensors data from the hardware communication module to the NUSensorsData container
 */
void BearSensors::copyFromHardwareCommunications()
{
    copyFromJoints();
}

/*! @brief Copys the joint sensor data from the Motors class to the NUSensorsData container.
 
    The Motors class is very old and uses two globals; JointPositions and JointLoads to store sensor data.
    Consequently, we need only copy from these old arrays into the new fancy NUSensorsData structure.
 */
void BearSensors::copyFromJoints()
{
    static const float NaN = std::numeric_limits<float>::quiet_NaN();
    
    std::vector<float> targets;
    m_motors->getTargets(targets);
    
    std::vector<float> joint(NUSensorsData::NumJointSensorIndices, NaN);
    float delta_t = (m_current_time - m_previous_time)/1000.0;
    for (size_t i=0; i<m_joint_ids.size(); i++)
    {
        joint[NUSensorsData::PositionId] = Motors::MotorSigns[i]*(JointPositions[i] - Motors::DefaultPositions[i])/195.379;         // I know, its a horrible way of converting from motor units to radians
        joint[NUSensorsData::VelocityId] = (joint[NUSensorsData::PositionId] - m_previous_positions[i])/delta_t;    
        joint[NUSensorsData::AccelerationId] = (joint[NUSensorsData::VelocityId] - m_previous_velocities[i])/delta_t;
        joint[NUSensorsData::TargetId] = Motors::MotorSigns[i]*(targets[i] - Motors::DefaultPositions[i])/195.379;;
        joint[NUSensorsData::StiffnessId] = NaN;
        joint[NUSensorsData::TorqueId] = Motors::MotorSigns[i]*JointLoads[i]*1.6432e-3;             // This torque conversion factor was measured for a DX-117, I don't know how well it applies to other motors
        m_data->set(*m_joint_ids[i], m_current_time, joint);
        
        m_previous_positions[i] = joint[NUSensorsData::PositionId];
        m_previous_velocities[i] = joint[NUSensorsData::VelocityId];
    }
}

