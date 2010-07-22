/*! @file FallProtection.cpp
    @brief Implementation of FallProtection class

    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

#include "FallProtection.h"
#include "NUWalk.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "motionconfig.h"
#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for FallProtection module
 */
FallProtection::FallProtection(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("FallProtection", data, actions)
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "FallProtection::FallProtection()" << endl;
    #endif
    m_walk = walk;
    #ifdef USE_FALL_PROTECTION
        m_enabled = true;
    #else
        m_enabled = false;
    #endif
}

/*! @brief Destructor for FallProtection module
 */
FallProtection::~FallProtection()
{
}

/*! @brief Enables the fall protection module
 */
void FallProtection::enable()
{
    #ifdef USE_FALL_PROTECTION
        m_enabled = true;
    #endif
}

/*! @brief Disables the fall protection module
 */
void FallProtection::disable()
{
    m_enabled = false;
}

/*! @brief Returns true if the fall protection is enabled, false otherwise
 */
bool FallProtection::enabled()
{
    #ifdef USE_FALL_PROTECTION
        return m_enabled;
    #else
        return false;
    #endif
}

/*! @brief Stops the fall protection */
void FallProtection::stop()
{
    stopHead();
    stopArms();
    stopLegs();
}

void FallProtection::stopHead()
{
    return;
}

void FallProtection::stopArms()
{
    return;
}

void FallProtection::stopLegs()
{
    return;
}

/*! @brief Kills the fall protection */
void FallProtection::kill()
{
    return;
}

/*! @brief Returns true if the fall protection is currently active, false otherwise */
bool FallProtection::isActive()
{
    if (m_data and m_data->isFalling())
        return true;
    else
        return false;
}

/*! @brief Returns true if the fall protection is using the head */
bool FallProtection::isUsingHead()
{
    return isActive();
}

/*! @brief Returns true if the fall protection is using the arms */
bool FallProtection::isUsingArms()
{
    return isActive();
}

/*! @brief Returns true if the fall protection is using the legs */
bool FallProtection::isUsingLegs()
{
    return isActive();
}

/*! @brief Produce actions from the data to protect the robot from a fall
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void FallProtection::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL or actions == NULL or (not m_enabled))
        return;
    m_data = data;
    m_actions = actions;
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "FallProtection::process()" << endl;
    #endif
    if (m_data->isFalling())
    {
        vector<float> velocity_larm(m_actions->getNumberOfJoints(NUActionatorsData::LeftArmJoints), 0);
        vector<float> velocity_rarm(m_actions->getNumberOfJoints(NUActionatorsData::RightArmJoints), 0);
        vector<float> velocity_lleg(m_actions->getNumberOfJoints(NUActionatorsData::LeftLegJoints), 0);
        vector<float> velocity_rleg(m_actions->getNumberOfJoints(NUActionatorsData::RightLegJoints), 0);
        
        vector<float> sensor_larm, sensor_rarm;
        vector<float> sensor_lleg, sensor_rleg;
        m_data->getJointPositions(NUSensorsData::LeftArmJoints, sensor_larm);
        m_data->getJointPositions(NUSensorsData::RightArmJoints, sensor_rarm);
        m_data->getJointPositions(NUSensorsData::LeftLegJoints, sensor_lleg);
        m_data->getJointPositions(NUSensorsData::RightLegJoints, sensor_rleg);
        
        m_actions->addJointPositions(NUActionatorsData::LeftLegJoints, 0, sensor_lleg, velocity_lleg, 0);
        m_actions->addJointPositions(NUActionatorsData::RightLegJoints, 0, sensor_rleg, velocity_rleg, 0);
        m_actions->addJointPositions(NUActionatorsData::LeftArmJoints, 0, sensor_larm, velocity_larm, 0);
        m_actions->addJointPositions(NUActionatorsData::RightArmJoints, 0, sensor_rarm, velocity_rarm, 0);
    }
}


