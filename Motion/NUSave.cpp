/*! @file NUSave.cpp
    @brief Implementation of NUSave class

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

#include "NUSave.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Behaviour/Jobs/MotionJobs/SaveJob.h"
#include "NUWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for NUSave module
 */
NUSave::NUSave(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("NUSave", data, actions)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUSave::NUSave()" << endl;
#endif
    m_walk = walk;
    
    m_block_left = MotionScript("BlockLeft");
    m_block_right = MotionScript("BlockRight");
    m_block_centre = MotionScript("BlockCentre");
    m_dive_left = MotionScript("DiveLeft");
    m_dive_right = MotionScript("DiveRight");
    
    m_head_completion_time = 0;
    m_arm_completion_time = 0;
    m_completion_time = 0;
}

/*! @brief Destructor for FallProtection module
 */
NUSave::~NUSave()
{
    kill();
}

/*! @brief Stops the save module */
void NUSave::stop()
{
    stopHead();
    stopArms();
    stopLegs();
}

void NUSave::stopHead()
{
}

void NUSave::stopArms()
{
}

void NUSave::stopLegs()
{
}

/*! @brief Kills the save module */
void NUSave::kill()
{
    if (isActive())
    {   // if the getup is currently running, the only way to kill it is to set the stiffnesses to 0
        m_completion_time = 0;
        m_head_completion_time = 0;
        m_arm_completion_time = 0;
        
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

/*! @brief Returns true if the save module is active */
bool NUSave::isActive()
{
    if (m_data == NULL or m_actions == NULL)
        return false;
    else if (m_data->CurrentTime <= m_completion_time)
        return true;
    else
        return false;
}

/*! @brief Returns true if the save module is using the head */
bool NUSave::isUsingHead()
{
    if (not isActive())
        return false;
    else if (m_data->CurrentTime <= m_head_completion_time)
        return true;
    else
        return false;
}

/*! @brief Returns true if the save module is using the arms */
bool NUSave::isUsingArms()
{
    return isActive();
}

/*! @brief Returns true if the save module is using the legs */
bool NUSave::isUsingLegs()
{
    return isActive();
}

/*! @brief Produce actions from the data to move the robot into a standing position
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void NUSave::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL || actions == NULL)
        return;
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUSave::process()" << endl;
#endif
}


/*! @brief Process a block job
 */
void NUSave::process(BlockJob* job)
{
}

/*! @brief Process a save job
 */
void NUSave::process(SaveJob* job)
{
}

