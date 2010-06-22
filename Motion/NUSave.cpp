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
#include "Behaviour/Jobs/MotionJobs/BlockJob.h"
#include "NUWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for NUSave module
 */
NUSave::NUSave(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("NUSave", data, actions), m_BLOCK_TRIGGER(1.8f), m_BLOCK_WIDTH(55.0f)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUSave::NUSave()" << endl;
#endif
    m_walk = walk;
    m_data = data;
    m_actions = actions;
    
    m_block_left = MotionScript("BlockLeft");
    m_block_right = MotionScript("BlockRight");
    m_block_centre = MotionScript("BlockCentre");
    m_dive_left = MotionScript("DiveLeft");
    m_dive_right = MotionScript("DiveRight");

    m_completion_time = 0;
    m_block_timestamp = 0;
    
    m_block_time = 0;
    m_block_position = vector<float>(2,0);
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
{   // save can't be stopped until it is completed
    return;
}

void NUSave::stopArms()
{   // save can't be stopped until it is completed
    return;
}

void NUSave::stopLegs()
{   // save can't be stopped until it is completed
    return;
}

/*! @brief Kills the save module */
void NUSave::kill()
{
    if (isActive())
    {   // if the save is currently running, the only way to kill it is to set the stiffnesses to 0
        m_completion_time = 0;
        
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

/*! @brief Returns true if the save module is ready to block */
bool NUSave::isReady()
{
    return isBlockAble();
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
    m_data = data;
    m_actions = actions;
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUSave::process()" << endl;
#endif
    if (not isActive())
        playSave();
}

void NUSave::playSave()
{
    if (m_walk)
        m_walk->kill();
    
    if (m_block_position[1] >= 3)
    {
        m_block_left.play(m_data, m_actions);
        m_completion_time = m_block_left.timeFinished();
    }
    else if (m_block_position[1] <= -3)
    {
        m_block_right.play(m_data, m_actions);
        m_completion_time = m_block_right.timeFinished();
    }
}

/*! @brief Process a block job
 */
void NUSave::process(BlockJob* job)
{
    job->getPosition(m_block_time, m_block_position);
    if (m_data)
        m_block_timestamp = m_data->CurrentTime;
}

/*! @brief Process a save job
 */
void NUSave::process(SaveJob* job)
{
    job->getPosition(m_block_time, m_block_position);  
    if (m_data)
        m_block_timestamp = m_data->CurrentTime;
}

/*! @brief Returns true if the current block is doable */
bool NUSave::isBlockAble()
{
    if (m_block_time > 0.3 and m_block_time < m_BLOCK_TRIGGER and fabs(m_block_position[1]) < m_BLOCK_WIDTH and m_data->CurrentTime - m_block_timestamp < 5000)
        return true;
    else
        return false;
}

