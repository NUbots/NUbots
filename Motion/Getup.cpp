/*! @file Getup.cpp
    @brief Implementation of Getup class

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

#include "Getup.h"
#include "NUWalk.h"
#include "Tools/MotionScript.h"

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "motionconfig.h"
#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Getup module
 */
Getup::Getup(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("Getup", data, actions)
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Getup::Getup()" << endl;
    #endif
    m_walk = walk;
    
    #ifdef USE_GETUP
        m_enabled = true;
    #else
        m_enabled = false;
    #endif
    m_completion_time = 0;
    m_head_completion_time = 0;
    m_arm_completion_time = 0;
    m_on_back = new MotionScript("StandUpBack");
    m_on_front = new MotionScript("StandUpFront");
    m_on_left = new MotionScript("OnLeftRoll");
    m_on_right = new MotionScript("OnRightRoll");
}

/*! @brief Destructor for FallProtection module
 */
Getup::~Getup()
{
    delete m_on_back;
    delete m_on_front;
    delete m_on_left;
    delete m_on_right;
}

/*! @brief Enable the getup */
void Getup::enable()
{
    #ifdef USE_GETUP
        m_enabled = true;
    #endif
}

/*! @brief Disable the getup */
void Getup::disable()
{
    m_enabled = false;
}

/*! @brief Returns true if the getup is enabled, false otherwise */
bool Getup::enabled()
{
    #ifdef USE_GETUP
        return m_enabled;
    #else
        return false;
    #endif
}

void Getup::stop()
{
    stopHead();
    stopArms();
    stopLegs();
}

void Getup::stopHead()
{   // the getup module can not be 'stopped' you need to let it complete its task
    return;
}

void Getup::stopArms()
{   // the getup module can not be 'stopped' you need to let it complete its task
    return;
}

void Getup::stopLegs()
{   // the getup module can not be 'stopped' you need to let it complete its task
    return;
}

void Getup::kill()
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

/*! @brief Returns true if the getup is executing, false otherwise */
bool Getup::isActive()
{
    if (not m_enabled or m_data == NULL or m_actions == NULL)
        return false;
    else if (m_data->CurrentTime <= m_completion_time)
        return true;
    else
        return false;
}

/*! @brief Returns true if the getup is currently using the head, false otherwise */
bool Getup::isUsingHead()
{
    if (not isActive())
        return false;
    else if (m_data->CurrentTime <= m_head_completion_time)
        return true;
    else
        return false;
}

/*! @brief Returns true if the getup is currently using the arms, false otherwise */
bool Getup::isUsingArms()
{
    if (not isActive())
        return false;
    else if (m_data->CurrentTime <= m_arm_completion_time)
        return true;
    else
        return false;
}

/*! @brief Returns true if the getup is currently using the legs, false otherwise */
bool Getup::isUsingLegs()
{
    return isActive();
}

/*! @brief Produce actions from the data to move the robot into a standing position
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void Getup::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL || actions == NULL)
        return;
    m_data = data;
    m_actions = actions;
    
    if (enabled() and m_data->isFallen())
    {
        #if DEBUG_NUMOTION_VERBOSITY > 1
            debug << "Getup::process()" << endl;
        #endif
        if (not isActive())
            playGetup();
    }
}

void Getup::playGetup()
{
    if (m_walk)
        m_walk->kill();
    
    vector<float> fallen;
    if (m_data->getFallen(fallen))
    {
        MotionScript* getup = m_on_front;
        if (fallen[1])
            getup = m_on_left;
        else if (fallen[2])
            getup = m_on_right;
        else if (fallen[3])
            getup = m_on_front;
        else if (fallen[4])
            getup = m_on_back;
        getup->play(m_data, m_actions);
        m_completion_time = getup->timeFinished();
        m_head_completion_time = getup->timeFinishedWithHead();
        m_arm_completion_time = max(getup->timeFinishedWithLArm(), getup->timeFinishedWithRArm());
    }
}


