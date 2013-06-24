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
#include "Motion/Kicks/MotionScript2013.h"

#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "motionconfig.h"
#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Getup module
 */
Getup::Getup(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUMotionProvider("Getup", data, actions)
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Getup::Getup()" << std::endl;
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
    getup_back_script_ = MotionScript2013::LoadFromConfigSystem("StandUpBack");
    getup_front_script_ = MotionScript2013::LoadFromConfigSystem("StandUpFront");
    getup_left_script_ = MotionScript2013::LoadFromConfigSystem("OnLeftRoll");
    getup_right_script_ = MotionScript2013::LoadFromConfigSystem("OnRightRoll");
}

/*! @brief Destructor for FallProtection module
 */
Getup::~Getup()
{
    delete getup_back_script_;
    delete getup_front_script_;
    delete getup_left_script_;
    delete getup_right_script_;
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
        
        std::vector<float> sensor_larm, sensor_rarm;
        std::vector<float> sensor_lleg, sensor_rleg;
        m_data->getPosition(NUSensorsData::LArm, sensor_larm);
        m_data->getPosition(NUSensorsData::RArm, sensor_rarm);
        m_data->getPosition(NUSensorsData::LLeg, sensor_lleg);
        m_data->getPosition(NUSensorsData::RLeg, sensor_rleg);
        
        m_actions->add(NUActionatorsData::LLeg, 0, sensor_lleg, 0);
        m_actions->add(NUActionatorsData::RLeg, 0, sensor_rleg, 0);
        m_actions->add(NUActionatorsData::LArm, 0, sensor_larm, 0);
        m_actions->add(NUActionatorsData::RArm, 0, sensor_rarm, 0);
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

/*! @brief Returns true if the getup requires the head */
bool Getup::requiresHead()
{
    if (m_data->CurrentTime <= m_head_completion_time)
        return true;
    else
        return false;
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
            debug << "Getup::process()" << std::endl;
        #endif
        if (not isActive())
            playGetup();
    }
}

void Getup::playGetup()
{
    if (m_walk)
        m_walk->kill();
    
    std::vector<float> fallen;
    if (m_data->get(NUSensorsData::Fallen, fallen))
    {
        MotionScript2013* getup = getup_front_script_;
        if (fallen[1])
            getup = getup_left_script_;
        else if (fallen[2])
            getup = getup_right_script_;
        else if (fallen[3])
            getup = getup_front_script_;
        else if (fallen[4])
            getup = getup_back_script_;
        getup->ScheduleEntireScript(m_data, m_actions);

        m_completion_time = getup->TimeFinished();
        m_head_completion_time = getup->TimeFinishedWithHead();
        m_arm_completion_time = std::max(getup->TimeFinishedWithLArm(),
                                         getup->TimeFinishedWithRArm());
    }
}


