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

#include "motionconfig.h"
#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Getup module
 */
Getup::Getup(NUWalk* walk)
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Getup::Getup()" << endl;
    #endif
    m_walk = walk;
    m_data = NULL;
    m_actions = NULL;
    
    #ifdef USE_GETUP
        m_enabled = true;
    #else
        m_enabled = false;
    #endif
    m_completion_time = 0;
    m_head_completion_time = 0;
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

/*! @brief Returns true if the getup is executing, false otherwise */
bool Getup::isActive()
{
    if (not m_enabled or m_data == NULL)
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
    
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Getup::process()" << endl;
    #endif
    if (not isActive())
        playGetup();
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
    }
}


