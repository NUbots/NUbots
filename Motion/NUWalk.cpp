/*! @file NUWalk.cpp
    @brief Implementation of nuwalk class

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

#include "Walks/walkconfig.h"
#include "NUWalk.h"
#ifdef USE_JWALK
    #include "Walks/JWalk/JWalk.h"
#endif
#ifdef USE_JUPPWALK
    #include "Walks/JuppWalk/JuppWalk.h"
#endif
#ifdef USE_NBWALK
    #include "Walks/NBWalk/NBWalk.h"
#endif
#ifdef USE_VSCWALK
#include "Walks/VSCWalk/VSCWalk.h"
#endif

#include "NUPlatform/NUSystem.h"
#include "Tools/debug.h"

NUWalk* NUWalk::getWalkEngine()
{
#ifdef USE_JWALK
    return new JWalk();
#else
    #ifdef USE_JUPPWALK
        return new JuppWalk();
    #else
        #ifdef USE_NBWALK
            return new NBWalk();
        #else
            #ifdef USE_VSCWALK
                return new VSCWalk();
            #endif
        #endif
    #endif
#endif
    return NULL;
}

/*! @brief Destructor for motion module
 */
NUWalk::~NUWalk()
{
    // nothing needs to be deleted at this level
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. 
*/
void NUWalk::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL || data == NULL)
        return;
    m_data = data;
    m_actions = actions;
    doWalk();
}

/*! @brief Walk with the given speed vector
 
    Use this function to precisely control the locomotion direction of the robot. 
    To instruct the robot to move as fast as possible, just put in a very large value, it will be clipped internally.
    To instruct the robot to stop specify all speeds to be zero. Alternatively, the robot will stop if no
    walk command has been issued in the last second.
 
    @param speed the desired walk velocity [x (cm/s), y (cm/s), rotation (rad/s)]
 */
void NUWalk::walkSpeed(const vector<float>& speed)
{
    if (speed.size() == 3)
    {
        m_speed_x = speed[0];
        m_speed_y = speed[1];
        m_speed_yaw = speed[2];
    }
    else if (speed.size() == 2)
    {
        m_speed_x = speed[0];
        m_speed_y = speed[1];
        m_speed_yaw = 0;
    }
    else if (speed.size() == 1)
    {
        m_speed_x = speed[0];
        m_speed_y = 0;
        m_speed_yaw = 0;
    }
    else if (speed.size() == 0)
    {
        m_speed_x = 0;
        m_speed_y = 0;
        m_speed_yaw = 0;
    }
    m_speed_timestamp = nusystem->getTime();
}

/*! @brief Walk to the given point by the given time
 
    Use this function to make use of the internal path planning algorithms. These algorithms are purely time-based and
    consequently they will try to get to the given point as fast as possible.
    To instruct the robot to move as fast as possible, just put in a small time value, the speed will be clipped internally.
    To instruct the robot to stop specify all coordinates to be zero. Alternatively, the robot will stop if no
    walk command has been issued in the last second.
 
    @warning Don't think that you can use this function to queue up points to walk to, or
             even that you can just call this function once and expect the robot to magically
             walk to the point. You need to call this function at every behaviour iteration with new
             data so that localisation feedback is used to walk to the point.
 
    @param time the desired time to reach the given point (ms)
    @param x the desired relative target [x (cm), y (cm), theta (rad)]
 */
void NUWalk::walkToPoint(double time, const vector<float>& position)
{
    m_point_time = time;
    if (position.size() == 3)
    {
        m_point_x = position[0];
        m_point_y = position[1];
        m_point_theta = position[2];
    }
    else if (position.size() == 2)
    {
        m_point_x = position[0];
        m_point_y = position[1];
        m_point_theta = 0;
    }
    m_point_timestamp = nusystem->getTime();
}

/*! @brief The primary walk function. This takes in the sensor data and the walk command to produce new nuactionators data
 */
void NUWalk::doWalk()
{
    // implementation is walk engine dependent
}


