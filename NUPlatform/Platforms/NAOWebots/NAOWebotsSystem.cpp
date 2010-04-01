/*! @file NAOWebotsSystem.cpp
    @brief Implementation of NAO in Webots system class

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

#include "NAOWebotsSystem.h"
#include "debug.h"
#include "debugverbositynusystem.h"

NAOWebotsSystem::NAOWebotsSystem(NAOWebotsPlatform* platform)
{
    debug << "NAOWebotsSystem::NAOWebotsSystem()" << endl;
    m_platform = platform;
    m_simulator_start_timestamp = getTimeOffset();
}

NAOWebotsSystem::~NAOWebotsSystem()
{
}

/*! @brief  Returns a timestamp in milliseconds since the epoch (ie. The UNIX timestamp)
            adjusted so that the clock pauses when the simulator does.
 */
long double NAOWebotsSystem::getPosixTimeStamp()
{
    return m_simulator_start_timestamp + m_platform->getTime()*1000;
}

/*! @brief Returns the time in milliseconds since the start of the program adjusted so
           that when the simulator is paused or sped up the clock is also paused or sped up.
 */
double NAOWebotsSystem::getTime()
{
    return m_platform->getTime()*1000;
}

/*! @brief  Returns the time in milliseconds since the start of the program adjusted so
            that when the simulator is paused or sped up the clock is also paused or sped up.
 
    This function compromises a little accuracy for a faster system call if the platform supports it
 */
double NAOWebotsSystem::getTimeFast()
{
    return getTime();
}




