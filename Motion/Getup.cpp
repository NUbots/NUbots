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
#include "motionconfig.h"
#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Getup module
 */
Getup::Getup()
{
    #if DEBUG_NUMOTION_VERBOSITY > 4
        debug << "Getup::Getup()" << endl;
    #endif
    #ifdef USE_GETUP
        m_enabled = true;
    #else
        m_enabled = false;
    #endif
}

/*! @brief Destructor for FallProtection module
 */
Getup::~Getup()
{
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
    if (not m_enabled)
        return false;
    else
        return false;
}

/*! @brief Returns true if the getup is currently using the head, false otherwise */
bool Getup::isUsingHead()
{
    if (not isActive())
        return false;
    else
        return true;
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
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "Getup::process()" << endl;
#endif
}


