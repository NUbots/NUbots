/*! @file NUMotion.cpp
    @brief Implementation of motion class

    @author Jason Kulk
 
 So what can Motion do?
    - play scripts
    - do kicks, blocks and saves
    - walk
    - do something separate with the head
 
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

#include <iostream>
using namespace std;

#include "NUPlatform/NUPlatform.h"
#include "NUMotion.h"
#include "Tools/debug.h"

/*! @brief Constructor for motion module
 */
NUMotion::NUMotion()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::NUMotion" << endl;
#endif
}

/*! @brief Destructor for motion module
 */
NUMotion::~NUMotion()
{
}

/*! @brief Process new sensor data, and produce actionator commands
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions by NUMotion provided the NUActionatorsData instance
                   has been initialised by NUActionators.
 */
void NUMotion::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
    
}

/*! @brief Process jobs
 */
void NUMotion::process(JobList jobs)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUMotion::process():" << endl;
#endif
    
    // I need to easily iterate over the job list
}

