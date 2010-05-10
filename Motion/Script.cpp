/*! @file Script.cpp
    @brief Implementation of Script class

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

#include "Script.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Behaviour/Jobs/MotionJobs/ScriptJob.h"
#include "NUWalk.h"

#include "debug.h"
#include "debugverbositynumotion.h"

/*! @brief Constructor for Script module
 */
Script::Script(NUWalk* walk)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "Script::Script()" << endl;
#endif
    m_walk = walk;
}

/*! @brief Destructor for FallProtection module
 */
Script::~Script()
{
}

/*! @brief Produce actions from the data to move the robot into a standing position
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void Script::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (data == NULL || actions == NULL)
        return;
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "Script::process()" << endl;
#endif
}

/*! @brief Processes a script job
 */
void Script::process(ScriptJob* job)
{
    
}


