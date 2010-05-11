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
NUSave::NUSave(NUWalk* walk)
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "NUSave::NUSave()" << endl;
#endif
    m_walk = walk;
}

/*! @brief Destructor for FallProtection module
 */
NUSave::~NUSave()
{
    kill();
}

/*! @brief Kills the save module
 */
void NUSave::kill()
{
}

/*! @brief Returns true is a saveor block is currently being executed */
bool NUSave::isActive()
{
    return false;
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

