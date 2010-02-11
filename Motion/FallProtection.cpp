/*! @file FallProtection.cpp
    @brief Implementation of FallProtection class

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

#include "FallProtection.h"
#include "debug.h"

/*! @brief Constructor for FallProtection module
 */
FallProtection::FallProtection()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "FallProtection::FallProtection()" << endl;
#endif

}

/*! @brief Destructor for FallProtection module
 */
FallProtection::~FallProtection()
{
}

/*! @brief Produce actions from the data to protect the robot from a fall
 
    @param data a pointer to the most recent sensor data storage class
    @param actions a pointer to the actionators data storage class. This variable will be filled
                   with the desired actions to move into the protection pose.
 */
void FallProtection::process(NUSensorsData* data, NUActionatorsData* actions)
{
    if (actions == NULL)
        return;
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "FallProtection::process()" << endl;
#endif
}


