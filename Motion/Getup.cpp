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

/*! @brief Constructor for Getup module
 */
Getup::Getup()
{
#if DEBUG_NUMOTION_VERBOSITY > 4
    debug << "Getup::Getup()" << endl;
#endif

}

/*! @brief Destructor for FallProtection module
 */
Getup::~Getup()
{
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

/*! @brief Returns true when the head is ready to be used for vision/localisation purposes.
 
    The idea is to start localising before we have finished getting up, so that we are ready to play
    once the getup is completed.
 
    @return true if the head is ready to be used for vision/localisation purposes
*/
bool Getup::headReady()
{
    return true;
}


