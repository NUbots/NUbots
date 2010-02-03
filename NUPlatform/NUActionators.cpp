/*! @file NUActionators.cpp
    @brief Partial implementation of base actuator class

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

#include "NUActionators.h"
#include "NUSystem.h"
#include "debug.h"

NUActionators::NUActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::NUActionators" << endl;
#endif
    m_data = new NUActionatorsData();
}

NUActionators::~NUActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::~NUActionators" << endl;
#endif
    if (m_data != NULL)
        delete m_data;
}

/*! @brief Processes the NUActionatorsData and sends the actions to the hardware
 
    @param data a pointer to the NUActionatorsData to be sent to the hardware
 */
void NUActionators::process(NUActionatorsData*& data)
{
    if (data == NULL)       // if the passed in pointer is NULL, then we need to set it to be equal to the internal NUActionatorsData
        data = m_data;
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::process" << endl;
#endif
    m_current_time = nusystem->getTime();
    copyToHardwareCommunications();
}

/* @brief A function to copy NUActionatorsData to the hardware communications system.
          This function must be implemented for each platform.
 */
void NUActionators::copyToHardwareCommunications()
{
}
