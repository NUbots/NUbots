/*! @file NUActionators.cpp
    @brief Partial implementation of base actuator class

    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSoundThread.h"
#include "NUPlatform/NUPlatform.h"

#include "debug.h"
#include "debugverbositynuactionators.h"

NUActionators::NUActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::NUActionators" << endl;
#endif
    m_data = new NUActionatorsData();
    m_sound_thread = new NUSoundThread();
}

NUActionators::~NUActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::~NUActionators" << endl;
#endif
    delete m_sound_thread;
    m_sound_thread = 0;
}

/*! @brief Processes the NUActionatorsData and sends the actions to the hardware
 
    @param data a pointer to the NUActionatorsData to be sent to the hardware
 */
void NUActionators::process(NUActionatorsData* data)
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NUActionators::process" << endl;
#endif
    m_data->preProcess(Platform->getTime());
    copyToHardwareCommunications();
    m_data->postProcess();
}

/*! @brief Returns a pointer to the NUActionatorsData object used to store actions for the hardware */
NUActionatorsData* NUActionators::getNUActionatorsData()
{
    return m_data;
}

/*! @brief Copies the sound actions to the sound player
 */
void NUActionators::copyToSound()
{
    /*bool l_isvalid;
    double l_time;
    vector<string> l_strings;
    if (m_data->getNextSounds(l_isvalid, l_time, l_strings))
    {
        if (l_time < m_current_time)
        {
            for (unsigned int i=0; i<l_strings.size(); i++)
                m_sound_thread->pushBack(l_strings[i]);
        }
    }*/
}
