/*! @file WatchDogThread.h
    @brief Declaration of the sense->move thread class.

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "WatchDogThread.h"
#include "NUbot.h"
#include "NUPlatform/NUSystem.h"

#include "debug.h"
#include "debugverbositynubot.h"
#include "debugverbositythreading.h"

#include <errno.h>

#if DEBUG_NUBOT_VERBOSITY > DEBUG_THREADING_VERBOSITY
    #define DEBUG_VERBOSITY DEBUG_NUBOT_VERBOSITY
#else
    #define DEBUG_VERBOSITY DEBUG_THREADING_VERBOSITY
#endif

/*! @brief Constructs the sense->move thread
 */

WatchDogThread::WatchDogThread(NUbot* nubot) : PeriodicThread(string("WatchDogThread"), 1000, 0)
{
    #if DEBUG_VERBOSITY > 0
        debug << "WatchDogThread::WatchDogThread(" << nubot << ") with priority " << static_cast<int>(m_priority) << endl;
    #endif
    m_nubot = nubot;
}

WatchDogThread::~WatchDogThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "WatchDogThread::~WatchDogThread()" << endl;
    #endif
}

void WatchDogThread::periodicFunction()
{
    cout << "WatchDog: " << nusystem->getRealTime() << endl;
    nusystem->displayBatteryState(m_nubot->SensorData, m_nubot->Actions);
}
