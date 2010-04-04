/*! @file NetworkThread.h
    @brief Implementation of the network thread class.

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

#include "NetworkThread.h"
#include "NUbot.h"

#include "debug.h"
#include "debugverbositynubot.h"
#include "debugverbositythreading.h"

#include <errno.h>

#if DEBUG_NUBOT_VERBOSITY > DEBUG_THREADING_VERBOSITY
    #define DEBUG_VERBOSITY DEBUG_NUBOT_VERBOSITY
#else
    #define DEBUG_VERBOSITY DEBUG_THREADING_VERBOSITY
#endif

/*! @brief Constructs the network thread
 */

NetworkThread::NetworkThread(NUbot* nubot) : ConditionalThread(string("NetworkThread"), THREAD_NETWORK_PRIORITY)
{
    #if DEBUG_VERBOSITY > 0
        debug << "NetworkThread::NetworkThread(" << nubot << ") with priority " << static_cast<int>(m_priority) << endl;
    #endif
    m_nubot = nubot;
}

NetworkThread::~NetworkThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "NetworkThread::~NetworkThread()" << endl;
    #endif
}

/*! @brief The network main loop
 
 */
void NetworkThread::run()
{
    #if DEBUG_VERBOSITY > 0
        debug << "NetworkThread::run()" << endl;
    #endif
    
    #ifdef THREAD_NETWORK_MONITOR_TIME
        double realstarttime, processstarttime, threadstarttime; 
        double realendtime, processendtime, threadendtime;
    #endif
    
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        waitForCondition();

        #ifdef THREAD_NETWORK_MONITOR_TIME
            realstarttime = NUSystem::getRealTime();
            processstarttime = NUSystem::getProcessTime();
            threadstarttime = NUSystem::getThreadTime();
        #endif
            
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
         
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

    #ifdef THREAD_NETWORK_MONITOR_TIME
        realendtime = NUSystem::getRealTime();
        processendtime = NUSystem::getProcessTime();
        threadendtime = NUSystem::getThreadTime();
        if (threadendtime - threadstarttime > 7)
            debug << "NetworkThread. Warning. Thread took a long time to complete. Time spent in this thread: " << (threadendtime - threadstarttime) << "ms, in this process: " << (processendtime - processstarttime) << "ms, in realtime: " << realendtime - realstarttime << "ms." << endl;
    #endif
        onLoopCompleted();
    } 
    errorlog << "SenseMoveThread is exiting. err: " << err << " errno: " << errno << endl;
}
