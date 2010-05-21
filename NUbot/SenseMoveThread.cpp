/*! @file SenseMoveThread.h
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

#include "SenseMoveThread.h"
#include "NUbot.h"

#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"
#include "NUPlatform/NUIO.h"

#ifdef USE_MOTION
    #include "Motion/NUMotion.h"
#endif

#include "debug.h"
#include "debugverbositynubot.h"
#include "debugverbositythreading.h"

#include <string>
#include <errno.h>
using namespace std;

#if DEBUG_NUBOT_VERBOSITY > DEBUG_THREADING_VERBOSITY
    #define DEBUG_VERBOSITY DEBUG_NUBOT_VERBOSITY
#else
    #define DEBUG_VERBOSITY DEBUG_THREADING_VERBOSITY
#endif

/*! @brief Constructs the sense->move thread
 */

SenseMoveThread::SenseMoveThread(NUbot* nubot) : ConditionalThread(string("SenseMoveThread"), THREAD_SENSEMOVE_PRIORITY)
{
    #if DEBUG_VERBOSITY > 0
        debug << "SenseMoveThread::SenseMoveThread(" << nubot << ") with priority " << static_cast<int>(m_priority) << endl;
    #endif
    m_nubot = nubot;
}

SenseMoveThread::~SenseMoveThread()
{
    #if DEBUG_VERBOSITY > 0
        debug << "SenseMoveThread::~SenseMoveThread()" << endl;
    #endif
}

/*! @brief The sense->move main loop
 
    When signalled the thread will quickly grab the new sensor data, compute a response, 
    and then send the commands to the actionators.
 
    Note that you can not safely use the job interface in this thread, if you need to add
    jobs provide a process function for this thread, and *another* process for the behaviour 
    thread which creates the jobs.
 
 */
void SenseMoveThread::run()
{
    #if DEBUG_VERBOSITY > 0
        debug << "SenseMoveThread::run()" << endl;
    #endif
    
    #ifdef THREAD_SENSEMOVE_MONITOR_TIME
        double entrytime;
        double realstarttime, processstarttime, threadstarttime; 
        double realendtime, processendtime, threadendtime;
    #endif
    
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        try 
        {
            #ifdef THREAD_SENSEMOVE_MONITOR_TIME
                entrytime = NUSystem::getRealTime();
            #endif
            waitForCondition();

            #ifdef THREAD_SENSEMOVE_MONITOR_TIME
                realstarttime = NUSystem::getRealTime();
                #ifndef TARGET_IS_NAOWEBOTS         // there is no point monitoring wait times in webots
                if (realstarttime - entrytime > 15)
                    debug << "SenseMoveThread. Warning. Waittime " << realstarttime - entrytime << "ms."<< endl;
                #endif
                processstarttime = NUSystem::getProcessTime();
                threadstarttime = NUSystem::getThreadTime();
            #endif
                
            // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
            m_nubot->SensorData = m_nubot->m_platform->sensors->update();
            #ifdef USE_MOTION
                m_nubot->m_motion->process(m_nubot->SensorData, m_nubot->Actions);
            #endif
            m_nubot->m_platform->actionators->process(m_nubot->Actions);
            // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

            #ifdef THREAD_SENSEMOVE_MONITOR_TIME
                realendtime = NUSystem::getRealTime();
                processendtime = NUSystem::getProcessTime();
                threadendtime = NUSystem::getThreadTime();
                if (threadendtime - threadstarttime > 0)
                    debug << "SenseMoveThread. Warning. Thread took a long time to complete. Time spent in this thread: " << (threadendtime - threadstarttime) << "ms, in this process: " << (processendtime - processstarttime) << "ms, in realtime: " << realendtime - realstarttime << "ms." << endl;
            #endif
        }
        catch (std::exception& e) 
        {
            m_nubot->unhandledExceptionHandler(e);
        }
        onLoopCompleted();
    }
    errorlog << "SenseMoveThread is exiting. err: " << err << " errno: " << errno << endl;
}
