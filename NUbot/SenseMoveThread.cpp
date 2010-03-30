/*! @file SenseMoveThread.h
 @brief Declaration of the sense->move thread class.
 
 @class SenseMoveThread
 @brief The sense->move thread that links sensor data to motion
 
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

#include "NUbot.h"

#include "debug.h"
#include "debugverbositynubot.h"
#include "debugverbositythreading.h"

#if DEBUG_VERBOSITY_NUBOT > DEBUG_VERBOSITY_THREADING
    #define DEBUG_VERBOSITY DEBUG_VERBOSITY_NUBOT
#else
    #define DEBUG_VERBOSITY DEBUG_VERBOSITY_THREADING

/*! @brief Constructs the sense->move thread
 */

SenseMoveThread::SenseMoveThread(NUbot* nubot) : ConditionalThread(string("SenseMoveThread"), THREAD_SENSEMOVE_PRIORITY)
{
    #if DEBUG_VERBOSITY > 0
        debug << "SenseMoveThread::SenseMoveThread(" << nubot << ") with priority " << THREAD_SENSEMOVE_PRIORITY << endl;
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
        #ifdef THREAD_MOTION_MONITOR_TIME
            entrytime = NUSystem::getRealTime();
        #endif
        waitForCondition();

        #ifdef THREAD_MOTION_MONITOR_TIME
            realstarttime = NUSystem::getRealTime();
            #ifndef TARGET_IS_NAOWEBOTS         // there is not point monitoring wait times in webots
            if (realstarttime - entrytime > 15)
                debug << "NUbot::SenseMoveThread. Waittime " << realstarttime - entrytime << "ms."<< endl;
            #endif
            processstarttime = NUSystem::getProcessTime();
            threadstarttime = NUSystem::getThreadTime();
        #endif
            
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        m_data = m_nubot->platform->sensors->update();
        #ifdef USE_MOTION
            nubot->motion->process(m_data, m_actions);
            #ifdef USE_WALKOPTIMISER
                nubot->walkoptimiser->process(m_data, m_actions);
            #endif
        #endif
        nubot->platform->actionators->process(m_actions);
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------

    #ifdef THREAD_MOTION_MONITOR_TIME
        realendtime = NUSystem::getRealTime();
        processendtime = NUSystem::getProcessTime();
        threadendtime = NUSystem::getThreadTime();
        if (threadendtime - threadstarttime > 5)
            debug << "NUbot::runThreadMotion. Thread took a long time to complete. Time spent in this thread: " << (threadendtime - threadstarttime) << "ms, in this process: " << (processendtime - processstarttime) << "ms, in realtime: " << realendtime - realstarttime << "ms." << endl;
    #endif
        nubot->signalMotionCompletion();
    } 
    errorlog << "runMotionThread is exiting. err: " << err << " errno: " << errno << endl;
    pthread_exit(NULL);
}
