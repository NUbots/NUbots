/*! @file Actuators.cpp
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

#include "NUSystem.h"
#include "Tools/debug.h"

#include <iostream>
using namespace std;

#ifdef __NU_SYSTEM_CLOCK_GETTIME
    struct timespec NUSystem::m_gettime_starttime;            //!< the program's start time according to gettime()
    struct timespec NUSystem::m_gettimefast_starttime;        //!< the program's start time according to the fast verion of gettime()

    #ifndef CLOCK_REALTIME_FAST                             // not all distros will have the CLOCK_REALTIME_FAST, fudge it if they don't
        #define CLOCK_REALTIME_FAST CLOCK_REALTIME
    #endif
#else
    ptime NUSystem::m_microsec_starttime = microsec_clock::local_time();
#endif

NUSystem::NUSystem()
{
    debug << "NUSystem::NUSystem()" << endl;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    clock_gettime(CLOCK_REALTIME, &m_gettime_starttime);
    clock_gettime(CLOCK_REALTIME_FAST, &m_gettimefast_starttime); 
#endif
}

NUSystem::~NUSystem()
{
}

/*! @brief Returns the time in seconds since the epoch (ie. The UNIX timestamp)
 
    This function can be slow on some platforms. Only use this if you need the best precision time
    the platform supports. Typically, the precision will be better than 1ms.
 */
double NUSystem::getPosixTimeStamp()
{
    static double timeinseconds;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    static struct timespec timenow;
    clock_gettime(CLOCK_REALTIME_FAST, &timenow);
    timeinseconds = timenow.tv_nsec/1e9 + timenow.tv_sec;
#else
    static ptime ptimenow;
    timeinseconds = (microsec_clock::universal_time() - from_time_t(0)).total_nanoseconds()/1e9;
#endif
    return timeinseconds;
}

/*! @brief Returns the real time in milliseconds since the start of the program.
 */
double NUSystem::getTime()
{
    static double timeinseconds;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    static struct timespec timenow;
    clock_gettime(CLOCK_REALTIME, &timenow);
    timeinseconds = (timenow.tv_nsec - m_gettime_starttime.tv_nsec)/1e6 + (timenow.tv_sec - m_gettime_starttime.tv_sec)*1e3;
#else
    static ptime timenow;
    timenow = microsec_clock::local_time();
    timeinseconds = (timenow - m_microsec_starttime).total_nanoseconds()/1e6;
#endif
    return timeinseconds;
}

/*! @brief Returns the real time in milliseconds since the start of the program.
 
    This function will be faster than getTime() on some platforms. Use this function if you don't need
    accuracy better than +/- 10ms.
 */
double NUSystem::getTimeFast()
{
    static double timeinseconds;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    static struct timespec timenow;
    clock_gettime(CLOCK_REALTIME_FAST, &timenow);
    timeinseconds = (timenow.tv_nsec - m_gettimefast_starttime.tv_nsec)/1e6 + (timenow.tv_sec - m_gettimefast_starttime.tv_sec)*1e3;
#else
    timeinseconds = getTime();
#endif
    return timeinseconds;
}

/*! @brief Returns the the time in milliseconds spent in this process since some arbitary point in the past. 
 
    Each process will need to be responsible for keeping tracking of their start time; I can't do it here
    because I can only save the starting process time where this class was created. However, usually
    the underlying implementation starts the clock from zero when the thread is created.
 
    This function might return the thread time if the platform doesn't support the process times. 
 */
double NUSystem::getProcessTime()
{
    static double timeinseconds;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    static struct timespec timenow;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &timenow);
    timeinseconds = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
#else
    timeinseconds = 1e3*clock()/float(CLOCKS_PER_SEC);
#endif
    return timeinseconds;
}

/*! @brief Returns the the time in milliseconds spent in this thread since the program started.  
 
    Each process will need to be responsible for keeping tracking of their start time; I can't do it here
    because I can only save the starting process time where this class was created. However, usually
    the underlying implementation starts the clock from zero when the thread is created.
 
    This function might return the process time if the platform doesn't support the thread times. 
 */
double NUSystem::getThreadTime()
{
    static double timeinseconds;
#ifdef __NU_SYSTEM_CLOCK_GETTIME
    static struct timespec timenow;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timenow);
    timeinseconds = timenow.tv_nsec/1e6 + timenow.tv_sec*1e3;
#else
    timeinseconds = 1e3*clock()/float(CLOCKS_PER_SEC);
#endif
    return timeinseconds;
}




