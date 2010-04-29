/*!  @file PeriodicThread.cpp
     @brief Implementation of (abstract) PeriodicThread class.
     
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

#include "PeriodicThread.h"
#include "NUPlatform/NUSystem.h"
#include "debug.h"
#include "debugverbositythreading.h"
    
#include <time.h>
#ifdef __USE_POSIX199309                // Check if clock_nanosleep is avaliable
    #define __NU_PERIODIC_CLOCK_NANOSLEEP 
#endif
#include <errno.h>

using namespace std;

/*! @brief Creates a thread
    @param name the name of the thread (used entirely for debug purposes)
    @param period the time in ms between each main loop execution
    @param priority the priority of the thread. If non-zero the thread will be a bona fide real-time thread.
 */
PeriodicThread::PeriodicThread(string name, int period, unsigned char priority) : Thread(name, priority), m_period(period)
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "PeriodicThread::PeriodicThread(" << m_name << ", " << m_period << ", " << static_cast<int>(m_priority) << ")" << endl;
    #endif
}

/*! @brief Stops the thread
 */
PeriodicThread::~PeriodicThread()
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "PeriodicThread::~PeriodicThread() " << m_name << endl;
    #endif
    this->stop();
}

/*! @brief Sleeps for the required amount of time
 */
void PeriodicThread::sleepThread()
{
    double timenow = nusystem->getTime();
    double requiredsleeptime = m_period - (timenow - m_start_time);
    if (requiredsleeptime < 0)
        debug << "PeriodicThread::sleep() " << m_name << " the thread took too long to complete: no time to sleep." << endl;
    else
        NUSystem::msleep(requiredsleeptime);
    
    m_start_time = nusystem->getTime();
}

/*! @brief Periodically runs
 */
void PeriodicThread::run()
{
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        sleepThread();
        try
        {
            periodicFunction();
        }
        catch (std::exception& e)
        {
            debug << "PeriodicThread::run(): " << m_name << " Unhandled exception: " << e.what() << endl;
        }
    }
}
