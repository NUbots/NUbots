/*!  @file PeriodicSignalerThread.cpp
     @brief Implementation of PeriodicSignalerThread class.
     
     @author Jason Kulk
 
 Copyright (c) 2011 Jason Kulk
 
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

#include "PeriodicSignalerThread.h"
#include "ConditionalThread.h"

#include "debug.h"
#include "debugverbositythreading.h"

using namespace std;

/*! @brief Creates a periodic signaller thread. Effectively, this class is an adapter turning a conditional thread into a periodic one.
    @param name the name of the thread (used entirely for debug purposes)
    @param listener the thread which will be signaled periodically by this thread to run
    @param period the time in ms between each main loop execution
 */
PeriodicSignalerThread::PeriodicSignalerThread(string name, ConditionalThread* listener, int period) : PeriodicThread(name, period, listener->getPriority()+1), m_listener(listener)
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "PeriodicSignalerThread::PeriodicSignalerThread(" << m_name << ", " << listener->m_name << ", " << m_period << ", " << static_cast<int>(m_priority) << ")" << endl;
    #endif
}

/*! @brief Stops the thread
 */
PeriodicSignalerThread::~PeriodicSignalerThread()
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "PeriodicSignalerThread::~PeriodicSignalerThread() " << m_name << endl;
    #endif
    stop();
}

void PeriodicSignalerThread::periodicFunction()
{
    m_listener->signal();
}

