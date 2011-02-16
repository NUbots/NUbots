/*!  @file DXSerialThread.cpp
     @brief Implementation of DXSerialThread class.
     
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

#include "DXSerialThread.h"
#include "Motors.h"
#include "Tools/Threading/ConditionalThread.h"

#include "debug.h"
#include "debugverbositynuplatform.h"
#include "nubotconfig.h"
#include "debugverbositythreading.h"
#ifdef THREAD_SENSEMOVE_PROFILE
    #include "Tools/Profiling/Profiler.h"
#endif

using namespace std;

/*! @brief Creates a thread
    @param motors the name of the thread (used entirely for debug purposes)
    @param period the time in ms between each main loop execution
 */
DXSerialThread::DXSerialThread(Motors* motors, int period) : PeriodicThread("DXSerialThread", period, 51), m_motors(motors)
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 0
        debug << "DXSerialThread::DXSerialThread(" << m_motors << ", " << m_period << ", " << static_cast<int>(m_priority) << ")" << endl;
    #endif
    m_sensor_thread = 0;
    start();
}

/*! @brief Stops the thread
 */
DXSerialThread::~DXSerialThread()
{
    #if DEBUG_NUPLATFORM_VERBOSITY > 0
        debug << "DXSerialThread::~DXSerialThread() " << m_name << endl;
    #endif
    stop();
}

void DXSerialThread::setSensorThread(ConditionalThread* thread)
{
    m_sensor_thread = thread;
}

void DXSerialThread::periodicFunction()
{
    #ifdef THREAD_SENSEMOVE_PROFILE
        Profiler prof = Profiler("DXSerialThread");
        prof.start();
    #endif
    
    if (m_motors)
    {
        m_motors->read();
        #ifdef THREAD_SENSEMOVE_PROFILE
            prof.split("MotorRead");
        #endif
        m_motors->write();
        #ifdef THREAD_SENSEMOVE_PROFILE
            prof.split("MotorWrite");
        #endif
        m_motors->request();
        #ifdef THREAD_SENSEMOVE_PROFILE
            prof.split("MotorRequest");
        #endif
    }
    
    #ifdef THREAD_SENSEMOVE_PROFILE
        debug << prof;
    #endif
    
    if (m_sensor_thread)
        m_sensor_thread->startLoop();
}
