/*!  @file ConditionalThread.cpp
     @brief Implementation of (abstract) ConditionalThread class.
     
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

#include "ConditionalThread.h"
#include "debug.h"
#include "debugverbositythreading.h"
#if DEBUG_THREADING_VERBOSITY > 2
    #include "NUPlatform/NUSystem.h"
#endif

using namespace std;

/*! @brief Creates a thread
    @param name the name of the thread (used entirely for debug purposes)
    @param priority the priority of the thread. If non-zero the thread will be a bona fide real-time thread.
 */
ConditionalThread::ConditionalThread(string name, unsigned char priority) : Thread(name, priority)
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "ConditionalThread::ConditionalThread(" << m_name << ", " << static_cast<int>(m_priority) << ")" << endl;
    #endif
    int err;
    err = pthread_mutex_init(&m_condition_mutex, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_condition_mutex." << endl;
    
    err = pthread_cond_init(&m_condition, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_condition." << endl;
    
    err = pthread_mutex_init(&m_running_mutex, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_running_mutex." << endl;
}

/*! @brief Stops the thread
 */
ConditionalThread::~ConditionalThread()
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "ConditionalThread::~ConditionalThread() " << m_name << endl;
    #endif
    this->stop();
    pthread_cond_destroy(&m_condition);
    pthread_mutex_destroy(&m_condition_mutex);
    pthread_mutex_destroy(&m_running_mutex);
}

/*! @brief Starts a single execution of the thread's main loop
 
    This is implemented such that if the loop is already running, a second signal is not sent; this will allow threads
    to drop frames properly.
 
    It also locks the m_running_mutex automatically.
 */
void ConditionalThread::startLoop()
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread::startLoop() " << m_name << endl;
    #endif
    int err = 0;
    err = pthread_mutex_trylock(&m_running_mutex);
    if (err == 0)
    {   // if the loop is not already running then, signal it to start
        pthread_mutex_lock(&m_condition_mutex);
        err = pthread_cond_signal(&m_condition);
        pthread_mutex_unlock(&m_condition_mutex);
    }
    else 
    {
        #if DEBUG_THREADING_VERBOSITY > 2
            debug << "ConditionalThread::startLoop() " << m_name << " is already running!" << endl;
        #endif
    }

}

/*! @brief Blocks this thread until the condition is signalled
 */
void ConditionalThread::waitForCondition()
{
    pthread_mutex_lock(&m_condition_mutex);
    pthread_cond_wait(&m_condition, &m_condition_mutex);
    pthread_mutex_unlock(&m_condition_mutex);
}

/*! @brief Unlocks the m_running_mutex, consequently allowing the main loop to be started again with startLoop()
 
    It would be really nice to have this happen automatically, at the end of each iteration...Because you must call
    it in your implementation of run() if you want it to ever run again ;).
 */
void ConditionalThread::onLoopCompleted()
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread::loopCompleted() " << m_name << " at " << nusystem->getTime() << endl;
    #endif
    pthread_mutex_unlock(&m_running_mutex);
}

/*! @brief Blocks the calling thread until the current iteration of the thread's loop completes
 */
void ConditionalThread::waitForLoopCompletion()
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread::waitForLoopCompletion() " << m_name << " at " << nusystem->getTime() << endl;
    #endif
    pthread_mutex_lock(&m_running_mutex);            // block if motion thread is STILL running
    pthread_mutex_unlock(&m_running_mutex);
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread::waitForLoopCompletion() " << m_name << " wait completed at " << nusystem->getTime() << endl;
    #endif
}
