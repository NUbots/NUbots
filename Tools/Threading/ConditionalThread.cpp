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
    #include "NUPlatform/NUPlatform.h"
#endif



/*! @brief Creates a thread
    @param name the name of the thread (used entirely for debug purposes)
    @param priority the priority of the thread. If non-zero the thread will be a bona fide real-time thread.
 */
ConditionalThread::ConditionalThread(std::string name, unsigned char priority) : Thread(name, priority)
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "ConditionalThread::ConditionalThread(" << m_name << ", " << static_cast<int>(m_priority) << ")" << std::endl;
    #endif
    int err;
    err = pthread_mutex_init(&m_condition_mutex, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_condition_mutex." << std::endl;
    
    err = pthread_cond_init(&m_condition, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_condition." << std::endl;
    
    err = pthread_mutex_init(&m_running_mutex, NULL);
    if (err != 0)
        errorlog << "ConditionalThread::ConditionalThread(" << m_name << ") Failed to create m_running_mutex." << std::endl;
    pthread_mutex_lock(&m_running_mutex);
}

/*! @brief Stops the thread
 */
ConditionalThread::~ConditionalThread()
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "ConditionalThread::~ConditionalThread() " << m_name << std::endl;
    #endif
    stop();
    pthread_cond_destroy(&m_condition);
    pthread_mutex_destroy(&m_condition_mutex);
    pthread_mutex_destroy(&m_running_mutex);
}

/*! @brief Starts a single execution of the thread's main loop
 * 	@param blocking set this to true if you want to block until this thread is ready to receive the signal
 */
void ConditionalThread::signal(bool blocking)
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread::signal() " << m_name << " at " << Platform->getTime() << std::endl;
    #endif
    if (blocking)
    	pthread_mutex_lock(&m_running_mutex);
    else
    {	// if its not blocking then do a trylock, if the lock fails return
    	int err = pthread_mutex_trylock(&m_running_mutex);
    	if (err !=0)
    	{
			#if DEBUG_THREADING_VERBOSITY > 2
				debug << "ConditionalThread::signal() " << m_name << " is not ready!" << std::endl;
			#endif
			return;
    	}
    }
	pthread_mutex_lock(&m_condition_mutex);
	pthread_cond_signal(&m_condition);
	pthread_mutex_unlock(&m_running_mutex);
	pthread_mutex_unlock(&m_condition_mutex);
}

/* @brief A non-blocking call to signal the start of the thread.
 
          Note. the strangeness of having this separate version when a default arguement would of sufficed, is because
                This version is used in a boost::bind that expects a function that takes no parameters.
 */
void ConditionalThread::signal()
{
    signal(false);
}

/*! @brief Blocks this thread until the signal() function is called
 */
void ConditionalThread::wait()
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread: " << m_name << " is waiting at " << Platform->getTime() << std::endl;
    #endif
    pthread_mutex_lock(&m_condition_mutex);
	pthread_mutex_unlock(&m_running_mutex);
    pthread_cond_wait(&m_condition, &m_condition_mutex);
    pthread_mutex_lock(&m_running_mutex);
    pthread_mutex_unlock(&m_condition_mutex);
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "ConditionalThread: " << m_name << " finished waiting at " << Platform->getTime() << std::endl;
    #endif
}
