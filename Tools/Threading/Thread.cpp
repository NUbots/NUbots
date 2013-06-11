/*!  @file Thread.cpp
     @brief Implementation of (abstract) Thread class.
     
     @author Aaron Wong, Jason Kulk
 
 Copyright (c) 2009, 2010 Aaron Wong, Jason Kulk
 
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

#include "Thread.h"
#include "debug.h"
#include "debugverbositythreading.h"



/*! @brief Creates a thread
    @param name the name of the thread (used entirely for debug purposes)
    @param priority the priority of the thread. If non-zero the thread will be a bona fide real-time thread.
 */
Thread::Thread(std::string name, unsigned char priority) : m_name(name), running(false), m_priority(priority)
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "Thread::Thread(" << m_name << ", " << static_cast<int>(m_priority) << ")" << std::endl;
    #endif
}

/*! @brief Stops the thread
 */
Thread::~Thread()
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "Thread::~Thread(): " << m_name << std::endl;
    #endif
    stop();
}

/*! @brief Starts the thread
    @return 0 is returned if successful, -1 is returned otherwise
 */
int Thread::start()
{
	if(running) 
		return -1;
    
    #if DEBUG_THREADING_VERBOSITY > 0
        debug << "Thread::start(): " << m_name << std::endl;
    #endif
    
    int err;
    err = pthread_create(&m_pthread, NULL, runThread, (void*) this);         // The last parameter is the arguement to the thread
    if (err != 0)
    {
        errorlog << "Thread::start(). Failed to create " << m_name << ". The error code was: " << err << std::endl;
        return -1;
    }
    
    if (m_priority > 0)
    {   // if the priority is non-zero then we create the thread as a bona fide real-time thread with the given priority
        sched_param param;
        param.sched_priority = m_priority;
        pthread_setschedparam(m_pthread, SCHED_FIFO, &param);            // Note. This will fail (quietly) if the underlying OS doesn't allow real-time
        
        // double check
        int actualpolicy;
        sched_param actualparam;
        pthread_getschedparam(m_pthread, &actualpolicy, &actualparam);
        #if DEBUG_THREADING_VERBOSITY > 0
            debug << "Thread::start(). " << m_name << " Policy: " << actualpolicy << " Priority: " << actualparam.sched_priority << std::endl;
        #endif
        if (actualpolicy != SCHED_FIFO)
            debug << "Thread::start(). " << m_name << ". Warning your thread does not have the correct policy." << std::endl;
        
        if (actualparam.sched_priority != m_priority)
            debug << "Thread::start(). " << m_name << ". Warning your thread does not have the correct priority." << std::endl;
    }

	return 0;
}

/*! @brief Blocks the calling thread until this thread is completed.
 */
int Thread::join()
{
    return pthread_join(m_pthread, NULL); 
}

/*! @brief Cancels the threads execution, and sets the running flag to false
 */
void Thread::stop()
{
    #if DEBUG_THREADING_VERBOSITY > 0
        debug << "Thread::stop(): " << m_name << std::endl;
    #endif
    pthread_cancel(m_pthread);
	running = false;
}

/*! @brief The static wrapper function to call the underlying run function.
 */
void* Thread::runThread(void* thread)
{
	reinterpret_cast<Thread*>(thread)->run();
    pthread_exit(NULL);
    return thread;
}
