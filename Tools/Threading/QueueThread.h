/*! @file QueueThread.h
    @brief Declaration of (abstract) QueueThread class.

    @class QueueThread 
    @brief This encapsulates a pthread, provides several additional features.
 
    This particular thread will execute its main loop when it is told to. That is,
    the loop begins when it receives a signal from higher-levels indicating that
    it has a reason to re-run. 
 
    The number of times to run is queued up, so if tell the thread to run three times 
    in quick succession the main loop will run exactly three times. A queue is provided
    which stores data for each execution.
 
    The QueueThread is a template, the template parameter selects the type of data
    in the queue.

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

#ifndef QUEUE_THREAD_H_DEFINED
#define QUEUE_THREAD_H_DEFINED

#include "Thread.h"

#include <string>
#include <deque>
#include <pthread.h>

template <typename T>
class QueueThread : public Thread
{
	public:
		QueueThread(std::string name, unsigned char priority);
        virtual ~QueueThread();
    
        void pushBack(T newdata);
    
    protected:
        virtual void run() = 0;                // To be overridden by code to run.
        void waitForCondition();              

    protected:
        pthread_mutex_t m_condition_mutex;     //!< lock for new data signal
        pthread_cond_t m_condition;            //!< signal for new data
    
        std::deque<T> m_queue;                 //!< the queue of the data for the thread
};

#include "QueueThread.cpp"                      // this is the standard way to do template classes if when you separate declaration and implementation.
                                                // just make sure that you don't compile QueueThread.cpp separately
#endif
