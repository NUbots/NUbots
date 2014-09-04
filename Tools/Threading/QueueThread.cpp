/*!  @file QueueThread.cpp
     @brief Implementation of (abstract) QueueThread class.
     
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

#include "QueueThread.h"
#include "debug.h"
#include "debugverbositythreading.h"



/*! @brief Creates a thread
    @param name the name of the thread (used entirely for debug purposes)
    @param priority the priority of the thread. If non-zero the thread will be a bona fide real-time thread.
 */
template <typename T>
QueueThread<T>::QueueThread(std::string name, unsigned char priority) : Thread(name, priority)
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "QueueThread::QueueThread(" << m_name << ", " << static_cast<int>(m_priority) << ")" << std::endl;
    #endif
    int err;
    err = pthread_mutex_init(&m_condition_mutex, NULL);
    if (err != 0)
        errorlog << "QueueThread::QueueThread(" << m_name << ") Failed to create m_condition_mutex." << std::endl;
    
    err = pthread_cond_init(&m_condition, NULL);
    if (err != 0)
        errorlog << "QueueThread::QueueThread(" << m_name << ") Failed to create m_condition." << std::endl;
}

/*! @brief Stops the thread
 */
template <typename T>
QueueThread<T>::~QueueThread()
{
    #if DEBUG_THREADING_VERBOSITY > 1
        debug << "QueueThread::~QueueThread() " << m_name << std::endl;
    #endif
    stop();
    pthread_cond_destroy(&m_condition);
    pthread_mutex_destroy(&m_condition_mutex);
}

/*! @brief Adds new data to the thread's queue. This also increments the number of required loops
    @param newdata the data to add to the queue
 */
template <typename T>
void QueueThread<T>::pushBack(T newdata)
{
    #if DEBUG_THREADING_VERBOSITY > 2
        debug << "QueueThread::pushBack(" << newdata << ") " << m_name << std::endl;
    #endif
    m_queue.push_back(newdata);
    
    pthread_mutex_lock(&m_condition_mutex);
    pthread_cond_signal(&m_condition);
    pthread_mutex_unlock(&m_condition_mutex);
}

/*! @brief Blocks this thread until the condition is signalled
 */
template <typename T>
void QueueThread<T>::waitForCondition()
{
    if (m_queue.size() == 0)
    {   // if there is no data in the queue to be processed then wait until new data is added
        pthread_mutex_lock(&m_condition_mutex);
        pthread_cond_wait(&m_condition, &m_condition_mutex);
        pthread_mutex_unlock(&m_condition_mutex);
    }
}
