/*! @file ConditionalThread.h
    @brief Declaration of (abstract) ConditionalThread class.

    @class ConditionalThread 
    @brief This encapsulates a pthread, provides several additional features.
 
    This particular thread will execute its main loop when it is told to. That is,
    the loop begins when it receives a signal from higher-levels indicating that
    it has a reason to re-run, such as the availability of new data.

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

#ifndef CONDITIONAL_THREAD_H_DEFINED
#define CONDITIONAL_THREAD_H_DEFINED

#include "Thread.h"

#include <string>
#include <pthread.h>

class ConditionalThread : public Thread
{
	public:
		ConditionalThread(std::string name, unsigned char priority);
        virtual ~ConditionalThread();
    
        void startLoop();
        void waitForLoopCompletion();
    
    protected:
        virtual void run() = 0;                // To be overridden by code to run.
        void waitForCondition();
        void onLoopCompleted();                  

    protected:
        pthread_mutex_t m_condition_mutex;     //!< lock for new data signal
        pthread_cond_t m_condition;            //!< signal for new data
        pthread_mutex_t m_running_mutex;       //!< mutex to indicate that the main loop is currently executing
        
};
#endif
