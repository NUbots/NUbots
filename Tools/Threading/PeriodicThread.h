/*! @file PeriodicThread.h
    @brief Declaration of (abstract) PeriodicThread class.

    @class PeriodicThread 
    @brief This encapsulates a pthread, provides several additional features.
 
    This particular thread will execute its main loop when at a specified rate.
 
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

#ifndef PERIODIC_THREAD_H_DEFINED
#define PERIODIC_THREAD_H_DEFINED

#include "Thread.h"

#include <string>
#include <pthread.h>

class PeriodicThread : public Thread
{
	public:
		PeriodicThread(std::string name, int period, unsigned char priority);
        virtual ~PeriodicThread();
    
        virtual void periodicFunction() = 0;   //!< the function which is called periodically
    
    protected:
        void run();
    
    private:
        void sleepThread();                    //!< sleeps the thread for the required amount of time

    protected:
        int m_period;                          //!< the period in ms
        double m_start_time;                   //!< the time the execution of the loop started (we need this to sleep properly)
};

#endif

