/*! @file PeriodicSignalerThread.h
    @brief Declaration of PeriodicSignalerThread class.

    @class PeriodicSignalerThread 
    @brief This encapsulates a pthread, provides several additional features.
 
    This particular thread will execute its main loop when at a specified rate.
 
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

#ifndef PERIODICSIGNALER_THREAD_H_DEFINED
#define PERIODICSIGNALER_THREAD_H_DEFINED

#include "PeriodicThread.h"
class ConditionalThread;

#include <string>
#include <pthread.h>

class PeriodicSignalerThread : public PeriodicThread
{
	public:
		PeriodicSignalerThread(std::string name, ConditionalThread* listener, int period);
        ~PeriodicSignalerThread();
    
        void periodicFunction();

    protected:
        double m_start_time;                   //!< the time the execution of the loop started (we need this to sleep properly)
        ConditionalThread* m_listener;         //!< the thread which is listening for the signals
};

#endif

