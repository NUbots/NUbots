/*! @file Thread.h
    @brief Declaration of (abstract) Thread class.

    @class Thread 
    @brief This encapsulates a pthread, provided several additional features.

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

#ifndef THREAD_H_DEFINED
#define THREAD_H_DEFINED

#include <pthread.h>
#include <string>

class Thread
{
	public:
		Thread(std::string name, unsigned char priority);
        virtual ~Thread();
		int start();
        int join();
		void stop();
    
    unsigned char getPriority() {return m_priority;};
    
    protected:
        virtual void run() = 0;                 // To be overridden by code to run.
    
	private:
		static void* runThread(void* thread);

	public:
        const std::string m_name;               //!< the name of the thread
    
    protected:
        bool running;                           //!< true if the thread is running, false if it hasn't been started yet, or it has been stopped
        const unsigned char m_priority;         //!< the priority of the thread. A priority of zero means this thread is not real-time
    private:
        pthread_t m_pthread;                    //!< the underlying pthread instance
};
#endif
