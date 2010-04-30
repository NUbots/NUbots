/*! @file WatchDogThread.h
    @brief Declaration of watchdog thread thread class.

    @class WatchDogThread
    @brief The watchdog thread handles low priority 'system' behaviour
 
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

#ifndef WATCHDOG_THREAD_H
#define WATCHDOG_THREAD_H

#include "Tools/Threading/PeriodicThread.h"

class NUbot;

/*! @brief The top-level class
 */
class WatchDogThread : public PeriodicThread
{
public:
    WatchDogThread(NUbot* nubot);
    ~WatchDogThread();
private:
    void periodicFunction();
    
private:
    NUbot* m_nubot;
};

#endif

