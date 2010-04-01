/*! @file SenseMoveThread.h
    @brief Declaration of the sense->move thread class.

    @class SenseMoveThread
    @brief The sense->move thread that links sensor data to motion
 
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

#ifndef SENSEMOVE_THREAD_H
#define SENSEMOVE_THREAD_H

#include "Tools/Threading/ConditionalThread.h"

class NUbot;

/*! @brief The top-level class
 */
class SenseMoveThread : public ConditionalThread
{
public:
    SenseMoveThread(NUbot* nubot);
    ~SenseMoveThread();
protected:
    void run();
    
private:
    NUbot* m_nubot;
};

#endif

