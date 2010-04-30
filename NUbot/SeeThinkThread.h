/*! @file SeeThinkThread.h
    @brief Declaration of the see->think thread class.

    @class SeeThinkThread
    @brief The see->think thread that links vision information with thoughts
 
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

#ifndef SEETHINK_THREAD_H
#define SEETHINK_THREAD_H

#include "Tools/Threading/ConditionalThread.h"
#include "Kinematics/Kinematics.h"

class NUbot;

/*! @brief The top-level class
 */
class SeeThinkThread : public ConditionalThread
{
public:
    SeeThinkThread(NUbot* nubot);
    ~SeeThinkThread();
protected:
    void run();
    
private:
    NUbot* m_nubot;
	Kinematics *m_kinematicModel;
};

#endif

