/*! @file NetworkThread.h
    @brief Declaration of the network thread class.

    @class NetworkThread
    @brief The network thread that links network information with public nubot storage
 
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

#ifndef NETWORK_THREAD_H
#define NETWORK_THREAD_H

#include "Tools/Threading/ConditionalThread.h"

class NUbot;

/*! @brief The top-level class
 */
class NetworkThread : public ConditionalThread
{
public:
    NetworkThread(NUbot* nubot);
    ~NetworkThread();
protected:
    void run();
    
private:
    NUbot* m_nubot;
};

#endif

