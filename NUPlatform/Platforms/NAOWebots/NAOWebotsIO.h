/*! @file NAOWebotsIO.h
    @brief Declaration of naoio class.
 
    @class NAOWebotsIO
    @brief NAOWebotsIO class for input and output to streams, files and networks on the NAO in Webots platform

    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NAOWebotsIO_H
#define NAOWebotsIO_H

#include "NUPlatform/NUIO.h"

class NUbot;
class NAOWebotsPlatform;
class NAOWebotsNetworkThread;

class NAOWebotsIO: public NUIO
{
// Functions:
public:
    NAOWebotsIO(NUbot* nubot, NAOWebotsPlatform* platform);
    ~NAOWebotsIO();
    
protected:
private:
    
// Members:
public:
protected:
private:
    NAOWebotsNetworkThread* m_network;      //!< the simulated network
    
};

#endif

