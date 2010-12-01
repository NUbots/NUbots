/*! @file NAOWebotsSystem.h
    @brief Declaration of NAO in Webots system class

    @author Jason Kulk
 
    @class NAOWebotsSystem
    @brief NAO in Webots system class
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NAOWEBOTSSYSTEM_H
#define NAOWEBOTSSYSTEM_H

#include "NUPlatform/NUSystem.h"
#include "NAOWebotsPlatform.h"

class NAOWebotsSystem : public NUSystem
{
public:
    NAOWebotsSystem(int argc, const char *argv[]);
    ~NAOWebotsSystem();
    
    virtual long double getPosixTimeStamp();
    virtual double getTime();
    virtual double getTimeFast(); 
private:
    NAOWebotsPlatform* m_platform;                      //!< In webots we need a pointer to the Robot so we can access the API
    long double m_simulator_start_timestamp;            //!< The unix timestamp in milliseconds when the simulation was started
};

#endif

