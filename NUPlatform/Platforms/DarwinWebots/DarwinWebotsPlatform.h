/*! @file DarwinWebotsPlatform.h
    @brief Declaration of Darwin in Webots class.

    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jason Kulk, Jed Rietveld
 
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

#ifndef DARWINWEBOTSPLATFORM_H
#define DARWINWEBOTSPLATFORM_H

#include "NUPlatform/NUPlatform.h"
#include "NUPlatform/Platforms/Webots/JRobot.h"

class DarwinWebotsPlatform : public NUPlatform, public webots::JRobot
{
// Functions:
public:
    DarwinWebotsPlatform(int argc, const char *argv[]);
    ~DarwinWebotsPlatform();
    
    double getTime();
protected:
    void initName();
    void initNumber();
    void initTeam();
    void initMAC();
    
    int m_argc;
    const char** m_argv; 
};

#endif
