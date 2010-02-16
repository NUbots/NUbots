/*! @file NAOWebotsPlatform.h
    @brief Declaration of NAO in Webots class.

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

#ifndef NAOWEBOTSPLATFORM_H
#define NAOWEBOTSPLATFORM_H

#include "NUPlatform/NUPlatform.h"
#include "JWebots/JRobot.h"

class NAOWebotsPlatform : public NUPlatform, public JRobot
{
// Functions:
public:
    NAOWebotsPlatform(int argc, const char *argv[]);
    ~NAOWebotsPlatform();
    
protected:
private:
    void setNameFromNumber();
    void setTeam(const string& name);
    
// Members:
public:
protected:
private:
    
};

#endif