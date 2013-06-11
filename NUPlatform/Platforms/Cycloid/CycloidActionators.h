/*! @file CycloidActionators.h
    @brief Declaration of Bear actionators class

    @author Jason Kulk
 
    @class CycloidActionators
    @brief The cycloid actionators class
 
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

#ifndef CYCLOIDACTIONATORS_H
#define CYCLOIDACTIONATORS_H

#include "NUPlatform/NUActionators.h"
class Motors;


class CycloidActionators : public NUActionators
{
public:
    CycloidActionators(Motors* motors);
    ~CycloidActionators();
    
private:
    void copyToHardwareCommunications();
    void copyToServos();
    
private:
    // Actionators
    static std::vector<std::string> m_servo_names;            //!< the names of the available joints (eg HeadYaw, AnklePitch etc)
    
    Motors* m_motors;
};

#endif

