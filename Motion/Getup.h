/*! @file Getup.h
    @brief Declaration of Getup class
 
    @class Getup
    @brief A module to move the robot into a standing position.
 
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

#ifndef GETUP_H
#define GETUP_H

#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
class NUWalk;
class MotionScript;

class Getup
{
public:
    Getup(NUWalk* walk);
    ~Getup();
    
    void enable();
    void disable();
    bool enabled();
    bool isActive();
    bool isUsingHead();
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
private:
    void playGetup();
private:
    NUWalk* m_walk;
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;
    
    bool m_enabled;
    MotionScript* m_on_back;
    MotionScript* m_on_front;
    MotionScript* m_on_left;
    MotionScript* m_on_right;

    double m_head_completion_time;
    double m_completion_time;
};

#endif

