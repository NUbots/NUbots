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

class NUSensorsData;
class NUActionatorsData;
class NUWalk;
class MotionScript2013;
#include "Motion/NUMotionProvider.h"

class Getup : public NUMotionProvider
{
public:
    Getup(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~Getup();
    
    void enable();
    void disable();
    bool enabled();
    
    void stop();
    void stopHead();
    void stopArms();
    void stopLegs();
    void kill();
    
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();
    
    bool requiresHead();
    bool requiresArms() {return true;}
    bool requiresLegs() {return true;}
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
private:
    void playGetup();
private:
    NUWalk* m_walk;
    
    bool m_enabled;
    MotionScript2013* getup_back_script_;
    MotionScript2013* getup_front_script_;
    MotionScript2013* getup_left_script_;
    MotionScript2013* getup_right_script_;

    double m_head_completion_time;
    double m_arm_completion_time;
    double m_completion_time;
};

#endif

