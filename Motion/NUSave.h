/*! @file NUSave.h
    @brief Declaration of a ball blocking class
 
    @class NUSave
    @brief A module to block the ball using the legs (suitable for both goal keeper and field player)
 
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

#ifndef NUSAVE_H
#define NUSAVE_H

class NUSensorsData;
class NUActionatorsData;
class BlockJob;
class SaveJob;
class NUWalk;
#include "Motion/NUMotionProvider.h"

class NUSave : public NUMotionProvider
{
public:
    NUSave(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~NUSave();
    
    void stop();
    void stopHead();
    void stopArms();
    void stopLegs();
    void kill();
    
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();
    
    bool requiresHead() {return true;}
    bool requiresArms() {return true;}
    bool requiresLegs() {return true;}
    
    void process(NUSensorsData* data, NUActionatorsData* actions);
    void process(BlockJob* job);
    void process(SaveJob* job);
private:
public:
private:
    NUWalk* m_walk;
    NUSensorsData* m_data;
    NUActionatorsData* m_actions;

    MotionScript m_block_left;
    MotionScript m_block_right;
    MotionScript m_block_centre;
    MotionScript m_dive_left;
    MotionScript m_dive_right;
    
    double m_head_completion_time;
    double m_arm_completion_time;
    double m_completion_time;
};

#endif

