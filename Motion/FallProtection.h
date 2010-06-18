/*! @file FallProtection.h
    @brief Declaration of FallProtection class
 
    @class FallProtection
    @brief A module to provide protect the robot from damage when falling over.
 
    The module puts the robot into a pose to minimise the impact with the ground.
    The selected pose depends on the fall direction, and the estimated time before
    impact.
 
    The module will also set the stiffness to 0 for all joints not required by the protection
    pose. The stiffness of joints moved by the protection pose are turned off once the joint
    has moved into the protection position. All of the joint stiffnesses are turned off just 
    before the impact.
 
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

#ifndef FALLPROTECTION_H
#define FALLPROTECTION_H

class NUSensorsData;
class NUActionatorsData;
class NUWalk;
#include "Motion/NUMotionProvider.h"

class FallProtection : public NUMotionProvider
{
public:
    FallProtection(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~FallProtection();
    
    bool enabled();
    void enable();
    void disable();
    
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
private:
    NUWalk* m_walk;                                 //!< local pointer to the walk engine
    
    bool m_enabled;
};

#endif

