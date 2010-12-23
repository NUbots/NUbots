/*! @file JWalk.h
    @brief Declaration of jason's walk class
 
    @class JWalk
    @brief A module to provide locomotion
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010, 2011 Jason Kulk
 
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

#ifndef JWALK_H
#define JWALK_H

#include "Motion/NUWalk.h"
class JWalkState;

class JWalk : public NUWalk
{
public:
    JWalk();
    ~JWalk();
protected:
    void doWalk();
private:
    void calculateGaitPhase();
public:
    // The JWalk blackboard
    float GaitPhase;
    JWalkState* LeftState;
    JWalkState* RightState;
    
    JWalkState* LeftStance;
    JWalkState* LeftPush;
    JWalkState* LeftSwing;
    JWalkState* LeftAccept;
    
    JWalkState* RightStance;
    JWalkState* RightPush;
    JWalkState* RightSwing;
    JWalkState* RightAccept;
    
    double CurrentTime;
    double PreviousTime;
protected:
private:
    float GaitFrequency;
};

extern JWalk* JWalkBlackboard;

#endif

