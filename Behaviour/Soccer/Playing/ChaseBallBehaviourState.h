/*! @file ChaseBallBehaviourState.h
    @brief Declaration of a wrapper state for the chase ball provider (ie the field-less demo behaviour)
 
    @class ChaseBallBehaviourState
    @brief A wrapper state for the chase ball provider (ie the field-less demo behaviour)

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

#ifndef CHASE_BALL_BEHAVIOUR_STATE_H
#define CHASE_BALL_BEHAVIOUR_STATE_H

class SoccerFSMState;
#include "../SoccerState.h"
class ChaseBallProvider;

class ChaseBallBehaviourState : public SoccerState
{
public:
    ChaseBallBehaviourState(SoccerFSMState* parent);
    ~ChaseBallBehaviourState();
    BehaviourState* nextState();
protected:
    void doState();
private:
    ChaseBallProvider* m_chase_provider;
};


#endif

