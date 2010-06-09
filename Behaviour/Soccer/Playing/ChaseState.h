/*! @file ChaseState.h
    @brief Declaration of the chase ball soccer state
 
    @class ChaseState
    @brief The chase ball soccer state machine
 
    There are 3 states in this machine
        - GoToBall in this state we move to the ball
        - FindTarget in this state we look for the target to kick the ball to
          This will either be the goal, team mate or open space. If we are well
          localised when entering this state we will transition straight to Kick
        - Kick in this state we kick the ball
 
    If we are not near the ball then we are in the GoToBall state, no exceptions.
    When we reach the ball we transition to the FindTarget state.
    When we have found our target with low enough uncertainity we transition to the Kick state.

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

#ifndef CHASE_SOCCER_FSM_STATE_H
#define CHASE_SOCCER_FSM_STATE_H

#include "../SoccerFSMState.h"
class GoToBall;
class FindTarget;
class Kick;

class ChaseState : public SoccerFSMState
{
public:
    ChaseState(SoccerFSMState* parent);
    ~ChaseState();
    BehaviourFSMState* nextState();
private:
    void doStateCommons();
    BehaviourState* nextStateCommons();
protected:
    GoToBall* m_go_to_ball;
    FindTarget* m_find_target;
    Kick* m_kick;
};


#endif

