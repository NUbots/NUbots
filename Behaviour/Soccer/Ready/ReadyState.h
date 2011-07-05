/*! @file ReadyState.h
    @brief Declaration of the ready soccer state
 
    @class ReadyState
    @brief The top-level ready state machine. There are only two states at this level
        - ReadyLostState where the robot will attempt to localise making certain we don't leave the field
        - ReadyMoveState where the robot will move to its kick off position
 
    We are in the lost state if we are lost, or the previous state was Initial, Penalised or Finished.
    We are in the move state if we are not in the lost state ;).

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

#ifndef READY_FSM_STATE_H
#define READY_FSM_STATE_H

class SoccerProvider;
#include "../SoccerFSMState.h"

class ReadyMoveState;
class ReadyLostState;

class ReadyState : public SoccerFSMState
{
public:
    ReadyState(SoccerProvider* provider);
    ~ReadyState();
protected:
    BehaviourFSMState* nextState();
    void doStateCommons();
    BehaviourFSMState* nextStateCommons();

    friend class ReadyMoveState;
    BehaviourFSMState* m_move_state;
    friend class ReadyLostState;
    BehaviourFSMState* m_lost_state;
};


#endif

