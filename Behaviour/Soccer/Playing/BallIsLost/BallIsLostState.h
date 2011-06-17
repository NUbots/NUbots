/*! @file BallIsLostState.h
    @brief Declaration of the chase ball soccer state
 
    @class BallIsLostState
    @brief The chase ball soccer state

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

#ifndef BALL_IS_LOST_SOCCER_FSM_STATE_H
#define BALL_IS_LOST_SOCCER_FSM_STATE_H

#include "../../SoccerFSMState.h"

class BallIsLostPan;
class BallIsLostSpin;
class BallIsLostMove;

class BallIsLostState : public SoccerFSMState
{
public:
    BallIsLostState(SoccerFSMState* parent);
    ~BallIsLostState();
    BehaviourFSMState* nextState();
private:
    void doStateCommons();
    BehaviourState* nextStateCommons();
protected:
    friend class BallIsLostPan;
    BehaviourState* m_lost_pan;
    friend class BallIsLostSpin;
    BehaviourState* m_lost_spin;
    friend class BallIsLostMove;
    BehaviourState* m_lost_move;
};


#endif

