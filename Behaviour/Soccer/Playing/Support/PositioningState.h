/*! @file PositioningState.h
    @brief Declaration of the chase ball soccer state
 
    @class PositioningState
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

#ifndef POSITIONING_SOCCER_FSM_STATE_H
#define POSITIONING_SOCCER_FSM_STATE_H

#include "../../SoccerFSMState.h"
class GoToPosition;

class PositioningState : public SoccerFSMState
{
public:
    PositioningState(SoccerFSMState* parent);
    ~PositioningState();
private:
    BehaviourFSMState* nextState();
    void doStateCommons();
    BehaviourState* nextStateCommons();
protected:
    GoToPosition* m_go_to_position;
};


#endif

