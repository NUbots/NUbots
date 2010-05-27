/*! @file PlayingState.h
    @brief Declaration of the playing soccer state
 
    @class PlayingState
    @brief The playing soccer state

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

#ifndef PLAYING_FSM_STATE_H
#define PLAYING_FSM_STATE_H

class SoccerProvider;
#include "SoccerFSMState.h"

class ChaseBallBehaviourState;

class PlayingState : public SoccerFSMState
{
public:
    PlayingState(SoccerProvider* provider);
    ~PlayingState();
    BehaviourFSMState* nextState() {return this;};
private:
    void doStateCommons();
private:
    ChaseBallBehaviourState* m_chase_state;
};


#endif

