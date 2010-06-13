/*! @file ReadyLostState.h
    @brief Declaration of the initial soccer state
 
    @class ReadyLostState
    @brief The initial soccer state

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

#ifndef READY_LOST_STATE_H
#define READY_LOST_STATE_H

#include "../SoccerFSMState.h"

class ReadyState;
class ReadyLostPan;
class ReadyLostSpin;

class ReadyLostState : public SoccerFSMState
{
public:
    ReadyLostState(ReadyState* parent);
    ~ReadyLostState();
protected:
    void doStateCommons();
    BehaviourState* nextStateCommons();
    BehaviourFSMState* nextState();
    
    ReadyState* m_ready_state;
    friend class ReadyLostPan;
    BehaviourState* m_lost_pan;
    friend class ReadyLostSpin;
    BehaviourState* m_lost_spin;
};


#endif

