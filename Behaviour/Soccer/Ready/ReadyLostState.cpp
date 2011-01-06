/*! @file ReadyLostState.cpp
    @brief Implementation of the initial soccer state

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

#include "ReadyLostState.h"
#include "../ReadyState.h"
#include "ReadyLostStates.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

ReadyLostState::ReadyLostState(ReadyState* parent) : SoccerFSMState(parent)
{
    m_ready_state = parent;
    m_lost_pan = new ReadyLostPan(this);
    m_lost_spin = new ReadyLostSpin(this);
    
    m_state = m_lost_pan;
}

ReadyLostState::~ReadyLostState()
{   
    delete m_lost_pan;
    delete m_lost_spin;
}

void ReadyLostState::doStateCommons()
{   // do behaviour that is common to all ready lost states
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "ReadyLostState" << endl;
    #endif
}

BehaviourState* ReadyLostState::nextStateCommons()
{   // do state transitions in the ready lost machine
    return m_state;
}

BehaviourFSMState* ReadyLostState::nextState()
{   // do state transitions in the ready machine
    if (m_state == m_lost_spin and not m_field_objects->self.lost())
        return m_ready_state->m_move_state;
    return this;
}

