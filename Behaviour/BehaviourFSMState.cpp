/*! @file BehaviourFSMState.cpp
    @brief Implementation of behaviour state class

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

#include "BehaviourFSMState.h"
#include "BehaviourProvider.h"

#include "debug.h"
#include "debugverbositybehaviour.h"



void BehaviourFSMState::doState()
{
    // do the behaviour common to all child states
    doStateCommons();
    
    // check for state changes common to all child states
    BehaviourState* nextstate = nextStateCommons();
    if (nextstate == m_state)               // then check if the current state wants to change the state
        nextstate = m_state->getNextState();
    
    // do state transition
    if (nextstate != m_state)
    {
        m_previous_state = m_state;
        m_state = nextstate;
        m_state_changed = true;
    }
    else
        m_state_changed = false;
    
    // perform the current state's behaviour
    m_state->process(m_jobs, m_data, m_actions, m_field_objects, m_game_info, m_team_info);
}


