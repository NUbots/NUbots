/*! @file ImLostState.h
    @brief Implementation of the ready soccer state

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

#include "ImLostState.h"
#include "ImLostStates.h"

#include "Behaviour/GameInformation.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

ImLostState::ImLostState(SoccerFSMState* parent) : SoccerFSMState(parent)
{
    m_lost_pan = new ImLostPan(this);
    m_lost_spin = new ImLostSpin(this);
    
    m_state = m_lost_pan;
}

ImLostState::~ImLostState()
{
    delete m_lost_pan;
    delete m_lost_spin;
}

void ImLostState::doStateCommons()
{   // do behaviour that is common to all sub lost states
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "ImLostState" << endl;
    #endif
}

BehaviourState* ImLostState::nextStateCommons()
{   // do state transitions in the lost state machine
    return m_state;
}

BehaviourFSMState* ImLostState::nextState()
{   // do state transitions in the playing state machine
    return this;
}


