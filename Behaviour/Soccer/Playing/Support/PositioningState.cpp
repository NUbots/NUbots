/*! @file PositioningState.h
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

#include "PositioningState.h"
#include "PositioningStates.h"

#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

PositioningState::PositioningState(SoccerFSMState* parent) : SoccerFSMState(parent)
{
    m_go_to_position = new GoToPosition(this);
    
    m_state = m_go_to_position;
}

PositioningState::~PositioningState()
{
    delete m_go_to_position;
}

void PositioningState::doStateCommons()
{   // do behaviour that is common to all sub positioning states
    #if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "PositioningState" << endl;
    #endif
}

BehaviourState* PositioningState::nextStateCommons()
{   // do state transitions in the positioning state machine
    return m_state;
}

BehaviourFSMState* PositioningState::nextState()
{   // do state transitions in the playing state machine
    return this;
}



