/*! @file PositioningStates.h
    @brief Declaration of the positioning states

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

#ifndef POSITIONING_STATES_H
#define POSITIONING_STATES_H

#include "../SoccerState.h"
class SoccerFSMState;       // PositioningState is a SoccerFSMState

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class GoToPosition : public SoccerState
{
public:
    GoToPosition(SoccerFSMState* parent) : SoccerState(parent) {}
    ~GoToPosition() {};
    BehaviourState* nextState()
    {   // do state transitions in the positioning state machine
        return this;
    }
protected:
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GoToPosition" << endl;
        #endif
    }
};

#endif

