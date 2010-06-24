/*! @file PassingChallengeProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes 
 
    @class PassingChallengeProvider
    @brief A simple chase ball behaviour for testing and demonstration purposes 

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

#ifndef PASSINGCHALLENGE_BEHAVIOUR_H
#define PASSINGCHALLENGE_BEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"

class PassingPositionState;
class PassingKickingState;
class PassingBallLostState;
class PassingImLostState;
class PassingPausedState;

#include <vector>
#include <string>

class PassingChallengeProvider : public BehaviourFSMProvider
{
public:
    PassingChallengeProvider(Behaviour* manager);
    ~PassingChallengeProvider();
    
    friend class PassingPositionState;
    PassingPositionState* m_position_state;
    friend class PassingKickingState;
    PassingKickingState* m_kick_state;
    friend class PassingBallLostState;
    PassingBallLostState* m_ball_lost_state;
    friend class PassingImLostState;
    PassingImLostState* m_lost_state;
    friend class PassingPausedState;
    PassingPausedState* m_paused_state;
protected:
    BehaviourState* nextStateCommons();
};


#endif

