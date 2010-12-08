/*! @file DribblingChallengeProvider.h
    @brief Declaration of simple chase ball behaviour for testing and demonstration purposes 
 
    @class DribblingChallengeProvider
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

#ifndef DRIBBLINGCHALLENGE_BEHAVIOUR_H
#define DRIBBLINGCHALLENGE_BEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"

class DribblingPausedState;
class DribblingMoveToBallState;
class DribblingScanForObstaclesState;
class DribblingKickingState;
class DribblingBallLostState;
class DribblingImLostState;

#include <vector>
#include <string>

class DribblingChallengeProvider : public BehaviourFSMProvider
{
public:
    DribblingChallengeProvider(Behaviour* manager);
    ~DribblingChallengeProvider();
    
protected:
    BehaviourState* nextStateCommons();
    
    friend class DribblingPausedState;
    BehaviourState* m_paused;
    friend class DribblingMoveToBallState;
    BehaviourState* m_move_to_ball;
    friend class DribblingScanForObstaclesState;
    BehaviourState* m_scan_for_obstacles;
    friend class DribblingKickingState;
    BehaviourState* m_kicking;
    friend class DribblingBallLostState;
    BehaviourState* m_ball_lost;
    friend class DribblingImLostState;
    BehaviourState* m_lost;
};


#endif

