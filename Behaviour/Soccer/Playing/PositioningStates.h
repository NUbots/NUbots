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

#include "Behaviour/BehaviourPotentials.h"

#include "Behaviour/Jobs/JobList.h"
#include "Vision/FieldObjects/FieldObjects.h"

#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class GoToPosition : public SoccerState
{
public:
    GoToPosition(SoccerFSMState* parent) : SoccerState(parent) {}
    ~GoToPosition() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the positioning state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GoToPosition" << endl;
        #endif
        
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        StationaryObject& owngoal = BehaviourPotentials::getOwnGoal(m_field_objects, m_game_info);
        vector<float> result = self.CalculatePositionToProtectGoalFromMobileObject(ball, owngoal, 75);
        
        float distance = sqrt(result[0]*result[0] + result[1]*result[1]);
        float bearing = atan2(result[1], result[0]);
        vector<float> speed = BehaviourPotentials::goToPoint(distance, bearing, ball.estimatedBearing());
        
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else if (ball.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
};

#endif

