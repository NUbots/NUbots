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

#include "../../SoccerState.h"
class SoccerFSMState;       // PositioningState is a SoccerFSMState

#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

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
        static bool m_close_approach = false;
        if (m_previous_time < m_data->CurrentTime-2000.) {
            m_close_approach = false;
        }
        
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        StationaryObject& owngoal = BehaviourPotentials::getOwnGoal(m_field_objects, m_game_info);
        StationaryObject& opponentgoal = BehaviourPotentials::getOpponentGoal(m_field_objects, m_game_info);
        
        
        vector<float> position;
        if (m_team_info->getPlayerNumber() == 1)
            position = self.CalculatePositionToProtectGoalFromMobileObject(ball, owngoal, 120);
        else if (m_team_info->getPlayerNumber() == 4)
            position = self.CalculatePositionToProtectGoalFromMobileObject(ball, owngoal, 60);
        else
            position = BehaviourPotentials::CalculateSupportPlayerPosition(ball, self);
        
        float distance = sqrt(position[0]*position[0] + position[1]*position[1]);
        float bearing = atan2(position[1], position[0]);
        
        
        vector<float> speed;
        /*if (m_team_info->getPlayerNumber() == 1 and (distance < 30. or m_close_approach and distance < 120.)
            or m_team_info->getPlayerNumber() != 1 and ( distance < 20. or m_close_approach and distance < 60. )) {
            speed = BehaviourPotentials::goToPointBackwards(distance, bearing, ball.estimatedBearing(), 10, 100, 200);
            m_close_approach = true;
        } else {*/
            speed = BehaviourPotentials::goToPoint(distance, bearing, ball.estimatedBearing(), 20, 100, 200);
            m_close_approach = false;
        //}
        //vector<float> result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 50, 100);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
        
        if (ball.TimeSinceLastSeen() > 15000)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
        else
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, 0.9*ball.estimatedDistance(), 9000, -1.57, 1.57));
        
        m_previous_time = m_data->CurrentTime;
    }
private:
    float m_previous_time;
};

#endif

