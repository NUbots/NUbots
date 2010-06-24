/*! @file ChaseStates.h
    @brief Declaration of the chase ball states

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

#ifndef CHASE_STATES_H
#define CHASE_STATES_H

#include "../SoccerState.h"
class SoccerFSMState;       // ChaseState is a SoccerFSMState

#include "Behaviour/BehaviourPotentials.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/TeamInformation.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class ChaseSubState : public SoccerState
{
public:
    ChaseSubState(ChaseState* parent) : SoccerState(parent), m_parent_machine(parent) {};
    virtual ~ChaseSubState() {};
protected:
    ChaseState* m_parent_machine;
};

class GoToBall : public ChaseSubState
{
public:
    GoToBall(ChaseState* parent) : ChaseSubState(parent) {}
    ~GoToBall() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "GoToBall" << endl;
        #endif
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else if (ball.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(ball));
        
        bool iskicking;
        m_data->getMotionKickActive(iskicking);
        if(!iskicking)
        {
            vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info));
            vector<float> result;
            // decide whether we need to dodge or not
            float leftobstacle = 255;
            float rightobstacle = 255;
            vector<float> temp;
            if (m_data->getDistanceLeftValues(temp) and temp.size() > 0)
                leftobstacle = temp[0];
            if (m_data->getDistanceRightValues(temp) and temp.size() > 0)
                rightobstacle = temp[0];
            
            // if the ball is too far away to kick and the obstable is closer than the ball we need to dodge!
            if (ball.estimatedDistance() > 20 and min(leftobstacle, rightobstacle) < ball.estimatedDistance())
                result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, min(ball.estimatedDistance(), 25.0f), 75);
            else
                result = speed;
            
            m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
        }
        
        if( (ball.estimatedDistance() < 20.0f) && BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info))
        {
            vector<float> kickPosition(2,0);
            vector<float> targetPosition(2,0);
            kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
            kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
            targetPosition[0] = kickPosition[0] + 1000.0f;
            targetPosition[1] = kickPosition[1];
            KickJob* kjob = new KickJob(0,kickPosition, targetPosition);
            m_jobs->addMotionJob(kjob);
        }
    }
};

class FindTarget : public ChaseSubState
{
public:
    FindTarget(ChaseState* parent) : ChaseSubState(parent) {}
    ~FindTarget() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "FindTarget" << endl;
        #endif
    }
};

class Kick : public ChaseSubState
{
public:
    Kick(ChaseState* parent) : ChaseSubState(parent) {}
    ~Kick() {};
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return this;
    }
protected:
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "Kick" << endl;
        #endif
    }
};


#endif

