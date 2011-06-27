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

#include "../../SoccerState.h"
class SoccerFSMState;       // ChaseState is a SoccerFSMState

#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"

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
    GoToBall(ChaseState* parent) : ChaseSubState(parent) 
    {
        m_time_since_pan = 0;
        m_pan_end_time = 0;
        m_pan_started = false;
        m_previous_time = 0;
    }
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
        if (m_parent->stateChanged() or m_data->CurrentTime - m_previous_time > 200)
        {
            m_time_since_pan = 0;
            m_pan_end_time = 0;
            m_pan_started = false;
        }
        
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        
        if (m_time_since_pan > 15000)
        {
            StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
            StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
            StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
            StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
            
            float bearing_to_yellow = self.CalculateBearingToStationaryObject(yellow_left);
            float bearing_to_blue = self.CalculateBearingToStationaryObject(blue_right);
            
            vector<StationaryObject> posts;
            if (fabs(bearing_to_yellow) < fabs(bearing_to_blue))
            {
                posts.push_back(yellow_left);
                posts.push_back(yellow_right);
            }
            else
            {
                posts.push_back(blue_left);
                posts.push_back(blue_right);
            }
            m_jobs->addMotionJob(new HeadPanJob(posts));
            m_time_since_pan = 0;
            m_pan_started = true;
            debug << m_data->CurrentTime << ": Goal Post Pan Started" << endl;
        }
        else
        {
            m_time_since_pan += m_data->CurrentTime - m_previous_time;
            if (not m_pan_started)
            {
                if (ball.isObjectVisible())
                {
                    debug << m_data->CurrentTime << ": Tracking ball" << endl;
                    m_jobs->addMotionJob(new HeadTrackJob(ball));
                }
                else if (ball.TimeSinceLastSeen() > 250)
                {
                    debug << m_data->CurrentTime << ": Ball Pan" << endl;
                    m_jobs->addMotionJob(new HeadPanJob(ball));
                }
            }
            else if (m_data->get(NUSensorsData::MotionHeadCompletionTime, m_pan_end_time) and m_pan_end_time < m_data->CurrentTime)
            {
                debug << m_data->CurrentTime << ": Goal Post Pan Completed" << endl;
                m_pan_started = false;
            }
        }
        
        
        bool iskicking;
        m_data->get(NUSensorsData::MotionKickActive, iskicking);
        if(!iskicking)
        {
            vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info));
            vector<float> result;
            // decide whether we need to dodge or not
            float leftobstacle = 255;
            float rightobstacle = 255;
            vector<float> temp;
            if (m_data->get(NUSensorsData::LDistance, temp) and temp.size() > 0)
                leftobstacle = temp[0];
            if (m_data->get(NUSensorsData::RDistance, temp) and temp.size() > 0)
                rightobstacle = temp[0];
            
            // if the ball is too far away to kick and the obstable is closer than the ball we need to dodge!
            if (ball.estimatedDistance() > 25 and min(leftobstacle, rightobstacle) < ball.estimatedDistance())
                result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, min(ball.estimatedDistance(), 25.0f), 75);
            else
                result = speed;
            
            m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
        }
        
        if( (ball.estimatedDistance() < 25.0f) && BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info))
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
        
        m_previous_time = m_data->CurrentTime;
    }
    
private:
    float m_time_since_pan;
    float m_pan_end_time;
    bool m_pan_started;
    float m_previous_time;
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

