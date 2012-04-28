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

#include <algorithm>

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
            m_pan_end_time = 0;
            m_pan_started = false;
            m_pan_finished = false;
        }
        
        Self& self = Blackboard->Objects->self;
        MobileObject& ball = Blackboard->Objects->mobileFieldObjects[FieldObjects::FO_BALL];
        bool iskicking;
        m_data->get(NUSensorsData::MotionKickActive, iskicking);
        
        // if (pan not run)
        //      if (pan start condition)
        //          start pan
        //      else if (its been a long time since a pan)
        //          set pan as not having run
        
        // if (not panning) then
        //      do usual track ball
        
        if (not m_pan_started and not iskicking)
        {   
            if (ball.estimatedDistance() < 100 and fabs(BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info)) < 1.3 and ball.TimeSeen() > 1000)
            {   
                StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
                StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
                StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
                StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
                
                float timesinceyellowgoalseen = min(yellow_left.TimeSinceLastSeen(), yellow_right.TimeSinceLastSeen());
                float timesincebluegoalseen = min(blue_left.TimeSinceLastSeen(), blue_right.TimeSinceLastSeen());
                float timesincegoalseen = min(timesinceyellowgoalseen, timesincebluegoalseen);
                float hackfactor = (9.0/35000.0)*timesincegoalseen + 1;
                
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
                m_jobs->addMotionJob(new HeadPanJob(posts, hackfactor));
                m_pan_started = true;
                m_pan_end_time = m_data->CurrentTime + 500;
                m_pan_time_captured = false;
                m_pan_finished = false;
                //cout << m_data->CurrentTime << ": Goal Post Pan Started" << endl;
            }
        }
        else if (m_pan_finished and m_data->CurrentTime - m_pan_end_time > 45000)
        {
            m_pan_started = false;
            m_pan_finished = false;
        }
        
        // this is a hack to get the pan end time right given the delay in the update of the motion sensors
        // above we set the pan time to be 1s ahead, and then only update the end time when its longer
        // ie. after it has been updated.
        if (m_pan_started and not m_pan_finished)
        {
            float endtime;
            m_data->get(NUSensorsData::MotionHeadCompletionTime, endtime);
            if (not m_pan_time_captured and endtime > m_pan_end_time)
            {
                m_pan_end_time = endtime;
                m_pan_time_captured = true;
            }
            if (m_data->CurrentTime >= m_pan_end_time)
                m_pan_finished = true;
        }
        
        // this is a HUGE hack
        // I am feeling really lazy at the moment; I am updating the time the ball was last seen even though it is NOT seen
        // This is to simply prevent the ball from being lost during the look away pan (this will also trick the other robots via a team information)
        if (m_pan_started and not m_pan_finished)       
            ball.updateTimeLastSeen(m_data->CurrentTime - 1000);
        
        if (not m_pan_started or m_pan_finished)
        {
            
            if (ball.TimeSinceLastSeen() > 2300)
            {
                //cout << m_data->CurrentTime << ": Ball Pan" << endl;
                m_jobs->addMotionJob(new HeadPanJob(ball, 0.5));
            }
            else if (ball.TimeSinceLastSeen() > 750)
            {
                //cout << m_data->CurrentTime << ": Ball Pan" << endl;
                if (ball.isObjectVisible())
                {
                    //cout << m_data->CurrentTime << ": Tracking ball" << endl;
                    m_jobs->addMotionJob(new HeadTrackJob(ball));
                } else {
                    m_jobs->addMotionJob(new HeadPanJob(ball, 0.1));
                }
            }
        }
        
        if(not iskicking)
        {
            vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info));
            vector<float> result;
            // decide whether we need to dodge or not
            vector<float> obstacles = BehaviourPotentials::getObstacleDistances(m_data);
            float leftobstacle = obstacles.at(0);
            float rightobstacle = obstacles.at(1);

            // if the ball is too far away to kick and the obstable is closer than the ball we need to dodge!
            result = speed;
            
            if (m_pan_started and not m_pan_finished and ball.estimatedDistance() < 20)
                result = vector<float>(3,0);
            
            m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
        }
        
        if((ball.estimatedDistance() < 21.0f) && BehaviourPotentials::opponentsGoalLinedUp(m_field_objects, m_game_info) && ball.TimeSeen() > 0 && m_pan_finished)
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
        
        //cout << m_data->CurrentTime << ": pan_started: " << m_pan_started << " pan_finished: " << m_pan_finished << " pan end time: " << m_pan_end_time << endl; 
        m_previous_time = m_data->CurrentTime;
    }
    
private:
    float m_pan_end_time;
    bool m_pan_started, m_pan_time_captured, m_pan_finished;
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


