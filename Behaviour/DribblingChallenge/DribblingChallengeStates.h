/*! @file PassingChallengeStates.h
    @brief Chase ball states

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

#ifndef PASSINGCHALLGENGE_STATES_H
#define PASSINGCHALLGENGE_STATES_H

#include "Behaviour/BehaviourState.h"
#include "DribblingChallengeProvider.h"

#include "Behaviour/BehaviourPotentials.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/TeamInformation.h"
#include "Behaviour/GameInformation.h"

#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/MotionFreezeJob.h"

#include <utility>

#include "debug.h"
#include "debugverbositybehaviour.h"

class DribblingChallengeSubState : public BehaviourState
{
public:
    DribblingChallengeSubState(DribblingChallengeProvider* provider){m_provider = provider;};
protected:
    DribblingChallengeProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class DribblingPausedState : public DribblingChallengeSubState
{
public:
    DribblingPausedState(DribblingChallengeProvider* provider) : DribblingChallengeSubState(provider) {};
    BehaviourState* nextState() 
    {
        return m_provider->m_state;
    }
    void doState() 
    {
        debug << "DribblingPausedState" << std::endl;
    }
};

// ----------------------------------------------------------------------------------------------------------------------- MoveToBallState
class DribblingMoveToBallState : public DribblingChallengeSubState
{
public:
    DribblingMoveToBallState(DribblingChallengeProvider* provider) : DribblingChallengeSubState(provider) 
    {
    }
    BehaviourState* nextState()
    {   
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.estimatedDistance() < 25)
            return m_provider->m_scan_for_obstacles;
        else
            return m_provider->m_state;
    }
    
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "DribblingMoveToBallState" << std::endl;
        #endif
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else if (ball.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(ball));
        
        std::vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info));
        std::vector<float> result;
        // decide whether we need to dodge or not
        float leftobstacle = 255;
        float rightobstacle = 255;
        std::vector<float> temp;
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
};

// ----------------------------------------------------------------------------------------------------------------------- ScanForObstacles
std::vector<MobileObjects> obstacles;
class DribblingScanForObstaclesState : public DribblingChallengeSubState
{
public:
    DribblingScanForObstaclesState(DribblingChallengeProvider* parent) : DribblingChallengeSubState(parent)
    {
        m_time_in_state = 0;
        m_previous_time = 0;
        m_pan_started = false;
        m_pan_end_time = 0;
    }
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        // we transition to the spin state when the pan is completed.
        if (m_pan_started and m_pan_end_time < m_data->CurrentTime and not m_provider->stateChanged())
            return m_provider->m_kicking;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "DribblingScanForObstaclesState" << std::endl;
        #endif
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        // keep track of the time in this state
        if (m_provider->stateChanged())
            reset();
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
        // grab the pan end time
        if (not m_pan_started and m_time_in_state > 100)
        {
            if (m_data->getMotionHeadCompletionTime(m_pan_end_time))
                m_pan_started = true;
        }
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        m_jobs->addMotionJob(new WalkJob(0, 0, 0));
        
        size_t size = m_field_objects->ambiguousFieldObjects.size()
        for (i=0; i<size; i++)
        {
            int id = m_field_objects->ambiguousFieldObjects[i]->getID();
            if(m_field_objects->ambiguousFieldObjects[i]->getID() == FieldObjects::FO_PINK_ROBOT_UNKNOWN)
                obstacles.push_back(m_field_objects->ambiguousFieldObjects[i]);
            else if(m_field_objects->ambiguousFieldObjects[i]->getID() == FieldObjects::FO_BLUE_ROBOT_UNKNOWN)
                obstacles.push_back(m_field_objects->ambiguousFieldObjects[i]);
            else if(m_field_objects->ambiguousFieldObjects[i]->getID() == FieldObjects::FO_ROBOT_UNKNOWN)
                obstacles.push_back(m_field_objects->ambiguousFieldObjects[i]);
        }
                
    }
private:
    void reset()
    {
        m_time_in_state = 0;
        m_pan_started = false;
        m_pan_end_time = 0;
        obstacles.clear();
    }
    float m_time_in_state;
    double m_previous_time;
    bool m_pan_started;
    double m_pan_end_time;
};

// ----------------------------------------------------------------------------------------------------------------------- DribblingKickingState
class DribblingKickingState : public DribblingChallengeSubState
{
public:
    DribblingKickingState(DribblingChallengeProvider* parent) : DribblingChallengeSubState(parent)
    {
        m_kicked = false;
        m_kick_was_active = false;
    }
protected:
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        if (m_kicked)
            return m_provider->m_move_to_ball;
        else
            return this;
            
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "DribblingKickingState" << std::endl;
        #endif
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else if (ball.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(ball));
        
        if (m_provider->stateChanged())
            m_kicked = false;
        
        bool iskicking;
        m_data->getMotionKickActive(iskicking);
        if(!iskicking)
        {
            std::vector<float> speed = BehaviourPotentials::goToBall(ball, self, BehaviourPotentials::CalculateBestKickBearing(obstacles, m_field_objects, m_game_info));     // need target bearing
            m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
        }
        
        if (not iskicking and m_kick_was_active)
            m_kicked = true;
        
        if( (ball.estimatedDistance() < 25.0f && fabs(BehaviourPotentials::CalculateBestKickBearing(obstacles, m_field_objects, m_game_info)) < 0.30))
        {
            std::vector<float> kickPosition(2,0);
            std::vector<float> targetPosition(2,0);
            kickPosition[0] = ball.estimatedDistance() * cos(ball.estimatedBearing());
            kickPosition[1] = ball.estimatedDistance() * sin(ball.estimatedBearing());
            targetPosition[0] = kickPosition[0] + 1000.0f;
            targetPosition[1] = kickPosition[1];
            KickJob* kjob = new KickJob(0,kickPosition, targetPosition);
            m_jobs->addMotionJob(kjob);
        }
    }
private:
    bool m_kicked;
    bool m_kick_was_active;
};

// ----------------------------------------------------------------------------------------------------------------------- BallLost
class DribblingBallLostState : public DribblingChallengeSubState
{
public:
    DribblingBallLostState(DribblingChallengeProvider* parent) : DribblingChallengeSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~DribblingBallLostState() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "BallIsLostSpin" << std::endl;
        #endif
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        m_spin_speed = m_ROTATIONAL_SPEED;
        
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else
            m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::BallAndLocalisation, m_spin_speed));
        
        m_jobs->addMotionJob(new WalkJob(0, 0, m_spin_speed));
    }
private:
    const float m_ROTATIONAL_SPEED;
    float m_spin_speed;
    float m_time_in_state;
    float m_previous_time;
};

// ----------------------------------------------------------------------------------------------------------------------- ImLost
class DribblingImLostState : public DribblingChallengeSubState
{
public:
    DribblingImLostState(DribblingChallengeProvider* parent) : DribblingChallengeSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~DribblingImLostState() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        return this;
    }
    void doState()
    {
        m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Localisation, m_spin_speed));
        m_jobs->addMotionJob(new WalkJob(0, 0, m_spin_speed));
    }
private:
    const float m_ROTATIONAL_SPEED;
    float m_spin_speed;
    float m_time_in_state;
    float m_previous_time;
};


#endif

