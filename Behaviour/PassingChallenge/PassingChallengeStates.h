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
#include "PassingChallengeProvider.h"

#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Behaviour/Jobs/MotionJobs/KickJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/MotionFreezeJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

class PassingChallengeSubState : public BehaviourState
{
public:
    PassingChallengeSubState(PassingChallengeProvider* provider){m_provider = provider;};
protected:
    PassingChallengeProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class PassingPausedState : public PassingChallengeSubState
{
public:
    PassingPausedState(PassingChallengeProvider* provider) : PassingChallengeSubState(provider) {};
    BehaviourState* nextState() 
    {
        return m_provider->m_state;
    }
    void doState() 
    {
        debug << "PassingChallenge: Paused" << endl;
    };
};

// ----------------------------------------------------------------------------------------------------------------------- Positioning
class PassingPositionState : public PassingChallengeSubState
{
public:
    PassingPositionState(PassingChallengeProvider* provider) : PassingChallengeSubState(provider) 
    {
    }
    BehaviourState* nextState()
    {   
        return m_provider->m_state;
    }
    
    void doState()
    {
        StationaryObject& yellow_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_LEFT_GOALPOST];
        StationaryObject& yellow_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_YELLOW_RIGHT_GOALPOST];
        StationaryObject& blue_left = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_LEFT_GOALPOST];
        StationaryObject& blue_right = m_field_objects->stationaryFieldObjects[FieldObjects::FO_BLUE_RIGHT_GOALPOST];
        
        if (yellow_left.isObjectVisible() and yellow_right.isObjectVisible())
        {
            float bearing = (yellow_left.ScreenXTheta() + yellow_right.ScreenXTheta())/2;
            float elevation = (yellow_left.ScreenYTheta() + yellow_right.ScreenYTheta())/2;
            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
        }
        else if (blue_left.isObjectVisible() and blue_right.isObjectVisible())
        {
            float bearing = (blue_left.ScreenXTheta() + blue_right.ScreenXTheta())/2;
            float elevation = (blue_left.ScreenYTheta() + blue_right.ScreenYTheta())/2;
            m_jobs->addMotionJob(new HeadTrackJob(elevation, bearing));
        }
        else if (yellow_left.TimeSinceLastSeen() > 250 and yellow_right.TimeSinceLastSeen() > 250 and blue_left.TimeSinceLastSeen() > 250 and blue_right.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation));
        
        
        
        vector<float> position(3,0);
        if (m_game_info->getPlayerNumber() == 2)
        {
            position[0] = -200;
            position[1] = 0;
            position[2] = 0;
        }
        else 
        {
            position[0] = 200;
            position[1] = 0;
            position[2] = 3.1416;
        }

        
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, position, 5, 60, 200);
        m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
    }
};

// ----------------------------------------------------------------------------------------------------------------------- Kicking
class PassingKickingState : public PassingChallengeSubState
{
public:
    PassingKickingState(PassingChallengeProvider* parent) : PassingChallengeSubState(parent) {}
    ~PassingKickingState() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the chase state machine
        return m_provider->m_state;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "PassingKickingState" << endl;
        #endif
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else if (ball.TimeSinceLastSeen() > 250)
            m_jobs->addMotionJob(new HeadPanJob(ball));
        
        bool iskicking;
        m_data->getMotionKickActive(iskicking);
        float bearing_to_goal;
        if (m_game_info->getPlayerNumber() == 2)
            bearing_to_goal = BehaviourPotentials::getBearingToOwnGoal(m_field_objects, m_game_info);
        else
            bearing_to_goal = BehaviourPotentials::getBearingToOpponentGoal(m_field_objects, m_game_info);
        
        if(!iskicking)
        {
            vector<float> speed = BehaviourPotentials::goToBall(ball, self, bearing_to_goal);
            m_jobs->addMotionJob(new WalkJob(speed[0], speed[1], speed[2]));
        }
        
        if( (ball.estimatedDistance() < 20.0f) && fabs(bearing_to_goal) < 3.1416/8)
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

// ----------------------------------------------------------------------------------------------------------------------- BallLost
class PassingBallLostState : public PassingChallengeSubState
{
public:
    PassingBallLostState(PassingChallengeProvider* parent) : PassingChallengeSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~PassingBallLostState() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "BallIsLostSpin" << endl;
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
class PassingImLostState : public PassingChallengeSubState
{
public:
    PassingImLostState(PassingChallengeProvider* parent) : PassingChallengeSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~PassingImLostState() {};
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

