/*! @file BallIsLostStates.h
    @brief Declaration of the ball is lost states

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

#ifndef BALL_IS_LOST_STATES_H
#define BALL_IS_LOST_STATES_H

#include "../../SoccerState.h"
#include "BallIsLostState.h"
#include "Behaviour/BehaviourPotentials.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/GameInformation/GameInformation.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Tools/Math/General.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class BallIsLostSubState : public SoccerState
{
public:
    BallIsLostSubState(BallIsLostState* parent) : SoccerState(parent), m_lost_machine(parent) {};
    virtual ~BallIsLostSubState() {};
protected:
    BallIsLostState* m_lost_machine;
};

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostPan
/*! @class BallIsLostPan
    In this state we stop and do a wide pan. When the pan is completed we move into the spin state.
 */
class BallIsLostPan : public BallIsLostSubState
{
public:
    BallIsLostPan(BallIsLostState* parent) : BallIsLostSubState(parent), m_ROTATIONAL_SPEED(0.6)
    {
        m_spin_speed = 0;
        m_time_in_state = 0;
        m_previous_time = 0;
        m_pan_started = false;
        m_pan_end_time = 0;
    }
    ~BallIsLostPan() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        // we transition to the spin state when the pan is completed.
        if (m_pan_started and m_pan_end_time < m_data->CurrentTime and not m_parent->stateChanged())
            return m_lost_machine->m_lost_spin;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << m_data->CurrentTime << ": BallIsLostPan" << endl;
        #endif
        // keep track of the time in this state
        bool kickIsActive = false;
        
        m_data->get(NUSensorsData::MotionKickActive, kickIsActive);
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "Kick Active: " << kickIsActive << " State Changed: " << m_parent->stateChanged() << " Time delta: " << m_data->CurrentTime - m_previous_time << endl;
        #endif
        if (m_parent->stateChanged() or kickIsActive or m_data->CurrentTime - m_previous_time > 200)
            reset();
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
        // grab the pan end time
        if (not m_pan_started and m_time_in_state > 200)
        {
            if (m_data->get(NUSensorsData::MotionHeadCompletionTime, m_pan_end_time))
                m_pan_started = true;
        }
        
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "Time in state: " << m_time_in_state << endl;
        #endif
        
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        
        m_jobs->addMotionJob(new WalkJob(0, 0, 0));
    }
private:
    void reset()
    {
        m_time_in_state = 0;
        m_pan_started = false;
        m_pan_end_time = 0;
        if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].estimatedBearing() < 0)
            m_spin_speed = -m_ROTATIONAL_SPEED;
        else
            m_spin_speed = m_ROTATIONAL_SPEED;
    }
    const float m_ROTATIONAL_SPEED;
    float m_spin_speed;
    float m_time_in_state;
    double m_previous_time;
    bool m_pan_started;
    double m_pan_end_time;
};

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostPan
/*! @class BallIsLostPan
    In this state we spin on the spot and do the nod. After 1 revolution we go into the move state
 */
class BallIsLostSpin : public BallIsLostSubState
{
public:
    BallIsLostSpin(BallIsLostState* parent) : BallIsLostSubState(parent), m_ROTATIONAL_SPEED(0.6)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~BallIsLostSpin() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        if (m_time_in_state > 5*(1.5*6.28/m_ROTATIONAL_SPEED))
            return m_lost_machine->m_lost_move;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << m_data->CurrentTime << ": BallIsLostSpin" << endl;
        #endif
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (m_parent->stateChanged())
        {   // decided which direction to spin based on the estimated bearing when we enter this state or the current walk speed if we are still walking
            m_time_in_state = 0;
            vector<float> walkspeed;
            if (m_data->get(NUSensorsData::MotionWalkSpeed, walkspeed) and walkspeed[2] != 0)        
                m_spin_speed = mathGeneral::sign(walkspeed[2])*m_ROTATIONAL_SPEED;
            else
                m_spin_speed = mathGeneral::sign(ball.estimatedBearing())*m_ROTATIONAL_SPEED;
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Ball, 10, 2000, -1.5, 1.5));
        }
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        
        m_jobs->addMotionJob(new WalkJob(0.12, 0, m_spin_speed*2.f));
    }
private:
    const float m_ROTATIONAL_SPEED;
    float m_spin_speed;
    float m_time_in_state;
    float m_previous_time;
};

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostMove
/*! @class BallIsLostMove
    In this state we move to one of four positions on the field, and pan for the ball the whole time.
 */
class BallIsLostMove : public BallIsLostSubState
{
public:
    BallIsLostMove(BallIsLostState* parent) : BallIsLostSubState(parent) 
    {
        m_position = vector<float>(3,0);
    }
    ~BallIsLostMove() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        Self& self = m_field_objects->self;
        vector<float> distance = self.CalculateDifferenceFromFieldState(m_position);
        if (distance[0] < 30)
            return m_lost_machine->m_lost_spin;
        else
            return this;
    }
    void doState()
    {
#if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << m_data->CurrentTime << ": BallIsLostMove" << endl;
#endif
        Self& self = m_field_objects->self;
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        StationaryObject& owngoal = BehaviourPotentials::getOwnGoal(m_field_objects, m_game_info);
        StationaryObject& opponentgoal = BehaviourPotentials::getOpponentGoal(m_field_objects, m_game_info);
    
        if (m_parent->stateChanged())
        {   // decided which location on the field to go to
            if (m_game_info->getPlayerNumber() == 1)
            {   // goal keeper goes back to his goal
                m_position[0] = owngoal.X() - 25*mathGeneral::sign(owngoal.X());
                m_position[1] = 0;
                m_position[2] = 0;
            }
            else if (m_game_info->getPlayerNumber() == 2)
            {   // player two goes to centre circle
                m_position[0] = 0;
                m_position[1] = 0;
                m_position[2] = 0;
            }
            else
            {   // goes two opponent penalty spot
                m_position[0] = opponentgoal.X() - 120*mathGeneral::sign(opponentgoal.X());
                m_position[1] = 0;
                m_position[2] = 0;
            }
        }
        vector<float> speed = BehaviourPotentials::goToFieldState(m_field_objects->self, m_position, 0, 55, 0);
        vector<float> result = BehaviourPotentials::sensorAvoidObjects(speed, m_data, 50, 100);
        m_jobs->addMotionJob(new WalkJob(result[0], result[1], result[2]));
        
        float pan_width = 1.5;
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Ball, 50, 9000, -pan_width, pan_width));
    }
private:
    vector<float> m_position;       // the target location in field coordinates
};

#endif

