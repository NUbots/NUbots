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

#include "../SoccerState.h"
#include "BallIsLostState.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/TeamInformation.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadNodJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
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
    BallIsLostPan(BallIsLostState* parent) : BallIsLostSubState(parent), m_ROTATIONAL_SPEED(0.1)
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
            debug << "BallIsLostPan" << endl;
        #endif
        // keep track of the time in this state
        if (m_parent->stateChanged())
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
        
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        else
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Ball, 6, 100, -0.95, 0.95));
        
        if (m_team_info->getPlayerNumber() != 1)
            m_jobs->addMotionJob(new WalkJob(0, 0, m_spin_speed));
        else
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
    BallIsLostSpin(BallIsLostState* parent) : BallIsLostSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_spin_speed = m_ROTATIONAL_SPEED;
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~BallIsLostSpin() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        if (m_time_in_state > 1000*(6.28/m_ROTATIONAL_SPEED))
            return m_lost_machine->m_lost_pan;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "BallIsLostSpin" << endl;
        #endif
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (m_parent->stateChanged())
        {   // decided which direction to spin based on the estimated bearing when we enter this state or the current walk speed if we are still walking
            m_time_in_state = 0;
            vector<float> walkspeed;
            if (m_data->getMotionWalkSpeed(walkspeed) and walkspeed[2] != 0)        
                m_spin_speed = mathGeneral::sign(walkspeed[2])*m_ROTATIONAL_SPEED;
            else
                m_spin_speed = mathGeneral::sign(ball.estimatedBearing())*m_ROTATIONAL_SPEED;
        }
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
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

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostMove
/*! @class BallIsLostMove
    In this state we move to one of four positions on the field, and pan for the ball the whole time.
 */
class BallIsLostMove : public BallIsLostSubState
{
public:
    BallIsLostMove(BallIsLostState* parent) : BallIsLostSubState(parent) {}
    ~BallIsLostMove() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        return this;
    }
    void doState()
    {
#if DEBUG_BEHAVIOUR_VERBOSITY > 1
        debug << "BallIsLostMove" << endl;
#endif
        if (m_parent->stateChanged())
        {   // decided which location on the field to go to
        }
        m_jobs->addMotionJob(new WalkJob(0, 0, 0));
        m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::BallAndLocalisation, 0));
    }
private:
};

#endif

