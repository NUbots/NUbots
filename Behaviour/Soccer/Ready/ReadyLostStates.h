/*! @file ReadyLostStates.h
    @brief Declaration of the robot is lost in ready states

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

#ifndef READY_LOST_STATES_H
#define READY_LOST_STATES_H

#include "../SoccerState.h"
#include "ReadyLostState.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"

#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"


class ReadyLostSubState : public SoccerState
{
public:
    ReadyLostSubState(ReadyLostState* parent) : SoccerState(parent), m_parent_machine(parent) {};
    virtual ~ReadyLostSubState() {};
protected:
    ReadyLostState* m_parent_machine;
};

// ----------------------------------------------------------------------------------------------------------------------- ReadyLostPan
/*! @class ReadyLostPan
    In this state we stop and do a wide localisation pan. When the pan is completed we move into a spin state.
 */
class ReadyLostPan : public ReadyLostSubState
{
public:
    ReadyLostPan(ReadyLostState* parent) : ReadyLostSubState(parent) 
    {
        reset();
    }
    ~ReadyLostPan() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the lost state machine
        if (m_pan_started and m_pan_end_time < m_data->CurrentTime and not m_parent_machine->stateChanged())
            return m_parent_machine->m_lost_spin;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "ReadyLostPan" << std::endl;
        #endif
        if (m_parent_machine->stateChanged())
            reset();
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        
        if (m_time_in_state < 1000)
            m_jobs->addMotionJob(new WalkJob(0.01, 0, 0));
        else
        {
            m_jobs->addMotionJob(new WalkJob(0, 0, 0));
            m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::Localisation, 300, 1e10, -1.57, 1.57));
        }
        
        // keep track of the time in this state
        m_previous_time = m_data->CurrentTime;
        
        // grab the pan end time
        if (not m_pan_started and m_time_in_state > 5000)
        {
            if (m_data->get(NUSensorsData::MotionHeadCompletionTime, m_pan_end_time))
                m_pan_started = true;
        }
    }
private:
    void reset()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "ReadyLostPan. Resetting" << std::endl;
        #endif
        m_time_in_state = 0;
        m_pan_started = false;
        m_pan_end_time = 0;
    }
    float m_time_in_state;
    double m_previous_time;
    bool m_pan_started;
    double m_pan_end_time;
};

// ----------------------------------------------------------------------------------------------------------------------- ReadyLostSpin
/*! @class ReadyLostSpin
    In this state we spin on the spot and do the nod. After 1.25 revolutions we go back to the pan state.
 */
class ReadyLostSpin : public ReadyLostSubState
{
public:
    ReadyLostSpin(ReadyLostState* parent) : ReadyLostSubState(parent), m_ROTATIONAL_SPEED(0.4)
    {
        m_time_in_state = 0;
        m_previous_time = 0;
    }
    ~ReadyLostSpin() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        if (m_time_in_state > 1.25*1000*(6.28/m_ROTATIONAL_SPEED))
            return m_parent_machine->m_lost_pan;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "ReadyLostSpin" << std::endl;
        #endif
        if (m_parent_machine->stateChanged())
            m_time_in_state = 0;
        else
            m_time_in_state += m_data->CurrentTime - m_previous_time;
        m_previous_time = m_data->CurrentTime;
        
        m_jobs->addMotionJob(new WalkJob(0, 0, m_ROTATIONAL_SPEED));
        m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::Localisation, m_ROTATIONAL_SPEED));
    }
private:
    const float m_ROTATIONAL_SPEED;
    float m_time_in_state;
    float m_previous_time;
};
#endif

