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
    BallIsLostPan(BallIsLostState* parent) : BallIsLostSubState(parent) {}
    ~BallIsLostPan() {};
protected:
    BehaviourState* nextState()
    {   // do state transitions in the ball is lost state machine
        double pan_end_time;
        debug << m_data << endl;
        if (m_data->getMotionHeadCompletionTime(pan_end_time) and pan_end_time < m_data->CurrentTime)
            return m_lost_machine->m_lost_spin;
        else
            return this;
    }
    void doState()
    {
        #if DEBUG_BEHAVIOUR_VERBOSITY > 1
            debug << "BallIsLostPan" << endl;
        #endif
        m_jobs->addMotionJob(new WalkJob(0, 0, 0));
        m_jobs->addMotionJob(new HeadPanJob(HeadPanJob::BallAndLocalisation));
    }
};

// ----------------------------------------------------------------------------------------------------------------------- BallIsLostPan
/*! @class BallIsLostPan
    In this state we spin on the spot and do the nod. After 1 revolution we go into the move state
 */
class BallIsLostSpin : public BallIsLostSubState
{
public:
    BallIsLostSpin(BallIsLostState* parent) : BallIsLostSubState(parent), m_ROTATIONAL_SPEED(0.4) {}
    ~BallIsLostSpin() {};
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
        float spinspeed;
        if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].estimatedBearing() < 0)
            spinspeed = -m_ROTATIONAL_SPEED;
        else
            spinspeed = m_ROTATIONAL_SPEED;
        m_jobs->addMotionJob(new WalkJob(0, 0, spinspeed));
        m_jobs->addMotionJob(new HeadNodJob(HeadNodJob::BallAndLocalisation, spinspeed));
    }
private:
    const float m_ROTATIONAL_SPEED;
};

#endif

