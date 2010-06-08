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

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "Behaviour/TeamInformation.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadTrackJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadPanJob.h"

#include "debug.h"
#include "debugverbositybehaviour.h"
using namespace std;

class GoToBall : public SoccerState
{
public:
    GoToBall(SoccerFSMState* parent) : SoccerState(parent) {}
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
        MobileObject& ball = m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL];
        if (ball.isObjectVisible())
            m_jobs->addMotionJob(new HeadTrackJob(ball));
        
        m_jobs->addMotionJob(new WalkJob(0, 0, 0));
    }
};

class FindTarget : public SoccerState
{
public:
    FindTarget(SoccerFSMState* parent) : SoccerState(parent) {}
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

class Kick : public SoccerState
{
public:
    Kick(SoccerFSMState* parent) : SoccerState(parent) {}
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

