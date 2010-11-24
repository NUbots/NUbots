/*! @file PausedWalkOptimisationState.h
    @brief A state where the walk optimisation is paused

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

#ifndef PAUSEDWALKOPTIMISATIONSTATE_H
#define PAUSEDWALKOPTIMISATIONSTATE_H

#include "WalkOptimisationProvider.h"
#include "WalkOptimisationState.h"

#include "Behaviour/Jobs/JobList.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"
#include "Behaviour/Jobs/MotionJobs/HeadJob.h"

#include "debug.h"

class PausedWalkOptimisationState : public WalkOptimisationState
{
public:
    PausedWalkOptimisationState(WalkOptimisationProvider* parent) : WalkOptimisationState(parent) {};
    virtual ~PausedWalkOptimisationState() {};
    virtual BehaviourState* nextState() {return this;};
    virtual void doState()
    {
        m_jobs->addMotionJob(new WalkJob(0,0,0));
        vector<float> zero(m_actions->getNumberOfJoints(NUActionatorsData::HeadJoints), 0);
        m_jobs->addMotionJob(new HeadJob(m_actions->CurrentTime + 500, zero));
    };
};

#endif

