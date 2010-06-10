/*! @file PenalisedState.cpp
    @brief Implementation of the initial soccer state

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

#include "PenalisedState.h"
#include "SoccerState.h"
#include "SoccerProvider.h"

#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/GameInformation.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Behaviour/Jobs/MotionJobs/HeadJob.h"
#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

PenalisedState::PenalisedState(SoccerProvider* provider) : SoccerState(provider)
{
}

PenalisedState::~PenalisedState()
{
}

BehaviourState* PenalisedState::nextState()
{
    return this;
}

void PenalisedState::doState()
{
    if (m_provider->stateChanged())
    {   // play a sound, and stop moving
        m_actions->addSound(m_data->CurrentTime, NUSounds::PENALISED);
        m_jobs->addMotionJob(new HeadJob(m_data->CurrentTime + 300, vector<float>(2,0)));
    }
    // In penalty the chest led should be red
    m_actions->addLeds(NUActionatorsData::ChestLeds, m_data->CurrentTime, 1, 0, 0);
    
    // In penalty we should not walk
    m_jobs->addMotionJob(new WalkJob(0,0,0));
}

