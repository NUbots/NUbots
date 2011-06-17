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
#include "../SoccerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

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
        m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::PENALISED);
        m_jobs->addMotionJob(new HeadJob(m_data->CurrentTime + 1000, vector<float>(2,0)));
    }
    // In penalty the chest led should be red
    vector<float> red(3,0);
    red[0] = 1;
    m_actions->add(NUActionatorsData::ChestLed, m_data->CurrentTime, red);
    
    // In penalty we should not walk
    m_jobs->addMotionJob(new WalkJob(0,0,0));
}

