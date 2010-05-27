/*! @file SetState.cpp
    @brief Implementation of the set soccer state

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

#include "SetState.h"
#include "SoccerState.h"
#include "SoccerProvider.h"

#include "Behaviour/Jobs/JobList.h"
#include "Behaviour/GameInformation.h"
#include "NUPlatform/NUSensors/NUSensorsData.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Behaviour/Jobs/MotionJobs/WalkJob.h"

SetState::SetState(SoccerProvider* provider) : SoccerState(provider)
{
}

SetState::~SetState()
{
}

BehaviourState* SetState::nextState()
{
    return this;
}

void SetState::doState()
{
    if (m_provider->stateChanged())
    {   // play a sound, and stop moving
        m_actions->addSound(m_data->CurrentTime, NUSounds::SET);
    }
    // In set the chest led should be yellow
    m_actions->addLeds(NUActionatorsData::ChestLeds, m_data->CurrentTime, 0.9, 1, 0.1);
    
    // In set if we have kick off the led should be on, and off when we don't have kick off
    if (m_game_info->haveKickoff())
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_data->CurrentTime, 1, 1, 0);
    else
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_data->CurrentTime, 0, 0, 0);
    
    m_jobs->addMotionJob(new WalkJob(0,0,0));
}

