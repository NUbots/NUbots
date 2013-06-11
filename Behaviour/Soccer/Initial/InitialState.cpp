/*! @file InitialState.cpp
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

#include "InitialState.h"
#include "../SoccerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Infrastructure/Jobs/MotionJobs/MotionKillJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

InitialState::InitialState(SoccerProvider* provider) : SoccerState(provider)
{
    m_firstrun = true;
}

InitialState::~InitialState()
{
}

BehaviourState* InitialState::nextState()
{
    return this;
}

void InitialState::doState()
{
    if (m_provider->stateChanged() or m_firstrun)
    {   // play a sound, and stop moving
        m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::INITIAL);
        //m_jobs->addMotionJob(new MotionKillJob());
        m_firstrun = false;
    }
    m_jobs->addMotionJob(new WalkJob(0.f,0.f,0.f));
    // In inital the chest led should be off
    m_actions->add(NUActionatorsData::ChestLed, m_data->CurrentTime, std::vector<float>(3,0));
    
    // In initial if we have kick off the led should be on, and off when we don't have kick off
    if (m_game_info->haveKickoff())
    {
        std::vector<float> yellow(3,1);
        yellow[2] = 0;
        m_actions->add(NUActionatorsData::RFootLed, m_data->CurrentTime, yellow);
    }
    else
        m_actions->add(NUActionatorsData::RFootLed, m_data->CurrentTime, std::vector<float>(3,0));
    
    // In initial if the left foot is pressed then we should swap teams
    if (m_provider->singleLeftBumperClick() or m_provider->longLeftBumperClick())
        m_game_info->doManualTeamChange();
}

