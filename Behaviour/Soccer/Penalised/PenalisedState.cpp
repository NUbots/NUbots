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
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"

#include "debug.h"

#include <sstream>
using namespace std;

PenalisedState::PenalisedState(SoccerProvider* provider) : SoccerState(provider)
{
    m_previous_penalty_reason = PENALTY_NONE;
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
        if (m_game_info->amIASubstitute())
            m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::SUBSTITUTE);
        else
            m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::PENALISED);
        m_jobs->addMotionJob(new HeadJob(m_data->CurrentTime + 1000, vector<float>(2,0)));
    }
    
    // In penalty the chest led should be red, or purple in sub
    vector<float> colour(3,0);
    colour[0] = 1;
    if (m_game_info->amIASubstitute())
        colour[2] = 1;
        
    m_actions->add(NUActionatorsData::ChestLed, m_data->CurrentTime, colour);
    
    // In penalty we should not walk
    m_jobs->addMotionJob(new WalkJob(0,0,0));
    
    // check if we have moved from being a substitute, to being penalised for a different reason
    // ie. we are about to enter the game
    if (m_game_info->getPenaltyReason() != m_previous_penalty_reason and m_previous_penalty_reason == PENALTY_SPL_SUBSTITUTE)
    {
        // Play a few sounds to tell people whats going on (penalised and the new player position)
        m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime, NUSounds::PENALISED);
        int newposition = m_game_info->getSubstituteNumber();
        if (newposition < 0 or newposition > m_game_info->getNumberOfPlayers())
            newposition = m_game_info->getPlayerNumber();
        
        stringstream sound;
        sound << newposition << ".wav";
        m_actions->add(NUActionatorsData::Sound, m_data->CurrentTime + 3000, sound.str());
        m_team_info->setPlayerNumber(newposition);
    }
    
    m_previous_penalty_reason = m_game_info->getPenaltyReason();
}

