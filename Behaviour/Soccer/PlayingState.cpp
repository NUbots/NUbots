/*! @file PlayingState.h
    @brief Implementation of the ready soccer state

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

#include "PlayingState.h"
#include "Playing/ChaseBallBehaviourState.h"

#include "SoccerProvider.h"

#include "Behaviour/GameInformation.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

PlayingState::PlayingState(SoccerProvider* provider) : SoccerFSMState(provider)
{
    m_chase_state = new ChaseBallBehaviourState(this);
    
    m_state = m_chase_state;
}

PlayingState::~PlayingState()
{
    delete m_chase_state;
}

void PlayingState::doStateCommons()
{
    if (m_provider->stateChanged())
    {   // play a sound when we enter the playing state
        m_actions->addSound(m_actions->CurrentTime, NUSounds::PLAYING);
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_actions->CurrentTime, 0, 0, 0);
    }
    // In playing the chest led should be green
    m_actions->addLeds(NUActionatorsData::ChestLeds, m_actions->CurrentTime, 0.1, 1, 0.1);
}


