/*! @file ReadyState.h
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

#include "ReadyState.h"
#include "Ready/ReadyMoveState.h"
#include "Ready/ReadyMarkState.h"

#include "SoccerProvider.h"

#include "Behaviour/GameInformation.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

ReadyState::ReadyState(SoccerProvider* provider) : SoccerFSMState(provider)
{
    m_move_state = new ReadyMoveState(this);
    m_mark_state = new ReadyMarkState(this);
    
    m_state = m_move_state;
}

ReadyState::~ReadyState()
{
    delete m_move_state;
    delete m_mark_state;
}

void ReadyState::doStateCommons()
{
    if (m_provider->stateChanged())
    {   // play a sound when we enter the ready state
        m_actions->addSound(m_actions->CurrentTime, NUSounds::READY);
    }
    // In set the chest led should be blue
    m_actions->addLeds(NUActionatorsData::ChestLeds, m_actions->CurrentTime, 0, 0.1, 1);
    
    // In set if we have kick off the led should be on, and off when we don't have kick off
    if (m_game_info->haveKickoff())
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_actions->CurrentTime, 1, 1, 0);
    else
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_actions->CurrentTime, 0, 0, 0);
}


