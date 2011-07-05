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
#include "ReadyMoveState.h"
#include "ReadyLostState.h"

#include "../SoccerProvider.h"

#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

ReadyState::ReadyState(SoccerProvider* provider) : SoccerFSMState(provider)
{
    m_move_state = new ReadyMoveState(this);
    m_lost_state = new ReadyLostState(this);
    
    m_state = m_move_state;
}

ReadyState::~ReadyState()
{
    delete m_move_state;
    delete m_lost_state;
}

void ReadyState::doStateCommons()
{
    if (m_provider->stateChanged())
    {   // play a sound when we enter the ready state
        m_actions->add(NUActionatorsData::Sound, m_actions->CurrentTime, NUSounds::READY);
    }
    // In ready the chest led should be blue
    vector<float> blue(3,0);
    blue[1] = 0.1;
    blue[2] = 1.0;
    m_actions->add(NUActionatorsData::ChestLed, m_actions->CurrentTime, blue);
    
    // In ready if we have kick off the led should be on, and off when we don't have kick off
    if (m_game_info->haveKickoff())
    {
        vector<float> yellow(3,1);
        yellow[2] = 0;
        m_actions->add(NUActionatorsData::RFootLed, m_actions->CurrentTime, yellow);
    }
    else
        m_actions->add(NUActionatorsData::RFootLed, m_actions->CurrentTime, vector<float>(3,0));
}

BehaviourFSMState* ReadyState::nextStateCommons()
{   // do state transitions in the ready state machine
    if (m_provider->stateChanged())
    {   // if the game state has changed decide whether to go into the lost state or start positioning immediately.
        if (m_provider->wasPreviousState(m_provider->m_initial) or m_provider->wasPreviousState(m_provider->m_finished) or m_provider->wasPreviousState(m_provider->m_penalised))
            return m_lost_state;
        else if (m_field_objects->self.lost())
            return m_lost_state;
        else
            return m_move_state;
    }
    else
        return static_cast<BehaviourFSMState*>(m_state);
}

BehaviourFSMState* ReadyState::nextState()
{   // the ready state machine can never modify the game state
    return this;
}

