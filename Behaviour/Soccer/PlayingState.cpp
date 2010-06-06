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
#include "Playing/ChaseState.h"
#include "Playing/PositioningState.h"
#include "Playing/BallIsLostState.h"
#include "Playing/ImLostState.h"

#include "SoccerProvider.h"

#include "Behaviour/GameInformation.h"
#include "Behaviour/TeamInformation.h"
#include "Vision/FieldObjects/FieldObjects.h"
#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "NUPlatform/NUActionators/NUSounds.h"

PlayingState::PlayingState(SoccerProvider* provider) : SoccerFSMState(provider)
{
    m_chase_state = new ChaseState(this);
    m_positioning_state = new PositioningState(this);
    m_ball_is_lost_state = new BallIsLostState(this);
    m_im_lost_state = new ImLostState(this);
    
    m_state = m_chase_state;
}

PlayingState::~PlayingState()
{
    delete m_chase_state;
    delete m_positioning_state;
    delete m_ball_is_lost_state;
    delete m_im_lost_state;
}

void PlayingState::doStateCommons()
{
    if (m_provider->stateChanged())
    {   // play a sound when we enter the playing state, turn the kick off light off
        m_actions->addSound(m_actions->CurrentTime, NUSounds::PLAYING);
        m_actions->addLeds(NUActionatorsData::RightFootLeds, m_actions->CurrentTime, 0, 0, 0);
    }
    // In playing the chest led should be green
    m_actions->addLeds(NUActionatorsData::ChestLeds, m_actions->CurrentTime, 0.1, 1, 0.1);
}

BehaviourFSMState* PlayingState::nextStateCommons()
{   // do state transitions in playing state machine
    if (m_field_objects->mobileFieldObjects[FieldObjects::FO_BALL].isLost())
        m_state = m_ball_is_lost_state;
    else if (m_team_info->amIClosestToBall())
        m_state = m_chase_state;
    else if (m_field_objects->self.isLost())
        m_state = m_im_lost_state;
    else
        m_state = m_positioning_state;
}

BehaviourFSMState* PlayingState::nextState()
{   // the playing state machine can never modify the game state machine
    return this;
}




