/*! @file ChaseBallBehaviourState.cpp
    @brief Implementation of the wrapper state for the chase ball provider (ie the field-less demo behaviour)


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

#include "ChaseBallBehaviourState.h"
#include "Behaviour/ChaseBall/ChaseBallProvider.h"

#include "Behaviour/Jobs/JobList.h"

ChaseBallBehaviourState::ChaseBallBehaviourState(SoccerFSMState* parent) : SoccerState(parent)
{
    m_chase_provider = new ChaseBallProvider(NULL, false);
}

ChaseBallBehaviourState::~ChaseBallBehaviourState()
{
    delete m_chase_provider;
}

BehaviourState* ChaseBallBehaviourState::nextState()
{
    return this;
}

void ChaseBallBehaviourState::doState()
{
    m_chase_provider->process(m_jobs, m_data, m_actions, m_field_objects, m_game_info, m_team_info);
}

