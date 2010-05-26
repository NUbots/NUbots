/*! @file ReadyMarkState.cpp
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

#include "ReadyMarkState.h"

#include "Behaviour/Jobs/JobList.h"

ReadyMarkState::ReadyMarkState(SoccerFSMState* parent) : SoccerState(parent)
{
}

ReadyMarkState::~ReadyMarkState()
{
}

BehaviourState* ReadyMarkState::nextState()
{
    return this;
}

void ReadyMarkState::doState()
{
}

