/*! @file WalkOptimisationProvider.cpp
    @brief Implementation of chase ball behaviour class

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

#include "WalkOptimisationProvider.h"
#include "GenerateWalkParametersState.h"
#include "EvaluateWalkParametersState.h"
#include "PausedWalkOptimisationState.h"
#include "Tools/Optimisation/Optimiser.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

using namespace std;

WalkOptimisationProvider::WalkOptimisationProvider(Behaviour* manager) : BehaviourFSMProvider(manager)
{
    m_generate = new GenerateWalkParametersState(this);
    m_evaluate = new EvaluateWalkParametersState(this);
    m_paused = new PausedWalkOptimisationState(this);
    
    m_state = m_paused;
    m_optimiser = new Optimiser("Test");
}

WalkOptimisationProvider::~WalkOptimisationProvider()
{
    delete m_generate;
    delete m_evaluate;
    delete m_paused;
    
    delete m_optimiser;
}

BehaviourState* WalkOptimisationProvider::nextStateCommons()
{
    while (m_game_info->getCurrentState() != GameInformation::PlayingState)
        m_game_info->doManualStateChange();
    
    if (singleChestClick() or longChestClick())
    {
        if (m_state == m_paused)
            return m_generate;
        else
            return m_paused;
    }
    else
        return m_state;
}


