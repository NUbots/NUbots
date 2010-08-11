/*! @file EvaluateWalkParametersState.cpp
    @brief A state to evaluate a set of walk parameters

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

#include "EvaluateWalkParametersState.h"

#include "WalkOptimisationProvider.h"
#include "EvaluateSpeedOfWalkParametersState.h"
#include "EvaluateStabilityOfWalkParametersState.h"

#include "NUPlatform/NUSensors/NUSensorsData.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

/*! @brief Construct a evaluate walk parameters state
    @param parent the walk optimisation provider
 */
EvaluateWalkParametersState::EvaluateWalkParametersState(WalkOptimisationProvider* parent) : m_parent(parent)
{
    m_evaluate_speed = new EvaluateSpeedOfWalkParametersState(this);
    m_evaluate_stability = new EvaluateStabilityOfWalkParametersState(this);
    
    m_state = m_evaluate_speed;
    m_speed_evaluation_completed = false;
}

/*! @brief Destroy the evaluate walk parameters state */
EvaluateWalkParametersState::~EvaluateWalkParametersState()
{
    delete m_evaluate_speed;
    m_evaluate_speed = 0;
    delete m_evaluate_stability;
    m_evaluate_stability = 0;
};

/*! @brief Returns the desired next state in the walk optimisation provider */
BehaviourState* EvaluateWalkParametersState::nextState()
{
    if (m_data->isFallen())// or speedEvaluationFinished())		// TODO: Put in a compile flag so I can turn the stability test on and off here
        return m_parent->m_generate;
    else
        return this;
}

/*! @brief Returns the desired next state in the evaluate walk parameters state machine */
BehaviourState* EvaluateWalkParametersState::nextStateCommons()
{   
    if (m_parent->stateChanged())
        return m_evaluate_speed;
    else
        return m_state;
}

/*! @brief Returns true if the speed evalution has completed. This is a hack to shortcut the stability measurement when I don't need it */
bool EvaluateWalkParametersState::speedEvaluationFinished()
{
    if (m_speed_evaluation_completed)
    {
        m_speed_evaluation_completed = false;
        return true;
    }
    else
        return false;
}

/*! @brief Marks the speed evaluation as being complete. This is a hack to shortcut the stability measurement when I don't need it */
void EvaluateWalkParametersState::markSpeedEvaluationCompleted()
{
    m_speed_evaluation_completed = true;
}

