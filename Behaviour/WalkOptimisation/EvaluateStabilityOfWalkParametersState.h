/*! @file EvaluateStabilityOfWalkParametersState.h
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

#ifndef EVALUATESTABILITYOFWALKPARAMETERSSTATE_H
#define EVALUATESTABILITYOFWALKPARAMETERSSTATE_H

#include "Behaviour/BehaviourFSMState.h"
class WalkOptimisationProvider;
class EvaluateWalkParametersState;

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <vector>
using namespace std;

// Evaluate Stability State Machine
class EvaluateStabilityOfWalkParametersState : public BehaviourFSMState
{
public:
    EvaluateStabilityOfWalkParametersState(EvaluateWalkParametersState* parent);
    ~EvaluateStabilityOfWalkParametersState();
    BehaviourState* nextStateCommons();
private:
    EvaluateWalkParametersState* m_parent;
    WalkOptimisationProvider* m_provider;
    vector<vector<float> > m_points;
    BehaviourState* m_init;
    BehaviourState* m_run;
    
    friend class EvaluateStabilityOfWalkParametersStartState;
    friend class EvaluateStabilityOfWalkParametersRunState;
};

// Initial go to start position state
class EvaluateStabilityOfWalkParametersStartState : public BehaviourState
{
public:
    EvaluateStabilityOfWalkParametersStartState(EvaluateStabilityOfWalkParametersState* parent);
    BehaviourState* nextState();
    void doState();
private:
    vector<float> getStartState();    
    void lookAtGoals();

    EvaluateStabilityOfWalkParametersState* m_parent;
    WalkOptimisationProvider* m_provider;
    
    vector<float> m_current_start_state;
};

// Actual state in which stability is measured
class EvaluateStabilityOfWalkParametersRunState : public BehaviourState
{
public:
    EvaluateStabilityOfWalkParametersRunState(EvaluateStabilityOfWalkParametersState* parent);
    BehaviourState* nextState();
    void doState();
private:
    void reset();
    void doPerturbation();
    void generatePerturbation();
    
    const float m_INITIAL_PERTURBATION_MAG;                 //!< The initial perturbation magnitude
    const float m_PERTURBATION_MAG_INC;                     //!< The increment applied to the perturbatio magnitude after each direction has been tested
    const int m_PERTURBATION_INTERVAL;                      //!< The number of steps between each perturbation
    const float m_PI;
    
    float m_perturbation_magnitude;
    float m_perturbation_direction;
    int m_step_count;
    int m_step_last_perturbed;
    int m_perturbation_count;
    float m_left_impact_time;
    float m_right_impact_time;
    
    EvaluateStabilityOfWalkParametersState* m_parent;
    WalkOptimisationProvider* m_provider;
    
};

#endif

