/*! @file EvaluateWalkParametersState.h
    @brief A state to evaluate a set of walk parameters
 
    @class EvaluateWalkParametersState
    @brief A walk optimisation state to evaluate the performance of a set of walk parameters.
           The speed, efficiency and stability of the walk is measured. 
           The speed and efficiency are measured simulataneously over a given path.
           The stability is measured separately.

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

#ifndef EVALUATEWALKPARAMETERSSTATE_H
#define EVALUATEWALKPARAMETERSSTATE_H

#include "Behaviour/BehaviourFSMState.h"
class WalkOptimisationProvider;
class EvaluateSpeedOfWalkParametersState;
class EvaluateStabilityOfWalkParametersState;

class EvaluateWalkParametersState : public BehaviourFSMState
{
public:
    EvaluateWalkParametersState(WalkOptimisationProvider* parent);
    ~EvaluateWalkParametersState();
private:
    BehaviourState* nextState();
    BehaviourState* nextStateCommons();
    bool speedEvaluationFinished();	
    void markSpeedEvaluationCompleted();
    
    WalkOptimisationProvider* m_parent;                 //!< the walk optimisation provider
    BehaviourState* m_evaluate_speed;                   //!< the speed evaluation state machine
    float m_speed;                                      //!< the measured speed over a trial
    float m_energy;                                     //!< the energy used over a trial
    BehaviourState* m_evaluate_stability;               //!< the stability evaluation state machine
    float m_stability;                                  //!< the stability over a trial
    
    bool m_speed_evaluation_completed;
    
    friend class EvaluateStabilityOfWalkParametersState;
    friend class EvaluateSpeedOfWalkParametersState; 
};


#endif

