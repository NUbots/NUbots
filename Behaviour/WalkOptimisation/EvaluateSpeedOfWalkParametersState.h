/*! @file EvaluateSpeedOfWalkParameters.h
    @brief A state to evaluate the speed and the efficiency of a set of walk parameters
 
    @class EvaluateSpeedOfWalkParameters
    @brief A walk optimisation state where the speed and efficiency of a set of walk parameters
           are measured over a predefined path
 
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

#ifndef EVALUATESPEEDOFWALKPARAMETERSSTATE_H
#define EVALUATESPEEDOFWALKPARAMETERSSTATE_H

#include "Behaviour/BehaviourState.h"
class WalkOptimisationProvider;
class EvaluateWalkParametersState;

#include <vector>
using namespace std;

class EvaluateSpeedOfWalkParametersState : public BehaviourState
{
public:
    EvaluateSpeedOfWalkParametersState(EvaluateWalkParametersState* parent);
    
    BehaviourState* nextState();
    void doState();
private:
    void startEvaluation();
    void tickEvaluation();
    void finishEvaluation();    
    void updateEnergy();
    
    vector<float> getStartPoint();
    bool pointReached();
    vector<float> getNextPoint();
    bool allPointsReached();

    void lookAtGoals();
private:
    EvaluateWalkParametersState* m_parent;
    WalkOptimisationProvider* m_provider;
    vector<vector<float> > m_points;
    
    vector<float> m_current_target_state;
    bool m_reverse_points;
    unsigned int m_current_point_index;
    
    double m_trial_start_time;
    float m_energy_used;
    double m_previous_time;
    vector<float> m_previous_positions;
};

#endif

