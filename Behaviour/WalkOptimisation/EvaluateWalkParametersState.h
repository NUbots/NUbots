/*! @file EvaluateWalkParametersState.h
    @brief A state to evaluate a set of walk parameters
 
    @class EvaluateWalkParametersState
    @brief A walk optimisation state to evaluate the performance of a set of walk parameters.
           The speed, efficiency and stability of the walk is measured. 

    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#include "WalkOptimisationState.h"
class WalkOptimisationProvider;

class EvaluateWalkParametersState : public WalkOptimisationState
{
public:
    EvaluateWalkParametersState(WalkOptimisationProvider* parent);
    ~EvaluateWalkParametersState();

    BehaviourState* nextState();
	void doState();
	float distance();
private:
	vector<float> getStartPoint();
	bool pointReached();
	vector<float> getNextPoint();
	bool allPointsReached();
	vector<float> reversePoint(const vector<float>& point);
	float calculateCircuitLength();

	bool m_reverse_points;
	unsigned int m_current_point_index;
};


#endif

