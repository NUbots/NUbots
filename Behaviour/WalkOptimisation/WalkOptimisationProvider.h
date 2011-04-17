/*! @file WalkOptimisationProvider.h
    @brief Declaration of walk optimisation behaviour for testing and demonstration purposes 
 
    @class WalkOptimisationProvider
    @brief A walk optimisation behaviour provider

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

#ifndef WALKOPTIMISATIONBEHAVIOUR_H
#define WALKOPTIMISATIONBEHAVIOUR_H

#include "Behaviour/BehaviourFSMProvider.h"
class WalkOptimisationState;
class Optimiser;
#include "Motion/Walks/WalkParameters.h"

#include <vector>
#include <string>
#include <fstream>
using namespace std;

class WalkOptimisationProvider : public BehaviourFSMProvider
{
public:
    WalkOptimisationProvider(Behaviour* manager);
    ~WalkOptimisationProvider();
    void tickOptimiser();
    
    float stoppingDistance();
protected:
    BehaviourState* nextStateCommons();
    void doBehaviourCommons();
private:
    void loadWayPoints();
    void loadId();
    void loadParameters(const string& name);
    void initOptimiser();
    float normalDistribution(float mean, float sigma);
public:
    WalkOptimisationState* m_generate;                 //!< the state in which the parameter generation is done, and preparations for its evaluation
    WalkOptimisationState* m_evaluate;                 //!< the state in which the parameter evaluation is done
    WalkOptimisationState* m_paused;                   //!< the optimisation process is paused in this state.
    
    vector<vector<float> > m_way_points;        //!< the way points over which to evaluate to speed and efficiency of the walk parameters
private:
    WalkParameters m_parameters;                //!< the current set of walk parameters
    string m_id;								//!< the id of the optimiser
    Optimiser* m_optimiser;                     //!< the optimiser itself
    
    float calculateFitness();					//!< calculates the fitness of the current parameters using the information from the two walk states
    vector<float> calculateFitnesses();			//!< calculates all of the fitnesses of the current parameters, so we can do multi-objective optimisation
    int m_iteration_count;						//!< the number of times the optimiser has been ticked
    int m_fall_count;                           //!< the number of times the optimiser has fallen
    
    ofstream m_log;
};


#endif

