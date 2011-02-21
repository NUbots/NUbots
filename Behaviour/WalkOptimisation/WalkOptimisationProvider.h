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
    void setDuration(float time);
    void setSpeed(float speed);
    void setEnergy(float energy);
    void setStability(float stability);
    
    float stoppingDistance();
    float normalDistribution(float mean, float sigma);
protected:
    BehaviourState* nextStateCommons();
    void doBehaviourCommons();
public:
    BehaviourState* m_generate;                 //!< the state in which the parameter generation is done, and preparations for its evaluation
    BehaviourState* m_evaluate;                 //!< the state in which the parameter evaluation is done
    BehaviourState* m_paused;                   //!< the optimisation process is paused in this state.
    
    vector<vector<float> > m_speed_points;      //!< the way points over which to evaluate to speed and efficiency of the walk parameters
    vector<vector<float> > m_stability_points;  //!< the way points over which to evaluate the stability of the walk parameters
private:
    WalkParameters m_parameters;                //!< the current set of walk parameters
    Optimiser* m_optimiser;                     //!< the optimiser itself
    
    float calculateFitness();					//!< calculates the fitness of the current parameters from m_duration, m_speed, m_energy, and m_stability
    float calculatePathDistance();				//!< calculates the distance of the speed evaluation path
    int m_iteration_count;						//!< the number of times the optimiser has been ticked
    int m_fall_count;                           //!< the number of times the optimiser has fallen
    float m_duration;							//!< the evaluation time in ms
    float m_energy;								//!< the energy used during evalution of walk parameters in J
    float m_stability;							//!< the stability of the walk parameters
    
    ofstream m_log;
};


#endif

