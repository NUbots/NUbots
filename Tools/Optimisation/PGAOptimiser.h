/*! @file PGRLOptimiser.h
    @brief Declaration of the PGRL optimiser
 
    @class PGRLOptimiser
    @brief An implementation of the Policy Gradient Reinforcement Learning algorithm
 
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

#ifndef PGA_OPTIMISER_H
#define PGA_OPTIMISER_H

#include "Optimiser.h"

#include <string>
#include <vector>
#include <iostream>
using namespace std;

class PGAOptimiser : public Optimiser
{
public:
    PGAOptimiser(string name, vector<Parameter> parameters);
    ~PGAOptimiser();
    
    vector<float> getNextParameters();
    void setParametersResult(const vector<float>& fitness);
    void setParametersResult(float fitness);
    
    void summaryTo(ostream& stream);
private:
    void generatePolicies();
    vector<float> calculateStep();

    void generateRandomPolices(const vector<Parameter>& seed);
    vector<float> generateRandomPolicy(const vector<Parameter>& seed);
    int getRandomDirection();
    
    void generateShuffledPolices(const vector<Parameter>& seed);
    vector<float> generateSigns();

    void generateOpponentPolicies(const vector<Parameter>& seed);
    void generateOpponents(const vector<Parameter>& seed, vector<float>& policy, vector<float>& opponent);

    void toStream(ostream& o) const;
    void fromStream(istream& i);
private:
    float m_step_size;							//!< the maxiumum step size
    float m_epsilon;							//!< the small step size used to estimate the gradient
    int m_num_particles;						//!< the number of random samples used to estimate the gradient
    int m_stalled_threshold;					//!< the number of iterations before the selected fitness is switched
    
    int m_random_policies_index;				//!< the index into m_random_policies of the parameters currently under evaluation
    vector<Parameter> m_current_parameters;		//!< the current set of parameters, that is the current set at which point we are estimating the gradient
    vector<vector<float> > m_random_policies;	//!< the m_num_particles of randomly generated polcies use to evaluate the gradient
    vector<float> m_fitnesses;					//!< the corresponding fitnesses of m_random_policies

    int m_selected_fitness;
    float m_best_fitness;
    int m_stall_count;
};

#endif

