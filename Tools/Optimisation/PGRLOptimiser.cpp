/*! @file PGRLOptimiser.cpp
    @brief Implemenation of PGRLOptimiser class
 
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

#include "PGRLOptimiser.h"
#include "Parameter.h"

#include <cstdlib>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
using namespace boost::accumulators;

#include "debug.h"
#include "nubotdataconfig.h"

/*! @brief Constructor for abstract optimiser
 	@param name the name of the optimiser. The name is used in debug logs, and is used for load/save filenames by default
 	@param parameters the initial seed for the optimisation
 */
PGRLOptimiser::PGRLOptimiser(std::string name, vector<Parameter> parameters) : Optimiser(name, parameters)
{
    m_min_step_size = 0.02;
    m_max_step_size = 0.05;			// 0.5 is too big, 0.2 might be OK to start with, 0.1 is too big 
    									// 0.05 is quite good. About 8.0 cm/s
    									// 0.025 is too small
    m_epsilon = 0.015;
    m_num_per_iteration = 10;
    
    srand(static_cast<unsigned int> (clock()*clock()*clock()));
    m_current_parameters = parameters;
    generateRandomPolices(m_current_parameters);
}

/*! @brief Destructor for the abstract optimiser */
PGRLOptimiser::~PGRLOptimiser()
{
}

void PGRLOptimiser::setParametersResult(float fitness)
{
    m_fitnesses.push_back(fitness);
    m_random_policies_index++;
    if (m_random_policies_index == m_random_policies.size())
    {
        debug << "fitnesses: " << m_fitnesses << endl;
        m_current_parameters += calculateStep();
        generateRandomPolices(m_current_parameters);
    }
}

vector<float> PGRLOptimiser::getNextParameters()
{
    return m_random_policies[m_random_policies_index];
}

/*! @brief Generates m_num_per_iteration random policies from the seed.
 	@param seed the seed set of parameters used to generate the random policies
 */
void PGRLOptimiser::generateRandomPolices(const vector<Parameter>& seed)
{
    m_random_policies_index = 0;
    m_fitnesses.clear();
    m_random_policies.clear();
    for (size_t i=0; i<m_num_per_iteration; i++)
        m_random_policies.push_back(generateRandomPolicy(seed));
    
    debug << "policies: " << endl;
    for (size_t i=0; i<m_random_policies.size(); i++)
        debug << m_random_policies[i] << endl;
}

/*! @brief Generates a single random policy from the seed, and returns it
 	@param seed the random policy seed
 */
vector<float> PGRLOptimiser::generateRandomPolicy(const vector<Parameter>& seed)
{
    vector<float> newpolicy;
    newpolicy.reserve(seed.size());
    for (size_t i=0; i<seed.size(); i++)
    {
        float epslion = m_epsilon*(seed[i].max() - seed[i].min());
        newpolicy.push_back(seed[i] + getRandomDirection()*epslion);
    }
    return newpolicy;
}

/*! @brief Returns -1,0,1 to be used as the random multipler of epsilon to give random directions
 */
int PGRLOptimiser::getRandomDirection()
{
    return rand()%3 - 1;
}

vector<float> PGRLOptimiser::calculateStep()
{
    int num_dim = m_current_parameters.size();
    vector<float> A(num_dim);
    
    // calculate the averages for negative, zero, and postive epsilons
    for (int i=0; i<num_dim; i++)
    {
        float average_minus = 0;
        float average_zero = 0;
        float average_plus = 0;
        accumulator_set<float, stats<tag::mean> > accum_minus(0);
        accumulator_set<float, stats<tag::mean> > accum_zero(0);
        accumulator_set<float, stats<tag::mean> > accum_plus(0);
        for (int j=0; j<m_num_per_iteration; j++)
        {
            if (m_random_policies[j][i] < m_current_parameters[i].get())
                accum_minus(m_fitnesses[j]);
            else if (m_random_policies[j][i] > m_current_parameters[i].get()) 
                accum_plus(m_fitnesses[j]);
            else
                accum_zero(m_fitnesses[j]);
        }
     	
        if (extract::count(accum_minus) == 0 or extract::count(accum_zero) == 0 or extract::count(accum_plus) == 0)
        {	// if by chance we get a set where one of the three possibilities has no entries, take no step in that direction
            A[i] = 0;
        }
        else
        {
            average_minus = mean(accum_minus);
            average_zero = mean(accum_zero);
            average_plus = mean(accum_plus);
            
            debug << "dim:" << i << " minus:" << average_minus << " average_zero:" << average_zero << " average_plus:" << average_plus << endl;
            
            if (average_zero > average_plus and average_zero > average_minus)
                A[i] = 0;
            else
                A[i] = m_max_step_size*tanh(average_plus - average_minus)*(m_current_parameters[i].max() - m_current_parameters[i].min());
        }
    }
    debug << A << endl;
}

void PGRLOptimiser::summaryTo(ostream& stream)
{
}

void PGRLOptimiser::toStream(ostream& o) const
{
}

void PGRLOptimiser::fromStream(istream& i)
{
}

