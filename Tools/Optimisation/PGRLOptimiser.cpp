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

#include "NUPlatform/NUPlatform.h"

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
    m_step_size = 0.01;        	   // Tune this
    m_epsilon = 0.01;              // Tune this
    m_num_particles = 12;      	   // Tune this
    m_stalled_threshold = 25;	   // Might need to tune this

    m_selected_fitness = 1;
    m_best_fitness = 0;
    m_stall_count = 0;

    load();
    if (m_current_parameters.empty())
    {
        m_current_parameters = parameters;
        generatePolicies();
    }
    save();
}

/*! @brief Destructor for the abstract optimiser */
PGRLOptimiser::~PGRLOptimiser()
{
}

void PGRLOptimiser::setParametersResult(const vector<float>& fitness)
{
	float selectedfitness = fitness[m_selected_fitness];
	setParametersResult(selectedfitness);

	if (selectedfitness <= m_best_fitness)
		m_stall_count++;
	else
	{
		m_stall_count = 0;
		m_best_fitness = selectedfitness;
	}

	if (m_stall_count >= m_stalled_threshold and m_random_policies_index == 0)
	{	// we want to switch because we have stalled, and we are able to switch because we just completed a gradient estimation
		m_best_fitness = 0;
		m_stall_count = 0;
		m_selected_fitness = (m_selected_fitness+1)%fitness.size();
	}
	debug << "PGRLOptimiser::setParametersResult(" << fitness << "). Using fitness " << m_selected_fitness << ". Stalled for " << m_stall_count << endl;
}

void PGRLOptimiser::setParametersResult(float fitness)
{
    m_fitnesses.push_back(fitness);
    m_random_policies_index++;
    if ((unsigned int) m_random_policies_index == m_random_policies.size())
    {
        m_current_parameters += calculateStep();
        generatePolicies();
    }
}

vector<float> PGRLOptimiser::getNextParameters()
{
    return m_random_policies[m_random_policies_index];
}

/*! @brief Generates a set of policies from the seed to estimate the gradient
 */
void PGRLOptimiser::generatePolicies()
{
	generateShuffledPolices(m_current_parameters);
}

/*! @brief Calculates a step in the direction of the gradient estimated through the polices selected
 *	@return the step (ie. the new parameters should be parameters += step)
 */
vector<float> PGRLOptimiser::calculateStep()
{
    int num_dim = m_current_parameters.size();
    vector<float> A(num_dim);

    // calculate the averages for negative, zero, and positive epsilons
    for (int i=0; i<num_dim; i++)
    {
        float average_minus = 0;
        float average_zero = 0;
        float average_plus = 0;
        accumulator_set<float, stats<tag::mean> > accum_minus(0);
        accumulator_set<float, stats<tag::mean> > accum_zero(0);
        accumulator_set<float, stats<tag::mean> > accum_plus(0);
        for (int j=0; j<m_random_policies.size(); j++)
        {
            if (m_random_policies[j][i] < m_current_parameters[i].get())
                accum_minus(m_fitnesses[j]);
            else if (m_random_policies[j][i] > m_current_parameters[i].get())
                accum_plus(m_fitnesses[j]);
            else
                accum_zero(m_fitnesses[j]);
        }

        if (extract::count(accum_minus) == 0 or extract::count(accum_zero) == 0 or extract::count(accum_plus) == 0)
        {	// if by chance we get a set where one of the three possibilities has no entries, take no step in that dimension
            A[i] = 0;
        }
        else
        {
            average_minus = mean(accum_minus);
            average_zero = mean(accum_zero);
            average_plus = mean(accum_plus);

            if (average_zero > average_plus and average_zero > average_minus)
                A[i] = 0;
            else
                A[i] = m_step_size*tanh(average_plus - average_minus)*(m_current_parameters[i].max() - m_current_parameters[i].min());
        }
    }
    return A;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------- Random Policies

/*! @brief Generates m_num_particles random policies from the seed.
 	@param seed the seed set of parameters used to generate the random policies
 */
void PGRLOptimiser::generateRandomPolices(const vector<Parameter>& seed)
{
    m_random_policies_index = 0;
    m_fitnesses.clear();
    m_random_policies.clear();
    for (int i=0; i<m_num_particles; i++)
        m_random_policies.push_back(generateRandomPolicy(seed));

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
        float value = seed[i] + getRandomDirection()*epslion;
        if (value < seed[i].min())
            value = seed[i].min();
        else if (value > seed[i].max())
            value = seed[i].max();
        newpolicy.push_back(value);
    }
    return newpolicy;
}

/*! @brief Returns -1,0,1 to be used as the random multiplier of epsilon to give random directions
 */
int PGRLOptimiser::getRandomDirection()
{
    return rand()%3 - 1;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------- Shuffled Policies

/*! @brief Generates a set of random policies from the seed
 *	@param seed the set of parameters which is used as a base to generate the random policies
 */
void PGRLOptimiser::generateShuffledPolices(const vector<Parameter>& seed)
{
	m_random_policies_index = 0;
	m_fitnesses.clear();

	vector<float> floatseed(seed.size());
	for (size_t i=0; i<seed.size(); i++)
		floatseed[i] = seed[i].get();
	m_random_policies = vector<vector<float> >(3*(m_num_particles/3), floatseed);

	for (size_t i=0; i<seed.size(); i++)
	{
		vector<float> temp = generateSigns();
		float epsilon = m_epsilon*(seed[i].max() - seed[i].min());
		for (size_t j=0; j<temp.size(); j++)
			m_random_policies[j][i] += epsilon*temp[j];
	}

	for (size_t i=0; i<m_random_policies.size(); i++)
		debug << m_random_policies[i] << endl;
}

/*! @brief Generates a vector of signs to be used when generating the random policies.
 *
 *         The vector will have a length that is a multiple of 3, and contain an equal number of (+1,-1,0).
 *         The place of the (+1,-1,0) will be random. The idea being to try an avoid the case where few particles
 *         are given epsilon's with one of the signs.
 *  @return the vector
 */
vector<float> PGRLOptimiser::generateSigns()
{
	int num_of_each = m_num_particles/3;
	size_t size = 3*num_of_each;				// round the number of particles to multiple of 3
	vector<float> signs(size);
	for (size_t i=0; i<num_of_each; i++)
	{
		signs[3*i] = 1;
		signs[3*i+1] = 0;
		signs[3*i+2] = -1;
	}
	random_shuffle(signs.begin(), signs.end());
	return signs;
}

// ---------------------------------------------------------------------------------------------------------------------------------------------- Opponent Policies

/*! @brief Generates m_num_particles random policies from the seed.
 	@param seed the seed set of parameters used to generate the random policies
 */
void PGRLOptimiser::generateOpponentPolicies(const vector<Parameter>& seed)
{
    m_random_policies_index = 0;
    m_fitnesses.clear();
    m_random_policies.clear();

    vector<float> policy, opponent;
    for (int i=0; i<m_num_particles/2; i++)
    {
    	generateOpponents(seed, policy, opponent);
        m_random_policies.push_back(policy);
    	m_random_policies.push_back(opponent);
    }

    for (size_t i=0; i<m_random_policies.size(); i++)
        debug << m_random_policies[i] << endl;
}

/*! @brief Generates a pair of policies (policy, opponent) near seed to estimate the gradient
 * 	@param seed the parameter set where we are estimating the derivative
 * 	@param policy the first policy near seed
 * 	@param opponent the 'opposite' policy near seed (ie this has the epsilon of opposite sign to policy)
 */
void PGRLOptimiser::generateOpponents(const vector<Parameter>& seed, vector<float>& policy, vector<float>& opponent)
{
	policy.clear();
	opponent.clear();
	for (size_t i=0; i<seed.size(); i++)
	{
		float epslion = m_epsilon*(seed[i].max() - seed[i].min());
		float direction = getRandomDirection();
		policy.push_back(seed[i] + direction*epslion);
		opponent.push_back(seed[i] - direction*epslion);
	}
}

void PGRLOptimiser::summaryTo(ostream& stream)
{
}

void PGRLOptimiser::toStream(ostream& o) const
{
    o << m_step_size << " " << m_epsilon << " " << m_num_particles << " " << m_stalled_threshold << endl;
    
    o << m_random_policies_index << endl;
    o << m_current_parameters << endl;
    o << m_random_policies << endl;
    o << m_fitnesses << endl;

    o << m_selected_fitness << " " << m_best_fitness << " " << m_stall_count << endl;
}

void PGRLOptimiser::fromStream(istream& i)
{
    i >> m_step_size;
    i >> m_epsilon;
    i >> m_num_particles;
    i >> m_stalled_threshold;
    
    i >> m_random_policies_index;
    i >> m_current_parameters;
    i >> m_random_policies;
    i >> m_fitnesses;

    i >> m_selected_fitness;
    i >> m_best_fitness;
    i >> m_stall_count;
}

