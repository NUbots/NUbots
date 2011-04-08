/*! @file EHCLSOptimiser.cpp
    @brief Implemenation of EHCLSOptimiser class
 
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

#include "EHCLSOptimiser.h"
#include "Parameter.h"

#include "NUPlatform/NUPlatform.h"

#include <boost/random.hpp>

#include "debug.h"

/*!
 */
EHCLSOptimiser::EHCLSOptimiser(std::string name, vector<Parameter> parameters) : Optimiser(name, parameters)
{
    m_best_parameters = parameters;
    m_best_delta_parameters = vector<float>(m_best_parameters.size(),0);

    m_current_parameters = parameters;
    m_previous_parameters = parameters;
    m_real_best_parameters = parameters;
    
    m_iteration_count = 0;
    m_best_performance = 0;
    m_real_best_performance = 0;
    m_alpha = 0.0;
    m_count_since_last_improvement = 0;
    
    m_improvement = 1e100;
    m_previous_improvement = 1e100;
    
    // for NBWalk in Webots 0.125, 5, 0.98 is the best combination of parameters
    // The algorithm appears to work best when the reset limit is low, this means we only search
    // along the direction of the last improvement for a short time. The reset fraction is high
    // so we don't back track to far     
    m_neta = 0.1;               // tune this parameter
    m_reset_limit = 5;            // tune this parameter
    m_reset_fraction = 0.995;      // tune this parameter
    
    load();
    save();
}

EHCLSOptimiser::~EHCLSOptimiser()
{
}

vector<float> EHCLSOptimiser::getNextParameters()
{
    m_previous_parameters = m_current_parameters;
    mutateBestParameters(m_current_parameters);
    return Parameter::getAsVector(m_current_parameters);
}

void EHCLSOptimiser::setParametersResult(float fitness)
{
    m_iteration_count++;
    m_current_performance = fitness;
    if (m_current_performance > m_best_performance)
    {
        m_previous_improvement = m_improvement;
        m_improvement = m_current_performance - m_best_performance;
        m_alpha = 0.95*fabs(tanh(fabs(m_improvement/m_previous_improvement)));
        m_best_delta_parameters = m_current_parameters - m_best_parameters;
        
        m_best_parameters = m_current_parameters;
        m_best_performance = m_current_performance;
        m_count_since_last_improvement = 0;
        
        debug << "Improvement: " << m_improvement << " alpha: " << m_alpha << endl;
    }
    else
        m_count_since_last_improvement++;
        
    if (m_current_performance > m_real_best_performance)
    {   // check if it is the best set of parameters I have ever seen
        m_real_best_parameters = m_current_parameters;
        m_real_best_performance = m_current_performance;
    }
    
    if (m_count_since_last_improvement > m_reset_limit)
    {	// check if we need to reset
        m_alpha = 0;
        m_count_since_last_improvement = 0;
        m_best_performance *= m_reset_fraction;
    }
}

/*! @brief Gets a new set of parameters to test based on the current best parameters
 @param walkparameters will be updated to contain the new paramters that we want to test
 */
void EHCLSOptimiser::mutateBestParameters(vector<Parameter>& parameters)
{
    mutateParameters(m_best_parameters, m_best_delta_parameters, parameters);
}

/*! @brief Generates a new set of parameters to be tested based on base_parameters and basedelta_parameters
 */
void EHCLSOptimiser::mutateParameters(vector<Parameter>& base_parameters, vector<float>& basedelta_parameters, vector<Parameter>& parameters)
{
    // generate phi to mutate the BestParameters
    float sigma = m_neta*exp(m_count_since_last_improvement/m_reset_limit - 1);			// 0.04
    vector<float> phi(base_parameters.size(), 0);
    for (size_t i=0; i<base_parameters.size(); i++)
        phi[i] = normalDistribution(0, sigma);
    
    // mutate the BestParameters
    vector<float> mutant;
    mutant.reserve(base_parameters.size());
    for (size_t i=0; i<base_parameters.size(); i++)
        mutant.push_back(base_parameters[i] + phi[i]*(base_parameters[i].max() - base_parameters[i].min()));
    
    // calculate the difference between the mutated state and the best one
    vector<float> deltamutant = mutant - base_parameters;
    
    // calculate the desired change in parameters
    vector<float> deltaparameters = m_alpha*basedelta_parameters + (1-m_alpha)*deltamutant;
    
    // now calculate the new parameters themselves
    parameters.resize(base_parameters.size());
    for (size_t i=0; i<base_parameters.size(); i++)
        parameters[i].set(base_parameters[i] + deltaparameters[i]);
}

/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 */
float EHCLSOptimiser::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = 1e6*Platform->getRealTime()*Platform->getRealTime()*Platform->getRealTime();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution
    
    return x;
}

void EHCLSOptimiser::summaryTo(ostream& stream)
{
    debug << "EHCLSOptimiserSummary" << endl;
}


void EHCLSOptimiser::toStream(ostream& o) const
{
    o << m_best_parameters << endl;
    o << m_best_delta_parameters << endl;
    o << m_current_parameters << endl;
    o << m_previous_parameters << endl;
    o << m_real_best_parameters << endl;
    
    o << m_iteration_count << " " << m_count_since_last_improvement << " " << m_alpha << " " << m_improvement << " " << m_previous_improvement << " " << m_neta << " " << m_reset_limit << " " << m_reset_fraction << endl;
    o << m_current_performance << " " << m_best_performance << endl;
}

void EHCLSOptimiser::fromStream(istream& i)
{
    i >> m_best_parameters;
    i >> m_best_delta_parameters;
    i >> m_current_parameters;
    i >> m_previous_parameters;
    i >> m_real_best_parameters;
    
    i >> m_iteration_count >> m_count_since_last_improvement >> m_alpha >> m_improvement >> m_previous_improvement >> m_neta >> m_reset_limit >> m_reset_fraction;
    i >> m_current_performance >> m_best_performance;
}

