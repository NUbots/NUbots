/*
 *  optimiser.cpp
 *  walkoptimiser
 *
 *  Created by jason on 7/09/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "WalkOptimiser.h"
#include "boost/random.hpp"
#include <math.h>

#define OPTIMISER_VERBOSITY         4
#define OPTIMISER_ASSESS            0

WalkOptimiser::WalkOptimiser(const WalkParameters& walkparameters, bool minimise)
{
    m_best_parameters = walkparameters;
    m_best_delta_parameters = walkparameters;
    for (int i=0; i<m_best_delta_parameters.size(); i++)
        m_best_delta_parameters[i] = 0;
    m_current_parameters = walkparameters;
    
    m_minimise = minimise;
    m_best_performance = 0;
    m_alpha = 0.0;
    m_count_since_last_improvement = 0;
    m_reset_limit = 10;
    
    m_improvement = 0;
    m_previous_improvement = 10000;
}

WalkOptimiser::~WalkOptimiser()
{
}

void WalkOptimiser::tick(float performance, WalkParameters& nextparameters)
{
    if (m_minimise == true && performance < m_best_performance || m_minimise == false && performance > m_best_performance)
    {
        m_improvement = m_best_performance - performance;
        m_alpha = 0.9*fabs(tanh(fabs(m_improvement/m_previous_improvement)));
        m_best_parameters = m_current_parameters;
        m_best_performance = performance;
        m_count_since_last_improvement = 0;
    }
    getNewParameters(nextparameters);
}

/*! @brief Gets a new set of parameters to test
    @param walkparameters will be updated to contain the new parameters that we want to test
 */
void WalkOptimiser::getNewParameters(WalkParameters& walkparameters)
{
    m_count_since_last_improvement++;
    if (m_count_since_last_improvement > m_reset_limit)
    {
        m_alpha *= 0.9;
        m_count_since_last_improvement = 0;
        if (m_minimise)
            m_best_performance *= 1.03;
        else
            m_best_performance *= 0.97;
    }
    mutateBestParameters(walkparameters);
    m_current_parameters = walkparameters;
}

/*! @brief Gets a new set of parameters to test based on the current best parameters
    @param walkparameters will be updated to contain the new paramters that we want to test
 */
void WalkOptimiser::mutateBestParameters(WalkParameters& walkparameters)
{
    mutateParameters(m_best_parameters, m_best_delta_parameters, walkparameters);
}

/*! Generates a new set of parameters to be tested based on base_parameters and basedelta_parameters
 */
void WalkOptimiser::mutateParameters(WalkParameters& base_parameters, WalkParameters& basedelta_parameters, WalkParameters& walkparameters)
{
    // generate phi to mutate the BestParameters
    float sigma = 0.06*exp(m_count_since_last_improvement/m_reset_limit - 1);
    vector<float> phi(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        phi[i] = normalDistribution(1, sigma);
    
    // mutate the BestParameters
    vector<float> mutant(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        mutant[i] = base_parameters[i] * phi[i];        // TODO: Add in C

    // calculate the difference between the mutated state and the best one
    vector<float> deltamutant(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        deltamutant[i] = base_parameters[i] - mutant[i];
    
    // calculate the desired change in parameters
    vector<float> deltaparameters(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        deltaparameters[i] = m_alpha*basedelta_parameters[i] + (1-m_alpha)*deltamutant[i];
    
    // now calculate the new parameters themselves
    vector<float> newparameters(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        newparameters[i] = base_parameters[i] + deltaparameters[i];
    
    // now copy the new parameters into the storage variable
    for (int i=0; i<base_parameters.size(); i++)
        walkparameters[i] = newparameters[i];
}


/* Returns a normal random variable from the normal distribution with mean and sigma
 */
float WalkOptimiser::normalDistribution(float mean, float sigma)
{
    static boost::mt19937 generator(1);
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution

    return x;
}
