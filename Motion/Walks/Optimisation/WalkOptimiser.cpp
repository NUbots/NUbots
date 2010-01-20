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
    if (m_minimise)
        m_best_performance = 100;
    else
        m_best_performance = 0;
    m_alpha = 0.0;
    m_count_since_last_improvement = 0;
    m_reset_limit = 10;
    
    m_improvement = 1e100;
    m_previous_improvement = 1e100;
}

WalkOptimiser::~WalkOptimiser()
{
}

void WalkOptimiser::tick(float performance, WalkParameters& nextparameters)
{
    if (m_minimise == true && performance < m_best_performance || m_minimise == false && performance > m_best_performance)
    {
        cout << "Improvement!" << endl;
        m_previous_improvement = m_improvement;
        m_improvement = m_best_performance - performance;
        m_alpha = 0.9*fabs(tanh(fabs(m_improvement/m_previous_improvement)));
        for (int i=0; i<m_best_parameters.size(); i++)
            m_best_delta_parameters[i] = m_current_parameters[i] - m_best_parameters[i];
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

/*! @brief Generates a new set of parameters to be tested based on base_parameters and basedelta_parameters
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


/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
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

/*! @brief Prints a human readable summary of the optimiser's state.
    The summary includes the current best performance, and the best set of walk parameters
 */
void WalkOptimiser::summaryTo(ostream& output)
{
    output << "WalkOptimiser Performance: " << m_best_performance << " with ";
    m_best_parameters.summaryTo(output);
}

void WalkOptimiser::csvTo(ostream& output)
{
}

/*! @brief Stores the entire contents of the WalkOptimiser in the stream
 */
ostream& operator<< (ostream& output, const WalkOptimiser& p)
{
    output << p.m_best_parameters;
    output << p.m_best_delta_parameters;   
    output << p.m_current_parameters;      
    output.write((char*) &p.m_minimise, sizeof(bool));
    output.write((char*) &p.m_best_performance, sizeof(float));
    output.write((char*) &p.m_alpha, sizeof(float));
    output.write((char*) &p.m_reset_limit, sizeof(int));
    output.write((char*) &p.m_count_since_last_improvement, sizeof(int));
    output.write((char*) &p.m_improvement, sizeof(float));
    output.write((char*) &p.m_previous_improvement, sizeof(float));
    
    return output;
}

/*! @brief Retrieves a stored WalkOptimiser from a stream
 */
istream& operator>> (istream& input, WalkOptimiser& p)
{
    input >> p.m_best_parameters;
    input >> p.m_best_delta_parameters;
    input >> p.m_current_parameters;
    char inbuffer[10];
    input.read(inbuffer, sizeof(bool));
    p.m_minimise = *((bool*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_best_performance = *((float*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_alpha = *((float*) inbuffer);
    
    input.read(inbuffer, sizeof(int));
    p.m_reset_limit = *((int*) inbuffer);
    
    input.read(inbuffer, sizeof(int));
    p.m_count_since_last_improvement = *((int*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_improvement = *((float*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_previous_improvement = *((float*) inbuffer);
    
    return input;
}
