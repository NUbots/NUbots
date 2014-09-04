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
#include <time.h>
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
    m_previous_parameters = m_current_parameters;
    m_real_best_parameters = walkparameters;
    
    m_iteration_count = 0;
    m_minimise = minimise;
    if (m_minimise)
        m_best_performance = 100;
    else
        m_best_performance = 0;
    m_real_best_performance = m_best_performance;
    m_alpha = 0.0;
    m_count_since_last_improvement = 0;
    m_reset_limit = 10;
    
    m_improvement = 1e100;
    m_previous_improvement = 1e100;
}

WalkOptimiser::~WalkOptimiser()
{
}

/*! @brief Ticks the optimisation process
    @param performance the performance of the current set of parameters
    @param nextparameters will be updated with the next set of parameters to test
 */
void WalkOptimiser::tick(float performance, WalkParameters& nextparameters)
{
    m_iteration_count++;
    m_current_performance = performance;
    if (m_minimise == true && performance < m_best_performance || m_minimise == false && performance > m_best_performance)
    {
        m_previous_improvement = m_improvement;
        m_improvement = m_best_performance - performance;
        m_alpha = fabs(tanh(fabs(m_improvement/m_previous_improvement)));
        for (int i=0; i<m_best_parameters.size(); i++)
            m_best_delta_parameters[i] = m_current_parameters[i] - m_best_parameters[i];
        m_best_parameters = m_current_parameters;
        m_best_performance = performance;
        m_count_since_last_improvement = 0;
        std::cout << "Improvement. m_alpha: " << m_alpha << " with: "; 
        m_best_parameters.summaryTo(std::cout);
        std::cout << "Delta:";
        m_best_delta_parameters.summaryTo(std::cout);
    }
    if (m_minimise == true && performance < m_real_best_performance || m_minimise == false && performance > m_real_best_performance)
    {   // check if it is the best set of parameters I have ever seen!
        m_real_best_parameters = m_current_parameters;
        m_real_best_performance = performance;
    }
    getNewParameters(nextparameters);
}

/*! @brief Gets a new set of parameters to test
    @param walkparameters will be updated to contain the new parameters that we want to test
 */
void WalkOptimiser::getNewParameters(WalkParameters& walkparameters)
{
    m_count_since_last_improvement++;
    m_previous_parameters = m_current_parameters;
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
    float sigma = 0.1*exp(m_count_since_last_improvement/m_reset_limit - 1);
    std::vector<float> phi(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        phi[i] = normalDistribution(1, sigma);
    
    // mutate the BestParameters
    std::vector<float> mutant(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        mutant[i] = base_parameters[i] * phi[i];        // TODO: Add in C

    // calculate the difference between the mutated state and the best one
    std::vector<float> deltamutant(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        deltamutant[i] = base_parameters[i] - mutant[i];
    
    // calculate the desired change in parameters
    std::vector<float> deltaparameters(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        deltaparameters[i] = m_alpha*basedelta_parameters[i] + (1-m_alpha)*deltamutant[i];
    
    // now calculate the new parameters themselves
    std::vector<float> newparameters(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        newparameters[i] = base_parameters[i] + deltaparameters[i];
    
    // now copy the new parameters into the storage variable
    for (int i=0; i<base_parameters.size(); i++)
        walkparameters[i] = newparameters[i];
}

/*! @brief Returns the optimiser's iteration count
 */
int WalkOptimiser::getIterationCount()
{
    return m_iteration_count;
}

/*! @brief Returns the optimiser's current best performance
 */
float WalkOptimiser::getBestPerformance()
{
    return m_real_best_performance;
}


/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 */
float WalkOptimiser::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = clock()*clock()*clock();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution

    return x;
}

/*! @brief Prints a human readable summary of the optimiser's state.
    The summary includes the current best performance, and the best set of walk parameters
 */
void WalkOptimiser::summaryTo(std::ostream& output)
{
    output << "WalkOptimiser Performance: " << m_real_best_performance << " with ";
    m_real_best_parameters.summaryTo(output);
}

/*! @brief Prints a csv of iteration count and best performance
 */
void WalkOptimiser::csvTo(std::ostream& output)
{
    output << m_iteration_count << ", " << m_current_performance << ", ";
    m_previous_parameters.csvTo(output);        // there is a bit of a miss match depending on when this function is called.
                                                // In general, we print after we tick the optimiser, so we really want to print the
                                                // previous parameters because the current ones are still under test!
    output << std::endl;
}

/*! @brief Stores the entire contents of the WalkOptimiser in the stream
 */
std::ostream& operator<< (std::ostream& output, const WalkOptimiser& p)
{
    output << p.m_best_parameters;
    output << p.m_best_delta_parameters;   
    output << p.m_current_parameters;     
    output << p.m_real_best_parameters;
    output.write((char*) &p.m_iteration_count, sizeof(int));
    output.write((char*) &p.m_minimise, sizeof(bool));
    output.write((char*) &p.m_best_performance, sizeof(float));
    output.write((char*) &p.m_real_best_performance, sizeof(float));
    output.write((char*) &p.m_alpha, sizeof(float));
    output.write((char*) &p.m_reset_limit, sizeof(int));
    output.write((char*) &p.m_count_since_last_improvement, sizeof(int));
    output.write((char*) &p.m_improvement, sizeof(float));
    output.write((char*) &p.m_previous_improvement, sizeof(float));
    
    return output;
}

/*! @brief Retrieves a stored WalkOptimiser from a stream
 */
std::istream& operator>> (std::istream& input, WalkOptimiser& p)
{
    input >> p.m_best_parameters;
    input >> p.m_best_delta_parameters;
    input >> p.m_current_parameters;
    input >> p.m_real_best_parameters;
    char inbuffer[10];
    input.read(inbuffer, sizeof(int));
    p.m_iteration_count = *((int*) inbuffer);
    
    input.read(inbuffer, sizeof(bool));
    p.m_minimise = *((bool*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_best_performance = *((float*) inbuffer);
    
    input.read(inbuffer, sizeof(float));
    p.m_real_best_performance = *((float*) inbuffer);
    
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
