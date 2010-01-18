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

WalkOptimiser::WalkOptimiser(const WalkParameters& walkparameters)
{
    m_best_parameters = walkparameters;
    m_best_delta_parameters = walkparameters;
    for (int i=0; i<m_best_delta_parameters.size(); i++)
        m_best_delta_parameters[i] = 0;
    m_current_parameters = walkparameters;
    
    BestSpeed = 0;
    BestCost = 0;
    
    SpeedCount = 0;            // the number of speeds received with the current settings
    SpeedSum = 0;              // the cumulative sum of the received speeds 
    SpeedCountLimit = 50;     // the number of speeds required before progressing the optimisation. The refresh rate is 10Hz, so this will be 25s
    SpeedImprovement = 0;
    SpeedPreviousImprovement = 2.5;
    
    PowerSum = 0;
    
    Alpha = 0.0;
    m_count_since_last_improvement = 0;
    m_reset_limit = 10;
    
    AssessSpeedCount = 0;           // the number of speeds received with the current settings that will be used to assess the speed accurately
    AssessSpeedSum = 0;             // the sum
    AssessPowerSum = 0;
    AssessSpeedCountLimit = 1000;   // the number of speeds required before an assessment is reported.
    
    Iteration = 0;
    CurrentSpeed = 0;
    CurrentCost = 0;
}

WalkOptimiser::~WalkOptimiser()
{
}

/*void WalkOptimiser::doOptimisation()
{
        
    if (currentstep == NULL)                            // Don't do optimisation on NULL steps
        return;
    
    if (currentstep->StepClass != CLASS_NORMAL)         // Only do optimisation on NORMAL steps (for now)
        return;
    
    if (currentstep->StepType != TYPE_FORWARD)          // Only do optimisation on FORWARD steps (for now)
        return;

#if OPTIMISER_VERBOSITY > 4
    thelog << "OPTIMISER: doOptimisation on " << currentstep->Name << " with speed " << currentspeed << endl;
#endif
    
    // Collect the Left and Right Step the first time I see them
    if (LeftStep == NULL && currentstep->StepLeft == true)
        LeftStep = currentstep;
    
    if (RightStep == NULL && currentstep->StepLeft == false)
        RightStep = currentstep;
    
#if OPTIMISER_ASSESS == 1
    assessParameters(currentstep, currentspeed);
#else
    // Decide when it is time to tick the optimisation (based on the SpeedCount exceeding the required number of measurements)
    SpeedCount++;
    SpeedSum += currentspeed;
    PowerSum += (3/1000.0)*(batteryValues[E_VOLTAGE_MAX] + batteryValues[E_VOLTAGE_MAX])*fabs(batteryValues[E_CURRENT]) - 21.0;          // 21.0W is the idle power consumption of the robot
    if (SpeedCount >= SpeedCountLimit || balanceFallenCount > 0)
    {
        tickOptimiser(SpeedSum/SpeedCount, PowerSum/SpeedCount);
        balanceFallenCount = 0;
        SpeedCount = 0;
        SpeedSum = 0;
        PowerSum = 0;
    }
#endif
}*/

/*! @brief Gets a new set of parameters to test
    @param walkparameters will be updated to contain the new parameters that we want to test
 */
void WalkOptimiser::getNewParameters(WalkParameters& walkparameters)
{
    mutateParameters(m_best_parameters, m_best_delta_parameters, walkparameters);
}

/*! Generates a new set of parameters to be tested based on base_parameters and basedelta_parameters
 */
void WalkOptimiser::mutateParameters(WalkParameters& base_parameters, WalkParameters& basedelta_parameters, WalkParameters& walkparameters)
{
    // generate phi to mutate the BestParameters
    float sigma = 0.15*exp(m_count_since_last_improvement/m_reset_limit - 1);
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
        deltaparameters[i] = Alpha*basedelta_parameters[i] + (1-Alpha)*deltamutant[i];
    
    // now calculate the new parameters themselves
    vector<float> newparameters(base_parameters.size(), 0);
    for (int i=0; i<base_parameters.size(); i++)
        newparameters[i] = base_parameters[i] + deltaparameters[i];
    
    // now copy the new parameters into the storage variable
    for (int i=0; i<base_parameters.size(); i++)
        walkparameters[i] = newparameters[i];
}
/*
void Optimiser::tickOptimiser(float speed, float power)
{
    static bool initialised = false;            // we need to initialise the optimisation on the first tick
    static float cost = 0;                      // the specific cost of transport
    
    cost = power/(NAO_WEIGHT*speed);

    CurrentSpeed = speed;
    CurrentCost = cost;
    
    if (initialised == false)
    {
        if (LeftStep == NULL || RightStep == NULL)          // This is highly unlikely, but just in case; we are not initialised unless we have both the left and right step
            return;
        
        SpeedImprovement = 0;
        Alpha = 0;
        BestSpeed = speed;
        BestCost = cost;
        initBestParameters();
        CountSinceLastImprovement = 0;
        SpeedPreviousImprovement = 2.5;
        CostPreviousImprovement = 0.3;
        initialised = true;
        #if OPTIMISER_VERBOSITY > 3
            thelog << "OPTIMISER: tickOptimiser initialised." << endl;
        #endif
    }
#if OPTIMISER_VERBOSITY > 3
    thelog << "OPTIMISER: tickOptimiser on " << LeftStep->Name << " and " << RightStep->Name << " with speed " << speed << ", power " << power << ", and cost " << cost << endl;
#endif
    if (cost < BestCost && balanceFallenCount == 0)            // speed > BestSpeed
    {   
        #if OPTIMISER_VERBOSITY > 2
            thelog << "OPTIMISER: tickOptimiser Improvement." << endl;
        #endif
        SpeedImprovement = speed - BestSpeed;
        CostImprovement = cost - BestCost;
        //Alpha = 0.9*fabs(tanh(SpeedImprovement/SpeedPreviousImprovement));
        Alpha = 0.9*fabs(tanh(fabs(CostImprovement/CostPreviousImprovement)));
        copyToBestParameters();
        BestSpeed = speed;
        BestCost = cost;
        CountSinceLastImprovement = 0;
        #if OPTIMISER_VERBOSITY > 2
            thelog << "OPTIMISER: tickOptimiser BestSpeed:" << BestSpeed << " BestCost: " << BestCost << "SpeedImprovement:" << SpeedImprovement << " PreviousSpeedImprovement:" << SpeedPreviousImprovement << "CostImprovement:" << CostImprovement << "PreviousCostImprovement:" << CostPreviousImprovement << " Alpha:" << Alpha << endl;
        #endif
        SpeedPreviousImprovement = SpeedImprovement;
        CostPreviousImprovement = CostImprovement;
    }
    else
        CountSinceLastImprovement++;
    
    if (CountSinceLastImprovement > ResetLimit)
    {
        Alpha *= 0.9;
        CountSinceLastImprovement = 0;
        BestSpeed *= 0.97;
        BestCost *= 1.1;
        #if OPTIMISER_VERBOSITY > 2
            thelog << "OPTIMISER: tickOptimiser Reseting" << endl;
        #endif
    }
    
    mutateBestParameters();
    Iteration++;
    return;
}*/

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
    
/*    double randoms[1000];
    double sum = 0;
    for (int f=0; f<1000; f++)
    {
        randoms[f] = normalDistribution(3, 0.5);
        sum += randoms[f];
    }
    
    float mean = sum/1000;
    sum = 0;
    for (int f=0; f<1000; f++)
    {
        sum += (randoms[f] - mean)*(randoms[f] - mean);
    }
    float sd = sqrt(sum/1000);
    
    thelog << "Mean" << mean << "SD" << sd << endl;*/
}
