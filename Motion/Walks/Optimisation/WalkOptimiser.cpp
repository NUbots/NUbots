/*
 *  optimiser.cpp
 *  walkoptimiser
 *
 *  Created by jason on 7/09/09.
 *  Copyright 2009 UoN. All rights reserved.
 *
 */

#include "optimiser.h"
#include "boost/random.hpp"
#include <math.h>

#define OPTIMISER_VERBOSITY         4
#define OPTIMISER_ASSESS            0

#define NAO_WEIGHT                  4.8*9.81

Optimiser::Optimiser()
{
    BestSpeed = 0;
    BestCost = 0;
    
    SpeedCount = 0;            // the number of speeds received with the current settings
    SpeedSum = 0;              // the cumulative sum of the received speeds 
    SpeedCountLimit = 50;     // the number of speeds required before progressing the optimisation. The refresh rate is 10Hz, so this will be 25s
    SpeedImprovement = 0;
    SpeedPreviousImprovement = 2.5;
    
    PowerSum = 0;
    
    LeftStep = NULL;           // I don't know which step I am going to be optimising, so I need to collect them as I go
    RightStep = NULL;
    
    Alpha = 0.0;
    CountSinceLastImprovement = 0;
    ResetLimit = 10;
    
    AssessSpeedCount = 0;           // the number of speeds received with the current settings that will be used to assess the speed accurately
    AssessSpeedSum = 0;             // the sum
    AssessPowerSum = 0;
    AssessSpeedCountLimit = 1000;   // the number of speeds required before an assessment is reported.
    
    Iteration = 0;
    CurrentSpeed = 0;
    CurrentCost = 0;
    initOptimiserLog();
}

Optimiser::~Optimiser()
{
}

void Optimiser::doOptimisation(Step* currentstep, float currentspeed)
{
    if (balanceFallenCount > 0)
        SpeedSum = 0.01;
        
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
}

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
    writeOptimiserLog();
    Iteration++;
    return;
}

/* Generates a new set of parameters to be tested based on BestParameters and BestDeltaParameters
 */
void Optimiser::mutateBestParameters()
{
    
    // generate phi to mutate the BestParameters
    float sigma = 0.15*exp(CountSinceLastImprovement/ResetLimit - 1);
    static float phi[SM_NUM_MODES][SH_NUM_JOINTS];
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            phi[i][j] = normalDistribution(1, sigma);
    
    #if OPTIMISER_VERBOSITY > 4
        thelog << "OPTIMISER: mutateBestParameters. Phi:" << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << phi[i][j] << ",";
        thelog << endl;
    #endif
    
    // mutate the BestParameters
    static float mutant[SM_NUM_MODES][SH_NUM_JOINTS];
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            mutant[i][j] = BestParameters[i][j] * phi[i][j];        // TODO: Add in C
    
    #if OPTIMISER_VERBOSITY > 4
        thelog << "OPTIMISER: mutateBestParameters. Mutant:" << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << mutant[i][j] << ",";
        thelog << endl;
    #endif
    
    // calculate the difference between the mutated state and the best one
    static float deltamutant[SM_NUM_MODES][SH_NUM_JOINTS];
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            deltamutant[i][j] = BestParameters[i][j] - mutant[i][j];
    
    #if OPTIMISER_VERBOSITY > 4
        thelog << "OPTIMISER: mutateBestParameters. Deltamutant:" << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << deltamutant[i][j] << ",";
        thelog << endl;
    #endif
    
    // calculate the desired change in parameters
    static float deltaparameters[SM_NUM_MODES][SH_NUM_JOINTS];
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            deltaparameters[i][j] = Alpha*BestDeltaParameters[i][j] + (1-Alpha)*deltamutant[i][j];
    
    #if OPTIMISER_VERBOSITY > 4
        thelog << "OPTIMISER: mutateBestParameters. Deltaparameters:" << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << deltaparameters[i][j] << ",";
        thelog << endl;
    #endif
    
    // now calculate the new parameters themselves
    static float newparameters[SM_NUM_MODES][SH_NUM_JOINTS];
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            newparameters[i][j] = BestParameters[i][j] + deltaparameters[i][j];
    
    #if OPTIMISER_VERBOSITY > 2
        thelog << "OPTIMISER: mutateBestParameters. New parameters:" << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << newparameters[i][j] << ",";
        thelog << endl;
    #endif
    
    // now put the new parameters in the steps so that they will be used
    for (int i=0; i<SM_NUM_MODES; i++)
    {
        for (int j=0; j<SH_NUM_JOINTS; j++)
        {
            LeftStep->StepSupportHardnesses[i][j] = newparameters[i][j];
            RightStep->StepSupportHardnesses[i][j] = newparameters[i][j];
        }
    }
    return;
}
 
/* Copies the parameters in the LeftStep to the BestParameters, and updates BestDeltaParameters

 The Left and Right Step are also compared to see if there are any differences --- which is an error.
 */
void Optimiser::copyToBestParameters()
{
    #if OPTIMISER_VERBOSITY > 4
        thelog << "OPTIMISER: copyToBestParameters." << endl;
        thelog << "OPTIMISER: Best Parameters before update." << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << BestParameters[i][j] << ",";
        thelog << endl;
    #endif
    for (int i=0; i<SM_NUM_MODES; i++)
    {
        for (int j=0; j<SH_NUM_JOINTS; j++)
        {
            BestDeltaParameters[i][j] = LeftStep->StepSupportHardnesses[i][j] - BestParameters[i][j];
            BestParameters[i][j] = LeftStep->StepSupportHardnesses[i][j];
            if (RightStep != NULL && fabs(LeftStep->StepSupportHardnesses[i][j] - RightStep->StepSupportHardnesses[i][j]) > 0.001)
                thelog << "OPTIMISER: tickOptimiser. WARNING. The left and right steps have different parameters." << endl;
        }
    }
    #if OPTIMISER_VERBOSITY > 3
        thelog << "OPTIMISER: Best Parameters after update." << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << BestParameters[i][j] << ",";
        thelog << endl;
        thelog << "OPTIMISER: Difference between best and previous best parameters." << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << BestDeltaParameters[i][j] << ",";
        thelog << endl;
    #endif
}

/* Initialises the BestParameters to be equal to those stored in the LeftStep, and sets the BestDeltaParameters to all zeros
 */
void Optimiser::initBestParameters()
{
    for (int i=0; i<SM_NUM_MODES; i++)
    {
        for (int j=0; j<SH_NUM_JOINTS; j++)
        {
            BestDeltaParameters[i][j] = 0;
            BestParameters[i][j] = LeftStep->StepSupportHardnesses[i][j];
        }
    }
}

/* Returns a normal random variable from the normal distribution with mean and sigma
 */
float Optimiser::normalDistribution(float mean, float sigma)
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

void Optimiser::assessParameters(Step* currentstep, float currentspeed)
{
    // Decide when it is time to tick the optimisation (based on the SpeedCount exceeding the required number of measurements)
    AssessSpeedCount++;
    AssessSpeedSum += currentspeed;
    AssessPowerSum += (3/1000.0)*(batteryValues[E_VOLTAGE_MAX] + batteryValues[E_VOLTAGE_MAX])*fabs(batteryValues[E_CURRENT]) - 21.0;          // 21.0W is the idle power consumption of the robot
    if (AssessSpeedCount >= AssessSpeedCountLimit)
    {
        float speed = AssessSpeedSum/AssessSpeedCount;
        float power = AssessPowerSum/AssessSpeedCount;
        float cost = power/(NAO_WEIGHT*speed);
        
        CurrentSpeed = speed;
        CurrentCost = cost;
        
        thelog << "OPTIMISER: Assessment: Avg. Speed: " << speed << "Cost: " << power/(NAO_WEIGHT*speed) << " Falls: " << balanceFallenCount << endl;
        thelog << "OPTIMISER: Parameters: " << endl;
        for (int i=0; i<SM_NUM_MODES; i++)
            for (int j=0; j<SH_NUM_JOINTS; j++)
                thelog << currentstep->StepSupportHardnesses[i][j] << ",";
        thelog << endl;
        balanceFallenCount = 0;
        AssessSpeedCount = 0;
        AssessSpeedSum = 0;
        AssessPowerSum = 0;
        writeOptimiserLog();
        Iteration++;
    }
}

void Optimiser::initOptimiserLog()
{
    optimiserlog.open("/var/log/optimiser.log");
    optimiserlog << "Time (s), Iteration, Speed, Cost, ParamNorm, BestSpeed, BestCost, BestParamNorm" << endl;
}

void Optimiser::writeOptimiserLog()
{
    optimiserlog << dcmTimeSinceStart << ", " << Iteration << ", " << CurrentSpeed << ", " << CurrentCost << ", ";
    // calculate the parameter norm
    float sqrdsum = 0;
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            sqrdsum += pow(LeftStep->StepSupportHardnesses[i][j], 2);
    float norm = sqrt(sqrdsum);
    optimiserlog << norm << ", ";
    optimiserlog << BestSpeed << ", " << BestCost << ", ";
    // calculate the parameter norm
    sqrdsum = 0;
    for (int i=0; i<SM_NUM_MODES; i++)
        for (int j=0; j<SH_NUM_JOINTS; j++)
            sqrdsum += pow(BestParameters[i][j], 2);
    norm = sqrt(sqrdsum);
    optimiserlog << norm << ", " << endl;
    
}

