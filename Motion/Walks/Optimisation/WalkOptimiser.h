/*! @file WalkOptimiser.h
    @brief Declaration of WalkOptimiser class
 
    @class WalkOptimiser
    @brief A module to optimise a walk engine.
 
    The walk optimiser needs to be generic, and either maximise the metric or minimise the metric.
    The walk optimiser also needs to simultaneously tune the walk parameters and the stiffnesses.
 
    @author Jason Kulk
 
 Copyright (c) 2009 Jason Kulk
 
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

class WalkOptimiser
{
    public:
        WalkOptimiser();
        ~WalkOptimiser();
    
        void doOptimisation();
    private:
        void tickOptimiser(float speed, float power);
        void mutateBestParameters();
    
        void initBestParameters();
        void copyToBestParameters();
    
        float normalDistribution(float mean, float sigma);
    
        void assessParameters(Step* currentstep, float currentspeed);
    
        void initOptimiserLog();
        void writeOptimiserLog();
    public:
        float BestSpeed;            // the speed in cm/s of the best set of parameters
        float BestCost;             // the cost of transport in J/(Ncm)
        float BestParameters[SM_NUM_MODES][SH_NUM_JOINTS];              // the best set of parameters
        float BestDeltaParameters[SM_NUM_MODES][SH_NUM_JOINTS];         // the difference BestParameters - PreviousBestParameters
    
        float CurrentParameters[SM_NUM_MODES][SH_NUM_JOINTS];
    private:
        Step* LeftStep;
        Step* RightStep;
    
        int SpeedCount;             // the number of speeds received with the current settings
        float SpeedSum;             // the cumulative sum of the received speeds 
        int SpeedCountLimit;        // the number of speeds required before progressing the optimisation
       
        float SpeedImprovement;     // the last speed improvement
        float SpeedPreviousImprovement;     // the previous speed improvement
        float CostImprovement;
        float CostPreviousImprovement;
    
        float PowerSum;             // the cumulative sums of the power
    
        float Alpha;
        int ResetLimit;
        int CountSinceLastImprovement;
    
        int AssessSpeedCount;       // the number of speeds received with the current settings that will be used to assess the speed accurately
        float AssessSpeedSum;       // the sum
        float AssessPowerSum;
        int AssessSpeedCountLimit;  // the number of speeds required before an assessment is reported.
    
        ofstream optimiserlog;
        int Iteration;
        float CurrentSpeed;
        float CurrentCost;
};


