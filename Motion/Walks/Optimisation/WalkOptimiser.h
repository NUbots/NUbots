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

#include "../WalkParameters.h"
#include <iostream>


class WalkOptimiser
{
    public:
        WalkOptimiser(const WalkParameters& walkparameters, bool minimise = true);
        ~WalkOptimiser();
    
        void getNewParameters(WalkParameters& walkparameters);
        void tick(float performance, WalkParameters& nextparameters);
    
        int getIterationCount();
        float getBestPerformance();
    
        void summaryTo(std::ostream& output);
        void csvTo(std::ostream& output);
        
        friend std::ostream& operator<< (std::ostream& output, const WalkOptimiser& p);
        friend std::istream& operator>> (std::istream& input, WalkOptimiser& p);
    private:
        void mutateBestParameters(WalkParameters& walkparameters);
        void mutateParameters(WalkParameters& base_parameters, WalkParameters& basedelta_parameters, WalkParameters& walkparameters);
    
        float normalDistribution(float mean, float sigma);

    private:
        WalkParameters m_best_parameters;              //!< the best set of parameters
        WalkParameters m_best_delta_parameters;        //!< the difference between the current best and the previous best (this 'gradient' is used by the line search part of the EHCLS)
        WalkParameters m_current_parameters;           //!< the current parameters under test
        WalkParameters m_previous_parameters;          //!< the previous parameters under test
        WalkParameters m_real_best_parameters;         //!< the actual best set of parameters ever seen
        
        int m_iteration_count;
        bool m_minimise;
        float m_current_performance;
        float m_best_performance;
        float m_real_best_performance;
        float m_alpha;
        int m_reset_limit;
        int m_count_since_last_improvement;
        float m_improvement, m_previous_improvement;
};


