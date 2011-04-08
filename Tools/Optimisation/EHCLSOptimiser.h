/*! @file EHCLSOptimiser.h
    @brief Declaration of hill climber with line search
 
    @class EHCLSOptimiser
    @brief A EHCLS Optimiser class
 
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

#ifndef EHCLS_OPTIMISER_H
#define EHCLS_OPTIMISER_H

#include "Optimiser.h"

class EHCLSOptimiser : public Optimiser
{
public:
    EHCLSOptimiser(string name, vector<Parameter> parameters);
    ~EHCLSOptimiser();
    
    vector<float> getNextParameters();
    void setParametersResult(float fitness);
    
    void summaryTo(ostream& stream);
private:
    void mutateBestParameters(vector<Parameter>& parameters);
    void mutateParameters(vector<Parameter>& base_parameters, vector<float>& basedelta_parameters, vector<Parameter>& parameters);
    
    void toStream(ostream& o) const;
    void fromStream(istream& i);
private:
    vector<Parameter> m_best_parameters;              //!< the best set of parameters
    vector<float> m_best_delta_parameters;        	  //!< the difference between the current best and the previous best (this 'gradient' is used by the line search part of the EHCLS)
    vector<Parameter> m_current_parameters;           //!< the current parameters under test
    vector<Parameter> m_previous_parameters;          //!< the previous parameters under test
    vector<Parameter> m_real_best_parameters;         //!< the actual best set of parameters ever seen
    
    int m_iteration_count;
    float m_current_performance;
    float m_best_performance;
    float m_real_best_performance;
    float m_alpha;
    int m_count_since_last_improvement;
    float m_improvement, m_previous_improvement;
    float m_neta;                                   //!< a parameter that controls the breadth of the search
    int m_reset_limit;	                            //!< a parameter that controls how quickly we give up searching along a line
    float m_reset_fraction;                         //!< a parameter that controls how much we reset
};

#endif

