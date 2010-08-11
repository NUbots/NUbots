/*! @file Optimiser.cpp
    @brief Implemenation of Optimiser class
 
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

#include "Optimiser.h"
#include "Parameter.h"

#include "debug.h"

/*!
 */
Optimiser::Optimiser(std::string name, vector<Parameter> parameters)
{
    m_name = name;
    m_parameters = parameters;
}

Optimiser::~Optimiser()
{
}

vector<float> Optimiser::getNextParameters()
{
    vector<float> parameters;
    parameters.reserve(m_parameters.size());
    for (size_t i=0; i<m_parameters.size(); i++)
        parameters.push_back(m_parameters[i].get());

    debug << "Optimiser::nextParameters() " << m_parameters << endl;
    return parameters;
}

void Optimiser::setParametersResult(float fitness)
{
    debug << "Optimiser::parameterResult(" << fitness << ")" << endl;
}

void Optimiser::summaryTo(ostream& stream)
{
    debug << "OptimiserSummary" << endl;
}