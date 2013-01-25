/*! @file FourierApproximator.h
    @brief Class to implement fourier approximator/network.
        Works by taking a weighted linear combination of a number of cosine functions for each output.
        Learning done by a gradient descent rule.
    Number of basis functions = num_outputs*(k+1)^num_inputs if coupled
                              = num_outputs*num_inputs*(k+1) otherwise
    @author Jake Fountain

 Copyright (c) 2013 Jake Fountain

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
#ifndef FOURIERAPPROXIMATOR_H
#define FOURIERAPPROXIMATOR_H

//#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ApproximatorInterface.h"
#include "FourierFunction.h"
using namespace std;

class FourierApproximator : public ApproximatorInterface
{
public:
    FourierApproximator(bool fully_coupled, float learning_rate);

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range=10);

    virtual void doLearningEpisode(vector< vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);

    virtual vector<float> getValues(vector<float> const& observations);

    virtual void saveApproximator(string agentName);

    virtual void loadApproximator(string agentName);

protected:

    int num_outputs;
    int num_inputs;
    bool fully_coupled;
    float learning_rate;

    vector<FourierFunction> value_action_functions;

};

#endif // FOURIERAPPROXIMATOR_H
