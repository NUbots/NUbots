/*! @file LinearApproximator.h
    @brief Implementation of a simple linear approximator. Missing save/load functionality. Was not completed
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
#ifndef LINEARAPPROXIMATOR_H
#define LINEARAPPROXIMATOR_H
#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "ApproximatorInterface.h"
#include <cstdlib>
using namespace std;

class LinearApproximator : public ApproximatorInterface
{
public:
    LinearApproximator();

    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 1);

    virtual void doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);

    virtual vector<float> getValues(vector<float> const& observations);

    virtual void saveApproximator(string agentName);

    virtual void loadApproximator(string agentName);

protected:
    vector<vector<float> > weights;

    int num_inputs;
    int num_outputs;


    vector<vector<float> > values_output;

};

#endif // LINEARAPPROXIMATOR_H
