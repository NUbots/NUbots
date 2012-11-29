/*! @file ApproximatorInterface.h
    @brief Standard interface for a function approximator mapping vector of floats to a vector of floats.

    @author Josiah Walker

 Copyright (c) 2012 Josiah Walker

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

#ifndef APPROXIMATORINTERFACE_H
#define APPROXIMATORINTERFACE_H

#include <vector>
#include<string>
using namespace std;

class ApproximatorInterface {
    
public:
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens)=0;
    
    virtual void doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1)=0;
    
    virtual vector<float> getValues(vector<float> const& observations)=0;
    
    virtual void saveApproximator(string agentName)=0;
    
    virtual void loadApproximator(string agentName)=0;
    
    
};

#endif



