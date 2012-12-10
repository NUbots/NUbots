/*! @file DictionaryApproximator.h
    @brief Uses a discretised lookup table derived from the continuous input vectors. Used in: MRLAgent.

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


#ifndef DICTAPPROXIMATOR_H
#define DICTAPPROXIMATOR_H

#include <sstream>
#include <fstream>
#include <vector>

#include <map>
#include <string>
#include <vector>
#include <iostream>
using namespace std;


class DictionaryApproximator {

private:
    int tileMultiplier,numInputs,numOutputs;
    map<string,float> approximator;
    float getValue(vector<float> const& observations,int action);
    float setValue(vector<float> const& observations,int action,float value);
    string getRepresentation(vector<float> const& observations,int action);
    
public:
    /*! @brief numberOfHiddens represents the tileMultiplier variable. This variable controls the resolution of the discretisation of the lookup table.
      Higher numberOfHiddens gives a higher resolution.
    */
    virtual void initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);
    
    virtual void doLearningEpisode(vector< vector<float> > const& observations, vector< vector<float> > const& values, float stepSize=0.1, int iterations=1);
    
    virtual vector<float> getValues(vector<float> const& observations);
    
    virtual void saveApproximator(string agentName);
    
    virtual void loadApproximator(string agentName);
    
    map<string,float>* getMap();
    
};

#endif
