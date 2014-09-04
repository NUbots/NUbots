/*! @file DictionaryApproximator.cpp
    @brief Uses a discrete lookup table derived from the continuous input std::vectors. Used in: MRLAgent.

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

#include "DictionaryApproximator.h"

void DictionaryApproximator::initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range) {
    numInputs = numberOfInputs;
    numOutputs = numberOfOutputs;
    tileMultiplier = numberOfHiddens;
   //Debug: std::cout<<"Approx init"<<std::endl;
}
    
void DictionaryApproximator::doLearningEpisode(std::vector<std::vector<float> > const& observations, std::vector< std::vector<float> > const& values, float stepSize, int iterations) {
    std::string tmp;
    for (int i = 0; i < observations.size(); i++) {
       //for each observation
        for (int j = 0; j < numOutputs; j++) {
          //for each possible action
            tmp = getRepresentation(observations[i],j);
            approximator[tmp] = values[i][j];//Assign the value function to be the input values.
        }
    }
}
    
std::vector<float> DictionaryApproximator::getValues(std::vector<float> const& observations) {
    std::vector<float> result;
    
    for (int i = 0; i < numOutputs; i++) {//For expectation_function from MRLAgent, i represents the ith entry of the predicted state.
        result.push_back(getValue(observations,i));
    }
    return result;
}

void DictionaryApproximator::saveApproximator(std::string agentName) {
    std::ofstream save_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName;
    save_file.open(file_name.str().c_str(),std::fstream::out);

    std::string tempstr;
    
    save_file << approximator.size();
    
    for (std::map<std::string,float>::iterator iter = approximator.begin(); iter != approximator.end(); iter++) {
        save_file << "\n" << iter->first << " " << iter->second;
        //std::cout<< "Saved ("<< iter->first << "," << iter->second<<")"<<std::endl;
    }

    save_file.close();
    
}

std::map<std::string,float>* DictionaryApproximator::getMap(){
    return (&approximator);
}

    
void DictionaryApproximator::loadApproximator(std::string agentName) {
    std::ifstream save_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName;
    save_file.open(file_name.str().c_str(),std::fstream::out);
    if(!save_file.good()) {
        throw std::string("DictionaryApproximator::loadApproximator - file not found: ") + file_name.str();
    }
    std::string tempstr;
    float tempval;
    int numvals;
    
    //get the number of values
    save_file >> numvals;
    
    for (int i = 0; i < numvals; i++) {
        save_file >> tempstr;
        save_file >> tempval;
        approximator[tempstr] = tempval;
        //Debug
        //std::cout<<"Loading: (s,a) = "<<tempstr<<", v = "<< tempval<<std::endl;
    }

    save_file.close();
}


std::string DictionaryApproximator::getRepresentation(std::vector<float> const& observations,int action) {
    std::stringstream result;
    
    for (int i = 0; i < observations.size(); i++) {
        result << (int)(observations[i]*tileMultiplier) << "_";//Changed seperator to underscore to avoid interfering with the save feature.
    }
    result << action;
    return result.str();
}

float DictionaryApproximator::getValue(std::vector<float> const& observations,int action) {

    std::string val = getRepresentation(observations,action);
    float result = approximator[val];

    return result;
}

float DictionaryApproximator::setValue(std::vector<float> const& observations,int action,float value) {
    std::string val = getRepresentation(observations,action);
    approximator[val] = value;
}
