/*! @file DictionaryApproximator.cpp
    @brief Uses a discrete lookup table derived from the continuous input vectors. Used in: MRLAgent.

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

void DictionaryApproximator::initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens) {
    numInputs = numberOfInputs;
    numOutputs = numberOfOutputs;
    tileMultiplier = numberOfHiddens;
   //Debug: cout<<"Approx init"<<endl;
}
    
void DictionaryApproximator::doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize, int iterations) {
    string tmp;
    for (int i = 0; i < observations.size(); i++) {
       //for each observation
        for (int j = 0; j < numOutputs; j++) {
          //for each possible action
            tmp = getRepresentation(observations[i],j);
            approximator[tmp] = values[i][j];//Assign the value function to be the input values.
        }
    }
}
    
vector<float> DictionaryApproximator::getValues(vector<float> const& observations) {
    vector<float> result;
    
    for (int i = 0; i < numOutputs; i++) {//For expectation_function from MRLAgent, i represents the ith entry of the predicted state.
        result.push_back(getValue(observations,i));
    }
    return result;
}

void DictionaryApproximator::saveApproximator(string agentName) {
    ofstream save_file;
    if((int)agentName.size()==0){
       save_file.open("DictionaryApprox.txt",ios_base::out);
    } else {
       save_file.open("ExpectationFunction.txt",ios_base::out);
    }
    string tempstr;
    
    save_file << approximator.size();
    
    for (map<string,float>::iterator iter = approximator.begin(); iter != approximator.end(); iter++) {
        save_file << "\n" << iter->first << " " << iter->second;
        //cout<< "Saved ("<< iter->first << "," << iter->second<<")"<<endl;
    }

    save_file.close();
    
}

map<string,float>* DictionaryApproximator::getMap(){
    return (&approximator);
}

    
void DictionaryApproximator::loadApproximator(string agentName) {
    ifstream save_file;
    if((int)agentName.size()==0){
       save_file.open("DictionaryApprox.txt",ios_base::in);
    } else {
       save_file.open("ExpectationFunction.txt",ios_base::in);
    }
    string tempstr;
    float tempval;
    int numvals;
    
    //get the number of values
    save_file >> numvals;
    
    for (int i = 0; i < numvals; i++) {
        save_file >> tempstr;
        save_file >> tempval;
        approximator[tempstr] = tempval;
        //Debug
        //cout<<"Loading: (s,a) = "<<tempstr<<", v = "<< tempval<<endl;
    }

    save_file.close();
}


string DictionaryApproximator::getRepresentation(vector<float> const& observations,int action) {
    stringstream result;
    
    for (int i = 0; i < observations.size(); i++) {
        result << (int)(observations[i]*tileMultiplier) << "_";//Changed seperator to underscore to avoid interfering with the save feature.
    }
    result << action;
    return result.str();
}

float DictionaryApproximator::getValue(vector<float> const& observations,int action) {

    string val = getRepresentation(observations,action);
    float result = approximator[val];

    return result;
}

float DictionaryApproximator::setValue(vector<float> const& observations,int action,float value) {
    string val = getRepresentation(observations,action);
    approximator[val] = value;
}
