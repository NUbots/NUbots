/*! @file RLearningInterface.h
    @brief Standard interface for any reinforcement learning agent

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

#ifndef RLEARNINGINTERFACE_H
#define RLEARNINGINTERFACE_H



class RLearningInterface {
    
public:
    virtual void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 1)=0;
    
    virtual void setParameters(float alpha=0.1f, float beta=0.5, float gamma=0.9f, float lambda=0.9f,int learningIterations=1, int memory_length = 10, bool use_soft_max = false)=0;
    
    virtual int getAction(vector<float> observations, vector<int> valid_actions)=0;//Must return integer between 0 and numberOfOutputs-1
    
    virtual void giveReward(float reward)=0;
    
    virtual void doLearning()=0;
    
    virtual void saveAgent(string agentName)=0;

    virtual void loadAgent(string agentName)=0;
    

};

#endif
