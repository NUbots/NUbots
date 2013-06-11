/*! @file RLAgent.h
    @brief Standard implementation of reinforcement learning agent.

    @author Jake Fountain

    Make your own RLAgent:
    1. Implement an RLAgent subclass. All that is required is the addition of a constructor initialising the function approximator.
    for example see DictionaryRLAgent.
    2. Follow the template below to implement a Motivated reinforcement learning agent in your code to explore a state space and make decisions.


    RLAgent rlagent;
    try{
        loadAgent(Filename);
    }catch (std::string s){
        rlagent.setParameters(0.1,0.5,0.5,1.0,1,5);//example parameters
        rlagent.initialiseAgent(observation_size,number_of_actions,resolution_of_FunctionApproximator);
    }

    for (number of iterations){
        int action = rlagent.getAction(observation);

        rlagent.giveReward(getRewardFromWorld());

        updateWorld(action);

        if(number of iterations has passed)
            rlagent.doLearning();
    }
    ---------------------------------------------------

 Copyright (c) 2012 Jake Fountain

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
#ifndef RLAGENT_H
#define RLAGENT_H

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"


#include <vector>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <cmath>

class RLAgent: public RLearningInterface
{
public:
    virtual void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 10);

    virtual void setParameters(float alpha=0.1f, float beta=0.5, float gamma=0.9f, float lambda=0.9f,int learningIterations=1, int memory_length = 10, bool use_soft_max = false);

    virtual int getAction(std::vector<float> observations,std::vector<int> valid_actions);//Must return integer between 0 and numberOfOutputs-1

    virtual void giveReward(float reward);

    virtual void doLearning();

    virtual void saveAgent(std::string agentName);

    virtual void loadAgent(std::string agentName);

    virtual void log(std::string text);


    std::vector<float> getValues(std::vector<float> v);
    int checkAction(std::vector<float> obs, std::vector<int> valid_actions);

    RLAgent();
    ~RLAgent();

protected:
    ApproximatorInterface* FunctionApproximator;
    float alpha;
    float beta;
    float gamma;
    float lambda;
    int learningIterations;
    int memory_length;//Determines the number of observations stored when saved.

    int num_inputs;
    int num_outputs;
    int num_hidden;

    std::vector<int> actions;
    std::vector<float> last_values;
    std::vector<std::vector<int> > action_validities;

    std::vector<std::vector<float> > values;
    std::vector<std::vector<float> > observations;
    std::vector<float> rewards;

    float max(std::vector<float> x, std::vector<int> valid_actions);

    int getSoftMaxAction(std::vector<float> values, std::vector<int> valid_actions);

    bool use_soft_max;

};

#endif // RLAGENT_H
