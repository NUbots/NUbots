/*! @file RLAgent.h
    @brief Standard implementation of reinforcement learning agent.

    @author Jake Fountain

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

class RLAgent: public RLearningInterface
{
public:
    virtual void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);

    virtual void setParameters(float alpha=0.1f, float beta=0.5, float gamma=0.9f, float lambda=0.9f,int learningIterations=1, int memory_length = 10);

    virtual int getAction(vector<float> observations);//Must return integer between 0 and numberOfOutputs-1

    virtual void giveReward(float reward);

    virtual void doLearning();

    virtual void saveAgent(string agentName);

    virtual void loadAgent(string agentName);

    virtual void log(string text);

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

    int last_action;
    vector<float> last_values;

    vector<vector<float> > values;
    vector<vector<float> > observations;
    vector<float> rewards;

    float max(vector<float> x);

};

#endif // RLAGENT_H
