/*! @file MRLAgent.h
    @brief Motivated reinforcement learning agent. Provides its own reward structure for self motivation based on novelty.
    Uses a fourier approximator to store the learnt expected reward "value" function.
    ---------------------------------------------------
    Make your own MRLAgent:(WARNING: The motivation wundt function has been tuned for use in head-behaviour, do not change.)
    Follow the template below to implement a Motivated reinforcement agent.


    MRLAgent mrlagent;
    mrlagent.setParameters(0.1,0.5,0.5,1.0,1,5);
    mrlagent.initialiseAgent(2,5,1);

    for (number of iterations){
        int action = mrlagent.getActionAndLearn(observation);
        updateWorld(action);
    }
    ---------------------------------------------------
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

#ifndef MRLAGENT_H
#define MRLAGENT_H
#include <vector>
#include <algorithm>

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"
#include "DictionaryApproximator.h"
#include "RLAgent.h"
#include "FourierApproximator.h"


class MRLAgent: public RLAgent
{
public:
    MRLAgent();
    ~MRLAgent();
    void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range = 10);

    void giveMotivationReward();
    float wundtFunction(float N);
    int getActionAndLearn(vector<float> observations, vector<int> valid_actions);
    void saveMRLAgent(string agentName);
    void loadMRLAgent(string agentName);

    map<string,float>* getMap();
    ApproximatorInterface* expectation_map;
};

#endif // MRLAGENT_H
