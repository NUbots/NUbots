/*! @file MRLAgent.h
    @brief Motivated reinforcement learning agent. Provides its own reward structure for self motivation based on novelty.
    Uses a dictionary approximator to store the learnt expected reward "value" function.

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

#include "ApproximatorInterface.h"
#include "RLearningInterface.h"
#include "DictionaryApproximator.h"
#include "RLAgent.h"



class MRLAgent: public RLAgent
{
public:
    MRLAgent();
    void initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens);

    float giveMotivationReward();
    float wundtFunction(float N);

};

#endif // MRLAGENT_H
