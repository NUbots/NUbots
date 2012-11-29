/*! @file MRLAgent.cpp
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

#include "MRLAgent.h"

#include <math.h>

MRLAgent::MRLAgent():RLAgent()
{
    FunctionApproximator = (ApproximatorInterface*)(new DictionaryApproximator());
}



/*! @brief
        Initialises agent by initialising function approximator
*/
void MRLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    //MRLFunctionApprox FuncApprox(numberOfInputs, numberOfOutputs, numberOfHiddens);
    //FunctionApproximator = FuncApprox;
    FunctionApproximator->initialiseApproximator(numberOfInputs, numberOfOutputs, numberOfHiddens);
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;

}



/*! @brief
    Main feature of the MRL agent. The novelty of the latest observation is calculated by taking the Euclidean norm
    square of the previous observation vectors with the latest.The motivation reward is then calculated by taking the Wundt function
    of the novelty.*/
float MRLAgent::giveMotivationReward(){
    float novelty=0;
    float memory_decay_factor= 0.9;
    int count = 0;//used to normalise the novelty
    for (int i=0; i<observations.size()-1;i++){
        for (int j=0; j<observations[i].size();j++){

            novelty+= memory_decay_factor*(observations[i][j]-observations[observations.size()-1][j])*(observations[i][j]-observations[observations.size()-1][j]);

            //novelty = sum of squared differences of current percept from all previous percepts, with time distant percepts decreasing in importance exponentially
        }
        memory_decay_factor*=memory_decay_factor;

        count++;
    }
    float motivation = wundtFunction(novelty/count);

    giveReward(motivation);
}



/*! @brief
        The wundt function is a linear combination of two sigmoids. It is similar to a decapitated gaussian distribution.
*/
float MRLAgent::wundtFunction(float N){
    float N1 = 1;//Location of max positive gradient
    float N2 = 2;//Location of max negative gradient

    float M1 = 1;//Maximum motivation
    float M2 = 0;//Minimum motivation
    float M3 = M2-M1;//maximum negative reward

    float p1 = 1;//Max pos gradient
    float p2 = 1;//Max negative gradient

    //Positive reward
    float F1 = M1/(1+exp(-p1*(2*N-N1)));
    //Negative Reward
    float F2 = M3/(1+exp(-p2*(2*N-N2)));

    return F1-F2;

}

