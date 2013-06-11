/*! @file LinearApproximator.cpp
    @brief Implementation of a simple linear approximator. Missing save/load functionality. Was not completed
    @author Jake Fountain

 Copyright (c) 2013 Jake Fountain

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
#include "LinearApproximator.h"

LinearApproximator::LinearApproximator():ApproximatorInterface()
{
    srand(0);
}

void LinearApproximator::initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range)
{
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    std::vector<std::vector<float> > w(num_outputs,std::vector<float>(num_inputs+1,0));
    for (int i = 0; i<w.size();i++){
         for (int j = 0; j<w[0].size();j++){
             w[i][j] =0;//rand()/(float)RAND_MAX;
         }
    }
    weights = w;
}

std::vector<float> LinearApproximator::getValues(std::vector<float> const& observations){
    std::vector<float> result;
    for(int i = 0; i<num_outputs; i++){
        float sum = 0;
        for( int j = 0; j<observations.size(); j++){
            sum+= observations[j]*weights[i][j];
        }
        sum+=weights[i][num_inputs];//bias
        result.push_back(sum);
    }
    //values_output.push_back(result);
    return result;

}

void LinearApproximator::doLearningEpisode(std::vector<std::vector<float> > const& observations, std::vector<std::vector<float> > const& values, float stepSize, int iterations){
    for (int obs = 0; obs<observations.size()-1; obs++){
        //Calculate the norm of the observation std::vector.
        float norm_squared = 0;
        for (int i = 0; i<observations[obs].size();i++){
            norm_squared+=observations[obs][i]*observations[obs][i];
        }
        //Update neurons:
        for (int i = 0; i<iterations;i++){
            std::vector<float> current_values = getValues(observations[obs]);
            for(int action = 0; action < num_outputs; action++){
                float delta = values[obs][action]-current_values[action];
                for(int input = 0; input<num_inputs; input++){
                    if (norm_squared !=0){
                        weights[action][input]+= stepSize*delta*observations[obs][input]/norm_squared;
                    }
                }
                weights[action][num_inputs]+= stepSize*delta;
            }
        }
    }



}



void LinearApproximator::saveApproximator(std::string agentName)
{
}


void LinearApproximator::loadApproximator(std::string agentName)
{
}

