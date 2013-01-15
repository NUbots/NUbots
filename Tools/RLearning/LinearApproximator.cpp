#include "LinearApproximator.h"

LinearApproximator::LinearApproximator()
{
    srand(0);
}

void LinearApproximator::initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range)
{
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    vector<vector<float> > w(num_outputs,vector<float>(num_inputs+1,0));
    for (int i = 0; i<w.size();i++){
         for (int j = 0; j<w[0].size();j++){
             w[i][j] =0;//rand()/(float)RAND_MAX;
         }
    }
    weights = w;
}

vector<float> LinearApproximator::getValues(vector<float> const& observations){
    vector<float> result;
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

void LinearApproximator::doLearningEpisode(vector<vector<float> > const& observations, vector<vector<float> > const& values, float stepSize, int iterations){
    for (int obs = 0; obs<observations.size()-1; obs++){
        //Calculate the norm of the observation vector.
        float norm_squared = 0;
        for (int i = 0; i<observations[obs].size();i++){
            norm_squared+=observations[obs][i]*observations[obs][i];
        }
        //Update neurons:
        for (int i = 0; i<iterations;i++){
            vector<float> current_values = getValues(observations[obs]);
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



void LinearApproximator::saveApproximator(string agentName)
{
}


void LinearApproximator::loadApproximator(string agentName)
{
}

