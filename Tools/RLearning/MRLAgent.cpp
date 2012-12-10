/*! @file MRLAgent.h
    @brief Motivated reinforcement learning agent. Provides its own reward structure for self motivation based on novelty.
    Uses a dictionary approximator to store the learnt expected reward "value" function.
    ---------------------------------------------------
    Make your own MRLAgent:Follow the template below to implement a Motivated reinforcement agent.


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

#include "MRLAgent.h"

#include <math.h>

MRLAgent::MRLAgent():RLAgent()
{

    FunctionApproximator = (ApproximatorInterface*)(new DictionaryApproximator());

    expectation_map = (ApproximatorInterface*)(new DictionaryApproximator());

}

MRLAgent::~MRLAgent(){
    delete expectation_map;

}


/*! @brief
        Initialises agent by initialising function approximator
*/
void MRLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    //MRLFunctionApprox FuncApprox(numberOfInputs, numberOfOutputs, numberOfHiddens);
    //FunctionApproximator = FuncApprox;


    FunctionApproximator->initialiseApproximator(numberOfInputs, numberOfOutputs, numberOfHiddens);
    expectation_map->initialiseApproximator(numberOfInputs+1, numberOfInputs,numberOfHiddens);

    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;
    last_action = 0;
    second_last_action = 0;


    //Perform initial observations, values and rewards list setups. Required to offset learning updates.
    vector<float> dummy_observation(numberOfInputs,0);    
    getAction(dummy_observation);
    giveMotivationReward();
}



/*! @brief
    Main feature of the MRL agent. The novelty of the latest observation is calculated by taking the Euclidean norm
    square of the previous observation vectors with the latest.The motivation reward is then calculated by taking the Wundt function
    of the novelty.*/

void MRLAgent::giveMotivationReward(){
    //Error in here somewhere:
    float novelty=0;

    vector<float> observation_action(num_inputs, 0);

    if ((int)observations.size()>=2){
       observation_action = observations[observations.size()-2];
    }

    observation_action.push_back(second_last_action);

    vector<float> expected_observation = expectation_map->getValues(observation_action);

    vector<float> actual_observation = observations[observations.size()-1];

    //calculate error in prediction:
    for (int j=0; j<expected_observation.size();j++){
        float diff = 0;
        if(actual_observation[j]!=0){
            diff = (expected_observation[j]-actual_observation[j])/actual_observation[j];
        }else{
            diff = expected_observation[j];
        }
        novelty+= diff*diff;
    }

    //Do learning for expectation_map
    vector<vector<float> > obs(1,observation_action);//Setup vector with one input to be used for expectation_map learning.
    vector<vector<float> > val(1,expected_observation);//setup unit list of values.
    //Change values:

    for (int j=0; j<expected_observation.size();j++){
        float diff = actual_observation[j]-expected_observation[j];

        val[0][j]+= lambda*diff;
    }
    expectation_map->doLearningEpisode(obs,val,1.0,1);



   /*OLD NOVELTY

    //Debug
    // cout<<"obs list size = "<<observations.size()<<endl;
    float memory_decay = 0.5;
    int count=0;
    //Iterate backwards over all observations and calculate a sum of euclidean metrics of latest observation with old observations.
    for (int i=observations.size()-2; i>=std::max(0,((int)observations.size()-memory_length-1));i--){


        for (int j=0; j<observations[i].size();j++){
            float diff = observations[observations.size()-1][j]-observations[i][j];
            novelty+= memory_decay*diff*diff;
        }

        count++;
        memory_decay*=memory_decay;

    }
*/




    float motivation = wundtFunction(novelty);

    giveReward(motivation);

}



/*! @brief
        The wundt function is a linear combination of two sigmoids, offset by -1. It is similar to a decapitated gaussian distribution.
        The most important parameters are N1 and N2, which give the rise and dip points of the distribution respectively.
*/
float MRLAgent::wundtFunction(float N){
    float N1 = 1;//Location of max positive gradient
    float N2 = 2;//Location of max negative gradient

    float M1 = 1;//Maximum motivation offset (eg: M1=2, M2 = 0, baseline = -1 gives range -1 to 1)
    float M2 = 0;//Minimum motivation offset
    float baseline = 0;
    float M3 = M2-M1;//maximum negative reward

    float p1 = 1;//Max pos gradient
    float p2 = 1;//Max negative gradient

    //Positive reward
    float F1 = M1/(1+exp(-p1*(N-N1)));
    //Negative Reward
    float F2 = M3/(1+exp(-p2*(N-N2)));

    return (F1+F2+baseline);

}
/*! @brief Gets the map from the dictionary approximator.
 */
 map<string,float>* MRLAgent::getMap(){
     return ((DictionaryApproximator*)FunctionApproximator)->getMap();
 }


 /*! @brief Main loop for MRLAgent.
  */
 int MRLAgent::getActionAndLearn(vector<float> observations){
     int action = getAction(observations);
     giveMotivationReward();
     doLearning();
     return action;
 }

 void MRLAgent::saveMRLAgent(){
     expectation_map->saveApproximator("ExpectationFunction.txt");
     saveAgent("");

 }

 void MRLAgent::loadMRLAgent(){
     expectation_map->loadApproximator("ExpectationFunction.txt");
     loadAgent("");

 }
