/*! @file MRLAgent.h
    @brief Motivated reinforcement learning agent. Provides its own reward structure for self motivation based on novelty.
    Uses a fourier approximator to store the learnt expected reward "value" function.
    ---------------------------------------------------
    Make your own MRLAgent:(WARNING: The motivation wundt function has been tuned for use in head-behaviour, do not change.)
    Follow the template below to implement a Motivated reinforcement learning agent.


    MRLAgent mrlagent;
    mrlagent.setParameters(0.1,0.5,0.5,1.0,1,5);//example parameters
    mrlagent.initialiseAgent(observation_size,number_of_actions,resolution_of_dictionaryApproximator);

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
    FunctionApproximator = (ApproximatorInterface*)(new FourierApproximator(false, (float)0.01));
    expectation_map = (ApproximatorInterface*)(new FourierApproximator(false, (float)0.01));
}

MRLAgent::~MRLAgent(){
    delete expectation_map;
}


/*! @brief
        Initialises agent by initialising function approximator
*/
void MRLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range){
    FunctionApproximator->initialiseApproximator(numberOfInputs, numberOfOutputs, numberOfHiddens,max_parameter_range);
    expectation_map->initialiseApproximator(numberOfInputs+numberOfOutputs, numberOfInputs,numberOfHiddens,max_parameter_range);
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;

    //Perform initial observations, values and rewards list setups. Required to offset learning updates.
    vector<float> dummy_observation(numberOfInputs,0);
    vector<int> vect(num_outputs,1);
    getAction(dummy_observation,vect);
    giveMotivationReward();
}



/*! @brief
    Main feature of the MRL agent. The novelty of the state-action is calculated by comparing the expected outcome of the action from the state
    to the actual outcome of the action. The expected outcome is learned as a second dictionary approximator: expectation_map.
    The motivation reward is then calculated by taking the Wundt function of the novelty.
    The agent will receive the highest reward when the expected outcome of an action differs optimally from the actual outcome.
    That is, the expected outcome must not be too different but not too similar to the actual outcome to generate reward.
    This causes the agent to seek out optimally novel experiences: not too complex to learn but not too simple to become 'bored'.
*/

void MRLAgent::giveMotivationReward(float env_rew){
    //Initialise novelty
    float novelty=0;
    //Initialise the state-action vector to be fed to expectation map.
    vector<float> observation_action(num_inputs, 0);

    //If less than 2 observations have been made, leave observation_action a zero vector. Otherwise set to second last observation.
    if ((int)observations.size()>=2){
       observation_action = observations[observations.size()-2];
    }
    //Add action to state in which the action is taken
    //NEW ACTION VECTOR
    vector<float> action_vec(num_outputs,0);
    action_vec[actions[actions.size()-2]] = 1;//Indicate
    observation_action.insert(observation_action.end(),action_vec.begin(),action_vec.end());
    //Get expected observation and actual observation.
    vector<float> expected_observation = expectation_map->getValues(observation_action);
    vector<float> actual_observation = observations[observations.size()-1];

    //Setup values for learning. Moves the expected_observation along the straight line toward actual_observation in R^n.
    vector<vector<float> > obs(1,observation_action);//Setup vector with one input to be used for expectation_map learning.
    vector<vector<float> > val(1,expected_observation);//setup unit list of values.
    //Change values:
    for (int j=0; j<expected_observation.size();j++){
        float diff = actual_observation[j]-expected_observation[j];

        val[0][j]+= diff;
    }


    /*If expected observation is the zero vector it is highly likely that the state has not been visited before.
    In this case set the novelty to zero.
    */
    float sum_exp = 0;
    for (int j=0; j<expected_observation.size();j++){
        sum_exp = expected_observation[j]*expected_observation[j];
    }
    if(sum_exp == 0){
        //Do learning for expectation_map
        expectation_map->doLearningEpisode(obs,val,lambda,10);
        //Update novelty belief:
        average_novelty+=novelty_learning_rate*(0-average_novelty);
        if(novelty>max_novelty) max_novelty=novelty;
        if(novelty<min_novelty) min_novelty = novelty;
        float motivation = wundtFunction(0);
        //cout<<"MRLAGENT:: giveMotivationReward - novelty = "<<0<<endl;
        giveReward(motivation+env_rew);
        return;

    }

    //If expected observation non-zero, calculate error in prediction to get novelty:

    for (int j=0; j<expected_observation.size();j++){
        float diff = 0;
        diff = (expected_observation[j]-actual_observation[j]);
        novelty+= diff*diff;
    }
    //Update novelty belief:
    average_novelty+=novelty_learning_rate*(novelty-average_novelty);
    if(novelty>max_novelty) max_novelty=novelty;
    if(novelty<min_novelty) min_novelty = novelty;

    //Do learning for expectation_map
    expectation_map->doLearningEpisode(obs,val,lambda,10);

    float motivation = wundtFunction(novelty);
    //cout<<"MRLAGENT:: giveMotivationReward - novelty = "<<novelty<< " Reward = "<< motivation<<" + "<<env_rew<<endl;
    giveReward(motivation+env_rew);

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

    END OLD NOVELTY
*/ 
}



/*! @brief
        The wundt function is a linear combination of two sigmoids, offset by the float, baseline. It is analogous to a decapitated gaussian distribution.
        The most important parameters are N1 and N2, which give the rise and fall points of the distribution respectively.
*/
float MRLAgent::wundtFunction(float N){

    //Set for the robot motivation headbehaviour learner.
    float N1 =50;//Location of max positive gradient
    float N2 = 200;//Location of max negative gradient

    float M1 = 1.5;//Maximum motivation offset (eg: M1=2, M2 = 0, baseline = -1 gives range -1 to 1)
    float M2 = 1;//Minimum motivation offset
    float baseline = -1;
    float M3 = M2-M1;//maximum negative reward

    float p1 = 0.1;//Max pos gradient
    float p2 = 0.1;//Max negative gradient

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


 /*! @brief Main loop for MRLAgent. Returns the agents decision as an integer as to which action to take. Also performs the learning for the secand last state-action pair.
  */
 int MRLAgent::getActionAndLearn(vector<float> observations, vector<int> valid_actions, float rew){
     int action = getAction(observations,valid_actions);
     giveMotivationReward(rew);
     doLearning();
     return action;
 }

 /*! @brief Saves the MRLAgent
  */

 void MRLAgent::saveMRLAgent(string agentName){
     expectation_map->saveApproximator(agentName+"_expectation_map");
     saveAgent(agentName);

 }
 /*! @brief Loads the MRLAgent
  */
 void MRLAgent::loadMRLAgent(string agentName){
     expectation_map->loadApproximator(agentName+"_expectation_map");
     loadAgent(agentName);

 }
