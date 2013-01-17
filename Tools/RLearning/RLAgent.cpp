/*! @file RLAgent.cpp
    @brief Standard implementation of reinforcement learning agent. Needs to be implemented in subclass by adding
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

#include "RLAgent.h"

/*! @brief Constructor
        Sets parameters to standard values and calls constructor of function approximator.
*/

RLAgent::RLAgent()
{
   setParameters(0.1,0.1,0.1,1.0,1,10);
   //Don't initialise function approximator. Must be done in subclass.

   srand(time(0));//Set seed for random generator for epsilon greedy method
}

/*! @brief Destructor
        deletes FunctionApproximator
*/
RLAgent::~RLAgent(){
    delete FunctionApproximator;
}



/*! @brief  Agent initialiser
    Initialises function approximator by calling initialiseApproximator
*/
void RLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range){
    FunctionApproximator->initialiseApproximator(numberOfInputs,numberOfOutputs,numberOfHiddens,max_parameter_range);

    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;
    vector<float> lv(numberOfOutputs,0.0);
    last_values = lv;


    //Perform initreturnial observations, values and rewards list setups. Required to offset learning updates.
    vector<float> dummy_observation(numberOfInputs,0);
    getAction(dummy_observation);    
    giveReward(0);

}



/*! @brief
    Saves the agent. Calling method saves the agent to a default file. Information includes:
        -Parameters
        -Function approximator
        -N(=memory_length) most recent rewards, values and observations
    Note: calling doLearning before saving agent will minimise the save file size.
 */
void RLAgent::saveAgent(string agentName){
    FunctionApproximator->saveApproximator(agentName+"_func_approx");
    //cout<<"funapp saved"<<endl;

    ofstream save_file;
    stringstream file_name;
    file_name<<"nubot/"<<agentName<<"_agent";
    save_file.open(file_name.str().c_str(),fstream::out);

    save_file << alpha<<" ";
    save_file << beta<<" ";
    save_file << gamma<<" ";
    save_file << lambda<<" ";
    save_file << learningIterations<<" ";
    save_file << memory_length <<" ";

    save_file << num_inputs << " ";
    save_file << num_outputs<<" ";
    save_file << num_hidden <<" ";
    if(use_soft_max){
        save_file << 1 <<" ";
    }
    else{
        save_file << 0 <<" ";
    }

    int memory_size = observations.size();

    save_file << memory_size <<"\n";

    //Save observations
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<(int)observations[i].size();j++){
            save_file<<observations[i][j]<<" ";
           // cout<<"Saving observation: "<< observations[i][j]<<endl;
        }
        save_file <<"\n";
    }

    //Save rewards
    for(int i =0; i< memory_size;i++){
        save_file<<rewards[i]<<" ";
    }
    save_file <<"\n";

    //Save values
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<values[i].size();j++){
            save_file<<values[i][j]<<" ";
        }
        save_file <<"\n";
    }

    //save actions
    for(int i = 0; i<memory_size; i++){
         save_file<<actions[i]<<" ";
    }


    cout<<"RLAgent Saved Successfully"<<endl;
    save_file.close();

}


/*! @brief
        Loads agent from file, default given as in save agent.Information includes:
            -Parameters
            -Function Approximator
            -N most recent rewards, observations, values  for motivation function

*/
void RLAgent::loadAgent(string agentName){

    FunctionApproximator->loadApproximator(agentName+"_func_approx");
    return;
    ifstream load_file;
    stringstream file_name;
    file_name<<"nubot/"<<agentName<<"_agent";
    load_file.open(file_name.str().c_str(),fstream::in);
    if(!load_file.good()) {
        throw string("RLAgent::loadAgent - file not found ") + file_name.str();
    }
    string tmp;

    load_file >> alpha;
    load_file >> beta;
    load_file >> gamma;
    load_file >> lambda;
    load_file >> learningIterations;
    load_file >> memory_length;

    load_file >> num_inputs;
    load_file >> num_outputs;
    load_file >> num_hidden;
    int use_sm;
    load_file >> use_sm;
    use_soft_max = false;
    if (use_sm == 1){
        use_soft_max = true;
    }

    int memory_size;
    load_file >> memory_size;

    vector<vector<float> > obs(memory_size, vector<float> (num_inputs,0));
    observations = obs;
    //Load observations:
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<(int)observations[i].size();j++){
            load_file >> observations[i][j];
        }
    }

    vector<float> rew(memory_size, 0);
    rewards = rew;

    //Load rewards
    for(int i =0; i< memory_size;i++){
        load_file >> rewards[i];
    }


    vector<vector<float> > val(memory_size, vector<float> (num_outputs,0));
    values = val;
    //Load values
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<values[i].size();j++){
            load_file>>values[i][j];
            //cout<<"Values: " << values[i][j]<< endl;
        }
    }

    vector<int> act(memory_size);
    actions = act;//init actions vector to zeros
    for(int i =0; i< memory_size;i++){
        load_file >> actions[i];
    }

    cout<<"RLAgent loaded successfully.";
}



/*! @brief
        Sets the parameters for the learning algorithm.
        @param
        alpha : learning rate/stepsize
        beta : probability of random action selection (0 to 1.0) or if using softmax, this is the temperature.
        gamma : look-ahead learning stepsize
        lambda : Variable free for use in subclass RLAgents. EG Used in MRLAgent as learning stepsize for expectation_map.
        learningIterations : number of learning iterations when RLAgent::doLearning() called.
        memory_length : The number of observations, rewards and values to keep in memory when saving. Should be at least 2
*/
void RLAgent::setParameters(float alpha, float beta, float gamma, float lambda,int learningIterations, int memory_length, bool use_soft_max){
    this->alpha = alpha;//see doLearning() method
    this->beta = beta;//using epsilon-greedy action selection with beta the probability of random selection (see getAction())
    this->gamma = gamma;//see doLearning() method
    this->lambda = lambda;//Variable for use in subclass RLAgents. EG Used in MRLAgent as learning stepsize for expectation_map.
    this->learningIterations = learningIterations;
    this->memory_length = memory_length;//The number of observations, rewards and values to keep in memory when saving.
    this -> use_soft_max = use_soft_max;
}


/*! @brief
        Gives a reward to the agent. The reward is stored in a vector until learning is called.
    Should be called after getAction(observation).
*/
void RLAgent::giveReward(float reward){
    //Store rewards for learning:
    rewards.push_back(reward);
    stringstream text;
    text<<"Reward: "<<reward<<"\n";
    string text_ = text.str();
    log(text_);
    //Store values for learning:
    values.push_back(last_values);


}


/*! @brief
        Picks the action that will maximise reward based on the function approximator. Returns an int between 0 and num_outputs-1.
        By default this method stores data for learning: observations.
*/
int RLAgent::getAction(vector<float> observation){
    int BestAction = 0;
    //Store observation for learning later on:
    observations.push_back(observation);
    //Logging:
    stringstream text;
    text << "Observation: ";
    for(int i = 0; i< observation.size();i++){
        text<<observation[i]<<" ";
    }
    text << " \n";

    string text_ = text.str();
    log(text_);

    //Store last values
    last_values = FunctionApproximator->getValues(observation);

    //Beta-greedy or softmax action choice:
    if (beta*RAND_MAX>rand() or use_soft_max){
        //randomly select action with probability beta
        if (use_soft_max){
            BestAction = getSoftMaxAction(last_values);
        }else{
            BestAction = rand()%num_outputs;
        }

        actions.push_back(BestAction);

        (BestAction);
        //Logging:
        stringstream text2;
        text2 << "Action Taken: "<<BestAction<<" \n";
        text_ = text2.str();
        log(text_);

        return BestAction;
    }

    //Otherwise choose best choice:
    //------------------------------------

    float BestReward = last_values[0];

    //Check if all values equal:
    bool allEqual = true;
    //Search for best action by finding largest value:
    for(int i = 0; i<last_values.size();i++){
        if (BestReward<last_values[i]){
            BestReward=last_values[i];
            BestAction=i;
            allEqual=false;
        } else if (BestReward>last_values[i]){
            allEqual = false;
        }
    }

    if (allEqual/*Return random selection*/){
        BestAction = rand()%num_outputs;\


        actions.push_back(BestAction);
        //Logging:
        stringstream text2;
        text2 << "Action Taken: "<<BestAction<<" \n";
        text_ = text2.str();
        log(text_);

        return BestAction;
    }

    actions.push_back(BestAction);


    //Logging:
    stringstream text2;
    text2 << "Action Taken: "<<BestAction<<" \n";
    text_ = text2.str();
    log(text_);

    return BestAction;
}


/*! @brief
        Performs learning by modifying the function approximator to reflect rewards and observations
*/
void RLAgent::doLearning(){
    bool learning_done = false;
        for(int observation_num = 0; observation_num<observations.size()-1;observation_num++){

            int second_last_action = actions[observation_num];
            int last_action = actions[observation_num+1];
           // cout<< "value for observation "<< observation_num<< ", action "<< second_last_action<<" of "<<actions.size()<<" updated from: " <<values[observation_num][second_last_action];

            //Q-learning:
            values[observation_num][second_last_action] += alpha*(rewards[observation_num+1]+gamma*max(values[observation_num+1])-values[observation_num][second_last_action]);//Page 55 of MoRL book Merrick+Maher
            //cout<<" to: " <<values[observation_num][second_last_action]<<"\n"<<endl;
            learning_done = true;

        }

    FunctionApproximator->doLearningEpisode(observations, values,0.01, learningIterations);

    if (learning_done){
        //Erase all observations, values, rewards; except last ones because learning cannot be done until reward given.
        observations.erase(observations.begin(),observations.end()-1);
        values.erase(values.begin(),values.end()-1);
        rewards.erase(rewards.begin(),rewards.end()-1);
        actions.erase(actions.begin(),actions.end()-1);


    }

}

/*! @brief
        Returns the maximum value of the floats in a vector<float>.
*/

float RLAgent::max(vector<float> x){
    float best_value = x[0];
    for(int i=1; i<x.size();i++){
        if (best_value<x[i]) best_value=x[i];
    }
    return best_value;
}


/*! @brief
       Logs strings to the file RL_log.log
*/
void RLAgent::log(string text){
    ofstream log_file;

    log_file.open("nubot/RLearning.log",ios_base::out);

    log_file << text;
    //cout<<"Logging: "<<text<<endl;
    log_file.close();
}

/*! @brief
       Returns the state-action values for the given state vector<float> v.
*/

vector<float> RLAgent::getValues(vector<float> v){
    return FunctionApproximator->getValues(v);
}


/*! @brief
       Checks the policy at state obs WITHOUT recording the action, reward, value, observation for learning.
*/
int RLAgent::checkAction(vector<float> obs){
    vector<float> values = FunctionApproximator->getValues(obs);
    int BestAction = 0;
    float BestReward = values[0];
    //Check if all values equal:

    for(int i = 0; i<values.size();i++){
        if (BestReward<values[i]){
            BestReward=values[i];
            BestAction=i;

        }
    }
    return BestAction;
}

int  RLAgent::getSoftMaxAction(vector<float> values){
    vector<float> probabilities;
    //calculate non-normalised probabilities
    float total_non_normalised =0;
    for(int i = 0; i<values.size();i++){
        float prob = exp(values[i]/beta);
        probabilities.push_back(prob);
        total_non_normalised +=prob;

    }

    //Normalise
    for(int i = 0; i<values.size();i++){
        probabilities[i]= probabilities[i]/total_non_normalised;
    }

    //randomly choose from each action with probability as given above
    float rnd = rand()/(float)RAND_MAX;
    int action = 0;
    float p = probabilities[action];
    while(rnd > p){
        p+=probabilities[++action];
    }
    return action;

}
