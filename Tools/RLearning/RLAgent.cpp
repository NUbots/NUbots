/*! @file RLAgent.cpp
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

#include "RLAgent.h"

/*! @brief Constructor
        Sets parameters to standard values and calls constructor of function approximator.
*/

RLAgent::RLAgent()
{
   setParameters(0.1,0.1,0.1,1.0,1,10);
   //Don't initialise function approximator. Must be done in subclass.
   //FunctionApproximator = new ApproximatorInterface();
   srand(time(0));//Set seed for random generator for epsilon greedy method

}



RLAgent::~RLAgent(){
    delete FunctionApproximator;
}



/*! @brief  Agent initialiser
  Initialises function approximator by calling initialiseApproximator
*/
void RLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    FunctionApproximator->initialiseApproximator(numberOfInputs,numberOfOutputs,numberOfHiddens);
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;
    vector<float> lv(numberOfOutputs,0.0);
    last_values = lv;
    last_action = 0;
    second_last_action = 0;

}



/*! @brief
    Saves the agent. Calling method saves the agent to a default file. Information includes:
        -Parameters
        -Function generator
        -Lifespan
        -N most recent rewards and observations for motivation function
 */
void RLAgent::saveAgent(string agentName){
    FunctionApproximator->saveApproximator(agentName);
    //cout<<"funapp saved"<<endl;

    ofstream save_file;
    string file_name = agentName + "_agent.txt";
    save_file.open("RLAgent.txt",ios_base::out);


    save_file << alpha<<" ";
    save_file << beta<<" ";
    save_file << gamma<<" ";
    save_file << lambda<<" ";
    save_file << learningIterations<<" ";
    save_file << memory_length <<" ";

    save_file << num_inputs << " ";
    save_file << num_outputs<<" ";
    save_file << num_hidden <<" ";

    //Add zero values to front of observations/rewards/values to make up to size memory_length.
    vector<float> zero_observation(num_inputs);
    vector<float> zero_value(num_inputs);
    //Assuming |obs|=|values|=|rewards|, extend lengths to memory_length if necessary.
    while((memory_length-(int)observations.size())>0){
        observations.insert(observations.begin(),zero_observation);
        rewards.insert(rewards.begin(),0);
        values.insert(values.begin(),zero_value);

    }


    //Save observations
    for(int i =0; i< memory_length;i++){
        for (int j=0;j<(int)observations[i].size();j++){
            save_file<<observations[i][j]<<" ";
           // cout<<"Saving observation: "<< observations[i][j]<<endl;
        }
        save_file <<"\n";
    }
    save_file <<"\n";

    //Save rewards
    for(int i =0; i< memory_length;i++){
        save_file<<rewards[i]<<" ";
    }
    save_file <<"\n";

    //Save values
    for(int i =0; i< memory_length;i++){
        for (int j=0;j<values[i].size();j++){
            save_file<<values[i][j]<<" ";
        }
        save_file <<"\n";
    }

    cout<<"MRLAgent Saved Successfully"<<endl;

}


/*! @brief
        Loads agent from file, default given as in save agent.
*/
void RLAgent::loadAgent(string agentName){
    FunctionApproximator->loadApproximator(agentName);

    ifstream load_file;
    string file_name = agentName+"_agent.txt";
    load_file.open("RLAgent.txt",fstream::in);
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

    vector<vector<float> > obs(memory_length, vector<float> (num_inputs,0));
    observations = obs;

    for(int i =0; i< memory_length;i++){
        for (int j=0;j<(int)observations[i].size();j++){
            load_file >> observations[i][j];
        }
    }

    vector<float> rew(memory_length, 0);
    rewards = rew;

    //Load rewards
    for(int i =0; i< memory_length;i++){
        load_file >> rewards[i];
    }


    vector<vector<float> > val(memory_length, vector<float> (num_outputs,0));
    values = val;
    //Load values
    for(int i =0; i< memory_length;i++){
        for (int j=0;j<values[i].size();j++){
            load_file>>values[i][j];
            //cout<<"Values: " << values[i][j]<< endl;
        }
    }
    cout<<"MRLAgent loaded successfully.";
}



/*! @brief
        Sets the parameters for the learning algorithm.
*/
void RLAgent::setParameters(float alpha, float beta, float gamma, float lambda,int learningIterations, int memory_length){
    cout<< "Setting RLAgent param..." <<endl;
    this->alpha = alpha;//see doLearning() method
    this->beta = beta;//using epsilon-greedy action selection with beta the probability of random selection (see getAction())
    this->gamma = gamma;//see doLearning() method
    this->lambda = lambda;//Used in MRLAgent as learning stepsize for expectation_map
    this->learningIterations = learningIterations;
    this->memory_length = memory_length;
    cout<< "RLAgent parameters set." <<endl;
}


/*! @brief
        Gives a reward to the agent. The reward is stored in a vector until learning is called.
    Should be called after getAction(observation).
*/
void RLAgent::giveReward(float reward){
    rewards.push_back(reward);
    stringstream text;
    text<<"Reward: "<<reward<<"\n";
    string text_ = text.str();
    log(text_);
    //Add last_values to values list.
    values.push_back(last_values);
}


/*! @brief
        Picks the action that will maximise reward based on the function approximator. Returns an int between 0 and num_outputs-1
*/
int RLAgent::getAction(vector<float> observation){
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
    last_values = FunctionApproximator->getValues(observation);

    //Beta-greedy action choice:
    if (beta*RAND_MAX>rand()){
        second_last_action = last_action;
        last_action = rand()%num_outputs;
        //Logging:
        stringstream text2;
        text2 << "Action Taken: "<<last_action<<" \n";
        text_ = text2.str();
        log(text_);

        return last_action;
    }

    //Otherwise choose best choice:
    //Search for best action by finding largest value:
    int BestAction = 0;
    float BestReward = last_values[0];
    //Check if all values equal:
    bool allEqual = true;
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
        second_last_action = last_action;
        last_action = rand()%num_outputs;
        //Logging:
        stringstream text2;
        text2 << "Action Taken: "<<last_action<<" \n";
        text_ = text2.str();
        log(text_);

        return last_action;
    }

    second_last_action = last_action;
    last_action = BestAction;


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
    for (int i =0;i<learningIterations;i++){
        /*Iterate backwards over all observations except latest observation as the next state is not known*/
        //for(int observation_num=observations.size()-2; observation_num>-1;observation_num--){
            //for(int action = 0; action<num_outputs; action++){
        int observation_num = observations.size()-2;
        //cout<< rewards[observation_num]<<endl;
        //Debug:
 //       cout<< "value for observation "<< observation_num<< ", action "<< second_last_action<<" updated from: " <<values[observation_num][second_last_action];

        //Q-learning:
        values[observation_num][second_last_action] += alpha*(rewards[observation_num+1]+gamma*max(values[observation_num+1])-values[observation_num][second_last_action]);//Page 55 of MoRL book Merrick+Maher

//        cout<<" to: " <<values[observation_num][second_last_action]<<"\n"<<endl;



            //}
        //}
    }
    FunctionApproximator->doLearningEpisode(observations, values);
}



float RLAgent::max(vector<float> x){
    float best_value = x[0];
    for(int i=1; i<x.size();i++){
        if (best_value<x[i]) best_value=x[i];
    }
    return best_value;
}

void RLAgent::log(string text){
    ofstream log_file;

    log_file.open("RL_log.txt",ios_base::out);

    log_file << text;
    //cout<<"Logging: "<<text<<endl;
    log_file.close();
}


vector<float> RLAgent::getValues(vector<float> v){
    return FunctionApproximator->getValues(v);
}

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
