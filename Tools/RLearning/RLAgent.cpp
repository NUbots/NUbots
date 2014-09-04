/*! @file RLAgent.cpp
    @brief Standard implementation of reinforcement learning agent.
    Needs to be implemented in subclass by adding constructor to create function approximators of your choosing.
    @author Jake Fountain
    ---------------------------------------------------
    Make your own RLAgent:
    1. Implement an RLAgent subclass. All that is required is the addition of a constructor initialising the function approximator.
    for example see DictionaryRLAgent.
    2. Follow the template below to implement a Motivated reinforcement learning agent in your code to explore a state space and make decisions.


    RLAgent rlagent;
    try{
        loadAgent(Filename);
    }catch (std::string s){
        rlagent.setParameters(0.1,0.5,0.5,1.0,1,5);//example parameters
        rlagent.initialiseAgent(observation_size,number_of_actions,resolution_of_FunctionApproximator);
    }

    for (number of iterations){
        int action = rlagent.getAction(observation);

        rlagent.giveReward(getRewardFromWorld());

        updateWorld(action);

        if(number of iterations has passed)
            rlagent.doLearning();
    }
    ---------------------------------------------------


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

RLAgent::RLAgent():RLearningInterface()
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
    std::vector<float> lv(numberOfOutputs,0.0);
    last_values = lv;


    //Perform initreturnial observations, values and rewards list setups. Required to offset learning updates.
    std::vector<float> dummy_observation(numberOfInputs,0);
    std::vector<int> vect(num_outputs,1);
    getAction(dummy_observation,vect);
    giveReward(0);

}



/*! @brief
    Saves the agent. Calling method saves the agent to a default file. Information includes:
        -Parameters
        -Function approximator
        -N(=memory_length) most recent rewards, values and observations
    Note: calling doLearning before saving agent will minimise the save file size.
 */
void RLAgent::saveAgent(std::string agentName){
    FunctionApproximator->saveApproximator(agentName+"_func_approx");
    //std::cout<<"funapp saved"<<std::endl;
    std::ofstream save_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName<<"_agent";
    save_file.open(file_name.str().c_str(),std::fstream::out);

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
           // std::cout<<"Saving observation: "<< observations[i][j]<<std::endl;
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
    save_file <<"\n";
    //save action validities
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<action_validities[i].size();j++){
            save_file<<action_validities[i][j]<<" ";
        }
        save_file <<"\n";
    }
    //std::cout << "RLAgent Saved Successfully" << std::endl;
    save_file.close();

}


/*! @brief
        Loads agent from file, default given as in save agent.Information includes:
            -Parameters
            -Function Approximator
            -N most recent rewards, observations, values  for motivation function

*/
void RLAgent::loadAgent(std::string agentName){

    FunctionApproximator->loadApproximator(agentName+"_func_approx");
    std::ifstream load_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName<<"_agent";
    load_file.open(file_name.str().c_str(),std::fstream::in);
    if(!load_file.good()) {
        throw std::string("RLAgent::loadAgent - file not found ") + file_name.str();
    }
    std::string tmp;

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

    std::vector<std::vector<float> > obs(memory_size, std::vector<float> (num_inputs,0));
    observations = obs;
    //Load observations:
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<(int)observations[i].size();j++){
            load_file >> observations[i][j];
        }
    }

    std::vector<float> rew(memory_size, 0);
    rewards = rew;

    //Load rewards
    for(int i =0; i< memory_size;i++){
        load_file >> rewards[i];
    }


    std::vector<std::vector<float> > val(memory_size, std::vector<float> (num_outputs,0));
    values = val;
    //Load values
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<values[i].size();j++){
            load_file>>values[i][j];
            //std::cout<<"Values: " << values[i][j]<< std::endl;
        }
    }

    std::vector<int> act(memory_size);
    actions = act;//init actions std::vector to zeros
    for(int i =0; i< memory_size;i++){
        load_file >> actions[i];
    }

    std::vector<std::vector<int> > validities(memory_size, std::vector<int> (num_outputs,1));
    action_validities = validities;
    //Load validities
    for(int i =0; i< memory_size;i++){
        for (int j=0;j<action_validities[i].size();j++){
            load_file>>action_validities[i][j];
            //std::cout<<"Values: " << values[i][j]<< std::endl;
        }
    }

    if(num_inputs == 0 or
        num_outputs == 0 or
        num_hidden == 0 )  {
        throw std::string("RLAgent::loadAgent - warning: num_inputs,outputs or hiddens are zero") + file_name.str();
    }
    std::cout<<"RLAgent::loadAgent - RLAgent loaded successfully.";

    load_file.close();
}



/*! @brief
        Sets the parameters for the learning algorithm.
        @param
        alpha : learning rate/stepsize
        beta : probability of random action selection (0 to 1.0) or if using softmax, this is the temperature(high temp gives more random selection).
        gamma : look-ahead learning stepsize
        lambda : Variable free for use in subclass RLAgents. EG Used in MRLAgent as learning stepsize for expectation_map.
        learningIterations : number of learning iterations when RLAgent::doLearning() called.
        memory_length : The number of observations, rewards and values to keep in memory when saving. Should be at least 2
        use_soft_max : when true the agent uses softmax with temperature beta to select actions. Otherwise the agent uses epsilon-greedy with probability of random choice beta.
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
    std::stringstream text;
    text<<"Reward: "<<reward<<"\n";
    std::string text_ = text.str();
    log(text_);
    //Store values for learning:
    values.push_back(last_values);


}


/*! @brief
        Picks the action that will maximise reward based on the function approximator. Returns an int between 0 and num_outputs-1.
        By default this method stores data for learning: observations.
*/
int RLAgent::getAction(std::vector<float> observation, std::vector<int> valid_actions) {

    if(valid_actions.size()!=num_outputs){
        std::cout<<"Throwing std::string RLAgent::getAction(..) ..."<<std::endl;
        throw std::string("RLAgent::getAction - valid actions std::list is incorrect length.");
    }
    int BestAction = rand()%num_outputs;
    //Store observation and validities for learning later on:
    observations.push_back(observation);
    action_validities.push_back(valid_actions);

    //Count the number of valid actions:
    int num_valid_actions = 0;
    for (int i = 0; i<valid_actions.size();i++){
        num_valid_actions+=valid_actions[i];
    }

    //Logging:
    std::stringstream text;
    text << "Observation: ";
    for(int i = 0; i< observation.size();i++){
        text<<observation[i]<<" ";
    }
    text << " \n";
    std::string text_ = text.str();
    log(text_);

    //Store last values
    last_values = FunctionApproximator->getValues(observation);

    //Beta-greedy or softmax action choice:
    if (beta*RAND_MAX>rand() or use_soft_max or num_valid_actions==0){
        //randomly select action with probability beta
        if (use_soft_max and num_valid_actions!=0){
            BestAction = getSoftMaxAction(last_values, valid_actions);
        }else{
            //Choose randomly from the valid actions
            if (num_valid_actions!=0){
                //Choose random action number
                int action_num = rand()%(num_valid_actions)+1;
                //count through valid actions until action_num counted
                int counted_valid_actions = 0;
                for(int i = 0; i<valid_actions.size();i++){
                    counted_valid_actions+=valid_actions[i];
                    if(counted_valid_actions == action_num){
                        BestAction = i;
                        break;
                    }

                }
            }
            else
            {
                //std::cout<< "RLAgent::getAction - warning: num_valid_actions is zero. Choosing Randomly"<<std::endl;
                BestAction = rand()%(num_outputs);
            }
        }
        //Store action for learning later
        actions.push_back(BestAction);        

        //Logging:
        std::stringstream text2;
        text2 << "Action Taken: "<<BestAction<<" \n";
        text_ = text2.str();
        log(text_);

        //Leave function
        return BestAction;
    }

    //Otherwise choose best choice:
    //------------------------------------
    //Select an initial value as the first of the available actions.
    float BestReward =0;
    int index =0;
    bool not_assigned = true;
    while(not_assigned){
        if(valid_actions[index]!=0){
            BestAction = index;
            BestReward = last_values[index];
            not_assigned = false;
        }
        index++;
        if (index >= valid_actions.size()) break;
    }


    //Check if all values equal while finding best action.
    bool allEqual = true;
    //Search for best action by finding largest value:
    for(int i = 0; i<last_values.size();i++){
        if (BestReward<last_values[i]){
            if(valid_actions[i]!=0){
                BestReward=last_values[i];
                BestAction=i;
            }
            allEqual=false;
        } else if (BestReward>last_values[i]){
            allEqual = false;
        }
    }

    if (allEqual/*Return random selection*/){
        if (num_valid_actions!=0){
            //Choose random action number
            int action_num = rand()%(num_valid_actions)+1;
            //count through valid actions until action_num counted
            int counted_valid_actions = 0;
            for(int i = 0; i<valid_actions.size();i++){
                counted_valid_actions+=valid_actions[i];
                if(counted_valid_actions == action_num){
                    BestAction = i;
                    break;
                }

            }
        }
        else{
            //std::cout<< "RLAgent::getAction - warning: num_valid_actions is zero. Choosing Randomly"<<std::endl;
            BestAction = rand()%(num_outputs);
        }

        actions.push_back(BestAction);
        //Logging:
        std::stringstream text2;
        text2 << "Action Taken: "<<BestAction<<" \n";
        text_ = text2.str();
        log(text_);

        return BestAction;
    }

    actions.push_back(BestAction);
   // cout<<"RLAgent::getAction - action chosen = "<< BestAction<<endl;

    //Logging:
    std::stringstream text2;
    text2 << "Action Taken: "<<BestAction<<" \n";
    text_ = text2.str();
    log(text_);

    return BestAction;
}


/*! @brief
        Performs learning by modifying the function approximator to reflect stored rewards and observations.
        Deletes stored past information after learning.
*/
void RLAgent::doLearning(){

    bool learning_done = false;
        for(int observation_num = 0; observation_num<observations.size()-1;observation_num++){

            int second_last_action = actions[observation_num];
            int last_action = actions[observation_num+1];
           // std::cout<< "value for observation "<< observation_num<< ", action "<< second_last_action<<" of "<<actions.size()<<" updated from: " <<values[observation_num][second_last_action];

            //Q-learning:
            values[observation_num][second_last_action] += alpha*(rewards[observation_num+1]+gamma*max(values[observation_num+1],action_validities[observation_num+1])-values[observation_num][second_last_action]);//Page 55 of MoRL book Merrick+Maher
            //std::cout<<" to: " <<values[observation_num][second_last_action]<<"\n"<<std::endl;
            learning_done = true;

        }

    FunctionApproximator->doLearningEpisode(observations, values,0.01, learningIterations);

    if (learning_done){
        //Erase all observations, values, rewards; except last ones because learning cannot be done until reward given.
        observations.erase(observations.begin(),observations.end()-1);
        values.erase(values.begin(),values.end()-1);
        rewards.erase(rewards.begin(),rewards.end()-1);
        actions.erase(actions.begin(),actions.end()-1);
        action_validities.erase(action_validities.begin(),action_validities.end()-1);

    }


}

/*! @brief
        Returns the maximum value of the floats in a vector<float>.
*/

float RLAgent::max(std::vector<float> x, std::vector<int> valid_actions){
    //Randomly choose start std::vector
    int ran = rand()%num_outputs;
    //choose best valid action value
    float best_value = x[ran];
    for(int i=0; i<x.size();i++){
        if (best_value<x[i] and valid_actions[i]!=0) best_value=x[i];
    }
    return best_value;
}


/*! @brief
       Logs strings to the file RL_log.log
*/
void RLAgent::log(std::string text){
    std::ofstream log_file;
    std::stringstream s;
    s<<save_location<<"RLearning.log";
    log_file.open((s.str()).c_str(),std::ios_base::app);

    log_file << text;
    //std::cout<<"Logging: "<<text<<std::endl;
    log_file.close();
}

/*! @brief
       Returns the state-action values for the given state std::vector<float> v.
*/

std::vector<float> RLAgent::getValues(std::vector<float> v){
    return FunctionApproximator->getValues(v);
}


/*! @brief
       Checks the policy at state obs WITHOUT recording the action, reward, value, observation for learning.
        Will not choose with epsilon-greedy but will choose with softmax if set use_soft_max == true.
*/
int RLAgent::checkAction(std::vector<float> obs, std::vector<int> valid_actions){
    if(valid_actions.size()!=num_outputs)
        throw std::string("RLAgent::getAction - valid actions list is incorrect length.");
    std::vector<float> values = FunctionApproximator->getValues(obs);
    if (use_soft_max) return getSoftMaxAction(values,valid_actions);
    int BestAction = 0;
    float BestReward = values[0];
    //Check if all values equal:
    for(int i = 0; i<values.size();i++){
        if (BestReward<values[i] and valid_actions[i]!=0){
            BestReward=values[i];
            BestAction=i;
        }
    }
    return BestAction;
}

int  RLAgent::getSoftMaxAction(std::vector<float> values, std::vector<int> valid_actions){

    std::vector<float> probabilities;
    //calculate non-normalised probabilities
    float total_non_normalised =0;
    for(int i = 0; i<values.size();i++){
        float prob = valid_actions[i]*exp(values[i]/beta);
        probabilities.push_back(prob);
        total_non_normalised +=prob;

    }

    //Normalise
    for(int i = 0; i<values.size();i++){
        probabilities[i]= probabilities[i]/total_non_normalised;
    }

    //randomly choose from each action with probability as given above
    float rnd = rand();
    int action = 0;
    float p = probabilities[action]*RAND_MAX;
    while(rnd > p){
        p+=probabilities[++action]*RAND_MAX;
    }
    return action;

}
